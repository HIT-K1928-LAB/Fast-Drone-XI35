"""SITL recovery extension; keep the reviewed run_yopo_px4ctrl.py beside this file.

Default: observe recovery eligibility without resuming. --auto-resume opts in.
BRAKE means release commands to PX4Ctrl's timeout hover, not an active brake path.
"""
import argparse
import json
import time

from run_yopo_px4ctrl import GazeboYopo, cfg, np, rospy, upstream
from std_msgs.msg import String


class RecoveryYopo(GazeboYopo):
    def __init__(self, execute, auto_resume=False):
        self.phase = "IDLE"
        self.auto_resume = auto_resume
        self.recoveries = 0
        self.max_recoveries = 3
        self.stable_since = None
        self.stable_anchor = None
        self.good_frames = 0
        self.good_since = None
        self.last_replan_stamp = -float("inf")
        self.last_odom_stamp = -float("inf")
        self.mu_lo = None
        self.last_report = -float("inf")
        rospy.init_node("yopo_net", anonymous=False)
        self.status_pub = rospy.Publisher("/yopo_minco/recovery_state", String,
                                          queue_size=1, latch=True)
        self.cmd_timeout = float(rospy.get_param("/px4ctrl/msg_timeout/cmd", 0.5))
        if not np.isfinite(self.cmd_timeout) or self.cmd_timeout <= 0:
            raise ValueError("Invalid PX4Ctrl command timeout")
        self.emit("waiting for a goal")
        super().__init__(execute)

    def emit(self, reason):
        payload = dict(phase=self.phase, reason=reason, recoveries=self.recoveries,
                       good_frames=self.good_frames, auto_resume=self.auto_resume)
        if getattr(self, "odom_init", False):
            payload["position"] = [float(x) if np.isfinite(x) else None for x in self._odom_pos()]
            payload["velocity"] = [float(x) if np.isfinite(x) else None for x in self._odom_vel()]
        if self.mu_lo is not None:
            payload["corridor_mu_lo"] = self.mu_lo.tolist()
            payload["safe_mu"] = float(self.safe_mu)
        data = json.dumps(payload, allow_nan=False)
        self.status_pub.publish(String(data=data))
        rospy.logwarn("YOPO RECOVERY: %s", data)

    def exclusive_output(self):
        """Registration check; fail closed if another command publisher exists."""
        if not self.execute:
            return True
        try:
            code, _, state = rospy.get_master().getSystemState()
            if code != 1:
                return False
            publishers = dict(state[0]).get("/position_cmd", [])
            return not (set(publishers) - {rospy.get_name()})
        except Exception:
            return False

    def stop(self, reason):
        # Only this exact upstream condition is recoverable. All other stops latch.
        if reason == "upstream corridor admission requested braking" and self.phase == "TRACK":
            if self.recoveries >= self.max_recoveries:
                reason = "recovery limit reached (3); manual review required"
            else:
                self.recoveries += 1
                self.phase = "BRAKE"
                self.brake_started = rospy.Time.now().to_sec()
                self.recovery_started_wall = time.monotonic()
                self.stable_since = self.stable_anchor = None
                self.good_frames = 0
                self.good_since = None
                self.last_replan_stamp = -float("inf")
                self.ctrl_time = None
                self.desire_init = False
                # active keeps the original goal owned; control_pub is phase-gated.
                self.emit("corridor rejected; releasing commands for PX4Ctrl hover")
                return
        super().stop(reason)
        self.phase = "STOPPED"
        self.emit(reason)

    def callback_set_goal(self, msg):
        with self.guard:
            if not self.exclusive_output():
                rospy.logwarn("Goal rejected: competing /position_cmd publisher or ROS master unavailable")
                return
            was_active = self.active
            super().callback_set_goal(msg)
            if self.active and not was_active:
                self.phase = "TRACK"
                self.recoveries = 0
                self.mu_lo = None
                self.last_odom_stamp = -float("inf")
                self.emit("goal accepted")

    def reset_stability(self):
        self.stable_since = self.stable_anchor = None
        self.good_frames = 0
        self.good_since = None

    def callback_odometry(self, msg):
        with self.guard:
            super().callback_odometry(msg)
            if self.phase not in ("BRAKE", "HOVER_REPLAN"):
                return
            stamp = msg.header.stamp.to_sec()
            if stamp < self.last_odom_stamp:
                self.stop("simulation clock/odometry moved backwards")
                return
            if stamp == self.last_odom_stamp:
                return
            self.last_odom_stamp = stamp
            p, v = self._odom_pos(), self._odom_vel()
            if not np.isfinite(p).all() or not np.isfinite(v).all() or p[2] < 0.6:
                self.stop("invalid odometry or recovery altitude below 0.6 m")
                return
            if stamp - self.brake_started < self.cmd_timeout + 0.5:
                self.reset_stability()
                return
            if np.linalg.norm(v) > 0.15:
                self.reset_stability()
                if self.phase == "HOVER_REPLAN":
                    self.phase = "BRAKE"
                    self.ctrl_time = None
                    self.emit("hover became unstable; waiting again")
                return
            if self.stable_anchor is None or np.linalg.norm(p - self.stable_anchor) > 0.10:
                self.stable_anchor = p.copy()
                self.stable_since = stamp
                self.good_frames = 0
                self.good_since = None
            if stamp - self.stable_since >= 1.0 and self.phase == "BRAKE":
                self.phase = "HOVER_REPLAN"
                self.emit("stable hover by timeout, speed and position checks; replanning without commands")

    def _corridor_mu_lo(self, radius_pred):
        values = super()._corridor_mu_lo(radius_pred)
        if not np.isfinite(values).all():
            raise ValueError("non-finite predicted corridor")
        self.mu_lo = values.copy()
        return values

    def recovery_problem(self):
        problem = self.input_problem()
        if problem:
            return problem
        if time.monotonic() - self.recovery_started_wall > 30.0:
            return "recovery timed out after 30 wall seconds"
        if rospy.Time.now().to_sec() < self.brake_started:
            return "simulation clock moved backwards"
        return None

    def candidate_ok(self):
        if self.brake or self.optimal_traj is None:
            return False
        duration = self.best_total_time
        if not np.isfinite(duration) or duration <= 0:
            return False
        # This sanity/height test is not a full geometric collision check.
        ts = np.linspace(0.0, duration, 101)
        pos = self.optimal_traj.position(ts)
        vel = self.optimal_traj.velocity(ts)
        acc = self.optimal_traj.acceleration(ts)
        return bool(np.isfinite(pos).all() and np.isfinite(vel).all()
                    and np.isfinite(acc).all() and np.min(pos[:, 2]) >= 0.6
                    and np.linalg.norm(pos[0] - self._odom_pos()) <= 0.15)

    def callback_depth(self, msg):
        with self.guard:
            if self.phase not in ("BRAKE", "HOVER_REPLAN"):
                super().callback_depth(msg)
                return
            self.depth_received = time.monotonic()
            self.latest_depth_stamp = msg.header.stamp
            self.latest_depth_shape = (msg.width, msg.height)
            problem = self.recovery_problem()
            if problem:
                self.stop(problem)
                return
            if self.phase != "HOVER_REPLAN" or self.stable_since is None:
                return
            stamp = msg.header.stamp.to_sec()
            if self.odom.header.stamp.to_sec() - self.stable_since < 1.0:
                return
            if stamp - self.last_replan_stamp < 0.10:
                return  # distinct images, at most 10 Hz while hovering
            self.last_replan_stamp = stamp
            try:
                self.seed_reference()  # measured hover P/V, zero A; discard old reference
                self.best_inner_w = None
                upstream.YopoNet.callback_depth(self, msg)  # inference/viz only
                problem = self.recovery_problem()
                if problem:
                    self.stop(problem)
                    return
                if self.candidate_ok():
                    self.good_frames += 1
                    if self.good_since is None:
                        self.good_since = stamp
                else:
                    self.good_frames = 0
                    self.good_since = None
                ready = (self.good_since is not None and self.good_frames >= 5
                         and stamp - self.good_since >= 0.4)
                if ready and self.auto_resume:
                    if not self.exclusive_output():
                        self.stop("competing command publisher or ROS master unavailable")
                        return
                    # Most recent candidate was solved from fresh actual hover PVA.
                    self.phase = "TRACK"
                    self.emit("RESUME: consecutive candidate checks passed")
                elif time.monotonic() - self.last_report >= 1.0:
                    self.last_report = time.monotonic()
                    self.emit("READY_PREVIEW: no commands; --auto-resume required" if ready
                              else "checking consecutive corridor/finite/height-valid plans")
                if self.phase != "TRACK":
                    self.ctrl_time = None  # discard every preview plan
            except Exception:
                self.stop("recovery planning exception")
                rospy.logerr("Recovery failed", exc_info=True)

    def control_pub(self, event):
        with self.guard:
            if self.phase in ("BRAKE", "HOVER_REPLAN"):
                problem = self.recovery_problem()
                if problem:
                    self.stop(problem)
                return  # no position command, including upstream EMPTY messages
            if self.phase == "TRACK":
                super().control_pub(event)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--auto-resume", action="store_true",
                        help="Enable bounded automatic recovery after corridor braking (SITL)")
    args = parser.parse_args(rospy.myargv()[1:])
    cfg["velocity"] = 1.0
    RecoveryYopo(args.execute, args.auto_resume)

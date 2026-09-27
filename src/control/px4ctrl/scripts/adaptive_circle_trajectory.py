#!/usr/bin/env python3

"""Adaptive circle trajectory publisher for the PX4 OFFBOARD FSM."""

from dataclasses import dataclass
import math
import threading


@dataclass(frozen=True)
class CircleCommand:
    position: tuple
    velocity: tuple
    acceleration: tuple
    jerk: tuple
    yaw: float
    yaw_rate: float


@dataclass(frozen=True)
class SearchDecision:
    state: str
    target_speed: float
    last_safe_speed: float
    emergency: bool
    rms_error: float = 0.0
    p95_error: float = 0.0
    peak_error: float = 0.0


class AdaptiveSpeedSearch:
    def __init__(
        self, initial_speed, speed_step, settle_time, window_time,
        pass_rms_error, pass_p95_error, fail_rms_error, fail_peak_error,
        emergency_error, emergency_hold_time, speed_limit=float("inf")
    ):
        self.target_speed = initial_speed
        self.last_safe_speed = 0.0
        self.speed_step = speed_step
        self.settle_time = settle_time
        self.window_time = window_time
        self.pass_rms_error = pass_rms_error
        self.pass_p95_error = pass_p95_error
        self.fail_rms_error = fail_rms_error
        self.fail_peak_error = fail_peak_error
        self.emergency_error = emergency_error
        self.emergency_hold_time = emergency_hold_time
        self.speed_limit = speed_limit
        self.state = "RAMPING"
        self._state_started_at = None
        self._samples = []
        self._critical_since = None

    def observe(self, now, error, at_target):
        if error >= self.emergency_error:
            if self._critical_since is None:
                self._critical_since = now
            elif now - self._critical_since >= self.emergency_hold_time:
                self.state = "EMERGENCY"
        else:
            self._critical_since = None
        if self.state == "EMERGENCY":
            return self._decision()

        if self.state == "RAMPING" and at_target:
            self.state = "SETTLING"
            self._state_started_at = now
        if self.state == "SETTLING" and now - self._state_started_at >= self.settle_time:
            self.state = "EVALUATING"
            self._state_started_at = now
            self._samples = []
        if self.state == "EVALUATING":
            self._samples.append(error)
            if now - self._state_started_at >= self.window_time:
                rms = math.sqrt(sum(value * value for value in self._samples) / len(self._samples))
                ordered = sorted(self._samples)
                index = max(0, math.ceil(0.95 * len(ordered)) - 1)
                p95 = ordered[index]
                peak = ordered[-1]
                if rms <= self.pass_rms_error and p95 <= self.pass_p95_error:
                    self.last_safe_speed = self.target_speed
                    next_speed = min(self.target_speed + self.speed_step, self.speed_limit)
                    if next_speed <= self.target_speed + 1e-9:
                        self.state = "LIMIT_REACHED"
                    else:
                        self.target_speed = next_speed
                        self.state = "RAMPING"
                    self._state_started_at = None
                elif rms >= self.fail_rms_error or peak >= self.fail_peak_error:
                    if self.last_safe_speed > 0.0:
                        self.target_speed = self.last_safe_speed
                        self.state = "COMPLETE"
                    else:
                        self.state = "EMERGENCY"
                else:
                    self._state_started_at = now
                    self._samples = []
                return self._decision(rms, p95, peak)
        return self._decision()

    def _decision(self, rms=0.0, p95=0.0, peak=0.0):
        return SearchDecision(
            self.state, self.target_speed, self.last_safe_speed,
            self.state == "EMERGENCY", rms, p95, peak)


def feasible_speed_limit(radius, max_velocity, max_acceleration, hard_max_speed=0.0):
    if radius <= 0.0:
        raise ValueError("radius must be positive")
    limits = []
    if max_velocity > 0.0:
        limits.append(max_velocity)
    if max_acceleration > 0.0:
        limits.append(math.sqrt(max_acceleration * radius))
    if hard_max_speed > 0.0:
        limits.append(hard_max_speed)
    return min(limits) if limits else float("inf")


def initial_circle_geometry(position, yaw, radius, clockwise):
    if radius <= 0.0:
        raise ValueError("radius must be positive")
    direction = -1.0 if clockwise else 1.0
    phase = yaw - direction * math.pi / 2.0
    center = (
        position[0] - radius * math.cos(phase),
        position[1] - radius * math.sin(phase),
    )
    return center, phase


def circle_command(
    center, altitude, radius, phase, speed, clockwise,
    tangential_acceleration=0.0
):
    if radius <= 0.0:
        raise ValueError("radius must be positive")
    if speed < 0.0:
        raise ValueError("speed must be non-negative")

    direction = -1.0 if clockwise else 1.0
    omega = direction * speed / radius
    angular_acceleration = direction * tangential_acceleration / radius
    cosine = math.cos(phase)
    sine = math.sin(phase)
    velocity = (-radius * omega * sine, radius * omega * cosine, 0.0)
    radial_jerk = -3.0 * radius * omega * angular_acceleration
    tangential_jerk = -radius * omega**3
    tangent_heading = phase + direction * math.pi / 2.0

    return CircleCommand(
        position=(center[0] + radius * cosine, center[1] + radius * sine, altitude),
        velocity=velocity,
        acceleration=(
            -radius * omega * omega * cosine - radius * angular_acceleration * sine,
            -radius * omega * omega * sine + radius * angular_acceleration * cosine,
            0.0,
        ),
        jerk=(
            radial_jerk * cosine - tangential_jerk * sine,
            radial_jerk * sine + tangential_jerk * cosine,
            0.0,
        ),
        yaw=math.atan2(math.sin(tangent_heading), math.cos(tangent_heading)),
        yaw_rate=omega,
    )


def quaternion_yaw(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def run_ros_node():
    import rospy
    from nav_msgs.msg import Odometry
    from px4ctrl.msg import FsmStatus
    from quadrotor_msgs.msg import PositionCommand
    from std_msgs.msg import Bool, Float64, String
    from std_srvs.srv import Trigger, TriggerResponse

    class AdaptiveCircleNode:
        def __init__(self):
            self._lock = threading.Lock()

            self.radius = float(rospy.get_param("~circle/radius", 2.0))
            self.clockwise = bool(rospy.get_param("~circle/clockwise", False))
            self.initial_speed = float(rospy.get_param("~circle/initial_speed", 0.5))
            self.speed_step = float(rospy.get_param("~circle/speed_step", 0.25))
            self.ramp_acceleration = float(
                rospy.get_param("~circle/speed_ramp_acceleration", 0.5))
            self.command_rate = float(rospy.get_param("~circle/command_rate", 50.0))
            self.hard_max_speed = float(
                rospy.get_param("~limits/hard_max_speed", 0.0))

            self.settle_time = float(rospy.get_param("~evaluation/settle_time", 3.0))
            self.window_time = float(rospy.get_param("~evaluation/window_time", 5.0))
            self.pass_rms_error = float(
                rospy.get_param("~evaluation/pass_rms_error", 0.12))
            self.pass_p95_error = float(
                rospy.get_param("~evaluation/pass_p95_error", 0.18))
            self.fail_rms_error = float(
                rospy.get_param("~evaluation/fail_rms_error", 0.20))
            self.fail_peak_error = float(
                rospy.get_param("~evaluation/fail_peak_error", 0.35))
            self.emergency_error = float(
                rospy.get_param("~evaluation/emergency_error", 0.50))
            self.emergency_hold_time = float(
                rospy.get_param("~evaluation/emergency_hold_time", 0.30))

            self.auto_start = bool(rospy.get_param("~safety/auto_start", False))
            self.feedback_timeout = float(
                rospy.get_param("~safety/feedback_timeout", 0.30))
            self.status_timeout = float(
                rospy.get_param("~safety/fsm_status_timeout", 0.50))
            self.speed_tolerance = float(
                rospy.get_param("~safety/speed_target_tolerance", 0.02))

            self.planner_frame = rospy.get_param("~frames/planner", "camera_init")
            self.fsm_max_velocity_param = rospy.get_param(
                "~limits/fsm_max_velocity_param",
                "/px4_offboard_fsm/planner/max_velocity")
            self.fsm_max_acceleration_param = rospy.get_param(
                "~limits/fsm_max_acceleration_param",
                "/px4_offboard_fsm/planner/max_acceleration")
            max_velocity = float(rospy.get_param(
                self.fsm_max_velocity_param,
                rospy.get_param("~limits/fsm_max_velocity_fallback", 20.0)))
            max_acceleration = float(rospy.get_param(
                self.fsm_max_acceleration_param,
                rospy.get_param("~limits/fsm_max_acceleration_fallback", 10.0)))

            self._validate(max_velocity, max_acceleration)
            centripetal_limit = math.sqrt(max_acceleration**2 - self.ramp_acceleration**2)
            self.speed_limit = feasible_speed_limit(
                self.radius, max_velocity, centripetal_limit,
                self.hard_max_speed)
            if self.initial_speed > self.speed_limit:
                raise ValueError(
                    "initial_speed %.3f exceeds the FSM-derived limit %.3f" %
                    (self.initial_speed, self.speed_limit))

            self.planner_command_topic = rospy.get_param(
                "~topics/planner_command", "/position_cmd")
            self.livo_odometry_topic = rospy.get_param(
                "~topics/livo_odometry", "/LIVO2/imu_propagate")
            self.px4_odometry_topic = rospy.get_param(
                "~topics/px4_odometry", "/mavros/local_position/odom")
            self.reference_topic = rospy.get_param(
                "~topics/reference", "/offboard_fsm/reference")
            self.fsm_status_topic = rospy.get_param(
                "~topics/fsm_status", "/offboard_fsm/status")
            self.emergency_topic = rospy.get_param(
                "~topics/emergency_hover", "/planning/Emergency_hover")
            self.speed_topic = rospy.get_param(
                "~topics/current_speed", "/adaptive_circle/current_speed")
            self.error_topic = rospy.get_param(
                "~topics/tracking_error", "/adaptive_circle/tracking_error")
            self.search_status_topic = rospy.get_param(
                "~topics/search_status", "/adaptive_circle/search_status")

            self.command_publisher = rospy.Publisher(
                self.planner_command_topic, PositionCommand, queue_size=1)
            self.emergency_publisher = rospy.Publisher(
                self.emergency_topic, Bool, queue_size=1)
            self.speed_publisher = rospy.Publisher(
                self.speed_topic, Float64, queue_size=1)
            self.error_publisher = rospy.Publisher(
                self.error_topic, Float64, queue_size=1)
            self.search_status_publisher = rospy.Publisher(
                self.search_status_topic, String, queue_size=1)

            self.livo_subscriber = rospy.Subscriber(
                self.livo_odometry_topic, Odometry, self._livo_callback,
                queue_size=1, tcp_nodelay=True)
            self.px4_subscriber = rospy.Subscriber(
                self.px4_odometry_topic, Odometry, self._px4_callback,
                queue_size=1, tcp_nodelay=True)
            self.reference_subscriber = rospy.Subscriber(
                self.reference_topic, Odometry, self._reference_callback,
                queue_size=1, tcp_nodelay=True)
            self.fsm_status_subscriber = rospy.Subscriber(
                self.fsm_status_topic, FsmStatus, self._fsm_status_callback,
                queue_size=1, tcp_nodelay=True)

            self.start_service = rospy.Service("~start", Trigger, self._start_callback)
            self.stop_service = rospy.Service("~stop", Trigger, self._stop_callback)

            self.livo_odometry = None
            self.px4_odometry = None
            self.reference = None
            self.fsm_status = None
            self.livo_received_at = 0.0
            self.px4_received_at = 0.0
            self.reference_received_at = 0.0
            self.fsm_status_received_at = 0.0
            self.active = False
            self.center = (0.0, 0.0)
            self.altitude = 0.0
            self.phase = 0.0
            self.current_speed = 0.0
            self.last_tick = 0.0
            self.trajectory_id = 0
            self.search = None
            self.last_decision = SearchDecision(
                "IDLE", 0.0, 0.0, False)
            self.last_error = 0.0
            self.last_metrics = (0.0, 0.0, 0.0)
            self.last_status_publish = 0.0

            self.timer = rospy.Timer(
                rospy.Duration(1.0 / self.command_rate), self._timer_callback)
            rospy.loginfo(
                "Adaptive circle ready: radius=%.2f m, initial=%.2f m/s, "
                "step=%.2f m/s, derived_limit=%.2f m/s, auto_start=%s",
                self.radius, self.initial_speed, self.speed_step,
                self.speed_limit, self.auto_start)
            rospy.loginfo("Start with: rosservice call %s/start", rospy.get_name())

        def _validate(self, max_velocity, max_acceleration):
            positive = {
                "circle/radius": self.radius,
                "circle/initial_speed": self.initial_speed,
                "circle/speed_step": self.speed_step,
                "circle/speed_ramp_acceleration": self.ramp_acceleration,
                "circle/command_rate": self.command_rate,
                "evaluation/window_time": self.window_time,
                "evaluation/emergency_error": self.emergency_error,
                "limits/fsm_max_velocity": max_velocity,
                "limits/fsm_max_acceleration": max_acceleration,
            }
            for name, value in positive.items():
                if value <= 0.0:
                    raise ValueError("%s must be positive" % name)
            if self.settle_time < 0.0 or self.emergency_hold_time < 0.0:
                raise ValueError("settle and emergency hold times cannot be negative")
            if self.ramp_acceleration >= max_acceleration:
                raise ValueError(
                    "speed ramp acceleration must be below the FSM acceleration limit")
            if not (
                self.pass_rms_error < self.fail_rms_error < self.emergency_error):
                raise ValueError(
                    "error thresholds must satisfy pass_rms < fail_rms < emergency")
            if not self.pass_p95_error < self.fail_peak_error:
                raise ValueError("pass_p95_error must be below fail_peak_error")

        def _livo_callback(self, message):
            with self._lock:
                self.livo_odometry = message
                self.livo_received_at = rospy.get_time()

        def _px4_callback(self, message):
            with self._lock:
                self.px4_odometry = message
                self.px4_received_at = rospy.get_time()

        def _reference_callback(self, message):
            with self._lock:
                self.reference = message
                self.reference_received_at = rospy.get_time()

        def _fsm_status_callback(self, message):
            with self._lock:
                self.fsm_status = message
                self.fsm_status_received_at = rospy.get_time()

        def _start_callback(self, _request):
            success, message = self._start(rospy.get_time())
            return TriggerResponse(success=success, message=message)

        def _stop_callback(self, _request):
            stopped = self._deactivate("stop requested", request_hover=True)
            message = "adaptive circle stopped; hover requested" if stopped else "already stopped"
            return TriggerResponse(success=True, message=message)

        def _inputs_ready_locked(self, now):
            if not all((
                self.livo_odometry, self.px4_odometry,
                self.reference, self.fsm_status)):
                return False, "waiting for odometry, reference, or FSM status"
            if now - self.livo_received_at > self.feedback_timeout:
                return False, "FAST-LIVO odometry is stale"
            if now - self.px4_received_at > self.feedback_timeout:
                return False, "PX4 odometry is stale"
            if now - self.reference_received_at > self.feedback_timeout:
                return False, "OFFBOARD reference is stale"
            if now - self.fsm_status_received_at > self.status_timeout:
                return False, "OFFBOARD FSM status is stale"
            return True, "ready"

        def _start(self, now):
            with self._lock:
                if self.active:
                    return True, "adaptive circle is already running"
                ready, reason = self._inputs_ready_locked(now)
                if not ready:
                    return False, reason
                if self.fsm_status.state != FsmStatus.HOVER:
                    return False, "start is allowed only while OFFBOARD FSM is in HOVER"

                position = self.livo_odometry.pose.pose.position
                yaw = quaternion_yaw(self.livo_odometry.pose.pose.orientation)
                self.center, self.phase = initial_circle_geometry(
                    (position.x, position.y), yaw, self.radius, self.clockwise)
                self.altitude = position.z
                self.current_speed = 0.0
                self.last_tick = now
                self.trajectory_id += 1
                self.search = AdaptiveSpeedSearch(
                    self.initial_speed, self.speed_step,
                    self.settle_time, self.window_time,
                    self.pass_rms_error, self.pass_p95_error,
                    self.fail_rms_error, self.fail_peak_error,
                    self.emergency_error, self.emergency_hold_time,
                    self.speed_limit)
                self.last_decision = self.search._decision()
                self.last_error = 0.0
                self.last_metrics = (0.0, 0.0, 0.0)
                self.active = True

            rospy.logwarn(
                "Adaptive circle START: center=(%.3f, %.3f), altitude=%.3f, "
                "speed search %.2f -> %.2f m/s",
                self.center[0], self.center[1], self.altitude,
                self.initial_speed, self.speed_limit)
            return True, "adaptive circle started"

        def _deactivate(self, reason, request_hover):
            with self._lock:
                was_active = self.active
                self.active = False
                self.current_speed = 0.0
                self.last_decision = SearchDecision(
                    "STOPPED", 0.0,
                    self.search.last_safe_speed if self.search else 0.0,
                    request_hover)
            if request_hover and was_active:
                self.emergency_publisher.publish(Bool(data=True))
            if was_active:
                rospy.logwarn("Adaptive circle stopped: %s", reason)
            return was_active

        @staticmethod
        def _position_error(reference, odometry):
            dx = reference.pose.pose.position.x - odometry.pose.pose.position.x
            dy = reference.pose.pose.position.y - odometry.pose.pose.position.y
            dz = reference.pose.pose.position.z - odometry.pose.pose.position.z
            return math.sqrt(dx * dx + dy * dy + dz * dz)

        def _timer_callback(self, _event):
            now = rospy.get_time()
            if self.auto_start:
                with self._lock:
                    should_start = (
                        not self.active and self.fsm_status is not None and
                        self.fsm_status.state == FsmStatus.HOVER)
                if should_start:
                    success, reason = self._start(now)
                    if not success:
                        rospy.logwarn_throttle(2.0, "Adaptive circle auto-start waiting: %s", reason)

            with self._lock:
                active = self.active
                if active:
                    ready, stale_reason = self._inputs_ready_locked(now)
                    fsm_state = self.fsm_status.state if self.fsm_status else -1
                    reference = self.reference
                    px4_odometry = self.px4_odometry
                else:
                    ready, stale_reason = False, "inactive"
                    fsm_state = -1
                    reference = None
                    px4_odometry = None

            if not active:
                self.speed_publisher.publish(Float64(data=0.0))
                self._publish_status(now)
                return
            if not ready:
                self._deactivate(stale_reason, request_hover=True)
                return
            if fsm_state not in (FsmStatus.HOVER, FsmStatus.EXTERNAL):
                self._deactivate(
                    "OFFBOARD FSM left HOVER/EXTERNAL", request_hover=False)
                return

            error = self._position_error(reference, px4_odometry)
            dt = min(max(now - self.last_tick, 0.0), 0.1)
            self.last_tick = now
            tangential_acceleration = 0.0

            if fsm_state == FsmStatus.EXTERNAL and dt > 0.0:
                target = self.search.target_speed
                speed_delta = max(
                    -self.ramp_acceleration * dt,
                    min(self.ramp_acceleration * dt, target - self.current_speed))
                self.current_speed += speed_delta
                tangential_acceleration = speed_delta / dt
                direction = -1.0 if self.clockwise else 1.0
                self.phase += direction * self.current_speed / self.radius * dt
                self.phase = math.atan2(math.sin(self.phase), math.cos(self.phase))
                at_target = abs(self.current_speed - target) <= self.speed_tolerance
                decision = self.search.observe(now, error, at_target)
                self.last_decision = decision
                if decision.rms_error > 0.0:
                    self.last_metrics = (
                        decision.rms_error, decision.p95_error,
                        decision.peak_error)
                if decision.emergency:
                    self._deactivate(
                        "tracking error %.3f m exceeded emergency threshold" % error,
                        request_hover=True)
                    return

            command = circle_command(
                self.center, self.altitude, self.radius, self.phase,
                self.current_speed, self.clockwise,
                tangential_acceleration)
            message = PositionCommand()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = self.planner_frame
            message.position.x, message.position.y, message.position.z = command.position
            message.velocity.x, message.velocity.y, message.velocity.z = command.velocity
            message.acceleration.x, message.acceleration.y, message.acceleration.z = (
                command.acceleration)
            message.jerk.x, message.jerk.y, message.jerk.z = command.jerk
            message.yaw = command.yaw
            message.yaw_dot = command.yaw_rate
            message.trajectory_id = self.trajectory_id
            message.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
            # Serialize the final active-state check with the stop service so a
            # stop request cannot be followed by one more trajectory command.
            with self._lock:
                if not self.active:
                    return
                self.command_publisher.publish(message)

            self.last_error = error
            self.speed_publisher.publish(Float64(data=self.current_speed))
            self.error_publisher.publish(Float64(data=error))
            self._publish_status(now)
            rospy.loginfo_throttle(
                1.0,
                "Adaptive circle: speed=%.2f target=%.2f limit=%.2f m/s "
                "error=%.3f m state=%s last_safe=%.2f",
                self.current_speed, self.search.target_speed,
                self.speed_limit, error, self.search.state,
                self.search.last_safe_speed)

        def _publish_status(self, now):
            if now - self.last_status_publish < 0.2:
                return
            self.last_status_publish = now
            rms, p95, peak = self.last_metrics
            text = (
                "state=%s speed=%.3f target=%.3f last_safe=%.3f "
                "limit=%.3f error=%.3f rms=%.3f p95=%.3f peak=%.3f" %
                (self.last_decision.state, self.current_speed,
                 self.last_decision.target_speed,
                 self.last_decision.last_safe_speed,
                 self.speed_limit, self.last_error, rms, p95, peak))
            self.search_status_publisher.publish(String(data=text))

    try:
        AdaptiveCircleNode()
    except ValueError as error:
        rospy.logfatal("Invalid adaptive circle configuration: %s", error)
        raise
    rospy.spin()


def main():
    import rospy
    rospy.init_node("adaptive_circle")
    run_ros_node()


if __name__ == "__main__":
    main()

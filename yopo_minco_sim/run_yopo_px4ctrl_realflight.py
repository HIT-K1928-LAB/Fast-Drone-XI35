"""Real-flight adapter: debug output by default; no arming or takeoff commands.
Native TensorRT engines are NOT supported by the inherited torch2trt loader.
Camera geometry and model outputs must be validated before command execution.
"""
import os
for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ[key] = "1"

import argparse
import sys
import time
import threading
from pathlib import Path

algorithm_root = Path(__file__).resolve().parents[1]
yopo_dir = algorithm_root / "third_party/YOPO/YOPO"
sys.path.insert(0, str(yopo_dir))

def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--weight', type=Path, default=yopo_dir / 'saved/YOPO_1/epoch50.pth',
                        help='PyTorch .pth only; native .engine requires a separate backend')
    parser.add_argument('--odom-topic', default='/kf_fusion/kf_imu_odom')
    parser.add_argument('--depth-topic', default='/camera/depth/image_rect_raw')
    parser.add_argument('--state-topic', default='/mavros/state')
    parser.add_argument('--depth-width', type=int, default=640)
    parser.add_argument('--depth-height', type=int, default=480)
    parser.add_argument('--pitch-angle-deg', type=float, default=0.0,
                        help='Measured mounting pitch; zero is an unverified initial value')
    for name, default in [
        ('speed', 0.3), ('max-speed', 0.5), ('max-acceleration', 1.0),
        ('goal-min-distance', 0.5), ('goal-max-distance', 2.0),
        ('arrive-distance', 0.25), ('arrive-speed', 0.15), ('arrive-hold', 0.5),
        ('hover-speed', 0.15), ('min-height', 0.6), ('max-height', 1.5),
        ('height-band', 0.3), ('max-tracking-error', 0.5),
        ('sensor-timeout', 0.3), ('state-timeout', 1.0), ('max-sensor-skew', 0.1),
        ('plan-timeout', 0.2), ('max-invalid-depth', 0.3),
        ('safe-radius', 0.35)]:
        parser.add_argument('--' + name, type=float, default=default)
    args = parser.parse_args(argv)
    import math
    for name, value in vars(args).items():
        if isinstance(value, float) and (not math.isfinite(value) or (name != 'pitch_angle_deg' and value <= 0)):
            parser.error(name + ' must be finite and positive')
    if args.depth_width <= 0 or args.depth_height <= 0:
        parser.error('depth dimensions must be positive')
    if not (args.arrive_distance < args.goal_min_distance < args.goal_max_distance):
        parser.error('require arrive-distance < goal-min-distance < goal-max-distance')
    if args.min_height >= args.max_height or args.speed > args.max_speed or args.max_invalid_depth >= 1:
        parser.error('invalid height, speed or invalid-depth limits')
    if args.weight.suffix != '.pth':
        parser.error('--weight accepts PyTorch .pth only, not ONNX/native TensorRT engines')
    return args


if __name__ == '__main__':
    # Retain ROS remapping arguments for rospy, exclude them from argparse.
    ARGS = parse_args([arg for arg in sys.argv[1:] if ':=' not in arg])


import numpy as np
import rospy
from scipy.spatial.transform import Rotation
from mavros_msgs.msg import State
from std_msgs.msg import Empty
from quadrotor_msgs.msg import PositionCommand
import test_yopo_ros as upstream
from config.config import cfg

# Both Publisher construction and message creation in upstream use this symbol.
upstream.PositionCommand = PositionCommand


class CheckedPublisher:
    def __init__(self, owner, publisher):
        self.owner, self.publisher = owner, publisher

    def publish(self, msg):
        values = [getattr(v, k) for v in (msg.position, msg.velocity, msg.acceleration)
                  for k in ("x", "y", "z")] + [msg.yaw, msg.yaw_dot]
        if not np.isfinite(values).all() or msg.trajectory_flag != msg.TRAJECTORY_STATUS_READY:
            self.owner.stop("invalid or non-READY command")
            return
        pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        if self.owner.stop_requested.is_set():
            self.owner.stop("user stop requested")
            return
        if np.linalg.norm(pos - self.owner._odom_pos()) > self.owner.args.max_tracking_error:
            self.owner.stop("reference tracking error exceeds configured limit")
            return
        problem = self.owner.input_problem()
        if problem:
            self.owner.stop(problem)
            return
        vel = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
        acc = np.array([msg.acceleration.x, msg.acceleration.y, msg.acceleration.z])
        problem = self.owner.reference_problem(pos, vel, acc)
        if problem:
            self.owner.stop(problem)
            return
        msg.header.frame_id = "world"
        self.publisher.publish(msg)


class RealFlightYopo(upstream.YopoNet):
    def __init__(self, args):
        self.args = args
        execute = args.execute
        self.arrival_since = None
        self.plan_started = -float("inf")
        self.goal_origin = None
        self.stop_requested = threading.Event()
        self.guard = threading.RLock()#locked， avoid modifying same variables simultaneously .
        self.active = False
        self.execute = execute
        self.fcu = State()
        self.state_received = self.odom_received = self.depth_received = -float("inf")#ensure odometry not received when start
        self.latest_depth_stamp = None
        self.latest_depth_shape = None
        rospy.init_node("yopo_net", anonymous=False)
        self.state_sub = rospy.Subscriber(args.state_topic, State, self.receive_state, queue_size=1)
        self.stop_sub = rospy.Subscriber("/yopo_minco/stop", Empty, self.receive_stop, queue_size=1)
        output = "/position_cmd" if execute else "/yopo_debug/position_cmd"
        rospy.logwarn("Real-flight adapter: output=%s. WAITING FOR GOAL; no automatic movement.", output)
        settings = dict(use_tensorrt=False, goal=[0, 0, 1.2], topk=1,
                        pitch_angle_deg=args.pitch_angle_deg, odom_topic=args.odom_topic,
                        depth_topic=args.depth_topic,
                        ctrl_topic=output, plan_from_reference=True, verbose=False)
        super().__init__(settings, str(args.weight))

    def warm_up(self):
        self.safe_radius = self.args.safe_radius
        self.safe_mu = 1.0 - np.exp(-self.safe_radius / self.radius_lambda)
        rospy.logwarn('Predicted corridor threshold %.3f m is NOT verified physical clearance.', self.safe_radius)
        super().warm_up()

    def receive_state(self, msg):
        # Do not wait behind GPU work merely to cache flight state.
        self.fcu = msg
        self.state_received = time.monotonic()

    def stop(self, reason):#stop publishing yopo position_command,not the power of drone  
        if self.active:
            rospy.logwarn("YOPO STOP: %s. Command publishing stopped; wait for PX4Ctrl AUTO_HOVER.", reason)
        self.active = False#YOPO task deactive
        self.desire_init = False#abolish the velocity,acceleration and position of this trajectory
        self.ctrl_time = None#clear the ctrl_time being recorded.

    def receive_stop(self, _msg):#/yopo_minco/stop收到stop消息，调用stop函数并打印原因
        self.stop_requested.set()
        with self.guard:
            self.stop("user request")

    def input_problem(self):#检查问题
        now = time.monotonic()
        if self.stop_requested.is_set():
            return "user stop requested"
        if now - self.odom_received > self.args.sensor_timeout or now - self.depth_received > self.args.sensor_timeout:
            return "odometry/depth missing or stale"
        if self.latest_depth_shape != (self.args.depth_width, self.args.depth_height):
            return "depth size does not match configured input dimensions"
        if self.odom.header.frame_id != "world":
            return "odometry must be in world"
        for stamp in (self.odom.header.stamp, self.latest_depth_stamp):
            age = (rospy.Time.now() - stamp).to_sec()
            if age < -0.05 or age > self.args.sensor_timeout:
                return "sensor timestamp out of range"
        if abs((self.odom.header.stamp - self.latest_depth_stamp).to_sec()) > self.args.max_sensor_skew:
            return 'depth/odometry timestamp skew exceeds limit'
        q = self.odom.pose.pose.orientation
        quat = np.array([q.x, q.y, q.z, q.w])
        if not np.isfinite(np.r_[self._odom_pos(), self._odom_vel(), quat]).all() or abs(np.linalg.norm(quat) - 1.0) > 0.05:
            return 'invalid odometry position, velocity or quaternion'
        if self.execute and (now - self.state_received > self.args.state_timeout or not self.fcu.connected
                             or not self.fcu.armed or self.fcu.mode != "OFFBOARD"):
            return "PX4 must be connected, armed and OFFBOARD"
        return None

    def seed_reference(self):#给新一轮规划准备起点。
        self.desire_pos = self._odom_pos()
        self.desire_vel = self._odom_vel()
        self.desire_acc = np.zeros(3)
        q = self.odom.pose.pose.orientation
        self.last_yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("ZYX")[0]
        self.last_control_msg = None
        self.ctrl_time = None
        self.brake = self.arrive = False
        self.desire_init = True

    def callback_odometry(self, msg):#收到里程计的调用后：保存；判断是否有任务；是否到达
        with self.guard:
            self.odom, self.odom_init = msg, True
            self.odom_received = time.monotonic()
            if not self.active:
                self.desire_init = False
                return
            near = np.linalg.norm(self._odom_pos() - self.goal) < self.args.arrive_distance
            slow = np.linalg.norm(self._odom_vel()) < self.args.arrive_speed
            if near and slow:
                if self.arrival_since is None:
                    self.arrival_since = time.monotonic()
                if time.monotonic() - self.arrival_since >= self.args.arrive_hold:
                    self.stop('arrived and settled')
            else:
                self.arrival_since = None 

    def callback_set_goal(self, msg):#收到目标进行安全检查，通过则启动YOPO任务
        with self.guard:
            if self.active:
                rospy.logwarn("Stop the current test before sending a new goal.")
                return
            self.stop_requested.clear()
            problem = self.input_problem()
            if problem:
                rospy.logwarn("Goal rejected: %s", problem)
                return
            if msg.header.frame_id != "world":
                rospy.logwarn("Goal frame must be world.")
                return
            p = self._odom_pos()
            if not self.args.min_height <= p[2] <= self.args.max_height or np.linalg.norm(self._odom_vel()) > self.args.hover_speed:
                rospy.logwarn("Goal rejected: establish stable hover within configured height bounds.")
                return
            # Initial tests remain at the current altitude, including RViz 2D goals.
            goal = np.array([msg.pose.position.x, msg.pose.position.y, p[2]])#用真实位置P[2]防止改变高度
            distance = np.linalg.norm(goal - p)
            if not np.isfinite(goal).all() or not self.args.goal_min_distance <= distance <= self.args.goal_max_distance:
                rospy.logwarn("Goal rejected: target outside configured distance limits.")
                return
            self.goal = goal
            self.goal_origin = p.copy()
            self.arrival_since = None
            self.seed_reference()
            self.active = True
            rospy.logwarn("GOAL ACCEPTED: %s; output=%s", goal.tolist(),
                          "/position_cmd" if self.execute else "/yopo_debug/position_cmd")

    def callback_depth(self, msg):#收到深度图消息后，加锁记录时间戳和尺寸,并交给父类规划
        with self.guard:
            self.depth_received = time.monotonic()
            self.latest_depth_stamp = msg.header.stamp
            self.latest_depth_shape = (msg.width, msg.height)
            if not self.active:
                return
            problem = self.input_problem()
            if problem:
                self.stop(problem)
                return
            try:
                started = time.monotonic()
                super().callback_depth(msg)
                self.plan_started = started
                if time.monotonic() - started > self.args.plan_timeout:
                    self.stop('planning exceeded time budget')
                    return
                problem = self.input_problem()
                if problem:
                    self.stop(problem)
                    return
                # Sample the entire selected trajectory, not only its endpoint.
                horizon = float(self.best_total_time)
                if not np.isfinite(horizon) or horizon <= 0 or horizon > 120:
                    self.stop('invalid trajectory duration')
                    return
                for t in np.linspace(0, horizon, max(2, int(np.ceil(horizon / 0.02)) + 1)):
                    problem = self.reference_problem(self.optimal_traj.position(t), self.optimal_traj.velocity(t), self.optimal_traj.acceleration(t))
                    if problem:
                        self.stop(problem)
                        return
                if self.brake:
                    self.stop("upstream corridor admission requested braking")
            except Exception:
                self.stop("depth/planning exception")
                rospy.logerr("Planning failed", exc_info=True)

    def reference_problem(self, pos, vel, acc):
        if not np.isfinite(np.r_[pos, vel, acc]).all():
            return 'non-finite trajectory'
        if np.linalg.norm(vel) > self.args.max_speed or np.linalg.norm(acc) > self.args.max_acceleration:
            return 'trajectory velocity/acceleration exceeds limit'
        if not self.args.min_height <= pos[2] <= self.args.max_height:
            return 'trajectory outside height limits'
        if self.goal_origin is not None:
            if abs(pos[2] - self.goal_origin[2]) > self.args.height_band:
                return 'trajectory outside hover height band'
            if np.linalg.norm(pos[:2] - self.goal_origin[:2]) > self.args.goal_max_distance + self.args.max_tracking_error:
                return 'trajectory outside local test area'
        return None

    def _process_depth(self, msg):
        if msg.encoding not in ('16UC1', '32FC1'):
            raise ValueError('Unsupported depth encoding: ' + msg.encoding)
        dtype = np.dtype(('>' if msg.is_bigendian else '<') + ('u2' if msg.encoding == '16UC1' else 'f4'))
        if msg.step < msg.width * dtype.itemsize or len(msg.data) < msg.step * msg.height:
            raise ValueError('Invalid depth data length/stride')
        depth = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                           strides=(msg.step, dtype.itemsize)).astype(np.float32)
        if msg.encoding == '16UC1':
            depth /= 1000.0
        invalid = ~np.isfinite(depth) | (depth < self.min_dis)
        if invalid.mean() > self.args.max_invalid_depth:
            raise ValueError('Too many invalid depth pixels')
        depth[invalid] = 0
        # Parent preprocessing expects packed native float32 meter depth.
        from sensor_msgs.msg import Image
        packed = Image()
        packed.height, packed.width = msg.height, msg.width
        packed.encoding, packed.step = '32FC1', msg.width * 4
        packed.data = depth.astype('<f4').tobytes()
        return super()._process_depth(packed)

    def control_pub(self, event):#周期性地从当前轨迹上取出一个控制点，并发布成 PositionCommand。
        with self.guard:
            if not self.active:
                return
            problem = self.input_problem()
            if problem:
                self.stop(problem)
                return
            if time.monotonic() - self.plan_started > self.args.plan_timeout and self.ctrl_time is not None:
                self.stop('trajectory plan stale')
                return
            if self.ctrl_time is None:
                return  # Wait for a newly solved trajectory after each goal.
            self.ctrl_time = max(0.0, time.monotonic() - self.plan_started - self.ctrl_dt)
            if self.ctrl_time + self.ctrl_dt > getattr(self, "best_total_time", self.traj_time):
                self.stop("trajectory expired")
                return
            if self.brake or self.arrive:
                self.stop("planner stop condition")
                return
            if not isinstance(self.ctrl_pub, CheckedPublisher):
                self.ctrl_pub = CheckedPublisher(self, self.ctrl_pub)
            try:
                super().control_pub(event)
            except Exception:
                self.stop("control sampling exception")
                rospy.logerr("Control sampling failed", exc_info=True)


if __name__ == "__main__":
    if not ARGS.weight.is_file():
        raise SystemExit('Weight file not found: ' + str(ARGS.weight))
    cfg['velocity'] = ARGS.speed
    RealFlightYopo(ARGS)

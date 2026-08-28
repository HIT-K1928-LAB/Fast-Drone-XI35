#!/usr/bin/env python3

"""Publish PositionCommand excitations for px4ctrl PID tuning.

The node is intentionally one-shot and does not inspect flight state. It
captures the current odometry pose, publishes a settling reference, runs the
requested excitation, returns to the captured pose, and then stops publishing.
"""

import argparse
import math
import sys

import rospy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Float64, UInt8


PHASE_IDLE = 0
PHASE_SETTLE = 1
PHASE_EXCITE_POSITIVE = 2
PHASE_EXCITE_NEGATIVE = 3
PHASE_RECOVER = 4


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def finite(value):
    return math.isfinite(value)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Publish bounded, odometry-relative PositionCommand excitations."
    )
    parser.add_argument(
        "--mode",
        choices=("step", "doublet", "sine", "chirp"),
        default="doublet",
        help="Excitation waveform (default: doublet).",
    )
    parser.add_argument(
        "--axis",
        choices=("forward", "lateral", "x", "y", "z", "yaw"),
        default="forward",
        help="Excitation direction. forward/lateral use the yaw captured at start.",
    )
    parser.add_argument(
        "--amplitude",
        type=float,
        default=0.15,
        help="Position amplitude in metres, or yaw amplitude in degrees.",
    )
    parser.add_argument("--hold", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--settle", type=float, default=2.0)
    parser.add_argument("--recover", type=float, default=4.0)
    parser.add_argument("--frequency", type=float, default=0.3)
    parser.add_argument("--start-frequency", type=float, default=0.1)
    parser.add_argument("--end-frequency", type=float, default=1.0)
    parser.add_argument(
        "--derivatives",
        choices=("zero", "consistent"),
        default="zero",
        help=(
            "For sine/chirp, publish zero v/a/jerk to isolate feedback PID, "
            "or mathematically consistent derivatives to test trajectory tracking."
        ),
    )
    parser.add_argument("--start-delay", type=float, default=0.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument(
        "--odom-topic", default="/gazebo/iris_0/odometry"
    )
    parser.add_argument("--cmd-topic", default="/position_cmd")
    parser.add_argument("--output-ns", default="/px4ctrl_tune")
    return parser.parse_args(argv)


def validate_args(args):
    numeric = (
        args.amplitude,
        args.hold,
        args.duration,
        args.settle,
        args.recover,
        args.frequency,
        args.start_frequency,
        args.end_frequency,
        args.start_delay,
        args.rate,
    )
    if not all(finite(value) for value in numeric):
        raise ValueError("all numeric arguments must be finite")
    if args.rate < 10.0 or args.rate > 200.0:
        raise ValueError("--rate must be within [10, 200] Hz")
    if min(args.hold, args.duration, args.settle, args.recover, args.start_delay) < 0.0:
        raise ValueError("durations must be non-negative")
    if args.mode in ("step", "doublet") and args.hold <= 0.0:
        raise ValueError("--hold must be positive for step and doublet")
    if args.mode in ("sine", "chirp") and args.duration <= 0.0:
        raise ValueError("--duration must be positive for sine and chirp")
    if args.axis == "yaw":
        if abs(args.amplitude) > 30.0:
            raise ValueError("yaw amplitude is limited to 30 degrees")
    elif abs(args.amplitude) > 0.5:
        raise ValueError("position amplitude is limited to 0.5 metres")
    if args.mode == "sine" and not 0.02 <= args.frequency <= 2.0:
        raise ValueError("--frequency must be within [0.02, 2.0] Hz")
    if args.mode == "chirp":
        if not 0.02 <= args.start_frequency < args.end_frequency <= 2.0:
            raise ValueError("chirp frequencies must satisfy 0.02 <= start < end <= 2.0")


class SetpointExcitation:
    def __init__(self, args):
        self.args = args
        self.odom = None
        self.origin = None
        self.direction = None

        self.cmd_pub = rospy.Publisher(args.cmd_topic, PositionCommand, queue_size=1)
        self.input_pub = rospy.Publisher(
            args.output_ns + "/excitation/input", Float64, queue_size=1
        )
        self.phase_pub = rospy.Publisher(
            args.output_ns + "/excitation/phase", UInt8, queue_size=1, latch=True
        )
        self.odom_sub = rospy.Subscriber(
            args.odom_topic, Odometry, self.odom_callback, queue_size=1
        )

    def odom_callback(self, msg):
        self.odom = msg

    def publish_phase(self, phase):
        self.phase_pub.publish(UInt8(data=phase))

    def wait_for_inputs(self):
        rospy.loginfo("Waiting for one odometry message on %s...", self.args.odom_topic)
        wait_rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if self.odom is not None:
                return True
            wait_rate.sleep()
        return False

    def capture_origin(self):
        pose = self.odom.pose.pose
        yaw = quaternion_to_yaw(pose.orientation)
        values = (pose.position.x, pose.position.y, pose.position.z, yaw)
        if not all(finite(value) for value in values):
            raise RuntimeError("odometry origin contains NaN or Inf")
        self.origin = values

        if self.args.axis == "forward":
            self.direction = (math.cos(yaw), math.sin(yaw), 0.0)
        elif self.args.axis == "lateral":
            self.direction = (-math.sin(yaw), math.cos(yaw), 0.0)
        elif self.args.axis == "x":
            self.direction = (1.0, 0.0, 0.0)
        elif self.args.axis == "y":
            self.direction = (0.0, 1.0, 0.0)
        elif self.args.axis == "z":
            self.direction = (0.0, 0.0, 1.0)
        else:
            self.direction = (0.0, 0.0, 0.0)

    def make_command(self, offset, velocity, acceleration, jerk):
        msg = PositionCommand()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"

        msg.position.x = self.origin[0] + self.direction[0] * offset
        msg.position.y = self.origin[1] + self.direction[1] * offset
        msg.position.z = self.origin[2] + self.direction[2] * offset
        msg.velocity.x = self.direction[0] * velocity
        msg.velocity.y = self.direction[1] * velocity
        msg.velocity.z = self.direction[2] * velocity
        msg.acceleration.x = self.direction[0] * acceleration
        msg.acceleration.y = self.direction[1] * acceleration
        msg.acceleration.z = self.direction[2] * acceleration
        msg.jerk.x = self.direction[0] * jerk
        msg.jerk.y = self.direction[1] * jerk
        msg.jerk.z = self.direction[2] * jerk

        msg.yaw = self.origin[3]
        msg.yaw_dot = 0.0
        if self.args.axis == "yaw":
            msg.yaw += offset
            msg.yaw_dot = velocity

        msg.trajectory_id = 1
        msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        return msg

    def publish_sample(self, offset, velocity=0.0, acceleration=0.0, jerk=0.0):
        if self.args.derivatives == "zero":
            velocity = 0.0
            acceleration = 0.0
            jerk = 0.0
        self.cmd_pub.publish(self.make_command(offset, velocity, acceleration, jerk))
        self.input_pub.publish(Float64(data=offset))

    def run_constant(self, offset, duration, phase):
        self.publish_phase(phase)
        start = rospy.Time.now()
        rate = rospy.Rate(self.args.rate)
        while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < duration:
            self.publish_sample(offset)
            rate.sleep()

    def run_sine(self):
        self.publish_phase(PHASE_EXCITE_POSITIVE)
        amplitude = self.command_amplitude()
        omega = 2.0 * math.pi * self.args.frequency
        start = rospy.Time.now()
        rate = rospy.Rate(self.args.rate)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed >= self.args.duration:
                return
            phase = omega * elapsed
            offset = amplitude * math.sin(phase)
            velocity = amplitude * omega * math.cos(phase)
            acceleration = -amplitude * omega * omega * math.sin(phase)
            jerk = -amplitude * omega * omega * omega * math.cos(phase)
            self.publish_sample(offset, velocity, acceleration, jerk)
            rate.sleep()

    def run_chirp(self):
        self.publish_phase(PHASE_EXCITE_POSITIVE)
        amplitude = self.command_amplitude()
        frequency_slope = (
            self.args.end_frequency - self.args.start_frequency
        ) / self.args.duration
        angular_acceleration = 2.0 * math.pi * frequency_slope
        start = rospy.Time.now()
        rate = rospy.Rate(self.args.rate)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed >= self.args.duration:
                return
            phase = 2.0 * math.pi * (
                self.args.start_frequency * elapsed
                + 0.5 * frequency_slope * elapsed * elapsed
            )
            omega = 2.0 * math.pi * (
                self.args.start_frequency + frequency_slope * elapsed
            )
            offset = amplitude * math.sin(phase)
            velocity = amplitude * omega * math.cos(phase)
            acceleration = amplitude * (
                -math.sin(phase) * omega * omega
                + math.cos(phase) * angular_acceleration
            )
            jerk = amplitude * (
                -math.cos(phase) * omega * omega * omega
                - 3.0 * math.sin(phase) * omega * angular_acceleration
            )
            self.publish_sample(offset, velocity, acceleration, jerk)
            rate.sleep()

    def command_amplitude(self):
        if self.args.axis == "yaw":
            return math.radians(self.args.amplitude)
        return self.args.amplitude

    def run(self):
        self.publish_phase(PHASE_IDLE)
        if not self.wait_for_inputs():
            return 1

        rospy.loginfo(
            "PID excitation starts in %.1f s: mode=%s axis=%s amplitude=%.3f%s. "
            "The script does not check flight state.",
            self.args.start_delay,
            self.args.mode,
            self.args.axis,
            self.args.amplitude,
            " deg" if self.args.axis == "yaw" else " m",
        )
        rospy.sleep(self.args.start_delay)

        self.capture_origin()
        amplitude = self.command_amplitude()
        self.run_constant(0.0, self.args.settle, PHASE_SETTLE)

        if self.args.mode == "step":
            self.run_constant(amplitude, self.args.hold, PHASE_EXCITE_POSITIVE)
        elif self.args.mode == "doublet":
            self.run_constant(amplitude, self.args.hold, PHASE_EXCITE_POSITIVE)
            self.run_constant(-amplitude, self.args.hold, PHASE_EXCITE_NEGATIVE)
        elif self.args.mode == "sine":
            self.run_sine()
        elif self.args.mode == "chirp":
            self.run_chirp()

        self.run_constant(0.0, self.args.recover, PHASE_RECOVER)

        self.publish_phase(PHASE_IDLE)
        self.input_pub.publish(Float64(data=0.0))
        rospy.loginfo(
            "Excitation complete; command publication stopped. px4ctrl should return to AUTO_HOVER."
        )
        return 0


def main():
    args = parse_args(rospy.myargv(argv=sys.argv)[1:])
    try:
        validate_args(args)
    except ValueError as error:
        print("argument error: {}".format(error), file=sys.stderr)
        return 2

    rospy.init_node("pid_setpoint_excitation")
    try:
        return SetpointExcitation(args).run()
    except (RuntimeError, rospy.ROSException, rospy.ROSInterruptException) as error:
        rospy.logerr("Excitation failed: %s", error)
        return 1


if __name__ == "__main__":
    sys.exit(main())

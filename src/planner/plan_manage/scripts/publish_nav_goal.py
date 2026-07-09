#!/usr/bin/env python3

import argparse
import math
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


def parse_args():
    parser = argparse.ArgumentParser(
        description="Publish an EGO Planner search_plan-compatible 3D target PoseStamped message.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "xyz",
        type=float,
        nargs="*",
        help="Target position: x y z. If omitted, enter interactive input mode.",
    )
    parser.add_argument(
        "--default-z",
        type=float,
        default=1.0,
        help="Default z used when only x y are provided.",
    )
    parser.add_argument(
        "--yaw",
        type=float,
        default=0.0,
        help="Target yaw in degrees.",
    )
    parser.add_argument("--frame-id", default="world", help="Header frame_id.")
    parser.add_argument("--topic", default="/search_plan/pos_cmd", help="Target topic.")
    parser.add_argument(
        "--repeat",
        type=int,
        default=3,
        help="Publish count. Repeating helps when subscribers connect slowly.",
    )
    parser.add_argument("--rate", type=float, default=2.0, help="Publish rate in Hz.")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    if len(args.xyz) not in (0, 2, 3):
        parser.error("provide either no position for interactive mode, or x y [z]")
    return args


def make_goal(x, y, z, yaw, frame_id):
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z

    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, math.radians(yaw))
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


def publish_goal(pub, rate, args, x, y, z):
    goal = make_goal(x, y, z, args.yaw, args.frame_id)
    for _ in range(args.repeat):
        if rospy.is_shutdown():
            break
        goal.header.stamp = rospy.Time.now()
        pub.publish(goal)
        rate.sleep()

    rospy.loginfo(
        "Published target to %s: frame=%s xyz=(%.2f, %.2f, %.2f) yaw=%.1f deg",
        args.topic,
        args.frame_id,
        x,
        y,
        z,
        args.yaw,
    )


def parse_goal_line(line, default_z):
    parts = line.strip().split()
    if len(parts) == 2:
        x, y = (float(v) for v in parts)
        return x, y, default_z
    if len(parts) == 3:
        return tuple(float(v) for v in parts)
    raise ValueError("please input: x y [z]")


def interactive_loop(pub, rate, args):
    print("Input target as: x y [z]. Press Ctrl-D or type q/quit/exit to stop.")
    while not rospy.is_shutdown():
        try:
            line = input("goal> ")
        except EOFError:
            print()
            break

        line = line.strip()
        if not line:
            continue
        if line.lower() in ("q", "quit", "exit"):
            break

        try:
            x, y, z = parse_goal_line(line, args.default_z)
        except ValueError as exc:
            print(exc)
            continue

        publish_goal(pub, rate, args, x, y, z)


def main():
    args = parse_args()
    if args.repeat < 1:
        print("--repeat must be >= 1", file=sys.stderr)
        return 1
    if args.rate <= 0.0:
        print("--rate must be > 0", file=sys.stderr)
        return 1

    rospy.init_node("publish_nav_goal", anonymous=True)
    pub = rospy.Publisher(args.topic, PoseStamped, queue_size=1, latch=True)
    rate = rospy.Rate(args.rate)

    if len(args.xyz) == 0:
        interactive_loop(pub, rate, args)
    else:
        x, y = args.xyz[0], args.xyz[1]
        z = args.xyz[2] if len(args.xyz) == 3 else args.default_z
        publish_goal(pub, rate, args, x, y, z)

    rospy.sleep(0.2)
    return 0


if __name__ == "__main__":
    sys.exit(main())

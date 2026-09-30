#!/usr/bin/env python3
"""One explicit user action submits an atomic world goal + start request.

No automatic retries: a lost response must not silently start another task.
"""
import math
import time
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from yopo_minco_planner.srv import StartGoal


def parse_goal(line):
    parts = line.split()
    if len(parts) != 3:
        raise ValueError("please input x y z in world frame")
    values = [float(v) for v in parts]
    if not all(math.isfinite(v) for v in values):
        raise ValueError("input should be finite numbers")
    return values


def main():
    rospy.init_node("minco_goal_console", anonymous=True, disable_signals=True)
    stop = rospy.Publisher("/yopo_minco/stop", Empty, queue_size=1, latch=False)
    print("x y z = set goal;resume = resume task;stop = cancel task; q = quit terminal")
    print("q/Ctrl-C cannot cancel the existing task;enter stop when you need to stop.")
    while not rospy.is_shutdown():
        try:
            line = input("minco> ").strip()
            if not line:
                continue
            if line == "q":
                break
            if line == "stop":
                until = time.monotonic() + 2.0
                while stop.get_num_connections() == 0 and time.monotonic() < until:
                    time.sleep(.05)
                if stop.get_num_connections() == 0:
                    print("planner not conneted:no stop was sent")
                    continue
                stop.publish(Empty())
                print("Stop request sent,check the node STOP log")
                continue
            if line == "resume":
                rospy.wait_for_service("/yopo_minco/resume", timeout=3)
                response = rospy.ServiceProxy("/yopo_minco/resume", Trigger)()
            else:
                x, y, z = parse_goal(line)
                rospy.wait_for_service("/yopo_minco/start_goal", timeout=3)
                goal = PoseStamped()
                goal.header.frame_id = "world"
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = x, y, z
                goal.pose.orientation.w = 1.0
                response = rospy.ServiceProxy("/yopo_minco/start_goal", StartGoal)(goal)
            print(("accepted:" if response.success else "rejected:") + response.message)
        except (ValueError, rospy.ROSException, rospy.ServiceException) as exc:
            print("request fail(no automatic retry):", exc)
        except (EOFError, KeyboardInterrupt):
            break


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

"""Hardware-in-the-loop check for FAST-LIVO2 IMU-rate odometry."""

import argparse
import threading
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class StampCollector:
    def __init__(self):
        self._lock = threading.Lock()
        self._stamps = []

    def callback(self, message):
        with self._lock:
            self._stamps.append(message.header.stamp.to_sec())

    def clear(self):
        with self._lock:
            self._stamps = []

    def snapshot(self):
        with self._lock:
            return list(self._stamps)


def rate(stamps):
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        return 0.0
    return (len(stamps) - 1) / (stamps[-1] - stamps[0])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--warmup", type=float, default=2.0)
    parser.add_argument("--minimum-ratio", type=float, default=0.95)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("test_imu_propagation_rate", anonymous=True)
    imu = StampCollector()
    odom = StampCollector()
    rospy.Subscriber("/livox/imu", Imu, imu.callback, queue_size=10000,
                     tcp_nodelay=True)
    rospy.Subscriber("/LIVO2/imu_propagate", Odometry, odom.callback,
                     queue_size=10000, tcp_nodelay=True)

    time.sleep(args.warmup)
    imu.clear()
    odom.clear()
    time.sleep(args.duration)

    imu_stamps = imu.snapshot()
    odom_stamps = odom.snapshot()
    imu_rate = rate(imu_stamps)
    odom_rate = rate(odom_stamps)
    ratio = odom_rate / imu_rate if imu_rate > 0.0 else 0.0
    odom_monotonic = all(
        current > previous
        for previous, current in zip(odom_stamps, odom_stamps[1:]))

    print("livox_imu_hz={:.3f}".format(imu_rate))
    print("imu_propagate_hz={:.3f}".format(odom_rate))
    print("rate_ratio={:.4f}".format(ratio))
    print("odom_timestamps_strictly_increasing={}".format(odom_monotonic))

    if imu_rate < 100.0:
        raise SystemExit("Livox IMU input is not running at the expected rate")
    if not odom_monotonic:
        raise SystemExit("IMU propagation timestamps are not strictly increasing")
    if ratio < args.minimum_ratio:
        raise SystemExit(
            "IMU propagation rate ratio {:.4f} is below {:.4f}".format(
                ratio, args.minimum_ratio))


if __name__ == "__main__":
    main()

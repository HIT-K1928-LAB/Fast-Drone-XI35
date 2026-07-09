#!/usr/bin/env python3
import copy
import rospy
import message_filters
from sensor_msgs.msg import Image

#python3 stereo_sync_throttle.py   _left_in:=/camera/infra1/image_rect_raw   _right_in:=/camera/infra2/image_recraw   _left_out:=/camera/infra1/image_rect_raw_downsample   _right_out:=/camera/infra2/image_rect_raw_downsample   _freq:=4.0   _slop:=0.02


class StereoSyncThrottle:
    def __init__(self):
        self.left_in = rospy.get_param("~left_in", "/camera/infra1/image_rect_raw")
        self.right_in = rospy.get_param("~right_in", "/camera/infra2/image_rect_raw")
        self.left_out = rospy.get_param("~left_out", "/camera/infra1/image_rect_raw_downsample")
        self.right_out = rospy.get_param("~right_out", "/camera/infra2/image_rect_raw_downsample")

        self.freq = float(rospy.get_param("~freq", 4.0))
        self.slop = float(rospy.get_param("~slop", 0.02))
        self.force_same_stamp = bool(rospy.get_param("~force_same_stamp", False))

        self.interval = 1.0 / self.freq
        self.last_pub_time = None
        self.count = 0

        self.left_pub = rospy.Publisher(self.left_out, Image, queue_size=10)
        self.right_pub = rospy.Publisher(self.right_out, Image, queue_size=10)

        left_sub = message_filters.Subscriber(self.left_in, Image)
        right_sub = message_filters.Subscriber(self.right_in, Image)

        sync = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub],
            queue_size=100,
            slop=self.slop,
            allow_headerless=False
        )
        sync.registerCallback(self.callback)

        rospy.loginfo("Stereo sync throttle started.")
        rospy.loginfo("left_in:  %s", self.left_in)
        rospy.loginfo("right_in: %s", self.right_in)
        rospy.loginfo("left_out: %s", self.left_out)
        rospy.loginfo("right_out:%s", self.right_out)
        rospy.loginfo("freq: %.2f Hz, slop: %.4f s", self.freq, self.slop)

    def callback(self, left_msg, right_msg):
        left_t = left_msg.header.stamp.to_sec()
        right_t = right_msg.header.stamp.to_sec()
        pair_t = 0.5 * (left_t + right_t)

        if self.last_pub_time is not None:
            if pair_t - self.last_pub_time < self.interval:
                return

        self.last_pub_time = pair_t

        left_out = copy.deepcopy(left_msg)
        right_out = copy.deepcopy(right_msg)

        if self.force_same_stamp:
            same_stamp = rospy.Time.from_sec(pair_t)
            left_out.header.stamp = same_stamp
            right_out.header.stamp = same_stamp

        self.left_pub.publish(left_out)
        self.right_pub.publish(right_out)

        self.count += 1
        if self.count % 20 == 0:
            rospy.loginfo(
                "published %d stereo pairs, stamp diff = %.6f s",
                self.count,
                abs(left_t - right_t)
            )

if __name__ == "__main__":
    rospy.init_node("stereo_sync_throttle")
    StereoSyncThrottle()
    rospy.spin()

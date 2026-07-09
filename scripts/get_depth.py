#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

bridge = CvBridge()
last_depth = None

def cb(msg):
    global last_depth
    if msg.encoding == "16UC1":
        last_depth = bridge.imgmsg_to_cv2(msg, "16UC1").astype(np.float32) / 1000.0
    elif msg.encoding == "32FC1":
        last_depth = bridge.imgmsg_to_cv2(msg, "32FC1")
    else:
        rospy.logwarn_throttle(1.0, "encoding: %s", msg.encoding)

def mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and last_depth is not None:
        d = last_depth[y, x]
        print("pixel ({}, {}) depth = {:.3f} m".format(x, y, d))

rospy.init_node("click_depth_viewer")
rospy.Subscriber("/camera/depth/image_rect_raw", Image, cb, queue_size=1)

cv2.namedWindow("depth")
cv2.setMouseCallback("depth", mouse)

rate = rospy.Rate(30)
while not rospy.is_shutdown():
    if last_depth is not None:
        vis = np.nan_to_num(last_depth.copy(), nan=0.0)
        vis = np.clip(vis, 0.0, 5.0) / 5.0
        vis = (vis * 255).astype(np.uint8)
        cv2.imshow("depth", vis)
        cv2.waitKey(1)
    rate.sleep()

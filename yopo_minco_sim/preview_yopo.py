import os
for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS",
             "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ[name] = "1"

import sys
from pathlib import Path

workspace = Path(__file__).resolve().parents[1]
yopo_dir = workspace / "third_party/YOPO/YOPO"
sys.path.insert(0, str(yopo_dir))

import rospy
from nav_msgs.msg import Odometry
from config.config import cfg
from test_yopo_ros import YopoNet

# 初次联调使用较低的规划速度。
cfg["velocity"] = 1.0

class PreviewYopo(YopoNet):
    def control_pub(self, event):
        # 禁用控制发布，也不>
        return

rospy.init_node("yopo_net", anonymous=False)

odom_topic = "/gazebo/iris_0/odometry"
rospy.loginfo("Waiting for Gazebo odometry...")
odom = rospy.wait_for_message(odom_topic, Odometry, timeout=15)
p = odom.pose.pose.position

# 固定的预览目标：初始位置沿世界坐标 +X 方向 6 米。
goal = [p.x + 6.0, p.y, max(p.z, 1.2)]
rospy.loginfo("Preview goal: %s", goal)

settings = {
    "use_tensorrt": False,
    "goal": goal,
    "topk": 1,
    "pitch_angle_deg": 0,
    "odom_topic": odom_topic,
    "depth_topic": "/iris_0/stereo_camera/depth/image_raw",
    "ctrl_topic": "/yopo_debug/position_cmd",
    "plan_from_reference": False,
    "verbose": False,
}

weight = yopo_dir / "saved/YOPO_1/epoch50.pth"
PreviewYopo(settings, str(weight))

#!/bin/bash
export ROS_MASTER_URI=http://192.168.31.28:11311
export ROS_IP=192.168.31.28
source /root/Fast-Drone-XI35/devel/setup.bash
SAVE_DIR="/root/data/mocap_ground_test"
mkdir -p "$SAVE_DIR"

TIME=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="$SAVE_DIR/orin08_ground_${TIME}.bag"

echo "Recording to: $BAG_NAME"
{
  date -Is
  rosnode info /px4ctrl
  rostopic info /kf_fusion/kf_fail
  rostopic info /vins_fusion/vins_fail
  rostopic info /position_cmd
} > "${BAG_NAME%.bag}_runtime.txt" 2>&1

rosparam dump "${BAG_NAME%.bag}_params.yaml"
rosbag record -O "$BAG_NAME" \
  /vrpn_client_node/orin08/pose \
  /vrpn_client_node/orin08/twist \
  /vrpn_client_node/orin08/accel \
  /motion_capture/motion_capture_odom \
  /mavros/state \
  /mavros/imu/data_raw \
  /mavros/imu/data \
  /imu_filter/data \
  /kf_fusion/kf_imu_odom \
  /kf_fusion/kf_fail \
  /kf_fusion/kf_ext \
  /kf_fusion/kf_static_check \
  /kf_fusion/kf_obv_odom \
  /vins_fusion/vins_fail \
  /mavros/rc/in \
  /mavros/extended_state \
  /mavros/battery \
  /mavros/setpoint_raw/attitude \
  /px4ctrl/takeoff_land \
  /debugPx4ctrl \
  /move_base_simple/goal \
  /traj_start_trigger \
  /yopo_minco/status \
  /yopo_minco/stop \
  /yopo_minco/best_trajectory \
  /position_cmd \
  /yopo_minco/debug/position_cmd \
  /camera/depth/image_rect_raw \
  /camera/depth/camera_info \
  /tf \
  /tf_static \
  /rosout

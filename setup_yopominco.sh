#!/bin/bash

export FD_WS=/root/Fast-Drone-XI35/Fast-Drone-XI35
cd "$FD_WS"

source /opt/ros/noetic/setup.bash
source "$FD_WS/devel/setup.bash"

source "$FD_WS/third_party/px4_sitl/Tools/setup_gazebo.bash" \
  "$FD_WS/third_party/px4_sitl" \
  "$FD_WS/third_party/px4_sitl/build/px4_sitl_default"

export ROS_PACKAGE_PATH="$FD_WS/third_party/px4_sitl:$FD_WS/third_party/px4_sitl/Tools/sitl_gazebo:$ROS_PACKAGE_PATH"

export GAZEBO_MODEL_PATH="$FD_WS/yopo_minco_sim/models:$FD_WS/third_party/px4_sitl/Tools/sitl_gazebo/models:$FD_WS/src/simulation/gazebo_models"

export GAZEBO_PLUGIN_PATH="$FD_WS/devel/lib:${GAZEBO_PLUGIN_PATH:-}"

export DISPLAY=:5

echo "Fast-Drone environment ready."

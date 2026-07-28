#!/usr/bin/env bash

# EuRoC 针对 bringup/tmux/flight.sh 的 profile 扩展。
#
# 顶层进程和每个 tmux pane 都按以下顺序加载：
#   1. source profile.env
#   2. source 本文件
#   3. source devel/setup.bash
#   4. call setup_profile_environment
#
# 本文件只定义函数，不要在顶层直接启动程序。这里可以直接读取 profile.env
# 中的变量，但 ROS 等子进程需要继承的变量必须显式 export。

# 环境 hook：顶层进程和每个 pane 都会调用一次。
# 每个 shell 都会独立初始化，因此该函数应保持可重复执行。
setup_profile_environment() {
    # EUROC_CONFIG 由 profile.env 赋值，并需要传给 pane 内启动的命令。
    export EUROC_CONFIG
}

# 以下函数描述 EuRoC 专用的窗口和 pane 命令。
# new_window/add_pane 是 flight.sh 提供的公共 helper。
start_localization() {
    new_window "vins location" \
        "roslaunch vins fast_drone_250.launch config_file:='$EUROC_CONFIG'" \
        manual

    add_pane "vins location" "kf_fusion" \
        "bash '$WORKSPACE/bringup/scripts/start_kf_fusion.sh'" \
        manual
}

start_rosbag() {
    new_window "rosbag" \
        "rosbag play" \
        manual
}

start_services() {
    new_window "services" \
        "roslaunch uav_utils rosout_file_logger.launch" \
        auto
    add_pane "services" "foxglove" \
        "sleep 5" auto \
        "roslaunch foxglove_bridge foxglove_bridge.launch" auto
}

# 布局 hook：只由顶层进程在 tmux 初始化后调用。
# 保持为文件最后一个函数，便于快速查看 EuRoC 的完整布局。
build_user_layout() {
    start_localization
    start_rosbag
    start_services
}

#!/usr/bin/env bash

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
    add_pane "services" "foxglove"\
        "sleep 5" auto \
        "roslaunch foxglove_bridge foxglove_bridge.launch" auto
}

build_user_layout() {
    start_localization
    start_rosbag
    start_services
}

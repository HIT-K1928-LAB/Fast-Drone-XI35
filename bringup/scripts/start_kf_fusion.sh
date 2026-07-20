#!/usr/bin/env bash

cleanup() {
    echo "Stopping launched background processes..."
    jobs -pr | xargs -r kill
    wait
}

trap cleanup INT TERM EXIT

if ! rosnode list | grep -q "/imu_filter_node"; then
    echo "/imu_filter_node is not running; will exec imu_filter.launch..."
    roslaunch imu_filter imu_filter.launch &
    sleep 1
fi
roslaunch kf_fusion kf_fusion.launch &
wait

#!/usr/bin/env bash

cleanup() {
    echo "Stopping launched background processes..."
    jobs -pr | xargs -r kill
    wait
}

trap cleanup INT TERM EXIT

roslaunch realsense2_camera rs_camera.launch &
sleep 1

roslaunch mavros px4.launch &
sleep 1

roslaunch imu_filter imu_filter.launch &
sleep 1

bash "$(dirname "$0")/kf_fusion.sh" &

wait

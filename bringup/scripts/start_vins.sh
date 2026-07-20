#!/usr/bin/env bash

set -u

VINS_CONFIG_FILE="${1:?Missing VINS config file}"

cleanup() {
    echo "Stopping launched background processes..."
    jobs -pr | xargs -r kill
    wait
}

trap cleanup INT TERM EXIT

sudo chmod 777 /dev/ttyTHS0
sleep 0.5;
roslaunch realsense2_camera rs_camera.launch &
sleep 2;
roslaunch mavros px4.launch &
sleep 2;
roslaunch imu_filter imu_filter.launch &
sleep 2;
roslaunch superpoint superpoint_frontend.launch config_file:="$VINS_CONFIG_FILE" &
sleep 2;
roslaunch vins fast_drone_250.launch config_file:="$VINS_CONFIG_FILE"
wait;

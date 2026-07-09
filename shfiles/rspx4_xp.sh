#!/usr/bin/env bash

cleanup() {
    echo "Stopping launched background processes..."
    jobs -pr | xargs -r kill
    wait
}

trap cleanup INT TERM EXIT

sudo chmod 777 /dev/ttyTHS0
sleep 0.5;
roslaunch uav_utils rosout_file_logger.launch &
sleep 2;
roslaunch realsense2_camera rs_camera.launch &
sleep 2;
roslaunch mavros px4.launch &
sleep 2;
roslaunch imu_filter imu_filter.launch &
sleep 2;
find ../ -path '*/template/*' -prune -o -type f -name 'fast_drone_250.yaml' -exec sed -i 's/use_external_front_end: 0/use_external_front_end: 1/1' {} \;
roslaunch superpoint superpoint_frontend.launch &
sleep 2;
roslaunch vins fast_drone_250.launch
wait;

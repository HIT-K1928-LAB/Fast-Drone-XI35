#!/bin/bash
# Documentation: src/realflight_modules/motion_capture/ReadMe.md
# modify the IP address to your motion capture system's broadcast IP !!!
motion_capture_broadcast_ip=192.168.31.15

cleanup() {
    echo "Stopping launched background processes..."
    jobs -pr | xargs -r kill
    wait
}

trap cleanup INT TERM EXIT

roslaunch vrpn_client_ros sample.launch server:=${motion_capture_broadcast_ip} &
sleep 2

roslaunch motion_capture motion_capture.launch &

wait

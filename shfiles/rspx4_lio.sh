sudo chmod 777 /dev/ttyTHS0
sleep 1;
ptp4l -i eth0 -S -m -l 6 & sleep 3;
roslaunch mavros px4.launch & sleep 4;
roslaunch imu_filter imu_filter.launch & sleep 1;
roslaunch livox_ros_driver2 msg_MID360.launch & sleep 3;
roslaunch fast_lio mapping_xi35.launch & sleep 4;
wait;

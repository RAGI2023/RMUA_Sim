#!/bin/bash
cd /basic_dev
source devel/setup.bash
rosrun imu_gps_odometry imu_gps_odometry &
rosrun controller_test controller_test 
# roslaunch yolov5_ros yolov5.launch 
#!/bin/bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI="http://192.168.101.1:11311"
export ROS_HOSTNAME=192.168.101.6
export ZMQ=1

cd /home/aeye/Documents/bag && rosbag record /mode /can_record /tor /sbg/imu_data


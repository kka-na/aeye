#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/inha/catkin_ws/devel/setup.bash

export ROS_MASTER_URI="http://192.168.101.1:11311"
export ROS_HOSTNAME=192.168.101.6
export ZMQ=1

sleep 20

sh /home/aeye/Documents/aeye/sh/can_bridge.sh

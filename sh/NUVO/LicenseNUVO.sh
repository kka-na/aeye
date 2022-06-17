#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/inha/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=""
export ROS_HOSTNAME=

sleep 3
cd /home/inha/catkin_ws
roscore

sleep 5
sh /home/inha/aeye/sh/bsd_pub.sh &
sh /home/inha/aeye/sh/error_check.sh &
sh /home/inha/aeye/sh/estop_check.sh &
sh /home/inha/aeye/sh/mode_check.sh &
sh /home/inha/aeye/sh/tor_check.sh

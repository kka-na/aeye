#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/inha/catkin_ws/devel/setup.bash

export ROS_MASTER_URI="http://192.168.101.1:11311"
export ROS_HOSTNAME=192.168.101.1

sleep 3
cd /home/inha/catkin_ws &
roscore &

sleep 20
#sh /home/inha/Desktop/sensors &
#sh /home/inha/Desktop/perception &
#roslaunch /home/inha/catkin_ws/src/niro/launch/sensor.launch &
#roslaunch /home/inha/catkin_ws/src/niro/launch/perception.launch &
sh /home/inha/aeye/sh/NUVO/bsd_pub.sh &
sh /home/inha/aeye/sh/NUVO/error_check.sh &
sh /home/inha/aeye/sh/NUVO/estop_check.sh &
sh /home/inha/aeye/sh/NUVO/mode_check.sh &
sh /home/inha/aeye/sh/NUVO/tor_check.sh

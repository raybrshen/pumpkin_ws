#!/bin/bash
MASTER=$HOSTNAME
source devel/setup.bash
export ROS_MASTER_URI=http://$MASTER:11311
roslaunch pumpkin pumpkin.launch

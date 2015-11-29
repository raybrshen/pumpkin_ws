#!/bin/bash
MASTER=pumpkinpi
source devel/setup.bash
export ROS_MASTER_URI=http://$MASTER:11311
rosrun pumpkin_qt pumpkin_qt

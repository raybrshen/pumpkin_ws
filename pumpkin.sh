#!/bin/bash
if [[ $1 == '--local' ]]; then
	MASTER=localhost
else
	MASTER=pumpkinpi
fi
cd `dirname $0`
source devel/setup.bash
export ROS_MASTER_URI=http://$MASTER:11311
roslaunch pumpkin pumpkin.launch

#!/bin/bash

# install dependencies
sudo apt-get install libqt4-dev
sudo apt-get install ros-indigo-actionlib
sudo apt-get install ros-indigo-moveit-ros
sudo apt-get install ros-indigo-serial
sudo apt-get install ros-indigo-rosserial
sudo apt-get install ros-indigo-rosserial-arduino
sudo apt-get install ros-indigo-rosserial-server
sudo apt-get install ros-indigo-qt-ros

# compile
cd `dirname $0`
catkin_make
# catkin make --pkg pumpkin messages pumpkin pumpkin interface
# catkin make --pkg pumpkin messages pumpkin qt

echo '=> done!'

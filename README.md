Dr Robot Hawk Robot (A.K.A Pumpkin) catkin workspace
====================================================

This repository contains catkin source files to Dr Robot Hawk Robot (A.K.A Pumpkin).

There are the following packages:
* analog_array: Which implements arduino analog servo reads messages.
* pumpkin: All applications to record and playback movements with pumpkin.
* pumpkin_moveit: Moveit package for pumpkin.
* pumpkin_description: Description package for pumpkin.

To build run:

    catkin_make
    
You need to run `catkin_make` twice because analog\_array package has self references.

To configure your IDE there are some instruction (here)[http://wiki.ros.org/IDEs].

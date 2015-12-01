Dr Robot Hawk Robot (A.K.A Pumpkin) catkin workspace
====================================================

This repository contains catkin source files to Dr Robot Hawk Robot (A.K.A Pumpkin).

There are the following packages:
* pumpkin: All applications to record and playback movements with pumpkin.
* pumpkin_moveit: Moveit package for pumpkin.
* pumpkin_description: Description package for pumpkin.
* pumpkin_messages: Contains all messages and actions used in our solution.
* pumpkin_interface: Main interface code for Arduino, SSC-32 and some remote file operations.
* pumpkin_qt: GUI for pumpkin.
* ~~analog_array: Which implements arduino analog servo reads messages. (moved to pumpkin_messages)~~

To build:

    $ catkin_make
    
Maybe you will need to run `catkin_make` twice because analog_array and pumpking_messages packages may have self references.

Basically you can run the main software on the robot using:

    $ roslaunch pumpkin pumpkin.launch

And the GUI node:

    $ rosrun pumpkin_qt pumpkin_qt
    
Some configuration files and initialization scripts can be found in the root directory:

* gui.sh: GUI script that configures ROS_MASTER and run pumpkin_qt.
* pumpkin.sh: Configuration and launch of pumpkin.launch
* init_pumpkin: Script that must be copied to /etc/init.rc in order to have a standalone computer booting the pumpkin master code.

To configure your IDE there are some instructions [here](http://wiki.ros.org/IDEs). After some tests we now use [Clion](https://www.jetbrains.com/clion/).

Documentation and manual can be found under `doc/` and `manual/` folders, respectively.

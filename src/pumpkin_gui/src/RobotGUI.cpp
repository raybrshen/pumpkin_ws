//
// Created by rafaelpaiva on 18/09/15.
//

#include "RobotGUI.h"

#include <XmlRpcValue.h>
#include <ros/ros.h>

RobotGUI::RobotGUI() : _group_box(Gtk::ORIENTATION_HORIZONTAL), _control_frame("Send Move Commands")
{
	//First, lets mount the window
	set_title("Pumpkin User Interface");
	set_border_width(10);

	_control_frame.add(_parts);
	_group_box.pack_start(_control_frame, Gtk::PACK_EXPAND_WIDGET, 5);
	add(_group_box);

	//Get robot params via ROS to mount the interface
	XmlRpc::XmlRpcValue config;
	ros::param::get("/pumpkin/config/ssc", config);

	//Create objects of the window, based on the configuration
	for (auto it = config.begin(); it != config.end(); ++it) {

	}
}

RobotGUI::~RobotGUI() { }


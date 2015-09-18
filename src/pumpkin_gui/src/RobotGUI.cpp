//
// Created by rafaelpaiva on 18/09/15.
//

#include "RobotGUI.h"

#include <XmlRpcValue.h>
#include <ros/ros.h>

RobotGUI::RobotGUI() : _group_box(Gtk::ORIENTATION_HORIZONTAL), _control_frame("Send Move Commands"),
                       _control_box(Gtk::ORIENTATION_VERTICAL), _control_send_button("Send Command")
{
	//First, lets mount the window
	set_title("Pumpkin User Interface");
	set_border_width(10);

	//Get robot params via ROS to mount the interface
	XmlRpc::XmlRpcValue config;
	ros::param::get("/pumpkin/config/ssc", config);

	//Create objects of the window, based on the configuration
	for (auto it_parts = config.begin(); it_parts != config.end(); ++it_parts) {
		for (auto it_pieces = it_parts->second.begin(); it_pieces != it_parts->second.end(); ++it_pieces) {
			//Mount a Move Block
		}
	}

	//Put the notebook into control box
	_control_box.pack_start(_parts_notebook);
	//Configure the "send command" button and add it to the control box
	_control_send_button.signal_clicked().connect(sigc::mem_fun(*this, &RobotGUI::send_command));
	//Put the control box into frame
	_control_frame.add(_parts_notebook);
	//Put frame into outer box
	_group_box.pack_start(_control_frame, Gtk::PACK_EXPAND_WIDGET, 5);
	//and... put box into window
	add(_group_box);
}

RobotGUI::~RobotGUI() { }

void RobotGUI::send_command() {
	//TODO send command to the SSC node
}

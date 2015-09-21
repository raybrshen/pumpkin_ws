//
// Created by rafaelpaiva on 18/09/15.
//

#include "RobotGUI.h"

#include <XmlRpcValue.h>
#include <ros/ros.h>

#include <boost/algorithm/string/replace.hpp>

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
		Gtk::Box && part_box = Gtk::Box(Gtk::ORIENTATION_VERTICAL, 5);
		for (auto it_pieces = it_parts->second.begin(); it_pieces != it_parts->second.end(); ++it_pieces) {
			//Mount a Move Block
			_blocks.push_back(MoveBlock(it_pieces->first, int(it_pieces->second["pulse_min"]),
			                            int(it_pieces->second["pulse_max"]), int(it_pieces->second["pulse_rest"])));
			//Mount command
			pumpkin_interface::SSCMove &&movement = pumpkin_interface::SSCMove();
			movement.channel = int(it_pieces->second["pin"]);
			movement.pulse = int(it_pieces->second["pulse_rest"]);
			movement.speed = 0;
			_command.request.list.push_back(movement);

			part_box.pack_start(_blocks.back());
		}
		Gtk::ScrolledWindow && part_scroll = Gtk::ScrolledWindow();
		part_scroll.set_policy(Gtk::POLICY_NEVER, Gtk::POLICY_ALWAYS);
		part_scroll.add(part_box);
		_parts_notebook.append_page(part_scroll, "_"+boost::replace_all_copy(it_parts->first, "_", " "), true);
		_parts_scrolls.push_back(part_scroll);
		_parts_boxes.push_back(part_box);
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

	show_all_children(true);
}

RobotGUI::~RobotGUI() { }

void RobotGUI::send_command() {
	//TODO send command to the SSC node
	for (int i = 0; i < _blocks.size(); i++) {
		unsigned long values = _blocks[i].get_values();
		_command.request.list[i].pulse = pumpkin_interface::SSCMove::_pulse_type(values & 0xFFFFFFFF);
		_command.request.list[i].speed = pumpkin_interface::SSCMove::_speed_type((values >> sizeof(int)) & 0xFFFFFFFF);
	}
	ros::service::call("move_ssc", _command);
	//TODO configure reception command function
}

//
// Created by rafaelpaiva on 18/09/15.
//

//Class instantiation
#include "RobotGUI.h"

//Boost includes
#include <boost/algorithm/string/replace.hpp>

RobotGUI::RobotGUI() : _group_box(Gtk::ORIENTATION_HORIZONTAL), _control_frame("Send Move Commands"),
                       _control_box(Gtk::ORIENTATION_VERTICAL), _control_send_button("Send Command"),
                       _time_box(Gtk::ORIENTATION_HORIZONTAL), _time_label("Time: "),
                       _time_spin(Gtk::Adjustment::create(0.0, 0.0, 65536.0, 1.0, 100.0))
{
	//First, lets mount the window
	set_title("Pumpkin User Interface");
	set_border_width(10);

	//Get robot params via ROS to mount the interface
	XmlRpc::XmlRpcValue config;
	ros::param::get("/pumpkin/config/ssc", config);

	//Create objects of the window, based on the configuration
	for (auto it_parts = config.begin(); it_parts != config.end(); ++it_parts) {
		_parts_boxes.push_back(new Gtk::Box(Gtk::ORIENTATION_VERTICAL, 5));
		_parts_scrolls.push_back(new Gtk::ScrolledWindow);
		Gtk::Box &part_box = _parts_boxes.back();
		Gtk::ScrolledWindow &part_scroll = _parts_scrolls.back();
		for (auto it_pieces = it_parts->second.begin(); it_pieces != it_parts->second.end(); ++it_pieces) {
			//Mount a Move Block
			if (int(it_pieces->second["pulse_min"]) == int(it_pieces->second["pulse_max"]))
				continue;
			//TODO change last parameter later (it's set to only activate, by default, the right hand commands
			_blocks.push_back(new MoveBlock(it_pieces->first, int(it_pieces->second["pin"]), int(it_pieces->second["pulse_min"]),
			                            int(it_pieces->second["pulse_max"]), int(it_pieces->second["pulse_rest"]), it_parts->first[0] == 'r'));

			part_box.pack_start(_blocks.back(), Gtk::PACK_SHRINK, 5);
		}
		part_scroll.set_policy(Gtk::POLICY_NEVER, Gtk::POLICY_ALWAYS);
		part_scroll.add(part_box);
		_parts_notebook.append_page(part_scroll, "_"+boost::replace_all_copy(it_parts->first, "_", " "), true);
	}

	//Set the time box
	_time_spin.set_tooltip_text("Sets the total time to execute the movement. Sets time to 0 to not send time parameter.");
	_time_spin.set_digits(0);
	_time_box.set_homogeneous(false);
	_time_box.pack_start(_time_label, Gtk::PACK_SHRINK, 5);
	_time_box.pack_start(_time_spin, Gtk::PACK_EXPAND_WIDGET, 5);

	//Put the notebook into control box
	_control_box.pack_start(_parts_notebook, Gtk::PACK_EXPAND_WIDGET);
	_control_box.pack_start(_time_box, Gtk::PACK_SHRINK, 5);
	_control_box.pack_start(_control_send_button, Gtk::PACK_SHRINK, 10);

	//Configure the "send command" button and add it to the control box
	_control_send_button.signal_clicked().connect(sigc::mem_fun(*this, &RobotGUI::send_command));
	//Put the control box into frame
	_control_frame.add(_control_box);
	//Put frame into outer box
	_group_box.pack_start(_control_frame, Gtk::PACK_EXPAND_WIDGET, 5);
	//and... put box into window
	add(_group_box);

	set_size_request(_control_frame.get_width()*2, _parts_boxes[0].get_height()+50);

	show_all_children(true);
}

RobotGUI::~RobotGUI() { }

void RobotGUI::send_command() {
	pumpkin_messages::SSCMoveCommand command;
	pumpkin_messages::SSCMoveList move;
	//TODO send command to the SSC node
	for (auto it = _blocks.begin(); it != _blocks.end(); ++it) {
		if (!it->is_active())
			continue;
		pumpkin_messages::SSCMove &&movement = pumpkin_messages::SSCMove();
		movement.channel = pumpkin_messages::SSCMove::_channel_type(it->get_pin());
		movement.pulse = pumpkin_messages::SSCMove::_pulse_type(it->get_pulse_value());
		movement.speed = pumpkin_messages::SSCMove::_speed_type(it->get_speed_value());
		ROS_INFO("Pulse: %d, Speed: %d", movement.pulse, movement.speed);
		move.list.emplace_back(movement);
	}
	move.time = pumpkin_messages::SSCMoveList::_time_type(_time_spin.get_value());
	command.request.move = move;
	_service.call(command);
	//TODO configure reception command function
}

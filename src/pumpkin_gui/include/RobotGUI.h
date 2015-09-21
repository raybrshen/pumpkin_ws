//
// Created by rafaelpaiva on 18/09/15.
//

#ifndef PROJECT_ROBOTGUI_H
#define PROJECT_ROBOTGUI_H

#include <gtkmm/window.h>
#include <gtkmm/button.h>
#include <gtkmm/box.h>
#include <gtkmm/scrolledwindow.h>
#include <gtkmm/notebook.h>
#include <gtkmm/frame.h>

#include "pumpkin_interface/SSCMoveCommand.h"
#include "pumpkin_interface/SSCMove.h"

#include "MoveBlock.h"

class RobotGUI : public Gtk::Window {
public:
	RobotGUI();
	virtual ~RobotGUI();

protected:
	//Widgets
	std::vector<MoveBlock> _blocks;
	Gtk::Box _group_box;
	std::vector<Gtk::ScrolledWindow> _parts_scrolls;
	std::vector<Gtk::Box> _parts_boxes;
	Gtk::Notebook _parts_notebook;
	Gtk::Frame _control_frame;
	Gtk::Box _control_box;
	Gtk::Button _control_send_button;

	pumpkin_interface::SSCMoveCommand _command;

	void send_command();

};


#endif //PROJECT_ROBOTGUI_H

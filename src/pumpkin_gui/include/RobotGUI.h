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

#include "MoveBlock.h"

class RobotGUI : public Gtk::Window {
public:
	RobotGUI();
	virtual ~RobotGUI();

protected:
	//Widgets
	std::vector<MoveBlock> _block;
	Gtk::Box _group_box;
	std::vector<Gtk::ScrolledWindow> _parts_scroll;
	std::vector<Gtk::Box> _parts_box;
	Gtk::Notebook _parts;
	Gtk::Frame _control_frame;

};


#endif //PROJECT_ROBOTGUI_H

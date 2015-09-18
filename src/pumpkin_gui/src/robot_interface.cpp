//
// Created by rafaelpaiva on 18/09/15.
//

#include <ros/ros.h>

#include <gtkmm/application.h>
#include "RobotGUI.h"

int main (int argc, char *argv[]) {

	ros::init(argc, argv, "robot_interface");
	auto app = Gtk::Application::create(argc, argv, "robot_interface.pumpkin.ros");

	RobotGUI gui;

	int status = app->run(gui);

	return status;
}

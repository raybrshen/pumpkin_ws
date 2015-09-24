//
// Created by rafaelpaiva on 18/09/15.
//

//ROS inlcudes
#include <ros/ros.h>

//GTKmm includes
#include <gtkmm/application.h>

//Class instatiation
#include "RobotGUI.h"

int main (int argc, char *argv[]) {

	ros::init(argc, argv, "robot_interface");

	if (!ros::param::has("/pumpkin/config/ssc")){
		ROS_ERROR("Ros SSC configure was not found! Run \"load_config\" module before.");
		return -1;
	}

	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<pumpkin_messages::SSCMoveCommand>("move_ssc");

	if (!client.isValid()) {
		ROS_ERROR("Service is unavailable now. Check the \"setup_ssc\" node.");
		return -2;
	}

	auto app = Gtk::Application::create(argc, argv, "robot_interface.pumpkin.ros");

	RobotGUI gui;

	gui.set_service(client);

	int status = app->run(gui);

	return status;
}

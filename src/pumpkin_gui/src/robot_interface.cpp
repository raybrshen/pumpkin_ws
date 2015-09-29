//
// Created by rafaelpaiva on 18/09/15.
//

//ROS inlcudes
#include <ros/ros.h>

//GTKmm includes
#include <gtkmm/application.h>
#include <pumpkin_messages/Files.h>

//Class instatiation
#include "RobotGUI.h"
#include "LoadConfig.h"

int main (int argc, char *argv[]) {

	ros::init(argc, argv, "robot_interface");
	ros::NodeHandle nh;

	auto app = Gtk::Application::create(argc, argv, "robot_interface.pumpkin.ros");

	int status;

	if (!ros::param::has("/pumpkin/config/ssc")){
		ros::ServiceClient config_client = nh.serviceClient<pumpkin_messages::Files>("file_lister");
		pumpkin_messages::Files msg;
		msg.request.type = (uint8_t)1;
		if (!config_client.call(msg))
			return -1;\

		std::string path(msg.response.files[0].folder), filename;

		LoadConfig load;
		load.link_file_string(&filename);
		load.fill_combobox(msg.response.files[0].filenames);
		status = app->run();

		if (status != 0)
			return status;

		if (!filename.empty())
			ros::param::set("~config_file", path+filename);

		ros::Rate r(1000);

		while (!ros::param::has("/pumpkin/config"))
			r.sleep();
	}

	ros::ServiceClient client = nh.serviceClient<pumpkin_messages::SSCMoveCommand>("move_ssc");

	if (!client.isValid()) {
		ROS_ERROR("Service is unavailable now. Check the \"setup_ssc\" node.");
		return -2;
	}

	RobotGUI gui;

	gui.set_service(client);

	status = app->run(gui);

	return status;
}

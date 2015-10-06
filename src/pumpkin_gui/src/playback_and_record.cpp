//
// Created by rafaelpaiva on 05/10/15.
//

#include <ros/ros.h>
#include <gtkmm/application.h>
#include <gtkmm/builder.h>
#include <glibmm.h>
#include "PlaybackRecordWindow.h"

using namespace pumpkin_gui;

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "playback_and_record");
	ros::NodeHandle nh;
	std::string pumpkin_folder;
	char * env = getenv("PUMPKIN_PATH");

	if (env == nullptr && !ros::param::has("~pumpkin_path"))
		ROS_FATAL("Configure PUMPKIN_PATH first!");

	ros::param::param<std::string>("~pumpkin_path", pumpkin_folder, std::string(env));

	pumpkin_folder += "_gui/glade/main.glade";

	auto app = Gtk::Application::create(argc, argv, "playback_record.pumpkin.ros");
	PlaybackRecordWindow *window = 0;

	auto builder = Gtk::Builder::create();
	try {
		builder->add_from_file(pumpkin_folder);
	} catch (const Glib::FileError &ex) {
		ROS_FATAL("File error: %s", ex.what().c_str());
	} catch (const Glib::MarkupError &ex) {
		ROS_FATAL("Markup error: %s", ex.what().c_str());
	} catch (const Gtk::BuilderError &ex) {
		ROS_FATAL("Builder error: %s", ex.what().c_str());
	}
	ROS_INFO("Glade file: %s", pumpkin_folder.c_str());
	builder->get_widget_derived("playback_record_window", window);

	ROS_INFO("Window builded");

	window->setFileService(nh);

	if (window)
		int status = app->run(*window);
	else
		ROS_FATAL("Error loading window");

	delete window;

	return 0;
}

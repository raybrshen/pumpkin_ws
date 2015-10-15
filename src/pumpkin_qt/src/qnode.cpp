/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "../include/pumpkin_qt/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace pumpkin_qt {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv, QObject *parent) :
	init_argc(argc),
	init_argv(argv),
	QThread(parent)
	{}

QNode::~QNode() {
	//delete _playback_client;
	//delete _record_client;
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"pumpkin_qt");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    _file_client = n.serviceClient<pumpkin_messages::Files>("file_lister");
	ROS_INFO("Waiting for \"file_lister\" ros server.");
	_file_client.waitForExistence();
	ROS_INFO("Server \"file_lister\" found.");
	start();
	return true;
}

void QNode::run() {
    double rate = 100;
    if (ros::param::has("/pumpkin/config/ros_rate")) {
        ros::param::get("/pumpkin/config/ros_rate", rate);
    }
    ros::Rate loop_rate(rate);
	while ( ros::ok() ) {
		ros::spinOnce();
        loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::callFiles() {
    pumpkin_messages::Files msg;
    msg.request.type = static_cast<uint8_t>(pumpkin_messages::FileType::PlaybackFile);
    if (!_file_client.call(msg)) {
		ROS_FATAL("Could not load the files directory");
		ros::shutdown();
		return;				//for security reasons
    }
	ROS_INFO("Paths loaded");
	Q_EMIT filesReady(QString::fromStdString(msg.response.base_path), std::move(msg.response.files));
}

void QNode::callConfigFiles()
{
	pumpkin_messages::Files msg;
	msg.request.type = static_cast<uint8_t>(pumpkin_messages::FileType::ConfigFile);
	if (!_file_client.call(msg)) {
		ROS_FATAL("Could not load the files directory");
		ros::shutdown();
		return;				//for security reasons
	}
	ROS_INFO("Config file list loaded.");
	Q_EMIT(configFilesReady(QString::fromStdString(msg.response.base_path + "/" + msg.response.files[0].folder), msg.response.files[0].filenames));
}

}  // namespace pumpkin_qt

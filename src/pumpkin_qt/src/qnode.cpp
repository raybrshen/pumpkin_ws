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
#include <ros/network.h>
#include <std_msgs/String.h>
#include "../include/pumpkin_qt/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace pumpkin_qt {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
    init_argv(argv),
    _playback_client("playback_action", false),
    _record_client("record_action", false)
	{}

QNode::~QNode() {
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
        Q_EMIT rosShutdown();
    }
    Q_EMIT filesReady(QString(msg.response.base_path.c_str()), std::move(msg.response.files));
}

}  // namespace pumpkin_qt

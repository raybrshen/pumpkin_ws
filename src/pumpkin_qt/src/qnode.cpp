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
#include <actionlib/client/simple_action_client.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace pumpkin_qt {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv, QString *ref) :
	init_argc(argc),
    init_argv(argv),
    _filename_ref(ref)
	{}

QNode::~QNode() {
    delete _playback_client;
    delete _record_client;
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
    _record_client = new actionlib::SimpleActionClient<pumpkin_messages::RecordAction>(n, "record_action", false);
    _playback_client = new actionlib::SimpleActionClient<pumpkin_messages::PlaybackAction>(n, "playback_action", false);
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
    Q_EMIT filesReady(QString::fromStdString(msg.response.base_path), std::move(msg.response.files));
}

void QNode::playbackFile() {
    pumpkin_messages::PlaybackGoal goal;
    goal.filename = _filename_ref->toStdString();
    ROS_INFO("Starting playback file %s", goal.filename.c_str());
    _playback_client->sendGoal(goal, boost::bind(&QNode::playbackDoneCallback, this, _1, _2),
                               boost::bind(&QNode::playbackActiveCallback, this),
                               boost::bind(&QNode::playbackFeedbackCallback, this, _1));
}

void QNode::recordFile() {

}

void QNode::playbackStop() {

}

void QNode::recordStop() {

}

void QNode::playbackActiveCallback() {
    Q_EMIT(lockTab(true));
}

void QNode::playbackDoneCallback(const actionlib::SimpleClientGoalState &goal,
                                 const pumpkin_messages::PlaybackResultConstPtr &result) {
    switch (static_cast<pumpkin_messages::IOState>(result->state)) {
        case pumpkin_messages::IOState::BadFile:
            ROS_ERROR("BAD FILE");
        break;
        case pumpkin_messages::IOState::EndOfFile:
            ROS_ERROR("END OF FILE");
        break;
        case pumpkin_messages::IOState::ErrorOpening:
            ROS_ERROR("ERROR OPENING FILE");
        break;
        case pumpkin_messages::IOState::OK:
            ROS_INFO("Playback executed with success.");
        break;
        default:
            ROS_FATAL("UNKNOWN ERROR");
    }
    Q_EMIT(lockTab(false));
}

void QNode::playbackFeedbackCallback(const pumpkin_messages::PlaybackFeedbackConstPtr &feedback) {
    double perc = feedback->percentage;
    perc*100;
    ROS_INFO("Percentage: %f", perc);
    Q_EMIT(playbackPercentage(int(round(perc))));
}

}  // namespace pumpkin_qt

/*
 * yaml_test.cpp
 *
 *  Created on: 2015-03-01
 *      Author: thiago
 */

#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <pumpkin_messages/analog_array.h>
#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>
#include <pumpkin_messages/RecordAction.h>
#include "file_type.h"

using namespace pumpkin_messages;

class RecordActionServer {
	//private members
	ros::NodeHandle _nh;
	ros::Subscriber _sub;
	actionlib::SimpleActionServer<pumpkin_messages::RecordAction> _server;
	YAML::Node _node;
	std::ofstream _file;
	unsigned int _count;
	ros::Time _start;
	double _max_time;
	pumpkin_messages::RecordFeedback _feedback;
	pumpkin_messages::RecordResult _result;

public:
	RecordActionServer(): _server(_nh, "recorder_action", false) {
		_server.registerGoalCallback(boost::bind(&RecordActionServer::onGoal, this));
		_server.registerPreemptCallback(boost::bind(&RecordActionServer::onPreempt, this));

		_sub = _nh.subscribe("/a_reads", 1000, &RecordActionServer::onRead, this);
		_server.start();
	}

	~RecordActionServer() {
		if (_file.is_open())
			_file.close();
	}

	void onGoal () {
		_count = 0;
		auto goal = _server.acceptNewGoal();
		std::string filename = goal->filename;
		_max_time = goal->max_time;
		if (filename.empty()) {
			std::ostringstream s;
			s << "raw_" << std::time(nullptr) << ".yaml";
			filename = s.str();
		}
		_file.open(filename.c_str(), std::fstream::out | std::fstream::trunc);
		_start = ros::Time::now();
		if (!_file.is_open()) {
			_result.time_elapsed = 0.0;
			_result.num_steps = 0;
			_result.state = static_cast<uint8_t>(IOState::ErrorOpening);
			_server.setAborted(_result);
		}
	}

	void onPreempt() {
		_file << "---\n";
		_file.close();
		_server.setPreempted();
	}

	void onRead(const pumpkin_messages::analog_arrayConstPtr& msg) {
		if (!_server.isActive())
			return;

		_node["header"]["seq"] = msg->header.seq;
		_node["header"]["stamp"]["secs"] = msg->header.stamp.sec;
		_node["header"]["stamp"]["nsecs"] = msg->header.stamp.nsec; // node["seq"] automatically becomes a sequence
		_node["header"]["frame_id"] = msg->header.frame_id;
		for (int i = 0; i < msg->an_read.size(); i++)
			_node["an_read"][i] = msg->an_read[i];

		_file << _node << std::endl;

		_feedback.num_steps = ++_count;
		_feedback.time_elapsed = (ros::Time::now() - _start).toSec();
		_server.publishFeedback(_feedback);

		//Error file
		if (!_file.good()) {
			_result.num_steps = _feedback.num_steps;
			_result.time_elapsed = _feedback.time_elapsed;
			_result.state = static_cast<uint8_t>(_file.eof() ? IOState::EndOfFile : IOState::BadFile);
			_server.setAborted(_result);
			_file.close();
			return;
		}

		//Movement finished
		if (_feedback.time_elapsed >= _max_time) {
			_result.time_elapsed = _feedback.time_elapsed;
			_result.num_steps = _feedback.num_steps;
			_result.state = static_cast<uint8_t>(IOState::OK);
			_server.setSucceeded(_result);
			_file.close();
			return;
		}

		_file << "---\n";
	}
};

/*YAML::Node node;
std::fstream fs;

void analogReadCallback(const pumpkin_messages::analog_arrayConstPtr& msg) {
	fs << "---";
	fs << std::endl;

	node["header"]["seq"] = msg->header.seq;  // it now is a map node
	node["header"]["stamp"]["secs"] = msg->header.stamp.sec;
	node["header"]["stamp"]["nsecs"] = msg->header.stamp.nsec; // node["seq"] automatically becomes a sequence
	node["header"]["frame_id"] = msg->header.frame_id;
	for (int i = 0; i < msg->an_read.size(); i++)
		node["an_read"][i] = msg->an_read[i];


	fs << node;
	fs << std::endl;
}*/

int main(int argc, char** argv) {

	ros::init(argc, argv, "recorder");
	ros::Time::init();

	RecordActionServer server();

	ros::spin();

	return 0;
}


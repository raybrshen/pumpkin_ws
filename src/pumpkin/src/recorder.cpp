/*
 * yaml_test.cpp
 *
 *  Created on: 2015-03-01
 *      Author: thiago
 */

#include <iostream>
#include <fstream>
#include <ctime>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <pumpkin_messages/analog_array.h>
#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>
#include <pumpkin_messages/RecordAction.h>

class RecordActionServer {
	//private attributes
	ros::NodeHandle _nh;
	actionlib::SimpleActionServer<pumpkin_messages::RecordAction> _server;
	YAML::Node _node;
	std::ofstream _file;
	unsigned long _count;
	ros::Time _start, _end;
	pumpkin_messages::RecordFeedback _feedback;
	pumpkin_messages::RecordResult _result;

public:
	RecordActionServer(): _server(_nh, "recorder_action", false) {

	}

	~RecordActionServer() {
		if (_file.is_open())
			_file.close();
	}

	void onGoal () {
		_count = 0;
		auto goal = _server.acceptNewGoal();
		std::string filename = goal->filename;
		if (filename.empty()) {
			std::ostringstream s;
			s << "raw_" << std::time(nullptr) << ".yaml";
			filename = s.str();
		}
		_file.open(filename.c_str(), std::fstream::out | std::fstream::trunc);
	}

	void onPreempt() {
		_file.close();
		_server.setPreempted();
	}

	void onRead(const pumpkin_messages::analog_arrayConstPtr& msg) {
		if (!_server.isActive())
			return;

		_file << "---\n";

		_node["header"]["seq"] = msg->header.seq;
		_node["header"]["stamp"]["secs"] = msg->header.stamp.sec;
		_node["header"]["stamp"]["nsecs"] = msg->header.stamp.nsec; // node["seq"] automatically becomes a sequence
		_node["header"]["frame_id"] = msg->header.frame_id;
		for (int i = 0; i < msg->an_read.size(); i++)
			_node["an_read"][i] = msg->an_read[i];

		_file << _node << std::endl;

		_feedback.num_steps = ++_count;
		_server.publishFeedback(_feedback);


	}
};

YAML::Node node;
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
}

int main(int argc, char** argv) {
	//ros::Rate loop_rate(30);

	std::time_t timestamp = std::time(nullptr);

	ros::init(argc, argv, "recorder");
	ros::Time::init();

	ros::NodeHandle service_handle;
	ros::CallbackQueue service_queue;
	service_handle.setCallbackQueue(&service_queue);

	std::string filename; //, path;
	std::ostringstream s;
	if (argc >= 2) {
		s << argv[1] << ".yaml";
		filename = s.str();
	} else {
		s << "raw_" << timestamp << ".yaml"; //rosrun - no parameters
		filename = s.str();
	}

	fs.open(filename.c_str(), std::fstream::out | std::fstream::trunc);




	ros::Subscriber sub = service_handle.subscribe("a_reads", 1000, analogReadCallback);

	ros::spin();

	fs.close();
	return 0;
}


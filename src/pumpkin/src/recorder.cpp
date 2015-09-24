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

//TODO: create a class to handle the file and YAML node, bind callback to class method

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
	std::string filename; //, path;
	std::ostringstream s;
	if (argc >= 2) {
		s << argv[1] << "-" << timestamp << ".yaml";
		filename = s.str();
	} else {
		filename = "raw-an_read.yaml"; //rosrun - no parameters
	}

	fs.open(filename.c_str(), std::fstream::out | std::fstream::trunc);

	ros::init(argc, argv, "recorder");
	ros::NodeHandle n;
	ros::Time::init();
	ros::Subscriber sub = n.subscribe("a_reads", 1000, analogReadCallback);

	ros::spin();

	fs.close();
	return 0;
}


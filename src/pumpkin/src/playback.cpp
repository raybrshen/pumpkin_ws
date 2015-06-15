/*
 *  playback.cpp
 *
 *  Created on: 2015-03-03
 *      Author: labrob
 */

#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include "yaml-cpp/yaml.h"
#include <cstdlib>
#include <ros/ros.h>
#include "serial/serial.h"

#define SSC_BAUDRATE 115200

uint16_t map(YAML::Node servo, std::vector<uint16_t> an_read) {

	uint16_t in_min = servo["arduino"]["analog_read_min"].as<uint16_t>();
	uint16_t in_max = servo["arduino"]["analog_read_max"].as<uint16_t>();
	uint16_t out_min = servo["ssc"]["pulse_min"].as<uint16_t>();
	uint16_t out_max = servo["ssc"]["pulse_max"].as<uint16_t>();
	int pin = servo["arduino"]["pin"].as<int>();

	if (pin > an_read.size())
		return 0;
	if ((in_max - in_min) == 0)
		return 0;
	uint16_t pulse = (uint16_t) ((an_read[pin] - in_min) * (out_max - out_min)
			/ (in_max - in_min) + out_min);

	if (pulse > out_max)
		return out_max;
	if (pulse < out_min)
		return out_min;
	else
		return pulse;
}


int main(int argc, char *argv[]) {
	std::string ssc_port, input_file, input_config_calib;

	if (argc >= 2) {
		ssc_port = argv[1];
		input_config_calib = argv[2];
		input_file = argv[3];
	} else {
	    printf("Usage: rosrun pumpkin playback <ssc_port> <input_config_calib> <input_file>\n");
    	ROS_ERROR("Failed to parse input files");
		exit(-1);
		exit(-1);
	}

	ros::init(argc, argv, "playback");
	ros::NodeHandle nh;

	YAML::Node pumpkin_config = YAML::LoadFile(input_config_calib);

	std::vector<YAML::Node> reads = YAML::LoadAllFromFile(input_file);
	std::vector<YAML::Node> servos;

	servos.push_back(pumpkin_config["right_arm"]["shoulder_0"]);
	servos.push_back(pumpkin_config["right_arm"]["shoulder_1"]);
	servos.push_back(pumpkin_config["right_arm"]["shoulder_2"]);
	servos.push_back(pumpkin_config["right_arm"]["elbow"]);
	servos.push_back(pumpkin_config["right_arm"]["wrist_0"]);
	servos.push_back(pumpkin_config["right_arm"]["wrist_1"]);

	servos.push_back(pumpkin_config["left_arm"]["shoulder_0"]);
	servos.push_back(pumpkin_config["left_arm"]["shoulder_1"]);
	servos.push_back(pumpkin_config["left_arm"]["shoulder_2"]);
	servos.push_back(pumpkin_config["left_arm"]["elbow"]);
	servos.push_back(pumpkin_config["left_arm"]["wrist_0"]);
	servos.push_back(pumpkin_config["left_arm"]["wrist_1"]);

	double ros_rate = pumpkin_config["ros_rate"].as<double>();
	ros::Rate loop_rate(ros_rate);

	serial::Serial ssc(ssc_port, SSC_BAUDRATE, serial::Timeout::simpleTimeout(1000));
	for (int r = 0; r < reads.size(); r++) {
		if (!reads[r].IsNull() && ros::ok()){
			if(reads[r]["an_read"].IsNull()) ROS_FATAL("NULL");
			std::vector<uint16_t> an_read =
				reads[r]["an_read"].as<std::vector<uint16_t> >();
			std::ostringstream stringStream;
			for (int i = 0; i < reads[r]["an_read"].size(); i++) {
				uint16_t pulse = map(servos[i], an_read);
				stringStream << "#" << servos[i]["ssc"]["pin"] << " " << "P" << pulse << " " ;
			}
			stringStream << "\r";
			// Send to ssc
			ssc.write(stringStream.str());
			loop_rate.sleep();
		}
	}
	ssc.write("#0 P0 #1 P0 #2 P0 #3 P0 #4 P0 #5 P0 #6 P0 #7 P0 #8 P0 #9 P0 #10 P0 #11 P0 #12 P0 #13 P0 #14 P0 #15 P0\r");
	ssc.write("#16 P0 #17 P0 #18 P0 #19 P0 #20 P0 #21 P0 #22 P0 #23 P0 #24 P0 #25 P0 #26 P0 #27 P0 #28 P0 #29 P0 #30 P0 #31 P0 #31 P0\r");
	ssc.close();
	return 0;
}

/*
 * yaml_read.cpp
 *
 *  Created on: 2015-03-03
 *      Author: labrob
 */

#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>


int main(int argc, char** argv) {

	std::time_t timestamp = std::time(nullptr);
	std::string input_recorded, input_config, output_config_calib; //, path;
	//ROS_FATAL("ARGC: %d", argc);
	if (argc >= 2) {
		input_recorded = argv[1];
		input_config = argv[2];
		output_config_calib = argv[3];
		ROS_FATAL("1");
	} else {
//TODO: write usage
		ROS_FATAL("2");
		exit(-1);
	}
	ROS_FATAL("%s", input_recorded.c_str());

	std::fstream fs;
	fs.open(output_config_calib.c_str(), std::fstream::out | std::fstream::trunc);

	std::vector<YAML::Node> reads = YAML::LoadAllFromFile(input_recorded.c_str());
	YAML::Node pumpkin_config = YAML::LoadFile(input_config.c_str());
	std::vector<YAML::Node> servos;

	ROS_FATAL("4");

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
	
//.as<std::vector<std::uint16_t> >()
	if(reads.size() > 0){
		if(reads[0]["an_read"]){
			for (std::size_t j = 0; j < reads[0]["an_read"].size(); j++) {
				ROS_FATAL("5");
				servos[j]["arduino"]["analog_read_min"] = 3000;
				servos[j]["arduino"]["analog_read_max"] = 0;
				ROS_FATAL("5");
			}
		}else{
			ROS_FATAL("5.5");// reads > 0 an_read < 0
			exit(-1);
		}
	}else{
		ROS_FATAL("6");// reads > 0 
		exit(-1);
	}

	int aread = 0;
	for (std::size_t i = 0; i < reads.size(); i++) {
		for (std::size_t j = 0; j < reads[i]["an_read"].size(); j++) {
			aread = reads[i]["an_read"][servos[j]["arduino"]["pin"].as<
					std::string>()].as<int>();
			if (aread > servos[j]["arduino"]["analog_read_max"].as<int>())
				servos[j]["arduino"]["analog_read_max"] = aread;
			if (aread < servos[j]["arduino"]["analog_read_min"].as<int>())
				servos[j]["arduino"]["analog_read_min"] = aread;
		}
	}
	ROS_FATAL("7");

	int reads_qtd = reads.size();
	YAML::Node initial = reads[0];
	YAML::Node final = reads.back();

	int ini_sec = initial["header"]["stamp"]["secs"].as<int>();
	int fin_sec = final["header"]["stamp"]["secs"].as<int>();

	pumpkin_config["ros_rate"] = (double) reads_qtd / (fin_sec - ini_sec);

	fs << pumpkin_config;
	fs.close();
	ROS_FATAL("8");
	return 0;
}

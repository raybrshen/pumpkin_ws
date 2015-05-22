/*
 *  playback_joint_states.cpp
 *
 *  Created on: 2015-03-03
 *      Author: Vinicius (vncprado@gmail.com)
 */

//#include <iostream>
//#include <sstream>
//#include <vector>
//#include <string>
//#include "yaml-cpp/yaml.h"
//#include <cstdlib>
//#include <ros/ros.h>
////#include "serial/serial.h"
//
////#define TEST_FILE "/home/thiago/catkin_ws/src/pumpkin/calib/raw-an_reads-000001.yaml"
////#define CONF_FILE "/home/thiago/catkin_ws/src/pumpkin/config/pumpkin_config_calib.yaml"
//
////#define SSC_BAUDRATE 115200
////#define SSC_SERIALPORT "/dev/ttyUSB0"
//
//float map2joint(YAML::Node servo, std::vector<uint16_t> an_read) {
//	return 0;
//}
//
//uint16_t map(YAML::Node servo, std::vector<uint16_t> an_read) {
//
//	uint16_t in_min = servo["arduino"]["analog_read_min"].as<uint16_t>();
//	uint16_t in_max = servo["arduino"]["analog_read_max"].as<uint16_t>();
//	uint16_t out_min = servo["ssc"]["pulse_min"].as<uint16_t>();
//	uint16_t out_max = servo["ssc"]["pulse_max"].as<uint16_t>();
//	int pin = servo["arduino"]["pin"].as<int>();
//
//	if (pin > an_read.size())
//		return 0;
//	if ((in_max - in_min) == 0)
//		return 0;
//	uint16_t pulse = (uint16_t) ((an_read[pin] - in_min) * (out_max - out_min)
//			/ (in_max - in_min) + out_min);
//
//	if (pulse > out_max)
//		return out_max;
//	if (pulse < out_min)
//		return out_min;
//	else
//		return pulse;
//}
//
//
//int main(int argc, char *argv[]) {
////	std::string input_file, input_config_calib;
////	//ROS_FATAL("ARGC: %d", argc);
////	if (argc >= 2) {
////		input_file = argv[1];
////		input_config_calib = argv[2];
////		//ROS_FATAL("1");
////	} else {
////		//TODO: write usage
////		//ROS_FATAL("2");
////		exit(-1);
////	}
//
//	ros::init(argc, argv, "playback2joint");
//	ros::NodeHandle nh;
//	ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
//	ros::Rate loop_rate(1);
//
//	sensor_msgs::JointState joint_state;
//	ROS_FATAL("1");
//	float i=0;
//	for (;;) {
////		if (i > -2 && i < 2){
//			ROS_FATAL("%f", i);
//			joint_state.header.stamp=ros::Time::now();
//			joint_state.name.push_back("chest_neck1");
//			joint_state.position.push_back(i);
//			joint_pub.publish(joint_state);
//			ros::spinOnce();
//			loop_rate.sleep();
//			i = i + 0.1;
////		}
////		else
//			i=1.5;
//	}
////	YAML::Node pumpkin_config = YAML::LoadFile(input_config_calib);
////
////	std::vector<YAML::Node> reads = YAML::LoadAllFromFile(input_file);
////	std::vector<YAML::Node> servos;
////
////	servos.push_back(pumpkin_config["right_arm"]["shoulder_0"]);
////	servos.push_back(pumpkin_config["right_arm"]["shoulder_1"]);
////	servos.push_back(pumpkin_config["right_arm"]["shoulder_2"]);
////	servos.push_back(pumpkin_config["right_arm"]["elbow"]);
////	servos.push_back(pumpkin_config["right_arm"]["wrist_0"]);
////	servos.push_back(pumpkin_config["right_arm"]["wrist_1"]);
////
////	servos.push_back(pumpkin_config["left_arm"]["shoulder_0"]);
////	servos.push_back(pumpkin_config["left_arm"]["shoulder_1"]);
////	servos.push_back(pumpkin_config["left_arm"]["shoulder_2"]);
////	servos.push_back(pumpkin_config["left_arm"]["elbow"]);
////	servos.push_back(pumpkin_config["left_arm"]["wrist_0"]);
////	servos.push_back(pumpkin_config["left_arm"]["wrist_1"]);
////
////	double ros_rate = pumpkin_config["ros_rate"].as<double>();
////	ros::Rate loop_rate(ros_rate);
//
//
////	//ROS_FATAL("3");
////
//////	serial::Serial ssc(ssc_port, SSC_BAUDRATE, serial::Timeout::simpleTimeout(1000));
////	for (int r = 0; r < reads.size(); r++) {
////		if (!reads[r].IsNull() && ros::ok()){
////			if(reads[r]["an_read"].IsNull()) ROS_FATAL("NULL");
////			std::vector<uint16_t> an_read =
////				reads[r]["an_read"].as<std::vector<uint16_t> >();
////			std::ostringstream stringStream;
////			for (int i = 0; i < reads[r]["an_read"].size(); i++) {
////				uint16_t pulse = map(servos[i], an_read);
////				stringStream << "#" << servos[i]["ssc"]["pin"] << " " << "P" << pulse << " " ;
////			}
////			//stringStream << " S50 " << "\r" ;
////			stringStream << "\r";
////			// Send to ssc
//////			std::cout << stringStream.str() << std::endl;
////			ssc.write(stringStream.str());
////			loop_rate.sleep();
////		}
////	}
////	ssc.write("#0 P0 #1 P0 #2 P0 #3 P0 #4 P0 #5 P0 #6 P0 #7 P0 #8 P0 #9 P0 #10 P0 #11 P0 #12 P0 #13 P0 #14 P0 #15 P0\r");
////	ssc.write("#16 P0 #17 P0 #18 P0 #19 P0 #20 P0 #21 P0 #22 P0 #23 P0 #24 P0 #25 P0 #26 P0 #27 P0 #28 P0 #29 P0 #30 P0 #31 P0 #31 P0\r");
////	ssc.close();
//	return 0;
//}
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include "yaml-cpp/yaml.h"

double map2joint_states(YAML::Node servo,
		boost::shared_ptr<const urdf::Joint>::element_type* joint,
		std::vector<uint16_t> an_read) {

	ROS_INFO("%s", joint->name.c_str());

	int in_min = servo["arduino"]["analog_read_min"].as<uint16_t>();
	int in_max = servo["arduino"]["analog_read_max"].as<uint16_t>();
	double out_min = joint->limits->lower; // understand and invert urdf (solidworks)
	double out_max = joint->limits->upper;
	int pin = servo["arduino"]["pin"].as<int>();

	ROS_INFO("%d %d %f %f %d", in_min, in_max, out_min, out_max, pin);

	if (pin > an_read.size())
		return 0;
	if ((in_max - in_min) == 0)
		return 0;
	double pos = (double) ((an_read[pin] - in_min) * (out_max - out_min)
			/ (in_max - in_min) + out_min);

	ROS_INFO("%f", pos);

	if (pos > out_max)
		return out_max;
	if (pos < out_min)
		return out_min;
	else
		return pos;
}

//uint16_t map(YAML::Node servo, std::vector<uint16_t> an_read) {
//
//	uint16_t in_min = servo["arduino"]["analog_read_min"].as<uint16_t>();
//	uint16_t in_max = servo["arduino"]["analog_read_max"].as<uint16_t>();
//	uint16_t out_min = servo["ssc"]["pulse_min"].as<uint16_t>();
//	uint16_t out_max = servo["ssc"]["pulse_max"].as<uint16_t>();
//	int pin = servo["arduino"]["pin"].as<int>();
//
//	if (pin > an_read.size())
//		return 0;
//	if ((in_max - in_min) == 0)
//		return 0;
//	uint16_t pulse = (uint16_t) ((an_read[pin] - in_min) * (out_max - out_min)
//			/ (in_max - in_min) + out_min);
//
//	if (pulse > out_max)
//		return out_max;
//	if (pulse < out_min)
//		return out_min;
//	else
//		return pulse;
//}

int main(int argc, char** argv) {

	ros::init(argc, argv, "playback_joint_states");
	ros::NodeHandle nh;
	ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>(
			"playback_joint_states", 1);

	std::string input_file, input_config_calib, input_urdf;
	if (argc >= 4) {
		input_file = argv[1];
		input_config_calib = argv[2];
		input_urdf = argv[3];
	} else {
		exit(-1);
	}

	YAML::Node pumpkin_config = YAML::LoadFile(input_config_calib);
	std::vector<YAML::Node> reads = YAML::LoadAllFromFile(input_file);

	double ros_rate = pumpkin_config["ros_rate"].as<double>();
	ros::Rate loop_rate(ros_rate);

	urdf::Model model;
	if (!model.initFile(input_urdf)) {
		ROS_ERROR("Failed to parse urdf file");
		return -1;
	}
	ROS_INFO("Successfully parsed urdf file");

	std::vector<YAML::Node> servos;

	typedef std::map<std::string, boost::shared_ptr<urdf::Joint> > UrdfJoints;

	UrdfJoints::const_iterator joint_it = model.joints_.begin();
	UrdfJoints::const_iterator joint_end = model.joints_.end();
	std::vector<boost::shared_ptr<const urdf::Joint>::element_type*> joints;
	for (; joint_it != joint_end; ++joint_it) {
		const boost::shared_ptr<const urdf::Joint>& urdf_joint =
				joint_it->second;
		if (urdf_joint->type == 1) { // all servo joints are revolute
			if (urdf_joint->name[0] == 'r') { //ROS_INFO("right arm");
				std::string joint_name = urdf_joint->name;
				servos.push_back(
						pumpkin_config["right_arm"][joint_name.substr(
								joint_name.find_first_of('_') + 1).c_str()]);
				joints.push_back(urdf_joint.get());
			}
			if (urdf_joint->name[0] == 'l') { //ROS_INFO("left arm");
				std::string joint_name = urdf_joint->name;
				servos.push_back(
						pumpkin_config["left_arm"][joint_name.substr(
								joint_name.find_first_of('_') + 1).c_str()]);
				joints.push_back(urdf_joint.get());
			}
		}
	}

	for (int r = 0; r < reads.size(); r++) {
		sensor_msgs::JointState joint_state;
		if (!reads[r].IsNull() && ros::ok()) {
			if (reads[r]["an_read"].IsNull())
				ROS_FATAL("NULL");
			std::vector<uint16_t> an_read = reads[r]["an_read"].as<
					std::vector<uint16_t> >();
			std::ostringstream stringStream;
			for (int i = 0; i < reads[r]["an_read"].size(); i++) {
				double pos = map2joint_states(servos[i], joints[i], an_read);
				joint_state.header.stamp = ros::Time::now();
				joint_state.name.push_back(joints[i]->name.c_str());
				joint_state.position.push_back(pos);

				joint_pub.publish(joint_state);

			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

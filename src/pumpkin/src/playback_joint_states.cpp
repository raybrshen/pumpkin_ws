/*
 *  playback_joint_states.cpp
 *
 *  Created on: 2015-03-03
 *      Author: Vinicius (vncprado@gmail.com)
 */

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

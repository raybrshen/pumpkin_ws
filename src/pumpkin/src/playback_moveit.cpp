/*
 *  playback_joint_states.cpp
 *  This implements playback from moveit model robot to real robot joint values
 *  Created on: 2015-06-15
 *      Author: Vinicius (vncprado@gmail.com)
 */

#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include "yaml-cpp/yaml.h"
#include "serial/serial.h"
#include <cstdlib>
#include <ros/ros.h>
#include <urdf/model.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>

#define SSC_BAUDRATE 115200

double mapJointStates2pumpkin(YAML::Node servo,
		boost::shared_ptr<const urdf::Joint>::element_type* joint,
		std::vector<double> positions, int positions_i) {
    
	double in_min = joint->limits->lower;
	double in_max = joint->limits->upper;
	double out_min = servo["ssc"]["pulse_min"].as<uint16_t>();
	double out_max = servo["ssc"]["pulse_max"].as<uint16_t>();
	int pin = servo["ssc"]["pin"].as<int>();

	if (pin > positions.size())
		return 0;
	if ((in_max - in_min) == 0)
		return 0;
	double pos = (double) ((positions[positions_i] - in_min)
			* (out_max - out_min) / (in_max - in_min) + out_min);

//	ROS_INFO("%s", joint->name.c_str());
//	ROS_INFO("%f %f %f %f %d", in_min, in_max, out_min, out_max, pin);
//	ROS_INFO("%f", pos);
	if (pos > out_max)
		return out_max;
	if (pos < out_min)
		return out_min;
	else
		return pos;
	return 0;
}

std::vector<YAML::Node> servos;
std::vector<boost::shared_ptr<const urdf::Joint>::element_type*> joints;
std::vector<trajectory_msgs::JointTrajectoryPoint> points;
std::vector<std::string> joint_names;
std::string ssc_port;
double ros_rate;

void fakeControllerJointStatesExecutionCallback(const sensor_msgs::JointStateConstPtr& ptr) {
	ROS_INFO("Starting execution of planned path");

//	std::vector<double> positions = ptr.get()->position;
//
//	for (int i=0; i<positions.size(); i++)
//		printf(" --- fake_controller_joint_states event! %f ", positions[i]);
//	printf("\n");

	ros::Rate loop_rate(ros_rate);
	serial::Serial ssc(ssc_port, SSC_BAUDRATE,
			serial::Timeout::simpleTimeout(1000));
	for (int p = 0; p < points.size(); p++) {
		if (!points.empty() && ros::ok()) {
			std::vector<double> positions = points[p].positions;
			std::ostringstream stringStream;
			for (int i = 0; i < joints.size(); i++) {
				// This find gets the correct position value of a joint in positions vector, based on joint_names
				int positions_i = std::find(joint_names.begin(),
						joint_names.end(), joints[i]->name.c_str())
						- joint_names.begin();
                if (positions_i >= positions.size()) {
                    continue;
                }
				uint16_t pulse = mapJointStates2pumpkin(servos[i], joints[i],
						positions, positions_i);
				stringStream << "#" << servos[i]["ssc"]["pin"] << " " << "P"
						<< pulse << " ";
			}
			stringStream << "\r";
			ROS_INFO("%s", stringStream.str().c_str());

			ssc.write(stringStream.str()); // Send to ssc
			loop_rate.sleep();
		}
	}

//	ssc.write(
//			"#0 P0 #1 P0 #2 P0 #3 P0 #4 P0 #5 P0 #6 P0 #7 P0 #8 P0 #9 P0 #10 P0 #11 P0 #12 P0 #13 P0 #14 P0 #15 P0\r");
//	ssc.write(
//			"#16 P0 #17 P0 #18 P0 #19 P0 #20 P0 #21 P0 #22 P0 #23 P0 #24 P0 #25 P0 #26 P0 #27 P0 #28 P0 #29 P0 #30 P0 #31 P0 #31 P0\r");
	ssc.close();
}

void plannedPathCallback(
		const moveit_msgs::DisplayTrajectoryConstPtr& trajectories) {
	ROS_INFO("Successfully get path planning");
	moveit_msgs::RobotState trajectory_start = trajectories->trajectory_start;
	moveit_msgs::RobotTrajectory trajectory = trajectories->trajectory[0];
	points = trajectory.joint_trajectory.points;
	joint_names = trajectory.joint_trajectory.joint_names;


}

int main(int argc, char *argv[]) {
	std::string input_config_calib, input_urdf, input_file;
	if (argc >= 2) {
		ssc_port = argv[1];
		input_config_calib = argv[2];
		input_urdf = argv[3];
	} else {
		printf(
				"Usage: rosrun pumpkin playback_moveit <ssc_port> <input_config_calib> <input_urdf>\n");
		ROS_ERROR("Failed to parse input files");
		exit(-1);
	}

	ros::init(argc, argv, "playback_moveit");
	ros::NodeHandle nh;
	ros::Subscriber planned_path = nh.subscribe(
			"/move_group/display_planned_path", 1000, plannedPathCallback);
	ros::Subscriber fake_controller_joint_states = nh.subscribe("/move_group/fake_controller_joint_states", 1000,
			fakeControllerJointStatesExecutionCallback);
	YAML::Node pumpkin_config = YAML::LoadFile(input_config_calib);
	ros_rate = pumpkin_config["ros_rate"].as<double>()/10;
	ROS_INFO("ros_rate %f", ros_rate);

	urdf::Model model;
	if (!model.initFile(input_urdf)) {
		ROS_ERROR("Failed to parse urdf file");
		exit(-1);
	}
	ROS_INFO("Successfully parsed urdf file");

	typedef std::map<std::string, boost::shared_ptr<urdf::Joint> > UrdfJoints;
	UrdfJoints::const_iterator joint_it = model.joints_.begin();
	UrdfJoints::const_iterator joint_end = model.joints_.end();

	for (; joint_it != joint_end; ++joint_it) {
		const boost::shared_ptr<const urdf::Joint>& urdf_joint =
				joint_it->second;
		if (urdf_joint->type == 1) { // all servo joints are revolute
			if (urdf_joint->name[0] == 'r') {
				std::string joint_name = urdf_joint->name;
				servos.push_back(
						pumpkin_config["right_arm"][joint_name.substr(
								joint_name.find_first_of('_') + 1).c_str()]);
				joints.push_back(urdf_joint.get());
			}
			if (urdf_joint->name[0] == 'l') {
				std::string joint_name = urdf_joint->name;
				servos.push_back(
						pumpkin_config["left_arm"][joint_name.substr(
								joint_name.find_first_of('_') + 1).c_str()]);
				joints.push_back(urdf_joint.get());
			}
		}
	}
	ROS_INFO("Servos and joints");

	ros::spin();

	return 0;
}

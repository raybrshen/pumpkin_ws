/*
 *  playback_joint_states.cpp
 *  Implements playback recorded files from real robot to joint state publisher in the model
 *  Created on: 2015-03-03
 *      Author: Vinicius (vncprado@gmail.com)
 */

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include "yaml-cpp/yaml.h"
#include <XmlRpcValue.h>

#include <analog_read/analog_array.h>

struct aux_struct {
	int pin;
	double value;
	int in_min, in_max; //min, max
	double out_lower, out_upper; //min, max
	std::string joint_name;
};

std::vector<aux_struct> aux_vec;

bool received = false;

void receiveData(const analog_read::analog_arrayConstPtr& reads)
{
	for (std::vector<aux_struct>::iterator it = aux_vec.begin(); it != aux_vec.end(); ++it) {
		if (it->pin > reads->an_read.size()) {
			ROS_INFO("Pin outbound received");
			it->value = 0.0;
		}

		it->value = (double) ((reads->an_read[it->pin] - it->in_min) * (it->out_upper - it->out_lower)
			/ (it->in_max - it->in_min) + it->out_lower);

		if (it->value > it->out_upper)
			it->value = it->out_upper;
		else if (it->value < it-> out_lower)
			it->value = it->out_lower;
	}

	//ROS_INFO("Received signal from Arduino.");
	received = true;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "arduino_to_rviz");
	ros::NodeHandle nh;
	ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("playback_joint_states", 1);
	ROS_INFO("Publisher loaded.");
	ros::Subscriber arduino_data = nh.subscribe<analog_read::analog_array>("a_reads", 1, receiveData);
	ROS_INFO("Publisher and Subscriber loaded successfully.");
	XmlRpc::XmlRpcValue config;
	urdf::Model model;

	std::string model_file;

	if (argc >= 2) {
		model_file = argv[1];
	} else {
		printf("Usage: rosrun pumpkin arduino_to_rviz <input_urdf>\n");
    	ROS_ERROR("Failed to parse input files");
		exit(-1);
	}

	//Load Model
	if (!model.initFile(model_file)) {
		ROS_ERROR("Failed to parse urdf file");
		exit(-1);
	}
	ROS_INFO("Successfully parsed urdf file");
	
	//Load config
	double ros_rate;
	ros::param::get("/pumpkin/config/arduino", config);
	ros::param::get("/pumpkin/config/ros_rate", ros_rate);

	//Setup ROS loop rate
	ros::Rate loop(ros_rate);

	//Iterate through Model joints to fill the auxiliar structure
	typedef std::map<std::string, boost::shared_ptr<urdf::Joint> > UrdfJoints;
	for (UrdfJoints::const_iterator joint_it = model.joints_.begin(); joint_it != model.joints_.end(); ++joint_it) {
		aux_struct aux;
		const boost::shared_ptr<const urdf::Joint> &urdf_joint = joint_it->second;
		XmlRpc::XmlRpcValue *cluster;

		//Joints here must be revolute
		if (urdf_joint->type != 1)
			continue;

		if (urdf_joint->name[0] == 'l') {
			cluster = &config["left_arm"];
			continue;	//TODO remove this after having left arm
		}
		else if (urdf_joint->name[0] == 'r') {
			cluster = &config["right_arm"];
		}
		else
			continue; //For now, we have only arms

		aux.joint_name = urdf_joint->name;
		const std::string &part = urdf_joint->name.substr(2);
		aux.pin = int((*cluster)[part]["pin"]);
		aux.in_min = int((*cluster)[part]["analog_read_min"]);
		aux.in_max = int((*cluster)[part]["analog_read_max"]);
		aux.out_lower = double(urdf_joint.get()->limits->lower);
		aux.out_upper = double(urdf_joint.get()->limits->upper);

		if (aux.in_max == aux.in_min) {
			ROS_INFO("ERROR, Analog pin %d has same value for minimum and maximum read.", aux.pin);
			exit(-1);
		}

		aux_vec.push_back(aux);
	}

	while (ros::ok()) {
		//If received last loop, so publish
		if (received) {
			sensor_msgs::JointState joint_state;
			for (std::vector<aux_struct>::iterator it = aux_vec.begin(); it != aux_vec.end(); ++it)
			{
				joint_state.name.push_back(it->joint_name.c_str());
				joint_state.position.push_back(it->value);
			}
			joint_state.header.stamp = ros::Time::now();

			joint_pub.publish(joint_state);
			received = false;
		}
		
		ros::spinOnce();
		loop.sleep();
	}

	return 0;
}

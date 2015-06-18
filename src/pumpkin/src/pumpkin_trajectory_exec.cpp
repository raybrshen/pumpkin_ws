/*
 * pumpkin_trajectory_exec.cpp
 *
 *  Created on: Jun 18, 2015
 *      Author: labrob
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

void trajectorExecutionCallback(const std_msgs::StringConstPtr& str) {
	printf("event! %s", str.get()->data.c_str());
}

void jointStatesExecutionCallback(const sensor_msgs::JointStateConstPtr& ptr) {
	std::vector<double> positions = ptr.get()->position;

	for (int i=0; i<positions.size(); i++)
		printf("event! %f ", positions[i]);
	printf("\n");
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "pumpkin_trajectory_exec");
	ros::NodeHandle nh;
//	ros::Subscriber planned_path = nh.subscribe("/trajector_execution_event", 1000, trajectorExecutionCallback);
	ros::Subscriber planned_path = nh.subscribe("/joint_states", 1000,
			jointStatesExecutionCallback);
	ros::spin();
}

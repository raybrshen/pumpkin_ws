#include <iostream>
#include <string>
#include <cstring>
#include <cmath>
//#include <cstdint>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "analog_read/analog_array.h"

//TODO: add the analog_read dependence to the package.xml

#define MAX_N_A_READS 6

uint16_t max_a_reads[MAX_N_A_READS];
uint16_t avg_a_reads[MAX_N_A_READS];
uint16_t min_a_reads[MAX_N_A_READS];
float angles[MAX_N_A_READS];

float map(uint16_t x, uint16_t in_min, uint16_t in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void analogReadCallback(const analog_read::analog_arrayConstPtr& msg) {
	for (int i = 0; i < MAX_N_A_READS; i++) {
		if (msg->an_read[i] > max_a_reads[i])
			max_a_reads[i] = msg->an_read[i];
		if (msg->an_read[i] < min_a_reads[i])
			min_a_reads[i] = msg->an_read[i];
		angles[i] = map((uint16_t)msg->an_read[i], min_a_reads[i], max_a_reads[i], 0.0, 180.0);
	}
	ROS_INFO("Angle: [%f]", angles[0]);
}

int main(int argc, char** argv) {

	//initialization
	for (int i = 0; i < MAX_N_A_READS; i++) {
		max_a_reads[i] = 0;
		avg_a_reads[i] = 0;
		min_a_reads[i] = 1024;
		angles[i] = 0.0;
	}

	ros::init(argc, argv, "sensor_reads");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("a_reads", 1000, analogReadCallback);

//	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
//	tf::TransformBroadcaster broadcaster;

	ros::Rate loop_rate(30);

//	const double degree = M_PI / 180;
//
//    // robot state
//	double tilt = 0, tinc = degree, swivel = 0, angle = 0, height = 0, hinc =
//			0.005;

    // message declarations
//	geometry_msgs::TransformStamped odom_trans;
//	sensor_msgs::JointState joint_state;
//	odom_trans.header.frame_id = "odom";
//	odom_trans.child_frame_id = "axis";
//
	while (ros::ok()) {
//		//update joint_state
//		joint_state.header.stamp = ros::Time::now();
//		joint_state.name.resize(3);
//		joint_state.position.resize(3);
//		joint_state.name[0] = "swivel";
//		joint_state.position[0] = swivel;
//		joint_state.name[1] = "tilt";
//		joint_state.position[1] = tilt;
//		joint_state.name[2] = "periscope";
//		joint_state.position[2] = height;
//
//		// update transform
//		// (moving in a circle with radius=2)
//		odom_trans.header.stamp = ros::Time::now();
//		odom_trans.transform.translation.x = cos(angle) * 2;
//		odom_trans.transform.translation.y = sin(angle) * 2;
//		odom_trans.transform.translation.z = .7;
//		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(
//				angle + M_PI / 2);
//
//		//send the joint state and transform
//		joint_pub.publish(joint_state);
//		broadcaster.sendTransform(odom_trans);
//
//		// Create new robot state
//		tilt += tinc;
//		if (tilt < -.5 || tilt > 0)
//			tinc *= -1;
//		height += hinc;
//		if (height > .2 || height < 0)
//			hinc *= -1;
//		swivel += degree;
//		angle += degree / 4;
//
//		// This will adjust as needed per iteration
//		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}

/*
 *  arduino_to_rviz
 *  Implements playback recorded files from real robot to joint state publisher in the models
 *      Author: Rafael de Paula Paiva (08.paiva@gmail.com)
 */

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include "yaml-cpp/yaml.h"

#include "pumpkin_messages/analog_array.h"

/*
y4[pin] = 0.001*(0.0582*x4[pin] + 0.2327*x3[pin] + 0.3490*x2[pin] + 0.2327*x1[pin] + 0.0582*x0[pin]) + (3.5168*y3[pin]) - (4.6637*y2[pin]) + (2.7621*y1[pin]) - (0.6161*y0[pin]);
x0[pin]=x1[pin];
x1[pin]=x2[pin];
x2[pin]=x3[pin];
x3[pin]=x4[pin];

y0[pin]=y1[pin];
y1[pin]=y2[pin];
y2[pin]=y3[pin];
y3[pin]=y4[pin];

y5[pin] = 0.001*(0.0909*x5[pin] + 0.4546*x4[pin] + 0.9093*x3[pin] + 0.9093*x2[pin] + 0.4546*x1[pin] + 0.0909*x0[pin]) - 100000*(-4.8983*y4[pin] + 9.5985*y3[pin] -9.4053*y2[pin] + 4.6085*y1[pin] -0.9033*y0[pin]);
*/

struct aux_struct {
	int pin;
    double in[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, filter[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
	double value;
	double in_min, in_max; //min, max
	double out_lower, out_upper; //min, max
	std::string joint_name;
};

std::vector<aux_struct> aux_vec;

bool received = false;

void receiveData(const pumpkin_messages::analog_arrayConstPtr& reads)
{
	for (std::vector<aux_struct>::iterator it = aux_vec.begin(); it != aux_vec.end(); ++it) {
		if (it->pin > reads->an_read.size()) {
			ROS_INFO("Pin outbound received");
			it->value = 0.0;
        } else {
            it->in[5] = double(reads->an_read[it->pin]);

            /*
             * Filter formula:
             * y5 = 0.000001 * (0.0277 * x5 + 0.1384 * x4 + 0.2769 * x3 + 0.2769 * x2 + 0.1384 * x1 + 0.0277 * x0)
             *      + 4.7967 * y4 - 9.2072 * y3 + 8.8404 * y2 - 4.2458 * y1 + 0.8160 * y0
             */
            it->filter[5] = 0.000001 * (0.0277 * it->in[5] + 0.1384 * it->in[4] + 0.2769 * it->in[3]
                                        + 0.2769 * it->in[2] + 0.1384 * it->in[1] + 0.0277 * it->in[0])
                            + 4.7967 * it->filter[4] - 9.2072 * it->filter[3] + 8.8404 * it->filter[2]
                            - 4.2458 * it->filter[1] + 0.8160 * it->filter[0];

            it->filter[0] = it->filter[1];
            it->filter[1] = it->filter[2];
            it->filter[2] = it->filter[3];
            it->filter[3] = it->filter[4];
            it->filter[4] = it->filter[5];

            it->in[0] = it->in[1];
            it->in[1] = it->in[2];
            it->in[2] = it->in[3];
            it->in[3] = it->in[4];
            it->in[4] = it->in[5];

            it->value = (it->filter[5] - it->in_min) * (it->out_upper - it->out_lower)
                                  / (it->in_max - it->in_min) + it->out_lower;

            if (it->value > it->out_upper)
                it->value = it->out_upper;
            else if (it->value < it->out_lower)
                it->value = it->out_lower;
        }
	}

	//ROS_INFO("Received signal from Arduino.");
	received = true;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "arduino_to_rviz");
	ros::NodeHandle nh;

	if (!ros::param::has("/pumpkin/config")) {
		ROS_ERROR("Robot configuration not set! Try to run node \"load_config\" in \"pumpkin package\".");
		return -1;
	}

	ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("playback_joint_states", 1024);
	//ROS_INFO("Publisher loaded.");
	ros::Subscriber arduino_data = nh.subscribe<pumpkin_messages::analog_array>("a_reads", 1024, receiveData);
	//ROS_INFO("Publisher and Subscriber loaded successfully.");
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
		aux.in_min = double(int((*cluster)[part]["analog_read_min"]));
		aux.in_max = double(int((*cluster)[part]["analog_read_max"]));
		aux.out_lower = urdf_joint.get()->limits->lower;
		aux.out_upper = urdf_joint.get()->limits->upper;

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

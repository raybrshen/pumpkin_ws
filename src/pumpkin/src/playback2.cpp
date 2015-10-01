//
// Created by rafaelpaiva on 24/09/15.
//

#include <yaml-cpp/yaml.h>
#include <XmlRpcValue.h>
#include <ros/ros.h>

#include "pumpkin_messages/SSCMoveList.h"

class PlaybackActionServer {

};

struct auxiliar_calibration {
	int arduino_min;
	int arduino_max;
	int ssc_min;
	int ssc_max;
	int ssc_pin;
};

std::vector<auxiliar_calibration> calib_vector(32);



int main (int argc, char *argv[]) {

	ros::init(argc, argv, "playback2");
	std::string file;
	ros::NodeHandle nh;

	if (argc > 1) {
		file = argv[1];
	} else {
		ROS_ERROR("No playback file inserted. Please load a file: rosrun pumpkin playback2 <playback_file>");
		return -1;
	}

	if (!ros::param::has("/pumpkin/config")) {
		ROS_ERROR("Pumpking configuration not set. Load running: roslaunch pumpkin load_config.launch");
		return -2;
	}

	XmlRpc::XmlRpcValue config;
	ros::param::get("/pumpkin/config", config);
	std::vector<YAML::Node> movements = std::move(YAML::LoadAllFromFile(file));

	/*
	 * Create calibration changes
	 * CAUTION!!!
	 * If the config doesn't have the same parts, in arduino and in ssc, we may have problems here.
	 */
	for (auto it = config["arduino"].begin(), end_it = config["arduino"].end(); it != end_it; ++it) {
		for (auto part_it = it->second.begin(), part_end_it = it->second.end(); part_it != part_end_it; ++part_it) {
			XmlRpc::XmlRpcValue && ssc_ref = std::move(config["ssc"][it->first][part_it->first]);
			calib_vector[int(part_it->second["pin"])] = std::move(auxiliar_calibration{
					int(part_it->second["analog_read_min"]),
			        int(part_it->second["analog_read_max"]),
			        int(ssc_ref["pulse_min"]),
			        int(ssc_ref["pulse_max"]),
					int(ssc_ref["pin"])
			});
		}
	}

	ROS_INFO("Adjustments calculated.");

	double ros_rate = double(config["ros_rate"]);
	ros::Rate loop(ros_rate);
	ros::Publisher ssc_pub = nh.advertise<pumpkin_messages::SSCMoveList>("move_ssc_topic", 32);

	for (auto step = std::begin(movements); step != std::end(movements) && ros::ok(); ++step) {
		if (!ros::ok()) {
			ROS_FATAL("Ros node was terminated before completition of playback.");
		} else if (step->IsNull()) {
			ROS_FATAL("Playback movement is null.");
		}

		if ((*step)["an_read"].IsNull()) {
			ROS_FATAL("NULL");
		}

		std::vector<uint16_t> reads = std::move((*step)["an_read"].as<std::vector<uint16_t> >());
		pumpkin_messages::SSCMoveList command;
		command.time = 0;

		for (int i = 0; i < reads.size(); i++) {
			pumpkin_messages::SSCMove comm_move;
			const auxiliar_calibration & aux = calib_vector[i];
			comm_move.channel = aux.ssc_pin;
			if (aux.arduino_max - aux.arduino_min <= 0)
				continue;
			else {
				comm_move.pulse = (uint16_t) ((reads[i] - aux.arduino_min) * (aux.ssc_max - aux.ssc_min)
				                    / (aux.arduino_max - aux.arduino_min) + aux.ssc_min);
				if (comm_move.pulse > aux.ssc_max) {
					ROS_WARN("Pulse overflow: %d", int(comm_move.pulse));
					comm_move.pulse = aux.ssc_max;
				} else if (comm_move.pulse < aux.ssc_min) {
					ROS_WARN("Pulse underflow: %d", int(comm_move.pulse));
					comm_move.pulse = aux.ssc_min;
				}
			}
			comm_move.speed = 0;
			command.list.emplace_back(comm_move);
			//ROS_INFO("#%d -> %d", comm_move.channel, comm_move.pulse);
		}

		ssc_pub.publish(command);

		ros::spinOnce();
		loop.sleep();
	}

	return 0;
}

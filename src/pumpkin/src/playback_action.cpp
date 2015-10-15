//
// Created by rafaelpaiva on 24/09/15.
//

#include <yaml-cpp/yaml.h>
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <file_type.h>
#include "pumpkin_messages/PlaybackAction.h"

#include "pumpkin_messages/SSCMoveList.h"

using namespace pumpkin_messages;

class PlaybackActionServer {
	struct auxiliar_calibration {
		int arduino_min;
		int arduino_max;
		int ssc_min;
		int ssc_max;
		int ssc_pin;
	};
	//private members
	ros::NodeHandle _nh;
	ros::Publisher _pub;
	actionlib::SimpleActionServer<PlaybackAction> _server;
	std::vector<auxiliar_calibration> _aux_vec;
	std::vector<YAML::Node> _movement;
	std::vector<YAML::Node>::iterator _step_it, _end_it;
	double _percentage_step;
	PlaybackFeedback _feedback;
	PlaybackResult _result;

public:
    PlaybackActionServer() : _server(_nh, "playback_action", false), _aux_vec(32) {
		_server.registerGoalCallback(boost::bind(&PlaybackActionServer::onGoal, this));
		_server.registerPreemptCallback(boost::bind(&PlaybackActionServer::onPreempt, this));

		_pub = _nh.advertise<SSCMoveList>("move_ssc_topic", 32);

        _server.start();
	}

	~PlaybackActionServer() {
		_pub.publish(SSCMoveList());
        _server.shutdown();
	}

	void onGoal() {
		if (_step_it != _end_it) {
			_pub.publish(SSCMoveList());
		}
		auto goal = _server.acceptNewGoal();
		std::string filename = goal->filename;
		_feedback.percentage = 0.0;

		try {
			_movement = std::move(YAML::LoadAllFromFile(filename));
		} catch (YAML::BadFile) {
            _result.state = static_cast<uint8_t>(IOState::BadFile);
			_server.setAborted(_result);
            ROS_FATAL("ERROR OPENING FILE");
		}

		_percentage_step = (double)100/_movement.size();
		_step_it = std::begin(_movement);
		_end_it = std::end(_movement);

        ROS_INFO("New Goal!");
	}

	void onPreempt() {
		_pub.publish(SSCMoveList());
		_result.state = static_cast<uint8_t>(IOState::OK);
		_server.setAborted(_result);
        //TODO reconfigure robot shutdown after end of scene
        _pub.publish(SSCMoveList());
	}

	void step() {
		if (!_server.isActive())
			return;

		if (_step_it->IsNull()) {
			_result.state = static_cast<uint8_t>(IOState::BadFile);
			_server.setAborted(_result);
			_step_it = _end_it;
            //TODO reconfigure robot shutdown after end of scene
            _pub.publish(SSCMoveList());
			return;
		} else if ((*_step_it)["an_read"].IsNull()) {
			_result.state = static_cast<uint8_t>(IOState::BadFile);
			_server.setAborted(_result);
			_step_it = _end_it;
            //TODO reconfigure robot shutdown after end of scene
            _pub.publish(SSCMoveList());
			return;
		}

        ROS_INFO("New step.");

		std::vector<uint16_t> reads = std::move((*_step_it)["an_read"].as<std::vector<uint16_t> >());

		SSCMoveList command;
		for (int i = 0; i < reads.size(); ++i) {
			SSCMove move;
			const auxiliar_calibration &aux = _aux_vec[i];
			move.channel = aux.ssc_pin;
            if (aux.arduino_max - aux.arduino_min <= 0) {
                ROS_WARN("Arduino calibration error");
				continue;
            } else {
				move.pulse = (uint16_t) ((reads[i] - aux.arduino_min) * (aux.ssc_max - aux.ssc_min)
				                              / (aux.arduino_max - aux.arduino_min) + aux.ssc_min);
				if (move.pulse > aux.ssc_max) {
					ROS_WARN("Pulse overflow: %d", int(move.pulse));
					move.pulse = aux.ssc_max;
				} else if (move.pulse < aux.ssc_min) {
					ROS_WARN("Pulse underflow: %d", int(move.pulse));
					move.pulse = aux.ssc_min;
				}
			}
			move.speed = 0;
			command.list.emplace_back(std::move(move));
		}

		++_step_it;

		if (command.list.size())
			_pub.publish(command);
        else
            ROS_WARN("Outbound error!");

		_feedback.percentage += _percentage_step;
		_server.publishFeedback(_feedback);

		if (_step_it == _end_it) {
			_result.state = static_cast<uint8_t>(IOState::OK);
			_server.setSucceeded(_result);
            //TODO reconfigure robot shutdown after end of scene
            _pub.publish(SSCMoveList());
		}

	}

    void calibrate (int arduino_pin, int arduino_min, int arduino_max, int ssc_min, int ssc_max, int ssc_pin) {
		_aux_vec[arduino_pin] = std::move(auxiliar_calibration {
				arduino_min,
		        arduino_max,
		        ssc_min,
		        ssc_max,
		        ssc_pin
		});
	}

};

int main (int argc, char *argv[]) {

	ros::init(argc, argv, "playback_action");
	ros::start();

	ros::Rate loop(1000);
	while (!ros::param::has("/pumpkin/config") && ros::ok()) {
		loop.sleep();
	}

	if (!ros::ok()) {
		ros::shutdown();
		return -1;
	}

	XmlRpc::XmlRpcValue config;
	ros::param::get("/pumpkin/config", config);
	//std::vector<YAML::Node> movements = std::move(YAML::LoadAllFromFile(file));
	PlaybackActionServer server;

	/*
	 * Create calibration changes
	 * CAUTION!!!
	 * If the config doesn't have the same parts, in arduino and in ssc, we may have problems here.
	 */
	for (auto it = config["arduino"].begin(), end_it = config["arduino"].end(); it != end_it; ++it) {
		for (auto part_it = it->second.begin(), part_end_it = it->second.end(); part_it != part_end_it; ++part_it) {
			XmlRpc::XmlRpcValue && ssc_ref = std::move(config["ssc"][it->first][part_it->first]);
			server.calibrate(int(part_it->second["pin"]),
			                 int(part_it->second["analog_read_min"]),
			                 int(part_it->second["analog_read_max"]),
			                 int(ssc_ref["pulse_min"]),
			                 int(ssc_ref["pulse_max"]),
			                 int(ssc_ref["pin"]));
		}
	}

	ROS_INFO("Adjustments calculated.");

	double ros_rate = double(config["ros_rate"]);
	loop = ros::Rate(ros_rate);

	while (ros::ok()) {
        ros::spinOnce();
		server.step();
		loop.sleep();
	}

	ros::shutdown();
	return 0;
}

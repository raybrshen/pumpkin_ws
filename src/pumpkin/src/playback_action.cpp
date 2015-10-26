//
// Created by rafaelpaiva on 24/09/15.
//

#include <yaml-cpp/yaml.h>
#include <XmlRpcValue.h>
#include <urdf/model.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <file_type.h>

#include "pumpkin_messages/PlaybackAction.h"
#include "sensor_msgs/JointState.h"
#include "pumpkin_messages/SSCMoveList.h"
#include "pumpkin_messages/Planner.h"

using namespace pumpkin_messages;

class PlaybackActionServer {

	enum State {
		Moving,
		RequestPlanning,
		ExecutingPlanning,
	};

	struct auxiliar_calibration {
		int arduino_min;
		int arduino_max;
		int ssc_min;
		int ssc_max;
		int ssc_pin;
		std::string joint_name;
		double joint_lower;
		double joint_upper;
	};
	//private members
	ros::NodeHandle _nh;
	ros::Publisher _ssc, _joints;
	ros::ServiceClient _planner_client;
	std::vector<std::string> _movement_files;
	actionlib::SimpleActionServer<PlaybackAction> _server;
	std::vector<auxiliar_calibration> _aux_vec;
	std::vector<YAML::Node> _movement;
	std::vector<YAML::Node>::iterator _step_it, _end_it;
	double _percentage_step;

	sensor_msgs::JointState _joint_state;
	PlaybackFeedback _feedback;
	PlaybackResult _result;
	State _state;

	unsigned int _movement_index, _plan_index;
	ros::Rate _loop;
	Planner _planner;

	void loadFile() {
		_movement = std::move(YAML::LoadAllFromFile(_movement_files[_movement_index]));
		_percentage_step = (double)100/_movement.size();
		_step_it = std::begin(_movement);
		_end_it = std::end(_movement);

		_feedback.percentage = 0.0;
		_feedback.movement_index = _movement_index << 1;
	}

public:
	PlaybackActionServer() : _server(_nh, "playback_action", false), _aux_vec(32), _loop(1000) {
		_server.registerGoalCallback(boost::bind(&PlaybackActionServer::onGoal, this));
		_server.registerPreemptCallback(boost::bind(&PlaybackActionServer::onPreempt, this));

		_planner_client = _nh.serviceClient<Planner::Request, Planner::Response>("/planner/planner");

		_ssc = _nh.advertise<SSCMoveList>("/move_ssc_topic", 32);
		_joints = _nh.advertise<sensor_msgs::JointState>("playback_joint_states", 1024);
		//_direct = true;
		_planner_client.waitForExistence();

        _server.start();
	}

	~PlaybackActionServer() {
		_ssc.publish(SSCMoveList());
        _server.shutdown();
	}

	///
	/// \brief onGoal Action Server onGoal callback function
	///
	void onGoal() {
		if (_step_it != _end_it) {
			_ssc.publish(SSCMoveList());
		}
		auto goal = _server.acceptNewGoal();
		_movement_files = std::move(goal->filenames);
		//_direct = goal->direct;
		_movement_index = 0;

		try {
			loadFile();
		} catch (YAML::BadFile) {
            _result.state = static_cast<uint8_t>(IOState::BadFile);
			_server.setAborted(_result);
            ROS_FATAL("ERROR OPENING FILE");
		}

		_state = Moving;
		_loop = ros::Rate(100);
        ROS_INFO("New Goal!");
	}

	///
	/// \brief onPreempt - Action Server onPreempt callback function
	///
	void onPreempt() {
		_ssc.publish(SSCMoveList());
		_result.state = static_cast<uint8_t>(IOState::OK);
		_server.setAborted(_result);
	}

	///
	/// \brief step - This is the main function of this class... It should be called in the main.
	/// It executes the server loop, seeking for the movements in the playback files
	///
	void step() {
		if (!_server.isActive()) {
			_loop.sleep();
			return;
		}

		SSCMoveList command;
		command.list.reserve(32);
		std::vector<uint16_t> reads;

		switch(_state) {
			case Moving:
				if (_step_it->IsNull()) {
					_result.state = static_cast<uint8_t>(IOState::BadFile);
					_server.setAborted(_result);
					_step_it = _end_it;
					_ssc.publish(SSCMoveList());
					return;
				} else if ((*_step_it)["an_read"].IsNull()) {
					_result.state = static_cast<uint8_t>(IOState::BadFile);
					_server.setAborted(_result);
					_step_it = _end_it;

					//if (_direct)
					_ssc.publish(SSCMoveList());
					return;
				}

				//ROS_INFO("New step.");

				reads = std::move((*_step_it)["an_read"].as<std::vector<uint16_t> >());

				++_step_it;

				double position;
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

						position = (reads[i] - aux.arduino_min) * (aux.joint_upper - aux.joint_lower)
						           / (aux.arduino_max - aux.arduino_min) + aux.joint_lower;

						if (position > aux.joint_upper) {
							ROS_WARN("Pulse overflow.");
							position = aux.joint_upper;
						} else if (position < aux.joint_lower) {
							ROS_WARN("Pulse underflow.");
							position = aux.joint_lower;
						}

						_joint_state.position[i] = position;
					}
					move.speed = 0;
					command.list.emplace_back(std::move(move));
				}
				command.time = 0;

				_joints.publish(_joint_state);
				if (command.list.size()) {
					_ssc.publish(command);
					command.list.clear();
				} else {
					ROS_WARN("Outbound error!");
				}

				_feedback.percentage += _percentage_step;
				_feedback.movement_index = _movement_index << 1;
				_server.publishFeedback(_feedback);

				if (_step_it == _end_it) {
					++_movement_index;
					_state = RequestPlanning;
				} else {
					_loop.sleep();
				}
			break;
			case RequestPlanning:
				if (_movement_index == _movement_files.size()) {
					_result.state = static_cast<uint8_t>(IOState::OK);
					_server.setSucceeded(_result);
					_ssc.publish(SSCMoveList());
				} else {
					_planner.request.initial_positions.reserve(_aux_vec.size());
					_planner.request.final_positions.reserve(_aux_vec.size());

					reads = std::move(_movement.back()["an_read"].as<std::vector<uint16_t> >());

					double value;
					for(int i = 0; i < reads.size(); ++i) {
						auxiliar_calibration & aux = _aux_vec[i];

						ROS_INFO("%d", reads[i]);

						if (aux.arduino_max == aux.arduino_min)
							continue;


						value = (reads[i] - aux.arduino_min) * (aux.joint_upper - aux.joint_lower)
														/ (aux.arduino_max - aux.arduino_min) + aux.joint_lower;
						if (value > aux.joint_upper) {
							ROS_WARN("Pulse overflow.");
							value = aux.joint_upper;
						} else if (value < aux.joint_lower) {
							ROS_WARN("Pulse underflow.");
							value = aux.joint_lower;
						}

						_planner.request.initial_positions.push_back(value);
					}

					loadFile();

					reads = std::move(_movement.front()["an_read"].as<std::vector<uint16_t> >());

					for(int i = 0; i < reads.size(); ++i) {
						auxiliar_calibration & aux = _aux_vec[i];

						if (aux.arduino_max == aux.arduino_min == 0)
							continue;


						double value = (double) ((reads[i] - aux.arduino_min) * (aux.joint_upper - aux.joint_lower)
														/ (aux.arduino_max - aux.arduino_min) + aux.joint_lower);
						if (value > aux.joint_upper) {
							ROS_WARN("Pulse overflow: %lf", value);
							value = aux.joint_upper;
						} else if (value < aux.joint_lower) {
							ROS_WARN("Pulse underflow: %lf", value);
							value = aux.joint_lower;
						}

						_joint_state.position[i] = value;
						_planner.request.final_positions.push_back(value);
					}

					_joints.publish(_joint_state);
					ROS_WARN("Starting request plan");
					if (!_planner_client.call(_planner)) {
						ROS_FATAL("COULD NOT CALL PLANNER SERVICE.");
						_ssc.publish(SSCMoveList());
						_result.state = static_cast<uint8_t>(IOState::BadFile);//Has to change here for a new code
						_server.setAborted(_result);
						_state = Moving;
					}
					_plan_index = 0;
					_state = ExecutingPlanning;
				}
			break;
			case ExecutingPlanning:
				if (_plan_index == _planner.response.joint_trajectory[0].points.size()) {
					_loop = ros::Rate(100);
					_state = Moving;
				} else {
					double position;
					ROS_WARN("Executing planning");
					_loop = ros::Rate(_planner.response.joint_trajectory[0].points[_plan_index].time_from_start);
					for (auto it = _planner.response.joint_trajectory.begin(); it != _planner.response.joint_trajectory.end(); ++it) {
						trajectory_msgs::JointTrajectoryPoint &trajectory = it->points[_plan_index];

						std::vector<std::string> &names = it->joint_names;

						for (int i = 0, size = names.size(); i < size; ++i) {
							SSCMove move;
							const std::string &name = names[i];
							auto it = std::find_if(_aux_vec.begin(), _aux_vec.end(),
							                       [name](const auxiliar_calibration &x) {
								                       return name == x.joint_name;
							                       });

							move.channel = it->ssc_pin;
							move.pulse = (uint16_t) ((trajectory.positions[it - _aux_vec.begin()] - it->joint_lower) *
							                         (it->ssc_max - it->ssc_min)
							                         / (it->joint_upper - it->joint_lower) + it->ssc_min);
							if (move.pulse > it->ssc_max) {
								ROS_WARN("Pulse overflow: %d", int(move.pulse));
								move.pulse = it->ssc_max;
							} else if (move.pulse < it->ssc_min) {
								ROS_WARN("Pulse underflow: %d", int(move.pulse));
								move.pulse = it->ssc_min;
							}
							move.speed = 0;
							command.list.emplace_back(move);

							position = (trajectory.positions[it - _aux_vec.begin()] - it->arduino_min) * (it->joint_upper - it->joint_lower)
							           / (it->arduino_max - it->arduino_min) + it->joint_lower;

							if (position > it->joint_upper) {
								ROS_WARN("Pulse overflow.");
								position = it->joint_upper;
							} else if (position < it->joint_lower) {
								ROS_WARN("Pulse underflow.");
								position = it->joint_lower;
							}

							_joint_state.position[i] = position;
						}
					}

					command.time = 0;
					_ssc.publish(command);
					command.list.clear();
					_joints.publish(_joint_state);

					_feedback.percentage = _plan_index/_planner.response.joint_trajectory[0].points.size();
					_feedback.movement_index = (_movement_index << 1) | 1;

					_plan_index++;
					_loop.sleep();
				}
			break;
		}
	}

	void calibrate (int arduino_pin, int arduino_min, int arduino_max, int ssc_min, int ssc_max, int ssc_pin,
					const std::string &joint_name, double joint_lower, double joint_upper) {
		_aux_vec[arduino_pin] = std::move(auxiliar_calibration{
				arduino_min,
				arduino_max,
				ssc_min,
				ssc_max,
				ssc_pin,
				joint_name,
				joint_lower,
				joint_upper,
		});
		_joint_state.name.push_back(joint_name);
		_joint_state.position.push_back(0.0);
	}

	void calibrate(double rate) {
		_loop = ros::Rate(rate);
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

	urdf::Model pumpkinModel;
	std::string model_name;

	if (!ros::param::get("/pumpkin/description_file", model_name)) {
		ROS_FATAL("Could not get model name");
		ros::shutdown();
		return -1;
	}

	pumpkinModel.initFile(model_name);

	PlaybackActionServer server;

	/*
	 * Create calibration changes
	 * CAUTION!!!
	 * If the config doesn't have the same parts, in arduino and in ssc, we may have problems here.
	 */
	auto joints = pumpkinModel.joints_;
	for (auto it = config["arduino"].begin(), end_it = config["arduino"].end(); it != end_it; ++it) {
		for (auto part_it = it->second.begin(), part_end_it = it->second.end(); part_it != part_end_it; ++part_it) {
			XmlRpc::XmlRpcValue & ssc_ref = config["ssc"][it->first][part_it->first];
			std::string joint_name = (it->first)[0] + ("_" + part_it->first);
			server.calibrate(int(part_it->second["pin"]),
			                 int(part_it->second["analog_read_min"]),
			                 int(part_it->second["analog_read_max"]),
			                 int(ssc_ref["pulse_min"]),
			                 int(ssc_ref["pulse_max"]),
							 int(ssc_ref["pin"]),
							 joint_name,
							 joints[joint_name]->limits->lower,
							 joints[joint_name]->limits->upper);
		}
	}

	ROS_INFO("Adjustments calculated.");

	double ros_rate = double(config["ros_rate"]);
	//loop = ros::Rate(ros_rate);
	server.calibrate(ros_rate);

	while (ros::ok()) {
        ros::spinOnce();
		server.step();
		//loop.sleep();
	}

	ros::shutdown();
	return 0;
}

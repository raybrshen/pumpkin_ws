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

namespace pumpkin {

	/*!
	 * \brief This class is the action server that handles the playback and scene.
	 *
	 * It has a quite complex structure. It creates an Action Server to serve the playback requisitons.
	 * It has also two publishers (that publish to the SSC and to the Joint State, for the RViz).
	 * And a Service Client to call the Pumpkin Planner
	 */
	class PlaybackActionServer {

		/*!
		 * \brief Enum to indicate in wich state the playback is.
		 */
		enum State {
			Moving, //!< The playback is in the midle of a movement
			RequestPlanning, //!< The playback is changing the movement, and calling the planner
			ExecutingPlanning, //!< The playback is executing the planned trajectory
		};

		/*!
		 * \brief Struct to handle the parameters for the linear conversion
		 */
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
		ros::NodeHandle _nh; //!< ROS node handle
		ros::Publisher _ssc; //!< Publishes to SSC
		ros::Publisher _joints; //!< Publishes to Joint States
		ros::ServiceClient _planner_client; //!< Service Client to the Pumpkin Planner
		std::vector<std::string> _movement_files; //!< The list of all files of an scene
		actionlib::SimpleActionServer<PlaybackAction> _server; //!< Action Server
		std::vector<auxiliar_calibration> _aux_vec; //!< Vector to hold the calibration parameters for each joint, indexed by Arduino PIN
		std::vector<YAML::Node> _movement; //!< The movements of an file
		std::vector<YAML::Node>::iterator _step_it, _end_it;
		double _percentage_step; //!< This hold the percentage incremental value
		double _rate; //!< This holds the rate of the loop

		sensor_msgs::JointState _joint_state;   //!< The joint state message
		PlaybackFeedback _feedback;             //!< The feedback message
		PlaybackResult _result;                 //!< The result message
		State _state;                           //!< Holds the state of the playback
		Planner _planner;                       //!< Pumplin Planner message

		unsigned int _movement_index;   //!< Holds the index of the actual position in the YAML vector
		unsigned int _plan_index;       //!< Holds the actual index of the movement in the scene list
		ros::Rate _loop;                //!< ROS Rate
		//ros::Duration _last_delta;

		/*!
		 * \brief This method loads the movement file, based on the `_movement_index`.
		 *
		 * It will also close the last movement, and set the iterators.
		 */
		void loadFile();

		/*!
		 * \brief This method runs the move the robot to a specific position in an movement.
		 *
		 * This method is called when the Playback is in `Moving` state. It sends the command to SSC and Joint States.
		 * If the movement ends, this methods sets the server to `RequestPlanning` state, and end the turn.
		 */
		void move();

		/*!
		 * \brief This method check if it is other movement to be done, and, in case, call the planner.
		 *
		 * This method is called when the Playback is in `RequestPlanning` state. It first checks if it have any other
		 * movement to be done. If so, it will calculate the inicial position to plan (based on the last position), open
		 * the next movement file, and calculate the final position. After, it calls the "Pumpkin Planner", and sets the
		 * Playback to the `ExecutingPlanning` state.
		 */
		void prepare();

		/*!
		 * \brief This method executes the planned transition between movements.
		 *
		 * This method is called when the Playback is in `ExecutingPlanning` state. It will send commands to SSC based on the
		 * response from the Pumpkin Planner service call. After finishing, it sets the Playback to the `Moving`state.
		 */
		void change();

	public:
		/*!
		 * \brief Constructor. This creates the Action Server and the client to the pumpkin planner.
		 */
		PlaybackActionServer() : _server(_nh, "/pumpkin/playback_action", false), _aux_vec(32), _loop(1000) {
			_server.registerGoalCallback(boost::bind(&PlaybackActionServer::onGoal, this));
			_server.registerPreemptCallback(boost::bind(&PlaybackActionServer::onPreempt, this));

			_planner_client = _nh.serviceClient<Planner::Request, Planner::Response>("/pumpkin/planner");

			_ssc = _nh.advertise<SSCMoveList>("/move_ssc_topic", 32);
			_joints = _nh.advertise<sensor_msgs::JointState>("/playback_joint_states", 1024);
			//_direct = true;
			_planner_client.waitForExistence();

			ROS_INFO("Planner started.");

			_joint_state.name = std::vector<std::string>(32);
			_joint_state.position = std::vector<double>(32);

			_server.start();
		}

		/*!
		 * \brief Destructor. Just send an "Turn OFF" message to the SSC and shutsdown the server.
		 */
		~PlaybackActionServer() {
			_ssc.publish(SSCMoveList());
			_server.shutdown();
		}

		/*!
		 * \brief Action Server onGoal callback function
		 */
		void onGoal();

		/*!
		 * \brief Action Server onPreempt callback function
		 */
		void onPreempt();

		/*!
		 * \brief This is the main function of this class... It should be called in the main.
		 * It executes the server loop, seeking for the movements in the playback files
		 */
		void step() {
			if (!_server.isActive()) {
				_loop.sleep();
				return;
			}

			switch (_state) {
				case Moving:
					move();
					break;
				case RequestPlanning:
					prepare();
					break;
				case ExecutingPlanning:
					change();
					break;
			}
		}

		/*!
		 * \brief This function is fot setting the `auxiliar_calibration` structure. Each call set one joint.
		 *
		 * \param arduino_pin   The related PIN on Arduino. That is used for indexing the calibration vector.
		 * \param arduino_min   The minimum value the Arduino can read from this joint.
		 * \param arduino_max   The maximum value the Arduino can read from this joint.
		 * \param ssc_min       The value sent to SSC to put the joint to its lower bound.
		 * \param ssc_max       The value sent to SSC to put the joint to its upper bound.
		 * \param ssc_pin       The SSC pin to send the position of the joint.
		 * \param joint_name    The name of the joint *AS IT APPEARS ON THE ROBOT MODEL*.
		 * \param joint_lower   The lower bound angle that the joint is able to be.
		 * \param joint_upper   The upper bount angle that the joint is able to be.
		 */
		void calibrate(int arduino_pin, int arduino_min, int arduino_max, int ssc_min, int ssc_max, int ssc_pin,
		               const std::string &joint_name, double joint_lower, double joint_upper);


		/*!
		 * \brief Finish adjustments on calibration.
		 *
		 * This method is required to the server acts ok. __Call it only after calling *calibrate* to all the joints__.
		 */
		void endCalibrate(double rate);

	};

	/*
	 * It load the movements from the next file. (THE MOVEMENT INDEX IS INCREMENTED BEFORE CALLING THIS METHOD).
	 * Sets the percentage step, and the iterators.
	 * And also the feedback message.
	 */
	void PlaybackActionServer::loadFile() {
		_movement = std::move(YAML::LoadAllFromFile(_movement_files[_movement_index]));
		_percentage_step = (double) 100 / _movement.size();
		_step_it = std::begin(_movement);
		_end_it = std::end(_movement);

		_feedback.percentage = 0.0;
		_feedback.movement_index = _movement_index << 1;
	}

	/*
	 * This method is the move function. It will abort the service if it finds any error on the YAML file loaded
	 * before (on loadFile).
	 */
	void PlaybackActionServer::move() {

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

		std::vector<uint16_t> reads;
		SSCMoveList command;
		command.list.reserve(32);

		//ROS_INFO("New step.");

		//Extracts the info here
		reads = std::move((*_step_it)["an_read"].as<std::vector<uint16_t> >());

		++_step_it;

		//Calculate the SSC command and the joint state.
		double position;
		for (int i = 0; i < reads.size(); ++i) {
			SSCMove move;
			const auxiliar_calibration &aux = _aux_vec[i];
			move.channel = aux.ssc_pin;
			if (aux.arduino_max - aux.arduino_min <= 0) {
				//ROS_WARN("Arduino calibration error");
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

		//Lets publish this thing
		_joints.publish(_joint_state);
		if (command.list.size()) {
			_ssc.publish(command);
		} else {
			ROS_WARN("Outbound error!");
		}

		//Send feedback
		_feedback.percentage += _percentage_step;
		_feedback.movement_index = _movement_index << 1;
		_server.publishFeedback(_feedback);

		//Check if the movement has ended
		if (_step_it == _end_it) {
			++_movement_index;
			_state = RequestPlanning;
		} else {
			_loop.sleep();
		}
	}

	void PlaybackActionServer::prepare() {
		//Check if there is still any movement
		//If not, sent and "Turn OFF" message to SSC, and succeed this goal.
		if (_movement_index == _movement_files.size()) {
			_result.state = static_cast<uint8_t>(IOState::OK);
			_server.setSucceeded(_result);
			_ssc.publish(SSCMoveList());
			return;
		}
		//But, if not...
		std::vector<uint16_t> reads;
		_planner.request.initial_positions.clear();
		_planner.request.final_positions.clear();
		_planner.response.joint_trajectory.clear();

		reads = std::move(_movement.back()["an_read"].as<std::vector<uint16_t> >());

		//Calculates the joint state of the last step of the movement
		//AS THE INITIAL POSITION OF THE PLANNED TRAJECTORY
		double value;
		for (int i = 0; i < reads.size(); ++i) {
			auxiliar_calibration &aux = _aux_vec[i];

			//ROS_INFO("%d", reads[i]);

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

		//Load next movement
		loadFile();

		reads.clear();  //Much probably unecessary, but, just for sure
		reads = std::move((*_step_it)["an_read"].as<std::vector<uint16_t> >());

		//Calculates the first step of the next movement
		//AS THE FINAL POSITION OF THE PLANNED TRAJECTORY
		for (int i = 0; i < reads.size(); ++i) {
			auxiliar_calibration &aux = _aux_vec[i];

			if (aux.arduino_max == aux.arduino_min)
				continue;


			value = (reads[i] - aux.arduino_min) * (aux.joint_upper - aux.joint_lower)
			        / (aux.arduino_max - aux.arduino_min) + aux.joint_lower;
			if (value > aux.joint_upper) {
				ROS_WARN("Pulse overflow: %lf", value);
				value = aux.joint_upper;
			} else if (value < aux.joint_lower) {
				ROS_WARN("Pulse underflow: %lf", value);
				value = aux.joint_lower;
			}

			//_joint_state.position[i] = value;
			_planner.request.final_positions.push_back(value);
		}

		//Now, let us call the planner
		ROS_INFO("Starting request plan");
		if (!_planner_client.call(_planner)) {
			//If the planner is not possible, abort
			ROS_FATAL("COULD NOT CALL PLANNER SERVICE.");
			_ssc.publish(SSCMoveList());
			_result.state = static_cast<uint8_t>(IOState::Other);
			_server.setAborted(_result);
			_state = Moving;
			return;
		}
		//If the planner is a success, prepare to send messages of the trajectory
		_plan_index = 0;
		_state = ExecutingPlanning;
		//_last_delta = ros::Duration(0);
		_loop = ros::Rate(_rate / 10);
		try {
			ROS_INFO("Planned with %d movements.", (int) _planner.response.joint_trajectory.at(0).points.size());
		} catch (std::exception &ex) {
			ROS_FATAL("Error obtaining the response trajectory: %s", ex.what());
		}
	}

	/*
	 * This method will send the messages for the trajectory
	 */
	void PlaybackActionServer::change() {
		//First, check if the trajectory is finished
		//If so, set the state to Moving, and reset the rate
		if (_plan_index == _planner.response.joint_trajectory[0].points.size()) {
			_loop = ros::Rate(_rate);
			_state = Moving;
			//ROS_ERROR("Mas será possível...");
			return;
		}
		//Otherwise...
		SSCMoveList command;
		command.list.reserve(32);
		double position;
		ROS_WARN("Executing planning");
		//_loop = ros::Rate(_planner.response.joint_trajectory[0].points[_plan_index].time_from_start);
		//Each element the joint_trajectory vector is a planned trajectory for a specific joint group
		for (auto it = _planner.response.joint_trajectory.begin();
		     it != _planner.response.joint_trajectory.end(); ++it) {
			trajectory_msgs::JointTrajectoryPoint &trajectory = it->points[_plan_index];

			std::vector<std::string> &names = it->joint_names;

			for (int i = 0, size = names.size(); i < size; ++i) {
				SSCMove move;
				const std::string &name = names[i];
				auto it = std::find_if(_aux_vec.begin(), _aux_vec.end(),
				                       [name](const auxiliar_calibration &x) {
					                       return name == x.joint_name;
				                       });
				position = trajectory.positions[it - _aux_vec.begin()];

				move.channel = it->ssc_pin;
				move.pulse = (uint16_t) ((position - it->joint_lower) * (it->ssc_max - it->ssc_min)
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

				_joint_state.position[i] = position;
			}
		}
		command.time = 0;
		//const ros::Duration &delta = _planner.response.joint_trajectory[0].points[_plan_index].time_from_start;
		//ROS_INFO("Delta T de: %f", delta.toSec());

		//_loop = ros::Rate(delta - _last_delta);
		//_last_delta = delta;

		_ssc.publish(command);
		_joints.publish(_joint_state);

		//Send the feedback
		_feedback.percentage = _plan_index / _planner.response.joint_trajectory[0].points.size();
		_feedback.movement_index = (_movement_index << 1) | 1;
		_server.publishFeedback(_feedback);

		_plan_index++;
		_loop.sleep();
	}

	/*
	 * When accepting a new goal, load the file list to the inner vector, ajusts the members and so on...
	 */
	void PlaybackActionServer::onGoal() {
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

	/*
	 * On preempt, abort!
	 */
	void PlaybackActionServer::onPreempt() {
		_ssc.publish(SSCMoveList());
		_result.state = static_cast<uint8_t>(IOState::OK);
		_server.setAborted(_result);
	}

	void PlaybackActionServer::calibrate(int arduino_pin, int arduino_min, int arduino_max, int ssc_min,
	                                     int ssc_max, int ssc_pin, const std::string &joint_name,
	                                     double joint_lower, double joint_upper) {
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
		_joint_state.name[arduino_pin] = joint_name;
		ROS_INFO("Created joint %s", joint_name.c_str());
	}

	void PlaybackActionServer::endCalibrate(double rate) {
		_rate = rate;
		_loop = ros::Rate(rate);
		while (_aux_vec.back().arduino_max == 0)
			_aux_vec.pop_back();
		while (_joint_state.name.back().empty()) {
			_joint_state.name.pop_back();
			_joint_state.position.pop_back();
		}
		_planner.request.initial_positions.reserve(_aux_vec.size());
		_planner.request.final_positions.reserve(_aux_vec.size());
	}

}

/*!
 * \brief This node runs a Playback Action Server, calibrate the action server, and runs it.
 */
int main (int argc, char *argv[]) {
	//Init ROS and wait the config to be set
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

	//Load config
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

	pumpkin::PlaybackActionServer server;

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
	//That end calibrate set the ROS rate and adjusts the inner vectors
	server.endCalibrate(ros_rate);

	//That server `step` is for the server to the stuff. The ROS rate is inside that
	while (ros::ok()) {
		ros::spinOnce();
		server.step();
		//loop.sleep();
	}

	ros::shutdown();
	return 0;
}

/*
 * yaml_test.cpp
 *
 *  Created on: 2015-03-01
 *      Author: thiago
 */

#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <pumpkin_messages/analog_array.h>
#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>
#include <pumpkin_messages/RecordAction.h>
#include "file_type.h"

using namespace pumpkin_messages;

namespace pumpkin {

	class RecordActionServer {
		//private members
		ros::NodeHandle _nh; //!< Node handle to set up the server
		ros::Subscriber _sub; //!< To subscribe to the Arduino
		actionlib::SimpleActionServer<pumpkin_messages::RecordAction> _server; //!< Action Server
		YAML::Node _node; //!< This will handle the message as a YAML node to further save
		std::ofstream _file; //!< Output file
		unsigned int _count; //!< This count the number of messages read in a single movement
		ros::Time _start; //!< This gets the time that the record is started
		double _max_time; //!< Maximum time for the movement
		bool _first_move; //!< Member to check if is the first message for the move or not (it is important to make the "---")
		bool _verbose; //!< Allow the verbose info
		pumpkin_messages::RecordFeedback _feedback; //!< The feedback message
		pumpkin_messages::RecordResult _result; //!< The result message

	public:
		/*!
		 * \brief Constructor. It set up the action server, and subscribe from Arduino to further record.
		 * /param setVerbose    Set this TRUE to print to the screen (or log) the raw content of the Arduino message.
		 */
		RecordActionServer(bool setVerbose = false) : _server(_nh, "/pumpkin/recorder_action", false),
		                                              _verbose(setVerbose) {
			_server.registerGoalCallback(boost::bind(&RecordActionServer::onGoal, this));
			_server.registerPreemptCallback(boost::bind(&RecordActionServer::onPreempt, this));

			_sub = _nh.subscribe("/a_reads", 1000, &RecordActionServer::onRead, this);
			_server.start();
		}

		/*!
		 * \brief Destructor. It closes any file that may be opened.
		 * And also shutdowns off the server and the subscription.
		 */
		~RecordActionServer() {
			_sub.shutdown();
			if (_file.is_open())
				_file.close();
			_server.shutdown();
		}

		/*!
		 * \brief Action callback method.
		 *
		 * This function is called everytime the action server receive a new order.
		 * It will start to record a new movement from here.
		 */
		void onGoal();

		/*!
		 * \brief Action callback method.
		 *
		 * This function is called everytime a preemption order is received.
		 * A preemption order is a imediate stop to the movement record.
		 */
		void onPreempt();

		/*!
		 * \brief Arduino subscription callback method.
		 *
		 * This function is called everytime this node receives a message from Arduino.
		 * If the server is down the message is discarded. Otherwise...
		 *
		 * This is the most important function. If the server is up, it will save the message in the YAML file,
		 * check the movement is not finished (within the time limit, not the preemption), and send a feedback
		 * message to the caller node.
		 */
		void onRead(const pumpkin_messages::analog_arrayConstPtr &msg);
	};

	/*
	 * Closes the file and set the server as preempted.
	 * (Finishing the job)
	 */
	void RecordActionServer::onPreempt() {
		//_file << "---" << std::endl;
		_file.close();
		_server.setPreempted();
		ROS_WARN("Preempted file.");
	}

	/*
	 * This function uses quite a decorated way.
	 * It accept the goal, open the file, and adjust the object to start recording.
	 * If the file could not be created, or opened, the movement is not recorded, and an error message
	 * is sent to the caller.
	*/
	void RecordActionServer::onGoal() {
		if (_file.is_open())
			_file.close();
		_count = 0;
		auto goal = _server.acceptNewGoal();
		std::string filename = goal->filename;
		_max_time = goal->max_time;
		if (filename.empty()) {
			std::ostringstream s;
			s << "raw_" << std::time(nullptr) << ".yaml";
			filename = s.str();
		}
		_file.open(filename.c_str(), std::fstream::out | std::fstream::trunc);
		_start = ros::Time::now();
		if (!_file.is_open()) {
			_result.time_elapsed = 0.0;
			_result.num_steps = 0;
			_result.state = static_cast<uint8_t>(IOState::ErrorOpening);
			_server.setAborted(_result);
		}
		_first_move = true;
		ROS_INFO("Starting recording...");
	}

	/*
	 * This is where the magic happens.
	 * See the doxygen comment above to understand.
	 */
	void RecordActionServer::onRead(const pumpkin_messages::analog_arrayConstPtr &msg) {
		if (!_server.isActive())
			return;

		//If the not the first move, create the separator
		if (!_first_move) {
			_file << "---" << std::endl;
		} else {
			_first_move = false;
		}

		//If the verbose, print the message
		//Be adviced. This is A LOT OF INFORMATION
		if (_verbose) {
			std::cout << msg << std::endl;
		}

		//If it is first time, it will create the YAML structure
		//Otherwise, it will just overwrite the data in the structure
		_node["header"]["seq"] = msg->header.seq;
		_node["header"]["stamp"]["secs"] = msg->header.stamp.sec;
		_node["header"]["stamp"]["nsecs"] = msg->header.stamp.nsec; // node["seq"] automatically becomes a sequence
		_node["header"]["frame_id"] = msg->header.frame_id;
		for (int i = 0; i < msg->an_read.size(); i++)
			_node["an_read"][i] = msg->an_read[i];

		//Prints the node
		_file << _node << std::endl;

		//Fill up the feedback message, and send
		_feedback.num_steps = ++_count;
		_feedback.time_elapsed = (ros::Time::now() - _start).toSec();
		_server.publishFeedback(_feedback);

		//Error file
		if (!_file.good()) {
			_result.num_steps = _feedback.num_steps;
			_result.time_elapsed = _feedback.time_elapsed;
			_result.state = static_cast<uint8_t>(_file.eof() ? IOState::EndOfFile : IOState::BadFile);
			_server.setAborted(_result);
			_file.close();
			ROS_ERROR("File error");
			return;
		}

		//Movement finished
		if (_feedback.time_elapsed >= _max_time) {
			_result.time_elapsed = _feedback.time_elapsed;
			_result.num_steps = _feedback.num_steps;
			_result.state = static_cast<uint8_t>(IOState::OK);
			_server.setSucceeded(_result);
			_file.close();
			ROS_INFO("Record successfull.");
			return;
		}
	}
}

/*YAML::Node node;
std::fstream fs;

void analogReadCallback(const pumpkin_messages::analog_arrayConstPtr& msg) {
	fs << "---";
	fs << std::endl;

	node["header"]["seq"] = msg->header.seq;  // it now is a map node
	node["header"]["stamp"]["secs"] = msg->header.stamp.sec;
	node["header"]["stamp"]["nsecs"] = msg->header.stamp.nsec; // node["seq"] automatically becomes a sequence
	node["header"]["frame_id"] = msg->header.frame_id;
	for (int i = 0; i < msg->an_read.size(); i++)
		node["an_read"][i] = msg->an_read[i];


	fs << node;
	fs << std::endl;
}*/

/*!
 * \brief This is the "recorder" node.
 *
 * It will just set up the Action server, and spin the ROS.
 */
int main(int argc, char** argv) {

	ros::init(argc, argv, "recorder");
	ros::start();

	pumpkin::RecordActionServer server;

	ros::spin();

	ros::shutdown();

	return 0;
}


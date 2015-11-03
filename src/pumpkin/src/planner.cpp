#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include "pumpkin_messages/Planner.h"
#include "file_type.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

namespace pmsg = pumpkin_messages;

namespace pumpkin {

	/**
	 * This class is for planning the transitions between movements in a scene.
	 * It implements a ROS Service from wich is communicated from "playback_action" node.
	 * And a PlanningPipeline that makes all stuff.
	 */
	class PumpkinPlanner {
		ros::ServiceServer _server;
		ros::NodeHandle _nh, _snh;
		ros::CallbackQueue _serverQueue;

		robot_model::RobotModelPtr _model;
		planning_scene::PlanningScenePtr _planningScene;
		//planning_interface::PlannerManagerPtr _planner;
		planning_pipeline::PlanningPipelinePtr _plannerPipeline;
		std::vector<robot_state::JointModelGroup *> _joint_groups;

		std::vector<int> _indexes;

	public:
		/**
		 * \brief Constructor. It loads the robot model, the scene (virtual world), the PlanningPipeline...
		 * and run the server.
		 */
		PumpkinPlanner() {
			ROS_INFO("%s", _nh.getNamespace().c_str());
			robot_model_loader::RobotModelLoader pumpkinModelLoader("robot_description");
			_model = pumpkinModelLoader.getModel();

			_planningScene.reset(new planning_scene::PlanningScene(_model));
			std::string plannerName;

			_plannerPipeline.reset(
					new planning_pipeline::PlanningPipeline(_model, _nh, "planning_plugin", "request_adapters"));
			_plannerPipeline->displayComputedMotionPlans(false);
			_plannerPipeline->checkSolutionPaths(false);
			_plannerPipeline->publishReceivedRequests(false);

			ROS_INFO("Model and scene loaded.");
			/*
			boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > plannerLoader;

			if (!_nh.getParam("planning_plugin", plannerName))
				ROS_FATAL_STREAM("Could not find planner plugin name");
			try
			{
				plannerLoader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
			}
			catch(pluginlib::PluginlibException& ex)
			{
				ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
			}
			try
			{
				_planner.reset(plannerLoader->createUnmanagedInstance(plannerName));
				if (!_planner->initialize(_model, _nh.getNamespace()))
					ROS_FATAL_STREAM("Could not initialize planner instance");
				ROS_INFO_STREAM("Using planning interface '" << _planner->getDescription() << "'");
			}
			catch(pluginlib::PluginlibException& ex)
			{
				const std::vector<std::string> &classes = plannerLoader->getDeclaredClasses();
				std::stringstream ss;
				for (std::size_t i = 0 ; i < classes.size() ; ++i)
					ss << classes[i] << " ";
				ROS_ERROR_STREAM("Exception while loading planner '" << plannerName << "': " << ex.what() << std::endl
								 << "Available plugins: " << ss.str());
			}
			*/

			_snh.setCallbackQueue(&_serverQueue);

			_server = _snh.advertiseService("/pumpkin/planner", &PumpkinPlanner::plan, this);
			ROS_INFO("Planner initialized.");
		}

		/**
		 * \brief Destructor. Just stops the server.
		 */
		~PumpkinPlanner() {
			_server.shutdown();
		}

		/**
		 * \brief This method if for load a joint model group to plan (based on the already calibrated robot joints).
		 * It also sets up a index vector, for knowing the indexes of the joints that come from the "playback_action".
		 */
		bool setJoints(const std::string &group_name, const std::map<std::string, int> &joints);

		/**
		 * \brief Callback function that is called each time the service is required.
		 */
		bool plan(pmsg::PlannerRequest &req, pmsg::PlannerResponse &res);

		/**
		 * \brief This function should be called in the main loop, since it's using two callbacks
		 * (one for the callback and other for the service).
		 * I don't know if it's really needed, but it wasn't working, but it is now.
		 */
		inline void rosCall() { _serverQueue.callAvailable(); }
	};

	bool PumpkinPlanner::plan(pmsg::PlannerRequest &req, pmsg::PlannerResponse &res) {

		ROS_INFO("Calling planner");

		planning_interface::MotionPlanRequest planReq;
		planning_interface::MotionPlanResponse planRes;
		moveit_msgs::RobotTrajectory resMsg;

		robot_state::RobotState &now = _planningScene->getCurrentStateNonConst();

		robot_state::RobotState after(_model);
		int i = 0;
		for (auto it = _joint_groups.begin(); it != _joint_groups.end(); ++it) {
			planReq.goal_constraints.clear();
			std::vector<double> joint_now, joint_after;
			(*it)->getVariableDefaultPositions(joint_now);
			for (auto it2 = joint_now.begin(); it2 != joint_now.end(); ++it2)
				std::cout << *it2 << ' ';
			std::cout << std::endl;
			//joint_now.resize((*it)->getVariableCount());
			joint_after = joint_now;
			for (int j = 0; j != joint_now.size(); ++j) {
				int index = _indexes[i++];
				if (index == -1)
					continue;
				joint_now[j] = req.initial_positions[index];
				joint_after[j] = req.final_positions[index];
			}
			for (auto it2 = joint_after.begin(); it2 != joint_after.end(); ++it2)
				std::cout << *it2 << ' ';
			std::cout << std::endl;

			for (auto it2 = (*it)->getActiveJointModelsBounds().begin();
			     it2 != (*it)->getActiveJointModelsBounds().end(); ++it2) {
				if ((*it2)->at(0).position_bounded_)
					std::cout << "Min position: " << (*it2)->at(0).min_position_ << ", max position: " <<
					(*it2)->at(0).max_position_ << std::endl;
				else
					std::cout << "Joint not bounded" << std::endl;
			}

			now.setJointGroupPositions(*it, joint_now);
			after.setJointGroupPositions(*it, joint_after);
		}
		planReq.allowed_planning_time = 1.0;
		//planReq.max_velocity_scaling_factor = 0.1;
		for (auto it = _joint_groups.begin(); it != _joint_groups.end(); ++it) {
			planReq.group_name = (*it)->getName();
			planReq.goal_constraints.clear();

			planReq.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(after, *it));
			//planning_interface::PlanningContextPtr context = _planner->getPlanningContext(_planningScene, planReq, planRes.error_code_);
			//ROS_INFO("Get context.");
			//context->solve(planRes);
			_plannerPipeline->generatePlan(_planningScene, planReq, planRes);
			if (planRes.error_code_.val != planRes.error_code_.SUCCESS) {
				ROS_ERROR("Could not get the planned trajectory. Error: %d", planRes.error_code_.val);
				return false;
			} else {
				ROS_INFO("Planned ok.");
			}
			_planningScene->setCurrentState(planRes.trajectory_->getLastWayPoint());
			//planRes.getMessage(resMsg);
			ROS_INFO("%d", (int) planRes.trajectory_->getWayPointCount());
			planRes.trajectory_->getRobotTrajectoryMsg(resMsg);
			res.joint_trajectory.emplace_back(resMsg.joint_trajectory);
		}
		ROS_INFO("Planning complete.");
		return true;
	}

	bool PumpkinPlanner::setJoints(const std::string &group_name, const std::map<std::string, int> &joints) {
		_joint_groups.push_back(_model->getJointModelGroup(group_name));
		const std::vector<std::string> &ordered_names = _joint_groups.back()->getVariableNames();
		for (std::vector<std::string>::const_iterator it = ordered_names.begin(); it != ordered_names.end(); ++it) {
			auto it2 = joints.find(*it);
			if (it2 == joints.end()) {
				ROS_WARN("Coud not find %s for %s part", it->c_str(), group_name.c_str());
				_indexes.push_back(-1);
			} else {
				ROS_WARN("Inserted joint %s at position %d", it->c_str(), it2->second);
				_indexes.push_back(it2->second);
			}
		}
		return true;
	}

}

int main(int argc, char *argv[]) {
	//Start ROS
	ros::init(argc, argv, "pumpkin_planner");
	ros::start();

	//Wait for the config to be loaded
	ros::Rate loop(1000);
	while (!ros::param::has("pumpkin/config")) {
		loop.sleep();
	}

	//Now we create the planner and all that stuff.
	pumpkin::PumpkinPlanner planner;

	//We load the pumpkin calib config
	XmlRpc::XmlRpcValue config;
	ros::param::get("/pumpkin/config/arduino", config);

	//And now we set the joints
	//I use a map to identify the joint name and the pin
	for (auto it = config.begin(); it != config.end(); ++it) {
		std::map<std::string, int> joint_names;
		for (auto it2 = it->second.begin(); it2 != it->second.end(); ++it2) {
			if (it2->second["analog_read_max"] == it2->second["analog_read_min"])
				continue;
			char side = it->first[0];
			if (side == 'l' || side == 'r') {
				std::string name = side + ("_" + it2->first);
				//ROS_WARN("%s", name.c_str());
				joint_names[name] = int(it2->second["pin"]);
			}
		}
		if (joint_names.size() > 0) if (!planner.setJoints(it->first, joint_names)) {
			ROS_FATAL_STREAM("Could not set joints in " << it->first);
			return -1;
		}
	}

	ROS_INFO("Joints set");

	while (ros::ok()) {
		planner.rosCall();
		ros::spinOnce();
	}
	ros::shutdown();

	return 0;
}

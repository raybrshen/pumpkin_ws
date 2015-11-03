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

	/*!
	 * \brief This class is for planning the transitions between movements in a scene.
	 *
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
		/*!
		 * \brief Constructor. It loads the robot model, the scene (virtual world), the PlanningPipeline...
		 * and run the server.
		 */
		PumpkinPlanner() {
			//Loads the robot model
			robot_model_loader::RobotModelLoader pumpkinModelLoader("robot_description");
			_model = pumpkinModelLoader.getModel();

			//Create the planning scene
			_planningScene.reset(new planning_scene::PlanningScene(_model));
			//std::string plannerName;

			//Sets the planner pipeline, and deactivate the messages published by it, to make the system simple.
			_plannerPipeline.reset(new planning_pipeline::PlanningPipeline(_model, _nh, "planning_plugin", "request_adapters"));
			_plannerPipeline->displayComputedMotionPlans(false);
			_plannerPipeline->checkSolutionPaths(false);
			_plannerPipeline->publishReceivedRequests(false);

			ROS_INFO("Model and scene loaded.");
			/*
			 * Below here was the version using the PlannerManager, but the Pipeline is more complete (and ease to use)
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

			//Set the callback to the service.
			//This works.
			_snh.setCallbackQueue(&_serverQueue);

			_server = _snh.advertiseService("/pumpkin/planner", &PumpkinPlanner::plan, this);
			ROS_INFO("Planner initialized.");
		}

		/*!
		 * \brief Destructor. Just stops the server.
		 */
		~PumpkinPlanner() {
			_server.shutdown();
		}

		/*!
		 * \brief This method if for load a joint model group to plan (based on the already calibrated robot joints).
		 *
		 * It also sets up a index vector, for knowing the indexes of the joints that come from the "playback_action".
		 * For instance, if the joint X is read in the PIN 12, and it appears as the third joint, the value of index "2
		 * + <number of joints of other already inserted groups>" is 12.
		 * This is because the index may vary because the vector is about all joint groups.
		 *
		 * \param group_name    The name of the joint group
		 * \param joints        The mapping from each joint_name and the related Arduino PIN
		 * /returns False only if no joint is found in the group. (True otherwise)
		 */
		bool setJoints(const std::string &group_name, const std::map<std::string, int> &joints);

		/*!
		 * \brief Callback function that is called each time the service is required.
		 *
		 * \param req   Service request
		 * \param res   Service response
		 * \return True if the planning was a success. (False otherwise)
		 */
		bool plan(pmsg::PlannerRequest &req, pmsg::PlannerResponse &res);

		/*!
		 * \brief This function should be called in the main loop.
		 *
		 * Since it's using two callbacks (one for the callback and other for the service).
		 * I don't know if it's really needed, but it wasn't working, but it is now.
		 */
		inline void rosCall() { _serverQueue.callAvailable(); }
	};

	bool PumpkinPlanner::plan(pmsg::PlannerRequest &req, pmsg::PlannerResponse &res) {

		ROS_INFO("Calling planner");

		planning_interface::MotionPlanRequest planReq;
		planning_interface::MotionPlanResponse planRes;
		moveit_msgs::RobotTrajectory resMsg;

		//now is a robot state related to the planning scene. So, modifing this state will modify the robot in the scene.
		robot_state::RobotState &now = _planningScene->getCurrentStateNonConst();

		//That after is the final robot state, that will be used to build the goal constraints.
		robot_state::RobotState after(_model);
		int i = 0;
		//For each group...
		for (auto it = _joint_groups.begin(); it != _joint_groups.end(); ++it) {
			//planReq.goal_constraints.clear();
			//These vectors will handle the joints positions, in the order they're seen in the robot model
			//(and also the robot state)
			std::vector<double> joint_now, joint_after;
			//Fill the first vector with the positions (so we have the positions... ok)
			(*it)->getVariableDefaultPositions(joint_now);
			/* Uncomment this to see the positions. (LOOK FOR THE SIZE OF THE VECTOR TOO)
			for (auto it2 = joint_now.begin(); it2 != joint_now.end(); ++it2)
				std::cout << *it2 << ' ';
			std::cout << std::endl;
			*/

			//Copy the joint_after with the joint_now positions
			//This will ensure that both joints are in the same positions for now
			joint_after = joint_now;
			/*
			 * Change the positions that we have an Arduino PIN associated
			 * It means that joint positions that we receive from the "playback_action"
			 * We will leave the joints that we don't receive, so (in theory) the joints that are not read
			 * will be still the same
			 */
			for (int j = 0; j != joint_now.size(); ++j) {
				int index = _indexes[i++];
				if (index == -1)
					continue;
				joint_now[j] = req.initial_positions[index];
				joint_after[j] = req.final_positions[index];
			}
			/* Uncomment this to see the final positions
			for (auto it2 = joint_after.begin(); it2 != joint_after.end(); ++it2)
				std::cout << *it2 << ' ';
			std::cout << std::endl;
			*/

			/* Uncomment this to see the joint limits. Use it to check bounds calibration.
			for (auto it2 = (*it)->getActiveJointModelsBounds().begin();
			     it2 != (*it)->getActiveJointModelsBounds().end(); ++it2) {
				if ((*it2)->at(0).position_bounded_)
					std::cout << "Min position: " << (*it2)->at(0).min_position_ << ", max position: " <<
					(*it2)->at(0).max_position_ << std::endl;
				else
					std::cout << "Joint not bounded" << std::endl;
			}
			*/

			//Set the robot states
			now.setJointGroupPositions(*it, joint_now);
			after.setJointGroupPositions(*it, joint_after);
		}

		//I put that the planner HAVE TO plan in less than 1s.
		planReq.allowed_planning_time = 1.0;
		//Uncomment this to scale the robot speed. For now, we are not using the speed get from the planner, so it' useless.
		//planReq.max_velocity_scaling_factor = 0.1;

		/*
		 * Now, for the planning. In a strange way. The planner only plans each joint group per time.
		 */
		for (auto it = _joint_groups.begin(); it != _joint_groups.end(); ++it) {
			//Sets the requirement
			planReq.group_name = (*it)->getName();
			planReq.goal_constraints.clear();   //Clear this for not repeat constraints

			//Set the constraints with a helper function, just use the final robot position and the joint group
			planReq.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(after, *it));

			//Solve
			_plannerPipeline->generatePlan(_planningScene, planReq, planRes);
			//If not solved, print some info.
			if (planRes.error_code_.val != planRes.error_code_.SUCCESS) {
				ROS_ERROR("Could not get the planned trajectory. Error: %d", planRes.error_code_.val);
				return false;
			} else {
				ROS_INFO("Planned ok.");
			}

			//Set the robot state to the last planning position (as an intermediate state)
			_planningScene->setCurrentState(planRes.trajectory_->getLastWayPoint());
			//planRes.getMessage(resMsg);
			//ROS_INFO("Planned %d positions.", (int) planRes.trajectory_->getWayPointCount());    //<-- get the number os positions

			//Push the planned trajectory to the robot
			planRes.trajectory_->getRobotTrajectoryMsg(resMsg);
			res.joint_trajectory.emplace_back(resMsg.joint_trajectory);
		}
		ROS_INFO("Planning complete.");
		return true;
	}

	bool PumpkinPlanner::setJoints(const std::string &group_name, const std::map<std::string, int> &joints) {
		//We have a brand new joint model group, yeah!
		_joint_groups.push_back(_model->getJointModelGroup(group_name));
		bool success = false;
		/*
		 * Now, pay atention here!!!
		 * We have to get in wich order the joints are disposed in the group, so we can make the indexes translation
		 * If, for instance, the first joint that we get from the group is associated with the PIN 3 in Arduino
		 * so, we should put the number 3 in the first index of the vector.
		 * But, if we don't have a PIN related to the joint. Let us put -1, so we know that we will skip that joint later.
		 * CHECK THIS AFTER, IF YOU FACE PROBLEMS LATER:
		 * There are two functions that give the joints in each positions. But one of them give from the very base joint
		 * to the final joint of the group. And another just the joints that are variable. I used that second... it should
		 * work IF the joints are simple. From the time that we have a composite joint (I really don't know, but i read that
		 * not ever each variable is associated with each joint), we face problems.
		 */
		const std::vector<std::string> &ordered_names = _joint_groups.back()->getVariableNames();
		for (std::vector<std::string>::const_iterator it = ordered_names.begin(); it != ordered_names.end(); ++it) {
			auto it2 = joints.find(*it);
			if (it2 == joints.end()) {
				ROS_WARN("Coud not find %s for %s part", it->c_str(), group_name.c_str());
				_indexes.push_back(-1);
			} else {
				ROS_WARN("Inserted joint %s at position %d", it->c_str(), it2->second);
				_indexes.push_back(it2->second);
				success = true;
			}
		}
		return success;
	}

}

/*!
 * \brief This node runs the Pumpkin Planner ROS Service
 *
 * This service runs a MoveIt planner and plans a trajectory between tho points
 * (identified as joint positions).
 * This is intended to be used with "playback_action", wich needs a trajectory between the
 * end of a movement and the begining of another.
 */
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

	ROS_DEBUG("Joints set");

	//Main loop. Let it go.
	while (ros::ok()) {
		planner.rosCall();
		ros::spinOnce();
	}

	//Bye ROS
	ros::shutdown();

	return 0;
}

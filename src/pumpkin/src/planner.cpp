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

namespace pmsg = pumpkin_messages;

class PumpkinPlanner {
	ros::ServiceServer _server;
	ros::NodeHandle _nh;

	robot_model::RobotModelPtr _model;
	planning_scene::PlanningScenePtr _planningScene;
	planning_interface::PlannerManagerPtr _planner;
	std::vector<robot_state::JointModelGroup *> _joint_groups;

	std::vector<int> _indexes;

public:
	PumpkinPlanner() {
		robot_model_loader::RobotModelLoader pumpkinModelLoader("robot_description");
		_model = pumpkinModelLoader.getModel();

		_planningScene.reset(new planning_scene::PlanningScene(_model));
		std::string plannerName;

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

		_server = _nh.advertiseService<PumpkinPlanner, pmsg::PlannerRequest, pmsg::PlannerResponse>("planner", &PumpkinPlanner::plan, this);

	}

	bool setJoints(const std::string & group_name, const std::map<std::string, int> & joints ) {
		_joint_groups.push_back(_model->getJointModelGroup(group_name));
		const std::vector<std::string> &ordered_names = _joint_groups.back()->getJointModelNames();
		for (std::vector<std::string>::const_iterator it = ordered_names.begin(); it != ordered_names.end(); ++it) {
			_indexes.push_back(joints.at(*it));
		}
	}

	bool plan(pmsg::PlannerRequest& req, pmsg::PlannerResponse& res) {

		planning_interface::MotionPlanRequest planReq;
		planning_interface::MotionPlanResponse planRes;

		robot_state::RobotState & now = _planningScene->getCurrentStateNonConst();
		robot_state::RobotState after(_model);
		int i = 0;
		for (auto it = _joint_groups.begin(); it != _joint_groups.end(); ++it) {
			std::vector<double> joint_now, joint_after;
			joint_now.resize((*it)->getVariableCount());
			joint_after.resize(joint_now.size());
			for (int j = 0; j != joint_now.size(); ++j) {
				joint_now[j] = req.initial_positions[_indexes[i]];
				joint_after[j] = req.final_positions[_indexes[i++]];
			}
			now.setJointGroupPositions(*it, joint_now);
			after.setJointGroupPositions(*it, joint_after);
			planReq.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(after, *it));
		}

		planning_interface::PlanningContextPtr context = _planner->getPlanningContext(_planningScene, planReq, planRes.error_code_);
		context->solve(planRes);
		moveit_msgs::MotionPlanResponse resMsg;
		planRes.getMessage(resMsg);
		res.joint_trajectory = resMsg.trajectory.joint_trajectory;
		res.error_codes = planRes.error_code_;
	}
};

int main (int argc, char *argv[]) {
	ros::init(argc, argv, "pumpkin_planner");
	ros::start();

	ros::Rate loop(1000);
	while (!ros::param::has("pumpkin/config")) {
		loop.sleep();
	}

	PumpkinPlanner planner;


	XmlRpc::XmlRpcValue config;
	ros::param::get("/pumpkin/config/arduino", config);

	for (auto it = config.begin(); it != config.end(); ++it) {
		std::map<std::string, int> joint_names;
		for (auto it2 = it->second.begin(); it2 != it->second.end(); ++it2) {
			char side = it->first[0];
			if (side == 'l' || side == 'r') {
				joint_names[side + ("_"+it2->first)] = int(it2->second["pin"]);
			}
			if (!planner.setJoints(it->first, joint_names)) {
				ROS_FATAL_STREAM("Could not set joints in" << it->first);
				return -1;
			}
		}
	}

	double rate;
	ros::param::get("/pumpkin/config/ros_rate", rate);
	loop = ros::Rate(rate);

	while (ros::ok()) {
		ros::spinOnce();
		loop.sleep();
	}

	ros::shutdown();

	return 0;
}

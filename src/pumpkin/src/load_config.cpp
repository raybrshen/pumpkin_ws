#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

int main(int argc, char *argv[]) {
	//Init ROS Node
	ros::init(argc, argv, "load_config");

	//Seek file and load it
	std::string config_file;
	ros::Rate loop(1000);

	while (!ros::param::has("~config_file"))
		loop.sleep();

	ros::param::get("~config_file", config_file);

	YAML::Node pumpkin_config = YAML::LoadFile(config_file);
	ROS_INFO("Opening config file: %s", config_file.c_str());
	//Iterates for group part
	for (YAML::const_iterator it_cluster = pumpkin_config.begin(); it_cluster != pumpkin_config.end(); ++it_cluster)
	{
		const std::string &cluster = it_cluster->first.as<std::string>();
		if (cluster == "ros_rate") {
			ros::param::set("/pumpkin/config/ros_rate", it_cluster->second.as<double>());
			continue;
		}
		//Iterate through parts
		for (YAML::const_iterator it_part = it_cluster->second.begin(); it_part != it_cluster->second.end(); ++it_part)
		{
			const std::string &part = it_part->first.as<std::string>();
			//Iterate through devices
			for (YAML::const_iterator it_dev = it_part->second.begin(); it_dev != it_part->second.end(); ++it_dev)
			{
				const std::string &dev = it_dev->first.as<std::string>();
				//Iterare through properties
				for (YAML::const_iterator it_prop = it_dev->second.begin(); it_prop != it_dev->second.end(); ++it_prop)
				{
					std::ostringstream ros_param_name;
					ros_param_name << "/pumpkin/config/" << dev << '/' << cluster << '/' << part
							<< '/' << it_prop->first.as<std::string>();
					ros::param::set(ros_param_name.str().c_str(),it_prop->second.as<uint16_t>());
					//ROS_INFO("Creating param [%s] with value: %d", ros_param_name.str().c_str(), it_prop->second.as<uint16_t>());
				}
			}
		}
	}

	return 0;
}

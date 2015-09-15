#include "ros/ros.h"
#include "yaml-cpp/yaml.h"
#include <sstream>

int main(int argc, char *argv[]) {
	//Init ROS Node
	ros::init(argc, argv, "load_config");

	//Seek file and load it
	std::string config_file;
	if (argc >= 2)
		config_file = argv[1];
	else {
	    printf("Usage: rosrun pumpkin load_config <config_file>\n");
    	ROS_ERROR("Failed to parse input files");
		exit(-1);
	}

	YAML::Node pumpkin_config = YAML::LoadFile(config file);
	//Iterates for group part
	for (YAML::const_iterator it_cluster = pumpkin_config.begin(); it != pumpkin_config.end(); ++it)
	{
		const std::string &cluster = it_cluster->first.as<std::string>();
		//Iterate through parts
		for (YAML::const_iterator it_part = it_cluster->second.begin(); it_part != it_cluster->second.end(); ++it_part)
		{
			const std::string &part = it_part->first.as<std::string>();
			//Iterate through devices
			for (YAML::const_iterator it_dev = it_part->second.begin(); it-dev != it_part->second.end(); ++it_dev)
			{
				const std::string &dev = it_dev->first.as<std::string>();
				//Iterare through properties
				for (YAML::const_iterator it_prop = it_dev->second.begin(); it_prop != it_dev->second.end(); ++it_dev)
				{
					ostringstream ros_param_name;
					ros_param_name << "/pumpkin/config/" << dev << '/' << cluster << '/' << part
							<< '/' << it_prop->first.as<std::string>();
					ros::param::setParam(ros_param_name.str().c_str(),it_prop->second.as<uint16_t>());
				}
			}
		}
	}

	return 0;
}

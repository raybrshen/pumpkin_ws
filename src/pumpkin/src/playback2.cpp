//
// Created by rafaelpaiva on 24/09/15.
//

#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

int main (int argc, char *argv[]) {

	std::string file;

	if (argc > 1) {
		file = argv[1];
	}



	ros::init(argc, argv, "playback2");

	return 0;
}

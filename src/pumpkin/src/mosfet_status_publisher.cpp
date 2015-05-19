#include "ros/ros.h"
#include "std_msgs/Bool.h"

int main(int argc, char **argv) {

	ros::init(argc, argv, "mosfet_status_publisher");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("mosfet_status", 1000);
	ros::Rate loop_rate(5);

	bool status;
	char* status_char;

	if (argc <= 1) {
		status = false;
	} else {
		status_char = argv[1];
		std::stringstream ss;
		ss << status_char;
		ss >> std::boolalpha >> status;
	}

	int count = 0;
	std_msgs::Bool mosfet_status;
	while (ros::ok()) {
		mosfet_status.data = status;
		chatter_pub.publish(mosfet_status);

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}

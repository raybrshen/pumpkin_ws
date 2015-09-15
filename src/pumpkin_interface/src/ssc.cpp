#include "simple_serial.h"
#include "pumpkin_interface/SSCMove.h"
#include "pumpkin_interface/SSCMoveCommand.h"
#include <stdlib.h>
#include <sstream>

#include <ros/ros.h>

SimpleSerial *ssc = NULL;
std::string port;

bool setupSSC();

/**
 * Service function. Receive a group of arm commands and send it to SSC.
 * /param &req request
 * /param &res response
 */
bool moveSSC(pumpkin_interface::SSCMoveCommand::Request &req, pumpkin_interface::SSCMoveCommand::Response &res) {
	char serial_in = '+';
	ros::Duration d(0.01);
	while (serial_in == '+') {
		ssc->writeString(std::string("Q\r"));
		serial_in = ssc->readChar();
		d.sleep();
	}

	if (serial_in != '.')
		if (!setupSSC())
			return false;

	if (req.list.size() != 0) {
		std::stringstream comm_mount;
		int channel, pulse, speed, time;
		for (int i = 0; i < req.list.size(); i++) {
			channel = req.list[i].channel;
			pulse = req.list[i].pulse;
			speed = req.list[i].speed;
			//TODO Configure the limits for each channel
			if (pulse < 500 || pulse > 2500)
				continue;
			comm_mount << '#' << channel << " P" << pulse;
			if (time > 0)
				comm_mount << " T" << speed;
			comm_mount << ' ';
		}
		if (req.time > 0)
			comm_mount << " T" << time;
		comm_mount << '\r';
		ssc->writeString(comm_mount.str());
	}
	return true;
}

bool setupSSC() {
	FILE *pipe = popen(("setserial -g "+port+"*").c_str(), "r");
	char buffer[128];
	std::string console = "";
	while(!feof(pipe)) {
	if(fgets(buffer, 128, pipe) != NULL)
		console += buffer;
	}

	pclose(pipe);

	//Separate ports into a vector of strings
	std::vector<std::string> port_list;
	std::string buf;
	for (std::string::iterator it = console.begin(); it != console.end(); ++it) 
	{
		if (*it != '\n')
			buf += *it;
		else
		{
			size_t pos;
			if ((pos = buf.find("UART")) != std::string::npos)	//If found UART
			{
				port_list.push_back(buf.substr(0, pos-2));
			}
			buf.clear();
		}
	}

	if (!port_list.size()) {
		ROS_INFO("There is not any serial device in any port of type: %s", port.c_str());
		return false;
	}

	for (std::vector<std::string>::iterator it = port_list.begin(); it != port_list.end(); ++it)
	{
		std::string ver;
		ssc = new SimpleSerial(*it, 115200);
		//ROS_INFO("Start serial on port: %s", (*it).c_str());
		ssc->writeString(std::string("VER\r"));
		ver = ssc->readLine();
		if (ver.find("SSC") != std::string::npos) {
			ros::param::set("/pumpkin/ssc_serial_port", (*it).c_str());
			ROS_INFO("SSC found on port: %s", (*it).c_str());
			return true;
		}
		delete ssc;
	}
	
	return false;
}

int main (int argc, char *argv[]) {
	// Initialize ROS.
	ros::init(argc, argv, "setup_ssc");
	ros::param::param<std::string>("~port", port, "/dev/ttyUSB");
	ROS_INFO("Looking for ssc on ports of type: %s", port.c_str());
	if (!setupSSC()) {
		ROS_INFO("SSC not found! Check your connections and try again.");
		return -1;
	}

	ros::NodeHandle nh;

	ros::ServiceServer srv = nh.advertiseService("move_ssc", moveSSC);

	ros::spin();

	return 0;
}

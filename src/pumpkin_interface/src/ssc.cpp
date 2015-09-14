#include "simple_serial.h"
#include "pumpkin_interface/SSCMove.h"
#include "pumpkin_interface/SSCMoveCommand.h"
#include <stdlib.h>
#include <stringstream>

#include <ros/ros.h>

SimpleSerial *ssc = NULL;
std::string port;

void moveSSC(pumpkin_interface::SSCMoveCommand::Request &req, pumpkin_interface::SSCMoveCommand::Response &res) {
	ssc->write(std::string("VER\r"));	
	std::string serial_in = ssc->readLine();

	if (ver.find("SSC") != std::string::npos)
		setupSSC();

	if (req.list.size() != 0) {
		stringstream comm_mount;
		int channel, pulse, time;
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
	}
}

void setupSSC() {
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
			break;
		}
		delete ssc;
	}
	
	if (!ssc)
		abort();
}

int main (int argc, char *argv[]) {
	// Initialize ROS.
	ros::init(argc, argv, "ssc_setup");
	ros::param::param<std::string>("~port", port, "/dev/ttyUSB");

	ros::spin();

	return 0;
}

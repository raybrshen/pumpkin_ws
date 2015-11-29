#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>

#include "rosserial_server/serial_session.h"

/*!
 * Arduino node
 * ============
 *
 * This node is a modification of the code of the *rosserial_server*.
 * It will look for any active serial port of an determined standard. (The default is _/dev/ttyACM_)
 * After that, it will try to run a rosserial communication with the device connected to this serial port.
 * If it respond, it is a programmed Arduino.
 *
 * Each Arduino connected will be ran into a specific thread. (1 Thread for 1 Arduino)
 */

int main(int argc, char* argv[])
{
	// Initialize ROS.
	ros::init(argc, argv, "arduino_setup");
	std::string port;
	ros::param::param<std::string>("~port", port, "/dev/ttyACM");
	int baud;
	ros::param::param<int>("~baud", baud, 57600);

	//Identifies the port via a system pipe
	FILE *pipe = popen(("setserial -g "+port+"*").c_str(), "r");
	std::string console = "";
	char buffer[128];
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

	//Run boost::asio io service in a background thread.
	boost::asio::io_service io_service_vector[port_list.size()];
	for (int i = 0; i < port_list.size(); ++i) {
		new rosserial_server::SerialSession(io_service_vector[i], port_list[i], baud);
		boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_vector[i]));
	}

	/*
	boost::asio::io_service io_service;
	new rosserial_server::SerialSession(io_service, port_list[0], baud);
	boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
	*/

	ros::spin();

	return 0;
}

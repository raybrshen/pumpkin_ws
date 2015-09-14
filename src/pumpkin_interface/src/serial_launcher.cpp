#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>

#include "rosserial_server/serial_session.h"
#include "simple_serial.h"

SimpleSerial *ssc = 0;
std::string buf;
std::vector<std::string> port_list;
FILE *pipe;

void setupSSC() {
	pipe = popen("setserial -g /dev/ttyUSB*", "r");
	console = "";
	while(!feof(pipe)) {
    	if(fgets(buffer, 128, pipe) != NULL)
    		console += buffer;
    }

	//Separate ports into a vector of strings
	
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
		ssc.writeString(std::string("VER\r"));
		ver = ssc.readLine();
		if (ver.find("SSC") != std::string::npos) {
			ros::param::set("/pumpkin/ssc_serial_port", (*it).c_str());
			ROS_INFO("SSC found on port: %s", (*it).c_str());
			break;
		}
		delete ssc;
	}

	pclose(pipe);
}

int main(int argc, char* argv[])
{
	// Initialize ROS.
	ros::init(argc, argv, "pumpkin_rosserial_launcher");
	std::string port;
	//ros::param::param<std::string>("~port", port, "/dev/ttyACM0");
	int baud;
	ros::param::param<int>("~baud", baud, 57600);

	//Identifies the port via a system pipe
	pipe = popen("setserial -g /dev/ttyACM*", "r");
	char buffer[128];
    std::string console = "";
    while(!feof(pipe)) {
    	if(fgets(buffer, 128, pipe) != NULL)
    		console += buffer;
    }

	//Separate ports into a vector of strings
	
	std::string buf;
	std::vector<std::string> port_list;
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

	pclose(pipe);

	setupSSC();

	/*
	boost::asio::io_service io_service;
	new rosserial_server::SerialSession(io_service, port_list[0], baud);
	boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
	*/

	ros::spin();

	if (ssc)
		delete ssc;
	return 0;
}

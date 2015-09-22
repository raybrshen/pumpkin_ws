#include "simple_serial.h"
#include "pumpkin_interface/SSCMove.h"
#include "pumpkin_interface/SSCMoveCommand.h"
#include <XmlRpcValue.h>

#include <ros/ros.h>
#include "serial/serial.h"

//SimpleSerial *ssc = NULL;
serial::Serial *ssc = NULL;
std::string port;
bool debug_comm = false;

bool setupSSC();

/**
 * Service function. Receive a group of arm commands and send it to SSC.
 * /param &req request
 * /param &res response
 */

int min_pulse[32];
int max_pulse[32];
int rest_pulse[32];

bool moveSSC(pumpkin_interface::SSCMoveCommand::Request &req, pumpkin_interface::SSCMoveCommand::Response &res) {
	unsigned char serial_in = '+';
	ros::Duration d(0.01);
	try {
		while (serial_in == '+') {
			ssc->write("Q\r");
			ssc->read(&serial_in, 1);
			ssc->flushInput();
			d.sleep();
		}
	} catch (serial::PortNotOpenedException) {
		delete ssc;
		if (!setupSSC())
			return false;
	}

	if (serial_in != '.')
		if (!setupSSC())
			return false;

	try {
		if (req.list.size() != 0) {
			std::stringstream comm_mount;
			int channel, pulse, speed;
			for (int i = 0; i < req.list.size(); i++) {
				channel = req.list[i].channel;
				pulse = req.list[i].pulse;
				speed = req.list[i].speed;
				if (pulse == -1)
					pulse = rest_pulse[channel];
				if (pulse < min_pulse[channel] || pulse > max_pulse[channel])
					continue;
				comm_mount << '#' << channel << " P " << pulse;
				if (speed > 0)
					comm_mount << " S " << speed;
				comm_mount << ' ';
			}
			if (req.time > 0)
				comm_mount << "T " << req.time;
			comm_mount << '\r';
			//Set where to send command (if test string to a terminal or to send to pumpkin)
			if (debug_comm)
				ROS_INFO("Command: %s", comm_mount.str().c_str());
			else
				ssc->write(comm_mount.str());
		}
	} catch (serial::IOException e) {
		ROS_ERROR("IOException ocurred. Error: %s.", e.what());
		return false;
	} catch (serial::PortNotOpenedException) {
		ROS_ERROR("The serial connection was closed during the transmission. Check connection status.");
		return false;
	} catch (serial::SerialException e) {
		ROS_ERROR("Error in serial connection. Error message: %s.", e.what());
		return false;
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

	std::string ver;
	for (std::vector<std::string>::iterator it = port_list.begin(); it != port_list.end(); ++it)
	{
		try {
			ssc = new serial::Serial(*it, 115200, serial::Timeout::simpleTimeout(1000));
			//ssc = new SimpleSerial(*it, 115200);
			//ROS_INFO("Start serial on port: %s", (*it).c_str());
			ssc->write("VER\r");
			ssc->readline(ver, 65536, "\r");
			if (ver.find("SSC") != std::string::npos) {
				ros::param::set("/pumpkin/ssc_serial_port", (*it).c_str());
				ROS_INFO("SSC found on port: %s", (*it).c_str());
				return true;
			}
		} catch (serial::PortNotOpenedException) {
			if (ssc)
				delete ssc;
			continue;
		} catch (serial::IOException) {
			ROS_WARN("Error trying to communicate with the port %s.", (*it).c_str());
				if (ssc)
			delete ssc;
			continue;
		} catch (serial::SerialException e) {
			ROS_WARN("Error trying to read or write on serial. Error message: %s.", e.what());
			delete ssc;
			continue;
		}
		delete ssc;
	}
	
	return false;
}

int main (int argc, char *argv[]) {
	// Initialize ROS.
	ros::init(argc, argv, "setup_ssc");
	ros::param::param<std::string>("~port", port, "/dev/ttyUSB");

	if (argc > 1) {
		if (!std::string(argv[1]).compare("debug")) {
			debug_comm = true;
			ROS_WARN("You are running \"setup_ssc\" in debug mode. All commands will be shown here.");
			ROS_WARN("To send commands to pumpkin, please, run without debug argumment.");
		} else {
			ROS_INFO("You can run this node in debug mode to show commands here instead of sending them to pumpkin.");
			ROS_INFO("To do so, just run this node with debug argumment, like: \"rosrun pumpkin_interface setup_ssc debug\".");
		}
	}

	ROS_INFO("Looking for ssc on ports of type: %s", port.c_str());
	if (!setupSSC()) {
		ROS_INFO("SSC not found! Check your connections and try again.");
		return -1;
	}

	XmlRpc::XmlRpcValue config;

	ros::param::get("/pumpkin/config/ssc", config);

	typedef XmlRpc::XmlRpcValue::iterator xml_iterator;
	for (xml_iterator block_it = config.begin(); block_it != config.end(); ++block_it) {
		//ROS_INFO("%s", block_it->first.c_str());
		for (xml_iterator part_it = block_it->second.begin(); part_it != block_it->second.end(); ++part_it) {
			int pin = int(part_it->second["pin"]);
			min_pulse[pin] = int(part_it->second["pulse_min"]);
			max_pulse[pin] = int(part_it->second["pulse_max"]);
			rest_pulse[pin] = int(part_it->second["pulse_rest"]);
			//ROS_INFO("%d", pin);
		}
	}

	ros::NodeHandle nh;

	ros::ServiceServer srv = nh.advertiseService("move_ssc", moveSSC);

	ros::spin();

	return 0;
}

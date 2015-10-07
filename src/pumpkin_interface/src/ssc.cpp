//#include "simple_serial.h"
#include "pumpkin_messages/SSCMove.h"
#include "pumpkin_messages/SSCMoveList.h"
#include "pumpkin_messages/SSCMoveCommand.h"
#include <XmlRpcValue.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "serial/serial.h"

//SimpleSerial *ssc = NULL;
serial::Serial *ssc = nullptr;
std::string port;
bool debug_flag = false;

bool setupSSC();

/**
 * Service function. Receive a group of arm commands and send it to SSC.
 * /param &req request
 * /param &res response
 */

int min_pulse[32];
int max_pulse[32];
//int rest_pulse[32];

void generate_move(const pumpkin_messages::SSCMoveList & move) {
	if (move.list.size() > 0) {
		std::stringstream comm_mount;
		for (auto it : move.list) {
			if (it.pulse != 0 && (it.pulse < min_pulse[it.channel] || it.pulse > max_pulse[it.channel])) {
				ROS_WARN("Received a move command to move the channel %d outbound. Review command or robot calibration.",
						it.channel);
				continue;
			}
			comm_mount << "# " << int(it.channel) << " P " << it.pulse;
			if (it.speed > 0)
				comm_mount << " S " << it.speed;
			comm_mount << ' ';
		}
		if (move.time > 0)
			comm_mount << "T " << move.time;

		comm_mount << '\r';

		if (debug_flag)
			ROS_INFO("Command: %s", comm_mount.str().c_str());
		else
			ssc->write(comm_mount.str());
	} else {
		std::stringstream off_comm;
		for (int i = 0; i < 32; ++i)
			off_comm << '#' << i << " P 0 ";
		off_comm << '\r';

		if (debug_flag)
			ROS_INFO("Command: %s", off_comm.str().c_str());
		else
			ssc->write(off_comm.str());
	}
}

bool moveSSC(pumpkin_messages::SSCMoveCommand::Request &req, pumpkin_messages::SSCMoveCommand::Response &res) {
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
		if (!setupSSC())
			return false;
	} catch (serial::SerialException) {
		ROS_ERROR("Error trying to read SSC status. Trying to connect again.");
		if (!setupSSC())
			return false;
	}

	if (serial_in != '.')
		if (!setupSSC())
			return false;

	try {
		generate_move(req.move);
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
	if (ssc)
		delete ssc;
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
		ROS_INFO("There is no any serial device in any port of type: %s", port.c_str());
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
	ros::NodeHandle service_handle, topic_handle;

	if (argc > 1) {
		if (!std::string(argv[1]).compare("debug")) {
			debug_flag = true;
			ROS_WARN("You are running \"setup_ssc\" in debug mode. All commands will be shown here.");
			ROS_WARN("To send commands to pumpkin, please, run without debug argumment.");
		} else {
			ROS_INFO("You can run this node in debug mode to show commands here instead of sending them to pumpkin.");
			ROS_INFO("To do so, just run this node with debug argumment, like: \"rosrun pumpkin_interface setup_ssc debug\".");
		}
	}

	ROS_INFO("Looking for ssc on ports of type: %s", port.c_str());
	if (!setupSSC()) {
		ROS_FATAL("SSC not found! Check your connections and try again.");
		return -1;
	}

	ros::Rate loop(1000);
	while(!ros::param::has("/pumpkin/config")) {
		loop.sleep();
		ROS_WARN("Cannot find robot configuration. Please run \"load_config\".");
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
			//rest_pulse[pin] = int(part_it->second["pulse_rest"]);
			//ROS_INFO("%d", pin);
		}
	}

	ros::CallbackQueue service_queue, topic_queue;

	service_handle.setCallbackQueue(&service_queue);
	topic_handle.setCallbackQueue(&topic_queue);

	ros::ServiceServer srv = service_handle.advertiseService("move_ssc", moveSSC);
	ros::Subscriber subs = topic_handle.subscribe("move_ssc_topic", 1024, generate_move);

	double rate;
	ros::param::get("/pumpkin/config/ros_rate", rate);
	loop = ros::Rate(rate);

	while (ros::ok()) {

		if (service_queue.callOne(ros::WallDuration(0)) == ros::CallbackQueue::Called) {
			service_queue.callAvailable(ros::WallDuration(0));
			ROS_INFO("Service called.");
		} else {
			topic_queue.callAvailable(ros::WallDuration(0));
			//ROS_INFO("Topic called.");
		}
		loop.sleep();
	}

	return 0;
}

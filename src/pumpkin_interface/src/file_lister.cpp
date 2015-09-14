#include "ros/ros.h"
#include "pumpkin_interface/Files.h"
#include "pumpkin_interface/Serials.h"
#include "file_type_codes.h"
#include "std_msgs/String.h"
#include <string>
#include <vector>
#include <set>
#include <iostream>
#include <sstream>
#include <cstdio>
#include <sys/types.h>
#include <dirent.h>

bool getdir (std::string &dir, std::string &ftype, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        std::cout << "Error(" << errno << ") opening " << dir << std::endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        std::string s = dirp->d_name;
		if (s.size() <= ftype.size())
			continue;
        if (s.compare (s.size() - ftype.size(), ftype.size(), ftype) == 0) {
			std_msgs::String s2;
			s2.data = s;
            files.push_back(s);
		}
    }
    closedir(dp);
    return 0;
}

bool ListSerialPorts(pumpkin_interface::Serials::Request &req, pumpkin_interface::Serials::Response &res)
{
	FILE *pipe = popen("setserial -g /dev/ttyACM*", "r");
	char buffer[64];
    std::string console = "";
    while(!feof(pipe)) {
    	if(fgets(buffer, 64, pipe) != NULL)
    		console += buffer;
    }

	pclose(pipe);

	//Separate ports into a vector of strings
	
	std::string buf2;
	std::set<std::string> bag;
	for (std::string::iterator it = console.begin(); it != console.end(); ++it) 
	{
		if (*it != '\n')
			buf2 += *it;
		else
		{
			bag.insert(buf2);
			buf2.clear();
		}
	}

	for (std::set<std::string>::iterator it = bag.begin(); it != bag.end(); ++it)
	{
		size_t pos;
		if ((pos = (*it).find("UART")) == std::string::npos)	//If not found UART
			continue;

		const std::string &port = (*it).substr(0, pos-2);
		
		/* TODO - find a way to check communication via rosserial */
		//string comm = "rosrun rosserial_python serial_node.py ";
		//system((comm+(*it)).c_str());

		

		res.terminals.push_back(port);
	}

	return true;
}

bool ListFiles (pumpkin_interface::Files::Request &req, pumpkin_interface::Files::Response &res)
{
    //Get pumpkin base folder
    FILE *pipe = popen("rospack find pumpkin", "r");
    if (!pipe) return false;
    char buffer[128];
    std::string folder = "";
    std::string extension;
    while(!feof(pipe)) {
    	if(fgets(buffer, 128, pipe) != NULL)
    		folder += buffer;
    }
	//try to remove a possible \n
	folder = folder.substr(0, folder.size()-1);
    
    //It doesn't look like so Object Oriented, but...
    switch(req.filetype) {
        case _TYPE_CONF:
            folder += "/config";
            extension = ".yaml";
        break;
        case _TYPE_DESC:
            folder += "/description/robots";
            extension = ".URDF";
        break;
        case _TYPE_MOVE:
            folder += "/playback";
            extension = ".yaml";
        break;
        default:
            ROS_INFO("ERROR! Type code not identified!");
            return false;
    }

    //Here, we can, hereafter, make a rosout showing an error code
    if (getdir(folder, extension, res.filenames) != 0)
        return false;

    res.folder = folder;

    return true;
}

int main (int argc, char** argv)
{

    ros::init(argc, argv, "lister_server");
    ros::NodeHandle n;

    ros::ServiceServer files = n.advertiseService("file_lister", ListFiles);
	ros::ServiceServer ports = n.advertiseService("port_lister", ListSerialPorts);
    ROS_INFO("Ready.");
    ros::spin();

    return 0;
}

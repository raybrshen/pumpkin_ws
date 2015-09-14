#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pumpkin_interface/Files.h"
#include <cstdlib>
#include "file_type_codes.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "file_listener_sample");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<pumpkin_interface::Files>("file_lister");
    pumpkin_interface::Files srv;
    srv.request.filetype = _TYPE_CONF;
    if (client.call(srv))
    {
        ROS_INFO("Folder: %s", srv.response.folder.c_str());
		for (std::vector<std::string>::iterator it = srv.response.filenames.begin(); it != srv.response.filenames.end(); ++it) {
			ROS_INFO("%s", (*it).c_str());
		}
    }
    else
    {
        ROS_ERROR("Failed to call service file_lister");
        return 1;
    }

    return 0;
}

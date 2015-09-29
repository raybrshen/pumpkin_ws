#include "ros/ros.h"
#include "file_type.h"
#include "pumpkin_messages/Files.h"
#include "std_msgs/String.h"
#include <boost/filesystem.hpp>

using namespace pumpkin_messages;
namespace bfs = boost::filesystem;

std::string pumpkin_path;

void get_files_recursively(const bfs::path &dir, const std::string &ext, std::vector<FileList> &found) {
	FileList files;
	if (!bfs::exists(dir))
		return;
	for (bfs::directory_iterator it(dir), end_it; it != end_it; ++it) {
		if (bfs::is_directory(it->path())) {
			get_files_recursively(it->path(), ext, found);
		} else {
			const std::string &filename = it->path().leaf().string();
			if (filename.compare(filename.size() - ext.size(), ext.size(), ext) == 0)
				files.filenames.push_back(filename);
		}
	}
	if (files.filenames.size() > 0) {
		files.folder = dir.string();
		found.emplace_back(std::move(files));
	}
}

bool ListFiles (Files::Request &req, Files::Response &res)
{
	std::string folder = pumpkin_path;
	std::string extension;
    

    switch(FileType(req.type)) {
        case FileType::ConfigFile:
            folder += "/config";
            extension = ".yaml";
        break;
        case FileType::RobotDescription:
            folder += "_description/robots";
            extension = ".URDF";
        break;
        case FileType::PlaybackFile:
            folder += "/playback";
            extension = ".yaml";
        break;
	    case FileType::Error:
        default:
            ROS_ERROR("ERROR! Type code not identified!");
            return false;
    }

	get_files_recursively(bfs::path(folder), extension, res.files);

    return res.files.size() > 0;
}

int main (int argc, char** argv)
{

    ros::init(argc, argv, "lister_server");
    ros::NodeHandle n;

	char * env;

	if ((env = getenv("PUMPKIN_PATH")) == nullptr) {
		ROS_FATAL("Cannot find pumpkin path inside environment variables.");
	}

	pumpkin_path = env;

    ros::ServiceServer files = n.advertiseService("file_lister", ListFiles);
    ROS_INFO("Ready.");
    ros::spin();

    return 0;
}

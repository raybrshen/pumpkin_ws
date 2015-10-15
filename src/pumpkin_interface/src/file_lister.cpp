#include "ros/ros.h"
#include "file_type.h"
#include "pumpkin_messages/Files.h"
#include "pumpkin_messages/FilesHandler.h"
#include <boost/filesystem.hpp>

using namespace pumpkin_messages;
namespace bfs = boost::filesystem;


std::string pumpkin_path;

void get_files_recursively(const bfs::path &dir, const std::string &ext, std::vector<FileList> &found) {
	FileList files;
	std::vector<FileList> children;
	if (!bfs::exists(dir))
		return;
	if (!bfs::is_directory(dir))
		return;
	bfs::directory_iterator end_it;
	for (bfs::directory_iterator it(dir); it != end_it; ++it) {
		if (bfs::is_directory(it->path())) {
			get_files_recursively(it->path(), ext, children);
		} else {
			const std::string filename = it->path().leaf().string();
			//ROS_INFO("Filename: %s", filename.c_str());
			if (filename.size() > ext.size())
				if (filename.compare(filename.size() - ext.size(), ext.size(), ext) == 0) {
					files.filenames.push_back(filename);
				}
		}
	}
	files.folder = dir.leaf().string();
	files.parent_folder = 0;
	//ROS_INFO("I'm %s", files.folder.c_str());
	found.emplace_back(std::move(files));
	if (children.size()) {
		//ROS_INFO("I have %d children.", (int)children.size());
		int pos = found.size();
		std::for_each(children.begin(), children.end(), [&pos](FileList &x) {
				x.parent_folder += pos;
		});
		found.insert(found.end(), children.begin(), children.end());
	}
}

bool HandleFiles(FilesHandler::Request &req, FilesHandler::Response &res) {
	bfs::path path(req.folder);
	if (!bfs::is_directory(path))
		return false;
	switch (static_cast<FHST>(req.service))
	{
		case FHST::CreateFile:
			ROS_FATAL("Create files is not yet implemented.");
			return false;
		break;
		case FHST::CreateFolder:
			path /= req.filename;
			if (bfs::exists(path)) {
				ROS_WARN("Folder already exists");
				res.result = static_cast<uint8_t>(IOState::FileAlreadyExists);
			} else {
				if (bfs::create_directory(path))
					res.result = static_cast<uint8_t>(IOState::OK);
				else
					res.result = static_cast<uint8_t>(IOState::ErrorCreating);
			}
		break;
		case FHST::DeleteFile:
			path /= req.filename;
			if (!bfs::exists(path)) {
				ROS_WARN("File not exists");
				res.result = static_cast<uint8_t>(IOState::FileNotExists);
			} else if (bfs::is_directory(path)) {
				ROS_WARN("Do not pretend to remove a file, but pass me a folder.");
				res.result = static_cast<uint8_t>(IOState::BadFile);
			} else {
				if (bfs::remove(path))
					res.result = static_cast<uint8_t>(IOState::OK);
				else
					res.result = static_cast<uint8_t>(IOState::ErrorDeleting);
			}
		break;
		case FHST::DeleteFolder:
			if (!bfs::exists(path)) {
				ROS_WARN("Folder not exists");
				res.result = static_cast<uint8_t>(IOState::FileNotExists);
			} else if (!bfs::is_directory(path)) {
				ROS_WARN("Do not pretend to remove a direcory, but pass me a file.");
				res.result = static_cast<uint8_t>(IOState::BadFile);
			} else {
				if (bfs::remove(path))
					res.result = static_cast<uint8_t>(IOState::OK);
				else
					res.result = static_cast<uint8_t>(IOState::ErrorDeleting);
			}
		break;
		default:
			ROS_FATAL("Instruction not identified");
			return false;
	}
	return true;
}

bool ListFiles (Files::Request &req, Files::Response &res)
{
	std::string folder = pumpkin_path;
	std::string extension;
    

    switch(static_cast<FileType>(req.type)) {
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
            ROS_ERROR("ERROR! Type code not identified!");
            return false;
    }

	ROS_INFO("Folder: %s", folder.c_str());
	res.base_path = folder.substr(0, folder.rfind('/'));
	get_files_recursively(bfs::path(folder), extension, res.files);

    return res.files.size() > 0;
}

int main (int argc, char** argv)
{

    ros::init(argc, argv, "file_lister");
    ros::NodeHandle n;

	char * env = getenv("PUMPKIN_PATH");

	if (env != nullptr) {
		pumpkin_path = env;
	} else if (!ros::param::get("~pumpkin_path", pumpkin_path)) {
		ROS_FATAL("Could not get PUMPKIN_PATH.");
		return -1;
	}

	ros::ServiceServer files = n.advertiseService("file_lister", ListFiles);
	ros::ServiceServer handler = n.advertiseService("file_handler", HandleFiles);
    ROS_INFO("Ready.");
    ros::spin();

    return 0;
}

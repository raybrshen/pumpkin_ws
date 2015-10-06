//
// Created by rafaelpaiva on 05/10/15.
//

#include "PlaybackRecordWindow.h"

using namespace pumpkin_gui;

PlaybackRecordWindow::PlaybackRecordWindow(BaseObjectType *c_wrap,
                                           const Glib::RefPtr<Gtk::Builder> glade) : Gtk::Window(c_wrap),
                                                                                     _file_client(_nh.serviceClient("file_lister")),
                                                                                     _playback_client(_nh, "/playback_action", true),
                                                                                     _record_client(_nh, "/record_action", true)
{
	glade->get_widget("playback_time_elapsed", _playback_time_scale);
	glade->get_widget("play_button", _play_button);
	glade->get_widget("stop_play_button", _stop_play_button);
	glade->get_widget("rec_button", _rec_button);
	glade->get_widget("stop_rec_button", _stop_rec_button);
	glade->get_widget("playback_file_name", _playback_file_name);
	glade->get_widget("record_file_name", _record_file_name);
	glade->get_widget("rec_min_spinner", _rec_min_spin);
	glade->get_widget("rec_sec_spinner", _rec_sec_spin);

	if (!_playback_time_scale || !_play_button || !_stop_play_button || !_rec_button || !_stop_rec_button ||
			!_playback_file_name || !_record_file_name || !_rec_min_spin || !_rec_sec_spin) {
		ROS_FATAL("Could not create window!");
	}

	glade->get_widget("folder_tree", _folder_tree);
	glade->get_widget("file_list", _file_tree);
	_folder_store = Glib::RefPtr<Gtk::TreeStore>::cast_static(glade->get_object("folder_store"));

	if (!_folder_tree || !_file_tree || !_folder_store) {
		ROS_FATAL("Could not load tree list");
	}

	//Seek files
	fillFolder();

}

void PlaybackRecordWindow::onStartPlayback() {
	_playback_client
}

void PlaybackRecordWindow::fillFolder() {

	pumpkin_messages::Files msg;
	msg.request.type = static_cast<uint8_t>(pumpkin_messages::FileType::PlaybackFile);
	_file_client.call<pumpkin_messages::Files>(msg);

	_file_store_vector.reserve(msg.response.files.size());
	int parent_index;
	for(int i = 0; i < msg.response.files.size(); ++i) {
		//New List Store
		_file_store_vector.emplace_back(Gtk::ListStore::create(_file_model));
		std::for_each(std::begin(msg.response.files[i].filenames), std::end(msg.response.files[i].filenames), [](std::string &file){
			auto & row = *(_file_store_vector.back()->append());
			row[_file_model.name] = file;
		});
		//Seek parent folder
		if (msg.response.files[i].parent_folder == 0) {
			_folder_store->append()
		}
	}

	//Add parent folder

}

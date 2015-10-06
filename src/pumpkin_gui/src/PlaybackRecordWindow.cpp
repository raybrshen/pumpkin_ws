//
// Created by rafaelpaiva on 05/10/15.
//

#include "PlaybackRecordWindow.h"
#include <stack>

using namespace pumpkin_gui;

PlaybackRecordWindow::PlaybackRecordWindow(BaseObjectType *c_wrap,
                                           const Glib::RefPtr<Gtk::Builder> &glade) : Gtk::Window(c_wrap),
                                                                                     _file_client(_nh.serviceClient<pumpkin_messages::Files>("file_lister")),
                                                                                     _playback_client(_nh, "/playback_action", false),
                                                                                     _record_client(_nh, "/record_action", false)
{
	ROS_INFO("Start building the window");
	_builder = glade;
	_builder->get_widget("playback_time_elapsed", _playback_time_scale);
	_builder->get_widget("play_button", _play_button);
	_builder->get_widget("stop_play_button", _stop_play_button);
	_builder->get_widget("rec_button", _rec_button);
	_builder->get_widget("stop_rec_button", _stop_rec_button);
	_builder->get_widget("playback_file_name", _playback_file_name);
	_builder->get_widget("record_file_name", _record_file_name);
	_builder->get_widget("rec_min_spinner", _rec_min_spin);
	_builder->get_widget("rec_sec_spinner", _rec_sec_spin);

	if (!_playback_time_scale || !_play_button || !_stop_play_button || !_rec_button || !_stop_rec_button ||
			!_playback_file_name || !_record_file_name || !_rec_min_spin || !_rec_sec_spin) {
		ROS_FATAL("Could not create window!");
	} else {ROS_INFO("Ate aqui td bem");}

	_builder->get_widget("folder_tree", _folder_tree);
	_builder->get_widget("file_list", _file_tree);
	_folder_store = Glib::RefPtr<Gtk::TreeStore>::cast_static(glade->get_object("folder_store"));

	if (!_folder_tree || !_file_tree || !_folder_store) {
		ROS_FATAL("Could not load tree list");
	} else {ROS_INFO("arveres criadas");}

	//Seek files
	fillFolder();

	//Link signals
	_file_tree->signal_row_activated().connect(sigc::mem_fun(*this, &PlaybackRecordWindow::onSelectFolder));
}

PlaybackRecordWindow::~PlaybackRecordWindow() {}

void PlaybackRecordWindow::onSelectFolder(const Gtk::TreeModel::Path &path, Gtk::TreeViewColumn *column)
{
	Gtk::TreeModel::iterator it = _folder_store->get_iter(path);
	if (it) {
		_file_tree->set_model(_file_store_vector[(*it)[_folder_model.id]]);
	}
}

void PlaybackRecordWindow::onStartPlayback() {
}

void PlaybackRecordWindow::fillFolder() {

	pumpkin_messages::Files msg;
	std::stack<std::pair<uint8_t, Gtk::TreeModel::iterator> > hierarchy;
	msg.request.type = static_cast<uint8_t>(pumpkin_messages::FileType::PlaybackFile);
	if (!_file_client.call<pumpkin_messages::Files>(msg))
		ROS_FATAL("Could not find node \"file_lister\".");

	_file_store_vector.reserve(msg.response.files.size());
	for(int i = 0; i < msg.response.files.size(); ++i) {
		auto & ref = msg.response.files[i];
		//New List Store
		_file_store_vector.emplace_back(Gtk::ListStore::create(_file_model));
		for(auto it = std::begin(ref.filenames); it != std::end(ref.filenames); ++it)
		{
			auto & row = *(_file_store_vector.back()->append());
			row[_file_model.name] = *it;
		}
		Gtk::TreeModel::iterator it;
		if (ref.parent_folder == 0) {
			//Parent case (when i = 0, if it was on only one tree)
			it = _folder_store->append();
		} else {
			uint8_t parent = ref.parent_folder-1;
			while (parent != hierarchy.top().first)
				hierarchy.pop();
			it = _folder_store->append(hierarchy.top().second->children());
		}
		(*it)[_folder_model.name] = ref.folder;
		(*it)[_folder_model.id] = i;
		hierarchy.push(std::pair<uint8_t, Gtk::TreeModel::iterator>(i, it));
	}

	_base_folder = msg.response.base_path;

	//Add parent folder

}

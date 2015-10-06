//
// Created by rafaelpaiva on 05/10/15.
//

#ifndef PROJECT_PLAYBACKRECORDWINDOW_H
#define PROJECT_PLAYBACKRECORDWINDOW_H

#include <gtkmm/window.h>
#include <gtkmm/button.h>
#include <gtkmm/label.h>
#include <gtkmm/scale.h>
#include <gtkmm/statusbar.h>
#include <gtkmm/treeview.h>
#include <gtkmm/liststore.h>
#include <gtkmm/treestore.h>
#include <gtkmm/spinbutton.h>
#include <gtkmm/entry.h>
#include <gtkmm/entrycompletion.h>
#include <gtkmm/statusbar.h>
#include <gtkmm/builder.h>

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

#include <pumpkin_messages/PlaybackAction.h>
#include <pumpkin_messages/RecordAction.h>
#include <pumpkin_messages/Files.h>
#include "file_type.h"

namespace pumpkin_gui {

	class FolderModel : public Gtk::TreeModelColumnRecord {
	public:
		FolderModel() {add(id); add(name); }
		Gtk::TreeModelColumn<int> id;
		Gtk::TreeModelColumn<Glib::ustring> name;
	};

	class FileModel : public Gtk::TreeModelColumnRecord {
	public:
		FileModel() {add(name);}
		Gtk::TreeModelColumn<Glib::ustring> name;
	};

	typedef Gtk::TreeModel::Children    ChildrenType;
	typedef Gtk::TreeModel::Row         RowType;

	class PlaybackRecordWindow : public Gtk::Window {
	public:
		PlaybackRecordWindow(BaseObjectType *c_wrap, const Glib::RefPtr<Gtk::Builder> &glade);

		virtual ~PlaybackRecordWindow();

		void setFileService(ros::NodeHandle &nh);
	protected:
		//Signals
		void onStartPlayback();

		void onStopPlayback();

		void onStartRecord();

		void onStopRecord();

		void onSelectFolder(const Gtk::TreeModel::Path &path, Gtk::TreeViewColumn* column);

		//Action server callbacks
		void playbackDoneCallback(const actionlib::SimpleClientGoalState &state,
		                          const pumpkin_messages::PlaybackResultConstPtr &result);

		void playbackActiveCallback();

		void playbackFeedbackCallback(const pumpkin_messages::PlaybackFeedbackConstPtr &feedback);

		void recordDoneCallback(const actionlib::SimpleClientGoalState &state,
		                        const pumpkin_messages::RecordResultConstPtr &result);

		void recordActiveCallback();

		void recordFeedbackCallback(const pumpkin_messages::RecordFeedbackConstPtr &feedback);

		//Widgets
		Gtk::Button *_play_button, *_stop_play_button,
				*_rec_button, *_stop_rec_button;
		Gtk::Label *_playback_file_name;
		Gtk::Scale *_playback_time_scale;
		Gtk::TreeView *_folder_tree, *_file_tree;
		Gtk::SpinButton *_rec_min_spin, *_rec_sec_spin;
		Gtk::Entry *_record_file_name;
		Gtk::Statusbar *_status_bar;
		Glib::RefPtr<Gtk::TreeStore> _folder_store;
		std::vector<Glib::RefPtr <Gtk::ListStore> > _file_store_vector;
		FolderModel _folder_model;
		FileModel _file_model;

		//Ros related and other members
		actionlib::SimpleActionClient<pumpkin_messages::PlaybackAction> _playback_client;
		actionlib::SimpleActionClient<pumpkin_messages::RecordAction> _record_client;
		ros::ServiceClient _file_client;
		ros::NodeHandle _nh;
		Glib::ustring _base_folder;
		Glib::RefPtr<Gtk::Builder> _builder;

		//Other methods
		void fillFolder();
	};

}

#endif //PROJECT_PLAYBACKRECORDWINDOW_H

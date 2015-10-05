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

class PlaybackRecordWindow : public Gtk::Window {
public:

protected:
	Gtk::Button *_play_button, *_stop_play_button,
			*_rec_button, *_stop_rec_button;
	Gtk::Label *_playback_file;
	Gtk::Scale *_playback_time_scale;
};


#endif //PROJECT_PLAYBACKRECORDWINDOW_H

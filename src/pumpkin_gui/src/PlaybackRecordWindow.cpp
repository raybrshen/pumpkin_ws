//
// Created by rafaelpaiva on 05/10/15.
//

#include "PlaybackRecordWindow.h"

PlaybackRecordWindow::PlaybackRecordWindow(BaseObjectType *c_wrap,
                                           const Glib::RefPtr<Gtk::Builder> glade) : _playback_client("/playback_action", true),
                                                                                     _record_client("/record_action", true)
{
	glade->get_widget("", );
}

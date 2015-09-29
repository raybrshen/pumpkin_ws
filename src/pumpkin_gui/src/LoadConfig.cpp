//
// Created by rafaelpaiva on 29/09/15.
//

#include "LoadConfig.h"

LoadConfig::LoadConfig() : _window_box(Gtk::ORIENTATION_VERTICAL, 10), _label("Select an calibration file:"),
                           _load_button("Load this file")
{
	//Window basic configuration
	set_title("Load configuration");
	set_border_width(20);
	set_resizable(false);

	_load_button.signal_clicked().connect(sigc::mem_fun(*this, &LoadConfig::select_file));

	_window_box.pack_start(_label, Gtk::PACK_SHRINK);
	_window_box.pack_start(_combo, Gtk::PACK_SHRINK);
	_window_box.pack_start(_load_button, Gtk::PACK_SHRINK);

	add(_window_box);

	show_all_children();
}

LoadConfig::~LoadConfig() {}

void LoadConfig::select_file() {
	const Glib::ustring &value = _combo.get_active_text();

	if (!value.empty()) {
		*_file = value;
	}

	hide();
}

void LoadConfig::fill_combobox(const std::vector<std::string> &filelist) {

	for (std::vector<std::string>::const_iterator it = filelist.begin(); it != filelist.end(); ++it) {
		_combo.append(*it);
	}

	_combo.set_active(1);

	show_all_children();
}

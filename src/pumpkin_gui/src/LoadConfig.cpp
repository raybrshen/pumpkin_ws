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

	_load_button.signal_clicked().connect(sigc::mem_fun(*this, &LoadConfig::on_button_pressed));
}

LoadConfig::~LoadConfig() {}

void LoadConfig::on_button_pressed() {
	const Glib::ustring &value = _combo.get_active_text();

	if (!value.empty()) {
		*_file = value;
	}

	this->close();
}

void LoadConfig::fill_combobox(const std::vector<std::string> &filelist) {

	for (std::vector<std::string>::iterator it = std::begin(filelist); it != std::end(filelist); ++it) {
		_combo.append(*it);
	}

	_combo.set_active(1);

	_window_box.pack_start(_label, Gtk::PACK_SHRINK);
	_window_box.pack_start(_combo, Gtk::PACK_SHRINK);
	_window_box.pack_start(_load_button, Gtk::PACK_SHRINK);

	show_all_children();
}

//
// Created by rafaelpaiva on 29/09/15.
//

#ifndef PROJECT_LOADCONFIG_H
#define PROJECT_LOADCONFIG_H

#include <gtkmm/window.h>
#include <gtkmm/label.h>
#include <gtkmm/button.h>
#include <gtkmm/box.h>
#include <gtkmm/comboboxtext.h>

class LoadConfig : public Gtk::Window {
public:
	LoadConfig();
	virtual ~LoadConfig();

	inline void link_file_string(std::string *ptr) {_file = ptr;}

	void fill_combobox(const std::vector<std::string> &filelist);

protected:

	std::string *_file;

	//widgets
	Gtk::ComboBoxText _combo;
	Gtk::Label _label;
	Gtk::Box _window_box;
	Gtk::Button _load_button;

	//signals
	void select_file();
};


#endif //PROJECT_LOADCONFIG_H

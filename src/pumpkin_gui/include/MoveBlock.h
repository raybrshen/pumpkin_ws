//
// Created by rafaelpaiva on 18/09/15.
//

#ifndef PROJECT_MOVEBLOCK_H
#define PROJECT_MOVEBLOCK_H

#include <gtkmm/grid.h>
#include <gtkmm/label.h>
#include <gtkmm/scale.h>

class MoveBlock : public Gtk::Grid {
public:
	MoveBlock(int pin, int default_pos, int min_pos, int max_pos, Glib::ustring name);
	virtual ~MoveBlock();

	long get_values() { return (int(_speed_scale.get_value()) << sizeof(int)) + int(_pulse_scale.get_value());}

protected:
	//Widgets
	Gtk::Label _part_name;
	Gtk::Label _pulse_label;
	Gtk::Label _speed_label;
	Gtk::Scale _pulse_scale;
	Gtk::Scale _speed_scale;

	int _pin;
	static const double _min_speed = 0;
	static const double _max_speed = 65535.0;
};


#endif //PROJECT_MOVEBLOCK_H

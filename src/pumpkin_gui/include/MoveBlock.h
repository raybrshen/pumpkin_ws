//
// Created by rafaelpaiva on 18/09/15.
//

#ifndef PROJECT_MOVEBLOCK_H
#define PROJECT_MOVEBLOCK_H

//GTKmm includes
#include <gtkmm/grid.h>
#include <gtkmm/label.h>
#include <gtkmm/spinbutton.h>
#include <gtkmm/togglebutton.h>

//Boost includes
#include <boost/utility.hpp>

class MoveBlock : public Gtk::Grid, public boost::noncopyable {
public:
	explicit MoveBlock(Glib::ustring name, int pin, int min_pos, int max_pos, int default_pos, bool activate);
	virtual ~MoveBlock() {};

	inline unsigned int get_pulse_value() const { return (unsigned int)_pulse_spin.get_value();}
	inline unsigned int get_speed_value() const { return (unsigned int)_speed_spin.get_value();}
	inline bool is_active() const {return _active_button.get_active();}
	inline int get_pin() const {return _pin;}
	void set_values (int pulse_pos, int speed_pos = 0);

protected:
	//Widgets
	Gtk::Label _part_name;
	Gtk::Label _pulse_label;
	Gtk::Label _speed_label;
	Gtk::SpinButton _pulse_spin;
	Gtk::SpinButton _speed_spin;
	Gtk::ToggleButton _active_button;

	int _pin;
	static constexpr double _min_speed = 0.0;
	static constexpr double _max_speed = 65535.0;
	static constexpr double _page_size= 100.0;
};


#endif //PROJECT_MOVEBLOCK_H

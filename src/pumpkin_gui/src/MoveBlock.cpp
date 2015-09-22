//
// Created by rafaelpaiva on 18/09/15.
//

#include "MoveBlock.h"

MoveBlock::MoveBlock(Glib::ustring name, int min_pos,
                     int max_pos, int default_pos) : _part_name(name), _pulse_label("Pulse: "), _speed_label("Speed: "),
                                                     _pulse_spin(Gtk::Adjustment::create(double(default_pos),double(min_pos),
                                                                                         double(max_pos), 1.0, MoveBlock::_page_size)),
                                                     _speed_spin(Gtk::Adjustment::create(0.0, _min_speed, _max_speed, 1.0, _page_size))
{
	//First, set the spin buttons
	_pulse_spin.set_digits(0);
	_speed_spin.set_digits(0);
	_pulse_spin.set_numeric(true);
	_speed_spin.set_numeric(true);

	//Set grid parameters
	this->set_column_homogeneous(false);
	this->set_row_homogeneous(false);
	this->set_border_width(2);

	//Insert children
	this->attach(_part_name, 0, 0, 2, 1);
	this->attach(_pulse_label, 0, 1, 1, 1);
	this->attach(_pulse_spin, 1, 1, 1, 1);
	this->attach(_speed_label, 0, 2, 1, 1);
	this->attach(_speed_spin, 1, 2, 1, 1);

	_pulse_label.set_halign(Gtk::ALIGN_END);
	_speed_label.set_halign(Gtk::ALIGN_END);

	this->set_column_spacing(5);
	this->set_row_spacing(5);

	//this->set_size_request(_pulse_label.get_allocated_width()*5,_pulse_label.get_allocated_height()*4);

	//Show children
	show_all_children(true);
}

void MoveBlock::set_values(int pulse_pos, int speed_pos) {
	_pulse_spin.set_value(double(pulse_pos));
	_speed_spin.set_value(double(speed_pos));
}

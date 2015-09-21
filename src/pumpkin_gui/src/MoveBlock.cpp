//
// Created by rafaelpaiva on 18/09/15.
//

#include "MoveBlock.h"

MoveBlock::MoveBlock(Glib::ustring name, int min_pos,
                     int max_pos, int default_pos) : _part_name(name)
{
	//First, set the scales of the move block
	_pulse_scale.set_range(double(min_pos), double(max_pos));
	_pulse_scale.set_digits(0);
	_speed_scale.set_range(MoveBlock::_min_speed, MoveBlock::_max_speed);
	_speed_scale.set_digits(0);

	this->set_values(default_pos);

	//Set labels
	_pulse_label.set_label("Pulse");
	_speed_label.set_label("Speed");

	//Set grid parameters
	this->set_column_homogeneous(false);
	this->set_row_homogeneous(false);
	this->set_border_width(2);

	//Insert children
	this->attach(_part_name, 0, 0, 2, 1);
	this->attach(_pulse_label, 0, 1, 1, 1);
	this->attach(_pulse_scale, 1, 1, 1, 1);
	this->attach(_speed_label, 0, 2, 1, 1);
	this->attach(_speed_scale, 1, 2, 1, 1);

	//Show children
	show_all_children(true);
}

void MoveBlock::set_values(int pulse_pos, int speed_pos) {
	_pulse_scale.set_value(double(pulse_pos));
	_speed_scale.set_value(double(speed_pos));
}

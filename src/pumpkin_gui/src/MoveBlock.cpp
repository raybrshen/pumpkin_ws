//
// Created by rafaelpaiva on 18/09/15.
//

#include "MoveBlock.h"

MoveBlock::MoveBlock(int pin, int default_pos, int min_pos,
                     int max_pos, Glib::ustring name) : _pin(pin), _part_name(name)
{
	//First, set the scales of the move block
	_pulse_scale.set_range(double(min_pos), double(max_pos));
	_pulse_scale.set_digits(0);
	_speed_scale.set_range(_min_speed, _max_speed);
	_speed_scale.set_digits(0);
}

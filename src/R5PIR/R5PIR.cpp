// 	Library for Rover 5 Platform PIR detector
//  Copyright (c) 2016  Robert H. Wortham <r.h.wortham@gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
#include "Arduino.h"
#include "R5PIR.h"

// constructor requires identification of sensor pin
R5PIR::R5PIR(const unsigned char bSensorPin)
{
	_bSensorPin = bSensorPin;
	pinMode( bSensorPin, INPUT );
}

// return the last measured range
unsigned char R5PIR::activated(void)
{
	return digitalRead(_bSensorPin);
}

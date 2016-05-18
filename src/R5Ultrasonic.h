// 	Library for Rover 5 Platform Ultrasonic range detector
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
#ifndef _R5ULTRASONIC_H_
#define _R5ULTRASONIC_H_

class R5Ultrasonic {
public:
	R5Ultrasonic(const unsigned char bSensorPin, const unsigned long ulMinMeasurementInterval);
	unsigned int measureRange(void);
	unsigned int range(void);

private:
	unsigned char _bSensorPin;
	unsigned int _uiRange;
	unsigned long _ulMinMeasurementInterval;
    unsigned long _ulLastRangeMeasurement;
};

#endif // _R5ULTRASONIC_H_

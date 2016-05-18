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
#include "Arduino.h"
#include "R5Ultrasonic.h"

#define ULTRASONIC_TIMEOUT 35000L

// uses the map above to map sensor readings to distances in mm
unsigned int R5Ultrasonic::measureRange(void)
{
	unsigned long ulDurationUS;
	unsigned long ulMillis = millis();

	if (_ulLastRangeMeasurement > ulMillis) // fix rollover issue
		_ulLastRangeMeasurement = 0L;

    // if we last measured the range less than _ulMinMeasurementInterval milliseconds ago
    // then return the previous stored reading
    if ( (ulMillis - _ulLastRangeMeasurement) >= _ulMinMeasurementInterval)
    {
		// Send out request pulse
		pinMode( _bSensorPin, OUTPUT );
		digitalWrite( _bSensorPin, LOW );
		delayMicroseconds( 2 );
		digitalWrite( _bSensorPin, HIGH );
		delayMicroseconds( 5 );
		digitalWrite( _bSensorPin, LOW );

		// Read in the response pulse
		pinMode( _bSensorPin, INPUT );
		ulDurationUS = pulseIn( _bSensorPin, HIGH, ULTRASONIC_TIMEOUT);

		// Convert from US to mm
		_uiRange = (5L*ulDurationUS)/29;

		_ulLastRangeMeasurement = millis();
	}

    return _uiRange;
}



// constructor requires identification of sensor pin
R5Ultrasonic::R5Ultrasonic(const unsigned char bSensorPin, const unsigned long ulMinMeasurementInterval)
{
	_bSensorPin = bSensorPin;
	_ulMinMeasurementInterval = ulMinMeasurementInterval;
	_uiRange = 0;
	_ulLastRangeMeasurement = 0L;
}

// return the last measured range
unsigned int R5Ultrasonic::range(void)
{
	return _uiRange;
}

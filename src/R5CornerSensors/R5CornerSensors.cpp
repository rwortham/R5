// 	Library for Rover 5 Platform Corner Sensors
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
#include "R5CornerSensors.h"

// this maps distance (600-50) to reflected light level (0-690)
// it was measured using white cardboard at 45' to each sensor corner
const int nIRCornerSensorMap[] =
		{600,  0,
		 500, 10,
		 400, 17,
		 300, 29,
		 200, 76,
		 150,137,
		 100,332,
		  60,677,
		  50,690};

// this maps distance (600-50) to reflected light level (0-186)
// it was measured using white cardboard parallel each side
const int nIRSideCornerSensorMap[] =
		{600,  0,
		 500,  1,
		 400,  3,
		 300,  9,
		 200, 15,
		 150, 28,
		 100, 55,
		  90, 89,
		  50,186};


// uses the map above to map sensor readings to distances in mm
int R5CornerSensors::_mapCornerSensor( int nSensorValue )
{
	int nDist = nIRCornerSensorMap[ sizeof(nIRCornerSensorMap)/sizeof(int) - 1 ];
	nSensorValue = max(0, nSensorValue); // -ve values not allowed
	unsigned int i = 2;
	while (i < sizeof(nIRCornerSensorMap)/sizeof(int) )
	{
		int x2 = nIRCornerSensorMap[i+1];
		if (nSensorValue < x2)
		{
			int y1 = nIRCornerSensorMap[i-2];
			int y2 = nIRCornerSensorMap[i];
			int x1 = nIRCornerSensorMap[i-1];

			nDist = y1 + (((y2 - y1)*(nSensorValue - x1))/(x2 - x1));
			break;
		}
	 	i+=2;
	}
	return nDist;
}


// uses the map above to map sensor readings to distances in mm for side calculations
int R5CornerSensors::_mapSideCornerSensor( int nSensorValue )
{
	int nDist = nIRSideCornerSensorMap[ sizeof(nIRSideCornerSensorMap)/sizeof(int) - 1 ];
	nSensorValue = max(0, nSensorValue); // -ve values not allowed
	unsigned int i = 2;
	while (i < sizeof(nIRSideCornerSensorMap)/sizeof(int) )
	{
		int x2 = nIRSideCornerSensorMap[i+1];
		if (nSensorValue < x2)
		{
			int y1 = nIRSideCornerSensorMap[i-2];
			int y2 = nIRSideCornerSensorMap[i];
			int x1 = nIRSideCornerSensorMap[i-1];

			nDist = y1 + (((y2 - y1)*(nSensorValue - x1))/(x2 - x1));
			break;
		}
	 	i+=2;
	}
	return nDist;
}

// constructor requires identification of input and output pins
R5CornerSensors::R5CornerSensors(const unsigned char *pbSensePins, const unsigned char *pbIRPins)
{
	memcpy(_sensePins, pbSensePins, sizeof(_sensePins));
	memcpy(_IRPins, pbIRPins, sizeof(_IRPins));
	_state = 0;
	// set all the array variables to zero
	for ( int i = 0; i < 4; i++)
	{
		_cornerDistance[i] = 0;
		_sideCornerDistance[i] = 0;
    	_edgeDistance[i] = 0;
    	_edgeAngle[i] = 0;
    	passiveLevel[i] = 0;
    	activeLevel[i] = 0;
    	bleedLevel[i] = 0;
	}
	_sensorRange = nIRCornerSensorMap[0]; // first position is the range of the sensors
	// sin(45) ~= 0.70 = 14/20
	_edgeRange = (_sensorRange * 14)/20 ;

	// set the output pins to output and the IR leds off
	for (int i=0; i < 4; i++)
	{
		pinMode(_IRPins[i], OUTPUT);
		digitalWrite(_IRPins[i], LOW);
	}
}

// puts the sensors into pasue mode to save power
unsigned char R5CornerSensors::setPause(const unsigned char bPause)
{
	if (bPause && (_state != 3))
	{
		for (int i=0; i < 4; i++)
		{
			digitalWrite(_IRPins[i], LOW);
		}
		_state = 3;
		return true;
	}
	else if (!bPause && (_state == 3))
	{
		_state = 0;
		return true;
	}
	return false;
}


// just returns true if _state == 3
unsigned char R5CornerSensors::getPause(void)
{
	return (_state == 3) ? true : false;
}

unsigned char R5CornerSensors::sense()
{
	switch(_state)
	{
		case 0: // measure ambient IR and store {all IR LEDS off}, pulse on 1 & 3
			for (unsigned int i = 0; i < 4; i++)
			{
				passiveLevel[i] = analogRead(_sensePins[i]);
			}
			digitalWrite(_IRPins[0], HIGH);
			digitalWrite(_IRPins[2], HIGH);
			_state++;
			break;

		case 1: // measure 1 & 3, pulse off 1&3, pulse on 2 & 4
			activeLevel[0] = analogRead(_sensePins[0]);
			activeLevel[2] = analogRead(_sensePins[2]);
			digitalWrite(_IRPins[0], LOW);
			digitalWrite(_IRPins[2], LOW);
			digitalWrite(_IRPins[1], HIGH);
			digitalWrite(_IRPins[3], HIGH);
			_state++;
			break;

		case 2: // measure 2 & 4, pulse off 2 & 4
			activeLevel[1] = analogRead(_sensePins[1]);
			activeLevel[3] = analogRead(_sensePins[3]);
			digitalWrite(_IRPins[1], LOW);
			digitalWrite(_IRPins[3], LOW);

			// we've been round all states, so calculate distance values
			_calculateDistances();
			_state = 0;
			break;

		case 3: // pause state - do nothing
			break;
	}

	return _state;
}

// assumes nothing in range on any sensors and records the IR bleed
void R5CornerSensors::calibrateBleed(void)
{
	for (unsigned int i = 0; i < 4; i++)
	{
  		bleedLevel[i] = max(activeLevel[i] - passiveLevel[i], 0);
	}
}

// returns true if any obstacle sensed at less than 80% range
unsigned char R5CornerSensors::somethingNear(void)
{
	int near = (_sensorRange * 8) / 10;
	for (unsigned int i=0; i < 4; i++)
	{
		if ( _cornerDistance[i] < near)
			return true;
	}
	return false;
}

// returns true if any pair of sensors at less than 80% edge range
unsigned char R5CornerSensors::edgeNear(void)
{
	int near = (_edgeRange * 8) / 10;

	for (unsigned int i=0; i < 4; i++)
	{
		if ( (_sideCornerDistance[i] < near) && (_sideCornerDistance[(i+1) % 4] < near) )
			return true;
	}
	return false;
}

// returns the index of the nearest corner, or 4 if none
unsigned int R5CornerSensors::nearestCorner(void)
{
	int nDist = _sensorRange;
	unsigned int nCorner = 4;

	for (unsigned int i=0; i < 4; i++)
	{
		if ( (_cornerDistance[i] < nDist) )
		{
			nDist = _cornerDistance[i];
			nCorner = i;
		}
	}
	return nCorner;
}


// returns the index of the nearest edge, or 4 if none
unsigned int R5CornerSensors::nearestEdge(void)
{
	int nDist = _edgeRange;
	unsigned int nEdge = 4;

	if ( edgeNear() )
	{
		for (unsigned int i=0; i < 4; i++)
		{
			if ( (_edgeDistance[i] < nDist) )
			{
				nDist = _edgeDistance[i];
				nEdge = i;
			}
		}
	}
	return nEdge;
}

int R5CornerSensors::getRange(void)
{
	return _sensorRange;
}

int R5CornerSensors::getEdgeRange(void)
{
	return _edgeRange;
}

int R5CornerSensors::getCornerDistance(const unsigned int nSensor)
{
	return _cornerDistance[nSensor % 4];
}

int R5CornerSensors::getSideCornerDistance(const unsigned int nSensor)
{
	return _sideCornerDistance[nSensor % 4];
}

int R5CornerSensors::getEdgeDistance(const unsigned int nEdge)
{
	return _edgeDistance[nEdge % 4];
}

int R5CornerSensors::getEdgeAngle(const unsigned int nEdge)
{
	return _edgeAngle[nEdge % 4];
}


// calculates the measured distances based on measurements, bleed level and estimated range.
// currently assumes linear sensor feedback, and equal range for all sensors
void R5CornerSensors::_calculateDistances(void)
{
	for (unsigned int i = 0; i < 4; i++)
	{
  		int maxRangeLevel = bleedLevel[i] + passiveLevel[i];
  		int reflectedLevel = activeLevel[i] - maxRangeLevel;
  		_cornerDistance[i] = _mapCornerSensor( reflectedLevel );
		// edge distances are tricky, because the sensors are not good at reflecting on left and right sides
		// this code does left and right, by re-calculating 'corners' based on actual sensor sensitivities for sides
		_sideCornerDistance[i] = _mapSideCornerSensor( reflectedLevel );
	}

	// this code does front and back using regular maths
	for (unsigned int i = 0; i < 4; i += 2)
	{
		// need to multiply by sin(45)/2 = 0.354 ~ 7/20
		_edgeDistance[i] = ((_cornerDistance[i] + _cornerDistance[(i+1) % 4])*7)/20;
		if (_edgeDistance[i] > _edgeRange)
			_edgeDistance[i] = _edgeRange;
	}

	for (unsigned int i = 1; i < 4; i += 2)
	{
		// need to multiply by sin(45)/2 = 0.354 ~ 7/20
		_edgeDistance[i] = ((_sideCornerDistance[i] + _sideCornerDistance[(i+1) % 4])*7)/20;
		if (_edgeDistance[i] > _edgeRange)
			_edgeDistance[i] = _edgeRange;
	}


	for (unsigned int i = 0; i < 4; i++)
	{
		int corner1, corner2;

		if (i % 2) // sides
		{
			corner1 = _sideCornerDistance[i];
			corner2 = _sideCornerDistance[(i+1) % 4];
		}
		else // front/back
		{
			corner1 = _cornerDistance[i];
			corner2 = _cornerDistance[(i+1) % 4];
		}

		// R5 sensors are ~ 200mm apart on sides, 150mm front/back
		// and we need 200 / cos 45 = 282.84, 150 / cos 45 = 212.13
		// 1 radian ~ 57.3 degrees
		_edgeAngle[i] = (int)(57.3 * atan( (double)(corner1 - corner2) /
								(double)(corner1 + corner2 + ((i % 2) ? 212 : 283) ) ) );
	}
}


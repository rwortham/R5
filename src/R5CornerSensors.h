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
#ifndef _R5CORNERSENSORS_H_
#define __R5CORNERSENSORS_H_

// #defines for corners and edges for readbility
#define R5_FRONT_RIGHT 0 // corners
#define R5_FRONT_LEFT 1
#define R5_REAR_LEFT 2
#define R5_REAR_RIGHT 3
#define R5_FRONT 0 // edges
#define R5_LEFT 1
#define R5_REAR 2
#define R5_RIGHT 3
#define R5_NONE 4



class R5CornerSensors {
public:
	R5CornerSensors(const unsigned char *pbSensePins, const unsigned char *pbIRPins);
	// the 4 corner sensors and corresponding assumed edge distances & angles are accessed via these arrays
	// 0 = sensor 1 FR, 1 = sensor 2 FL, 2 = sensor 3 RL, 4 = sensor 4 RR  -- distances are in mm
	// edges: 0 = front, 1 = left, 2 = rear, 3 = right


	int passiveLevel[4];// passiveLevel is the measured passive IR level at each corner
	int activeLevel[4]; // activeLevel is the measured active IR level i.e. when the IR LED is on
	int bleedLevel[4];  // bleedLevel is the measured active IR level stored by calibrateRange()
	void calibrateBleed(void);
	int getRange(void);
	int getEdgeRange(void);
	unsigned char sense(void);
	unsigned char setPause(const unsigned char bPause);
	unsigned char getPause(void);
	unsigned char somethingNear(void) ; // returns true if any obstacle sensed at all
	unsigned char edgeNear(void) ; // returns true if any pair of sensors activated
	int getCornerDistance(const unsigned int nSensor);
	int getSideCornerDistance(const unsigned int nSensor);
	int getEdgeDistance(const unsigned int nEdge);
	int getEdgeAngle(const unsigned int nEdge);
	unsigned int nearestCorner(void); // returns the index of the nearest corner, or 4 if none
	unsigned int nearestEdge(void); // returns the index of the nearest edge, or 4 if none


private:
	// state variable is incremented each time Sense is called
	// must be a delay of 1mS or so between each Sense call, and must be called regularly to
	// maintain current values in sensor outputs (i.e. put in main control loop
	// 0 = measure ambient IR and store {all IR LEDS off}, pulse on 1 & 3
	// 1 = measure 1 & 3, pulse off 1&3, pulse on 2 & 4
	// 2 = measure 2 & 4, pulse off 2 & 4
	// 3 = all LEDS off, no sensing - this is the pause state
	unsigned char _state;
	unsigned char _sensePins[4];
	unsigned char _IRPins[4];
	int _cornerDistance[4]; // distance in mm of obstacle from corner sensor
	int _sideCornerDistance[4]; // this array stores 'corner' distances using sensitivity from the sides
	int _edgeDistance[4]; // distance in mm of a wall (if one exists) on each side of the Rover
	int _edgeAngle[4]; // angle of the wall in degrees, relative to the side of the rover

	int _sensorRange;
	int _edgeRange;
	void _calculateDistances(void);
	int _mapCornerSensor(int nSensorValue );
	int _mapSideCornerSensor(int nSensorValue );
};


#endif // __R5CORNERSENSORS_H_

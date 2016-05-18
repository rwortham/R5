// 	Library for Rover 5 Platform Drive motors
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
#ifndef _R5MOTORCONTROL_H_
#define _R5MOTORCONTROL_H_

// these are the valid return types from all commands
// *** Note they match the INSTINCT_ returns for ease of coding ***

#define R5_FAIL			0
#define R5_SUCCESS		1
#define R5_IN_PROGRESS	2
#define R5_ERROR		3

// These are the internal states that the R5MotorControl class can be in
#define R5_NORMAL	0	// not doing any specific movement
#define R5_STOPPED	1	// last command was to stop()
#define R5_ROTATE	2	// busy turning on the spot
#define R5_DRIVE	3	// busy moving forwards in a straight line
#define R5_REVERSE	4	// busy moving backwards in a straight line

#define R5_CNTPER1000MM	1538L // this is the number of quadrature detector clicks per 1000mm travelled.
#define R5_CNTPER100DEG	630L // how many quad clicks to turn 100'

class R5MotorControl {
public:
	R5MotorControl(const unsigned char *pbDrivePins, const unsigned char *pbDirectionPins, const unsigned char *pbCurrentSensePins);

	// these commands return R5_PENDING if a high level command is underway
	// if the command is completed then they return R5_SUCCESS
	// If an external error happens they return R5_FAIL, and R5_ERROR is reserved for internal errors
	// The high level commands require quadrature detector inputs to driveMotors()
	// the low level commands setSpeed(), setRudder(), setReverse() setParalyse() do not.
	unsigned char setSpeed(const int nSpeed);
	unsigned char setRudder(const int nRudder); // imagine a rudder of a boat 0 = straight ahead
    								// 100 full right, -100 full left
    								// can be thought of as a 'rotation rate' when speed is zero
	unsigned char setReverse(const unsigned char bReverse); // an alternative way to make speed = -speed
	unsigned char setParalyse(const unsigned char bParalyse); // stop the robot moving
	unsigned char getParalyse(void);
	unsigned char getReverse(void);
	unsigned char resetDistanceTravelled(void); // reset the distance counter

	int getSpeed(void);
	int getRudder(void);
	int getMotorCurrent(const unsigned char bMotor);
	long getDistanceTravelled(void); // returns distance travelled in mm (approximately!)
	unsigned char getBehaviourState(void); 	// returns the internal state of the behaviour
    										// T5_NORMAL - not doing any specific movement just moving according
    										//			to speed and rudder
    										// T5_STOPPED - last command was to stop()
    										// T5_ROTATE - busy turning on the spot
    										// T5_DRIVE - driving forward in straight line
    										// T5_REVERSE - driving backward in straight line
	// these are higher level movement commands that use the quad detectors
	// they will return R5_FAIL if another command in this group is underway
	// if bCheckForComplete is true then they will return R5_SUCCESS if the command is now complete, otherwise R5_PENDING
	// they can be overidden only by stop() which will always stop the robot and clear state
	unsigned char stop(void);
	unsigned char stopAndRotate(const int nAngle, const unsigned char bCheckForComplete); // stop and rotate +/-180 degrees
	unsigned char move(const int nDistance, const unsigned char bCheckForComplete); // move nDistance mm forward or backward
	unsigned char stopTurnLeft(const unsigned char bCheckForComplete); // turn ~90' left
	unsigned char stopTurnRight(const unsigned char bCheckForComplete); // turn ~90' right
	unsigned char stopTurnAround(const unsigned char bCheckForComplete); // turn ~180' right
	unsigned char driveForward(const int nDistance, const unsigned char bCheckForComplete); // drive forward mm
	unsigned char driveBackward(const int nDistance, const unsigned char bCheckForComplete); // drive backward mm

	void driveMotors(const long lLeftRead, const long lRightRead); // need to call this in main loop to affect outputs

private:
	// The 2 motors and their drive directions are accessed via these arrays, set up in the constructor
	// 0 = Motor 1 Left, 1 = Motor 2 Right
	unsigned char _drivePins[2];
	unsigned char _directionPins[2];
	unsigned char _currentSensePins[2];
	int _speed; // speed 0 - 100%
	int _rudder; // an imagined 'rudder' that differentially affects track speed +-100%
	int _m1;
	int _m2;
	unsigned char _reverse;
	unsigned char _paralyse;
	long _distanceTravelled;
	long _leftQuadRead;
	long _rightQuadRead;
	long _leftQuadDesired;
	long _rightQuadDesired;
	unsigned char _behaviourState;

	void _calculateOutputs(void);
};


#endif // _R5MOTORCONTROL_H_

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
#include "Arduino.h"
#include "R5MotorControl.h"


// constructor requires identification of output pins for speed and direction control
R5MotorControl::R5MotorControl(const unsigned char *pbDrivePins, const unsigned char *pbDirectionPins, const unsigned char *pbCurrentSensePins)
{
	memcpy(_drivePins, pbDrivePins, sizeof(_drivePins));
	memcpy(_directionPins, pbDirectionPins, sizeof(_directionPins));
	memcpy(_currentSensePins, pbCurrentSensePins, sizeof(_currentSensePins));
	_speed = 0;
	_rudder = 0;
	_m1 = _m2 = 0;
	_paralyse = false;
	_distanceTravelled = 0L;
	_leftQuadRead = _rightQuadRead = 0L;
	_leftQuadDesired = _rightQuadDesired = 0L;
	_behaviourState = R5_NORMAL;


	// set the output pins to output, direction to forward and speed to zero
	// sense the current sensing pins to inputs
	for (int i=0; i < 2; i++)
	{
		pinMode(_drivePins[i], OUTPUT);
		analogWrite(_drivePins[i], 0);
		pinMode(_directionPins[i], OUTPUT);
		digitalWrite(_directionPins[i], LOW);
		pinMode(_currentSensePins[i], INPUT);
	}
}

// read the motor current in mA. 0 = sum of two motors, 1 = left, 2 = right
// analogRead() returns 1023 for 5A -> 5mA per unit
int R5MotorControl::getMotorCurrent(const unsigned char bMotor)
{
	int nLeftMotor, nRightMotor;
	int nCurrent = 0;

	switch (bMotor)
	{
		case 0:
			nLeftMotor = analogRead(_currentSensePins[0]);
			nRightMotor = analogRead(_currentSensePins[1]);
			nCurrent = 5 * ( nLeftMotor + nRightMotor );
			break;
		case 1:
			nLeftMotor = analogRead(_currentSensePins[0]);
			nCurrent = 5 * nLeftMotor;
			break;
		case 2:
			nRightMotor = analogRead(_currentSensePins[1]);
			nCurrent = 5 * nRightMotor;
			break;
	}

	return nCurrent;
}

// whatever we are doing, abort and stop now
unsigned char R5MotorControl::stop(void)
{
	_behaviourState = R5_STOPPED;
	_speed = 0;
	_rudder = 0;
	_reverse = false;
	_calculateOutputs();
	return R5_SUCCESS;
}

// turn ~90' left
unsigned char R5MotorControl::stopTurnLeft(const unsigned char bCheckForComplete)
{
	return stopAndRotate( -90, bCheckForComplete );
}

// turn ~90' right
unsigned char R5MotorControl::stopTurnRight(const unsigned char bCheckForComplete)
{
	return stopAndRotate( 90, bCheckForComplete );
}

// turn ~180' depending on previous rudder setting - might be smoother
unsigned char R5MotorControl::stopTurnAround(const unsigned char bCheckForComplete)
{
	return stopAndRotate( (_rudder < 0) ? -180 : 180, bCheckForComplete);
}

// drive forward mm
unsigned char R5MotorControl::driveForward(const int nDistance, const unsigned char bCheckForComplete)
{
	return move( nDistance, bCheckForComplete);
}

// drive backward mm
unsigned char R5MotorControl::driveBackward(const int nDistance, const unsigned char bCheckForComplete)
{
	return move( -nDistance, bCheckForComplete);
}


// stop and rotate +/-180 degrees
unsigned char R5MotorControl::stopAndRotate(const int nAngle, const unsigned char bCheckForComplete)
{
	if ( _behaviourState == R5_STOPPED )
	{
		_behaviourState = R5_NORMAL;
		if ( bCheckForComplete ) // we are checking to see if we've finished, and we have!
			return R5_SUCCESS;
	}
	else if ( (_behaviourState == R5_ROTATE) && bCheckForComplete)
	{
		return R5_IN_PROGRESS;
	}

	if ( ( _behaviourState == R5_NORMAL ) && !bCheckForComplete)
	{
		_behaviourState = R5_ROTATE;
		// calculate desired quadrature positions once this maneovre is complete
		long lTicks = ((long)nAngle * R5_CNTPER100DEG) / 100L;
		_leftQuadDesired = _leftQuadRead + lTicks;
		_rightQuadDesired = _rightQuadRead - lTicks;

		// then set speed and rudder
		_speed = 0; // rotating on the spot
		_rudder = (nAngle > 0) ? 70 : -70; // start turning with 70% rudder
		_calculateOutputs();
		return R5_IN_PROGRESS;
	}
	return R5_FAIL;
}

// move nDistance mm forward or backward
unsigned char R5MotorControl::move(const int nDistance, const unsigned char bCheckForComplete)
{
	if ( _behaviourState == R5_STOPPED )
	{
		_behaviourState = R5_NORMAL;
		if ( bCheckForComplete ) // we are checking to see if we've finished, and we have!
			return R5_SUCCESS;
	}
	else if ( ((_behaviourState == R5_DRIVE) || (_behaviourState == R5_REVERSE)) && bCheckForComplete)
	{
		return R5_IN_PROGRESS;
	}


	if ( (_behaviourState == R5_NORMAL ) && !bCheckForComplete)
	{
		_behaviourState = (nDistance < 0) ? R5_REVERSE : R5_DRIVE;

		// calculate desired quadrature positions once this manoeuvre is complete
		long lTicks = nDistance;
		lTicks = (lTicks * R5_CNTPER1000MM) / 1000;
		_leftQuadDesired = _leftQuadRead + lTicks;
		_rightQuadDesired = _rightQuadRead + lTicks;

		// then set speed and rudder
		_speed = 60; // set speed to 60% for now - maybe make dependent on distance
		_rudder = 0; // move in straight line
		_reverse = (nDistance < 0);
		_calculateOutputs();
		return R5_IN_PROGRESS;
	}
	return R5_FAIL; // false if we are busy with something else
}


unsigned char R5MotorControl::setSpeed(int nSpeed)
{
	if ( _behaviourState == R5_STOPPED )
		_behaviourState = R5_NORMAL;
	if ( _behaviourState == R5_NORMAL )
	{
		// speed is bounded to +-100%
		_speed = min(100, max(-100, nSpeed));
		_calculateOutputs();
		return R5_SUCCESS;
	}
	return R5_FAIL;
}

unsigned char R5MotorControl::setRudder(int nRudder)
{
	if ( _behaviourState == R5_STOPPED )
		_behaviourState = R5_NORMAL;
	if ( _behaviourState == R5_NORMAL )
	{
		// rudder is bounded to +-100%
		_rudder = min(100, max(-100, nRudder));
		_calculateOutputs();
		return R5_SUCCESS;
	}
	return R5_FAIL;
}

unsigned char R5MotorControl::setReverse(unsigned char bReverse)
{
	if ( _behaviourState == R5_STOPPED )
		_behaviourState = R5_NORMAL;
	if ( _behaviourState == R5_NORMAL )
	{
		_reverse = bReverse;
		_calculateOutputs();
		return R5_SUCCESS;
	}
	return R5_FAIL;
}

unsigned char R5MotorControl::setParalyse(const unsigned char bParalyse)
{
  _paralyse = bParalyse;
  _calculateOutputs();
  return R5_SUCCESS;
}

int R5MotorControl::getSpeed(void)
{
	return _speed;
}

int R5MotorControl::getRudder(void)
{
	return _rudder;
}

unsigned char R5MotorControl::getReverse(void)
{
	return _reverse;
}

unsigned char R5MotorControl::getParalyse(void)
{
	return _paralyse;
}

unsigned char R5MotorControl::resetDistanceTravelled(void)
{
	_distanceTravelled = 0L;
	return R5_SUCCESS;
}

long R5MotorControl::getDistanceTravelled(void)
{
	return _distanceTravelled;
}

unsigned char R5MotorControl::getBehaviourState(void)
{
	return _behaviourState;
}

void R5MotorControl::_calculateOutputs(void)
{
	int speed;
	// speed = (m1+m2)/2
	// rudder = m1 - m2
	// motor speed bounded to +-100

	speed = _reverse ? _speed * -1 : _speed;
	_m1 = speed + (_rudder); // increase sensitivity of rudder - 100% not 50%
	_m2 = speed - (_rudder);

	if (abs(_m1) > 100)
	{
		int mAdj = (_m1 > 0)? 100 : -100;
		_m2 = _m2 - (_m1 - mAdj);
		_m1 = mAdj;
	}

	if (abs(_m2) > 100)
	{
		int mAdj = (_m2 > 0)? 100 : -100;
		_m1 = _m1 - (_m2 - mAdj);
		_m2 = mAdj;
	}
}

// need to call this in the main loop - it actually updates the motor outputs
// and reads the quadrature detectors to update _distanceTravelled
void R5MotorControl::driveMotors(long lLeftRead, long lRightRead)
{
	unsigned int speedM1, speedM2;
	// long lLeftRead, lRightRead;
	long lLeftDist, lRightDist;

	// lLeftRead = _pLeftEncoder->read();
	// lRightRead = _pRightEncoder->read();
	lLeftDist = lLeftRead - _leftQuadRead;
	lRightDist = lRightRead - _rightQuadRead;
	_leftQuadRead = lLeftRead; // store values for next iteration
	_rightQuadRead = lRightRead;
	// take the average 1000/2 = 500
    _distanceTravelled += (500L*(lLeftDist + lRightDist))/ R5_CNTPER1000MM;

	// now switch based on state i.e. what this behaviour is busy with
	switch ( _behaviourState )
	{
		case R5_NORMAL:	// nothing to do here
		case R5_STOPPED:
			break;

		case R5_ROTATE:	// have we turned enough yet ?
			if ( _rudder > 0 ) // turning right so left quad increasing
			{
				if ( _leftQuadRead >= _leftQuadDesired )
					_m1 = 0;
				if ( _rightQuadRead <= _rightQuadDesired )
					_m2 = 0;
			}
			else // turning left so right quad increasing
			{
				if ( _leftQuadRead <= _leftQuadDesired )
					_m1 = 0;
				if ( _rightQuadRead >= _rightQuadDesired )
					_m2 = 0;
			}
			if ( (_m1 == 0) && (_m2 == 0) ) //we are there
			{
				_behaviourState = R5_STOPPED;
				_speed = 0;
				_rudder = 0;
				_reverse = 0;
			}
			break;

		case R5_DRIVE:	// have we gone far enough yet ?
			if ( _leftQuadRead >= _leftQuadDesired )
				_m1 = 0;
			if ( _rightQuadRead >= _rightQuadDesired )
				_m2 = 0;
			if ( (_m1 == 0) && (_m2 == 0) ) //we are there
			{
				_behaviourState = R5_STOPPED;
				_speed = 0;
				_rudder = 0;
				_reverse = 0;			}
			break;


		case R5_REVERSE:	// have we reversed far enough yet ?
			if ( _leftQuadRead <= _leftQuadDesired )
				_m1 = 0;
			if ( _rightQuadRead <= _rightQuadDesired )
				_m2 = 0;
			if ( (_m1 == 0) && (_m2 == 0) ) //we are there
			{
				_behaviourState = R5_STOPPED;
				_speed = 0;
				_rudder = 0;
				_reverse = 0;
			}
			break;
	}
	speedM1 = abs(_m1);
	speedM2 = abs(_m2);

    analogWrite(_drivePins[0], _paralyse ? 0 : (speedM1*255)/100);
    analogWrite(_drivePins[1], _paralyse ? 0 : (speedM2*255)/100);
    digitalWrite(_directionPins[0], (_m1 < 0) ? HIGH : LOW);
    digitalWrite(_directionPins[1], (_m2 < 0) ? HIGH : LOW);
}

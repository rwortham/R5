// 	Library for Rover 5 Platform Head Controller
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
// This class supports the basic head with 2 degrees of freedom, but no attached sensors
//
#include "Arduino.h"
#include "Servo.h" // adding this include uses 139 bytes of RAM somehow
#include "R5HeadControl.h"

// constructor requires identification of servos
R5HeadControl::R5HeadControl(Servo *pServoHHead, Servo *pServoVHead, const unsigned char bHCentreAngle, const unsigned char bVForwardAngle)
{
	_nHScanInterval = 0;
	_bHMinServoMovement = 10;
	_bHMinServoPosition = 0;
	_bHMaxServoPosition = 180;
	_bHScanDirection = 0;
	_ulLastDriveHHeadTime = 0L;

	_nVScanInterval = 0;
	_bVMinServoMovement = 10;
	_bVMinServoPosition = 0;
	_bVMaxServoPosition = 180;
	_bVScanDirection = 0;
	_ulLastDriveVHeadTime = 0L;

	_bHCentreAngle = bHCentreAngle;
	_bVForwardAngle = bVForwardAngle;
	_pServoHHead = pServoHHead;
    _pServoVHead = pServoVHead;

    _bParalyse = 0;
}

void R5HeadControl::setParalyse(const unsigned char bParalyse)
{
  _bParalyse = bParalyse;
}

unsigned char R5HeadControl::getParalyse(void)
{
	return _bParalyse;
}

unsigned int R5HeadControl::getHScanInterval(void)
{
	return _nHScanInterval;
}

unsigned int R5HeadControl::getVScanInterval(void)
{
	return _nVScanInterval;
}

// set the servo parameters required for horizontal scanning
void R5HeadControl::setHScanParams(const unsigned char bMinServoMovement, const unsigned char bMinServoPosition, const unsigned char bMaxServoPosition)
{
	_bHMinServoMovement = bMinServoMovement;
	_bHMinServoPosition = bMinServoPosition;
	_bHMaxServoPosition = bMaxServoPosition;
}

// Sets the scan interval for one complete horizontal sweep in mS. Set to zero to stop scanning
void R5HeadControl::setHScanInterval(const unsigned int nScanInterval)
{
	_nHScanInterval = nScanInterval;
	_ulLastDriveHHeadTime = millis(); // stop where we are for one cycle before reading value
}

// set the servo parameters required for vertical scanning
void R5HeadControl::setVScanParams(const unsigned char bMinServoMovement, const unsigned char bMinServoPosition, const unsigned char bMaxServoPosition)
{
	_bVMinServoMovement = bMinServoMovement;
	_bVMinServoPosition = bMinServoPosition;
	_bVMaxServoPosition = bMaxServoPosition;
}

// Sets the scan interval for one complete vertical sweep in mS. Set to zero to stop scanning
void R5HeadControl::setVScanInterval(const unsigned int nScanInterval)
{
	_nVScanInterval = nScanInterval;
	_ulLastDriveHHeadTime = millis();
}

// centres the head and looks forward
void R5HeadControl::lookAhead(void)
{
	_pServoHHead->write(_bHCentreAngle);
	_pServoVHead->write(_bVForwardAngle);
}

// looks forward and to the side we are nearest
// this is the minimum distance for the head to move to start a hoizontal scan
void R5HeadControl::lookNearestSide(void)
{
	int nServoPos = _pServoHHead->read();

	if ((nServoPos - _bHMinServoPosition) < (_bHMaxServoPosition - nServoPos))
		nServoPos = _bHMinServoPosition;
	else
		nServoPos = _bHMaxServoPosition;

	_pServoHHead->write(nServoPos);
	_pServoVHead->write(_bVForwardAngle);
}

// this function is called frequently by the robots main loop, but not necessarily at equal time intervals
// so we have to do some calculations to work out if we need to move the motors, and if so how far.
// we have to do a complete head sweep x'->y' every bXScanInterval milliseconds, that's (x-y)' in total
// so the number of milliseconds per n degrees is n*bXScanInterval/(x-y)
// if we have elapsed at least this number of milliseconds then calculate the new servo position and set it.
void R5HeadControl::driveHead(void)
{
	unsigned long ulNow = millis();
	unsigned long ulMillisPerNDegrees;
	int nDegreesToMove;
	int nServoCycle;
	int nServoPos;

	// check if we are paralysed and do nothing if we are
	if (_bParalyse)
		return;

	if (_nHScanInterval > 0)
	{
		nServoCycle = _bHMaxServoPosition - _bHMinServoPosition;
		ulMillisPerNDegrees = ((unsigned long)_bHMinServoMovement * (unsigned long)_nHScanInterval)/ nServoCycle;

		// see if its time to move again ..
		if ( (ulNow - _ulLastDriveHHeadTime) >= ulMillisPerNDegrees)
		{
			// if it is then notify the current position BEFORE we move. This allows sensors to take readings before the motors move
			// because mechanics moves much more slowly than software
			nServoPos = _pServoHHead->read();
			notifyHMovement(_bHScanDirection, nServoPos);
			// if servo is at an endstop, notify that too and change direction
			if ((nServoPos >= _bHMaxServoPosition) || (nServoPos <= _bHMinServoPosition))
			{
				notifyHEndstop(_bHScanDirection, nServoPos);
				_bHScanDirection = !_bHScanDirection;
			}

			// now calculate how many degrees to move, and in what direction
			nDegreesToMove = (((ulNow - _ulLastDriveHHeadTime) * nServoCycle) / _nHScanInterval) % nServoCycle;
			if (_bHScanDirection )
			{
				nServoPos = min((int)_bHMaxServoPosition, nServoPos + nDegreesToMove);
			}
			else
			{
				nServoPos = max(_bHMinServoPosition, nServoPos - nDegreesToMove);
			}
			_pServoHHead->write(nServoPos); // instruct the servo to move. The actual movement will take a while

			_ulLastDriveHHeadTime = ulNow; // remember when we last moved
		}
	}

	// now do the same for the vertical motor
	if (_nVScanInterval > 0)
	{
		nServoCycle = 2 * (_bVMaxServoPosition - _bVMinServoPosition);
		ulMillisPerNDegrees = ((unsigned long)_bVMinServoMovement * (unsigned long)_nVScanInterval)/ nServoCycle;
		if ( (ulNow - _ulLastDriveVHeadTime) >= ulMillisPerNDegrees)
		{
			// if it is then notify the current position BEFORE we move. This allows sensors to take readings before the motors move
			// because mechanics moves much more slowly than software
			nServoPos = _pServoVHead->read();
			notifyVMovement(_bVScanDirection, nServoPos);
			// if servo is at an endstop, notify that too and change direction
			if ((nServoPos >= _bVMaxServoPosition) || (nServoPos <= _bVMinServoPosition))
			{
				notifyVEndstop(_bVScanDirection, nServoPos);
				_bVScanDirection = !_bVScanDirection;
			}

			// now calculate how many degrees to move, and in what direction
			nDegreesToMove = (((ulNow - _ulLastDriveVHeadTime) * nServoCycle) / _nVScanInterval) % nServoCycle;
			if (_bVScanDirection )
			{
				nServoPos = min((int)_bVMaxServoPosition, nServoPos + nDegreesToMove);
			}
			else
			{
				nServoPos = max(_bVMinServoPosition, nServoPos - nDegreesToMove);
			}
			_pServoVHead->write(nServoPos); // instruct the servo to move. The actual movement will take a while

			_ulLastDriveVHeadTime = ulNow; // remember when we last moved
		}
	}
}

// these notifications do nothing but can be subclassed if the head has sensors
void R5HeadControl::notifyHEndstop(const unsigned char bScanDirection, const unsigned char bServoPosition)
{
}

void R5HeadControl::notifyHMovement(const unsigned char bScanDirection, const unsigned char bServoPosition)
{
}

void R5HeadControl::notifyVEndstop(const unsigned char bScanDirection, const unsigned char bServoPosition)
{
}

void R5HeadControl::notifyVMovement(const unsigned char bScanDirection, const unsigned char bServoPosition)
{
}


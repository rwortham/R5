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
#ifndef _R5HEADCONTROL_H_
#define _R5HEADCONTROL_H_

class R5HeadControl {
public:
	// constructor - link to the servo hardware and define central position forward looking position
	R5HeadControl(Servo *pServoHHead, Servo *pServoVHead, const unsigned char bHCentreAngle, const unsigned char bVForwardAngle);
	void setHScanParams(const unsigned char bMinServoMovement, const unsigned char bMinServoPosition, const unsigned char bMaxServoPosition);
	void setHScanInterval(const unsigned int nScanInterval);
	void setVScanParams(const unsigned char bMinServoMovement, const unsigned char bMinServoPosition, const unsigned char bMaxServoPosition);
	void setVScanInterval(const unsigned int nScanInterval);
	void setParalyse(const unsigned char bParalyse); // stop the head moving
	unsigned int getHScanInterval(void);
	unsigned int getVScanInterval(void);
	unsigned char getParalyse(void);
	void lookAhead(void);
	void lookNearestSide(void);
	void driveHead(void);	// called my the main loop to give cpu to the head

protected:
	virtual void notifyHEndstop(const unsigned char bScanDirection, const unsigned char bServoPosition);
	virtual void notifyHMovement(const unsigned char bScanDirection, const unsigned char bServoPosition);
	virtual void notifyVEndstop(const unsigned char bScanDirection, const unsigned char bServoPosition);
	virtual void notifyVMovement(const unsigned char bScanDirection, const unsigned char bServoPosition);

	// these params are protected so they can be accessed by the R5SensingHead class
	Servo *_pServoHHead;				// the servo itself
	unsigned char _bHCentreAngle;
	unsigned char _bVForwardAngle;

	unsigned char _bHMinServoPosition;	// the minimum servo position, default 0
	unsigned char _bHMaxServoPosition;	// the maximum servo position, default 180

	Servo *_pServoVHead;
	unsigned char _bVMinServoPosition;
	unsigned char _bVMaxServoPosition;

private:
	unsigned char _bParalyse;
	// parameters for the horizontal servo
	unsigned int  _nHScanInterval;		// number of mS for a complete scan. Set to zero to stop scanning
	unsigned char _bHScanDirection;		// the current scan direction
	unsigned char _bHMinServoMovement;	// the minimum number of degrees that the servo will move
    unsigned long _ulLastDriveHHeadTime;	// the last time the servo was updated

	// parameters for the vertical servo
	unsigned int  _nVScanInterval;
	unsigned char _bVScanDirection;
	unsigned char _bVMinServoMovement;
    unsigned long _ulLastDriveVHeadTime;
};

#endif // _R5HEADCONTROL_H_

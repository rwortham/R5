// 	Library for Rover 5 Platform Head Controller with Ultrasonic distance sensor
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
#ifndef _R5SENSINGHEAD_H_
#define _R5SENSINGHEAD_H_

// the maximum range we can sense in mm
#define R5_HEAD_MAXRANGE 4000
#define R5_HEAD_MINRANGE 50
// width of a corridor needed for the robot in mm
#define R5_ROBOT_WIDTH 300

class R5SensingHead : public R5HeadControl {
public:
	// constructor - link to the servo and sensing hardware and specify the size of the sensing matrix
	R5SensingHead(Servo *pServoHHead, Servo *pServoVHead, const unsigned char bHCentreAngle, const unsigned char bVForwardAngle,
		R5Ultrasonic *pUltrasonic, const unsigned char bHCells, const unsigned char bVCells, const unsigned char bSmoothing);
	unsigned int getLeftEndStopRange(void);
	unsigned int getRightEndStopRange(void);
	unsigned int getTopEndStopRange(void);
	unsigned int getBottomEndStopRange(void);
	unsigned char getHCells(void);
	unsigned char getVCells(void);
	unsigned int getRangeAtCell(const unsigned char bHCell, const unsigned char bVCell);
	unsigned char getHMostOpenAngle(const unsigned char bVCoord);
	unsigned char senseMatrixReady(void);
	unsigned int getMinRange(void);
	unsigned char senseHMatrixReady(const unsigned char bVCoord);
	unsigned int getHMinRange(const unsigned char bVCoord);
	unsigned char senseVMatrixReady(const unsigned char bHCoord);
	unsigned int getVMinRange(const unsigned char bHCoord);
	void clearSenseMatrix(void);

protected:
	virtual void notifyHEndstop(const unsigned char bScanDirection, const unsigned char bServoPosition);
	virtual void notifyHMovement(const unsigned char bScanDirection, const unsigned char bServoPosition);
	virtual void notifyVEndstop(const unsigned char bScanDirection, const unsigned char bServoPosition);
	virtual void notifyVMovement(const unsigned char bScanDirection, const unsigned char bServoPosition);

private:
	void updateSenseMatrix(const unsigned char bHServoPosition, const unsigned char bVServoPosition);
	long taylorFPSin(const int nAngle);

	R5Ultrasonic *_pUltrasonic;
	unsigned char _bHCells;
	unsigned char _bVCells;
	unsigned char _bSmoothing;
	unsigned int *_pSenseMatrix;
	unsigned int _uiLeftEndStopRange;
	unsigned int _uiRightEndStopRange;
	unsigned int _uiTopEndStopRange;
	unsigned int _uiBottomEndStopRange;
};

#endif // _R5SENSINGHEAD_H_

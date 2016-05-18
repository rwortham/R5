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
#include "Arduino.h"
#include "Servo.h" // adding this include uses 139 bytes of RAM somehow
#include "R5HeadControl.h"
#include "R5Ultrasonic.h"
#include "R5SensingHead.h"

// constructor requires identification of servos and sensor and the size of our sensing matrix
R5SensingHead::R5SensingHead(Servo *pServoHHead, Servo *pServoVHead, const unsigned char bHCentreAngle, const unsigned char bVForwardAngle,
		R5Ultrasonic *pUltrasonic, const unsigned char bHCells, const unsigned char bVCells, const unsigned char bSmoothing)
	: R5HeadControl(pServoHHead, pServoVHead, bHCentreAngle, bVForwardAngle)
{
	int nBuffSize;

	_pUltrasonic = pUltrasonic;
	_bHCells = bHCells;
	_bVCells = bVCells;
	_bSmoothing = bSmoothing;
	_pSenseMatrix = 0;

	nBuffSize = _bHCells * _bVCells * sizeof(unsigned int);
	_pSenseMatrix = (unsigned int *)malloc(nBuffSize);
	if (_pSenseMatrix)
		clearSenseMatrix();
	else
	{
		// if we can't alloc the buffer then we have no matrix to write into
		_bHCells = 0;
		_bVCells = 0;
	}
}

// clear the sensor array and reset to initial values
void R5SensingHead::clearSenseMatrix(void)
{
	int nArraySize = _bHCells * _bVCells;
	for (int i = 0; i < nArraySize; i++)
		*(_pSenseMatrix+i) = 0; // set to zero
}

void R5SensingHead::notifyHEndstop(const unsigned char bScanDirection, const unsigned char bServoPosition)
{
	unsigned int uiRange = _pUltrasonic->measureRange();
	if (bScanDirection)
	{
		_uiLeftEndStopRange = uiRange;
	}
	else
	{
		_uiRightEndStopRange = uiRange;
	}
}

void R5SensingHead::notifyHMovement(const unsigned char bScanDirection, const unsigned char bServoPosition)
{
	updateSenseMatrix(bServoPosition, _pServoVHead->read());
}

void R5SensingHead::notifyVEndstop(const unsigned char bScanDirection, const unsigned char bServoPosition)
{
	unsigned int uiRange = _pUltrasonic->measureRange();
	if (bScanDirection)
	{
		_uiBottomEndStopRange = uiRange;
	}
	else
	{
		_uiTopEndStopRange = uiRange;
	}
}

void R5SensingHead::notifyVMovement(const unsigned char bScanDirection, const unsigned char bServoPosition)
{
	updateSenseMatrix(_pServoHHead->read(), bServoPosition);
}

// if the head has moved we need to take a new rangefinding, and then update the correct cell in the
// sense matrix. The updating is done as a moving average
void R5SensingHead::updateSenseMatrix(const unsigned char bHServoPosition, const unsigned char bVServoPosition)
{
	unsigned int uiRange;
	unsigned char bHCoord;
	unsigned char bVCoord;
	unsigned int *pCell;

	if (!_pSenseMatrix || !_bHCells || !_bVCells)
		return;

	// measure the range and bound result
	uiRange = _pUltrasonic->measureRange();
	uiRange = min(uiRange, R5_HEAD_MAXRANGE);
	uiRange = max(uiRange, R5_HEAD_MINRANGE);

	// calculate which cell we need to update
	bHCoord = (_bHMaxServoPosition - bHServoPosition) / ((_bHMaxServoPosition - _bHMinServoPosition)/ (_bHCells - 1));
	bHCoord = min(bHCoord, _bHCells - 1);
	bVCoord = (_bVMaxServoPosition - bVServoPosition) / ((_bVMaxServoPosition - _bVMinServoPosition)/ (_bVCells - 1));
	bVCoord = min(bVCoord, _bVCells - 1);

	pCell = _pSenseMatrix + (bHCoord + (bVCoord * _bHCells));
	if (_bSmoothing && *pCell) // only do smoothing if we have a previous reading in the array
	{
		*pCell = (((100L - (unsigned long)_bSmoothing) * (unsigned long)uiRange) + ((unsigned long)_bSmoothing * (unsigned long)*pCell)) / 100L;
	}
	else
	{
		*pCell = uiRange;
	}
}

unsigned int R5SensingHead::getLeftEndStopRange(void)
{
	return _uiLeftEndStopRange;
}

unsigned int R5SensingHead::getRightEndStopRange(void)
{
	return _uiRightEndStopRange;
}

unsigned int R5SensingHead::getTopEndStopRange(void)
{
	return _uiTopEndStopRange;
}

unsigned int R5SensingHead::getBottomEndStopRange(void)
{
	return _uiBottomEndStopRange;
}

unsigned char R5SensingHead::getHCells(void)
{
	return _bHCells;
}

unsigned char R5SensingHead::getVCells(void)
{
	return _bVCells;
}

unsigned int R5SensingHead::getRangeAtCell(const unsigned char bHCoord, const unsigned char bVCoord)
{
	unsigned int *pCell;

	if ((bHCoord < _bHCells) && (bVCoord < _bVCells) )
	{
		pCell = _pSenseMatrix + (bHCoord + (bVCoord * _bHCells));
		return *pCell;
	}

	return 0;
}

// looks over the horizontal array at vCoord and works out the most open angle to move forwards
unsigned char R5SensingHead::getHMostOpenAngle(const unsigned char bVCoord)
{
	unsigned char bAngle = 0;
	unsigned int *pCell;
	unsigned int uiMaxRange = 0;
	unsigned int uiMinRange = R5_HEAD_MAXRANGE;
	unsigned int nMaxCell = 0;

	pCell = _pSenseMatrix + (bVCoord * _bHCells); // point to the correct row
	// scan over each cell and see which one has the largest value
	for (unsigned char i = 0; i < _bHCells; i++)
	{
		if (*pCell > uiMaxRange)
		{
			uiMaxRange = *pCell;
			nMaxCell = i;
		}
		if (*pCell && (*pCell < uiMinRange)) // ignore sero values when looking for minimum
		{
			uiMinRange = *pCell;
		}
		pCell++;
	}

	// if there is no real difference across the scan, then return mid position
	if ( (uiMaxRange - uiMinRange) < 10 )
	{
		nMaxCell = _bHCells / 2;
	}

	// So, now we know which cell it is, we need to calculate the corresponding cell angle
	bAngle = _bHMaxServoPosition - ((nMaxCell * (unsigned int)(_bHMaxServoPosition - _bHMinServoPosition)) /
									(unsigned int)(_bHCells - 1));

	return bAngle;
}

// looks over the entire array and returns true if all values are present
unsigned char R5SensingHead::senseMatrixReady(void)
{
	unsigned char bReady = true;
	unsigned int *pCell;

	pCell = _pSenseMatrix;
	// scan over each cell and see if any are zero
	for (unsigned char i = (_bHCells * _bVCells); i > 0; i--)
	{
		if (*pCell == 0)
		{
			bReady = false;
		}
		pCell++;
	}

	return bReady;
}

// looks over the entire array and returns the minimum value
unsigned int R5SensingHead::getMinRange(void)
{
	unsigned int *pCell;
	unsigned int uiMinRange = R5_HEAD_MAXRANGE;

	pCell = _pSenseMatrix;
	// scan over each cell and see which one has the smallest value
	for (unsigned char i = (_bHCells * _bVCells); i > 0; i--)
	{
		if (*pCell && (*pCell < uiMinRange)) // ignore sero values when looking for minimum
		{
			uiMinRange = *pCell;
		}
		pCell++;
	}

	return uiMinRange;
}


// looks over the horizontal array at bVCoord and returns true if all values are present
unsigned char R5SensingHead::senseHMatrixReady(const unsigned char bVCoord)
{
	unsigned char bReady = true;
	unsigned int *pCell;

	pCell = _pSenseMatrix + (bVCoord * _bHCells); // point to the correct row
	// scan over each cell and see if any are zero
	for (unsigned char i = 0; i < _bHCells; i++)
	{
		if (*pCell == 0)
		{
			bReady = false;
		}
		pCell++;
	}

	return bReady;
}

// looks over the horizontal array at bVCoord and returns the minimum value ahead
// ranges in the side lobes are discounted if the robot will pass by unhindered
unsigned int R5SensingHead::getHMinRange(const unsigned char bVCoord)
{
	unsigned int *pCell;
	unsigned int uiMinRange = R5_HEAD_MAXRANGE;
	unsigned int nServoCycle = _bHMaxServoPosition - _bHMinServoPosition;

	pCell = _pSenseMatrix + (bVCoord * _bHCells); // point to the correct row
	// scan over each cell and see which one has the smallest value
	for (unsigned char i = 0; i < _bHCells; i++)
	{
		if (*pCell && (*pCell < uiMinRange)) // ignore sero values when looking for minimum
		{
			// calculate the range beyond which the value is not relevant
			// first calculate the angle from forward relating to this sensor position
			int nAngle = (int)(_bHMinServoPosition + ((i * nServoCycle) /
									(unsigned int)(_bHCells - 1))) - (int)_bHCentreAngle;

			// get absolute value of this angle and find the sin
			nAngle = (nAngle > 0) ? nAngle : -nAngle;
			long lSine = taylorFPSin(nAngle);
			int nMaxDist = (50L * R5_ROBOT_WIDTH) / (lSine ? lSine : 1L); // this is the max dist we care about

			if (*pCell < nMaxDist)
			{
				if (nAngle)
				{
					// if we are offset from centre then the distance we need to use is reduced by sin(90-x)
					lSine = taylorFPSin(90 - nAngle);
					uiMinRange = (unsigned int)(((long)*pCell * lSine) / 100L);
				}
				else
				{
					// this is just an optimisation to avoid calling taylorFPSin(90) = 1
					uiMinRange = *pCell;
				}
			}
		}
		pCell++;
	}

	return uiMinRange;
}

// take an angle in degrees and return 100 * sin(nAngle)
// uses Taylor expansion to 2nd term
long R5SensingHead::taylorFPSin(const int nAngle)
{
	// for speed use integer maths and scale by 100
	// scale by 100 * pi/180 = 1.7453 = 175/100
	long lAngle = (nAngle * 175L) / 100L; // this is 100 * angle in rads
	long lSine = lAngle - ((lAngle * lAngle * lAngle) / 60000L); // this is 100 * sin(angle) to within 3.5% at 80'

	return lSine;
}

// looks over the vertical array at bHCoord and returns true if all values are present
unsigned char R5SensingHead::senseVMatrixReady(const unsigned char bHCoord)
{
	unsigned char bReady = true;
	unsigned int *pCell;

	pCell = _pSenseMatrix + bHCoord; // point to the column
	// scan over each cell and see if any are zero
	for (unsigned char i = 0; i < _bVCells; i++)
	{
		if (*pCell == 0)
		{
			bReady = false;
		}
		pCell += _bHCells; // move to the next row
	}

	return bReady;
}

// looks over the vertical array at bHCoord and returns the minimum value
unsigned int R5SensingHead::getVMinRange(const unsigned char bHCoord)
{
	unsigned int *pCell;
	unsigned int uiMinRange = R5_HEAD_MAXRANGE;

	pCell = _pSenseMatrix + bHCoord;
	// scan over each cell and see which one has the smallest value
	for (unsigned char i = 0; i < _bVCells; i++)
	{
		if (*pCell && (*pCell < uiMinRange)) // ignore sero values when looking for minimum
		{
			uiMinRange = *pCell;
		}
		pCell += _bHCells; // move to the next row
	}

	return uiMinRange;
}

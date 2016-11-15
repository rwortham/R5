// 	Library for Rover 5 Platform
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
#ifndef _R5_H_
#define __R5_H_

#include "R5Output.h"
#include "R5CornerSensors.h"
#include "R5MotorControl.h"
#include "R5HeadControl.h"
#include "R5Ultrasonic.h"
#include "R5SensingHead.h"
#include "R5PIR.h"
#include "R5Voice.h"
#include "R5Vocalise.h"
#include "R5EEPROM.h"

// implementation of MyMonitor is in Robot_Instinct but definitions are here
// because Arduino sketches have no concept of include files

class MyMonitor : public R5ExecStackMonitor {
public:
  MyMonitor(Instinct::Names *pNames, R5Voice *pVocaliser, R5Output *pOut) : R5ExecStackMonitor(pNames, pVocaliser, pOut) {};
  unsigned char nodeExecuted(const Instinct::PlanNode * pPlanNode);
  unsigned char nodeSuccess(const Instinct::PlanNode * pPlanNode);
  unsigned char nodeInProgress(const Instinct::PlanNode * pPlanNode);
  unsigned char nodeFail(const Instinct::PlanNode * pPlanNode);
  unsigned char nodeError(const Instinct::PlanNode * pPlanNode);
  unsigned char nodeSense(const Instinct::ReleaserType *pReleaser, const int nSenseValue);
};


#endif // __R5_H_

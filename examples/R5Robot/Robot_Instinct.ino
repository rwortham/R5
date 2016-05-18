//  R5 Robot Platform - Interface to Instinct planner
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

// contains the interface to the Instinct Planner, 
// including the 3 required callback classes Senses, Actions & Monitor

// Global functions defined in this file
char * getProgmemStr(char *pStrBuff, const int nBuffLen, const unsigned char nStrOffset, char *progBuff);
char * getNodeTypeName(char *pBuff, const int nBuffLen, const unsigned char bNodeType);

// define the available robot actions
#define ACTION_SETSPEED 1
#define ACTION_MOVEBY 2
#define ACTION_TURN 3
#define ACTION_STOP 4
#define ACTION_SLEEP 5
#define ACTION_WAKE 6
#define ACTION_HMOVEHEAD 7
#define ACTION_VMOVEHEAD 8
#define ACTION_FAIL 9 // a test action that always fails
#define ACTION_HSCAN 10
#define ACTION_VSCAN 11
#define ACTION_WAIT 12
#define ACTION_TURNTOMOSTOPEN 13
#define ACTION_TURNMOSTOPENDIR 14
#define ACTION_WAIT_HSCANREADY 15
#define ACTION_WAIT_VSCANREADY 16
#define ACTION_RESET_HUMAN_DETECTOR 17
#define ACTION_WAIT_SCANREADY 18
#define ACTION_SCAN 19
#define ACTION_CONF_HUMAN 20
#define ACTION_FLASH_COLOUR 21

// define the available robot senses
// these return distances to obstacles in units 0-3
#define SENSE_FRONT_RIGHT 1
#define SENSE_FRONT_LEFT 2
#define SENSE_REAR_LEFT 3
#define SENSE_REAR_RIGHT 4
#define SENSE_FRONT 5
#define SENSE_REAR 6
#define SENSE_RANDOM 7
#define SENSE_SLEEPING 8
#define SENSE_FIFTY 9
#define SENSE_RANGE 10
#define SENSE_FRONT_RANGE 11
#define SENSE_PIR 12
#define SENSE_LEFT 13
#define SENSE_RIGHT 14
#define SENSE_NEAREST_CORNER 15
#define SENSE_NEAREST_EDGE 16
#define SENSE_MOTOR_CURRENT 17
#define SENSE_MOVING_HSCANINTERVAL 18
#define SENSE_STOPPED_VSCANINTERVAL 19
#define SENSE_MIN_RANGE_AHEAD 20
#define SENSE_HSCANREADY 21
#define SENSE_VSCANREADY 22
#define SENSE_HUMAN_AHEAD 23
#define SENSE_SCANREADY 24
#define SENSE_CONFIRMED_HUMAN 25

#define MAX_DIST_FOR_HUMAN 800
#define MIN_TRAVEL_BETWEEN_HUMANS 1500

// store strings in flash memory to save RAM. Accessed via getProgmemStr
const char PROGMEM szNodeType[] = "AP!APE!C!CE!D!A!";

class MySenses : public Instinct::Senses {
public:
  int readSense(const Instinct::instinctID nSense);
};

class MyActions : public Instinct::Actions {
public:
  MyActions() {_sleepStart = 0L; _waitStart = 0L; _nNewHeading = 0; _bConfirmedHuman = 0;};
  unsigned char executeAction(const Instinct::actionID nAction, const int nActionValue, const unsigned char bCheckForComplete);
  unsigned char robotSleep(const int nSleepCount, const unsigned char bCheckForComplete);
  unsigned char robotWait(const int nWaitCount, const unsigned char bCheckForComplete);
  unsigned char robotWake(const unsigned char bCheckForComplete);
  unsigned char confirmedHuman(void) {return _bConfirmedHuman;};
  
private:
  unsigned long _sleepStart;
  unsigned long _waitStart;
  int _nNewHeading;
  unsigned char _bConfirmedHuman;
};

class MyMonitor : public Instinct::Monitor {
public:
  unsigned char nodeExecuted(const Instinct::PlanNode * pPlanNode);
  unsigned char nodeSuccess(const Instinct::PlanNode * pPlanNode);
  unsigned char nodeInProgress(const Instinct::PlanNode * pPlanNode);
  unsigned char nodeFail(const Instinct::PlanNode * pPlanNode);
  unsigned char nodeError(const Instinct::PlanNode * pPlanNode);
  unsigned char nodeSense(const Instinct::ReleaserType *pReleaser, const int nSenseValue);
};


// The Instinct Planner Structures
MySenses mySenses;
MyActions myActions;
MyMonitor myMonitor;
Instinct::instinctID PlanSize[INSTINCT_NODE_TYPES] = { 0, 0, 0, 0, 0, 0 };
Instinct::CmdPlanner myPlan(PlanSize, &mySenses, &myActions, &myMonitor);



// map the senses requested by the planner with those available from the robot
int MySenses::readSense(const Instinct::instinctID nSense)
{
  int nRtn = 0;
  
  switch ( nSense )
  {
    case SENSE_FRONT_RIGHT:
      nRtn = sensors.getCornerDistance(R5_FRONT_RIGHT);
      break;
    case SENSE_FRONT_LEFT:
      nRtn = sensors.getCornerDistance(R5_FRONT_LEFT);
      break;
    case SENSE_REAR_LEFT:
      nRtn = sensors.getCornerDistance(R5_REAR_LEFT);
      break;
    case SENSE_REAR_RIGHT:
      nRtn = sensors.getCornerDistance(R5_REAR_RIGHT);
      break;
    case SENSE_FRONT:
      nRtn = sensors.getEdgeDistance(R5_FRONT);
      break;
    case SENSE_REAR:
      nRtn = sensors.getEdgeDistance(R5_REAR);
      break;
    case SENSE_LEFT:
      nRtn = sensors.getEdgeDistance(R5_LEFT);
      break;
    case SENSE_RIGHT:
      nRtn = sensors.getEdgeDistance(R5_RIGHT);
      break;
    case SENSE_NEAREST_CORNER:
      nRtn = sensors.nearestCorner();
      break;
    case SENSE_NEAREST_EDGE:
      nRtn = sensors.nearestEdge();
      break;
    case SENSE_RANDOM: // return 1-100
      nRtn = random(100)+1;
      break;
    case SENSE_SLEEPING:
      nRtn = sensors.getPause();
      break;
    case SENSE_FIFTY:
      nRtn = 50;
      break;
    case SENSE_RANGE:
      nRtn = myRanger.measureRange(); // this will take up to 30mS to return
      break;
    case SENSE_FRONT_RANGE: // this is the instantaneous range that we can see ahead using the IR sensors and the sense matrix if its ready
      if (myHead.senseHMatrixReady(0))
        nRtn = myHead.getHMinRange(0); // the actual min range ahead as detected by the ultrasonic scanner
      else
        nRtn = sensors.getRange(); // the max range that the corner sensors can return
      // try and return the minimum distance to obstacles that are ahead right now
      nRtn = min(sensors.getCornerDistance(R5_FRONT_RIGHT), nRtn);
      nRtn = min(sensors.getCornerDistance(R5_FRONT_LEFT), nRtn);
      nRtn = min(sensors.getEdgeDistance(R5_FRONT), nRtn);
      break;
    case SENSE_MIN_RANGE_AHEAD: // this is the range of free space we can sense ahead. May be out of date depending on speed and scan rate
      if (!myHead.senseHMatrixReady(0)) // if the scan is not ready then I cannot see ahead
        nRtn = 0;
      else     
        nRtn = myHead.getHMinRange(0);
      break;
    case SENSE_PIR:
      nRtn = myPIR.activated();
      break;
    case SENSE_MOTOR_CURRENT:
      nRtn = motors.getMotorCurrent(0);
      break;
    case SENSE_MOVING_HSCANINTERVAL:
      if (motors.getSpeed() > 0) // if we are moving forwards or backwards. We might still be rotating
      {
        nRtn = myHead.getHScanInterval();
        if (!nRtn) // zero means no scanning so this is effectively a large interval
          nRtn = 32000;
      }    
      break;
    case SENSE_STOPPED_VSCANINTERVAL:
      if ((motors.getSpeed() == 0) && (motors.getRudder() == 0)) // we must not be moving at all
      {
        nRtn = myHead.getVScanInterval();
        if (!nRtn)
          nRtn = 32000;
      }    
      break;
    case SENSE_HSCANREADY: // check if we have scan values for looking ahead
      nRtn = myHead.senseHMatrixReady(0);
      break;
    case SENSE_VSCANREADY: // check if we have scan values for looking up and down ahead
      nRtn = myHead.senseVMatrixReady(2);
      break;
    case SENSE_SCANREADY: // check if we have scan values for all cells
      nRtn = myHead.senseMatrixReady();
      break;
    case SENSE_HUMAN_AHEAD: // returns 1 if there 'might' be a human ahead
      if (myActions.confirmedHuman() || (myPIR.activated() && myHead.senseHMatrixReady(0) &&
          (myHead.getHMinRange(0) <= MAX_DIST_FOR_HUMAN) && (motors.getDistanceTravelled() >= MIN_TRAVEL_BETWEEN_HUMANS)))
      {
        nRtn = 1;
      }
      break;
    case SENSE_CONFIRMED_HUMAN: // return the result of ACTION_CONF_HUMAN 0 = not detected, 1 = maybe, 2 = confirmed
      nRtn = myActions.confirmedHuman();
      break;
  }
  return nRtn;
}

// map the actions available to the planner with those provided by the robot
unsigned char MyActions::executeAction(const Instinct::actionID nAction, const int nActionValue, const unsigned char bCheckForComplete)
{
  unsigned char bRtn = INSTINCT_ERROR;
  
  switch(nAction)
  {
    case ACTION_SETSPEED:
      bRtn = motors.setSpeed(nActionValue);
      break;
    case ACTION_MOVEBY:
      bRtn = motors.move(nActionValue, bCheckForComplete);
      break;
    case  ACTION_TURN:
      bRtn = motors.stopAndRotate(nActionValue, bCheckForComplete);
      break;
    case ACTION_STOP:
      bRtn = motors.stop();
      break;
    case ACTION_SLEEP:
      bRtn = robotSleep(nActionValue, bCheckForComplete);
      break;
    case ACTION_WAKE:
      bRtn = robotWake(bCheckForComplete);
      break;
    case ACTION_WAIT:
      bRtn = robotWait(nActionValue, bCheckForComplete);
      break;
    case ACTION_HMOVEHEAD:
      servoHHead.write(nActionValue);
      bRtn = INSTINCT_SUCCESS;
      break;
    case ACTION_VMOVEHEAD:
      servoVHead.write(nActionValue);
      bRtn = INSTINCT_SUCCESS;
      break;
    case ACTION_FAIL:
      bRtn = INSTINCT_FAIL;
      break;
    case ACTION_HSCAN:
      if (!myHead.getHScanInterval()) // if we've not been scanning horizontally, the old values are invalid
      {
        myHead.lookNearestSide();
        myHead.clearSenseMatrix();
      }
      if (nActionValue != myHead.getHScanInterval()) // only change parameters if they are not already set
      {
        myHead.setVScanInterval(0); // stop vertical scanning
        myHead.setHScanInterval(nActionValue); // start nActionValue mS sweeps
      }  
      bRtn = INSTINCT_SUCCESS;
      break;
    case ACTION_VSCAN:
      if (!myHead.getVScanInterval()) // if we've not been scanning vertically, the old values are invalid
      {
        myHead.lookNearestSide();
        myHead.clearSenseMatrix();
      }
      if (nActionValue != myHead.getVScanInterval()) // only change parameters if they are not already set
      {
        myHead.setHScanInterval(0); // stop horizontal scanning
        myHead.setVScanInterval(nActionValue); // start nActionValue mS sweeps
      }  
      bRtn = INSTINCT_SUCCESS;
      break;
    case ACTION_SCAN:
      if (!myHead.getHScanInterval() || !myHead.getVScanInterval()) // if we've not been scanning, the old values are invalid
      {
        myHead.lookNearestSide();
        myHead.clearSenseMatrix();
      }
      {
        // calculate intervals for a sawtooth scan pattern.
        unsigned int nVScanInterval = nActionValue * myHead.getVCells();
        unsigned int nHScanInterval = nVScanInterval * myHead.getHCells(); 
        if (nHScanInterval != myHead.getHScanInterval()) // only change parameters if they are not already set
        {
          myHead.setHScanInterval(nHScanInterval);
        }  
        if (nVScanInterval != myHead.getVScanInterval()) // only change parameters if they are not already set
        {
          myHead.setVScanInterval(nVScanInterval);
        }
      } 
      bRtn = INSTINCT_SUCCESS;
      break;
    case ACTION_TURNTOMOSTOPEN: // quite complex. It reads the most open way forward, stops and turns towards it
       // nAction value determines which vertical row of sensor cells to look at
       // only works if we've been scanning ahead long enough.
       if (!bCheckForComplete) // if this is the first time through
       {
         unsigned char bVCoord = min(nActionValue, myHead.getVCells());
         _nNewHeading = 0-(myHead.getHMostOpenAngle(bVCoord)-75); // fudge here as 75' is straight ahead as head is wonky (should be 90')
       }
       bRtn = motors.stopAndRotate(_nNewHeading, bCheckForComplete);
       break;
    case ACTION_TURNMOSTOPENDIR: // Reads the most open way forward (left or right), and turns nActionValue degrees that way. Uses infrared to get instantaneous values  
       if (!bCheckForComplete)
       {
          unsigned char bCorner = sensors.nearestCorner();
          switch (bCorner)
          {
            case R5_FRONT_LEFT:
              _nNewHeading = nActionValue;
              break;
            case R5_FRONT_RIGHT:
              _nNewHeading = -nActionValue;
              break;
            default:
              _nNewHeading = (random(1) > 0) ? nActionValue: -nActionValue;
              break;
          }
       }
       bRtn = motors.stopAndRotate(_nNewHeading, bCheckForComplete);
       break;
    case ACTION_WAIT_HSCANREADY:
      bRtn = robotWait(myHead.getHScanInterval()+1000, bCheckForComplete);
      if (myHead.senseHMatrixReady(nActionValue))
        bRtn = INSTINCT_SUCCESS;
      else if (bRtn == INSTINCT_SUCCESS) // this is success from the wait
        bRtn = INSTINCT_FAIL; // timeout
      break;
    case ACTION_WAIT_VSCANREADY:
      bRtn = robotWait(myHead.getVScanInterval()+1000, bCheckForComplete);
      if (myHead.senseVMatrixReady(nActionValue))
        bRtn = INSTINCT_SUCCESS;
      else if (bRtn == INSTINCT_SUCCESS) // this is success from the wait
        bRtn = INSTINCT_FAIL; // timeout
      break;
    case ACTION_WAIT_SCANREADY: // wait until the scan is ready, or timeout
      bRtn = robotWait(max(myHead.getHScanInterval(), myHead.getVScanInterval())+1000, bCheckForComplete);
      if (myHead.senseMatrixReady())
        bRtn = INSTINCT_SUCCESS;
      else if (bRtn == INSTINCT_SUCCESS) // this is success from the wait
        bRtn = INSTINCT_FAIL; // timeout
      break;
    case ACTION_RESET_HUMAN_DETECTOR:
      motors.resetDistanceTravelled();
      _bConfirmedHuman = 0;
      bRtn = INSTINCT_SUCCESS;
      break;
   case ACTION_CONF_HUMAN:
      if (!bCheckForComplete) // cycle starts - no human detected yet
      {
        _bConfirmedHuman = 0;
        // don't entice human just yet as we need PIR to clear
        displayClear();
      }
      if (!myPIR.activated()) // step 1 - PIR now ready to detect a human
        _bConfirmedHuman = 1;
      else if (_bConfirmedHuman == 1) // step 2 - human confirmed so no need to wait any longer
        _bConfirmedHuman = 2;
        
      if (_bConfirmedHuman == 2) // no need to wait further
        bRtn = INSTINCT_SUCCESS;
      else
      {
        bRtn = robotWait(nActionValue, bCheckForComplete);
        if ((bRtn == INSTINCT_SUCCESS)) // check PIR at timeout
          if (myPIR.activated())
            _bConfirmedHuman = 2;
          else
            _bConfirmedHuman = 0;
        else
          displayRainbow();   
      }
      if (bRtn == INSTINCT_SUCCESS)
        displayClear();      
      break;
   case ACTION_FLASH_COLOUR: // lowest 3 bits are RGB, remainder is delay in mS
      flashColour(nActionValue & 0x07);
      bRtn = robotWait(nActionValue & 0xFFF8, bCheckForComplete);
      if (bRtn == INSTINCT_SUCCESS)
        displayClear();       
      break;
}
  
  return INSTINCT_RTN_COMBINE(bRtn, nAction);
}

// puts the robot to sleep for nSellTime seconds (note wait uses mS, but this is Seconds)
unsigned char MyActions::robotSleep(const int nSleepTime, const unsigned char bCheckForComplete)
{
  unsigned char bSuccess;
  unsigned long uiMillis = millis();

  if (_sleepStart > uiMillis) // avoid huge sleep on rollover
    _sleepStart = 0L;
  
  if (!bCheckForComplete && nSleepTime)
  {
    motors.stop(); // stop the robot
    sensors.setPause(true); // pause the sensors to save power
    myHead.setHScanInterval(0); // stop the head scanning
    myHead.setVScanInterval(0);
    _sleepStart = uiMillis;
    bSuccess = INSTINCT_IN_PROGRESS;
  }
  else if (!nSleepTime || (uiMillis > (_sleepStart + ((unsigned long)nSleepTime * 1000L))))
  {
    sensors.setPause(false); // enable the sensors
    _sleepStart = 0L;
    bSuccess = INSTINCT_SUCCESS;
  }
  else
  {
    bSuccess = INSTINCT_IN_PROGRESS;
  }
  return bSuccess;
}

unsigned char MyActions::robotWake(const unsigned char bCheckForComplete)
{
  sensors.setPause(false);
  _sleepStart = 0;
  return INSTINCT_SUCCESS;
}

// this just waits for nWaitTime mS, useful for waiting while scanning around
unsigned char MyActions::robotWait(const int nWaitTime, const unsigned char bCheckForComplete)
{
  unsigned char bSuccess;
  unsigned long uiMillis = millis();

  if (_waitStart > uiMillis) // avoid huge sleep on rollover
    _waitStart = 0L;
  
  if (!bCheckForComplete && nWaitTime)
  {
    _waitStart = uiMillis;
    bSuccess = INSTINCT_IN_PROGRESS;
  }
  else if (!nWaitTime || (uiMillis > (_waitStart + (unsigned long)nWaitTime)))
  {
    _waitStart = 0L;
    bSuccess = INSTINCT_SUCCESS;
  }
  else
  {
    bSuccess = INSTINCT_IN_PROGRESS;
  }
  return bSuccess;
}

unsigned char MyMonitor::nodeExecuted(const Instinct::PlanNode * pPlanNode)
{
  writeOutput(pPlanNode, "E", NULL, 0);
  return true;
}

unsigned char MyMonitor::nodeSuccess(const Instinct::PlanNode * pPlanNode)
{
  writeOutput(pPlanNode, "S", NULL, 0);
  return true;
}

unsigned char MyMonitor::nodeInProgress(const Instinct::PlanNode * pPlanNode)
{
  writeOutput(pPlanNode, "P", NULL, 0);
  return true;
}

unsigned char MyMonitor::nodeFail(const Instinct::PlanNode * pPlanNode)
{
  writeOutput(pPlanNode, "F", NULL, 0);
  return true;
}

unsigned char MyMonitor::nodeError(const Instinct::PlanNode * pPlanNode)
{
  writeOutput(pPlanNode, "Z", NULL, 0);
  return true;
}

unsigned char MyMonitor::nodeSense(const Instinct::ReleaserType *pReleaser, const int nSenseValue)
{
  writeOutput(NULL, "R", pReleaser, nSenseValue);
  return true;
}


// copies node type names from PROGMEM to save RAM
char * getNodeTypeName(char *pStrBuff, const int nBuffLen, const unsigned char bNodeType)
{
  return getProgmemStr(pStrBuff, nBuffLen, bNodeType, szNodeType);
}

// access a string from progBuff with offset nStrOffset. Strings separated by '!'
char * getProgmemStr(char *pStrBuff, const int nBuffLen, const unsigned char nStrOffset, const char *progBuff)
{
  char *pBuff = pStrBuff;
  int i = 0;
  int j = 0;
  int len = 1;
  unsigned char ch;

  if (!pStrBuff || (nBuffLen < 1) || !progBuff) // no buffers
    return pBuff;
  
  *pBuff = 0; // ensure the return string is always zero terminated
  while (ch = pgm_read_byte_far(progBuff + i))
  {
    i++;
    
    if (len == nBuffLen)
    {
      break;
    }
    else if (ch == '!')
    {
      j++;
      *pBuff = 0;
    }
    else if (j == nStrOffset)
    {
      *pBuff = ch;
      pBuff++;
      len++;
      *pBuff = 0; // ensure the return string is always zero terminated
    }
    else if (j > nStrOffset)
      break;
  }
  
  return pStrBuff;
}

// search for a string from progBuff . If found return 0 based index. If not return -1. Strings separated by '!'
int findProgmemStr(const char *pStrBuff, const char *progBuff)
{
  char *pBuff;
  int i = 0;
  int nString = 0;
  unsigned char bMatch;
  unsigned char ch;

  if (!pStrBuff || !progBuff) // no buffer
    return -1;
  
  pBuff = (char *)pStrBuff;
  bMatch = 1;
  while (ch = pgm_read_byte_far(progBuff + i))
  {
    i++;

    if (bMatch && (ch == '!') && (*pBuff == 0))
    {
      // strings matched to the end
      return nString;
    }
    else if (ch == '!')
    {
      // start on the next string
      nString++;
      pBuff = (char *)pStrBuff;
      bMatch = 1;
    }
    else if (bMatch && (*pBuff == ch)) // matching so far
    {
      pBuff++;
    }
    else
      bMatch = 0;    
  }
  
  return -1;
}



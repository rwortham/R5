//  R5 Robot Platform
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

#include <Debug.h>
#include <avr/wdt.h>
#include <Servo.h> // adding this include uses 139 bytes of RAM somehow
#include <SoftwareSerial.h> // adding this uses 68 bytes of RAM
#include <Wire.h> // adding this uses 207 bytes of RAM
#include <SPI.h> // this uses no RAM
#include <Encoder.h>
#include <WiFly.h> // this uses no RAM
#include <EasyVR.h> // this uses no RAM
#include <RTClib.h> // this uses no RAM
#include <Adafruit_NeoPixel.h> // uses heap to store 3 bytes per pixel
#include <EEPROM.h>
#include <Instinct.h>
#include <R5.h>


//#define R5_EASYVR 1 // define this only if easyVR voice recognition module installed - code currently only initialises it, but does not use it

R5EEPROM myMemory;

// this structure defined in R5EEPROM.h stores the remote server connection parameters    
R5ServerParamsType myServerParams;

/*char szWifiSSID {"InstinctWifi"}
char szWifiPassword {"64842431"}
char szServerIP {"192.168.137.1"}
int nPort = 3000;
*/

        
// variables defined in Robot_Instinct.ino
extern Instinct::CmdPlanner myPlan;
extern Instinct::Names myNames;
extern MyMonitor myMonitor;


// defined in other files
boolean initialiseWifi(const char *pSSID, const char *pPassword, const unsigned char bRetry);
boolean tcpConnect(const char * pIPAddress, const int nPort, const unsigned char bRetry);
void displayRainbow(void);
void flashColour(unsigned char bRGB);
void displayClear(void);
unsigned char initialiseVocaliser(const char *pFirstWords);


boolean addElementName(Instinct::instinctID bRuntime_ElementID, char *pElementName);

// defines for the Ultrasonic rangefinder
#define ULTRASONIC_PIN 8
#define ULTRASONIC_MIN_INTERVAL 300

#define PIR_PIN 9

// functions defined in this file
void processSerial(void);
unsigned char parseRobotCommand(const char *pCmd);
void writeOutput(const Instinct::PlanNode * pPlanNode, const char *pType, const Instinct::ReleaserType *pReleaser, const int nSenseValue);
void reportSensorValues(void);
void processWifi(void);

class MyOutput : public R5Output {
public:
 virtual void outputData(const char *pszData);
 virtual void outputVocaliseData(const char *pszData);
};

MyOutput myOutput;

// store strings in flash memory to save RAM. Accessed via getProgmemStr
#define R5_MSG_BUFF_SIZE 100
const char PROGMEM szRobotMessages[] = "Use HELP for command options.!Invalid Command !PLAN commands: A D M R S U!RESET in 1s, then hang, Damn it.!" //
                                        "Wifi Parameters Sucessfully Updated.!";
const char PROGMEM szRobotSetupMessages[] = "RTC not running.!Wire1.begin()!Robot Initialising ...!EasyVR not detected.!Robot Running ...!" //
                                        "Established InstinctServer Connection.!Established Wifi Connection!Loading Plan from EEPROM!Emic2 Not Detected!" //
                                        "Hello. This is the R5 Robot. Please Watch, and Listen Carefully.!" //
                                        "Hello. I am Buddy the Robot. Nice to meet you. Please Watch, and Listen Carefully.!";

// ******** these are the static structures that comprise the robot *********

// a global byte of flags. Bits 0/1 - enable monitor output to Serial/Wifi. Bit 2 - enable sensor reporting at plan rate. Bit 3 - enable HeadMatrix reporting
// Bit 4 - attempt Wifi & Instinct Server Connection on boot, Bit 5 - attempt restore of plan from EEPROM on boot
// Bit 6 - enable global plan monitoring, Bit 7 - enable vocalisation
// Bit 8 - load stored vocalisation params on boot
// Bit 9 - enable reporting of plan monitor data
// Bit 10 - enable reporting of vocalisation data


unsigned int uiGlobalFlags = 0x13; // Default just output to Serial, Wifi & Instinct Server connection on boot

// the number of plan cycles per second - set using the RATE command
unsigned int uiPlanRate = 10;

// 8 pixel RGB display
#define PIXEL_PIN 10
Adafruit_NeoPixel myPixelStrip = Adafruit_NeoPixel(8, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// the servos that control head movement
Servo servoHHead;
Servo servoVHead;

// real time clock module
RTC_DS1307 rtc;

// Wifi module
WiFly wifly(&Serial2);

// Text to speech output
R5Voice myVoice(&Serial3);

// Voice Recognition module
#ifdef R5_EASYVR
SoftwareSerial VRPort(12,13);
EasyVR easyVR(VRPort);
#endif

// sensor structures
const unsigned char cornerInputs[] = {A0, A1, A2, A3};
const unsigned char cornerOutputs[] = {24, 25, 26, 27};
R5CornerSensors sensors(cornerInputs, cornerOutputs); 
R5Ultrasonic myRanger(ULTRASONIC_PIN, ULTRASONIC_MIN_INTERVAL);
R5PIR myPIR(PIR_PIN);

  
// quadrature detectors for motors. Note the encoder values are read and passed to the R5MotorControl as part of the main execution loop
Encoder leftEncoder(19, 23);
Encoder rightEncoder(18, 22);

// motor structures
const unsigned char motorSpeeds[] = {4, 5};
const unsigned char motorDirections[] = {29, 28};
const unsigned char motorCurrents[] = {A4, A5};
R5MotorControl motors(motorSpeeds, motorDirections, motorCurrents);

// Head Controller central head position is 75' and forward looking is 180'
// 12H*3V array of sensor readings, smoothing set to take weighted average of current and last readings
// 10% of old value and 90% of new one
R5SensingHead myHead(&servoHHead, &servoVHead, 75, 180, &myRanger, 5, 2, 10);

// the setup routine runs once when you press reset:
void setup()
{
    char szMsgBuff[R5_MSG_BUFF_SIZE];

    wdt_reset(); // this code fails because on reboot the Watchdog is set to 15mS and reboots too quickly, before it gets this far.
    wdt_disable(); // the watchdog is used by the RESET command, so we must disable it on reboot
    Serial.begin(115200);    
   
    myPixelStrip.begin();
    displayClear();

    myMemory.getServerParams(&myServerParams);
    uiGlobalFlags = myMemory.getGlobalFlags();
    uiPlanRate = myMemory.getPlanRate();

    displayRainbow(); // start the rainbow effect

    Serial.println(getProgmemStr(szMsgBuff, sizeof(szMsgBuff), 2, szRobotSetupMessages));
    delay(2000); // give the boards a chance to initialise after power on
    
    // check if we must try to connect to Instinct Server
    if (uiGlobalFlags & 0x10)
      connectInstinctServer(szMsgBuff);
    else
      uiGlobalFlags = uiGlobalFlags & 0xFFFD; // don't to write the the wifi board if its not connected         


#ifdef AVR
    Wire.begin(); // the SPI bus is needed to talk to the RTC
#else
    myOutput.outputData(getProgmemStr(szMsgBuff, sizeof(szMsgBuff), 1, szRobotSetupMessages));
    Wire1.begin(); // Shield I2C pins connect to alt I2C bus on Arduino Due
#endif

    displayRainbow();
    
    rtc.begin();
    if (! rtc.isrunning()) {
      myOutput.outputData(getProgmemStr(szMsgBuff, sizeof(szMsgBuff), 0, szRobotSetupMessages));
    }
    else
    {
      // following line sets the RTC to the date & time this sketch was compiled
      // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      // This line sets the RTC with an explicit date & time, for example to set
      // January 21, 2014 at 3am you would call:
      // rtc.adjust(DateTime(2015, 1, 14, 12, 30, 0));
    }
    // set the rtc LED to flash at 1Hz
    rtc.writeSqwPinMode(SquareWave1HZ);

    displayRainbow();
   
    // attach the head servos to the correct pins
    servoHHead.attach(6);
    servoVHead.attach(7);
    // define how we are going to scan horizontally and vertically, and stop it for now
    myHead.setHScanParams(30, 15, 135); // - 120' sweep, so each sense cell is 30'. Centre is 75' not 90'
    myHead.setVScanParams(45, 135, 180); // just divide V sweep into two
    myHead.setHScanInterval(0);  
    myHead.setVScanInterval(0);  
    myHead.lookAhead();
    myHead.setParalyse(true); // stop head moving    
    motors.setParalyse(true); // stop robot moving until its sensors are working
    
#ifdef R5_EASYVR
    displayRainbow();

    // start the interface to the Voice Recognition module
    VRPort.begin(9600);
    if (!easyVR.detect())
    {  
        myOutput.outputData(getProgmemStr(szMsgBuff, sizeof(szMsgBuff), 3, szRobotSetupMessages));
    }
#endif

    displayRainbow();

    // start the Emic2 Text to Speech module
    Serial3.begin(9600);
    // volume 18 = max, default is 0, min is -48
    // speed = 180, default is 200, range 75-600
    // voice 3 = Uppity Ursula, voice 2 = Beautiful Betty
    if (!myVoice.initialiseVoice(18, 180, 0))
    {
        myOutput.outputData(getProgmemStr(szMsgBuff, sizeof(szMsgBuff), 8, szRobotSetupMessages));
    }

    displayRainbow();

    // say hello if I'm speaking
    if(uiGlobalFlags & 0x80)
    {
#ifdef R5_BUDDY
        myVoice.speak(getProgmemStr(szMsgBuff, sizeof(szMsgBuff), 10, szRobotSetupMessages), 3000, false, 0, true);
#else
        myVoice.speak(getProgmemStr(szMsgBuff, sizeof(szMsgBuff), 9, szRobotSetupMessages), 3000, false, 0, true);
#endif
    }
    
    displayRainbow();

    // check if we should restore the plan from EEPROM on boot
    if(uiGlobalFlags & 0x20)
    {
      myOutput.outputData(getProgmemStr(szMsgBuff, sizeof(szMsgBuff), 7, szRobotSetupMessages));
      myMemory.readData(&myPlan, myNames.elementBufferSize(), myNames.elementBuffer());

      if(uiGlobalFlags & 0x40) // turn on global plan monitoring
      {
        myPlan.setGlobalMonitorFlags(1, 1, 1, 1, 1, 1);
      }
    }

    displayRainbow();

    if(uiGlobalFlags & 0x100) // restore the speak rules from EEPROM on boot
    {
      myMemory.getSpeakRules(myMonitor.getSpeakRule(0,0));
    }
}

// connect to the InstinctServer
unsigned char connectInstinctServer(char *pMsgBuff)
{
    // disable the flag that writes monitor output to wifi. Stops the wifi commands being interrupted by outputData()
    uiGlobalFlags = uiGlobalFlags & 0xFFFD;         
    if (initialiseWifi(myServerParams.szWifiSSID, myServerParams.szWifiPassword, myServerParams.bWifiRetry))
    {
      myOutput.outputData(getProgmemStr(pMsgBuff, R5_MSG_BUFF_SIZE, 6, szRobotSetupMessages));
      if(tcpConnect(myServerParams.szServerIP, myServerParams.uiServerPort, myServerParams.bServerRetry))
      {
        uiGlobalFlags = uiGlobalFlags | 0x02; // enable the flag that writes monitor output to wifi
        myOutput.outputData(getProgmemStr(pMsgBuff, R5_MSG_BUFF_SIZE, 5, szRobotSetupMessages));
        return true;
      }
    }
    return false;
}


// the loop routine runs over and over again forever:
void loop()
{
    static unsigned long ulOldTimerMilliSecs = 0;
    static unsigned long ulOldRateMilliSecs = 0;
    static unsigned long ulOldSenseMilliSecs = 0;
    static int nStartupLoopCounter = 0;
  
    unsigned long ulMilliSecs = millis();

    if (ulMilliSecs < ulOldTimerMilliSecs) // rollover after ~ 50 days so handle this just for completeness
      ulOldTimerMilliSecs = 0;
    
    if (ulMilliSecs < ulOldRateMilliSecs) // rollover after ~ 50 days so handle this just for completeness
      ulOldRateMilliSecs = 0;
    
    if (ulMilliSecs < ulOldSenseMilliSecs) // rollover after ~ 50 days so handle this just for completeness
      ulOldSenseMilliSecs = 0;

    if ( nStartupLoopCounter < 100 ) // count the first 100 loops
      nStartupLoopCounter++;

    if ( nStartupLoopCounter < 50 )
    {
      sensors.sense(); // start sensing immediately
      displayRainbow(); // show the rainbow effect during the startup routine
      delay(25); // do this for the first ~ 2 seconds
    }
    else if ( nStartupLoopCounter == 50 ) // calibrate the sensors and enable motors after 50 loops
    {
       char szMsgBuff[20];
       sensors.calibrateBleed();
       motors.setParalyse(false);
       myHead.setParalyse(false);
       // report that the robot is running - this initiates cmdfile.txt in InstinctServer
       myOutput.outputData(getProgmemStr(szMsgBuff, sizeof(szMsgBuff), 4, szRobotSetupMessages));
       displayClear(); // clear the display to indicate we are ready to go
    }  
    else if ( nStartupLoopCounter > 50 ) // ready to execute behaviours
    {
      // call processTimers once per second
      if ((ulMilliSecs - ulOldTimerMilliSecs) >= 1000)
      {
        ulOldTimerMilliSecs = ulMilliSecs;
        myPlan.processTimers(1);
      }

      // IR sense at 8 times the plan rate to get current sensor data
      unsigned int uiSensorRate = uiPlanRate ? uiPlanRate : 1;
      if ((ulMilliSecs - ulOldSenseMilliSecs) >= (125/uiSensorRate))
      {
        ulOldSenseMilliSecs = ulMilliSecs;
        sensors.sense();
      }

      if (uiPlanRate) // if uiPlanRate is zero then we do not run the plan at all
      {
        if ((ulMilliSecs - ulOldRateMilliSecs) >= (1000/uiPlanRate))
        {
          ulOldRateMilliSecs = ulMilliSecs;
          if (uiGlobalFlags & 0x04)
          {
            reportSensorValues();
          }
          if (uiGlobalFlags & 0x08)
          {
            reportHeadMatrix();
          }
          myPlan.runPlan();
          // displayRainbow(); // enable just for testing
          // flashColour(0x06); // 6 = yellow
        }
      }
    }

    motors.driveMotors(leftEncoder.read(), rightEncoder.read());
    myHead.driveHead();

    processSerial();
    processWifi();

    myVoice.processVoice();
}

// read the Serial port and process commands for the robot
void processSerial()
{
  static char cmd[80];
  static unsigned char cmdLen = 0;

    while ( Serial.available() > 0 )
    {
      char c = Serial.read();
      if (c == 10) // NL
      {
        cmd[cmdLen] = 0; // zero term the string
        myOutput.outputData(cmd);
        parseRobotCommand(cmd);
        cmdLen = 0;
      }
      else
      {
        cmd[cmdLen] = c;
        cmdLen++;
        if (cmdLen > (sizeof(cmd)-1)) // avoid buffer overrun
          cmdLen = 0;
      }      
    }
}


// read the wifi port and process commands for the robot
void processWifi()
{
  static char cmd[80];
  static unsigned char cmdLen = 0;

    while ( wifly.available() > 0 )
    {
      char c = wifly.read();
      if (c == 10) // NL
      {
        cmd[cmdLen] = 0; // zero term the string
        myOutput.outputData(cmd);
        parseRobotCommand(cmd);
        cmdLen = 0;
      }
      else
      {
        cmd[cmdLen] = c;
        cmdLen++;
        if (cmdLen > (sizeof(cmd)-1)) // avoid buffer overrun
          cmdLen = 0;
      }      
    }
}



// Process the various types of commands PLAN, STOP, START, RESET, DUMP
// 
unsigned char parseRobotCommand(const char *pCmd)
{
  char szCmd[20];
  char szMsgBuff[R5_MSG_BUFF_SIZE];
  unsigned char bRtn = true;
  unsigned char bSayOK = false;

  static const char PROGMEM szFmt[] = {"%s "};
  szCmd[0] = 0;
  int nRtn = sscanf_P(pCmd, szFmt, szCmd);

  if (!nRtn || !strlen(szCmd))
  {
    myOutput.outputData(getProgmemStr(szMsgBuff, sizeof(szMsgBuff), 0, szRobotMessages));
    return false;
  }

  static const char PROGMEM szCommands[] = {"PLAN!STOP!START!RESET!DUMP!TIME!SETTIME!REPORT!RATE!CAL!CON!PELEM!RSENSE!RACTION!HSTOP!HSTART!"
            "SPLAN!RPLAN!SCONF!RCONF!SWIFI!CONF!HELP!VER!SHOWIFI!SHOCONF!SHOREPORT!SHORATE!SHONAMES!SPEAKRULE!SHORULES!SRULES!RRULES!CNAMES!"};

  strupr(szCmd); // make command words case insensitive
  nRtn = findProgmemStr(szCmd, szCommands);

  switch (nRtn)
  {
  case 0: // PLAN - execute one of the Instinct CmdPlanner commands
     if (strlen(pCmd) > (strlen(szCmd)+1))
     {
        bRtn = myPlan.executeCommand(pCmd + strlen(szCmd)+1, szMsgBuff, sizeof(szMsgBuff));
        myOutput.outputData(szMsgBuff);
     }
     else
     {
        myOutput.outputData(getProgmemStr(szMsgBuff, sizeof(szMsgBuff), 2, szRobotMessages));
        bRtn = false;
     }
     bSayOK = false;      
     break;
  case 1: // STOP - Disable the robot motors
      motors.setParalyse(true);
      bSayOK = true;
      break;
  case 2: // START - Enable the robot motors
      motors.setParalyse(false);
      bSayOK = true;
      break;
  case 3: // RESET - this annoyingly does not work due to watchdog resetting too fast after reboot - needs a bootloader fix
      myOutput.outputData(getProgmemStr(szMsgBuff, sizeof(szMsgBuff), 3, szRobotMessages));
      wdt_enable(WDTO_1S);
      bSayOK = false;      
      break;
  case 4: // DUMP - Dump a complete listing of the Instinct Plan
      {
        Instinct::PlanNode aNode;
        Instinct::instinctID nMaxID = myPlan.maxElementID();
        for (Instinct::instinctID i = 0; i < nMaxID; i++)
        { 
          static const char PROGMEM szFmt[] = {"D N %i"};
          snprintf_P(szCmd, sizeof(szCmd), szFmt, i+1);
          if (myPlan.executeCommand(szCmd, szMsgBuff, sizeof(szMsgBuff)))
          {
            myOutput.outputData(szMsgBuff);
          }
        }
        bSayOK = nMaxID ? 0 : 1; // just say OK if the plan is empty
      }    
      break;
  case 5: // TIME - Report the time
      {
        DateTime now = rtc.now();
        static const char PROGMEM szFmt[] = {"%04u/%02u/%02u %02u:%02u:%02u"};
        snprintf_P(szMsgBuff, sizeof(szMsgBuff), szFmt, now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
        myOutput.outputData(szMsgBuff);
        bSayOK = false;            
      }
      break;
  case 6: // SETTIME - Set the time
      if (strlen(pCmd) > (strlen(szCmd)+1))
      {
        int nYr, nMth, nDay, nHr, nMin, nSec;
        nYr = nMth = nDay = nHr = nMin = nSec = 0;
        static const char PROGMEM szFmt[] = {"%u %u %u %u %u %u"};
        sscanf_P(pCmd + strlen(szCmd), szFmt, &nYr, &nMth, &nDay, &nHr, &nMin, &nSec);
        rtc.adjust(DateTime(nYr, nMth, nDay, nHr, nMin, nSec));
        bSayOK = true;           
      }
      break;
  case 7: // REPORT - Turn on/off reporting to Serial port and wifi and also sensor value reporting
          // e.g. REPORT 1 0 1 0 enables Serial, Disables Wifi, enables sensor value reporting, disables reporting of the head matrix
          // uiGlobalFlags Bit 0 - write to Serial, Bit 1 - write to Wifi, Bit 2 - Report Sensors, Bit 3 - Report HeadMatrix
          // Bit 9 - enable reporting of plan monitor data, Bit 10 - enable reporting of vocalisation data

      if (strlen(pCmd) > (strlen(szCmd)+1))
      {
        int nSerial, nWifi, nSensors, nHeadMatrix, nReportPlan, nReportVocalise;
        nSerial = nWifi = nSensors = nHeadMatrix = nReportPlan = nReportVocalise = 0;
        static const char PROGMEM szFmt[] = {"%i %i %i %i %i %i"};
        sscanf_P(pCmd + strlen(szCmd), szFmt, &nSerial, &nWifi, &nSensors, &nHeadMatrix, &nReportPlan, &nReportVocalise);
        uiGlobalFlags = (uiGlobalFlags & 0xF9F0) | ((nSerial ? 0x01 : 0x0) | (nWifi ? 0x02 : 0x0) |
              (nSensors ? 0x04 : 0x0) | (nHeadMatrix ? 0x08 : 0x0) | (nReportPlan ? 0x0200 : 0x0) | (nReportVocalise ? 0x0400 : 0x0) );
        bSayOK = true;      
      }
      break;
  case 8: // RATE - how many times per second do we process the plan? 0 stops plan processing
      if (strlen(pCmd) > (strlen(szCmd)+1))
      {
        uiPlanRate = 0;
        static const char PROGMEM szFmt[] = {"%u"};
        sscanf_P(pCmd + strlen(szCmd), szFmt, &uiPlanRate);
        bSayOK = true;      
      }
      break;
  case 9: // CAL - recalibrate sensors
      sensors.calibrateBleed();    
      bSayOK = true;      
      break;
  case 10: // CON - connect to wifi - useful if server started after robot is booted
      bSayOK = true;
      if(bRtn = tcpConnect(myServerParams.szServerIP, myServerParams.uiServerPort, 1)) // just try once
      {
        uiGlobalFlags = uiGlobalFlags | 0x02; // enable the flag that writes monitor output to wifi
        myOutput.outputData(getProgmemStr(szMsgBuff, sizeof(szMsgBuff), 5, szRobotSetupMessages));
        bSayOK = false;
      }      
      break;
  case 11: // PELEM - associating a name with a plan element ID
      if (strlen(pCmd) > (strlen(szCmd)+1))
      {
        static const char PROGMEM szFmt[] = {"%s %u"};
        char szName[30];
        unsigned int uiRuntime_ElementID = 0;
        char * pParams = (char *)pCmd + strlen(szCmd);
        // allow both space and = as delimeter between element name and its ID
        char * pDelim = strchr(pParams, '=');
        if (pDelim)
          *pDelim = ' ';          
        sscanf_P(pParams, szFmt, szName, &uiRuntime_ElementID);
        bSayOK = myNames.addElementName((Instinct::instinctID)uiRuntime_ElementID, szName);      
      }
      break;
  case 12: // RSENSE - associating a name with a robot sense ID
      break;
  case 13: // RACTION - associating a name with a robot action ID
    // we just ignore this for now
      break;
  case 14: // HSTOP - stop robot head from scanning
      myHead.setParalyse(true);
      bSayOK = true;      
      break;
  case 15: // HSTART - allow robot head to scan
      myHead.setParalyse(false);
      bSayOK = true;      
      break;
  case 16: // SPLAN - save robot plan in EEPROM
      bRtn = myMemory.writeData(&myPlan, myNames.elementBufferSize(), myNames.elementBuffer());
      bSayOK = true;      
      break;
  case 17: // RPLAN - read robot plan from EEPROM
      bRtn = myMemory.readData(&myPlan, myNames.elementBufferSize(), myNames.elementBuffer());
      bSayOK = true;      
      break;
  case 18: // SCONF - save robot config in EEPROM
      myMemory.setServerParams(&myServerParams);
      myMemory.setGlobalFlags(uiGlobalFlags);
      myMemory.setPlanRate(uiPlanRate);
      bSayOK = true;      
      break;
  case 19: // RCONF - read robot config from EEPROM
      myMemory.getServerParams(&myServerParams);
      uiGlobalFlags = myMemory.getGlobalFlags();
      uiPlanRate = myMemory.getPlanRate();
      bSayOK = true;      
      break;
  case 20: // SWIFI - set the Wifi params - SSID, Password, Wifi Retries, ServerIP, ServerPort, ServerRetries
    bRtn = false;
    bSayOK = true;      
    if (strlen(pCmd) > (strlen(szCmd)+1))
    {
      unsigned int uiWifiRetry, uiServerRetry;
      uiWifiRetry = uiServerRetry = 1;
      static const char PROGMEM szFmt[] = {"%s %s %u %s %u %u"};
      if (sscanf_P(pCmd + strlen(szCmd), szFmt, myServerParams.szWifiSSID, myServerParams.szWifiPassword, &uiWifiRetry,
            myServerParams.szServerIP, &myServerParams.uiServerPort, &uiServerRetry) == 6)
      {
        myServerParams.bWifiRetry = uiWifiRetry;
        myServerParams.bServerRetry = uiServerRetry;
        myOutput.outputData(getProgmemStr(szMsgBuff, sizeof(szMsgBuff), 4, szRobotMessages));
        bSayOK = false;
        bRtn = true;
      } 
    }
    break;
  case 21: // CONF - sets config flags - Bit 4 - attempt Wifi & Instinct Server Connection on boot, Bit 5 - attempt restore of plan from EEPROM on boot
          // Bit 6 - enable full plan monitoring on boot, Bit 7 - vocalise monitor trace, Bit 8 - read speak rules from EEPROM on boot
      if (strlen(pCmd) > (strlen(szCmd)+1))
      {
        int nConWifi, nReadPlan, nMonitorPlan, nVocalise, nReadRules;
        nConWifi = nReadPlan = nMonitorPlan = nVocalise = nReadRules = 0;
        static const char PROGMEM szFmt[] = {"%i %i %i %i %i"};
        if ( sscanf_P(pCmd + strlen(szCmd), szFmt, &nConWifi, &nReadPlan, &nMonitorPlan, &nVocalise, &nReadRules) == 5)
        {
          uiGlobalFlags = (uiGlobalFlags & 0xFE0F) | ((nConWifi ? 0x10 : 0x0) | (nReadPlan ? 0x20 : 0x0) | (nMonitorPlan ? 0x40 : 0x0) | 
                (nVocalise ? 0x80 : 0x0) | (nReadRules ? 0x100 : 0x0));
          bSayOK = true;
        }      
      }
      break;
 case 22: // HELP - return help to the user
      if (strlen(pCmd) > (strlen(szCmd)+1))
      {
        char szOption[20];
        unsigned char nOption;
        static const char PROGMEM szFmt[] = {"%s"};
        szOption[0] = 0;
        sscanf_P(pCmd + strlen(szCmd), szFmt, szOption);
        strupr(szOption); // command words are case insensitive
        nOption = findProgmemStr(szOption, szCommands);
        bRtn = helpCommand(szMsgBuff, nOption);
      }
      else
      {
        char szOption[20];
        unsigned char nOption = 0;
        szMsgBuff[0] = 0;
        while (strlen(getProgmemStr(szOption, sizeof(szOption), nOption, szCommands)) > 0)
        {
          nOption++;
          bRtn = true;
          if ((strlen(szMsgBuff)+ strlen(szOption) + 3) > sizeof(szMsgBuff))
          {
            myOutput.outputData(szMsgBuff);
            szMsgBuff[0] = 0;
          }
          strncat(szMsgBuff, szOption, sizeof(szMsgBuff));
          strncat(szMsgBuff, " ", sizeof(szMsgBuff));         
        }
        if (strlen(szMsgBuff) > 0)
          myOutput.outputData(szMsgBuff);
      }
      bSayOK = !bRtn;
              
      break;
  case 23: // VER - report the date and time of last compilation
      {
        DateTime compileTime(F(__DATE__), F(__TIME__));
        static const char PROGMEM szFmt[] = {"%04u/%02u/%02u %02u:%02u:%02u"};
        snprintf_P(szMsgBuff, sizeof(szMsgBuff), szFmt, compileTime.year(), compileTime.month(), compileTime.day(),
                                  compileTime.hour(), compileTime.minute(), compileTime.second());
        myOutput.outputData(szMsgBuff);            
      }
      break;
 case 24: // SHOWIFI - report the current wifi params - SSID PW WifiRetry IP Port ServerRetry
      {
        static const char PROGMEM szFmt[] = {"%s %s %u %s %u %u"};
        snprintf_P(szMsgBuff, sizeof(szMsgBuff), szFmt, myServerParams.szWifiSSID, myServerParams.szWifiPassword, (unsigned int)myServerParams.bWifiRetry,
            myServerParams.szServerIP, myServerParams.uiServerPort, (unsigned int)myServerParams.bServerRetry);
        myOutput.outputData(szMsgBuff);            
      }
      break;
case 25: // SHOCONF - show config startup flags - ConnectWifi ReadPlan MonitorPlan Vocalise ReadSpeakRules
      {
        int nConWifi = (uiGlobalFlags & 0x10) ? 1 : 0;
        int nReadPlan = (uiGlobalFlags & 0x20) ? 1 : 0;
        int nMonitorPlan = (uiGlobalFlags & 0x40) ? 1 : 0;
        int nVocalise = (uiGlobalFlags & 0x80) ? 1 : 0;
        int nReadRules = (uiGlobalFlags & 0x100) ? 1 : 0;
        static const char PROGMEM szFmt[] = {"%i %i %i %i %i"};
        snprintf_P(szMsgBuff, sizeof(szMsgBuff), szFmt, nConWifi, nReadPlan, nMonitorPlan, nVocalise, nReadRules);
        myOutput.outputData(szMsgBuff);            
      }
      break;
case 26: // SHOREPORT - show report flags - Serial, Wifi, sensor value reporting, head matrix reporting
      {
        int nSerial = (uiGlobalFlags & 0x01) ? 1 : 0;
        int nWifi = (uiGlobalFlags & 0x02) ? 1 : 0;
        int nSensors = (uiGlobalFlags & 0x04) ? 1 : 0;
        int nHeadMatrix = (uiGlobalFlags & 0x08) ? 1 : 0;
        int nReportPlan = (uiGlobalFlags & 0x0200) ? 1 : 0;
        int nReportVocalise = (uiGlobalFlags & 0x0400) ? 1 : 0;
        static const char PROGMEM szFmt[] = {"%i %i %i %i %i %i"};
        snprintf_P(szMsgBuff, sizeof(szMsgBuff), szFmt, nSerial, nWifi, nSensors, nHeadMatrix, nReportPlan, nReportVocalise);
        myOutput.outputData(szMsgBuff);            
      }
      break;
  case 27: // SHORATE - show how many times per second do we process the plan? 0 means no plan processing
      {
        static const char PROGMEM szFmt[] = {"%u"};
        snprintf_P(szMsgBuff, sizeof(szMsgBuff), szFmt, uiPlanRate);
        myOutput.outputData(szMsgBuff);      
      }
      break;
  case 28: // SHONAMES - show the stored plan element names
      {
        Instinct::instinctID bMaxID = myNames.maxElementNameID();
        for (Instinct::instinctID i = 1; i <= bMaxID; i++)
        {
          char *pName;
          if ( pName = myNames.getElementName(i) )
          {
            static const char PROGMEM szFmt[] = {"%s=%u"};
            snprintf_P(szMsgBuff, sizeof(szMsgBuff), szFmt, pName, i);
            myOutput.outputData(szMsgBuff);
          }
        }
      }
      break;
  case 29: // SPEAKRULE N N N N N N - set rule - NodeType Status Timeout RepeatMyself RptTimeout AlwaysSpeak
      bSayOK = true;
      if (strlen(pCmd) > (strlen(szCmd)+1))
      {
        unsigned int uiNodeType, uiStatus, uiTimeout, uiRepeatMyself, uiRptTimeout, uiAlwaysSpeak;
        uiNodeType = uiStatus = uiTimeout = uiRepeatMyself = uiRptTimeout = uiAlwaysSpeak = 0;
        static const char PROGMEM szFmt[] = {"%u %u %u %u %u %u"};
        if ( sscanf_P(pCmd + strlen(szCmd), szFmt, &uiNodeType, &uiStatus, &uiTimeout, &uiRepeatMyself, &uiRptTimeout, &uiAlwaysSpeak) == 6)
        {
          bRtn = myMonitor.setSpeakRule((unsigned char)uiNodeType, (unsigned char)uiStatus,
                         uiTimeout, (unsigned char)uiRepeatMyself, uiRptTimeout, (unsigned char)uiAlwaysSpeak);
        }      
      }
      break;
  case 30: // SHORULES - show speak rules
      for (unsigned char i = 0; i < INSTINCT_NODE_TYPES; i++)
      {
        char szNodeTypeName[5];
        getNodeTypeName(szNodeTypeName, sizeof(szNodeTypeName), i);
        
        for (unsigned char j = 0; j < INSTINCT_RUNTIME_NOT_RELEASED; j++)
        {
          R5SpeakRulesType *pRule = myMonitor.getSpeakRule(i, j); 
          static const char PROGMEM szFmt[] = {"%s %u %u %u %u %u"};
          snprintf_P(szMsgBuff, sizeof(szMsgBuff), szFmt, szNodeTypeName, j, pRule->uiTimeout, pRule->bRepeatMyself, pRule->uiRptTimeout, pRule->bAlwaysSpeak);
          myOutput.outputData(szMsgBuff);
        }
      }
      break;
  case 31: // SRULES - save speak rules in EEPROM
      bSayOK = true;
      bRtn = myMemory.setSpeakRules(myMonitor.getSpeakRule(0,0));
      break;
  case 32: // RRULES - read speak rules from EEPROM
      bSayOK = true;
      bRtn = myMemory.getSpeakRules(myMonitor.getSpeakRule(0,0));
      break;
  case 33: // CNAMES - clear plan element names
      bSayOK = true;
      bRtn = myNames.clearElementNames();
      break;
  default:
    getProgmemStr(szMsgBuff, sizeof(szMsgBuff), 1, szRobotMessages);
    strncat(szMsgBuff, szCmd, sizeof(szMsgBuff));
    myOutput.outputData(szMsgBuff);
    myOutput.outputData(getProgmemStr(szMsgBuff, sizeof(szMsgBuff), 0, szRobotMessages));
    bRtn = false;
    break;
  }

  if (bSayOK)
  {
    myOutput.outputData(bRtn ? "OK" : "Fail");
  }  
  return bRtn;  
}

// return help details for a specific command line option
// use pMsgBuff as a char buffer 
unsigned char helpCommand(char *pMsgBuff, unsigned char nOption)
{
  unsigned char bRtn = false;
  unsigned char bHelpLine = 0;

  static const char PROGMEM szCommandHelp[] = {"PLAN!"
"STOP - Disable the robot motors!"
"START - Enable the robot motors!"
"RESET - does not work - needs a bootloader fix!"
"DUMP - Dump a complete listing of the Instinct Plan!"
"TIME - Report the time!"
"SETTIME YYYY MM DD HH MM SS - Set the time!"
"REPORT N N N N N N - Enable/disable reporting - Serial Wifi Sensors HeadMatrix Plan Vocalise!"
"RATE N - Set plan rate - cycles per second - 0 to stop plan execution!"
"CAL - recalibrate sensors!"
"CON - connect to wifi - useful if server started after robot is booted!"
"PELEM [name]=[ID] - associate a name with a plan element ID!"
"RSENSE [name]=[ID] - associate a name with a robot sense ID!"
"RACTION [name]=[ID] - associate a name with a robot action ID!"
"HSTOP - stop robot head from scanning!"
"HSTART - allow robot head to scan!"
"SPLAN - save robot plan in EEPROM!"
"RPLAN - read robot plan from EEPROM!"
"SCONF - save robot config in EEPROM!"
"RCONF - read robot config from EEPROM!"
"SWIFI SSID PW WifiRetry IP Port ServerRetry - set wifi params!"
"CONF N N N N N - set config flags - ConnectWifi ReadPlan MonitorPlan Vocalise ReadSpeakRules!"
"HELP - return command list to the user - HELP [CMD] - command help!"
"VER - return date and time of last compilation!"
"SHOWIFI - show wifi params - SSID PW WifiRetry IP Port ServerRetry!"
"SHOCONF - show startup flags - ConnectWifi ReadPlan MonitorPlan Vocalise ReadSpeakRules!"
"SHOREPORT - show report flags - Serial, Wifi, sensor values, head matrix values!"
"SHORATE - show plan cycle rate. 0 means no plan processing!"
"SHONAMES - show plan element names stored in the robot!"
"SPEAKRULE N N N N N N - set rule - NodeType Status Timeout RepeatMyself RptTimeout AlwaysSpeak!"
"SHORULES - show speak rules!"
"SRULES - save speak rules in EEPROM!"
"RRULES - read speak rules from EEPROM!"
"CNAMES - clear plan element names!"
};
  

  if (nOption == 0) // this is the PLAN command, so help comes from the CmdPlanner class
  {
    while(strlen(getProgmemStr(pMsgBuff, R5_MSG_BUFF_SIZE, bHelpLine, myPlan.help())) > 0)
    {
      bHelpLine++;
      bRtn = true;
      myOutput.outputData(pMsgBuff);
    }
  }
  else if (strlen(getProgmemStr(pMsgBuff, R5_MSG_BUFF_SIZE, nOption, szCommandHelp)) > 0)
  {
    bRtn = true;
    myOutput.outputData(pMsgBuff);      
  }

  return bRtn;
}

// log execution type and node data
void writeOutput(const Instinct::PlanNode * pPlanNode, const char *pType, const Instinct::ReleaserType *pReleaser, const int nSenseValue)
{
  char szDisplayBuff[80];
  int nLen;
  const static char PROGMEM szFmt[] = {" %i"};

  // if we are not writing to console or wifi, then save time
  if (!(uiGlobalFlags & 0x03) || !(uiGlobalFlags & 0x0200))
    return;
  
  strcpy(szDisplayBuff, pType);
  strcat(szDisplayBuff, " ");
  
  nLen = strlen(szDisplayBuff);
  if (pPlanNode)
  {
    getNodeTypeName(szDisplayBuff + nLen, sizeof(szDisplayBuff) - nLen, pPlanNode->bNodeType);
    strcat(szDisplayBuff, " ");
    nLen = strlen(szDisplayBuff);
    myPlan.displayNodeCounters(szDisplayBuff + nLen, sizeof(szDisplayBuff)- nLen, pPlanNode);
  }
  if (pReleaser)
  {
    myPlan.displayReleaser(szDisplayBuff + nLen, sizeof(szDisplayBuff)- nLen, pReleaser);
    nLen = strlen(szDisplayBuff);
    snprintf_P(szDisplayBuff + nLen, sizeof(szDisplayBuff)- nLen, szFmt, nSenseValue);
  }
  
  myOutput.outputData(szDisplayBuff);
}

// report the values of the 4 corner sensors, plus the rudder in degrees and distance travelled in mm
// 0 = sensor 1 FR, 1 = sensor 2 FL, 2 = sensor 3 RL, 4 = sensor 4 RR  -- distances are in mm
// edges: 0 = front, 1 = left, 2 = rear, 3 = right
void reportSensorValues(void)
{
  char szDisplayBuff[100];
  int nCornerDist[4];
  int nEdgeDist[4];
  int nEdgeAngle[4];
  
  for (int i = 0; i < 4; i++)
  {
      nCornerDist[i] = sensors.getCornerDistance(i);
      nEdgeDist[i] = sensors.getEdgeDistance(i);
      nEdgeAngle[i] = sensors.getEdgeAngle(i);
  }
  static const char PROGMEM szFmt[] = {"X %i %i %i %i %i %i %i %i %i %i %i %i %i %li %i %i %i %i"};
  snprintf_P(szDisplayBuff, sizeof(szDisplayBuff), szFmt, nCornerDist[0], nCornerDist[1], nCornerDist[2], nCornerDist[3],
                                                                  nEdgeDist[0], nEdgeDist[1], nEdgeDist[2], nEdgeDist[3],
                                                                  nEdgeAngle[0], nEdgeAngle[1], nEdgeAngle[2], nEdgeAngle[3],
                                                                  motors.getRudder(), motors.getDistanceTravelled(),
                                                                  motors.getMotorCurrent(0), myPIR.activated(),
                                                                  myRanger.range(), myHead.getHMinRange(0) );
  myOutput.outputData(szDisplayBuff);
}

// report all the cells in the head sensor matrix as a Y record
void reportHeadMatrix(void)
{
  char szDisplayBuff[100];
  char szElemBuff[12];
  static const char PROGMEM szFmt1[] = {"Y"};
  static const char PROGMEM szFmt2[] = {" %i"};
  
  strcpy_P(szDisplayBuff, szFmt1);

  for (unsigned char v = myHead.getVCells(); v > 0; v--)
  {
    for (unsigned char h = myHead.getHCells(); h > 0; h--)
    {
      unsigned int uiRange = myHead.getRangeAtCell(h-1, v-1);
      snprintf_P(szElemBuff, sizeof(szElemBuff), szFmt2, uiRange);
      if (sizeof(szDisplayBuff) - strlen(szDisplayBuff) > strlen(szElemBuff) )
        strcat(szDisplayBuff, szElemBuff);
    }
    
  }
  myOutput.outputData(szDisplayBuff);
}

// send output data to both the Serial monitor and the Wifi, depending on the bit settings from the REPORT command
// prefix a millisecond timestamp
void MyOutput::outputData(const char *pszData)
{
  unsigned long uiMillis = millis();
  char szMillisBuff[15];

  static const char PROGMEM szFmt[] = {"%010lu "};
  snprintf_P(szMillisBuff, sizeof(szMillisBuff), szFmt, uiMillis);
  
  if (uiGlobalFlags & 0x01)
  {
    if ( Serial.print(szMillisBuff) != strlen(szMillisBuff) )
    {
        Serial.flush();
        Serial.println(F("szMillisBuff lost data"));
    }
    if ( Serial.println(pszData) != (strlen(pszData)+2) )
    {
        Serial.flush();
        Serial.println(F("pszData lost data"));
    }
 }
  
  if (uiGlobalFlags & 0x02)
  {
    wifly.send(szMillisBuff);
    wifly.send(pszData);
    wifly.send("\n");
  }
}

// log textual data from the vocaliser
void MyOutput::outputVocaliseData(const char *pszData)
{
  if (uiGlobalFlags & 0x0400)
    outputData(pszData);
}


// 	Library for Rover 5 Platform EEPROM
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
// The EEPROM is used to store robot configuration across power cycles
// It stores the following in the EEPROMStorage struct within the EEPROM.
// - The WiFi SSID and password
// - Instinct-Server IP address and port number
// - A byte containing global flags that control robot operation
// - The Instinct Plan (stored in binary form)
//
#ifndef _R5EEPROM_H_
#define _R5EEPROM_H_

#define R5_WIFI_SSID	20
#define R5_WIFI_PW		20
#define R5_SERVER_IP	16

typedef struct {
    char szWifiSSID[R5_WIFI_SSID];
    char szWifiPassword[R5_WIFI_PW];
    char szServerIP[R5_SERVER_IP]; // "255.255.255.255"
    unsigned int uiServerPort;
	unsigned char bWifiRetry = 2;
	unsigned char bServerRetry = 3;
} R5ServerParamsType;

typedef struct {
	R5ServerParamsType sServerParams;
    unsigned int uiGlobalFlags;
    R5SpeakRulesType speakRules[INSTINCT_NODE_TYPES][INSTINCT_RUNTIME_NOT_RELEASED];
    unsigned int uiPlanRate;
    unsigned int uiDataLen; // the length of binary data stored in EEPROM
    int nPlanID;
    Instinct::instinctID bPlanElements[INSTINCT_NODE_TYPES]; // number of plan elements
    char bPlan; // the first byte of the byte stream that is the plan
    char bData; // after all the plan bytes, we can store binary data in this structure
} EEPROMStorageType;

class R5EEPROM {
public:

	unsigned int  getGlobalFlags(void);
	unsigned int  getPlanRate(void);
	unsigned char getServerParams(R5ServerParamsType *pServerParams);
	unsigned char getSpeakRules(R5SpeakRulesType *pSpeakRules);
	unsigned int  setGlobalFlags(const unsigned int uiFlags);
	unsigned char setPlanRate(const unsigned int uiPlanRate);
	unsigned char setServerParams(R5ServerParamsType *pServerParams);
	unsigned char setSpeakRules(R5SpeakRulesType *pSpeakRules);
	unsigned char readData(Instinct::CmdPlanner *pPlan, const unsigned int uiBuffLen, unsigned char *pData);
	unsigned char writeData(Instinct::CmdPlanner *pPlan, const unsigned int uiDataLen, unsigned char *pData);

private:
	unsigned int getStringAtOffset(char *pBuff, const unsigned int nBuffLen, const unsigned int nOffset);
	unsigned int setStringAtOffset(char *pBuff, const unsigned int nOffset);
	unsigned char setIntAtOffset(const int nValue, const unsigned int nOffset);
	int getIntAtOffset(const unsigned int nOffset);
	unsigned int setBytesAtOffset(unsigned char *pBuff, const unsigned int nLength, const unsigned int nOffset);
	unsigned int getBytesAtOffset(unsigned char *pBuff, const unsigned int nLength, const unsigned int nOffset);
};

#endif // _R5EEPROM_H_
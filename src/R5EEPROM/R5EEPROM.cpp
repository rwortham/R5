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
// It stores the following in the EEPROMStorageType struct within the EEPROM.
// - The WiFi SSID and password
// - Instinct-Server IP address and port number
// - A byte containing global flags that control robot operation
// - The Instinct Plan (stored in binary form)
//
#include "Arduino.h"
#include "EEPROM.h"
#include "Instinct.h"
#include "R5Output.h"
#include "R5Voice.h"
#include "R5Vocalise.h"
#include "R5EEPROM.h"

extern EEPROMClass EEPROM;

unsigned char R5EEPROM::getServerParams(R5ServerParamsType *pServerParams)
{
	EEPROMStorageType *pEEPROM = 0;
	int nAddr = (int)&pEEPROM->sServerParams;

	return (getBytesAtOffset((unsigned char *)pServerParams, sizeof(R5ServerParamsType), nAddr) ? 1 : 0);
}

unsigned char R5EEPROM::setServerParams(R5ServerParamsType *pServerParams)
{
	EEPROMStorageType *pEEPROM = 0;
	int nAddr = (int)&pEEPROM->sServerParams;

	return (setBytesAtOffset((unsigned char *)pServerParams, sizeof(R5ServerParamsType), nAddr) ? 1 : 0);
}

unsigned char R5EEPROM::getSpeakRules(R5SpeakRulesType *pSpeakRules)
{
	EEPROMStorageType *pEEPROM = 0;
	int nAddr = (int)&pEEPROM->speakRules;

	return (getBytesAtOffset((unsigned char *)pSpeakRules,
			sizeof(R5SpeakRulesType)*INSTINCT_NODE_TYPES*INSTINCT_RUNTIME_NOT_RELEASED, nAddr) ? 1 : 0);
}

unsigned char R5EEPROM::setSpeakRules(R5SpeakRulesType *pSpeakRules)
{
	EEPROMStorageType *pEEPROM = 0;
	int nAddr = (int)&pEEPROM->speakRules;

	return (setBytesAtOffset((unsigned char *)pSpeakRules,
			sizeof(R5SpeakRulesType)*INSTINCT_NODE_TYPES*INSTINCT_RUNTIME_NOT_RELEASED, nAddr) ? 1 : 0);
}

unsigned int R5EEPROM::getGlobalFlags(void)
{
	EEPROMStorageType *pEEPROM = 0;
	int nAddr = (int)&pEEPROM->uiGlobalFlags;

	return getIntAtOffset(nAddr);
}

unsigned int R5EEPROM::setGlobalFlags(const unsigned int uiFlags)
{
	EEPROMStorageType *pEEPROM = 0;
	int nAddr = (int)&pEEPROM->uiGlobalFlags;

	return setIntAtOffset(uiFlags, nAddr);
}

unsigned int R5EEPROM::getPlanRate(void)
{
	EEPROMStorageType *pEEPROM = 0;
	int nAddr = (int)&pEEPROM->uiPlanRate;

	return getIntAtOffset(nAddr);
}

unsigned char R5EEPROM::setPlanRate(const unsigned int uiPlanRate)
{
	EEPROMStorageType *pEEPROM = 0;
	int nAddr = (int)&pEEPROM->uiPlanRate;

	return setIntAtOffset(uiPlanRate, nAddr);
}

// retrieve a zero terminated string from the EEPROM at nOffset into pBuff.
// return length of string
unsigned int R5EEPROM::getStringAtOffset(char *pBuff, const unsigned int nBuffLen, const unsigned int nOffset)
{
	unsigned int nLen = 0;
	unsigned char byte;

	while(nLen < (nBuffLen-1))
	{
		byte = EEPROM.read(nOffset + nLen);
		*pBuff = byte;
		pBuff++;
		if (byte == 0)
			break;
		nLen++;
	}
	*pBuff = 0;
	return nLen;
}

// write a zero terminated string to the EEPROM at nOffset.
// return length of string
unsigned int R5EEPROM::setStringAtOffset(char *pBuff, const unsigned int nOffset)
{
	unsigned int nLen = 0;
	unsigned char byte;

	while (byte = *pBuff)
	{
		EEPROM.update(nOffset + nLen, byte);
		pBuff++;
		nLen++;
	}
	EEPROM.update(nOffset + nLen, 0);

	return nLen;
}

// retrieve a stream of bytes from the EEPROM at nOffset into pBuff. return bytes read
unsigned int R5EEPROM::getBytesAtOffset(unsigned char *pBuff, const unsigned int nLength, const unsigned int nOffset)
{
	unsigned int nLen = 0;

	while(nLen < nLength)
	{
		*pBuff = EEPROM.read(nOffset + nLen);
		pBuff++;
		nLen++;
	}
	return nLen;
}

// write bytes the EEPROM at nOffset. return bytes written
unsigned int R5EEPROM::setBytesAtOffset(unsigned char *pBuff, const unsigned int nLength, const unsigned int nOffset)
{
	unsigned int nLen = 0;
	unsigned char byte;

	while (nLen < nLength)
	{
		EEPROM.update(nOffset + nLen, *pBuff);
		pBuff++;
		nLen++;
	}

	return nLen;
}

// retrieve an integer from the EEPROM at nOffset.
// return the integer
int R5EEPROM::getIntAtOffset(const unsigned int nOffset)
{
	unsigned int byte1 = EEPROM.read(nOffset);
	unsigned int byte2 = EEPROM.read(nOffset+1);

	return int(byte1+(byte2 << 8));
}

// write an integer to the EEPROM at nOffset.
// return true/false
unsigned char R5EEPROM::setIntAtOffset(const int nValue, const unsigned int nOffset)
{
	unsigned char byte1 = nValue & 0xFF;
	unsigned char byte2 = (nValue & 0xFF00) >> 8;

	EEPROM.update(nOffset, byte1);
	EEPROM.update(nOffset+1, byte2);

	return true;
}

// read the plan back into RAM from EEPROM
unsigned char R5EEPROM::readData(Instinct::CmdPlanner *pPlan, const unsigned int uiBuffLen, unsigned char *pData)
{
	EEPROMStorageType *pEEPROM = 0;
	Instinct::PlanNode sPlanNode;
	Instinct::instinctID bPlanElements[INSTINCT_NODE_TYPES];
	Instinct::instinctID bElemCount = 0;

	// first we read the number of Node structures we are going to read
	getBytesAtOffset((unsigned char *)bPlanElements, sizeof(bPlanElements), (int)pEEPROM->bPlanElements);

	for (int i = 0; i < INSTINCT_NODE_TYPES; i++)
		bElemCount += bPlanElements[i];

	if ( !pPlan->initialisePlan(bPlanElements) )
		return false;

	pPlan->setPlanID(getIntAtOffset((int)&pEEPROM->nPlanID));
	int nAddr = (int)&pEEPROM->bPlan;

	// read each element from the EPROM into a buffer and then write it to the plan
	for ( Instinct::instinctID i = 0; i < bElemCount; i++)
	{
		getBytesAtOffset(&sPlanNode.bNodeType, sizeof(sPlanNode.bNodeType), nAddr); // read the node type
		int nSize = pPlan->sizeFromNodeType(sPlanNode.bNodeType);
		if (!nSize) // some bad thing has happened
			return false;
		nAddr += sizeof(sPlanNode.bNodeType);
		getBytesAtOffset((unsigned char *)&sPlanNode.sElement, nSize, nAddr); // complete the sPlanNode
		pPlan->addNode(&sPlanNode);
		nAddr += nSize;
	}

	// copy the binary data from EEPROM to the buffer
	unsigned int uiDataLen = getIntAtOffset((int)&pEEPROM->uiDataLen);
	unsigned int uiLen = min(uiDataLen, uiBuffLen);
	if (pData && uiLen)
	{
		getBytesAtOffset(pData, uiLen, nAddr);
	}
	return true;
}

// write the plan to the EEPROM
unsigned char R5EEPROM::writeData(Instinct::CmdPlanner *pPlan, const unsigned int uiDataLen, unsigned char *pData)
{
	Instinct::PlanNode sPlanNode;
	Instinct::instinctID bPlanElements[INSTINCT_NODE_TYPES];
	Instinct::instinctID bElemCount = pPlan->planSize(); // total number of elements;
	unsigned int nPlanMemoryUsage = pPlan->planUsage(NULL); // total space used in RAM

	// calculate how much EEPROM we will use and return false if not enough
	unsigned int nEEPROMUsage = nPlanMemoryUsage + (bElemCount * sizeof(sPlanNode.bNodeType)) + uiDataLen;
	if (EEPROM.length() < (nEEPROMUsage + sizeof(EEPROMStorageType)))
		return false;

	Instinct::instinctID bMaxElementID = pPlan->maxElementID();
	EEPROMStorageType *pEEPROM = 0;

	// first we store the number of Node structures we are going to write out
	pPlan->planSize(bPlanElements); // get the array of element counts from the plan
	setBytesAtOffset(bPlanElements, sizeof(bPlanElements), (int)&pEEPROM->bPlanElements);

	setIntAtOffset(pPlan->getPlanID(), (int)&pEEPROM->nPlanID);

	int nAddr = (int)&pEEPROM->bPlan;

	// read each element from the plan into a buffer and then write it to EEPROM
	for ( Instinct::instinctID i = 0; i < bMaxElementID; i++)
	{
		if (pPlan->getNode(&sPlanNode, i+1))
		{
			int nSize = pPlan->sizeFromNodeType(sPlanNode.bNodeType);
			if (!nSize) // some bad thing has happened
				return false;
			nSize += sizeof(sPlanNode.bNodeType);
			nAddr += setBytesAtOffset((unsigned char *)&sPlanNode, nSize, nAddr);
		}
	}

	if (pData && uiDataLen)
	{
		// store the binary data at the end
		setBytesAtOffset(pData, uiDataLen, nAddr);
		setIntAtOffset(uiDataLen, (int)&pEEPROM->uiDataLen);
	}

	return true;
}





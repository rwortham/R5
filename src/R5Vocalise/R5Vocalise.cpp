// 	Library for Rover 5 Platform Vocaliser for Instinct Monitor output
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
#include "Instinct.h"
#include "R5Output.h"
#include "R5Voice.h"
#include "R5Vocalise.h"

#define RPT_T 3333 // the default repeat timeout

// initialise variables
R5ExecStackMonitor::R5ExecStackMonitor(Instinct::Names *pNames, R5Voice *pVocaliser, R5Output *pOut)
{
  pPlanNames = pNames;
  pVoice = pVocaliser;
  pOutput = pOut;
  nExecStackPointer = -1;
  nExecStackDepth = 0;

  //#define INSTINCT_ACTIONPATTERN      0
  //#define INSTINCT_ACTIONPATTERNELEMENT 1
  //#define INSTINCT_COMPETENCE       2
  //#define INSTINCT_COMPETENCEELEMENT    3
  //#define INSTINCT_DRIVE          4
  //#define INSTINCT_ACTION         5
  //#define INSTINCT_RUNTIME_NOT_TESTED    0
  //#define INSTINCT_RUNTIME_SUCCESS    1
  //#define INSTINCT_RUNTIME_IN_PROGRESS  2
  //#define INSTINCT_RUNTIME_ERROR      3
  //#define INSTINCT_RUNTIME_FAILED     4
  //#define INSTINCT_RUNTIME_NOT_RELEASED 5 -- can't have a running element in this state

  // INSTINCT_RUNTIME_NOT_TESTED, INSTINCT_RUNTIME_SUCCESS, INSTINCT_RUNTIME_IN_PROGRESS, INSTINCT_RUNTIME_ERROR, INSTINCT_RUNTIME_FAILED
//  { {{500,0,RPT_T,0},  {2000,0,RPT_T,0}, {0,0,0,0}, {5000,0,RPT_T,1}, {0,0,0,0}}, // INSTINCT_ACTIONPATTERN - AP's are only important if they ERROR
//    {{0,0,0,0},        {0,0,0,0},        {0,0,0,0}, {5000,0,RPT_T,1}, {0,0,0,0}}, // INSTINCT_ACTIONPATTERNELEMENT - don't speak about this unless it errors
//    {{800,0,RPT_T,0},  {3000,0,RPT_T,0}, {0,0,0,0}, {5000,0,RPT_T,1}, {0,0,0,0}}, // INSTINCT_COMPETENCE
//    {{0,0,0,0},        {0,0,0,0},        {0,0,0,0}, {5000,0,RPT_T,1}, {0,0,0,0}}, // INSTINCT_COMPETENCEELEMENT - don't speak about this unless it errors
//    {{3000,0,RPT_T,1}, {3000,0,RPT_T,1}, {0,0,0,0}, {5000,0,RPT_T,1}, {0,0,0,0}}, // INSTINCT_DRIVE - starting something new and success are important
//    {{800,0,RPT_T,0},  {500,0,RPT_T,0},  {0,0,0,0}, {5000,0,RPT_T,1}, {5000,0,RPT_T,1}}  // INSTINCT_ACTION - actions are only important if they fail or error
//  };

  // initialise the rules
  memset(speakRules, 0, sizeof(speakRules));

  setSpeakRule(INSTINCT_ACTIONPATTERN,        INSTINCT_RUNTIME_NOT_TESTED, 500,  0, RPT_T, 0);
  setSpeakRule(INSTINCT_ACTIONPATTERN,        INSTINCT_RUNTIME_SUCCESS,    2000, 0, RPT_T, 0);
  setSpeakRule(INSTINCT_ACTIONPATTERN,        INSTINCT_RUNTIME_ERROR,      5000, 0, RPT_T, 1);

  setSpeakRule(INSTINCT_ACTIONPATTERNELEMENT, INSTINCT_RUNTIME_ERROR,      5000, 0, RPT_T, 1);

  setSpeakRule(INSTINCT_COMPETENCE,           INSTINCT_RUNTIME_NOT_TESTED, 800,  0, RPT_T, 0);
  setSpeakRule(INSTINCT_COMPETENCE,           INSTINCT_RUNTIME_SUCCESS,    3000, 0, RPT_T, 0);
  setSpeakRule(INSTINCT_COMPETENCE,           INSTINCT_RUNTIME_ERROR,      5000, 0, RPT_T, 1);

  setSpeakRule(INSTINCT_COMPETENCEELEMENT,    INSTINCT_RUNTIME_ERROR,      5000, 0, RPT_T, 1);

  setSpeakRule(INSTINCT_DRIVE,                INSTINCT_RUNTIME_NOT_TESTED, 3000, 0, RPT_T, 1);
  setSpeakRule(INSTINCT_DRIVE,                INSTINCT_RUNTIME_SUCCESS,    3000, 0, RPT_T, 1);
  setSpeakRule(INSTINCT_DRIVE,                INSTINCT_RUNTIME_ERROR,      5000, 0, RPT_T, 1);

  setSpeakRule(INSTINCT_ACTION,               INSTINCT_RUNTIME_NOT_TESTED, 800,  0, RPT_T, 0);
  setSpeakRule(INSTINCT_ACTION,               INSTINCT_RUNTIME_SUCCESS,    500,  0, RPT_T, 0);
  setSpeakRule(INSTINCT_ACTION,               INSTINCT_RUNTIME_ERROR,      5000, 0, RPT_T, 1);
  setSpeakRule(INSTINCT_ACTION,               INSTINCT_RUNTIME_FAILED,     5000, 0, RPT_T, 1);
}

// modify a speak rule
unsigned char R5ExecStackMonitor::setSpeakRule(const unsigned char bNodeType, const unsigned char bStatus,
                                            const unsigned int uiTimeout, const unsigned char bRepeatMyself,
                                            const unsigned int uiRptTimeout, const unsigned char bAlwaysSpeak)
{
  // check array limits are within bounds
  if ((bNodeType >= INSTINCT_NODE_TYPES) || (bStatus >= INSTINCT_RUNTIME_NOT_RELEASED))
    return false;

  R5SpeakRulesType *pRule = &speakRules[bNodeType][bStatus];

  pRule->uiTimeout = uiTimeout;
  pRule->bRepeatMyself = bRepeatMyself;
  pRule->uiRptTimeout = uiRptTimeout;
  pRule->bAlwaysSpeak = bAlwaysSpeak;

  return true;
}

// just return a pointer to the private data structure
R5SpeakRulesType * R5ExecStackMonitor::getSpeakRule(const unsigned char bNodeType, const unsigned char bStatus)
{
	R5SpeakRulesType *pRules = &speakRules[bNodeType][bStatus];

	return pRules;
}

unsigned char R5ExecStackMonitor::nodeExecuted(const Instinct::PlanNode * pPlanNode)
{
  nExecStackPointer++;
  if (nExecStackPointer >= R5_EXEC_STACK_DEPTH)
  {
        static const char PROGMEM szFmt[] = {"ERROR - nExecStackPointer overflow %i"};
        char szMsgBuff[50];
        snprintf_P(szMsgBuff, sizeof(szMsgBuff), szFmt, nExecStackPointer);
		Serial.println(szMsgBuff);
        nExecStackPointer--;
  }

  if (nExecStackPointer < 0)
  {
        static const char PROGMEM szFmt[] = {"ERROR - nExecStackPointer underflow %i"};
        char szMsgBuff[50];
        snprintf_P(szMsgBuff, sizeof(szMsgBuff), szFmt, nExecStackPointer);
	    Serial.println(szMsgBuff);
        nExecStackPointer++;
  }

  // check if the plan node is the same as last cycle, or if it has changed or restarted
  if ( (nExecStackDepth > nExecStackPointer) && (pPlanNode->bNodeType == sExecStack[nExecStackPointer].bNodeType) &&
        (pPlanNode->sElement.sReferences.bRuntime_ElementID == sExecStack[nExecStackPointer].bRuntime_ElementID) )
  {
    // same element as last cycle
    // update the stack depth
    nExecStackDepth = max(nExecStackDepth, nExecStackPointer+1);
  }
  else
  {
    // this is a different element than last cycle
    sExecStack[nExecStackPointer].bNodeType = pPlanNode->bNodeType;
    sExecStack[nExecStackPointer].bRuntime_ElementID = pPlanNode->sElement.sReferences.bRuntime_ElementID;
    sExecStack[nExecStackPointer].bStatus = INSTINCT_RUNTIME_NOT_TESTED;
    nExecStackDepth = nExecStackPointer+1;
    vocalise(&sExecStack[nExecStackPointer]);
  }

  return true;
}

unsigned char R5ExecStackMonitor::nodeSuccess(const Instinct::PlanNode * pPlanNode)
{
  sExecStack[nExecStackPointer].bStatus = INSTINCT_RUNTIME_SUCCESS;
  vocalise(&sExecStack[nExecStackPointer]);
  // the new stack depth is the current pointer offset
  nExecStackDepth = nExecStackPointer;
  nExecStackPointer--;
  return true;
}

unsigned char R5ExecStackMonitor::nodeInProgress(const Instinct::PlanNode * pPlanNode)
{
  sExecStack[nExecStackPointer].bStatus = INSTINCT_RUNTIME_IN_PROGRESS;
  vocalise(&sExecStack[nExecStackPointer]);
  // stack depth remains unchanged
  nExecStackPointer--;
  return true;
}

unsigned char R5ExecStackMonitor::nodeFail(const Instinct::PlanNode * pPlanNode)
{
  sExecStack[nExecStackPointer].bStatus = INSTINCT_RUNTIME_FAILED;
  vocalise(&sExecStack[nExecStackPointer]);
  // the new stack depth is the current pointer offset
  nExecStackDepth = nExecStackPointer;
  nExecStackPointer--;
  return true;
}

unsigned char R5ExecStackMonitor::nodeError(const Instinct::PlanNode * pPlanNode)
{
  sExecStack[nExecStackPointer].bStatus = INSTINCT_RUNTIME_ERROR;
  vocalise(&sExecStack[nExecStackPointer]);
  // the new stack depth is the current pointer offset
  nExecStackDepth = nExecStackPointer;
  nExecStackPointer--;
  return true;
}

// don't do anything with sense notifications
unsigned char R5ExecStackMonitor::nodeSense(const Instinct::ReleaserType *pReleaser, const int nSenseValue)
{
  return true;
}

void R5ExecStackMonitor::vocalise(const R5ExecStackElementType *pExecStackElement)
{
	char szMsgBuff[90];
	char szEndBuff[20];
	char *pElementName;

//static const char PROGMEM szFmt[] = {"nExecStackDepth=%i nExecStackPointer=%i pExecStackElement->bStatus=%i"};
//snprintf_P(szMsgBuff, sizeof(szMsgBuff), szFmt, nExecStackDepth, nExecStackPointer, (int)pExecStackElement->bStatus);
//Serial.println(szMsgBuff);

	szMsgBuff[0] = 0; // zero terminate the string buffers
	szEndBuff[0] = 0;

	// the speak rule determine the parameters for requesting vocalisation
	R5SpeakRulesType *pSpeakRule = &speakRules[pExecStackElement->bNodeType][pExecStackElement->bStatus];

	// check uiTimeout early on to optimise processing
	if (!pSpeakRule || !pSpeakRule->uiTimeout)
		return; // don't vocalise anything

	// add any preamble here, such as I've
	// then these are the words before and after the element name
	char *pBuff = szMsgBuff + strlen(szMsgBuff);
	int nBuffLen = sizeof(szMsgBuff) - strlen(szMsgBuff);

	switch (pExecStackElement->bStatus)
	{
		case INSTINCT_RUNTIME_NOT_TESTED:
		{
			// switch wording based on node type [for the first time]
			switch (pExecStackElement->bNodeType)
			{
				case INSTINCT_DRIVE:
				{
#ifdef R5_BUDDY
					static const char PROGMEM szFmt[] = {"Buddy is trying to "};
#else
					static const char PROGMEM szFmt[] = {"R5 is trying to "};
#endif
					snprintf_P(pBuff, nBuffLen, szFmt);
					// static const char PROGMEM szFmt2[] = {""};
					// snprintf_P(szEndBuff, sizeof(szEndBuff), szFmt2);
				}
				break;
				case INSTINCT_ACTION:
				case INSTINCT_ACTIONPATTERN:
				{
					static const char PROGMEM szFmt[] = {"Doing "};
					snprintf_P(pBuff, nBuffLen, szFmt);
					// static const char PROGMEM szFmt2[] = {""};
					// snprintf_P(szEndBuff, sizeof(szEndBuff), szFmt2);
				}
				break;
				default:
				{
					static const char PROGMEM szFmt[] = {"Attempting "};
					snprintf_P(pBuff, nBuffLen, szFmt);
					// static const char PROGMEM szFmt2[] = {""};
					// snprintf_P(szEndBuff, sizeof(szEndBuff), szFmt2);
				}
				break;
			}
		}
		break;
		case INSTINCT_RUNTIME_IN_PROGRESS:
		{
			static const char PROGMEM szFmt[] = {""};
			snprintf_P(pBuff, nBuffLen, szFmt);
			static const char PROGMEM szFmt2[] = {" in progress."};
			snprintf_P(szEndBuff, sizeof(szEndBuff), szFmt2);
		}
		break;
		case INSTINCT_RUNTIME_FAILED:
		{
			// say failed
#ifdef R5_BUDDY
			static const char PROGMEM szFmt[] = {"Buddy tried "};
#else
			static const char PROGMEM szFmt[] = {"R5 tried "};
#endif
			snprintf_P(pBuff, nBuffLen, szFmt);
			static const char PROGMEM szFmt2[] = {" and failed."};
			snprintf_P(szEndBuff, sizeof(szEndBuff), szFmt2);
		}
		break;
		case INSTINCT_RUNTIME_ERROR:
		{
			// say plan error
			static const char PROGMEM szFmt[] = {"Problem with "};
			snprintf_P(pBuff, nBuffLen, szFmt);
			static const char PROGMEM szFmt2[] = {". Check the plan."};
			snprintf_P(szEndBuff, sizeof(szEndBuff), szFmt2);
		}
		break;
		case INSTINCT_RUNTIME_SUCCESS:
		{
			// say successful, or done or completed or something
			// switch wording based on node type [for the first time]
			switch (pExecStackElement->bNodeType)
			{
				case INSTINCT_DRIVE:
				{
#ifdef R5_BUDDY
					static const char PROGMEM szFmt[] = {"Buddy has finished "};
#else
					static const char PROGMEM szFmt[] = {"R5 has finished "};
#endif
					snprintf_P(pBuff, nBuffLen, szFmt);
					// static const char PROGMEM szFmt2[] = {" successfully."};
					// snprintf_P(szEndBuff, sizeof(szEndBuff), szFmt2);
				}
				break;
				default:
				{
					static const char PROGMEM szFmt[] = {"Finished "};
					snprintf_P(pBuff, nBuffLen, szFmt);
				}
				break;
			}
		}
		break;
	}

	// now convert the pExecStackElement->bRuntime_ElementID into the node name
	pElementName = pPlanNames->getElementName(pExecStackElement->bRuntime_ElementID);

	if (pElementName)
	{
		char szWords[60]; // a buffer to receive the element name as words
		nameToWords(szWords, sizeof(szWords), pElementName);
		strncat(szMsgBuff, szWords, sizeof(szMsgBuff) - strlen(szMsgBuff));
	}
	else
	{
		static const char PROGMEM szFmt[] = {"Step %i"};
		pBuff = szMsgBuff + strlen(szMsgBuff);
		nBuffLen = sizeof(szMsgBuff) - strlen(szMsgBuff);
		snprintf_P(pBuff, nBuffLen, szFmt, pExecStackElement->bRuntime_ElementID);
	}

	// add the end words
	if (szEndBuff[0])
		strncat(szMsgBuff, szEndBuff, sizeof(szMsgBuff) - strlen(szMsgBuff));

	if ( pVoice->speak(szMsgBuff, pSpeakRule->uiTimeout, pSpeakRule->bRepeatMyself,
					pSpeakRule->uiRptTimeout, pSpeakRule->bAlwaysSpeak) )
	{
		static const char PROGMEM szFmt[] = {" ** %u %i %u %i"};
		pBuff = szMsgBuff + strlen(szMsgBuff);
		nBuffLen = sizeof(szMsgBuff) - strlen(szMsgBuff);
		snprintf_P(pBuff, nBuffLen, szFmt, pSpeakRule->uiTimeout, (int)pSpeakRule->bRepeatMyself,
						pSpeakRule->uiRptTimeout, (int)pSpeakRule->bAlwaysSpeak);
		pOutput->outputVocaliseData(szMsgBuff);
	}
	else
	{
		static const char PROGMEM szFmt[] = {" !! %u %i %u %i"};
		pBuff = szMsgBuff + strlen(szMsgBuff);
		nBuffLen = sizeof(szMsgBuff) - strlen(szMsgBuff);
		snprintf_P(pBuff, nBuffLen, szFmt, pSpeakRule->uiTimeout, (int)pSpeakRule->bRepeatMyself,
						pSpeakRule->uiRptTimeout, (int)pSpeakRule->bAlwaysSpeak);
		pOutput->outputVocaliseData(szMsgBuff);
	}
}

// convert something like ReverTurnAvoid to Reverse Turn Avoid, and Turn45LOrR to Turn 45 L or R
void R5ExecStackMonitor::nameToWords(char * pBuff, const unsigned int nBuffLen, const char * pElementName)
{
	int nLen = strlen(pElementName);
	unsigned char bPrevNumber = false;
	unsigned char bNewWord = false;
	int nCharsOut = 1; // initialise to 1 to allow for writing space + char

	*pBuff = 0;

	for (int i = 0; i < nLen; i++)
	{
		// ASCII Uppercase 65-90, Lowercase 97-122, Numbers 48-57
		char ch = *(pElementName+i);
		if ((ch >= 'A') && (ch <= 'Z'))
		{
			bPrevNumber = false;
			// new word, write out character
			bNewWord = true;
		}
		else if ((ch >= '0') && (ch <= '9'))
		{
			// new word if last char was not a number, write out number
			bNewWord = !bPrevNumber;
			bPrevNumber = true;
		}
		else
		{
			// continue same word unless preceeded by a number, write out character
			bNewWord = bPrevNumber;
			bPrevNumber = false;
		}

		if (nCharsOut >= nBuffLen)
			break;

		if ((i > 0) && bNewWord)
		{
			*pBuff = ' ';
			pBuff++;
			nCharsOut++;
		}

		*pBuff = ch;
		pBuff++;
		nCharsOut++;
	}

	// ensure string is zero terminated
	*pBuff = 0;
}


// 	Library for Rover 5 Platform - R5Vocaliser processes Instinct monitor output
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
#ifndef _R5VOCALISE_H_
#define __R5VOCALISE_H_

#define R5_EXEC_STACK_DEPTH 20

// structure to hold parameters for how to speak
typedef struct {
	unsigned int uiTimeout;
	unsigned char bRepeatMyself;
	unsigned int uiRptTimeout;
	unsigned char bAlwaysSpeak;
} R5SpeakRulesType;

typedef struct {
	unsigned char bNodeType;
	Instinct::instinctID bRuntime_ElementID;
	unsigned char bStatus; // store the last return status
} R5ExecStackElementType;

class R5ExecStackMonitor : public Instinct::Monitor {
public:
	R5ExecStackMonitor(Instinct::Names *pNames, R5Voice *pVocaliser, R5Output *pOut);
	unsigned char nodeExecuted(const Instinct::PlanNode * pPlanNode);
	unsigned char nodeSuccess(const Instinct::PlanNode * pPlanNode);
	unsigned char nodeInProgress(const Instinct::PlanNode * pPlanNode);
	unsigned char nodeFail(const Instinct::PlanNode * pPlanNode);
	unsigned char nodeError(const Instinct::PlanNode * pPlanNode);
	unsigned char nodeSense(const Instinct::ReleaserType *pReleaser, const int nSenseValue);
	unsigned char setSpeakRule(const unsigned char bNodeType, const unsigned char bStatus,
						const unsigned int uiTimeout, const unsigned char bRepeatMyself,
						const unsigned int uiRptTimeout, const unsigned char bAlwaysSpeak);
	R5SpeakRulesType * getSpeakRule(const unsigned char bNodeType, const unsigned char bStatus);

private:
	Instinct::Names *pPlanNames;
	R5Voice *pVoice;
	R5Output *pOutput;
  void vocalise(const R5ExecStackElementType *pExecStackElement);
  void nameToWords(char * pBuff, const unsigned int nBuffLen, const char * pElementName);
  // this is the storage stack
  int nExecStackPointer = -1; // points to the current element being addressed in the stack
  int nExecStackDepth = 0; // the current stack depth
  // the execution stack
  R5ExecStackElementType sExecStack[R5_EXEC_STACK_DEPTH];
  // store rules for all node types and event types
  R5SpeakRulesType speakRules[INSTINCT_NODE_TYPES][INSTINCT_RUNTIME_NOT_RELEASED];
};

#endif // __R5VOCALISE_H_

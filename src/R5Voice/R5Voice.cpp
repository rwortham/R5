// 	Library for Rover 5 Platform Emic2 Voice Module
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
#include "R5Voice.h"

// constructor to initialise variables
R5Voice::R5Voice(Stream *pStream)
{
	pSerial = pStream;
	szVoiceBuffer[0] = 0;
	szLastVoiceBuffer[0] = 0;
	uiVoiceTimeout = 0;
	uiRepeatTimeout = 0;
	ulStartMilliSecs = 0L;
	ulRepeatMilliSecs = 0L;
}

// handles the initial config of the Emic2 module
unsigned char R5Voice::initialiseVoice(const int nVol, const unsigned int uiSpeed, const unsigned int uiVoice)
{
	int nTimeout = 500; // 5 seconds

	pSerial->print('\n'); // send a return and wait for the prompt
	// When the Emic 2 has initialized and is ready, it will send a single ':' character, so wait here until we receive it
	while (nTimeout && (pSerial->read() != ':'))
	{
		delay(10);  // 10mS Short delay
		nTimeout--;
	}

	if (!nTimeout)
		return false;

	// set the volume, words per minute and voice
	pSerial->print("V");
	pSerial->println(nVol);
	while (pSerial->read() != -1); // waits till its done that
	pSerial->print("W");
	pSerial->println(uiSpeed);
	while (pSerial->read() != -1); // waits till its done that
	pSerial->print("N");
	pSerial->println(uiVoice);

	return true;
}

// called frequently by main loop. Checks if eMic2 ready to receive next string
// if timeout occurs then clear the buffer ready for new string
void R5Voice::processVoice(void)
{
	unsigned long ulMilliSecs = millis();

	if (szLastVoiceBuffer[0])
	{
		// should we forget we said something in the past?
		if ((ulRepeatMilliSecs + (unsigned long)uiRepeatTimeout) < ulMilliSecs)
		{
			szLastVoiceBuffer[0] = 0;
		}
	}

	if (!szVoiceBuffer[0]) // return if nothing to say
		return;


	if ((ulStartMilliSecs + (unsigned long)uiVoiceTimeout) > ulMilliSecs)
	{
		// not timed out yet
		if ( pSerial->read() != ':')   // When the Emic 2 has initialized and is ready, it will send a single ':' character
		{
			return; // not ready yet
		}
		while (pSerial->read() != -1); // clear any additional chars in receive buffer
		pSerial->print('S');
		pSerial->println(szVoiceBuffer);
		strncpy(szLastVoiceBuffer, szVoiceBuffer, sizeof(szLastVoiceBuffer));
		ulRepeatMilliSecs = ulMilliSecs;
	}
	// either we've timed out, or we've spoken
	szVoiceBuffer[0] = 0;
}

// request to speak something. If something else in buffer then fails, unless AlwaysSpeak in which case it overwrites buffer, unless we've just said it
unsigned char R5Voice::speak(const char *pWords, unsigned int uiTimeout, unsigned char bRepeatMyself, unsigned int uiRptTimeout, unsigned char bAlwaysSpeak)
{
 	if (!uiTimeout || !pWords || !(*pWords))
 		return false;

	if (szVoiceBuffer[0]) // we've already got something to say
	{
		if (!bAlwaysSpeak)
			return false;
		else
		{
			// if what we want to say now is the same as what we are waiting to say, we are done
			// this avoids updating the start time of the speech request
			if (!strcmp(pWords, szVoiceBuffer))
			{
//Serial.print("Already waiting to say ");
//Serial.println(pWords);
				return false;
			}
		}
	}

	// I just said this, so if I'm not repeating then return false
	if (!bRepeatMyself && !strcmp(pWords, szLastVoiceBuffer))
	{
//Serial.print("don't repeat >> ");
//Serial.println(pWords);
		return false;
	}

	strncpy(szVoiceBuffer, pWords, sizeof(szVoiceBuffer));
	szVoiceBuffer[sizeof(szVoiceBuffer)-1] = 0; // make sure its always zero terminated
	uiVoiceTimeout = uiTimeout;
	uiRepeatTimeout = uiRptTimeout;
	ulStartMilliSecs = millis();
	//	pSerial->print('\n'); // tell Emic2 we've got something to say - not needed

	return true;
}

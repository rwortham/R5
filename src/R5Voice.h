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
#ifndef _R5VOICE_H_
#define _R5VOICE_H_

class R5Voice {
public:
	R5Voice(Stream *pStream);
	unsigned char initialiseVoice(const int nVol, const unsigned int uiSpeed, const unsigned int uiVoice);
	void processVoice(void);
	unsigned char speak(const char *pWords, unsigned int uiTimeout, unsigned char bRepeatMyself, unsigned int uiRptTimeout, unsigned char bAlwaysSpeak);

private:
	char szVoiceBuffer[80]; // what I'm going to say next
	char szLastVoiceBuffer[80]; // what I last said
	unsigned int uiVoiceTimeout; // the max time to wait to say what's in the VoiceBuffer
	unsigned int uiRepeatTimeout; // the max time to remember what we last said
	unsigned long ulStartMilliSecs;
	unsigned long ulRepeatMilliSecs;
	Stream *pSerial;
};

#endif // _R5VOICE_H_

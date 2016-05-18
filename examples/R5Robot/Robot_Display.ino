//  R5 Robot Platform - Display
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

// Global functions defined in this file
void displayRainbow(void);
void displayClear(void);
void flashColour(unsigned char bRGB);


void displayClear(void)
{
  myPixelStrip.clear();
  myPixelStrip.show();
}


// show beautiful colour display to entice human to move
void displayRainbow(void)
{
  unsigned int uiCol = millis() / 16; // change up 64 times per second
  for(unsigned int i = 0; i < myPixelStrip.numPixels(); i++)
  {
    myPixelStrip.setPixelColor(i, colorWheel(((i * 256 / myPixelStrip.numPixels()) + (uiCol & 255)) & 255));
  }
  myPixelStrip.show();
}

// flash colour determined by bottom 3 pixels
void flashColour(unsigned char bRGB)
{
  if (myPixelStrip.getPixelColor(0))
    displayClear();
  else
  {
    unsigned long ulColour = myPixelStrip.Color(
      (bRGB & 0x04) ? 255 : 0,
      (bRGB & 0x02) ? 255 : 0,
      (bRGB & 0x01) ? 255 : 0);
    for(int i = 0; i<myPixelStrip.numPixels(); i++)
    {
      myPixelStrip.setPixelColor(i, ulColour);
    } 
    myPixelStrip.show();
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
unsigned long colorWheel(unsigned char bWheelPos) {
  bWheelPos = 255 - bWheelPos;
  if(bWheelPos < 85) {
    return myPixelStrip.Color(255 - bWheelPos * 3, 0, bWheelPos * 3);
  }
  if(bWheelPos < 170) {
    bWheelPos -= 85;
    return myPixelStrip.Color(0, bWheelPos * 3, 255 - bWheelPos * 3);
  }
  bWheelPos -= 170;
  return myPixelStrip.Color(bWheelPos * 3, 255 - bWheelPos * 3, 0);
}

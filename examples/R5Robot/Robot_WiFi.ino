//  R5 Robot Platform - Seeeduino Wifi Interface
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
boolean initialiseWifi(const char *pSSID, const char *pPassword, const unsigned char bRetry);
boolean tcpConnect(const char * pIPAddress, const int nPort, const unsigned char bRetry);


// handles the initial config of the wifi module and returns when it is connected
boolean initialiseWifi(const char *pSSID, const char *pPassword, const unsigned char bRetry)
{
    unsigned char bRetryCount = 0;
      
    Serial2.end();
    delay(100);
    Serial2.begin(230400);
    
    if ( !wifly.commandMode() ) // check if we can talk to wifi module
    {
       // this code should only execute the first time the wifi module is accessed
      Serial.println(F("Wifi commandMode failed. Probably wrong baud rate."));
      // need to change the baud rate to 230400
      Serial2.end();
      delay(100);
      Serial2.begin(9600);
      if ( !wifly.sendCommand("set u b 230400\r", "AOK"))
      {
        Serial.println(F("set u b 230400\r Failed."));
      }
      else
      {
        if ( wifly.save()); // save config with faster baud rate for next reboot
          Serial.println(F("Saved baud rate 230400"));
        wifly.sendCommand("set u i 230400\r", "AOK"); // set immediate baud rate
        Serial2.end();
        delay(100);
        Serial2.begin(230400);
      }
    }
   
    // check if WiFly is associated with AP(SSID)
    if (wifly.isAssociated(pSSID))
      return true;
      
    Serial.print(F("Wifi not associated with "));
    Serial.println(pSSID);

    wifly.sendCommand("set w j 1\r", "AOK"); //turn on auto join
    
    // WIFLY_AUTH_OPEN / WIFLY_AUTH_WPA1 / WIFLY_AUTH_WPA1_2 / WIFLY_AUTH_WPA2_PSK
    while (bRetryCount < bRetry)
    {
      if (wifly.join(pSSID, pPassword, WIFLY_AUTH_WPA1_2))
      {
        Serial.print(F("save wifi config "));
        if ( wifly.save())    // connection successful, so save configuration
           Serial.print(F("success."));
        else
           Serial.print(F("failed."));
        
        return true;
        
      } else
      {
         // Serial.print("Failed to join ");
         // Serial.println(szSSID);
         // Serial.println("Wait 0.1 second and try again...");
         delay(100);
         bRetryCount++;
      }
    }
    return false;
}

// establish the TCP connection to the remote InstinctServer
boolean tcpConnect(const char * pIPAddress, const int nPort, const unsigned char bRetry)
{
    unsigned char bRetryCount = 0;
    
    if (!wifly.isAssociated())
    {
      Serial.println(F("Wifi not associated"));
      return false; // cannot connect if no wifi
    }
    
    while(bRetryCount < bRetry)
    {
      if (wifly.connect(pIPAddress, nPort))
      {
        //connected to the remote service
        return true;
      }
      Serial.print(F("TCP connect failed to "));
      Serial.print(pIPAddress);
      Serial.print(F(" port "));
      Serial.println(nPort);
      bRetryCount++;
    }

    return false;    
}

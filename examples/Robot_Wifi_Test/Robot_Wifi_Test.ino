#include <Debug.h>

#include <Servo.h> // adding this include uses 139 bytes of RAM somehow
#include <SoftwareSerial.h> // adding this uses 68 bytes of RAM
#include <EEPROM.h>
#include <WiFly.h> // this uses no RAM
#include <Instinct.h>
#include <R5.h>

// Wifi module
// Arduino       WiFly
//  2    <---->    TX
//  3    <---->    RX
//SoftwareSerial WiFlyUART(12, 13);
WiFly wifly(&Serial2);

boolean tcpConnect(const int nRetry);
boolean initialiseWifi(const int nRetry);

void setup() {
    char szMsgBuff[80];
    
    Serial.begin(115200);
    Serial2.begin(115200);
    
    Serial.println("Initialised. Waiting 2 seconds");
    delay(2000);

    if (initialiseWifi(2)) // try this twice
      if(tcpConnect(3))
        Serial.println("Established InstinctServer Connection");// try this three times

/*    if ( !wifly.commandMode() )
    {
       // this code should only execute the first time the wifi module is accessed
      Serial.println("Wifi commandMode failed. Probably wrong baud rate.");
      // need to change the baud rate to 115200
      Serial2.end();
      delay(100);
      Serial2.begin(9600);
      if ( !wifly.sendCommand("set u b 115200\r", "AOK"))
      {
        Serial.println("set u b 115200\r Failed.");
      }
      else
      {
        if ( wifly.save()); // save config with faster baud rate for next reboot
          Serial.println("Saved config with baud rate at 115200");
        wifly.sendCommand("set u i 115200\r", "AOK"); // set immediate baud rate
        Serial2.end();
        delay(100);
        Serial2.begin(115200);
      }
    }
   
    char szSSID[] = {"InstinctWifi"};    

    //Initialize the wifi shield
    if (!wifly.init())
      Serial.println("Wifi init failed");
    else
    {
        // check if WiFly is associated with AP(SSID)
        if (!wifly.isAssociated(szSSID))
        {
            Serial.print("Wifi not associated with ");
            Serial.println(szSSID);

            wifly.sendCommand("set w j 1\r", "AOK"); //turn on auto join
            
            // WIFLY_AUTH_OPEN / WIFLY_AUTH_WPA1 / WIFLY_AUTH_WPA1_2 / WIFLY_AUTH_WPA2_PSK
           while (!wifly.join(szSSID, "64842431", WIFLY_AUTH_WPA1_2))
           {
              Serial.print("Failed to join ");
              Serial.println(szSSID);
              Serial.println("Wait 0.1 second and try again...");
              delay(100);
           }
        }
        if ( !wifly.save())    // save configuration
          Serial.println("save() failed.");
    }

    if (wifly.isAssociated(szSSID))
    {
      while(!wifly.connect("192.168.137.1",3000)) //connect the remote service
      {
        Serial.println("TCP connect failed");
      }
    }
    else
      Serial.println("Wifi no longer associated");

    strcpy(szMsgBuff, "TCP connected\n");
    Serial.print(szMsgBuff);
    
    wifly.write((uint8_t*)szMsgBuff, strlen(szMsgBuff));
    */
}

void loop() {
    static unsigned long ulOldMicroSecs = 0;
  
    unsigned long ulMicroSecs = micros();
    unsigned long ulDelayMicroSecs;
    static unsigned long ulSecCounter = 1;
    static unsigned long ulSecs = 0;

    char szBuffer[80];

    ulDelayMicroSecs = ulMicroSecs - ulOldMicroSecs;

    while (wifly.available()) {
      Serial.write(wifly.read());
    }
  
    while (Serial.available()) {
      wifly.write(Serial.read());
    }

  if (!ulSecCounter)
  {
    Serial.println("Tick");
    
    sprintf(szBuffer, "Tick %08li, Loop Delay %li \n", ulSecs, ulDelayMicroSecs); 
      wifly.write((uint8_t *)szBuffer, strlen(szBuffer));
  }

  if (!ulSecCounter)
  {
    ulSecCounter = 1000000 / ulDelayMicroSecs;
    ulSecs++;
  }
  ulOldMicroSecs = ulMicroSecs;
  ulSecCounter--;
}



// handles the initial config of the wifi module and returns when it is connected
boolean initialiseWifi(const int nRetry)
{
    const char *pSSID = {"InstinctWifi"};
    const char *pPassword = {"64842431"};
    int nRetryCount = 0;
      
    Serial2.end();
    delay(100);
    Serial2.begin(115200);
    
    if ( !wifly.commandMode() ) // check if we can talk to wifi module
    {
       // this code should only execute the first time the wifi module is accessed
      Serial.println("Wifi commandMode failed. Probably wrong baud rate.");
      // need to change the baud rate to 115200
      Serial2.end();
      delay(100);
      Serial2.begin(9600);
      if ( !wifly.sendCommand("set u b 115200\r", "AOK"))
      {
        Serial.println("set u b 115200\r Failed.");
      }
      else
      {
        if ( wifly.save()); // save config with faster baud rate for next reboot
          Serial.println("Saved baud rate 115200");
        wifly.sendCommand("set u i 115200\r", "AOK"); // set immediate baud rate
        Serial2.end();
        delay(100);
        Serial2.begin(115200);
      }
    }
   
    // check if WiFly is associated with AP(SSID)
    if (wifly.isAssociated(pSSID))
      return true;
      
    Serial.print("Wifi not associated with ");
    Serial.println(pSSID);

    wifly.sendCommand("set w j 1\r", "AOK"); //turn on auto join
    
    // WIFLY_AUTH_OPEN / WIFLY_AUTH_WPA1 / WIFLY_AUTH_WPA1_2 / WIFLY_AUTH_WPA2_PSK
    while (nRetryCount < nRetry)
    {
      if (wifly.join(pSSID, pPassword, WIFLY_AUTH_WPA1_2))
      {
        Serial.print("save wifi config ");
        if ( wifly.save())    // connection successful, so save configuration
           Serial.print("success.");
        else
           Serial.print("failed.");
        
        return true;
        
      } else
      {
         // Serial.print("Failed to join ");
         // Serial.println(szSSID);
         // Serial.println("Wait 0.1 second and try again...");
         delay(100);
         nRetryCount++;
      }
    }
    return false;
}

// establish the TCP connection to the remote InstinctServer
boolean tcpConnect(const int nRetry)
{
    const char * pIPAddress = {"192.168.137.1"};
    int nPort = 3000;
    int nRetryCount = 0;
    
    if (!wifly.isAssociated())
    {
      Serial.println("Wifi not associated");
      return false; // cannot connect if no wifi
    }
    
    while(nRetryCount < nRetry)
    {
      if (wifly.connect(pIPAddress, nPort))
      {
        //connected to the remote service
        return true;
      }
      Serial.print("TCP connect failed to ");
      Serial.print(pIPAddress);
      Serial.print(" port ");
      Serial.println(nPort);
      nRetryCount++;
    }
    
    // wifly.write((uint8_t*)szMsgBuff, strlen(szMsgBuff));
}


// Simple tester for Ultrasonic sensor
#include <Servo.h>
#include <EEPROM.h>
#include <Instinct.h>
#include <R5.h>

R5Ultrasonic myRanger(8, 10);

void setup()
{
    // Open up a serial connection to send readings back
    Serial.begin( 115200 );
}

void loop()
{
    unsigned long ulTime = micros();
    unsigned int nRangeMM = myRanger.measureRange();
    unsigned long ulDelay = micros() - ulTime;   
    Serial.print("Delay = ");
    Serial.print(ulDelay);
    Serial.print( " Distance = " );
    Serial.print(nRangeMM);
    Serial.println("mm");    
    delay( 400 );
}


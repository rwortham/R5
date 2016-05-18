#include <Servo.h>
#include <EEPROM.h>
#include <Instinct.h>
#include <R5.h>

Servo hServo;
Servo vServo;

// defines for the Ultrasonic rangefinder
#define ULTRASONIC_PIN 8
#define ULTRASONIC_MIN_INTERVAL 300
#define PIR_PIN 9

R5Ultrasonic myRanger(ULTRASONIC_PIN, ULTRASONIC_MIN_INTERVAL);
R5SensingHead myHead(&hServo, &vServo, 75, 180, &myRanger, 5, 1, 30);
R5PIR myPIR(PIR_PIN);

void setup()
{
  Serial.begin(115200);    

  hServo.attach(6);
  vServo.attach(7);
  vServo.write(180);
  
  myHead.setHScanParams(30, 15, 135);
  myHead.setHScanInterval(2500);  
//  myHead.setHScanInterval(0);  
  myHead.setVScanParams(45, 135, 180);
  myHead.setVScanInterval(0);
  myHead.lookAhead();  

  Serial.println("! to stop/start matrix reporting, S to stop/start head scanning");
}

void loop()
{
  static int nLoop = 100;
  static boolean bDisplay = true;
  static int nMinRange = R5_HEAD_MAXRANGE;
  static boolean bPIR = false;
  
  myHead.driveHead();

  nMinRange = min(nMinRange, myHead.getHMinRange(0));

  if (nLoop == 100)
  {
    if (bDisplay)
    {
      Serial.println("======================================================================================================");
      for ( int nVCoord = 0;  nVCoord < myHead.getVCells(); nVCoord++)
      {
        for ( int nHCoord = 0;  nHCoord < myHead.getHCells(); nHCoord++)
        {
          Serial.print(nHCoord);
          Serial.print("|");
          int nRange = myHead.getRangeAtCell(nHCoord, nVCoord);
          int nVal = min(2000, nRange);
          nVal = nVal/20;
          for ( int i = 0; i < nVal; i++)
            Serial.print("*");
          Serial.print("|");
          Serial.println(nRange);
        }
        Serial.println("");
       }
       Serial.print("senseHMatrixReady(0) = ");
       Serial.println((int)myHead.senseHMatrixReady(0));
  
       Serial.print("getHMostOpenAngle(0) = ");
       Serial.println((int)myHead.getHMostOpenAngle(0));
  
       Serial.print("getHMinRange(0) = ");
       Serial.println(myHead.getHMinRange(0));
       
       Serial.print("Interval min range  = "); // this the minimum sensed since the last printout
       Serial.println(nMinRange);
       
       Serial.print("myPIR.activated()  = ");
       Serial.println((int)myPIR.activated());

       nMinRange = R5_HEAD_MAXRANGE; // reset as we've printed the old value;
     }
     nLoop = 0;
  }
  nLoop ++;

  if (Serial.available() > 0)
  {
      char val = Serial.read();
      if (val == '!')
        bDisplay = !bDisplay; // just lets us stop reporting so we can read whats on the screen
      else if (val == 'S') // toggle the scanning
      {
        myHead.setHScanInterval(myHead.getHScanInterval() ? 0 : 2500);
      }
  }

  if ( bPIR != myPIR.activated() )
  {
       static unsigned long lastTime = 0;
       unsigned long lTime = millis();
       
       Serial.print(lTime -lastTime);
       Serial.print(" myPIR.activated()  = ");
       Serial.println((int)myPIR.activated());
       bPIR = myPIR.activated();
       lastTime = lTime;    
  }
  
  delay(random(5,40));
}

#include <Wire.h>
#include "I2C_Anything.h"
String circumference_str;
float circumference =123.01231;
int8_t x;
void setup(){

  Wire.begin(7);
  Serial.begin(9600);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
}

volatile boolean haveData = false;
volatile float fnum;
volatile long foo;
void loop(){

  Serial.println("working..");
   if (haveData)
    {
    Serial.print ("Received fnum = ");
    Serial.println (fnum,8);  
    Serial.print ("Received foo = ");
    Serial.println (foo,8);  
    haveData = false;  
    }  // end
  
  
}


void requestEvent() {
  float bigNum = 12239.25463443;
  I2C_writeAnything(bigNum);
}




void receiveEvent (int howMany)
 {
 if (howMany >= (sizeof fnum) + (sizeof foo))
   {
   I2C_readAnything (fnum);   
   I2C_readAnything (foo);   
   haveData = true;
   x=fnum;     
   }  // end if have enough data
 
 }  // end of receiveEvent

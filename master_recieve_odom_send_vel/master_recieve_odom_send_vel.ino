#include "I2C_Anything.h"

#include <Wire.h>
volatile float send_odom;
float x;
void setup(){
  Wire.begin(7);
  Wire.onReceive (receiveEvent);
  Serial.begin(9600);

}
volatile boolean haveData = false;
volatile float fnum;
volatile long foo;

void loop(){
  send_odom++;
  delay(500);
  if(haveData){
    Serial.println(fnum);
    Serial.println(foo);
  }
  
  
}





void requestEvent() {
  I2C_writeAnything (send_odom);
  
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

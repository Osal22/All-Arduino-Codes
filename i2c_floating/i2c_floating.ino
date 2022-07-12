#include <Wire.h>
#include "I2C_Anything.h"
void setup(){
  Wire.begin();
  Serial.begin(9600);
  
  
}


void loop(){
  volatile float circumference;
  byte a,b;
  int16_t rec;

  int16_t omega1,omega2,omega3,omega4;
  
  float cir;
Wire.requestFrom(7, 4);
I2C_readAnything(cir);
Serial.println(cir);
float s4 =12.234;
double foo;
 Wire.beginTransmission (7);
    I2C_writeAnything (s4);
    I2C_writeAnything (foo++);
    Wire.endTransmission ();


  
}

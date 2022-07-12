#include <Wire.h>


void setup()

{

  Wire.begin(); // join i2c bus (address optional for master)

}


int8_t x = 0;


void loop()

{

  Wire.beginTransmission(4); // transmit to device #4

  

  Wire.write(x);              // sends one byte  

  Wire.endTransmission();    // stop transmitting


  x++;

  delay(500);

}

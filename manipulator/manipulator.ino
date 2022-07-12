#include <Servo.h>

Servo base;  // create servo object to control a servo
Servo link1;
Servo link2;
Servo link3;




void setup() {
  base.attach(5);  // attaches the servo on pin 9 to the servo object
  link1.attach(6);
  link2.attach(9);
  link3.attach(10);
  Serial.begin(115200);
  base.write(90);   
  link1.write(90);
  link2.write(90);
  link3.write(90);
}

void loop() {
                 
  Serial.println("base link angle:");
   while (Serial.available() == 0) {}  
    String base_angle_str = Serial.readString();    
    int    base_angle=base_angle_str.toInt(); 
    Serial.println(base_angle);
    base.write(base_angle);     

   Serial.println("link1 angle:");
   while (Serial.available() == 0) {}  
    String link1_angle_str = Serial.readString();    
    int    link1_angle=link1_angle_str.toInt(); 
    Serial.println(link1_angle);
    link1.write(link1_angle);
               
  Serial.println("link2 angle:");
   while (Serial.available() == 0) {}  
    String link2_angle_str = Serial.readString();    
    int    link2_angle=link2_angle_str.toInt(); 
  Serial.println(link2_angle);
    link2.write(link2_angle);
    
  Serial.println("link3 angle:");
   while (Serial.available() == 0) {}  
    String link3_angle_str = Serial.readString();    
    int    link3_angle=link3_angle_str.toInt(); 
    Serial.println(link3_angle);
    link3.write(link3_angle);


                  
}

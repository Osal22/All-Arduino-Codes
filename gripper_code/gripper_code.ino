#include <Servo.h>
Servo gripper;

void setup() {
  gripper.attach(5);
  gripper.write(90);
  Serial.begin(115200);

}

void loop() {
    Serial.println("grippr angle:");
   while (Serial.available() == 0) {}  
    String gripper_angle_str = Serial.readString();    
    int    gripper_angle=gripper_angle_str.toInt(); 
    Serial.println(gripper_angle);
    gripper.write(gripper_angle);
  

}

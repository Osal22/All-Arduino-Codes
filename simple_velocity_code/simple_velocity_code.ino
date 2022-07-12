/* Odometry by Huzaifa*/
#include <util/atomic.h>
#include <Encoder.h>
#include <Wire.h>
#define pulses_per_rev 1387
#define one_rev_degree 360
//#define PI 3.14159265
#define PWM 5
#define IN2 6
#define IN1 7


float puls_per_one_rev=pulses_per_rev/one_rev_degree;
float puls_per_one_rev_prev;
float diameter_m=0.07112;
float circumference=0;
float circumference_prev=0;
float send_odom;

volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
volatile long int encoder_pos = 0;
long int last_encoder_pos = 0;
int value = -10;
long int last_millis = 0;
int motor_value = 255;
byte myArray[2];
float rpm_speed;
float target = 0.0000;
int8_t x ;

Encoder myEnc(2, 3);
//   avoid using pins with LEDs attached


void setup() {
  Wire.begin(7);
  Serial.begin(9600);
  Serial.println("Odom Testing ");
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(PWM, 0);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
}

long oldPosition  = -999;

void loop() {
    long newPosition = myEnc.read();
    if (newPosition != oldPosition) {
    puls_per_one_rev=newPosition/3.825;
    //circumference=PI*0.03556*puls_per_one_rev/180;
    circumference=PI*0.07112*puls_per_one_rev/360;
    float delta_d=(((float)circumference) - (float)circumference_prev); 
    float delta_t=(float)((micros() - (float)last_millis))/1000;
    rpm_speed = delta_d/delta_t;
    Serial.print("delta d:= ");
    Serial.println(delta_d);
    Serial.print("delta_t:= ");
    Serial.println(delta_t);
    //Serial.println(newPosition);
    //Serial.print("in degrees wheel has moved:= ");
    //Serial.print(newPosition/3.825);
    //Serial.println();
    Serial.print("circumference:= ");
    Serial.println(circumference,4);
    Serial.print("rpm speed:= ");
    Serial.print(rpm_speed);
    Serial.println();
    delay(20);
    oldPosition = newPosition;
    circumference_prev=circumference;
    send_odom=newPosition;
    last_millis=micros();
  }
  


}
  

_video/* Odometry by Huzaifa*/
#include <util/atomic.h>
#include <Encoder.h>
#include <Wire.h>
#include "I2C_Anything.h"
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
volatile float send_odom;

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
double x ;
const byte MY_ADDRESS =7;

Encoder myEnc(2, 3);
//   avoid using pins with LEDs attached


void setup() {
  Wire.begin (MY_ADDRESS);
  Serial.begin(9600);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  Serial.println("Odom Testing ");
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(PWM, 0);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
}

volatile boolean haveData = false;
volatile float fnum;
volatile long foo;

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
//  if (haveData)
//    {
//    Serial.print ("Received fnum = ");
//    Serial.println (fnum,8);  
//    Serial.print ("Received foo = ");
//    Serial.println (foo,8);  
//    haveData = false;  
//    }  // end

  target=fnum; //velocity is in m/s
//    targ/et=fnum;
//    float kp =    50 ; ///;
//    float kd =    20 ;//0.035;
//    float ki =    15 ; //0.0;

      float kp =    200 ; ///;
    float kd =    20 ;//0.035;
    float ki =    5 ; //0.0;


  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  float e = rpm_speed - target;
     // derivative
  float dedt = (e - eprev) / (deltaT);

  // integral
  eintegral = eintegral + e * deltaT;
  // control signal
  float u = kp * e + kd * dedt + ki * eintegral;
   Serial.println(u);
  // motor power
  float pwr = fabs(u);
  if ( pwr > 255 ) {
    pwr = 255;
  }
  
  // motor direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }
  if (target == 0) {
    pwr = 0;
  }
  // signal the motor
  setMotor(dir, pwr, PWM, IN1, IN2);
  // store previous error
  eprev = e;

  
  
}

void requestEvent() {
//  fl/oat circum=12123.3213;
  ;
  I2C_writeAnything (circumference);
  
}


void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  //  Serial.print("analog value  ");
  //  Serial.print(pwmVal);
  //Serial.println("  ");
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    //Serial.println("f");
  }
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    //Serial.println("r");
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    //Serial.println("st");
  }
}


   




// called by interrupt service routine when incoming data arrives
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
  

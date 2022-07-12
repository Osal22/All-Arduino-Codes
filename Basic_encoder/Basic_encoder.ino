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
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
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
  //Serial.println(puls_per_one_rev);
  
 
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
   
    
   
    Serial.println(newPosition);
    Serial.print("in degrees wheel has moved:= ");
    Serial.print(newPosition/3.825);
    puls_per_one_rev=newPosition/3.825;
    //circumference=PI*0.03556*puls_per_one_rev/180;
    circumference=PI*0.07112*puls_per_one_rev/360;
    
    Serial.println();
    Serial.print("circumference:=   ");
    Serial.println(circumference,4);
    
    
    Serial.println();  
    //target = x;
    target=0.0;
     // PID constants
//  float kp =    2 ; ///;
//  float kd =    0.035 ;//0.035;
//  float ki =    0.111 ; //0.0;
    float kp =    50 ; ///;
    float kd =    20 ;//0.035;
    float ki =    15 ; //0.0;



  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
//    rpm_speed = (((float)circumference - (float)circumference_prev) / 1387.0) * 60.0 * (1000 / (millis() - last_millis));
float delta_d=(((float)circumference) - (float)circumference_prev); 
float delta_t=(float)((micros() - last_millis))/1000;
   rpm_speed = delta_d/delta_t;

  float e = rpm_speed - target;
     // derivative
  float dedt = (e - eprev) / (deltaT);

  // integral
  eintegral = eintegral + e * deltaT;
  // control signal
  float u = kp * e + kd * dedt + ki * eintegral;
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
  Serial.print("u:=  ");
  Serial.println(pwr);
  Serial.print(target);
  Serial.print("   ");

  
  Serial.print("rpm speed:=     ");
  Serial.print(rpm_speed);
  Serial.print("   ");

 
  
    delay(10);
   circumference=circumference_prev;
   last_millis = micros();
   oldPosition = newPosition;
   send_odom=newPosition;
  }
   
    


   

      
  
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  //  Serial.print("analog value  ");
  //  Serial.print(pwmVal);
  Serial.println("  ");
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    Serial.println("f");
  }
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    Serial.println("r");
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    Serial.println("st");
  }
}


//void readEncoder() {
//  int b = digitalRead(ENCB);
//  if (b > 0) {
//    posi++;
//    encoder_pos++;
//  }
//  else {
//    posi--;
//    encoder_pos--;
//  }
//}

void requestEvent() {
  int16_t bigNum = send_odom;
  byte myArray[2];
  myArray[0] = (bigNum >> 8) & 0xFF;
  myArray[1] = bigNum & 0xFF;
  Serial.print("  ");
  Serial.print(bigNum);
  Serial.println();
  Wire.write(myArray, 2);
}


void receiveEvent(int howMany)
{

  x = Wire.read();    // receive byte as an integer
  Serial.println(x);         // print the integer

}

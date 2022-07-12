
#include <Encoder.h>
#include <util/atomic.h>
#include <Wire.h>
#define ENCA 2
#define ENCB 3
#define PWM 5
#define IN2 6
#define IN1 7

Encoder motor_1(2,3);

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
int16_t rpm_speed;
int target = 0;
int8_t x ;
void setup() {
  Wire.begin(7);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  //attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  //   attachInterrupt(digitalPinToInterrupt(ENCB), readEncoder_1, RISING);

  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(PWM, 0);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  //Serial.println("target pos");
}
long oldPosition  = -999;
void loop() {
  long newPosition=motor_1.read();
 
  //target=0;
  target = x;
  // int target = 250*sin(prevT/1e6);
  //target = -1 * 50;

  // PID constants
  float kp =    2 ; ///;
  float kd =    0.035 ;//0.035;
  float ki =    0.111 ; //0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;

  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  int circumference=0;
  rpm_speed = (((float)newPosition - (float)oldPosition) / 374.0) * 60.0 * (1000 / (millis() - last_millis));
  // error

  int e = rpm_speed - target;

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
  // Serial.print("target:=  ");
  Serial.print(target);
  Serial.print("   ");
  //Serial.print("posi:=       ");
  //  Serial.print(pos);
  //  Serial.print("   ");
  //Serial.print("rpm speed:=     ");
  Serial.print(rpm_speed);
  Serial.print("   ");
  //Serial.print("value of recieved x is:=   ");
  //  Serial.print(x);
  // Serial.print("   ");
  //  Serial.println();

  last_encoder_pos = encoder_pos;
  last_millis = millis();
  delay(20);
   if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
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

void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
    encoder_pos++;
  }
  else {
    posi--;
    encoder_pos--;
  }
}

void requestEvent() {
  int16_t bigNum = last_encoder_pos;
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

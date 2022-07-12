   #include <Wire.h>
  #include <ArduinoHardware.h>
  #include <ros.h>
  #include <geometry_msgs/Twist.h>
  #include <std_msgs/String.h>
  #include <tf/transform_broadcaster.h>
  #include <tf/tf.h>
  #include "I2C_Anything.h"
  #include <math.h>
 
  #define pwm_c 5
  #define EN_L 8
  #define IN1_L 32
  #define IN2_L 33
  
  #define EN_R 9
  #define IN1_R 34
  #define IN2_R 35
  
  #define EN_R_b 10
  #define IN1_R_b 36
  #define IN2_R_b 37
   
  
  #define EN_L_b 11
  #define IN1_L_b 38
  #define IN2_L_b 39
  
  double w_r = 0, w_l = 0;
  
  
  double wheel_rad = 0.0325, wheel_sep = 0.295;
  
  geometry_msgs::TransformStamped t;
  tf::TransformBroadcaster broadcaster;
  char base_link[] = "/base_link";
  char odom[] = "/odom";
  float  xytCounts[2];
  ros::NodeHandle nh;
  std_msgs::String str_msg;
  ros::Publisher chatter("chatter", &str_msg);
  int lowSpeed = 200;
  int highSpeed = 50;
  double speed_ang = 0, speed_lin , speed_ylin, angle_c, x_c, y_c;
  
  float w1=0.0,w2=0.0,w3=0.0,w4=0.0,wsum=0.0,lx=0.15,ly=0.25,rad=1.0;
  
  
  void messageCb( const geometry_msgs::Twist& msg) {
  
      angle_c = msg.angular.z;
      x_c     = msg.linear.x;
       y_c     = msg.linear.y;
      wsum=angle_c*(lx+ly);
      w1=1/rad*(x_c-y_c-wsum);
      w2=1/rad*(x_c+y_c+wsum);
      w3=1/rad*(x_c+y_c-wsum);
      w4=1/rad*(x_c-y_c+wsum);
  
  }
  
  
  ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
  
  void setup() {
    Wire.begin();
//    Wire.onR/eceive(receiveEvent);
    pinMode(pwm_c,OUTPUT);
    digitalWrite(pwm_c,0);
    Serial1.begin(57600);
    nh.initNode();
    nh.advertise(chatter);
    nh.subscribe(sub);
    broadcaster.init(nh);
  
  }
  
volatile boolean haveData = false;
volatile float fnum;
volatile long foo;

   
  void loop() {
    double s1=0.0001,s2=0.00001,s3=0.00001,s4=0.0001;
    long foo1 = 0,foo2 = 0,foo3 = 0,foo4 = 0;
    long foo_r1 = 0,foo_r2 = 0,foo_r3 = 0,foo_r4 = 0;
  s1=w1;
  s2=w2;
  s3=w3;
  s4=w4;
  
//  analogWrite(pwm_c, s1);
//  Serial1.println(s1);
//  Serial1.println(s2);
//  Serial1.println(s3);
//  Serial1.println(s4);
  
//    int16_t omega1,omega2,ome/ga3,omega4;
    float omega1=0,omega2=0,omega3=0,omega4=0;

    Wire.requestFrom(4, 4);
    I2C_readAnything (omega1);
    
    Wire.requestFrom(5, 4);
    I2C_readAnything (omega2);
   
    Wire.requestFrom(6, 4);
    I2C_readAnything (omega3);
    
   
    Wire.requestFrom(7, 4);
    I2C_readAnything(omega4);
    //Serial.println(omega4,6);
     float sum;
//    if(omega1==NAN)
//    {
//      omega1=0.000001;
//    }
//    if(omega2==NAN)
//    {
//      omega2=0.0000001;
//    }
//    if(omega3==NAN)
//    {
//      omega3=0.0000001;
//    }
//    if(omega4==NAN)
//    {
//      omega4=0.0000001;
//    }
   
      

       
    
    Wire.beginTransmission(4); 
    I2C_writeAnything (s1);
    I2C_writeAnything (foo4++);
    Wire.endTransmission();
    
    Wire.beginTransmission(5); 
     I2C_writeAnything (s2);
     I2C_writeAnything (foo3++);
    Wire.endTransmission();
    
    Wire.beginTransmission(6); 
    //Wire.write(s3);
     I2C_writeAnything (s3);
     I2C_writeAnything (foo1++);   
    Wire.endTransmission();
      
    Wire.beginTransmission (7);
    I2C_writeAnything (s4);
    I2C_writeAnything (foo2
    ++);
    Wire.endTransmission ();
    
    xytCounts[0] = (omega4+omega1+omega2+omega3) / 4;
    xytCounts[1] = (0 - omega1 + omega2 + omega3 - omega4) / 4;
    xytCounts[2] = (0 - omega1 + omega2 - omega3 + omega4) / 4;
   
  
    
  
    t.header.frame_id = odom;
    t.child_frame_id = base_link;
    
    t.transform.translation.x = (xytCounts[0]);
    t.transform.translation.y = (xytCounts[1]);
    
    t.transform.rotation = tf::createQuaternionFromYaw(xytCounts[2]);
    t.header.stamp = nh.now();
    
    broadcaster.sendTransform(t);
  
   
    nh.spinOnce();
    
    delay(10);
  
  }

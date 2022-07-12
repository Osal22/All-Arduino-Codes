//YWROBOT
//Compatible with the Arduino IDE 1.0
//Library version:1.1
#include <LiquidCrystal_I2C.h>
int incomingByte = 0;   // for incoming serial data
String my_recieved_data ="..";
int encoder_data;

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup()
{
  Serial.begin(115200);
  
  lcd.init();                      // initialize the lcd 
  //
  // Print a message to the LCD.
 
  
  
}


void loop()
{
   if (Serial.available() > 0) {
  // read the incoming byte:
                
  incomingByte=Serial.read();
  my_recieved_data = Serial.readString();
  // say what you got:
  //Serial.print("I received: ");
  //Serial.println(my_recieved_data);
  }
        
  Serial.println(encoder_data);
  Serial.println("velocity_commndas");
  Serial.println();
  lcd.backlight();
  //lcd.setCursor(1,0);
  //lcd.print("my_data");
  //lcd.print(my_recieved_data);
  //lcd.setCursor(0,1);
  int index = my_recieved_data.indexOf(';');
  int index_1=my_recieved_data.indexOf(';',index+1);
  int index_2=my_recieved_data.indexOf(';',index_1+1);
  String sub_S = my_recieved_data.substring(0,index);
  String sub_S_1 = my_recieved_data.substring(index+1,index_1);
  String sub_S_2 = my_recieved_data.substring(index_1+1,index_2);
  float x_vel=sub_S.toFloat();
  float y_vel=sub_S_1.toFloat();
  float z_vel=sub_S_2.toFloat();
  lcd.setCursor(1,0);
  lcd.print(x_vel,6);
  lcd.setCursor(8,0);
  lcd.print(y_vel,6);
  lcd.setCursor(1,1);
  lcd.print(z_vel,6);
  //lcd.print(my_recieved_data);
  //lcd.print(sub_S);
  //lcd.setCursor(0,2);
  lcd.setCursor(2,3);
  lcd.print(incomingByte);
  my_recieved_data="";
  encoder_data++;
}

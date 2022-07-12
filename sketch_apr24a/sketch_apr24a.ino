String data_buff="1253.53626;1231;1232131;2112321;";
void setup()
{
  Serial.begin(9600);
  
}
void loop()
{
  int index = data_buff.indexOf(';');
  int index_1=data_buff.indexOf(';',index+1);
  int index_2=data_buff.indexOf(';',index_1+1);
  String sub_S = data_buff.substring(0,index);
  String sub_S_1 = data_buff.substring(index+1,index_1);
  String sub_S_2 = data_buff.substring(index_1+1,index_2);
  double x_vel=sub_S.toFloat();
  float y_vel=sub_S_1.toFloat();
  float z_vel=sub_S_2.toFloat();

  Serial.println(sub_S);
  Serial.println(sub_S_1);
  Serial.println(sub_S_2);



  
  Serial.print("x is := ");
  Serial.print(x_vel,6);
  Serial.print(" ");
  Serial.println(index);
  
  Serial.print("y is := ");
  Serial.print(y_vel);
  Serial.print(" ");
  Serial.println(index_1);
  
  Serial.print("z is := ");
  Serial.print(z_vel);
  Serial.print(" ");
  Serial.println(index_2);
  
}

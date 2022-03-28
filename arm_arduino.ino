
#include "HCPCA9685.h"
#define  I2CAdd 0x40

HCPCA9685 HCPCA9685(I2CAdd);


void setup() 
{
  HCPCA9685.Init(SERVO_MODE);

  HCPCA9685.Sleep(false);
  Serial.begin(9600) ; 
  int pos_1,pos_2,pos_3,pos_4,pos;
  pos_1 = 90;
  pos_2 = 90;
  pos_3 = 90;
  pos_4 = 90;
  for(pos = 0; pos <= 90; pos++)
  {
    HCPCA9685.Servo(7, pos);
    delay(10);
  }
  for(pos = 0; pos <= 90; pos++)
  {
    HCPCA9685.Servo(8, pos);
    delay(10);
  }
  for(pos = 0; pos <= 90; pos++)
  {
    HCPCA9685.Servo(9, pos);
    delay(10);
  }
  for(pos = 0; pos <= 90; pos++)
  {
    HCPCA9685.Servo(12, pos);
    delay(10);
  }}


void loop() 
{
  unsigned int pos1,pos2,pos3,pos4,pos5,pos,pos_1,pos_2,pos_3,pos_4;
  
    if(Serial.available()) {
    pos1 = Serial.read();
    pos2 = Serial.read();
    pos3 = Serial.read();
    pos4 = Serial.read();
    
   if(pos_1 > pos1)
   {for(pos = pos_1;pos >= pos1;pos--)
   {
    HCPCA9685.Servo(7, pos);
    delay(10);
   }}
   else
   {for(pos = pos_1; pos <= pos1; pos++)
  {
    HCPCA9685.Servo(7, pos);
    delay(10);
  }}
  if(pos_2 > pos2)
   {for(pos = pos_2;pos >= pos2;pos--)
   {
    HCPCA9685.Servo(8, pos);
    delay(10);
   }}
   else
   {for(pos = pos_2; pos <= pos2; pos++)
  {
    HCPCA9685.Servo(8, pos);
    delay(10);
  }}
  if(pos_3 > pos3)
  { for(pos = pos_3;pos >= pos3;pos--)
   {
    HCPCA9685.Servo(9, pos);
    delay(10);
   }}
   else
   {for(pos = pos_3; pos <= pos3; pos++)
  {
    HCPCA9685.Servo(9, pos);
    delay(10);
  }}
  if(pos_4 > pos4)
   {for(pos = pos_4;pos >= pos4;pos--)
   {
    HCPCA9685.Servo(12, pos);
    delay(10);
   }}
   else
   {for(pos = pos_4; pos <= pos4; pos++)
  {
    HCPCA9685.Servo(12, pos);
    delay(10);
  }
    }
  }
  Serial.println(pos1);
  Serial.println(pos2);
  Serial.println(pos3);
  Serial.println(pos4);
}

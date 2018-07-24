#include <Wire.h>
#include "i2cAnything.h"
#include <Servo.h>

Servo myservo;

int servopin=6;

//float feedback;
//float voltage;
float cmd1 = 0;
float cmd2 = 0;
float cmd3 = 0;
float cmd4 = 0;
float cmd5 = 0;
float tailcommand = 0;

float command1_fdbk = 0;
float command2_fdbk = 0;
float command3_fdbk = 0;
float command4_fdbk = 0;
float command5_fdbk = 0;

float feedback1 =0;
float feedback2 = 0;
float feedback3 = 0;
float feedback4 = 0;
float feedback5 = 0;

int address1 = 2;
int address2 = 3;
int address3 = 4;
int address4 = 5;
int address5 = 6;

void setup(){
  Serial.begin(115200);
  Wire.begin();
  myservo.attach(6);
  
}

void loop()
{
  //first, read the command(s) from the serial monitor
if (Serial.available())
{
 char myChar = Serial.read();
 if (myChar == '!')
 {
   cmd1 = Serial.parseFloat();
   cmd2 = Serial.parseFloat();
   cmd3 = Serial.parseFloat();
   cmd4 = Serial.parseFloat();
   cmd5 = Serial.parseFloat();
   tailcommand = Serial.parseFloat();
   tailcommand+=90;
   if(tailcommand>180){
    tailcommand=180;
   }
   else if(tailcommand<0){
    tailcommand=0;
   }
   myservo.write(int(tailcommand));
   //now send feedback about what we saw
//   Serial.print(cmd1);
//   Serial.print("\t");
//   Serial.print(command1_fdbk);
//   Serial.print("\t");
   Serial.print(feedback1);
   Serial.print("\t");
//   Serial.print(cmd2);
//   Serial.print("\t");
//   Serial.print(command2_fdbk);
//   Serial.print("\t");
   Serial.print(feedback2);
   Serial.print("\t");
   Serial.print(feedback3);
   Serial.print("\t");
   Serial.print(feedback4);
   Serial.print("\t");
   Serial.print(feedback5);
   Serial.println();
 }
}
  //then, send the command to the servo board
  Wire.beginTransmission (address1);
  I2C_writeAnything (cmd1);
  Wire.endTransmission ();
  //now, receive feedback from the servo board.
   Wire.beginTransmission(address1);
   Wire.requestFrom(address1, sizeof(feedback1)+sizeof(cmd1));              // request 8 bytes from slave device #2
   I2C_readAnything(feedback1);
   I2C_readAnything(command1_fdbk);
   Wire.endTransmission();
   Wire.beginTransmission (address2);
    I2C_writeAnything (cmd3);
    Wire.endTransmission ();
   Wire.beginTransmission(address2);
   Wire.requestFrom(address2, sizeof(feedback3)+sizeof(cmd3));              // request 8 bytes from slave device #2
   I2C_readAnything(feedback3);
   I2C_readAnything(command3_fdbk);
   Wire.endTransmission();
   
   delay(1);
   
}

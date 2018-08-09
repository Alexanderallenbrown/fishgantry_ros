#include <Wire.h>
#include "i2cAnything.h"

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

float feedback1 = 0;
float feedback2 = 0;
float feedback3 = 0;
float feedback4 = 0;


int address1 = 2;
int address2 = 3;
int address3 = 4;
int address4 = 5;


void setup() {
  Serial.begin(115200);
  Wire.begin();
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
      tailcommand += 90;
      if (tailcommand > 90.0) {
        tailcommand = 90.0;
      }
      else if (tailcommand < -90.0) {
        tailcommand = -90.0;
      }
      //now send feedback about what we saw
      Serial.print(feedback1);
      Serial.print("\t");
      Serial.print(feedback2);
      Serial.print("\t");
      Serial.print(feedback3);
      Serial.print("\t");
      Serial.print(feedback4);
      Serial.println();
    }
  }
  //Write Command to Axis 1
  Wire.beginTransmission (address1);
  I2C_writeAnything (cmd1);
  Wire.endTransmission ();
  //now, receive feedback from Axis 1
  Wire.beginTransmission(address1);
  Wire.requestFrom(address1, sizeof(feedback1) + sizeof(cmd1));            
  I2C_readAnything(feedback1);
  I2C_readAnything(command1_fdbk);
  Wire.endTransmission();
  
  //Write Command to Axis 2
  Wire.beginTransmission (address2);
  I2C_writeAnything (cmd2);
  Wire.endTransmission ();
  //Receive Feedback from Axis 2
  Wire.beginTransmission(address2);
  Wire.requestFrom(address2, sizeof(feedback2) + sizeof(cmd2));            
  I2C_readAnything(feedback2);
  I2C_readAnything(command2_fdbk);
  Wire.endTransmission();

  //Write Command to Axis 3
  Wire.beginTransmission (address3);
  I2C_writeAnything (cmd3);
  Wire.endTransmission ();
  //Receive Feedback from Axis 3
  Wire.beginTransmission(address3);
  Wire.requestFrom(address3, sizeof(feedback3) + sizeof(cmd3));            
  I2C_readAnything(feedback3);
  I2C_readAnything(command3_fdbk);
  Wire.endTransmission();

  //Write Command to Axis 4, which INCLUDES THE TAIL!!
  Wire.beginTransmission (address4);
  I2C_writeAnything (cmd4);
  I2C_writeAnything (tailcommand);
  Wire.endTransmission ();
  //Receive Feedback from Axis 4
  Wire.beginTransmission(address4);
  Wire.requestFrom(address4, sizeof(feedback4) + sizeof(cmd4));            
  I2C_readAnything(feedback4);
  I2C_readAnything(command4_fdbk);
  Wire.endTransmission();

  delay(1);

}

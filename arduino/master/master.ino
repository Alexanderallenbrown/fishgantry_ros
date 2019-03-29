#include <WSWire.h>
//#include <Wire.h>

#include "i2cAnything.h"

//float feedback;
//float voltage;
float cmd1 = 0;
float cmd2 = 0;
float cmd3 = 0;
float cmd4 = 0;
float cmd5 = 0;
float tailcommand = 0;
float command4vec[] = {0,0};

float command1_fdbk = 0;
float command2_fdbk = 0;
float command3_fdbk = 0;
float command4_fdbk = 0;
float command5_fdbk = 0;
float command6_fdbk = 0;

float feedback1 = 0;
float feedback2 = 0;
float feedback3 = 0;
float feedback4 = 0;
float feedback5 = 0;
float feedback6 = 0;

float ofeedback1 = 0;
float ofeedback2 = 0;
float ofeedback3 = 0;
float ofeedback4 = 0;
float ofeedback5 = 0;
float ofeedback6 = 0;

float tau = 0.1;//seconds
float dt = .01;//seconds
float ftnow = 0;
unsigned long dtmicros=100;
unsigned long tnow=0;
unsigned long oldt=0;

int address1 = 102;
int address2 = 103;
int address3 = 104;
int address4 = 105;
int address5 = 6;
int address6 = 8;//needed because we have two y axis motors

bool spoof1 = false;
bool spoof2 = true;
bool spoof3 = true;
bool spoof4 = true;
bool spoof5 = true;
bool spoof6 = true;


void setup() {
  Serial.begin(115200);
  Wire.begin(101);
}

void loop()
{
  tnow = micros();
  dtmicros = tnow-oldt;
  dt = dtmicros*1.0e-6;
  ftnow+=dt;
  oldt = tnow;
  //first, read the command(s) from the serial monitor

  
  if (Serial.available())
  {
    char myChar = Serial.read();
    if (myChar == '!')
    {
//      delayMicroseconds(1000);
      cmd1 = Serial.parseFloat();
      cmd2 = Serial.parseFloat();
      cmd3 = Serial.parseFloat();
      cmd4 = Serial.parseFloat();
      cmd5 = Serial.parseFloat();
      tailcommand = Serial.parseFloat();
      tailcommand += 90;
      if (tailcommand > 180.0) {
        tailcommand = 90.0;
      }
      else if (tailcommand < 0.0) {
        tailcommand = 0.0;
      }

      
      
      
      //now send feedback about what we saw
      Serial.print(ftnow,4);
      Serial.print("\t");
      Serial.print(feedback1,4);
      Serial.print("\t");
      Serial.print(feedback6,4);
      Serial.print("\t");
      Serial.print(feedback3,4);
      Serial.print("\t");
      Serial.print(feedback4),4;
      Serial.print("\t");
      Serial.print(feedback5,4);
      Serial.println();

      //now send i2c as appropriate
      sendI2C();
      
    }
    else{
      //char junk = Serial.read();
    }
  }

//process any spoofed variables
procSpoofs();

  delay(1);

}


void sendI2C(){
  //Write Command to Axis 1
  if(!spoof1){
  Wire.beginTransmission (address1);
  I2C_writeAnything (cmd1);
  Wire.endTransmission ();
//  delayMicroseconds(10);
  //now, receive feedback from Axis 1
  Wire.beginTransmission(address1);
  Wire.requestFrom(address1, sizeof(feedback1) + sizeof(cmd1)); 
  delayMicroseconds(100);            
  I2C_readAnything(feedback1);
  I2C_readAnything(command1_fdbk);
  Wire.endTransmission();
  }
delayMicroseconds(10);
  if(!spoof2){
  //Write Command to Axis 2
  Wire.beginTransmission (address2);
  I2C_writeAnything (cmd5);
  Wire.endTransmission ();
  //Receive Feedback from Axis 2
  Wire.beginTransmission(address2);
  Wire.requestFrom(address2, sizeof(feedback5) + sizeof(cmd5));  
  delayMicroseconds(100);           
  I2C_readAnything(feedback5);
  I2C_readAnything(command5_fdbk);
  Wire.endTransmission();
  }
delayMicroseconds(10);
if(!spoof3){
  //Write Command to Axis 3
  Wire.beginTransmission (address3);
  I2C_writeAnything (cmd3);
  Wire.endTransmission ();
  //Receive Feedback from Axis 3
  Wire.beginTransmission(address3);
  Wire.requestFrom(address3, sizeof(feedback3) + sizeof(cmd3)); 
  delayMicroseconds(100);           
  I2C_readAnything(feedback3);
  I2C_readAnything(command3_fdbk);
  Wire.endTransmission();
}
//
delayMicroseconds(10);
if(!spoof4){
  //Write Command to Axis 4, which INCLUDES THE TAIL!!
  command4vec[0] = cmd4;
  command4vec[1] = tailcommand;
  Wire.beginTransmission (address4);
//  I2C_writeAnything (command4vec);
I2C_writeAnything (cmd4);
I2C_writeAnything (tailcommand);
  Wire.endTransmission ();
  //Receive Feedback from Axis 4
  Wire.beginTransmission(address4);
  Wire.requestFrom(address4, sizeof(feedback4) + sizeof(cmd4)); 
  delayMicroseconds(100);            
  I2C_readAnything(feedback4);
  I2C_readAnything(command4_fdbk);
  Wire.endTransmission();
}

delayMicroseconds(10);


if(!spoof6){
  //Write Command to Axis 6, which is one of the y axis motors
  float thiscmd = cmd2;
  Wire.beginTransmission (address6);
  I2C_writeAnything (thiscmd);
  Wire.endTransmission ();
  //Receive Feedback from Axis 3
  Wire.beginTransmission(address6);
  Wire.requestFrom(address6, sizeof(feedback6) + sizeof(thiscmd)); 
  delayMicroseconds(100);            
  I2C_readAnything(feedback6);
  I2C_readAnything(command6_fdbk);
  Wire.endTransmission();
}
delayMicroseconds(10);
if(!spoof5){
  //Write Command to Axis 5, which is one of the y axis motors
  float thiscmd = cmd2;
  Wire.beginTransmission (address5);
  I2C_writeAnything (thiscmd);
  Wire.endTransmission ();
  //Receive Feedback from Axis 3
  Wire.beginTransmission(address5);
  Wire.requestFrom(address5, sizeof(feedback2) + sizeof(thiscmd));  
  delayMicroseconds(100);           
  I2C_readAnything(feedback2);
  I2C_readAnything(command2_fdbk);
  Wire.endTransmission();
}
  
}

void procSpoofs(){
    //if needed, instead of actually sending these commands anywhere, let's just run each through a low-pass filter to simulate.
  //this will allow us to test interfaces to the master quickly and easily.
  
  if(spoof1){feedback1 = feedback1+dt/tau*(cmd1-feedback1);}
  if(spoof2){feedback5 = feedback5+dt/tau*(cmd2-feedback5);}
  if(spoof3){feedback3 = feedback3+dt/tau*(cmd3-feedback3);}
  if(spoof4){feedback4 = feedback4+dt/tau*(cmd4-feedback4);}
  if(spoof5){feedback2 = feedback2+dt/tau*(cmd5-feedback2);}
//  delayMicroseconds(10);
}


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
float command4vec[] = {0,0};

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

int address1 = 2;
int address2 = 3;
int address3 = 4;
int address4 = 5;
int address5 = 6;
int address6 = 7;//this is becauase need to double up on y axis

void setup(){
  Serial.begin(115200);
  Wire.begin(1);
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
   Serial.print(ftnow);
   Serial.print("\t");
   Serial.print(feedback1);
   Serial.print("\t");
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
  //now, instead of actually sending these commands anywhere, let's just run each through a low-pass filter to simulate.
  //this will allow us to test interfaces to the master quickly and easily.
  feedback1 = feedback1+dt/tau*(cmd1-feedback1);
  feedback2 = feedback2+dt/tau*(cmd2-feedback2);
  feedback3 = feedback3+dt/tau*(cmd3-feedback3);
  feedback4 = feedback4+dt/tau*(cmd4-feedback4);
  feedback5 = feedback5+dt/tau*(cmd5-feedback5);

   
   delay(1);
   
}

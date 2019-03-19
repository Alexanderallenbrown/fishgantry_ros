#include <WSWire.h>
#include "i2cAnything.h"
// Assign your channel in pins
#define CHANNEL_A_PIN 0
#define CHANNEL_B_PIN 1

float testpos = 1.2345678;
//for sending data back through i2c
volatile byte* posFloatPtr;
int Address = 103;

///// NOTE: This version homes the unit if the command is 111.10 exactly ///////
//// ALSO: -222.20 disables the axis (enabled by default) ////
//// ALSO: -333.30 enables the axis (enabled by default) ////

boolean closedloop = true;
boolean menable = true;// this is the motor enable

//pulley radius is .01165 meters (.91/2)
float m2rad = -.638/1.76;

float kp = 50.0;
float ki = 0.0;
float kd = 2.0;
float command = 0;
float oldcommand = 0;
float oldoldcommand = 0;
float battery_voltage = 4.0;

float sinfreq = 2;
float sinamp = PI/2;
float sinangle = 0;

unsigned long microsnow = 0;
unsigned long oldmicros = 0;
int dtmicros = 1000;
float dt = .01;

float e = 0;
float dedt = 0;
float inte = 0;
float olde = 0;


volatile long unCountShared = 0;


int cpr = 64*130;

float posrad = 0;
float oldposrad = 0;
float velrads = 0;

int in1pin = 4;
int in2pin = 5;
int enpin = 6;

int lim1pin = 10;
int lim2pin = 8;

int potval;
float fV;
int V;

void setup() {
  Wire.begin(Address);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  // put your setup code here, to run once:
//  Serial.begin(115200);
delayMicroseconds(1000);
  pinMode(enpin, OUTPUT);
  delayMicroseconds(1000);
  pinMode(in1pin, OUTPUT);
  delayMicroseconds(1000);
  pinMode(in2pin, OUTPUT);
  pinMode(lim1pin,INPUT);
  pinMode(lim2pin,INPUT);
  //attach the interrupts
  pinMode(CHANNEL_A_PIN,INPUT_PULLUP);
  pinMode(CHANNEL_B_PIN,INPUT_PULLUP);
  attachInterrupt(2, channelA, CHANGE);
  attachInterrupt(3, channelB, CHANGE);

}

void homeit(){
    Serial.print("HOMING...");
  //limit lim1 is pin 10, limit lim2 is pin 8.
  //home the axis
  while(digitalRead(lim1pin)==LOW){
    digitalWrite(in1pin, LOW);
    digitalWrite(in2pin, HIGH);
    analogWrite(enpin, 200);
  }
  
  unCountShared=0;
  }

void loop() {

  //encoder read
  static long unCount;
  noInterrupts();
  unCount = unCountShared;
  interrupts();

  //timing
  microsnow = micros();
  dtmicros = microsnow - oldmicros;
  dt = dtmicros / (1000000.0);
  oldmicros = microsnow;

  posrad = (unCount * 2.0 * PI) / (cpr * 1.0);
  velrads = (posrad - oldposrad) / dt;


  //read potentiometer
  //int potval = analogRead(0);


  //now compute the voltage command
  if (closedloop) {
    //figure out where we want motor to go

    //filter the command position using a second order filter
    //COMMAND IS NOW READ FROM i2c MASTER!!!
    //command = newcommand;
    
    //now compute the error
    if(command==-111.1){
      command=0;
      homeit();
      Serial.println("Homing Command");
    }
    else if(command==-222.2){
      menable = false;
      command=posrad;
      Serial.println("DISABLE COMMAND");
    }
    else if(command==-333.3){
      menable = true;
      command=posrad;
      Serial.println("ENABLE COMMAND");
    }

    //command is in meters, so we need to convert before computing error
    
    e = command*m2rad - posrad;//command*m2rad should give command in radians.
    //error derivative
    dedt = (e - olde) / dt;
    //integral of error
    inte = 0;//inte + e * dt;
    //store old error value
    olde = e;
    //compute the voltage signal
    fV = (kp * e + kd * dedt + ki * inte)*255.0/battery_voltage;
    if(fV<-255){
      fV=-255;
    }
    else if(fV>255){
      fV=255;
    }
    //convert to counts (signed) -255 to 255 for analogWrite
    V = int(fV);//fV * 255.0/battery_voltage;
    
  }


  else {
    //this is open loop then... so just map potentiometer position to V directly
    fV = (potval - 512) * 255.0 / 512.0;
    V = int(fV);

  }



if(menable){
  if (V < 0) {
    digitalWrite(in1pin, LOW);
    digitalWrite(in2pin, HIGH);
    analogWrite(enpin, abs(V));
  }
  else {
    digitalWrite(in1pin, HIGH);
    digitalWrite(in2pin, LOW);
    analogWrite(enpin, abs(V));
  }
}
else{
  analogWrite(enpin,0);
}
//
//  Serial.print(dt, 5);
//  Serial.print("\t");
//  Serial.print(V);
//  Serial.print("\t");
//  Serial.print(potval);
//  Serial.print("\t");
//  Serial.print(posrad, 5);
//
//  if(closedloop){
//  Serial.print("\t");
//  Serial.print(e, 3);
//  Serial.print("\t");
//  Serial.print(dedt, 3);
//  Serial.print("\t");
//  Serial.print(inte, 3);
//  }
//
//  Serial.print(menable);
//
//
//  Serial.println();

  delayMicroseconds(1000);
}


  // function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
  //noInterrupts();
  I2C_writeAnything(posrad/m2rad);
  I2C_writeAnything(command);
  //interrupts();
}
void receiveEvent(int howMany){
  if (howMany >= (sizeof command))
   {
    //noInterrupts();
   I2C_readAnything (command); 
      
    //interrupts();
   }  // end if have enough data
}

// simple interrupt service routine
void channelA()
{
  //Serial.println("A");
  if (digitalRead(CHANNEL_A_PIN) == HIGH)
  {
    if (digitalRead(CHANNEL_B_PIN) == LOW)
    {
      unCountShared++;
    }
    else
    {
      unCountShared--;
    }
  }
  else
  {
    if (digitalRead(CHANNEL_B_PIN) == HIGH)
    {
      unCountShared++;
    }
    else
    {
      unCountShared--;
    }
  }
}

void channelB()
{
  //Serial.println("B");
  if (digitalRead(CHANNEL_B_PIN) == HIGH)
  {
    if (digitalRead(CHANNEL_A_PIN) == HIGH)
    {
      unCountShared++;
    }
    else
    {
      unCountShared--;
    }
  }
  else
  {
    if (digitalRead(CHANNEL_A_PIN) == LOW)
    {
      unCountShared++;
    }
    else
    {
      unCountShared--;
    }
  }
}

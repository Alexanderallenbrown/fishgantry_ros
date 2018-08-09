#include <Servo.h>
#include <Wire.h>
#include "i2cAnything.h"
// Assign your channel in pins
#define CHANNEL_A_PIN 0
#define CHANNEL_B_PIN 1

float testpos = 1.2345678;
//for sending data back through i2c
volatile byte* posFloatPtr;
int Address = 5;

Servo tailservo;
float servocommand = 0.0;

boolean closedloop = true;

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


int cpr = 64*20;

float posrad = 0;
float oldposrad = 0;
float velrads = 0;

int in1pin = 4;
int in2pin = 5;
int enpin = 6;

int potval;
float fV;
int V;

void setup() {
  Wire.begin(Address);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  // put your setup code here, to run once:
  Serial.begin(115200);
delayMicroseconds(1000);
  pinMode(enpin, OUTPUT);
  delayMicroseconds(1000);
  pinMode(in1pin, OUTPUT);
  delayMicroseconds(1000);
  pinMode(in2pin, OUTPUT);
  //attach the interrupts
  attachInterrupt(2, channelA, CHANGE);
  attachInterrupt(3, channelB, CHANGE);
  tailservo.attach(11);

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
    sinangle+=dt*sinfreq;
//    float newcommand = sinamp*sin(sinangle)+(potval - 512.0) * 1.0 * PI / 512.0; //one full revolution
    float newcommand = sinamp*sin(sinangle); //one full revolution

    //filter the command position using a second order filter
    //COMMAND IS NOW READ FROM i2c MASTER!!!
    //command = newcommand;
    
    //now compute the error
    e = command - posrad;
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

  if(servocommand>90.0){
    servocommand = 90.0;
  }
  else if(servocommand<-90.0){
    servocommand = -90.0;
  }
  tailservo.write(int(servocommand)+90.0);

  Serial.print(dt, 5);
  Serial.print("\t");
  Serial.print(V);
  Serial.print("\t");
  Serial.print(potval);
  Serial.print("\t");
  Serial.print(posrad, 5);

  if(closedloop){
  Serial.print("\t");
  Serial.print(e, 3);
  Serial.print("\t");
  Serial.print(dedt, 3);
  Serial.print("\t");
  Serial.print(inte, 3);
  }

  Serial.println();

  delayMicroseconds(1000);
}


  // function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
  //noInterrupts();
  I2C_writeAnything(posrad);
  I2C_writeAnything(command);
  //interrupts();
}
void receiveEvent(int howMany){
  if (howMany >= ((sizeof command)+(sizeof servocommand)))
   {
    //noInterrupts();
   I2C_readAnything (command); 
   I2C_readAnything (servocommand);
      
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
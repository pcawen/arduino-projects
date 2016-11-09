
#include "Wire.h"
#include <PID_v1.h>
#include <Adafruit_PWMServoDriver.h>
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  150 //150 //368 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 //600 //467 //this is the 'maximum' pulse length count (out of 4096)
uint8_t servonum = 15;
unsigned long previousMillis = 0;
unsigned long stabilizationWaitMillis = 10000;
boolean doneStabilizing = false;  

/*float Kp = 0.8;
float Ki = 0.07;
float Kd = 0.12;*/
float Kp = 1.2;
float Ki = 0.07;
float Kd = 0.12;
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,Kp,Ki,Kd, DIRECT);

void setup() {
    Serial.begin(115200);
    //Serial.print("Inser tunning values Kp,Ki,Kd,");
    boolean done = false;
    /*while(!done){
      if (Serial.available()){
        Kp = Serial.readStringUntil(',').toFloat();
        Serial.read();
        Ki = Serial.readStringUntil(',').toFloat();
        Serial.read();
        Kd = Serial.readStringUntil(',').toFloat();
        Serial.read();
        done = true;
      }
    }
    
    Serial.print("Kp");
    Serial.print(Kp);
    Serial.print("Ki");
    Serial.print(Ki);
    Serial.print("Kd");
    Serial.println(Kd);
    Serial.print("Send any character");
    while(!Serial.available()){}*/
    /*Serial.print("Inser setpoint");
    while(!done){
      if (Serial.available()){
        //float tSetPoint;
        Setpoint = Serial.readString().toFloat();
        //Setpoint = Setpoint;
        done = true;
      }
    }*/
    
    myPID.SetTunings(Kp,Ki,Kd);
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(SERVOMIN, SERVOMAX);

    Setpoint = 0;
}

int i = -25;

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
  
  for (i; i < 25 ;i++) {
    Input = i;
    myPID.Compute();
    Serial.print(Input);Serial.print(",");Serial.println(Output);
    delay(500);
  }
}



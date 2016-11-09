/*
 * Reads the servo position of a modified servo throught the servo 
 * pot conected to the analog input A0.
 * After reding the servo position you canchange it and make the servo
 * to go to the reded position.
 * 
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  153
#define SERVOMAX  505

uint8_t servoNumber = 0;
int servoPosition = 0;
int analogPosition = 0;
void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);//60
  servoNumber = 15;//15
  Serial.println("Ready");
}

void loop() {
  Serial.println("Select option");
  Serial.println("r - Release servo");
  Serial.println("p - read Position");
  Serial.println("g - Go to position");
  Serial.println("s - Scan all positions");
  boolean done = false;
  while(!done){
    if (Serial.available()){
      char chr = Serial.read();
      done = true;
      switch (chr) {
      case 'r':
        releaseServo();
        break;
      case 'p':
        readCurrentPos();
        Serial.print("Current analog position: ");Serial.println(analogPosition);
        break;
      case 's':
        scanAllPositions();
        break;
      case 'g':
        goToPosition();
        break;
      default: 
      break;
      }
    }
  }
  delay(5);
}

void releaseServo(){
  pwm.setPWM(servoNumber, 4095, 4095);
}

void readCurrentPos(){
  analogPosition = 0;
  int i;
  for(i = 0; i < 11; i++){
    analogPosition = analogPosition + analogRead(A0);
    delay(3);
  }
  analogPosition = analogPosition/11;
}

void scanAllPositions(){
  servoPosition = -176;
  setServoPosition();
  delay(2000);
  int i;
  for(i = -176; i < 177; i++){
    servoPosition = i;
    setServoPosition();
    delay(1000);
    //readCurrentPos();
    Serial.print(servoPosition);Serial.print(";");Serial.print(analogPosition);Serial.print(";");Serial.println(map(analogPosition, 440, 73, -176, 176));
  }
}

void goToPosition(){
  servoPosition = map(analogPosition, 440, 74, -176, 176);
  Serial.print("Analog value: ");Serial.println(analogPosition);
  Serial.print("servo shoud go to: ");Serial.println(servoPosition);
  setServoPosition();
}

void setServoPosition(){
  //Serial.print("Positioning servo number: ");Serial.print(servoNumber);Serial.print(" to ");Serial.print(servoPosition);Serial.println(" degree");
  int duty;
  //duty = map(servoPosition, 0, 352, SERVOMIN, SERVOMAX);
  duty = map(servoPosition, -176, 176, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoNumber, 0, duty);
}

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  153//153//184
#define SERVOMAX  505

uint8_t servoNumber = 0;
int servoPosition = 0;
int analogPosition = 0; // 1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20
int positionsMap[440] = {175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,//20
                         175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,//40
                         175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,//60
                         175,175,175,175,175,175,175,175,175,175,175,175,175,175,173,171,170,168,166,165,//80
                         164,162,161,160,159,158,157,155,154,153,152,151,150,149,148,147,146,145,144,143,//100
                         142,142,141,140,139,138,137,136,135,134,133,132,131,130,130,129,128,127,126,125,//120
                         124,123,123,122,121,120,119,118,117,116,115,115,114,113,112,111,110,109,108,107,//140
                         106,105,104,103,102,101,100, 99, 98, 97, 96, 95, 95, 94, 93, 92, 91, 90, 89, 88,//160
                          88, 86, 85, 84, 83, 83, 82, 81, 80, 79, 78, 77, 76, 75, 74, 74, 73, 72, 71, 70,//180
                          69, 69, 68, 67, 66, 65, 64, 63, 62, 61, 60, 59, 58, 57, 57, 56, 55, 54, 53, 52,//200
                          51, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 42, 41, 40, 39, 38, 37, 36, 35, 34,//220
                          33, 33, 32, 31, 30, 29, 28, 27, 26, 25, 25, 24, 23, 22, 21, 21, 20, 19, 18, 17,//240
                          16, 15, 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4,  3,  2,  1,  1,  0, -1, -2,//260
                          -3, -4, -5, -6, -7, -8, -8, -9,-10,-11,-12,-13,-14,-15,-16,-17,-17,-18,-19,-20,//280
                         -21,-22,-23,-24,-25,-26,-26,-27,-28,-29,-30,-31,-32,-33,-34,-34,-35,-36,-37,-38,//300
                         -39,-40,-41,-42,-43,-43,-44,-45,-46,-47,-48,-49,-49,-50,-51,-52,-53,-54,-55,-56,//320
                         -57,-58,-59,-59,-60,-61,-62,-63,-64,-64,-65,-66,-67,-68,-69,-70,-71,-72,-73,-73,//340
                         -74,-75,-76,-77,-78,-79,-80,-81,-82,-83,-84,-85,-86,-87,-88,-89,-90,-91,-91,-92,//360
                        // 1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18   19   20
                         -93, -94, -95, -96, -97, -98, -99,-100,-101,-101,-102,-103,-104,-105,-106,-107,-108,-108,-109,-110,//380
                        -111,-112,-113,-114,-114,-115,-116,-117,-117,-118,-119,-120,-120,-121,-122,-123,-123,-124,-125,-125,//400
                        -126,-127,-127,-128,-128,-129,-129,-130,-131,-131,-132,-132,-133,-133,-134,-134,-135,-135,-136,-136,//420
                        -137,-137,-138,-138,-138,-139,-139,-140,-140,-140,-141,-141,-142,-142,-142,-143,-143,-143,-144,-145//440
                         };

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
  for(i = -176; i < 176; i++){
    servoPosition = i;
    setServoPosition();
    delay(1000);
    readCurrentPos();
    Serial.print(servoPosition);Serial.print(";");Serial.println(analogPosition);//Serial.print(";");Serial.println(map(analogPosition, 439, 73, -170, 160));
  }
}

void goToPosition(){
  //servoPosition = map(analogPosition, 439, 73, -160, 160);
  servoPosition = positionsMap[analogPosition];
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

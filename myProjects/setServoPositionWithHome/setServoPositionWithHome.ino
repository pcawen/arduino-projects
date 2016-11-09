//TODO:

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  153//153//184 //150
#define SERVOMAX  505//420//500 //500
#define SERVOS_ARRAY_SIZE 16
//Home positions                        |-Right(Black) leg| x  |-Left(White) leg|  x RA LA  x  x
//Home positions                         0  1    2   3   4  5   6   7    8  9  10 11 12 13 14 15
int homePositions[SERVOS_ARRAY_SIZE] = {-8, 0, -90, 15, 15, 0, -5, 15, 176, 0, 55, 0, 0, 0, 0, 0};
int targetPositions[SERVOS_ARRAY_SIZE];
long currentPosition[SERVOS_ARRAY_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0};

unsigned long previousMillis[SERVOS_ARRAY_SIZE];
long speeds[SERVOS_ARRAY_SIZE] = {30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30};
boolean movementDoneLst[SERVOS_ARRAY_SIZE] = {false , false , false , false , false , false , false , false , false , false , false, false , false , false , false , false };

//General wait to avod using delay wich stop all proceses
unsigned long previousWaitMillis = 0;
unsigned long currentWaitMillis = 0;

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  initialize();
  Serial.println("Ready");
}

void initialize(){
  int i;
  for (i = 0; i < SERVOS_ARRAY_SIZE ;i++) {
    targetPositions[i] = homePositions[i];
    currentPosition[i] = homePositions[i];
    setServoPosition(i, homePositions[i]);//Moving servos to home
  }
}

void loop() {
  Serial.println("Select option");
  Serial.println("s - Set position");
  Serial.println("p - Print current positions");
  Serial.println("m - Preset movements");
  boolean done = false;
  while(!done){
    if (Serial.available()){
      char chr = Serial.read();
      done = true;
      switch (chr) {
      case 's':
        selectServo();
        break;
      case 'p':
        printCurrentPos();
        break;
      case 'm':
        presetMovementMenu();
        break;
      default: 
      break;
      }
    }
  }
  delay(5);
}

void presetMovementMenu(){
  Serial.println("Select movement");
  Serial.println("b - A bow");
  Serial.println("c - A complete bow");
  Serial.println("h - Go to home");
  Serial.println("r - Balance right");
  Serial.println("l - Balance left");
  Serial.println("s - A step");
  Serial.println("e - Exit");
  boolean done = false;
  while(!done){
    if (Serial.available()){
      char chr = Serial.read();
      done = true;
      switch (chr) {
      case 'b':
        doABow();
        break;
      case 'c':
        doCompleteBow();
        break;
      case 'h':
        goToHome();
        break;
      case 'r':
        balanceRight();
        break;
      case 'l':
        balanceLeft();
        break;
      case 's':
        aStep();
        break;
      case 'e':
        break;
      default: 
      break;
      }
    }
  }
}

//----Individual movement-----------
void selectServo(){
  Serial.println("Insert servo number: 0 to 15 (99 to quit)");
  uint8_t servoNumber = 0;
  boolean done = false;
  while(!done){
    if (Serial.available()){
      servoNumber = Serial.parseInt();
      done = true;
    }
  }
  if(servoNumber != 99){
    introducePosition(servoNumber);
  }
}

void introducePosition(uint8_t servoNumber){
  Serial.print("Setting position for servo ");Serial.println(servoNumber);;
  Serial.println("Insert servo position: -176 to 176 (999 to quit)");
  int servoPosition = 0;
  boolean done = false;
  while(!done){
    if (Serial.available()){
      servoPosition = Serial.parseInt();
      done = true;
    }
  }
  if(servoPosition != 999){
    targetPositions[servoNumber] = servoPosition;
    doMovemnt();
    introducePosition(servoNumber);
  } else {
    selectServo();
  }
}

//----Commons------------
void setServoPosition(uint8_t servoNumber, int servoPosition){
  //Serial.print("Positioning servo number: ");Serial.print(servoNumber);Serial.print(" to ");Serial.println(servoPosition);
  int duty;
  //duty = map(servoPosition, 0, 352, SERVOMIN, SERVOMAX);
  duty = map(servoPosition, -176, 176, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoNumber, 0, duty);
}

void printCurrentPos(){
  Serial.println("Printing current positions");
  int i;
  for (i = 0; i < SERVOS_ARRAY_SIZE ;i++) {
    //Serial.print(i);Serial.print(" T: ");Serial.print(targetPositions[i]);Serial.print(" C: ");Serial.println(currentPosition[i]);
    Serial.print(i);Serial.print(".");Serial.print(targetPositions[i]);Serial.print(".");Serial.print(currentPosition[i]);Serial.print("|");
  }
}

boolean wait(long interval){
  unsigned long currentMillis = millis();
  if (currentMillis - previousWaitMillis >= interval) {
    return true;
  } else {
    return false; 
  }
}

//-----Movements---------------
void doMovemnt(){
  resetDoneLst();
  setPreviousMillis();
  while( !isMovementDone() ){
    unsigned long currentMillis = millis();
    int i;
    for (i = 0; i < SERVOS_ARRAY_SIZE ;i++) {
      moveServo(currentMillis, i);
    }
  }
  Serial.println("Movement completed");
}

boolean isMovementDone(){
  boolean done = true;
  int i;
  for (i = 0; i < SERVOS_ARRAY_SIZE ;i++) {
    if(movementDoneLst[i] == false){
      return false;
    }
  }
  return done;
}

void setPreviousMillis(){
  unsigned long currentMillis = millis();
  int i;
  for (i = 0; i < SERVOS_ARRAY_SIZE ;i++) {
    previousMillis[i] = currentMillis;
  }
}

void resetDoneLst(){
  int i;
  for (i = 0; i < SERVOS_ARRAY_SIZE ;i++) {
    movementDoneLst[i] = false;
  }
}

void moveServo(unsigned long currentMillis, int servoNumber){
  if(currentPosition[servoNumber] != targetPositions[servoNumber]){
    if (currentMillis - previousMillis[servoNumber] >= speeds[servoNumber]) {
      previousMillis[servoNumber] = currentMillis;
      if(currentPosition[servoNumber] < targetPositions[servoNumber]){
        currentPosition[servoNumber]++;
      }else{
        currentPosition[servoNumber]--;
      }
      setServoPosition(servoNumber, currentPosition[servoNumber]);
    }
  } else {
    movementDoneLst[servoNumber] = true;
  }
}

//-----Preset movements--------
void goToHome(){
  Serial.println("Going home");
  int i;
  for (i = 0; i < SERVOS_ARRAY_SIZE ;i++) {
    targetPositions[i] = homePositions[i];
  }
  doMovemnt();
  presetMovementMenu();
}

void doABow(){
  Serial.println("Doing a bow");
  targetPositions[1] = 40;
  targetPositions[3] = 115;
  targetPositions[7] = -35;
  targetPositions[9] = -100;
  speeds[3] = 15;
  speeds[9] = 15;
  doMovemnt();
  speeds[3] = 30;
  speeds[9] = 30;
  presetMovementMenu();
}

void doCompleteBow(){
  Serial.println("Doing a complete bow");
  Serial.println("Doing a bow");
  targetPositions[1] = 40;
  targetPositions[3] = 115;
  targetPositions[7] = -35;
  targetPositions[9] = -100;
  speeds[3] = 15;
  speeds[9] = 15;
  doMovemnt();
  previousWaitMillis = millis();
  Serial.println("start whaiting");
  while(!wait(2000)){
    //Do some other stuff that needs attention
    //Serial.println("Doing somethig");
  }
  Serial.println("end whaiting");
  Serial.println("Going home");
  int i;
  for (i = 0; i < SERVOS_ARRAY_SIZE ;i++) {
    targetPositions[i] = homePositions[i];
  }
  doMovemnt();
  speeds[3] = 30;
  speeds[9] = 30;
  presetMovementMenu();
}

void balanceRight(){
  Serial.println("Balancing to the right");
  targetPositions[0] = -38;
  targetPositions[4] = -15;
  targetPositions[6] = -35;
  targetPositions[10] = 25;
  doMovemnt();
  presetMovementMenu();
}

void balanceLeft(){
  Serial.println("Balancing to the Left");
  targetPositions[0] = 22;
  targetPositions[4] = 45;
  targetPositions[6] = 25;
  targetPositions[10] = 85;
  doMovemnt();
  presetMovementMenu();
}

/*void aStep(){
  Serial.println("Doing a step forward");
  targetPositions[0] = -38;
  targetPositions[4] = -15;
  targetPositions[6] = -35;
  targetPositions[10] = 25;
  doMovemnt();
  speeds[10] = 50;
  targetPositions[8] = 100;
  targetPositions[9] = -100;
  targetPositions[10] = 55;
  doMovemnt();
  speeds[10] = 30;
  targetPositions[1] = -40;
  targetPositions[8] = 176;
  targetPositions[6] = 25;
  targetPositions[7] = -15;
  targetPositions[0] = -8;
  targetPositions[10] = 65;
  doMovemnt();
  presetMovementMenu();
}*/
/*void aStep(){
  Serial.println("Doing a step forward");
  targetPositions[0] = -38;
  targetPositions[4] = -15;
  targetPositions[6] = -35;
  targetPositions[10] = 25;
  doMovemnt();
  previousWaitMillis = millis();
  //while(!wait(3000)){}
  speeds[10] = 70;
  targetPositions[8] = 130;
  targetPositions[9] = -100;
  targetPositions[10] = 55;
  targetPositions[6] = -5;
  doMovemnt();
  previousWaitMillis = millis();
  //while(!wait(3000)){}
  speeds[10] = 30;
  targetPositions[1] = -40;
  targetPositions[3] = 20;
  targetPositions[8] = 176;
  targetPositions[6] = 25;
  targetPositions[7] = -15;
  targetPositions[0] = -8;
  targetPositions[10] = 65;
  //targetPositions[4] = 0;
  //targetPositions[9] = -70;//hace que se pisen los pies
  doMovemnt();
  previousWaitMillis = millis();
  //while(!wait(3000)){}
  //targetPositions[1] = 40;
  //targetPositions[3] = 0;
  //targetPositions[9] = -80;
  
  targetPositions[7] = -40;
  //targetPositions[2] = -50;
  doMovemnt();
  presetMovementMenu();
}*/
/*void aStep(){
  Serial.println("Doing a step forward");
  speeds[6] = 40;
  speeds[8] = 15;
  targetPositions[0] = -38;
  targetPositions[4] = -15;
  targetPositions[6] = -35;
  targetPositions[10] = 25;
  //----
  targetPositions[7] = 50;
  targetPositions[8] = 100;
  targetPositions[9] = -30;
  doMovemnt();
  speeds[6] = 30;
  //speeds[8] = 30;
  //-----
  targetPositions[8] = 20;
  targetPositions[9] = -80;
  doMovemnt();
  speeds[8] = 30;
  targetPositions[6] = -5;
  targetPositions[8] = 150;
  targetPositions[9] = -100;
  targetPositions[10] = 55;
  doMovemnt();
  targetPositions[1] = -30;
  doMovemnt();
  //speeds[1] = 100;
  //speeds[3] = 300;
  //speeds[7] = 100;
  //speeds[2] = 100;
  targetPositions[1] = -70;
  targetPositions[7] = -40;
  targetPositions[3] = -20;
  targetPositions[2] = -80;
  doMovemnt();
  speeds[1] = 30;
  speeds[3] = 30;
  speeds[7] = 30;
  speeds[2] = 30;
  presetMovementMenu();
}*/

void aStep(){
  Serial.println("Doing a step forward");
  //----
  speeds[0] = 20;
  speeds[1] = 20;
  speeds[2] = 20;
  speeds[3] = 20;
  speeds[4] = 20;
  speeds[5] = 20;
  speeds[6] = 20;
  speeds[7] = 20;
  speeds[8] = 20;
  speeds[9] = 20;
  speeds[10] = 20;
  //----
  //speeds[6] = 13;
  //speeds[8] = 5;
  targetPositions[0] = -30;
  targetPositions[4] = 0;
  targetPositions[6] = -5;
  targetPositions[8] = 100;
  targetPositions[9] = -100;
  targetPositions[10] = 50;
  doMovemnt();
  //speeds[6] = 16;
  //speeds[8] = 10;
  targetPositions[1] = -30;
  targetPositions[7] = 20;
  doMovemnt();
  targetPositions[0] = -15;
  targetPositions[6] = 0;
  doMovemnt();
  targetPositions[0] = -5;
  targetPositions[4] = 15;
  targetPositions[6] = 10;
  targetPositions[10] = 60;
  doMovemnt();
  targetPositions[1] = -45;
  targetPositions[2] = -60;
  targetPositions[3] = 40;
  doMovemnt();
  presetMovementMenu();
}


/*
 * Starting with self balancing from front to back and then left to right.
 */

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <PID_v1.h>
#include <Adafruit_PWMServoDriver.h>
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 //this is the 'maximum' pulse length count (out of 4096)
#define SERVOS_ARRAY_SIZE 16
//Home positions                        |-Left(Black) leg| x  |-Right(White) leg|  x RA LA  x  x
//Home positions                         0  1    2   3   4  5   6   7    8  9  10 11 12 13 14 15
//int homePositions[SERVOS_ARRAY_SIZE] = {-8, 0, -90, 15, 15, 0, -5, 15, 176, 0, 55, 0, 0, 0, 0, 0};
//int targetPositions[SERVOS_ARRAY_SIZE];
//long currentPosition[SERVOS_ARRAY_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0};

//Servo legs identifiers
uint8_t hipFB_R = 9; //Hip front back
uint8_t hipFB_L = 3;
uint8_t hipLR_R = 10; //Hip left right
uint8_t hipLR_L = 4;
uint8_t ankleLR_R = 6;
uint8_t ankleLR_L = 0;

unsigned long previousMillis = 0;
unsigned long stabilizationWaitMillis = 5000;
boolean doneStabilizing = false;  

float Kp = 0.5;
float Ki = 30;
float Kd = 0.05;
//Define Variables we'll be connecting to
double SetpointBF, InputBF, OutputBF,
       SetpointLR, InputLR, OutputLR;
//Specify the links and initial tuning parameters
PID bfPID(&InputBF, &OutputBF, &SetpointBF,Kp,Ki,Kd, DIRECT);
PID lrPID(&InputLR, &OutputLR, &SetpointLR,Kp,Ki,Kd, DIRECT);

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here (MPU6050 mpu(0x69);)
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(108);
    mpu.setYGyroOffset(-12);
    mpu.setZGyroOffset(25);
    mpu.setZAccelOffset(1492); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    pwm.begin();
    pwm.setPWMFreq(60);

    //Move all servos to home
    //Serial.println(F("Seting robot to home state"));
    //initialize();
    //Serial.println(F("End seting robot to home state"));
    
    bfPID.SetMode(AUTOMATIC);
    bfPID.SetOutputLimits(-176, 176);
    bfPID.SetControllerDirection(REVERSE);
    bfPID.SetSampleTime(30);
    
    lrPID.SetMode(AUTOMATIC);
    lrPID.SetOutputLimits(-176, 176);
    //bfPID.SetControllerDirection(REVERSE);
    lrPID.SetSampleTime(30);

    SetpointBF = 0;
    SetpointLR = 0;
    previousMillis = millis();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {

        //doneStabilizing
        if(doneStabilizing){
          InputBF = (ypr[1] * 180/M_PI);
          bfPID.Compute();
          //Serial.print(InputBF);Serial.print(F(","));Serial.println(OutputBF);
          //Substract 15 to one servo because the legs have diference in home position
          setServoPosition(hipFB_L, (OutputBF - 15)*-1);
          setServoPosition(hipFB_R, OutputBF);

          InputLR = (ypr[2] * 180/M_PI);
          //A dead spot to avoid cotinius equilibration movent
          //if(abs(InputLR) >= 2) {
            lrPID.Compute();
            Serial.print(InputLR);Serial.print(F(","));Serial.println(OutputLR);
            //Constraint the output to avoid servos going out of limits
            double constraintOutput, tempOutputLR;
            tempOutputLR = (OutputLR-55)*-1;
            constraintOutput = constrain(tempOutputLR, -150, 90);
            setServoPosition(hipLR_R, constraintOutput);
            tempOutputLR = (OutputLR-15)*-1;
            constraintOutput = constrain(tempOutputLR, -10, 150);
            setServoPosition(hipLR_L, constraintOutput);
            constraintOutput = constrain(OutputLR, -50, 40);
            setServoPosition(ankleLR_R, constraintOutput*-1);
            constraintOutput = constrain(OutputLR, -30, 30);
            setServoPosition(ankleLR_L, constraintOutput*-1);
          //}
        }else{
          if(millis() - previousMillis > stabilizationWaitMillis){
            doneStabilizing = true;
          }
        }

    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

void setServoPosition(uint8_t servoNumber, double servoPosition){
  Serial.print(F("Seting servo "));Serial.print(servoNumber);Serial.print(F(" To pos "));Serial.println(servoPosition);
  int duty = map(servoPosition, -176, 176, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoNumber, 0, duty);
}

/*void initialize() {
  int i;
  for (i = 0; i < SERVOS_ARRAY_SIZE ;i++) {
    setServoPosition(i, homePositions[i]);//Moving servos to home
  }
}*/



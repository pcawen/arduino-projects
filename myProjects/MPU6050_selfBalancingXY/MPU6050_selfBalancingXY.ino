/*
 * Basic program to test theintegrationof the 16 Ch pwm, 
 * MPU6050 and the use of PID librariy and its tunnings.
 * It waits stabilizationWaitMillis befor actuallyrunning the code
 * to wait the MPU6050 to stabilize.
 */

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <PID_v1.h>
#include <Adafruit_PWMServoDriver.h>
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  150 //150 //368 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 //600 //467 //this is the 'maximum' pulse length count (out of 4096)
uint8_t servonum = 15;
unsigned long previousMillis = 0;
unsigned long stabilizationWaitMillis = 5000;
boolean doneStabilizing = false;  

/*float Kp = 0.8;
float Ki = 0.07;
float Kd = 0.12;*/
float Kp = 0.5;
float Ki = 30;
float Kd = 0.05;
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,Kp,Ki,Kd, DIRECT);

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here (MPU6050 mpu(0x69);)
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
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
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
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

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
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

    pwm.setPWM(servonum, 0, (SERVOMAX-SERVOMIN)/2 + SERVOMIN);

    //---Uncomment this to set tunning by serial port
    /*
    Serial.print("Inser tunning values Kp,Ki,Kd,");
    boolean done = false;
    while(!done){
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
    //Serial.print("Send any character");
    //while(!Serial.available()){}
    myPID.SetTunings(Kp,Ki,Kd);
    //---------------------------------------*/
    
    myPID.SetMode(AUTOMATIC);
    //myPID.SetOutputLimits(SERVOMIN, SERVOMAX);
    myPID.SetOutputLimits(-176, 176);
    myPID.SetControllerDirection(REVERSE);
    myPID.SetSampleTime(30);//30//200 by default

    Setpoint = 0;
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
        // other program behavior stuff here
        
        //Serial.print("X Servo position: ");Serial.println((SERVOMAX-SERVOMIN)/2 + SERVOMIN + ypr[1] * 180/M_PI);
        //pwm.setPWM(servonum, 0, (SERVOMAX-SERVOMIN)/2 + SERVOMIN + ypr[1] * 180/M_PI);
        
        //doneStabilizing
        if(doneStabilizing){
          //Input = (ypr[2] * 180/M_PI) * -1;
          Input = (ypr[2] * 180/M_PI);
          //Serial.print("Input: ");Serial.print(Input);
          myPID.Compute();
          //Serial.print(" PID Output: ");Serial.println(Output);
          Serial.print(Input);Serial.print(",");Serial.println(Output);
          //pwm.setPWM(servonum, 0, Output);
          setServoPosition(Output);
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

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.print("ypr\t");
            /*Serial.print(ypr[0] * 180/M_PI);
            Serial.print(" ");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print(" ");
            Serial.println(ypr[2] * 180/M_PI);*/ 
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

void setServoPosition(double servoPosition){
  int duty;
  //duty = map(servoPosition, 0, 352, SERVOMIN, SERVOMAX);
  duty = map(servoPosition, -176, 176, SERVOMIN, SERVOMAX);
  pwm.setPWM(servonum, 0, duty);
}



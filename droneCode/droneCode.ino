
// ================================================================
// ===                      Description                       ===
// ================================================================
//Purpose of this sketch is to rotate a single DC motor at a constant speed
//then, have it speed up/slow down bassed on accl data


// ================================================================
// ===                      wiring                       ===
// ================================================================
//On the H-Bridge, tie the motor1Pin (2 on breadboard) to high and motor2Pin (7 on breadboard) to Low
//SDA is A4
//SCA is A5
const int mpuInteruptPin = 2;
const int motorAPin = 10;
const int motorBPin = 11;
const int motorCPin = 6;
const int motorDPin = 9;




#define LED_PIN 13 // (on board LED)
bool blinkState = false;


// ================================================================
// ===                      Error Codes                       ===
// ================================================================
/*
{1,1}      Failed to initialize MPU
{1,1,1} failed to initialize DMP

*/



// ================================================================
// ===                        Switches                          ===
// ================================================================
#define OUTPUT_READABLE_YAWPITCHROLL
//#define VERBOSE_SERIAL

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

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


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
float stdYPR[3] = {0,0,0};       //


const int baseSpeed = 128; //base speed we are rotating the motor
int power=0; //goes from 0 to 100

unsigned long clock;
unsigned long previousAMicros = 0;
unsigned long previousBMicros = 0;
unsigned long previousCMicros = 0;
unsigned long previousDMicros = 0;

const int maxAcclValue = 30; //experimentally detirmened
const int minAcclValue = -35;  //experimentally detirmined


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void setup() 
{ 
  pinMode(9, OUTPUT);
  pinMode(motorAPin, OUTPUT);
  pinMode(motorBPin, OUTPUT);
  pinMode(motorCPin, OUTPUT);
  pinMode(motorDPin, OUTPUT);

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  

  //9600 is magical. Havent tested to see if any other baud rate is not magical
  Serial.begin(9600);
  setUpMPU();
  Serial.print("ready");
} 
 
 
void loop() 
{
  //clock = micros();
    getYawPitchRoll(&ypr[0]); //fill acceleration array
    // Serial.println(micros()-clock);
    if(Serial.available()){
      power=Serial.read();
    }

    noInterrupts();
    spinRotorA( getSpeedChangeMagnitude(ypr[1] - stdYPR[1], ypr[2] - stdYPR[2]) , minAcclValue, maxAcclValue);
    //Serial.println(micros()-clock);
    spinRotorB( getSpeedChangeMagnitude(ypr[1] - stdYPR[1], -ypr[2] - stdYPR[2]) , minAcclValue, maxAcclValue);
    spinRotorC( getSpeedChangeMagnitude(ypr[1] - stdYPR[1], -ypr[2] - stdYPR[2]) , minAcclValue, maxAcclValue);
    spinRotorD( getSpeedChangeMagnitude(ypr[1] - stdYPR[1], -ypr[2] - stdYPR[2]) , minAcclValue, maxAcclValue);
    interrupts();
}

//take the (pitch actual - pitch desired) and the (roll actual - roll desired), return speed to change
int getSpeedChangeMagnitude(float pitch, float roll) {
  return (int) (power/100)* (pitch + roll)/2;
}



// speed change is number, give max and min to make it releative, controls motorA
int spinRotorA(int speedChange, int minNum, int maxNum) {
  int actualSpeedChange =  map(speedChange, minNum, maxNum, -baseSpeed, 255-baseSpeed);
  int spedeSent = baseSpeed + actualSpeedChange;
  analogWrite(motorAPin, spedeSent);
  //Serial.print("Motor A: ");   Serial.print(spedeSent);

  return spedeSent;
}

// speed change is number, give max and min to make it releative, controls motor B
int spinRotorB(int speedChange, int minNum, int maxNum) {
  int actualSpeedChange =  map(speedChange, minNum, maxNum, -baseSpeed, 255-baseSpeed);
  int spedeSent = baseSpeed + actualSpeedChange;
  analogWrite(motorBPin, spedeSent);
  //Serial.print("   Motor B: "); Serial.print(spedeSent);
  return spedeSent;
}

// speed change is number, give max and min to make it releative, controls motor C
int spinRotorC(int speedChange, int minNum, int maxNum) {
  int actualSpeedChange =  map(speedChange, minNum, maxNum, -baseSpeed, 255-baseSpeed);
  int spedeSent = baseSpeed + actualSpeedChange;
  analogWrite(motorCPin, spedeSent);
  //Serial.print("  Motor C: "); Serial.print(spedeSent);
  return spedeSent;
}

// speed change is number, give max and min to make it releative, controls motor D
int spinRotorD(int speedChange, int minNum, int maxNum) {
  int actualSpeedChange =  map(speedChange, minNum, maxNum, -baseSpeed, 255-baseSpeed);
  int spedeSent = baseSpeed + actualSpeedChange;
  analogWrite(motorDPin, spedeSent);
  //Serial.print("    Motor D: "); Serial.println(spedeSent);
  return spedeSent;
}




unsigned long run_motor(int motorPin, int motorSpeed, unsigned long previousMicros) {
 //motorSpeed is a given value from 0 to 255
 //time on = 1035 + 3.42745*motorSpeed microseconds
 //time off = 21 millisenonds

//set variable that holds current time to the time since the program started
unsigned long currentMicros = micros();
if ((currentMicros - previousMicros) <= 1035 + round(3.42745*motorSpeed)){
 digitalWrite(motorPin, HIGH);
 }
else if (currentMicros - previousMicros <= 21000) {
  digitalWrite(motorPin, LOW);
 }
else {
  previousMicros = currentMicros;
 }
 return previousMicros;
 
}

void errorMPUInitializationFailure() {
  boolean error[2] = {1,1};
  blinkErrorCode(&error[0],2);
}
void errorDMPInitializationFailure() {
  boolean error[3] = {1,1,1};
  blinkErrorCode(&error[0],3);
}


void blinkErrorCode(boolean* errorCode, int len) {
  while (true) {
    int i=0;
    while(i<len) {
      digitalWrite(LED_PIN, errorCode[i]==1 ? (HIGH):(LOW));
      delay(500);
      i++; 
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
    delay(1000);
  }
}

//0 is yaw, 1 is pitch, 2 is roll
//ypr must be a array size 3
void getYawPitchRoll(float *ypr) {
// if programming failed, don't try to do anything
    if (!dmpReady) return;

    // if MPU interrupt or extra packet(s) not available, exit function
    if (!mpuInterrupt && fifoCount < packetSize) {
      return;
    }
    
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;


            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            ypr[0]=ypr[0] * 180/M_PI;
            ypr[1]=ypr[1] * 180/M_PI;
            ypr[2]=ypr[2] * 180/M_PI;
            
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            Serial.print("ypr\t");
            Serial.print(ypr[0]);
            Serial.print("\t");
            Serial.print(ypr[1]);
            Serial.print("\t");
            Serial.println(ypr[2]);
        #endif
    }
}

void setUpMPU() {
     // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    TWBR = 12; // set 400kHz mode @ 16MHz CPU or 200kHz mode @ 8MHz CPU  
    #ifdef VERBOSE_SERIAL    
      Serial.println(F("Initializing I2C devices..."));
    #endif
    mpu.initialize();

    // verify connection
    #ifdef VERBOSE_SERIAL    
      Serial.println(F("Testing device connections..."));
    #endif
    if (mpu.testConnection()) {
      #ifdef VERBOSE_SERIAL    
        Serial.println(F("MPU6050 connection successful"));
      #endif
    } else {
      Serial.println(F("MPU6050 connection failed"));
      errorMPUInitializationFailure();
    }

  /*
    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
  */
    // load and configure the DMP
    #ifdef VERBOSE_SERIAL    
      Serial.println(F("Initializing DMP..."));
    #endif
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(-82);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        #ifdef VERBOSE_SERIAL
          Serial.println(F("Enabling DMP..."));
        #endif
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        #ifdef VERBOSE_SERIAL
          Serial.println(F("Enabling interrupt detection (Arduino external interrupt 2)..."));
        #endif
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
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
        errorDMPInitializationFailure();
       
    }
}

 


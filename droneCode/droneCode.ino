#include <Servo.h>

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

const int motor0Pin = 10; //A
const int motor1Pin = 11; //B
const int motor2Pin = 6; //C
const int motor3Pin = 9; //D





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
// ===                      Constants                           ===
// ================================================================
const int minThorttle = 900; //min throttle
const int maxThorttle = 2000; //max throttle
/*
const int m0YPR[] = { 1, -1,  1};
const int m1YPR[] = {-1,  1,  1};
const int m2YPR[] = {-1,  1, -1};
const int m3YPR[] = { 1, -1, -1};
redone below, values assigned at beggining of setUp()
*/
int mYPR[4][3];

        
const int armSpeed = 900;
const int hoverSpeed = 1200; //random untested value - should be where drone hovers
const int launchSpeed = 1400; //iffy tested exact testing needed -experimentally detirmed to be around here

const int minAcclValue = 0;  //experimentally detirmined
const int maxAcclValue = 100; //experimentally detirmened

const int minYaw = -180;  //TODO: Find real values
const int maxYaw = 180;  //TODO: Find real Values

const int minPitch = -60; //TODO: find real Values
const int maxPitch = 60; //TODO: find real Values

const int minRoll = -70; //TODO: find real Values
const int maxRoll = 70; //TODO: find real Values


const int oldPoints=3;  //history of old motor speed points
//TODO1:Find true value
const float correctionMod = 0.7; //stabilization modifier (ie correction factor multiplied by this value), based on instantaenous ypr
const float dCorrectionMod = 0.3;  //take the derivitive of ypr, this is weighting

const float yawCorrectionMod = 0; //stabilization modifier (ie correction factor multiplied by this value)


const float calibrationPercision = 0.1;  //must be positive

/*
Your offsets:	-4592	-537	700	-1214	-18	0
-4550	-523	753	-1269	-23	-2
-4548	-526	751	-1267	-22	-2



Data is printed as: acelX acelY acelZ giroX giroY giroZ

*/






// ================================================================
// ===                        Switches                          ===
// ================================================================
//#define OUTPUT_READABLE_YAWPITCHROLL
//#define VERBOSE_SERIAL
//#define OUTPUT_YPR_DIFERENCE
//#define MOTOR_SPEEDS
#define MOTOR_SPEEDS1


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
float stdYPR[3];       //base values when its flat

bool newData = false;


int throttle=700; //goes from 0 to 100


Servo motor[4];
float mSpeed[4][oldPoints+1];    //stores old values of ypr, ypr is most current then oldYPR[0] then oldYPR[1]. motor 1,2,3,4 and then [x][0] is speed current, [x][1] is speed
float mSpeedP[4];

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void setup() { 
  //stupid arduino wont initialize 2D arrays
  //set motor multipliers (how a change efects motor speed). 
  int buffer[4][3] = { { 1, -1,  1}, //1
                       {-1,  1,  1}, //1
                       {-1,  1, -1},  //-1
                       { 1, -1, -1}  };  //-1
           
           //fill array
    for (int x=0; x<4; x++){
     for (int y=0; y<3; y++){
       mYPR[x][y] = buffer[x][y];
     }
    }
        
        
  //actualy setup()
  
  setUpMPU();
  
  motor[0].attach(motor0Pin);    // attached to pin 
  motor[1].attach(motor1Pin);    // attached to pin 
  motor[2].attach(motor2Pin);    // attached to pin 
  motor[3].attach(motor3Pin);    // attached to pin 

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  

  //9600 is magical. Havent tested to see if any other baud rate are not magical
  Serial.begin(9600);
     for(int i=0; i<4;i++) {
      motor[i].writeMicroseconds(armSpeed); //low throtle
   } 
   
   #ifdef VERBOSE_SERIAL
      Serial.println(F("Calibrating yaw pitch roll"));
   #endif
   
   //calibrate
  calibrateYPR();
  
  
    // wait for ready
    Serial.println(millis()/1000.);
    Serial.println(F("Press any key to begin: "));
    digitalWrite(LED_PIN,HIGH);
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again  
  

 
}
 
 
void loop() {
  
    newData=getYawPitchRoll(&ypr[0]); //fill acceleration array, and return if we got new data
    
    
    #ifdef OUTPUT_YPR_DIFERENCE
       for(int i=0;i<3; i++) {
         Serial.print(ypr[i]-stdYPR[i]);
         Serial.print(" ");
       }   
       Serial.println();
    #endif
    
    
    if(Serial.available()){
      throttle=Serial.parseInt()*10; //GOES FROM 90 TO 200
      if(throttle<0) { //calibrate
          Serial.println("Calibrating");
          calibrateYPR();
          Serial.println(F("Press any key to begin: "));
          while (Serial.available() && Serial.read()); // empty buffer
          while (!Serial.available());                 // wait for data
          while (Serial.available() && Serial.read()); // empty buffer again
          throttle=700;

      }else if(throttle < 900 || throttle > 2000) {
        throttle=0;
      }
    }

    if(newData) {
      updateSpeeds();
    
    
    noInterrupts();
    spinRotor(motor[0], mSpeed[0][0] ); //spin rotor A
    spinRotor(motor[1], mSpeed[1][0] ); //spin rotor B
    spinRotor(motor[2], mSpeed[2][0]); //spin rotor C
    spinRotor(motor[3], mSpeed[3][0] );  //spin rotor D
    
    
    #ifdef MOTOR_SPEEDS1
    
       for(int i=1;i<2; i++) {
         Serial.print((int) mSpeed[i][0]);
         Serial.print("\t");
       }  
       
       for(int i=1;i<2; i++) {
         Serial.print(ypr[i]-stdYPR[i]);
         Serial.print("\t");
       }  
       
       for(int i=1;i<2; i++) {
         Serial.print((int) mSpeedP[i]);
         Serial.print("\t");
       }  
       
      Serial.println();
    #endif
   
   #ifdef MOTOR_SPEEDS
    Serial.println();
   #endif 
    

    
    interrupts();
    }
}

void updateSpeeds() {
  
    for (int i=0; i<4;i++) {
      mSpeed[i][0] = correctionMod*getSpeedChangeMagnitude( mYPR[i][0]*(ypr[0]-stdYPR[0]),  mYPR[i][1]*(ypr[1]-stdYPR[1]),  mYPR[i][2]*(ypr[2]-stdYPR[2]) ) //from instantaneous
                 + dCorrectionMod*getDComponent(&mSpeed[i][0], oldPoints, i); //
  
      mSpeedP[i]= 1*getSpeedChangeMagnitude( mYPR[i][0]*(ypr[0]-stdYPR[0]),  mYPR[i][1]*(ypr[1]-stdYPR[1]),  mYPR[i][2]*(ypr[2]-stdYPR[2]) ); //from instantaneous
  
  }
  for (int t=oldPoints; t>0; t--) {
    for(int i=0;i<4;i++) {
     mSpeed[i][t]=mSpeed[i][t-1]; 
    }
  }
}


//old speed is an array of speeds where [0] is most current and [numOfPoints-1] is oldest
//predictAhead predicts how many points ahead there are
//TODO: make it so if we have extra points in our history, get a better taylor series
float getDComponent(float* oldValues, int numOfPoints, int motor) {
  float predictAhead=1.0;

     //get the derivatives 
     float d[numOfPoints][numOfPoints];
    //fill the base
    for(int i=0;i<numOfPoints-1;i++) {
     d[0][i]=oldValues[i]; 
    }
    
    for (int i=1;i<numOfPoints;i++) {
      for(int j=0; j<numOfPoints-i; j++) {
        d[i][j]=d[i-1][j+1]-d[i-1][j]; //technically divided by 1 
      }
    }

  float futurePoint = d[0][0];
  for(int i=1; i<numOfPoints-1;i++) {
   futurePoint = futurePoint + d[i][0] /**pow(1,i)*/ /( (float) factorial(i) ); 
  }
  if(motor==1) {
    Serial.print(futurePoint);
    Serial.print("\t");
  }
  return futurePoint;
  
}

int factorial(int n) {
 if(n>2) {
  return n*factorial(n-1);
 } else {
   return n;
 }
}

//take the (pitch actual - pitch desired) and the (roll actual - roll desired), return speed to change
float getSpeedChangeMagnitude(float yaw, float pitch, float roll) {

  //adjust values to -33 to 33 (so total is -99 or 99)
   //yaw
  yaw = mapFloat(yaw, minYaw, maxYaw, -33.0* yawCorrectionMod / 1.0, 33.0 * yawCorrectionMod / 1.0); //to account for multiplying by correction factor when retruned
  //pitch
  pitch = mapFloat(pitch, minPitch, maxPitch, -33.0, 33.0);
 //roll 
  roll = mapFloat(roll, minRoll, maxRoll, -33.0, 33.0);
  
  float cor = (yaw + pitch + roll);
  return cor;
  
}


 
// speed change is number, give max and min to make it releative, controls motorA
int spinRotor(Servo motor, float speedChange) {
  int spedeSent = (int) speedChange+throttle;
  
  #ifdef MOTOR_SPEEDS
    Serial.print(spedeSent);
    Serial.print("\t");
  #endif
  
  motor.writeMicroseconds(spedeSent);
  return spedeSent;
}


void calibrateYPR() {
  bool calibrated=false;
  int trials;
  do{
      trials=16;
     do {
       filStdYPR(&stdYPR[0]);
       delay(100);
       trials=trials/2;
     } while( testStdYPR(&stdYPR[0],trials) && trials > 4 );
   calibrated = trials <= 4;
   }while(!calibrated);
   
}

bool testStdYPR(float *YPR, int trials) {
  //just to make sure its not crazy
  if( abs(YPR[0]+YPR[1]+YPR[2]) > abs(maxYaw+maxPitch+maxRoll)) {
    Serial.println("Crazy!");
    return false;
  }
      
  int trial = trials;
  
    float yDif=0;
    float pDif=0;
    float rDif=0;
    
     while(trial>0) {

     while( !getYawPitchRoll(&ypr[0]) ){delay(10);} //fill acceleration array with a new packet
       #ifdef OUTPUT_YPR_DIFERENCE
         for(int i=0;i<3; i++) {
           Serial.print(ypr[i]-YPR[i]);
           Serial.print("\t");
         }   
         Serial.println();
      #endif
      delay(200);

       
       yDif=ypr[0]-YPR[0];
       pDif=ypr[1]-YPR[1];
       rDif=ypr[2]-YPR[2];

       
        if( (abs(yDif) < calibrationPercision) &&  (abs(pDif) < calibrationPercision) && (abs(rDif) < calibrationPercision) ) {
         trial--; 
         delay(150); //if 100, it breaks, and ypr[i]-stdYPR[i] diverges signifigantly, then stabilizes at around //think s its because its too close to the speed of the MPU6050

        } else {
          Serial.println("FAILLLLl");
         return false; //this isnt a good std, break out 
        }
     }
     

     return true;
  
}

void filStdYPR(float *stdYPR) {

  //give blank slate
  stdYPR[0]=0.0;
  stdYPR[1]=0.0;
  stdYPR[2]=0.0;
  
 //average of 20 values, weighted towards the newest values
 
 int i = 0;
 float tempYPR[3];
 while(i<20) {
   //if mpu has signaled its ready or theres a packet waiting
    while(!getYawPitchRoll(&tempYPR[0])) {delay(3);}
    
    if(abs(tempYPR[0]) < 10000.0) { //no overflow. shouldnt be over 10,000.0
      stdYPR[0] = (2.0*stdYPR[0]+tempYPR[0])/3.0;  
    }
    
    if(abs(tempYPR[1]) < 10000.0) { //to stop overflow - shouldnt be over 10,000.0
      stdYPR[1] = (2.0*stdYPR[1]+tempYPR[1])/3.0;  
    } 
    
    if(abs(tempYPR[2]) < 10000.0) { //to stop overflow on the float - shouldnt be over 10,000.0
      stdYPR[2] = (2.0*stdYPR[2]+tempYPR[2])/3.0;  
    } 
    
    i++;  
 }
 return;
}



void errorMPUInitializationFailure() {
  boolean error[2] = {1,1};
  blinkErrorCode(&error[0],2);
  return;
}
void errorDMPInitializationFailure() {
  boolean error[3] = {1,1,1};
  blinkErrorCode(&error[0],3);
  return;
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
  return;
}


//0 is yaw, 1 is pitch, 2 is roll
//ypr must be a array size 3
//returns true if new packet, else returns false
bool getYawPitchRoll(float *YPR) {
// if programming failed, don't try to do anything
    if (!dmpReady) return false;

    // if MPU interrupt or extra packet(s) not available, exit function
    if (!mpuInterrupt && fifoCount < packetSize) {
      return false;
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
            mpu.dmpGetYawPitchRoll(YPR, &q, &gravity);
            YPR[0]=YPR[0] * 180/M_PI;
            YPR[1]=YPR[1] * 180/M_PI;
            YPR[2]=YPR[2] * 180/M_PI;
            
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            

            Serial.print("ypr\t");
            Serial.print(YPR[0]);
            Serial.print("\t");
            Serial.print(YPR[1]);
            Serial.print("\t");
            Serial.println(YPR[2]);
        #endif
    }

    return true;
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

    // load and configure the DMP
    #ifdef VERBOSE_SERIAL    
      Serial.println(F("Initializing DMP..."));
    #endif
    
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-4537);
    mpu.setYAccelOffset(-540);
    mpu.setZAccelOffset(738); // 1688 factory default for my test chip
    mpu.setXGyroOffset(-1257);
    mpu.setYGyroOffset(-22);
    mpu.setZGyroOffset(-2);
    


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


bool isMPUStable() {
  float YPR[3];
  while(! getYawPitchRoll(&YPR[0])){}
  delay(1500);
   int trial = 8;
  
  //accumalate the error ic case its all going in 1 direction
    float yDif=0;
    float pDif=0;
    float rDif=0;
    
     while(trial>0) {

     while( !getYawPitchRoll(&ypr[0]) ){delay(10);} //fill acceleration array with a new packet
       #ifdef OUTPUT_YPR_DIFERENCE
         for(int i=0;i<3; i++) {
           Serial.print(ypr[i]-YPR[i]);
           Serial.print("\t");
         }   
         Serial.println();
      #endif
      delay(200);

       
       yDif=ypr[0]-YPR[0];
       pDif=yDif+ypr[1]-YPR[1];
       rDif=yDif+ ypr[2]-YPR[2];

       
        if( (abs(yDif)/(float)trial < 10) &&  (abs(pDif)/trial < 10) && (abs(rDif)/trial < 10) ) {
         trial--; 
         delay(150); //if 100, it breaks, and ypr[i]-stdYPR[i] diverges signifigantly, then stabilizes at around //think s its because its too close to the speed of the MPU6050

        } else {
          Serial.print(yDif);
         return false; //this isnt a good std, break out 
        }
     }
     
     
     Serial.print("\n \n");

     return true;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

 


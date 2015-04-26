/*
Coded by Marjan Olesch
Sketch from Insctructables.com
Open source - do what you want with this code!
*/
#include <Servo.h>


int value = 1000; // set values you need to zero
bool pause=false;

const int motor0Pin = 10; //A
const int motor1Pin = 11; //B
const int motor2Pin = 6; //C
const int motor3Pin = 9; //D

Servo motor[4];

void setup() {

  motor[0].attach(motor0Pin);    // attached to pin 
  motor[1].attach(motor1Pin);    // attached to pin 
  motor[2].attach(motor2Pin);    // attached to pin 
  motor[3].attach(motor3Pin);    // attached to pin 
  
      Serial.begin(9600);    // start serial at 9600 baud

  
  

      // wait for ready
    Serial.println(F("Press any key to arm: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
    arm(&motor[0]);
   
    
        // wait for ready
    Serial.println(F("Press any key to begin: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
 
}

void loop() {

//First connect your ESC WITHOUT Arming. Then Open Serial and follo Instructions
 
   for(int i=0; i<4;i++) {
    motor[i].writeMicroseconds(value);
   } 
   
  if(Serial.available()) {
    int in=Serial.parseInt();
    value = (in>=1000 && in <= 2000) ? in:value;    // Parse an Integer from Serial
    pause =! pause;
  }
  
     Serial.println(value);
  
  
  

}

void arm(Servo *motor) {
    //begin arming sequence
   for(int i=0; i<4;i++) {
      motor[i].writeMicroseconds(1000); //low throtle
   } 
   
   delay(5000); //5 seconds is overkill, but want to be sure. This must be written after 4 beeps

  
  Serial.begin(9600);    // start serial at 9600 baud
  
   for(int i=0; i<4;i++) {
      motor[i].writeMicroseconds(2000); //high throtle
   } 
   
   delay(50); //let it get up to speed/register but dont let if lift
   
   for(int i=0; i<4;i++) {
      motor[i].writeMicroseconds(1000); //low throtle
   } 
   
   //end arming sequence
  }

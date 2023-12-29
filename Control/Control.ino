#include "Motors.h"

// Define motor pins based on your hardware setup
const int motorA1 = 10;
const int motorA2 = 9;
const int motorA3 = 5;
const int motorA4 = 11;
const int motorB1 = 50;
const int motorB2 = 3;
const int motorB3 = 4;
const int motorB4 = 52;
const int motorC1 = 26;
const int motorC2 = 24;
const int motorC3 = 7;
const int motorC4 = 22;
const int motorD1 = 43;
const int motorD2 = 41;
const int motorD3 = 12;
const int motorD4 = 45;


// Create an instance of the Motors class
Motors myMotors(motorA1, motorA2, motorA3, motorA4, motorB1, motorB2, motorB3, motorB4, motorC1, motorC2, motorC3, motorC4, motorD1, motorD2, motorD3, motorD4);

void setup() {
  // Initialize motors
  myMotors.InitializeMotors();
  Serial.begin(9600);
  Serial.println("Motors initialized");
}

void loop() {
  // Move forward at speed 150 for 2 seconds
 

  // Turn left at speed 100 for 1 second
 myMotors.turnLeft(150);
  delay(1000);
 myMotors.moveMotors(0, 150);
  delay(2000);
  
 

  // Turn right at speed 100 for 1 second
  /*myMotors.turnRight(128);
  delay(1000);*/

  // Stop motors for 1 second
  myMotors.stopMotors();
  delay(1000);
}
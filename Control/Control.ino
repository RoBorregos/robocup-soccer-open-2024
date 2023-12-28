#include "Motors.h"

// Define motor pins based on your hardware setup
const int motorA1 = 10;
const int motorA2 = 9;
const int motorB1 = 50;
const int motorB2 = 3;
const int motorC1 = 26;
const int motorC2 = 24;
const int motorD1 = 2;
const int motorD2 = 1;

// Create an instance of the Motors class
Motors myMotors(motorA1, motorA2, motorB1, motorB2, motorC1, motorC2, motorD1, motorD2);

void setup() {
  // Initialize motors
  myMotors.InitializeMotors();
  Serial.begin(9600);
  Serial.println("Motors initialized");
}

void loop() {
  // Move forward at speed 150 for 2 seconds
  myMotors.moveMotors(0, 150);
  delay(2000);

  // Turn left at speed 100 for 1 second
  myMotors.turnLeft(100);
  delay(1000);

  // Stop motors for 1 second
  myMotors.stopMotors();
  delay(1000);

  // Turn right at speed 100 for 1 second
  myMotors.turnRight(100);
  delay(1000);

  // Stop motors for 1 second
  myMotors.stopMotors();
  delay(1000);
}
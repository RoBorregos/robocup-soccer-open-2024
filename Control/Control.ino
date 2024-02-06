#include <Arduino.h>

#include "Motor.h"
#include "PID.h"

uint8_t encoderPin = 3;
uint8_t speedPin = 4;
uint8_t in1 = 5;
uint8_t in2 = 6;
uint8_t stby = 7;


Motor motor1(encoderPin, speedPin, in1, in2, stby);
PID pid1(1.0); 

void setup() {
  Serial.begin(9600);
  motor1.InitializeMotor();
  motor1.InitializeDriver();
  pid1.setSetpoint(100.0); 
  pid1.setOutputLimits(0, 255); 
}

void loop() {
  float currentSpeed = motor1.getRPM();
  float controlValue = pid1.computeP(currentSpeed);
  analogWrite(speedPin, controlValue);
}
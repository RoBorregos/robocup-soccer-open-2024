#include <Arduino.h>

#include "Motor.h"
#include "PID.h"

int encoderPin = 3;
int speedPin = 4;
int in1 = 5;
int in2 = 6;
int stby = 7;


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
#include "Motors.h"

const int motorIn1 = 9;
const int motorIn2 = 10;
const int motorPWM = 5;
const int motorSTBY = 11;

const int motor2In1 = 3;   
const int motor2In2 = 50;  
const int motor2PWM = 4;   
const int motor2STBY = 52; 

const int motor3In1 = 24;  
const int motor3In2 = 26;  
const int motor3PWM = 7;   
const int motor3STBY = 22; 

const int motor4In1 = 45; 
const int motor4In2 = 43; 
const int motor4PWM = 12;  
const int motor4STBY = 41;

void setup()
{

  Serial.begin(9600);

  Motors myMotors(
      motorPWM, motorIn1, motorIn2, motorSTBY,
      motor2PWM, motor2In1, motor2In2, motor2STBY,
      motor3PWM, motor3In1, motor3In2, motor3STBY,
      motor4PWM, motor4In1, motor4In2, motor4STBY);

  myMotors.InitializeMotors();
  myMotors.InitializeDriver();
  myMotors.setSpeed(motorPWM, 128);
  myMotors.setSpeed(motor2PWM, 128);
  myMotors.setSpeed(motor3PWM, 128);
  myMotors.setSpeed(motor4PWM, 128);

  myMotors.moveForward();
}

void loop()
{
}


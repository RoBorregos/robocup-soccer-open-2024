#include "Motors.h"

//check motor polarity individually for IN1 and IN2 when robot movement is set to 0 degrees. If not correct, swap IN1 and IN2

//encoders: 2, 3, 18, 19
const int motor4In1 = 9;
const int motor4In2 = 10;
const int motor4PWM = 5;
const int motor4STBY = 11;

//
const int motor3In1 = 52;   
const int motor3In2 = 50;  
const int motor3PWM = 4;   
const int motor3STBY = 8; 

//
const int motorIn1 = 24;  
const int motorIn2 = 26;  
const int motorPWM = 7;   
const int motorSTBY = 22; 

const int motor2In1 = 45; 
const int motor2In2 = 43; 
const int motor2PWM = 13;  
const int motor2STBY = 41;


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
  myMotors.setSpeed(motorPWM, 160);
  myMotors.setSpeed(motor2PWM, 160);
  myMotors.setSpeed(motor3PWM, 160);
  myMotors.setSpeed(motor4PWM, 160);

  myMotors.moveMotors(45, 160);
  delay(1000);
  myMotors.stopMotors();
  myMotors.moveMotors(135, 160);
  delay(1000);
  myMotors.stopMotors();
  myMotors.moveMotors(225, 160);
  delay(1000);
  myMotors.stopMotors();
  myMotors.moveMotors(315, 160);
  delay(1000);
  myMotors.stopMotors();
  

}

void loop()
{
}

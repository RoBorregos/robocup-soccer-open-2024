// Define motor control pins

/*#include <Arduino.h>

const int motorIn1 = 9; 
const int motorIn2 = 10;
const int motorPWM = 5; 
const int motorSTBY = 11; 

const int motor2In1 = 3;  // 
const int motor2In2 =50;  // 
const int motor2PWM = 4;  // 
const int motor2STBY =52; // 

const int motor3In1 = 24;  // 
const int motor3In2 = 26;  // 
const int motor3PWM = 7;  // 
const int motor3STBY = 22; // 

const int motor4In1 = 41;  // 43
const int motor4In2 = 43;  // 41
const int motor4PWM = 12;  // 
const int motor4STBY = 45; // 

void setup() {
  // Set motor control pins as outputs
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorSTBY, OUTPUT);

  //
  pinMode(motor2In1, OUTPUT);
  pinMode(motor2In2, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2STBY, OUTPUT);
  //
  pinMode(motor3In1, OUTPUT);
  pinMode(motor3In2, OUTPUT);
  pinMode(motor3PWM, OUTPUT);
  pinMode(motor3STBY, OUTPUT);
  //
  pinMode(motor4In1, OUTPUT);
  pinMode(motor4In2, OUTPUT);
  pinMode(motor4PWM, OUTPUT);
  pinMode(motor4STBY, OUTPUT);

  // Enable the motor driver
  digitalWrite(motorSTBY, HIGH);
  digitalWrite(motor2STBY, HIGH);
  digitalWrite(motor3STBY, HIGH);
  digitalWrite(motor4STBY, HIGH);
  //PWM
  analogWrite(motorPWM, 128); // Set speed (0 to 255)
  analogWrite(motor2PWM, 128); // Set speed (0 to 255)
  analogWrite(motor3PWM, 128); // Set speed (0 to 255)
  analogWrite(motor4PWM, 128); // Set speed (0 to 255)
  
}

void loop() {
  // Spin motor forward
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  digitalWrite(motor2In1, HIGH);
  digitalWrite(motor2In2, LOW);
  digitalWrite(motor3In1, HIGH);
  digitalWrite(motor3In2, LOW);
  digitalWrite(motor4In1, HIGH);
  digitalWrite(motor4In2, LOW);
  delay(2000);

  // Spin motor in reverse
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, HIGH);
  digitalWrite(motor2In1, LOW);
  digitalWrite(motor2In2, HIGH);
  digitalWrite(motor3In1, LOW);
  digitalWrite(motor3In2, HIGH);
  digitalWrite(motor4In1, LOW);
  digitalWrite(motor4In2, HIGH);
  delay(2000);

  // Stop motor
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, LOW);
  digitalWrite(motor2In1, LOW);
  digitalWrite(motor2In2, LOW);
  digitalWrite(motor3In1, LOW);
  digitalWrite(motor3In2, LOW);
  digitalWrite(motor4In1, LOW);
  digitalWrite(motor4In2, LOW);
  delay(1000);
}
*/
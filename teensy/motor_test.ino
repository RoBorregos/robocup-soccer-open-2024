// Define motor control pins

#include <Arduino.h>

const int motorIn1 = 35; 
const int motorIn2 = 34;
const int motorPWM = 33; 

const int motor2In1 = 21;  // 
const int motor2In2 =22;  // 
const int motor2PWM = 23;  // 

const int motor3In1 = 32;  // 
const int motor3In2 = 31;  // 
const int motor3PWM = 29;  // 

const int motor4In1 = 5;  // 43
const int motor4In2 = 4;  // 41
const int motor4PWM = 3;  // 

void setup() {
  // Set motor control pins as outputs
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorPWM, OUTPUT);

  //
  pinMode(motor2In1, OUTPUT);
  pinMode(motor2In2, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  //
  pinMode(motor3In1, OUTPUT);
  pinMode(motor3In2, OUTPUT);
  pinMode(motor3PWM, OUTPUT);

  pinMode(motor4In1, OUTPUT);
  pinMode(motor4In2, OUTPUT);
  pinMode(motor4PWM, OUTPUT);



  //PWM
  analogWrite(motorPWM, 200); // Set speed (0 to 255)
  analogWrite(motor2PWM, 200); // Set speed (0 to 255)
  analogWrite(motor3PWM, 200); // Set speed (0 to 255)
  analogWrite(motor4PWM, 200); // Set speed (0 to 255)
  
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
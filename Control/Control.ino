// Define motor control pins
//there are 56 pulses per revolution

//measuring we get 24 pulses per revolution

#include <Arduino.h>
#include "Motor.h"
// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
#define ENC_IN_RIGHT_A 3
uint8_t in1 = 9; 
uint8_t in2 = 10;
uint8_t speedPin = 5; 
uint8_t stby = 11;
int _currentMillis = 0;
int _interval = 2000;
int _previousMillis = 0;
int rpm = 0;
float _ang_velocity = 0;

Motor motor1(ENC_IN_RIGHT_A, speedPin, in1, in2, stby);
 
// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;
 
void setup() {
 
  // Open the serial port at 9600 bps
  Serial.begin(9600); 
 
  // Set pin states of the encoder
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
 
  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
  motor1.InitializeMotor();
  motor1.InitializeDriver();
  motor1.setSpeed(speedPin, 255);
}
 
void loop() {
  _currentMillis = millis();
  if(_currentMillis - _previousMillis > _interval){
    _previousMillis = _currentMillis;
     right_wheel_pulse_count = 0;
     rpm = 0;
  }
  Serial.print(" RPM: ");
  motor1.moveForward();
  rpm = (right_wheel_pulse_count * 60 / 63);
  _ang_velocity = (rpm * 2 * PI) / 60;
  Serial.print(" RPM: ");
  Serial.println(rpm);
  Serial.print(" Angular Velocity: ");
  Serial.println(_ang_velocity);


  
  
}
 
// Increment the number of pulses by 1
void right_wheel_pulse() {
  right_wheel_pulse_count++;
}
/*#include <Arduino.h>
#include "Motor.h"
#include "Motors.h"
#include "PID.h"

uint8_t encoderPin = 2;
uint8_t in1 = 9; 
uint8_t in2 = 10;
uint8_t speedPin = 5; 
uint8_t stby = 11; 
int currentMillis = 0;
int interval = 1000; 
int previousMillis = 0;



Motor motor1(encoderPin, speedPin, in1, in2, stby);

PID pid1(1);

void setup() {
  Serial.begin(9600);
  motor1.InitializeMotor();
  motor1.InitializeDriver();
  motor1.setSpeed(speedPin, 255);
  //setpoint cannot be a pwm
 // pid1.setSetpoint(100.0);

  //pid1.setOutputLimits(100, 255);
}

void loop() {
currentMillis = millis();
if(currentMillis - previousMillis > interval){
  previousMillis = currentMillis;
  motor1.moveForward(); 
//if setpoint - rpm is 0, then there is no error and motor stops

  float currentSpeed = motor1.getRPM();

  //float controlValue = pid1.computeP(currentSpeed);

  //motor1.setSpeed(speedPin, controlValue); // Set the speed of the motor
    Serial.println(currentSpeed);
  
}


}
*/




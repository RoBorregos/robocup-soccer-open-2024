#ifndef constants_h
#define constants_h
#include <stdint.h>

// Motor 4
const uint8_t MOTOR4_IN1 = 23;
const uint8_t MOTOR4_IN2 = 22;
const uint8_t MOTOR4_PWM = 15;

// Motor 3
const uint8_t MOTOR3_IN1 = 6;
const uint8_t MOTOR3_IN2 = 25;
const uint8_t MOTOR3_PWM = 20;

// Motor 1
const uint8_t MOTOR1_IN1 = 8;
const uint8_t MOTOR1_IN2 = 9;
const uint8_t MOTOR1_PWM = 10;

// Motor 2
const uint8_t MOTOR2_IN1 = 14;
const uint8_t MOTOR2_IN2 = 11;
const uint8_t MOTOR2_PWM = 21;

// Serial pins
const uint8_t PIN_SERIAL1_TX = 0;
const uint8_t PIN_SERIAL1_RX = 1;

// Receive signals
const uint8_t RECEIVE_BNO = 's';
const uint8_t RECEIVE_CAM = 'c';
const uint8_t RECEIVE_DISTANCE = 'd';

#endif  
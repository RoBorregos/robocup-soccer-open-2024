#ifndef constants_h
#define constants_h
#include <stdint.h>

// Motor 4
const uint8_t MOTOR3_IN1 = 23;
const uint8_t MOTOR3_IN2 = 22;
const uint8_t MOTOR3_PWM = 15;

// Motor 3
const uint8_t MOTOR2_IN1 = 25;
const uint8_t MOTOR2_IN2 = 6;
const uint8_t MOTOR2_PWM = 20;

// Motor 1
const uint8_t MOTOR4_IN1 = 9;
const uint8_t MOTOR4_IN2 = 8;
const uint8_t MOTOR4_PWM = 10;

// Motor 2
const uint8_t MOTOR1_IN1 = 14;
const uint8_t MOTOR1_IN2 = 11;
const uint8_t MOTOR1_PWM = 21;

// Serial pins


// Receive signals
const uint8_t RECEIVE_BNO = 's';
const uint8_t RECEIVE_BALL_ANGLE= 'c';
const uint8_t RECEIVE_BALL_DISTANCE = 'd';
const uint8_t RECEIVE_GOAL_ANGLE = 'g';
const uint8_t RECEIVE_GOAL_DISTANCE = 'k';

#endif  
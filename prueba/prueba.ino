/*Main code for the goalkeeper robot*/
// Work in progress. Still defining objectives and requirements
//  The robot moves in its own axis to follow the ball and moves right or left to keep the ball in the center of the camera using PID controller for the camera tracking and Bang Bang for the traslation
#include <Arduino.h>
#include "serial.h"
#include "motors.h"
#include "constants.h"

#include <typeinfo>

#define PIN_SERIAL1_TX (0u)
#define PIN_SERIAL1_RX (1u)

SerialCommunication serialComm(Serial1);

PID pid_w(0.3, 0.0016, 35, 200);

float angle_line = 0; 
float bno_angle = 0; 
float target_angle = 0; 

Motors myMotors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);


void setup()
{
   
    Serial.begin(115200);
    while (!Serial && millis() < 10000UL)
        ;
    Serial.println("Started");
    Serial1.begin(9600);
}

void loop()
{

    angle_line = serialComm.Receive(RECEIVE_LINE_ANGLE);
    bno_angle = serialComm.Receive(RECEIVE_BNO);
    double speed_w = pid_w.Calculate(target_angle, bno_angle);
    Serial.println(angle_line);
    if(angle_line == -1){
        myMotors.MoveMotorsImu(0, 160, speed_w);
    } else{
        myMotors.MoveMotorsImu(angle_line, 160, speed_w);
    }
   
    
}
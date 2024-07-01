#include <iostream>
#include <Wire.h>
#include "Bno.h"
#include <Pixy2SPI_SS.h>
#include <Arduino.h>
#include "PID.h"
#include "Motors.h"
#include <typeinfo>

float angle = 0;
double adjust_angle = 0; 
double translation_angle = 0; 

BNO055 myBNO;
Motors myMotors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);

PID pid_w(0.6, 0, 0, 200);



void setup()
{
    Serial1.begin(115200);
    Serial.begin(9500);

    myMotors.InitializeMotors();
    myBNO.InitializeBNO();
}

void loop()
{
    myBNO.GetBNOData();
    angle = myBNO.GetYaw();
    translation_angle = 0;
    adjust_angle = translation_angle - 90;
    double speed_w = pid_w.Calculate(0, angle);
    if(speed_w != 0){
    myMotors.MoveMotorsImu(0, 0, speed_w);
    Serial.print(speed_w);
     Serial.print("angle");
     Serial.println(angle);
    }
}
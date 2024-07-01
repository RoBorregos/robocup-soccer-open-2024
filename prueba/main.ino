#include <iostream>
#include <Wire.h>
#include "Bno.h"
#include <Pixy2SPI_SS.h>
#include <Arduino.h>
#include "PID.h"
#include "Motors.h"
#include <typeinfo>

float bno_angle = 0;
float distance_pixels = 0;
float distance_pixels_protect = 0;
float ball_angle = 0;
float ball_distance = 0;
float goal_angle = 0;
float goal_distance = 0;
double last_time = 0;
double current_time = 0;
float angle = 0;
float ball_angle_180 = 0;
bool ball_found = false;
bool goal_blue_found = false;
double ponderated_angle = 0;
double target_angle = 0;
int goal_threshold = 9;

Motors myMotors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);

void setup()
{
    Serial1.begin(115200);
    Serial.begin(9500);
    myMotors.InitializeMotors();
  
}

void loop()
{
    // ----------------- Gather data from OpenMV camera via UART ----------------- //
    if (Serial1.available())
    {
        String camString = Serial1.readStringUntil('\n');
        ball_distance = camString.toFloat();
        ball_angle = camString.substring(camString.indexOf(' ') + 1).toFloat();
        goal_angle = camString.substring(camString.indexOf(' ', camString.indexOf(' ') + 1) + 1, camString.lastIndexOf(' ')).toFloat();
        distance_pixels = camString.substring(camString.lastIndexOf(' ') + 1).toFloat();
        distance_pixels_protect = camString.substring(camString.indexOf(' ', camString.lastIndexOf(' ') + 1)).toFloat();
    }

    // ----------------- Gather data from Pixy2 camera via SPI ----------------- //

        //--------------------Separate coordinate plane--------------------------//

        if (ball_angle < 180)
        {
            ball_angle_180 = -ball_angle;
        }
        else if (ball_angle > 180)
        {
            ball_angle_180 = 360 - ball_angle;
        }

        //---------------------------Logic for ball found-------------------------------//

        if (ball_angle == 0)
        {
            ball_found = false;
        }
        else
        {
            ball_found = true;
        }

        //-------------------------Implemenatation to center robot in goal-------------------------------------//

        if (ball_found)
        {

                ball_angle = 360 - ball_angle; 
                myMotors.MoveMotors(ball_angle, 200);
                Serial.print("BALL ANGLE: ");
                Serial.println(ball_angle);
          
        }
}

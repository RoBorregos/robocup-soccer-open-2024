#include <iostream>
#include <Wire.h>
#include "prueba/Bno.h"
#include <Pixy2SPI_SS.h>
#include <Arduino.h>
#include "constantsMPU.h"
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

Pixy2SPI_SS pixy;
Bno myBNO;
Motors myMotors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);

PID pid_w(0.8, 0, 5, 200);
PID pid_t_ball(60, 1, 0, 320);
PID pid_t_goal(20, 1, 0, 255);

void setup()
{
    Serial1.begin(115200);
    Serial.begin(9500);
    pixy.init();
    myMotors.InitializeMotors();
    myBNO.InitializeBNO();
}

void loop()
{

    // ----------------- Gather data from OpenMV camera via UART ----------------- //
    myBNO.getBNOData();
    angle = myBNO.getYaw();
    if (Serial1.available())
    {
        String camString = Serial1.readStringUntil('\n');
        ball_distance = camString.toFloat();
        ball_angle = camString.substring(camString.indexOf(' ') + 1).toFloat();
        goal_angle = camString.substring(camString.indexOf(' ', camString.indexOf(' ') + 1) + 1, camString.lastIndexOf(' ')).toFloat();
        distance_pixels = camString.substring(camString.lastIndexOf(' ') + 1).toFloat();
        distance_pixels_protect = camString.substring(camString.indexOf(' ', camString.lastIndexOf(' ') + 1)).toFloat();
    }
    String angleString = String(angle);
    String ballDistance = String(ball_distance);
    String ballAngle = String(ball_angle);
    String goalAngle = String(goal_angle);
    String distancePixels = String(distance_pixels);
    Serial.print(angleString);
    Serial.print(",");
    Serial.print(ballDistance);
    Serial.print(",");
    Serial.print(ballAngle);
    Serial.print(",");
    Serial.print(goalAngle);
    Serial.print(",");
    Serial.print(distancePixels);
    Serial.println();

    // ----------------- Gather data from Pixy2 camera via SPI ----------------- //

    int i;
    pixy.ccc.getBlocks();

    if (pixy.ccc.numBlocks)
    {
        Serial.print("Detected ");
        Serial.println(pixy.ccc.numBlocks);
        for (i = 0; i < pixy.ccc.numBlocks; i++)
        {
            Serial.print("  block ");
            Serial.print(i);
            Serial.print(": ");
            pixy.ccc.blocks[i].print();
        }
    }

    //--------------------PID controller for the robot--------------------------//
    double speed_w = pid_w.Calculate(target_angle, bno_angle);
    double speed_t_goal = pid_t_goal.Calculate(180, goal_angle);
    double speed_t_ball = pid_t_ball.Calculate(0, ball_distance);

    if (speed_w != 0)
    {
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
        //--------------------Implementation if ball found-----------------------------------//
        //&& distance_pixels > 90 && distance_pixels != 0
        Serial.println(ball_found);

        //-------------------------Implemenatation to center robot in goal-------------------------------------//

        if (ball_found)
        {

            // Ball
            Serial.println("BALL FOUND");
            if (ball_angle_180 > -15 && ball_angle_180 < 15)
            {

                myMotors.MoveMotorsImu(0, abs(speed_t_ball), speed_w);
            }
            else
            {
                ball_angle = 360 - ball_angle;
                double differential = ball_angle * 0.12;
                ponderated_angle = ball_angle - differential;
                ponderated_angle = ball_angle > 180 ? ball_angle - differential : ball_angle + differential;
                myMotors.MoveMotorsImu(ponderated_angle, abs(speed_t_ball), speed_w);
            }
        }
        else if (goal_angle > 0)
        {
            if (goal_angle < (185 - goal_threshold) && ball_found == false)
            {
                myMotors.MoveMotorsImu(270, abs(speed_t_goal), speed_w);
            }
            else if (goal_angle > (185 + goal_threshold) && ball_found == false)
            {
                myMotors.MoveMotorsImu(90, abs(speed_t_goal), speed_w);
            }
            else
            {
                myMotors.MoveMotorsImu(ball_angle, 0, speed_w);
            }
        }
    }
}
}
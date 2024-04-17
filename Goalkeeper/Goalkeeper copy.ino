/*Main code for the goalkeeper robot*/
// Work in progress. Still defining objectives and requirements
//  The robot moves in its own axis to follow the ball and moves right or left to keep the ball in the center of the camera using PID controller for the camera tracking and Bang Bang for the traslation
#include <Arduino.h>
#include "constants.h"
#include "PID.h"
#include "serial.h"
#include "Motors.h"
#include <typeinfo>

#define PIN_SERIAL1_TX (0u)
#define PIN_SERIAL1_RX (1u)

SerialCommunication serialComm(Serial1);

PID pid_w(0.3, 0.0016, 35, 200);
PID pid_t_ball(60, 1, 0, 320);
PID pid_t_goal(20, 1, 0, 255);

float bno_angle = 0;
float ball_angle = 0;
float ball_distance = 0;
float goal_angle = 0;
float goal_distance = 0;
float ball_angle_180 = 0;
bool ball_found = false;
double ponderated_angle = 0;
double target_angle = 0; 

Motors myMotors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);

void setup()
{
    myMotors.InitializeMotors();
    Serial.begin(115200);
    while (!Serial && millis() < 10000UL)
        ;
    Serial.println("Started");
    Serial1.begin(115200);
}

void loop()
{

if (Serial1.available() > 0) {

    String data = Serial1.readStringUntil('\n');
    int values[5]; 
    int index = 0;
    char* ptr = strtok(const_cast<char*>(data.c_str()), ",");
    while (ptr != NULL && index < 5) { 
      values[index++] = atoi(ptr);
      ptr = strtok(NULL, ",");
    }

    int ball_distance = values[0];
    int ball_angle = values[1];
    int goal_angle = values[2];
    int goal_distance = values[3];
    int bno_angle = values[4];

    Serial.print("Angle: ");
    Serial.println(ball_distance);
    Serial.print("Ball Distance ");
    Serial.println(ball_angle);
    Serial.print("Ball Angle: ");
    Serial.println(goal_angle);
    Serial.print("Goal Angle: ");
    Serial.println(goal_distance);
    Serial.print("Goal Distance: "); 
    Serial.println(bno_angle);
}


//--------------------Receive data from sensors--------------------------//

   

//--------------------PID controller for the robot--------------------------//

    double speed_w = pid_w.Calculate(target_angle, bno_angle);
    double speed_t_goal = pid_t_goal.Calculate(180, goal_angle);
    double speed_t_ball = pid_t_ball.Calculate(0, ball_distance);

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
    //if(angle_line == -1){
    if (ball_found)
    {
        // Ball
        if (ball_angle_180 > -15 && ball_angle_180 < 15)
        {
            myMotors.MoveMotorsImu(0, abs(speed_t_ball), speed_w);
        }
        else
        {
            ball_angle = 360 - ball_angle;
            double differential = ball_angle * 0.15;
            ponderated_angle = ball_angle - differential;
            ponderated_angle = ball_angle > 180 ? ball_angle - differential : ball_angle + differential;
            myMotors.MoveMotorsImu(ponderated_angle, abs(speed_t_ball), speed_w);
        }
    }

//-------------------------Implementation to center robot in goal-------------------------------------//

    else if (goal_angle != 0)
    {
        if (goal_angle < (185 - goal_threshold) && ball_found == false)
        {
            myMotors.MoveMotorsImu(270, abs(speed_t_goal), speed_w);
            Serial.println("LEFT GOAL");
        }
        else if (goal_angle > (185 + goal_threshold) && ball_found == false)
        {
            myMotors.MoveMotorsImu(90, abs(speed_t_goal), speed_w);
            Serial.println("RIGHT GOAL");
        }
        else
        {
            myMotors.MoveMotorsImu(0, 0, speed_w);
            Serial.println("STOP");
        }
    }
    //}
    /*else{
        myMotors.MoveMotorsImu(angle_line, 150, speed_w);
    }*/
}
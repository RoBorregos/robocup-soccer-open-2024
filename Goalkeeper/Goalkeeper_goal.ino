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
// tune this
PID pid_t_ball(60, 1, 0, 320);
PID pid_t_goal(20, 1, 0, 255);

// Receive Data from ESP32
float bno_angle = 0;
float ball_angle = 0;
float ball_distance = 0;
float goal_angle = 0;
float goal_distance = 0;
float ball_angle_180 = 0;

double last_time = 0;

// Data used for the control
float diff_angle = 0;
float target_goal_angle = 0;
bool ball_found = false;
double ponderated_angle = 0;

// PID
double kp = 1.05;
double ki = 3.2;
double kd = 0.2;
double target_angle = 0; // frame
double traslation_angle = 0;
double last_error = 0;
double all_error = 0;
double max_error = 30;

// Bang Bang / histéresis / On-off
float ball_threshold = 9;
float goal_threshold = 13.5;

// Traslation
int speed_traslational = 0;
int moving_angle = 0;

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
    Serial1.begin(9600);
}

void loop()
{

    // Receive Data
    bno_angle = serialComm.Receive(RECEIVE_BNO);
    ball_angle = serialComm.Receive(RECEIVE_BALL_ANGLE);
    ball_distance = serialComm.Receive(RECEIVE_BALL_DISTANCE);
    goal_angle = serialComm.Receive(RECEIVE_GOAL_ANGLE);
    goal_distance = serialComm.Receive(RECEIVE_GOAL_DISTANCE);

    // PID & Motor Control
    double speed_w = pid_w.Calculate(target_angle, bno_angle);
    double speed_t_goal = pid_t_goal.Calculate(180, goal_angle);
    double speed_t_ball = pid_t_ball.Calculate(0, ball_distance);
    Serial.println(ball_distance);

    // Separate ball angle 180 and -180
    if (ball_angle < 180)
    {
        ball_angle_180 = -ball_angle;
    }
    else if (ball_angle > 180)
    {
        ball_angle_180 = 360 - ball_angle;
    }


    if (ball_angle == 0)
    {
        ball_found = false;
    }
    else
    {
        ball_found = true;
    }
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
    else if (goal_angle != 0)
    {
        // Goal
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
}
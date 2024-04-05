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

PID pid_w(1.05, 3.2, 0.2, 30);
// tune this
PID pid_t_ball(3, 0, 0, 255);
PID pid_t_goal(3, 0, 0, 255);

// Receive Data from ESP32
float bno_angle = 0;
float ball_angle = 0;
float ball_distance = 0;
float goal_angle = 0;
float goal_distance = 0;

// Data used for the control
float diff_angle = 0;
float target_goal_angle = 0;
bool ball_found = false;

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

// Sampling Time
double last_time = 0;
double current_time = 0;
double delta_time = 0;

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

    // Separate ball angle 180 and -180
    if (ball_angle == 0)
    {
    }
    else if (ball_angle == 10)
    {
        ball_angle = 0;
    }
    else if (ball_angle < 180)
    {
        ball_angle = -ball_angle;
    }
    else if (ball_angle > 180)
    {
        ball_angle = 360 - ball_angle;
    }

    // PID & Motor Control
    double speed_w = pid_w.Calculate(target_angle, bno_angle);

    Serial.print("BNO:");
    Serial.print(bno_angle);
    Serial.print(" ");
    Serial.print("Ball Angle:");
    Serial.print(ball_angle);
    Serial.print(" ");
    Serial.print("Goal Angle:");
    Serial.print(goal_angle);
    Serial.print(" ");

    if (ball_angle == 0)
    {
        ball_found = false;
    }else{
      ball_found = true; 
    }

    if (ball_found)
    {
        // Ball
        double speed_t = pid_t_ball.Calculate(180 - ball_angle, ball_angle);
        myMotors.MoveMotorsImu(ball_angle, speed_t, speed_w);
        Serial.println("ball found ");
        Serial.println(speed_t);

    }
    else
    {
        // Goal
        double speed_t = pid_t_goal.Calculate(180, goal_angle);
        if (goal_angle < (185 - goal_threshold) && ball_found == false)
        {
            myMotors.MoveMotorsImu(270, 180, speed_w * .75);
            Serial.println("LEFT GOAL");
        }
        else if (goal_angle > (185 + goal_threshold) && ball_found == false)
        {
            myMotors.MoveMotorsImu(90, 180, speed_w * .75);
            Serial.println("RIGHT GOAL");
        }
        else
        {
            myMotors.MoveMotorsImu(0, 0, speed_w);
            Serial.println("STOP");
        }
    }

    delay(20);
}

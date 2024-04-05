/*Main code for the goalkeeper robot*/
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
PID pid_t_ball(3, 0, 0, 255);
PID pid_t_goal(3, 0, 0, 255);

// Receive Data from ESP32
float bno_angle = 0;
float ball_angle = 0;
float ball_distance = 0;
float goal_angle = 0;
float goal_distance = 0;

// Data used for the control
bool ball_found = false;

// PID
double target_angle = 0; // frame

// Bang Bang / histéresis / On-off
float ball_threshold = 9;
float goal_threshold = 13.5;

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
    ball_angle = (ball_angle == 10 || ball_angle == 0) ? 0 : (ball_angle < 180) ? -ball_angle
                                                         : (ball_angle > 180)   ? 360 - ball_angle
                                                                                : ball_angle;

    // PID for angular speed
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

    // Ball found
    ball_found = (ball_angle != 0);

    // PID logic for translational speed depending on ball or field detection
    if (ball_found)
    {
        // Ball detected
        double speed_t = pid_t_ball.Calculate(180 - ball_angle, ball_angle);
        myMotors.MoveMotorsImu(ball_angle, speed_t, speed_w);
    }
    else
    {
        // Goal detected
        double speed_t = pid_t_goal.Calculate(180, goal_angle);
        if (!ball_found)
        {
            int direction = (goal_angle < (185 - goal_threshold)) ? 270 : (goal_angle > (185 + goal_threshold)) ? 90
                                                                                                                : 0;
            String message = (goal_angle < (185 - goal_threshold)) ? "LEFT GOAL" : (goal_angle > (185 + goal_threshold)) ? "RIGHT GOAL"
                                                                                                                         : "STOP";
            double speed = (goal_angle < (185 - goal_threshold) || goal_angle > (185 + goal_threshold)) ? speed_w * .75 : speed_w;

            myMotors.MoveMotorsImu(direction, 180, speed);
            Serial.println(message);
        }
    }

    delay(20);
}

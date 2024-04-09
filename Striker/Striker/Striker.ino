/*Main code for the goalkeeper robot*/
// Work in progress. Still defining objectives and requirements
//  The robot moves in its own axis to follow the ball and moves right or left to keep the ball in the center of the camera using PID controller for the camera tracking and Bang Bang for the traslation
#include <Arduino.h>
#include "constants.h"
#include "PID.h"
#include "serial.h"
#include "Motors.h"
#include <typeinfo>

/*
Robot logic for striker to score goal

1. Implement logic to find ball
2. If robot finds ball then ball_found = true
3. It is necessary to implement a sampling time to know when the robot saw the ball recently, so he knows he has te ball
4. If ball_found = true then robot moves to the ball implementing goal keeper logic
5. If ball_distance < 25 and ball_found = true then i have the ball and it is close to the robot
6. But if in the next sampling time i dont see ball, but i have seen it before close to me < 25 of distance then i have the ball
7. If i see the ball again in the next sampling time then the process repeats
8. If i dont see the ball in the next sampling time then i have to score the goal
9. Once robot has_ball = true then robot moves to the goal in the angle from the center of the goal
10. Move forward until goal_distance < 40
11. Use kicker to strike goal
12. Move back to the center of the field
13. Repeat the process
*/

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

// Data used for the control
bool ball_found = false;
double ponderated_angle = 0;

// PID
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
        // logica delatero
    }
}


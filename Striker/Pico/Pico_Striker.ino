#include <Arduino.h>
#include "constants.h"
#include "PID.h"
#include "Motors.h"
#include <typeinfo>

#define PIN_SERIAL1_TX (0u)
#define PIN_SERIAL1_RX (1u)

PID pid_w(.8, 0.001, 90, 200);
PID pid_t_ball(60, 1, 0, 320);
PID pid_t_goal(20, 1, 0, 255);

float ball_angle_180 = 0;
bool ball_found = false;
bool goal_blue_found = false;
double ponderated_angle = 0;
double target_angle = 0;
int goal_threshold = 13;

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

    if (Serial1.available() > 0)
    {
        //--------------------Receive data from sensors--------------------------//
        String data = Serial1.readStringUntil('\n');
        double values[5];
        int index = 0;
        char *ptr = strtok(const_cast<char *>(data.c_str()), ",");
        while (ptr != NULL && index < 5)
        {
            values[index++] = atoi(ptr);
            ptr = strtok(NULL, ",");
        }

        double bno_angle = values[0];
        double ball_distance = values[1];
        double ball_angle = values[2];
        double goal_angle = values[3];
        double distance_pixels = values[4];

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
            Serial.println(ball_found);

            //--------------------Implementation if ball found-----------------------------------//
            if (ball_found || distance_pixels > 100 || distance_pixels == 0)
            {
                Serial.println("pelota found");
                if (ball_angle_180 > -15 && ball_angle_180 < 15)
                {

                    myMotors.MoveMotorsImu(0, abs(speed_t_ball), speed_w);
                }
                else
                {
                    Serial.println("pel angle");
                    ball_angle = 360 - ball_angle;
                    double differential = ball_angle * 0.09;
                    ponderated_angle = ball_angle - differential;
                    ponderated_angle = ball_angle > 180 ? ball_angle - differential : ball_angle + differential;
                    myMotors.MoveMotorsImu(ponderated_angle, abs(speed_t_ball), speed_w);
                }
            }
            else
            {
                myMotors.MoveMotorsImu(180, 170, speed_w);
                delay(110);
            }
        }
    }
}

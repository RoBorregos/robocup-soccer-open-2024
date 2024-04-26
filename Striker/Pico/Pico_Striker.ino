// Work in progress. Still defining objectives and requirements
//  The robot moves in its own axis to follow the ball and moves right or left to keep the ball in the center of the camera using PID controller for the camera tracking and Bang Bang for the traslation
/*#include <Arduino.h>
#include "constants.h"
#include "PID.h"
#include "Motors.h"
#include <typeinfo>

#define PIN_SERIAL1_TX (0u)
#define PIN_SERIAL1_RX (1u)

PID pid_w(.8, 0.001, 90, 200);
// PID pid_w(1,0,0, 200);
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
        Serial.print("BNO ANGLE");
        Serial.println(bno_angle);
        double ball_distance = values[1];
        Serial.print("Ball Distance");
        Serial.println(ball_distance);
        double ball_angle = values[2];
        Serial.print("ball_angle");
        Serial.println(ball_angle);
        double goal_angle = values[3];
        Serial.print("goal_angle");
        Serial.println(goal_angle);
        double distance_pixels = values[4];
        Serial.print("distance_pixels");
        Serial.println(distance_pixels);
        /*
        Serial.print("Angle: ");
        Serial.println(bno_angle);
        Serial.print("Ball Distance ");
        Serial.println(ball_distance);
        Serial.print("Ball Angle: ");
        Serial.println(ball_angle);
        Serial.print("Goal Angle: ");
        Serial.println(goal_angle);
        Serial.print("Goal Distance: ");
        Serial.println(goal_distance);
        */

        //--------------------Receive data from sensors--------------------------//

        //--------------------PID controller for the robot--------------------------//
        /*double speed_w = pid_w.Calculate(target_angle, bno_angle);
        // double speed_t_goal = pid_t_goal.Calculate(180, goal_angle);
        // double speed_t_ball = pid_t_ball.Calculate(0, ball_distance);
        double speed_t_goal = 140;
        double speed_t_ball = 140;

        if (speed_w != 0)
        {
            // myMotors.MoveMotorsImu(0, 0, speed_w);

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
            // if(angle_line == -1){
            // Serial.println("hola");
            //
                if (ball_found || distance_pixels > 100 || distance_pixels == 0 )
                {
                  
                    // Ball
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
                else{
                  myMotors.MoveMotorsImu(180, 170, speed_w);
                  delay(110);
                  //myMotors.StopMotors();
                }
                //-------------------------Implementation to center robot in goal-------------------------------------//
                
                
            }
        
    }
}*/

// Work in progress. Still defining objectives and requirements
//  The robot moves in its own axis to follow the ball and moves right or left to keep the ball in the center of the camera using PID controller for the camera tracking and Bang Bang for the traslation
#include <Arduino.h>
#include "constants.h"
#include "PID.h"
#include "Motors.h"
#include <typeinfo>

#define PIN_SERIAL1_TX (0u)
#define PIN_SERIAL1_RX (1u)

PID pid_w(.8, 0.001, 90, 200);
// PID pid_w(1,0,0, 200);
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
        Serial.print("BNO ANGLE");
        Serial.println(bno_angle);
        double ball_distance = values[1];
        Serial.print("Ball Distance");
        Serial.println(ball_distance);
        double ball_angle = values[2];
        Serial.print("ball_angle");
        Serial.println(ball_angle);
        double goal_angle = values[3];
        Serial.print("goal_angle");
        Serial.println(goal_angle);
        double distance_pixels = values[4];
        Serial.print("distance_pixels");
        Serial.println(distance_pixels);
        /*
        Serial.print("Angle: ");
        Serial.println(bno_angle);
        Serial.print("Ball Distance ");
        Serial.println(ball_distance);
        Serial.print("Ball Angle: ");
        Serial.println(ball_angle);
        Serial.print("Goal Angle: ");
        Serial.println(goal_angle);
        Serial.print("Goal Distance: ");
        Serial.println(goal_distance);
        */

        //--------------------Receive data from sensors--------------------------//

        //--------------------PID controller for the robot--------------------------//
        double speed_w = pid_w.Calculate(target_angle, bno_angle);
        // double speed_t_goal = pid_t_goal.Calculate(180, goal_angle);
        // double speed_t_ball = pid_t_ball.Calculate(0, ball_distance);
        double speed_t_goal = 100;
        double speed_t_ball = 100;

        if (speed_w != 0)
        {
            // myMotors.MoveMotorsImu(0, 0, speed_w);

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
            // if(angle_line == -1){
            // Serial.println("hola");
            //
                if (ball_found)
                {
                  myMotors.MoveMotorsImu(ball_angle_180, abs(speed_t_ball), speed_w);
                  
                    // Ball
                    /*Serial.println("pelota found");
                    if (ball_angle_180 > -15 && ball_angle_180 < 15)
                    {

                        myMotors.MoveMotorsImu(0, abs(speed_t_ball), speed_w);
                    }
                    /*else
                    {
                        Serial.println("pel angle");
                        ball_angle = 360 - ball_angle;
                        double differential = ball_angle * 0.09;
                        ponderated_angle = ball_angle - differential;
                        ponderated_angle = ball_angle > 180 ? ball_angle - differential : ball_angle + differential;
                        myMotors.MoveMotorsImu(ponderated_angle, abs(speed_t_ball), speed_w);
                    }*/
                }
              
                //-------------------------Implementation to center robot in goal-------------------------------------//
                
                
            }
        
    }
}
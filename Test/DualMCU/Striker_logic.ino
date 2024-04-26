#include "Arduino.h"
#include "constants.h"
#include "PID.h"
#include "Motors.h"
#include <typeinfo>

#define PIN_SERIAL1_TX (0u)
#define PIN_SERIAL1_RX (1u)

//60
PID pid_w(0.3, 0.0016, 35, 200);
PID pid_t_ball(50, 1, 0, 320);
PID pid_t_goal(20, 1, 0, 255);

Motors myMotors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);


float ball_angle_180 = 0;
bool ball_found = false;
bool has_ball = false;
bool goal_found = false;
double ponderated_angle = 0;
double target_angle = 0;
double last_time_ball_seen = 0;
double time_threshold = 50;
double score_angle = 0; 

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
    double values[5]; 
    int index = 0;
    char* ptr = strtok(const_cast<char*>(data.c_str()), ",");
    while (ptr != NULL && index < 5) { 
      values[index++] = atoi(ptr);
      ptr = strtok(NULL, ",");
    }

    double bno_angle = values[0];
    double ball_distance = values[1];
    double ball_angle = values[2];
    double goal_angle = values[3];
    double goal_distance = values[4];

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


    double speed_w = pid_w.Calculate(target_angle, bno_angle);
    double speed_t_goal = pid_t_goal.Calculate(180, goal_angle);
    double speed_t_ball = pid_t_ball.Calculate(0, ball_distance);

//--------------------------------------- Separate coordinate plane ---------------------------------//

    if (ball_angle < 180)
    {
        ball_angle_180 = -ball_angle;
    }
    else if (ball_angle > 180)
    {
        ball_angle_180 = 360 - ball_angle;
    }

//--------------------------------------- Ball found logic ---------------------------------------//

    if (ball_angle == 0)
    {
        ball_found = false;
    }
    else
    {
        ball_found = true;
        last_time_ball_seen = millis();
        Serial.println("Ball found");
    }

//--------------------------------------- Goal found logic ---------------------------------------//

    if(goal_angle == 0)
    {
        goal_found = false;
    }
    else
    {
        goal_found = true;
        //Serial.println("GOAL found");
    }

//--------------------------------------- Has ball logic ----------------------------------------//

    if (ball_found && ball_distance < 20 && millis() - last_time_ball_seen < time_threshold && ball_angle_180 > -30 && ball_angle_180 < 30)
    {
        has_ball = true;
        Serial.println("HAS BAALLL");
    }
    else
    {
        has_ball = false;
    }

//--------------------------------------- Scoring goal logic -------------------------------------//

    if(has_ball && goal_found && goal_distance > 30)
    {
        // move towards goal
        myMotors.MoveMotorsImu(360 - goal_angle, abs(speed_t_ball), speed_w);
    }//peruana
    else if(has_ball && goal_found && goal_distance < 30)
    {
        // shoot
        myMotors.MoveMotorsImu(360 - goal_angle, abs(speed_t_ball), speed_w);
    }
    else if (has_ball && !goal_found)
    {
        // logica de no tener porteria
        myMotors.MoveMotorsImu(360 - goal_found, abs(speed_t_ball), speed_w);
    }
    else if (!has_ball && !goal_found)
    {
        // logica de no tener balon
        myMotors.MoveMotorsImu(360 - goal_angle, abs(speed_t_ball), speed_w);
    }

//--------------------------------------- Follow ball logic --------------------------------------------//

    if (ball_found)
    {
        score_angle = abs(goal_angle - ball_angle);
        if (ball_angle_180 > -15 && ball_angle_180 < 15 && !goal_found)
        {
            myMotors.MoveMotorsImu(0, abs(speed_t_ball), speed_w);
        }
        
        else if(score_angle > 15)
        {
            ball_angle = 360 - ball_angle;
            double differential = ball_angle * 0.15;
            ponderated_angle = ball_angle - differential;
            ponderated_angle = ball_angle > 180 ? ball_angle - differential : ball_angle + differential;
            myMotors.MoveMotorsImu(ponderated_angle, abs(speed_t_ball), speed_w);
        }
    }
    }
}


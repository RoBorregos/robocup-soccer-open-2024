#include <Arduino.h>
#include <Motors.h>
#include <typeinfo>
#include <serial.h>
#include <PID.h>
#include <constants.h>
#include <iostream>
#include "Bno.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#define PIN_SERIAL1_TX (0u)
#define PIN_SERIAL1_RX (1u)

//----------------Object Declaration----------------------//

SerialCommunication serialComm(Serial1);
Adafruit_ADS1115 ads_left;
Adafruit_ADS1115 ads_right;
Adafruit_ADS1115 ads_back;
BNO055 myBNO;

//----------------PID Constants----------------------//

PID pid_w(0.45, 0.0001, 80, 200);
PID pid_t_ball(60, 1, 0, 320);
PID pid_t_goal(20, 1, 0, 255);

//----------------Variables----------------------//

float bno_angle = 0;
float ball_angle = 0;
float ball_distance = 0;
float move_line_angle = 0;
float goal_angle = 0;
float ball_angle_180 = 0;
double last_time = 0;
double current_time = 0;
bool ball_found = false;
double ponderated_angle = 0;
double target_angle = 0;
int last_ads_value = 0;
int current_ads_value = 0;
double last_time = 0;
double current_time = 0;

//----------------Estructure for photo sensors----------------------//

struct PhotoValues
{
    int adc285;
    int adc276;
    int adc265;
    int adc254;
    int adc105;
    int adc96;
    int adc85;
    int adc74;
    int adc195;
    int adc186;
    int adc175;
    int adc164;
};

//----------------Arrays for photo sensors----------------------------------//

int angle_values[] = {285, 276, 265, 254, 105, 96, 85, 74, 195, 186, 175, 164};
int last_ads_values[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//-----------------------Motor declaration----------------------------------//

Motors myMotors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);

void setup()
{
    //-----------------------Initialization----------------------------------//

    myMotors.InitializeMotors();
    myBNO.InitializeBNO();

    //-----------------------ADS1115 Initialization----------------------------------//

    ads_left.begin(0x48);
    ads_left.setGain(GAIN_FOUR);
    ads_right.begin(0x49);
    ads_right.setGain(GAIN_FOUR);
    ads_back.begin(0x4A);
    ads_back.setGain(GAIN_FOUR);
    Serial.begin(230400);

    //-----------------------Serial Communication----------------------------------//

    while (!Serial && millis() < 10000UL)
        ;
    Serial.println("Started");
    Serial1.begin(230400);
}

void loop()
{
    //-----------------------Photo values read----------------------------------//

    PhotoValues photo_values = {
        // LEFT
        .adc285 = ads_left.readADC_SingleEnded(0),
        .adc276 = ads_left.readADC_SingleEnded(1),
        .adc265 = ads_left.readADC_SingleEnded(2),
        .adc254 = ads_left.readADC_SingleEnded(3),
        // RIGHT
        .adc105 = ads_right.readADC_SingleEnded(0),
        .adc96 = ads_right.readADC_SingleEnded(1),
        .adc85 = ads_right.readADC_SingleEnded(2),
        .adc74 = ads_right.readADC_SingleEnded(3),
        // BACK
        .adc195 = ads_back.readADC_SingleEnded(0),
        .adc186 = ads_back.readADC_SingleEnded(1),
        .adc175 = ads_back.readADC_SingleEnded(2),
        .adc164 = ads_back.readADC_SingleEnded(3)};

    //-----------------------Assign photo cases----------------------------------//

    for (int i = 0; i < 12; i++)
    {
        switch (i)
        {
        case 0:
            current_ads_value = photo_values.adc285;
            break;
        case 1:
            current_ads_value = photo_values.adc276;
            break;
        case 2:
            current_ads_value = photo_values.adc265;
            break;
        case 3:
            current_ads_value = photo_values.adc254;
            break;
        case 4:
            current_ads_value = photo_values.adc105;
            break;
        case 5:
            current_ads_value = photo_values.adc96;
            break;
        case 6:
            current_ads_value = photo_values.adc85;
            break;
        case 7:
            current_ads_value = photo_values.adc74;
            break;
        case 8:
            current_ads_value = photo_values.adc195;
            break;
        case 9:
            current_ads_value = photo_values.adc186;
            break;
        case 10:
            current_ads_value = photo_values.adc175;
            break;
        case 11:
            current_ads_value = photo_values.adc164;
            break;
        }

        //-----------------------Move line angle logic for white lines----------------------------------//

        if (current_ads_value > last_ads_values[i] + 410)
        {
            move_line_angle += angle_values[i];
            avg++; /*
             if(i >= 0 && i <= 3){
               fright++;
             }
             if(i >= 8 && i <= 11){
               fleft++;
             }*/
        }
        last_ads_values[i] = current_ads_value;
    }
    /*
        if(fright > 0 && fleft > 0){
            move_line_angle = 180;
        }   else if (avg > 0){
            move_line_angle = (move_line_angle / avg) + 180;
        }else {
            move_line_angle = -1;
        }*/

    //-----------------------BNO values read----------------------------------//

    myBNO.getBNOData();
    bno_angle = myBNO.getYaw();

    //-----------------------Read data from camera----------------------------------//

    if (Serial.available())
    {
        ball_distance = Serial.parseInt();
        ball_angle = Serial.parseInt();
        goal_angle = Serial.parseInt();
    }

    //-----------------------PID calculation----------------------------------//

    double speed_w = pid_w.Calculate(target_angle, bno_angle);
    double speed_t_goal = pid_t_goal.Calculate(180, goal_angle);
    double speed_t_ball = pid_t_ball.Calculate(0, ball_distance);

    //-----------------------Movement logic----------------------------------//

    if (move_line_angle == -1)
    {

        //-----------------------Separate ball angle 180 and -180----------------------------------//

        if (ball_angle < 180)
        {
            ball_angle_180 = -ball_angle;
        }
        else if (ball_angle > 180)
        {
            ball_angle_180 = 360 - ball_angle;
        }
        //-----------------------Ball found logic----------------------------------//

        if (ball_angle == 0)
        {
            ball_found = false;
        }
        else
        {
            ball_found = true;
        }

        //-----------------------Ball found movement----------------------------------//

        if (ball_found)
        {

            //--------------------Position the robot in front of the ball---------------//

            if (ball_angle_180 > -15 && ball_angle_180 < 15)
            {
                myMotors.MoveMotorsImu(0, abs(speed_t_ball), speed_w);
            }

            //-------------------Position into the ponderated angle---------------------//

            else
            {
                ball_angle = 360 - ball_angle;
                double differential = ball_angle * 0.15;
                ponderated_angle = ball_angle - differential;
                ponderated_angle = ball_angle > 180 ? ball_angle - differential : ball_angle + differential;
                myMotors.MoveMotorsImu(ponderated_angle, abs(speed_t_ball), speed_w);
            }
        }

        //--------------Ball not found movement, align robot to center of goal---------------//

        else if (goal_angle != 0)
        {
            // Goal
            if (goal_angle < (185 - goal_threshold) && ball_found == false)
            {
                myMotors.MoveMotorsImu(270, abs(speed_t_goal), speed_w);
                // Serial.println("LEFT GOAL");
            }
            else if (goal_angle > (185 + goal_threshold) && ball_found == false)
            {
                myMotors.MoveMotorsImu(90, abs(speed_t_goal), speed_w);
                // Serial.println("RIGHT GOAL");
            }
            else
            {
                myMotors.MoveMotorsImu(0, 0, speed_w);
                // Serial.println("STOP");
            }
        }
    }
    //-------------------------------Move line movement------------------------------+----------//

    else
    {
        if (abs(move_line_angle - ponderated_angle) > 150 && abs(move_line_angle - ponderated_angle) < 210)
        {
            last_time = millis();
            while (millis() - last_time < 300)
            {
                millis() - last_time;
                myMotors.MoveMotorsImu(move_line_angle, 255, speed_w);
                // bno_angle = serialComm.Receive(RECEIVE_BNO);
                speed_w = pid_w.Calculate(0, bno_angle);
            }
            myMotors.MoveMotorsImu(0, 0, speed_w);
        }
        else
        {
            last_time = millis();
            while (millis() - last_time < 190)
            {
                millis() - last_time;
                myMotors.MoveMotorsImu(move_line_angle, 180, speed_w);
                // bno_angle = serialComm.Receive(RECEIVE_BNO);
                speed_w = pid_w.Calculate(0, bno_angle);
            }
        }
    }
}
/Main code for the goalkeeper robot/
// Work in progress. Still defining objectives and requirements
//  The robot moves in its own axis to follow the ball and moves right or left to keep the ball in the center of the camera using PID controller for the camera tracking and Bang Bang for the traslation
#include <Arduino.h>
#include <Motors.h>
#include <typeinfo>
#include <serial.h>
#include <PID.h>
#include <constants.h>

SerialCommunication serialComm(Serial1);
#define PIN_SERIAL1_TX (0u)
#define PIN_SERIAL1_RX (1u)

PID pid_w(0.45,0.0001,80, 200);
// tune this
PID pid_t_ball(60, 1, 0, 320);
PID pid_t_goal(20, 1, 0, 255);

// Receive Data from ESP32
float bno_angle = 0;
float ball_angle = 0;
float ball_distance = 0;
float moveLineAngle = 0;
float goal_angle = 0;
float goal_distance = 0;
float ball_angle_180 = 0;

double last_time = 0;
double current_time = 0; 

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
    Serial.begin(9600);
    while (!Serial && millis() < 10000UL);
    Serial.println("Started");
    Serial1.begin(9600);
}

void loop()
{
    int testtime = millis();
    // Receive Data
    bno_angle = serialComm.Receive(RECEIVE_BNO);
    Serial.print("BNO: ");
    Serial.println(bno_angle);
    Serial.print("BNO LASTED: ");
    Serial.println(millis() - testtime);
    testtime = millis();
    ball_angle = serialComm.Receive(RECEIVE_BALL_ANGLE);
    Serial.print("BALL ANGLE: ");
    Serial.println(ball_angle);
    Serial.print("BALL ANGLE LASTED: ");
    Serial.println(millis() - testtime);
    testtime = millis();
    ball_distance = serialComm.Receive(RECEIVE_BALL_DISTANCE);
    Serial.print("BALL DISTANCE: ");
    Serial.println(ball_distance);
    Serial.print("BALL DISTANCE LASTED: ");
    Serial.println(millis() - testtime);
    testtime = millis();
    goal_angle = serialComm.Receive(RECEIVE_GOAL_ANGLE);
    Serial.print("GOAL ANGLE: ");
    Serial.println(ball_angle);
    //goal_distance = serialComm.Receive(RECEIVE_GOAL_DISTANCE);
    //moveLineAngle= serialComm.Receive(RECEIVE_LINE_ANGLE);
    Serial.print("moveLineAngle: ");
    Serial.println(moveLineAngle);
    Serial.print("WHITE LINE LASTED: ");
    Serial.println(millis() - testtime);
    testtime = millis();

    // PID & Motor Control
    double speed_w = pid_w.Calculate(target_angle, bno_angle);
    double speed_t_goal = pid_t_goal.Calculate(180, goal_angle);
    double speed_t_ball = pid_t_ball.Calculate(0, ball_distance);
    //Serial.println(ball_distance);
    if(moveLineAngle == -1){
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
            //Serial.println("LEFT GOAL");
        }
        else if (goal_angle > (185 + goal_threshold) && ball_found == false)
        {
            myMotors.MoveMotorsImu(90, abs(speed_t_goal), speed_w);
            //Serial.println("RIGHT GOAL");
        }
        else
        {
            myMotors.MoveMotorsImu(0, 0, speed_w);
            //Serial.println("STOP");
        }
        
    }
    }
    else{
      if (abs(moveLineAngle-ponderated_angle) > 150 && abs(moveLineAngle-ponderated_angle) < 210){
          last_time = millis();
          while(millis() - last_time < 300){
            millis() - last_time;
            myMotors.MoveMotorsImu(moveLineAngle, 255, speed_w);
            bno_angle = serialComm.Receive(RECEIVE_BNO);
            speed_w = pid_w.Calculate(0,bno_angle); 
          }
          myMotors.MoveMotorsImu(0, 0, speed_w);
        } else {
          last_time = millis();
          while(millis() - last_time < 190){
            millis() - last_time;
            myMotors.MoveMotorsImu(moveLineAngle, 180, speed_w);
            bno_angle = serialComm.Receive(RECEIVE_BNO);
            speed_w = pid_w.Calculate(0,bno_angle);
          }
        }
    } 
    //Serial.print("EVERYTHING ELSE LASTED ");
    //Serial.println(millis() - testtime);
    //testtime = millis();
}
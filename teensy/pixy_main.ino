#include <iostream>
#include <Wire.h>
#include "Bno.h"
#include <cmath>
#include "PID.h"
#include <Pixy2SPI_SS.h>
#include "Motors.h"
#include <Servo.h>

/*
Pixy camera resolution: 316 x 208
pixy.ccc.blocks[i].m_x The x location of the center of the detected object (0 to 316)
pixy.ccc.blocks[i].m_y The y location of the center of the detected object (0 to 208)
*/

const int esc_pin = 6;
double goal_angle_opposite = 0;
const int min_speed = 1000;
const int mid_speed = 1500;
const int max_speed = 2000;
const int delay_time = 3000;
float bno_angle = 0;
bool ball_seen_openmv = false;
double target_angle = 0;
float ball_angle_180 = 0;
float distance_pixels = 0;
float distance_pixels_protect = 0;
float ball_angle = 0;
float ball_distance = 0;
float goal_angle = 0;
bool ball_found = false;
double ponderated_angle = 0;
double angle_degrees = 0;
float goal_distance = 0;
double last_time = 0;
double current_time = 0;
int goal_threshold = 15;
const int FRAME_HEIGHT = 104;
const int FRAME_WIDTH = 158;
const int FRAME_ROBOT = 20;
const int FRAME_CIRCLE = 150;

Pixy2SPI_SS pixy;
BNO055 my_bno;
Servo esc;

PID pid_w(0.6, 0.01, 6, 200);
Motors myMotors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);

void setup()
{
  esc.attach(esc_pin);
  Serial1.begin(115200);
  Serial.begin(9500);
  esc.writeMicroseconds(min_speed);
  pixy.init();
  my_bno.InitializeBNO();
  analogReadResolution(12);
  delay(delay_time);
}

double radiansToDegrees(double radians)
{
  return radians * (180.0 / M_PI);
}

void loop()
{

  // ----------------- Gather data from OpenMV camera via UART ----------------- //

  int photoValue = analogRead(A2);
  int photoValue1 = analogRead(A7);
  int photoValue2 = analogRead(A3);
  int photoValue3 = analogRead(A8);
  int photoValue4 = analogRead(A9);
  my_bno.GetBNOData();
  bno_angle = my_bno.GetYaw();
  if (Serial1.available())
  {
    String camString = Serial1.readStringUntil('\n');
    ball_distance = camString.toFloat();
    ball_angle = camString.substring(camString.indexOf(' ') + 1).toFloat();
    goal_angle = camString.substring(camString.indexOf(' ', camString.indexOf(' ') + 1) + 1, camString.lastIndexOf(' ')).toFloat();
    distance_pixels = camString.substring(camString.lastIndexOf(' ') + 1).toFloat();
    ball_seen_openmv = (ball_distance != 0 || ball_angle != 0);
    Serial.println(goal_angle);
  }

  // ----------------- Gather data from Pixy2 camera via SPI ----------------- //

  int i;
  pixy.ccc.getBlocks();
  int pixy_blocks = pixy.ccc.numBlocks;
  bool ball_seen_pixy = false;

  if (pixy.ccc.numBlocks)
  {
    for (i = 0; i < pixy.ccc.numBlocks; i++)
    {
      double cx = pixy.ccc.blocks[i].m_x;
      double cy = pixy.ccc.blocks[i].m_y;
      double relative_cx = cx - 158;
      double relative_cy = cy - 208;
      double magnitude_distance = sqrt(relative_cx * relative_cx + relative_cy * relative_cy);
      double total_distance = 1.2415 * magnitude_distance - 5.2805;

      double angle_radians = atan2(relative_cy, relative_cx);
      angle_degrees = radiansToDegrees(angle_radians);

      angle_degrees += 95;
      if (angle_degrees < 0)
      {
        angle_degrees += 360;
      }
      if (angle_degrees >= 360)
      {
        angle_degrees -= 360;
      }

      Serial.print("ANGLE PIXY: ");
      Serial.println(angle_degrees);

      if (pixy.ccc.blocks[i].m_signature == 1)
      {
        ball_seen_pixy = true;
        /* ball_distance = total_distance;
         ball_angle = 360 - angle_degrees;*/
      }
      else
      {
        ball_seen_pixy = false;
      }
    }
  }

  //----------------------- Calculate PIDS-----------------------------//

  double speed_w = pid_w.Calculate(target_angle, bno_angle);
  double speed_t_goal = 150;
  double speed_t_ball = 150;

  if (speed_w != 0)
  {
    if (photoValue > 2400 || photoValue1 > 2200)
    {
      myMotors.MoveMotorsImu(90, 200, speed_w);
      delay(300);
      Serial.println("Derecha");
    }
    else if (photoValue2 > 2500)
    {
      myMotors.MoveMotorsImu(270, 200, speed_w);
      delay(300);
      Serial.println("IZQUIERDA");
    }
    else if (photoValue3 > 2400 || photoValue4 > 2000 ) {
      myMotors.MoveMotorsImu(180, 240, speed_w);
      delay(400);
      Serial.println("ATRAS");
    }
    else
    {
      if (ball_angle < 180)
      {
        ball_angle_180 = -ball_angle;
      }
      else if (ball_angle > 180)
      {
        ball_angle_180 = 360 - ball_angle;
      }
      ball_angle_180 = ball_angle_180 * (-1);

      //------------------Both camera detection cases ------------------//
      if (ball_seen_pixy && ball_seen_openmv)
      {
        myMotors.MoveMotorsImu(angle_degrees, speed_t_ball, speed_w);
        Serial.println("Both cameras see the ball");
        esc.writeMicroseconds(mid_speed);
      }

      /*if (ball_seen_pixy && distance_pixels < 85)
      {

        if (goal_angle > 180)
        {
          Serial.print("DISTANCE ");
          Serial.println(distance_pixels);
          target_angle = -150;
          speed_w = pid_w.Calculate(target_angle, bno_angle);
          if (speed_w != 0)
          {
            Serial.print("speed: ");
            Serial.println(speed_w);
            myMotors.MoveMotorsImu(0, 0, speed_w);
          }
        }
        else
        {
          target_angle = 150;
          speed_w = pid_w.Calculate(target_angle, bno_angle);
          myMotors.MoveMotorsImu(0, 0, speed_w);
        }

        // delay(3000);
        // myMotors.StopMotors();

        // target_angle = 0;

        // Serial.pri ntln(goal_angle_opposite);
        Serial.println("Only pixy sees the ball and is in front of the robot");
        esc.writeMicroseconds(mid_speed);
      }*/
      else if (ball_seen_pixy && !ball_seen_openmv)
      {
        myMotors.MoveMotorsImu(angle_degrees, speed_t_ball, speed_w);
        Serial.println("Only pixy sees the ball");
        esc.writeMicroseconds(mid_speed);
      }
      else if (!ball_seen_pixy && ball_seen_openmv)
      {
        double differential = ball_angle_180 * 0.15;
        ponderated_angle = ball_angle + differential;
        myMotors.MoveMotorsImu(ponderated_angle, abs(speed_t_ball), speed_w);
        Serial.println("Only OpenMV sees the ball");
      }
      else if (goal_angle > 0)
      {
        if (goal_angle < (185 - goal_threshold) && ball_found == false)
        {
          myMotors.MoveMotorsImu(90, abs(speed_t_goal), speed_w);
        }
        else if (goal_angle > (185 + goal_threshold) && ball_found == false)
        {
          myMotors.MoveMotorsImu(270, abs(speed_t_goal), speed_w);
        }
        else
        {
          myMotors.MoveMotorsImu(ball_angle, 0, speed_w);
        }
      }
      else
      {
        Serial.println("No camera sees the ball");
        myMotors.StopMotors();
      }
    }
  }
}

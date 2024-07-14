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

double goal_angle_opposite = 0;
unsigned long start_millis;
unsigned long current_millis;
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
double last_ball_angle = 0;
int goal_threshold = 15;
const int FRAME_HEIGHT = 104;
const int FRAME_WIDTH = 158;
const int FRAME_ROBOT = 20;
const int FRAME_CIRCLE = 150;
const unsigned long period = 100;
const int delay_time = 3000;
const int min_speed = 1000;
const int mid_speed = 1500;
const int max_speed = 2000;
const int esc_pin = 6;
bool goal_seen = false;
int photoValue;
int photoValue1;
int photoValue2;
 int analogPin1;
 int analogPin2;
 int photo_value5;
 int photo_value6;
 int photo_value7;

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
  Serial.begin(9600);
  esc.writeMicroseconds(min_speed);
  pixy.init();
  my_bno.InitializeBNO();
  analogReadResolution(12);
  delay(delay_time);
  start_millis = millis();
}

double radiansToDegrees(double radians)
{
  return radians * (180.0 / M_PI);
}

void loop()
{
  // ----------------- Gather data from OpenMV camera via UART ----------------- //
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
  }

  if (goal_angle > 0)
  {
    goal_seen = true;
  }
  else
  {
    goal_seen = false;
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

      if (pixy.ccc.blocks[i].m_signature == 1)
      {
        ball_seen_pixy = true;
      }
    }
  }

  //----------------------- Calculate PIDS-----------------------------//

  double speed_w = pid_w.Calculate(target_angle, bno_angle);
  double speed_t_goal = 150;
  double speed_t_ball = 150;

  photoValue = analogRead(A2);
  photoValue1 = analogRead(A7);
  photoValue2 = analogRead(A3);
  analogPin1 = analogRead(A8);
  analogPin2 = analogRead(A9);
  photo_value5 = analogRead(A15);
  photo_value6 = analogRead(A17);
  photo_value7 = analogRead(A6);

  Serial.print("PHOTO VALUE 0: ");
  Serial.println(photoValue);
  Serial.print("PHOTO VALUE 1: ");
  Serial.println(photoValue1);
  Serial.print("PHOTO VALUE 2: ");
  Serial.println(photoValue2);
  Serial.print("PHOTO VALUE 3: ");
  Serial.println(analogPin1);
  Serial.print("PHOTO VALUE 4: ");
  Serial.println(analogPin2);
  Serial.print("PHOTO VALUE 5: ");
  Serial.println(photo_value5);
  Serial.print("PHOTO VALUE 6: ");
  Serial.println(photo_value6);
  Serial.print("PHOTO VALUE 7: ");
  Serial.println(photo_value7);


  //------------------ Angle normalization ------------------//
  if (ball_angle < 180)
  {
    ball_angle_180 = -ball_angle;
  }
  else if (ball_angle > 180)
  {
    ball_angle_180 = 360 - ball_angle;
  }
  ball_angle_180 = ball_angle_180 * (-1);

  // ----------------------------- Follow ball and avoid lines ------------------------//
  if (photo_value5 > 3500 || photo_value6 > 2300 || photo_value7 > 3000)
  {
    myMotors.MoveMotorsImu(0, 200, speed_w);
    delay(300);
    Serial.println("Adelante");
  }
  else if (photoValue > 2300 || photoValue1 > 2300)
  {
    myMotors.MoveMotorsImu(90, 200, speed_w);
    delay(300);
    Serial.println("DERECHA");
  }
  else if (analogPin1 > 2300 || analogPin2 > 1700)
  {
    myMotors.MoveMotorsImu(180, 200, speed_w);
    delay(300);
    Serial.println("ATRAS");
  }
  else if (photoValue2 > 2500)
  {
    myMotors.MoveMotorsImu(270, 200, speed_w);
    delay(300);
    Serial.println("Izquierda");
  }
  else if (ball_seen_pixy)
  {
    Serial.println("PIXY CAM");
    myMotors.MoveMotorsImu(angle_degrees, speed_t_ball, speed_w);
    Serial.println(angle_degrees);
  }
  else if (ball_seen_openmv)
  {
    Serial.println("OPENMV CAM");
    myMotors.MoveMotorsImu(ball_angle, speed_t_ball, speed_w);
    Serial.println(ball_angle);
  }
  else
  {
    // ------------------ Center robot in goal --------------//
    if (goal_angle > 0)
    {
      if (goal_angle < (185 - goal_threshold) && ball_found == false)
      {
        Serial.println("CENTER");
        myMotors.MoveMotorsImu(90, abs(speed_t_goal), speed_w);
      }
      else if (goal_angle > (185 + goal_threshold) && ball_found == false)
      {
        Serial.println("CENTER");
        myMotors.MoveMotorsImu(270, abs(speed_t_goal), speed_w);
      }
      //--------------- Move to goal angle to protect ---------//
      else
      {
        myMotors.MoveMotorsImu(goal_angle, 120, speed_w);
        Serial.println("NO DETECTA NADA, MOVER ATRAS");
      }

      //------------------------- Move depending on the photo transistors detected ------------------//
      if (photo_value5 > 3500 || photo_value6 > 2300 || photo_value7 > 3000)
      {
        myMotors.MoveMotorsImu(0, 200, speed_w);
        delay(300);
        Serial.println("Adelante");
      }
      else if (photoValue > 2300 || photoValue1 > 2300)
      {
        myMotors.MoveMotorsImu(90, 200, speed_w);
        delay(300);
        Serial.println("DERECHA");
      }
      else if (analogPin1 > 2300 || analogPin2 > 1700)
      {
        myMotors.MoveMotorsImu(180, 200, speed_w);
        delay(300);
        Serial.println("ATRAS");
      }
      else if (photoValue2 > 2500)
      {
        myMotors.MoveMotorsImu(270, 200, speed_w);
        delay(300);
        Serial.println("Izquierda");
      }
    }
  }
}
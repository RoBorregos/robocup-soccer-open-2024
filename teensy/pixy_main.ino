#include <iostream>
#include <Wire.h>
#include "Bno.h"
#include <cmath>
#include "PID.h"
#include <Pixy2SPI_SS.h>
#include "Motors.h"

/*
Pixy camera resolution: 316 x 208
pixy.ccc.blocks[i].m_x The x location of the center of the detected object (0 to 316)
pixy.ccc.blocks[i].m_y The y location of the center of the detected object (0 to 208)
*/

/*
Pixy cam con la que veo de default puedo ver la distancia y el angulo de todas mis cosas, hay un rango en donde ya no vi mi pelota. Entonces se debe
de tener tres condiciones de cuando no veo la pelota checar si la puedo ver en la openmv, de cuando la veo en las dos y de cuando solo la veo en la pixy. En
base a eso puedo acercarme a la pelota en su ball angle y cuando ya la tenga enfrente (osea que la vea con la camara frontal openmv a cierta distancia y angulo)
 y si esta ahi prendo dribbler y avanzo hacia el angulo de la pelota de acuerdo a mi openmv en caso de no ver la pelota
en ningun lado, me dirigo al angulo donde la vi por ultima vez
*/
#include <Servo.h>

// Create a Servo object to control the ESC
Servo esc;

// Define the pin for the ESC signal wire
const int escPin = 6;

// Define speed levels
const int minSpeed = 1000; // Minimum speed (1000 microseconds)
const int midSpeed = 1500; // Mid speed (1500 microseconds)
const int maxSpeed = 2000; // Maximum speed (2000 microseconds)
const int delayTime = 3000; // Delay time in milliseconds
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
float goal_distance = 0;
double last_time = 0;
double current_time = 0;
float angle = 0;
const int FRAME_HEIGHT = 104;
const int FRAME_WIDTH = 158;
const int FRAME_ROBOT = 20;
int goal_threshold = 15;
const int FRAME_CIRCLE = 150;

Pixy2SPI_SS pixy;
BNO055 my_bno;
PID pid_w(0.6, 0.01, 6, 200);
Motors myMotors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);

void setup()
{
  esc.attach(escPin);
  Serial1.begin(115200);
  Serial.begin(9500);
  esc.writeMicroseconds(minSpeed);
  pixy.init();
  my_bno.InitializeBNO();
  delay(3000);
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
  /*Serial.print(ball_distance);
  Serial.print(",");
  Serial.print(ball_angle);
  Serial.println();*/

  // ----------------- Gather data from Pixy2 camera via SPI ----------------- //

  int i;
  pixy.ccc.getBlocks();
  int pixy_blocks = pixy.ccc.numBlocks;
  bool ball_seen_pixy = false;

  if (pixy.ccc.numBlocks)
  {
    //Serial.println(pixy.ccc.numBlocks);
    for (i = 0; i < pixy.ccc.numBlocks; i++)
    {
      double cx = pixy.ccc.blocks[i].m_x;
      double cy = pixy.ccc.blocks[i].m_y;
      double relative_cx = cx - FRAME_WIDTH;
      double relative_cy = cy - FRAME_HEIGHT;
      double magnitude_distance = sqrt(relative_cx * relative_cx + relative_cy * relative_cy);
      double total_distance = 1.2415 * magnitude_distance - 5.2805;

      double angle_radians = atan2(relative_cy, relative_cx);
      double angle_degrees = radiansToDegrees(angle_radians);

      angle_degrees += 70;
      if (angle_degrees < 0)
      {
        angle_degrees += 360;
      }
      if (angle_degrees >= 360)
      {
        angle_degrees -= 360;
      }

      /*Serial.print("Block ");
      Serial.print(i);
      Serial.print(": Distance = ");
      Serial.print(total_distance);
      Serial.print(" cm, Angle = ");
      Serial.print(angle_degrees);
      Serial.println(" degrees");*/

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
  //Serial.println(ball_seen_pixy);

  double speed_w = pid_w.Calculate(target_angle, bno_angle);
  double speed_t_goal = 150;
  double speed_t_ball = 150;
  if (speed_w != 0)
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
      myMotors.MoveMotorsImu(ball_angle, speed_t_ball, speed_w);
      Serial.println("Both cameras see the ball");
      Serial.print("BALL ANGLE: ");
      Serial.println(ball_angle);
      //esc.writeMicroseconds(midSpeed);
    }
    else if (ball_seen_pixy && !ball_seen_openmv)
    {
       myMotors.MoveMotorsImu(0, speed_t_ball, speed_w);
      
      Serial.println("Only pixy sees the ball");
    }
    else if (!ball_seen_pixy && ball_seen_openmv)
    {
      Serial.println("PELOTA ANGLE");

      double differential = ball_angle_180 * 0.15;
      ponderated_angle = ball_angle + differential;
      Serial.print("PONDERATED: ");
      Serial.println(ponderated_angle);
      myMotors.MoveMotorsImu(ponderated_angle, abs(speed_t_ball), speed_w);
      //myMotors.MoveMotorsImu(ball_angle, abs(speed_t_ball), speed_w);
      Serial.print("BALL ANGLE: ");
      Serial.println(ball_angle);
      
      //esc.writeMicroseconds(midSpeed);
    }else if (goal_angle > 0)
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
      Serial.println("No camera sees the ball, move to last known position");
      myMotors.StopMotors();
    }
  }
}

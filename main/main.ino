#include <iostream>
#include <Wire.h>
#include "Bno.h"
#include <cmath>
#include <Pixy2SPI_SS.h>

/*
Pixy camera resolution: 316 x 208
pixy.ccc.blocks[i].m_x The x location of the center of the detected object (0 to 316)
pixy.ccc.blocks[i].m_y The y location of the center of the detected object (0 to 208)
*/

/*
So i have my pixy as my main cam which 
*/

float bno_angle = 0;
float distance_pixels = 0;
float distance_pixels_protect = 0;
float ball_angle = 0;
float ball_distance = 0;
float goal_angle = 0;
float goal_distance = 0;
double last_time = 0;
double current_time = 0;
float angle = 0;
const int FRAME_HEIGHT = 104;
const int FRAME_WIDTH = 158;

Pixy2SPI_SS pixy;
BNO055 my_bno;

void setup()
{
  Serial1.begin(115200);
  Serial.begin(9500);
  pixy.init();
  my_bno.InitializeBNO();
}

double radiansToDegrees(double radians)
{
  return radians * (180.0 / M_PI);
}

void loop()
{

  // ----------------- Gather data from OpenMV camera via UART ----------------- //
   my_bno.GetBNOData();
   angle = my_bno.GetYaw();
   Serial.println(angle);
  if (Serial1.available())
  {
    String camString = Serial1.readStringUntil('\n');
    ball_distance = camString.toFloat();
    ball_angle = camString.substring(camString.indexOf(' ') + 1).toFloat();
    goal_angle = camString.substring(camString.indexOf(' ', camString.indexOf(' ') + 1) + 1, camString.lastIndexOf(' ')).toFloat();
    distance_pixels = camString.substring(camString.lastIndexOf(' ') + 1).toFloat();
    distance_pixels_protect = camString.substring(camString.indexOf(' ', camString.lastIndexOf(' ') + 1)).toFloat();
  }
  String angleString = String(angle);
  String ballDistance = String(ball_distance);
  String ballAngle = String(ball_angle);
  String goalAngle = String(goal_angle);
  String distancePixels = String(distance_pixels);
  Serial.print(angleString);
  Serial.print(",");
  Serial.print(ballDistance);
  Serial.print(",");
  Serial.print(ballAngle);
  Serial.print(",");
  Serial.print(goalAngle);
  Serial.print(",");
  Serial.print(distancePixels);
  Serial.println();

  // ----------------- Gather data from Pixy2 camera via SPI ----------------- //

  int i;
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks)
  {
    Serial.println(pixy.ccc.numBlocks);
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
 
      angle_degrees += 8;
      if (angle_degrees < 0)
      {
        angle_degrees += 360;
      }
      if (angle_degrees >= 360)
      {
        angle_degrees -= 360;
      }

      Serial.print("Block ");
      Serial.print(i);
      Serial.print(": Distance = ");
      Serial.print(total_distance);
      Serial.print(" cm, Angle = ");
      Serial.print(angle_degrees);
      Serial.println(" degrees");

    }
  }
}

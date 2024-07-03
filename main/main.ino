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

void setup()
{
  Serial1.begin(115200);
  Serial.begin(9500);
  pixy.init();
}

double radiansToDegrees(double radians)
{
  return radians * (180.0 / M_PI);
}

void loop()
{

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

      Serial.print("Total distance: ");
      Serial.println(total_distance);
      double angle_radians = atan2(relative_cy, relative_cx);
      double angle_degrees = radiansToDegrees(angle_radians);

      Serial.print("Angle: ");
      Serial.println(angle_degrees);

      angle_degrees += 8;
      if (angle_degrees < 0)
      {
        angle_degrees += 360;
      }
      if (angle_degrees >= 360)
      {
        angle_degrees -= 360;
      }
    }
  }
}


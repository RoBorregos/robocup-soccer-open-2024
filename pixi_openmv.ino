#include <iostream>
#include <Wire.h>
#include "prueba/Bno.h"
#include <Pixy2SPI_SS.h>

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

Pixy2SPI_SS pixy;
Bno myBNO;

void setup()
{
  Serial1.begin(115200);
  Serial.begin(9500);
  pixy.init();
  myBNO.InitializeBNO();
}

void loop()
{

  // ----------------- Gather data from OpenMV camera via UART ----------------- //
  myBNO.getBNOData();
  angle = myBNO.getYaw();
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
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    for (i = 0; i < pixy.ccc.numBlocks; i++)
    {
      Serial.print("  block ");
      Serial.print(i);
      Serial.print(": ");
      pixy.ccc.blocks[i].print();
    }
  }
}
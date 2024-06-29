/*#include <iostream>
#include "Bno.h"
#include <Wire.h>

float bno_angle = 0;
float distance_pixels = 0; 
float ball_angle = 0;
float ball_distance = 0;
float goal_angle = 0;
float goal_distance = 0;
double last_time = 0;
double current_time = 0;


#define RXD2 16
#define TXD2 17

BNO055 myBNO;

float angle = 0;

void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
    myBNO.InitializeBNO();
}

void loop()
{
    myBNO.getBNOData();
    angle = myBNO.getYaw();
    if (Serial.available())
    {
        String camString = Serial.readStringUntil('\n');
        ball_distance = camString.toFloat();
        ball_angle = camString.substring(camString.indexOf(' ') + 1).toFloat();
        goal_angle = camString.substring(camString.indexOf(' ', camString.indexOf(' ') + 1) + 1, camString.lastIndexOf(' ')).toFloat();
        distance_pixels = camString.substring(camString.lastIndexOf(' ') + 1).toFloat();
    }
    String angleString = String(angle);
    String ballDistance = String(ball_distance);
    String ballAngle = String(ball_angle);
    String goalAngle = String(goal_angle);
    //String goalDistance = String(goal_distance);
    String distancePixels = String(distance_pixels);
    Serial2.print(angleString);
    Serial2.print(",");
    Serial2.print(ballDistance);
    Serial2.print(",");
    Serial2.print(ballAngle);
    Serial2.print(",");
    Serial2.print(goalAngle);
    Serial2.print(",");
    /*Serial2.print(goalDistance);
    Serial2.print(",");*/
    //Serial2.print(distancePixels);
    //Serial2.println();
//}

#include <iostream>
#include "Bno.h"
#include <Wire.h>

float bno_angle = 0;
float distance_pixels = 0; 
float distance_pixels_protect = 0; 
float ball_angle = 0;
float ball_distance = 0;
float goal_angle = 0;
float goal_distance = 0;
double last_time = 0;
double current_time = 0;


#define RXD2 16
#define TXD2 17

BNO055 myBNO;

float angle = 0;

void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
    myBNO.InitializeBNO();
}

void loop()
{
    myBNO.GetBNOData();
    angle = myBNO.GetYaw();
    if (Serial.available())
    {
        String camString = Serial.readStringUntil('\n');
        ball_distance = camString.toFloat();
        ball_angle = camString.substring(camString.indexOf(' ') + 1).toFloat();
        goal_angle = camString.substring(camString.indexOf(' ', camString.indexOf(' ') + 1) + 1, camString.lastIndexOf(' ')).toFloat();
        distance_pixels = camString.substring(camString.lastIndexOf(' ') + 1).toFloat();
        distance_pixels_protect = camString.substring(camString.indexOf(' ', camString.lastIndexOf(' ')+1)).toFloat();


    }
    String angleString = String(angle);
    String ballDistance = String(ball_distance);
    String ballAngle = String(ball_angle);
    String goalAngle = String(goal_angle);
    //String goalDistance = String(goal_distance);
    String distancePixels = String(distance_pixels);
    Serial2.print(angleString);
    Serial2.print(",");
    Serial2.print(ballDistance);
    Serial2.print(",");
    Serial2.print(ballAngle);
    Serial2.print(",");
    Serial2.print(goalAngle);
    Serial2.print(",");
    /*Serial2.print(goalDistance);
    Serial2.print(",");*/
    Serial2.print(distancePixels);
    Serial2.println();
}
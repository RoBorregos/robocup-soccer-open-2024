#include <iostream>
#include "Bno.h"
#include <Wire.h>
#include "Transmission.h"

#define RXD2 16
#define TXD2 17

float bno_angle = 0;
float distance_pixels = 0;
float ball_angle = 0;
float ball_distance = 0;
float goal_angle = 0;

BNO055 myBNO;

void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
    myBNO.InitializeBNO();
}

void loop()
{
    myBNO.getBNOData();
    bno_angle = myBNO.getYaw();
    if (Serial.available())
    {
        ParseCamString(ball_distance, ball_angle, goal_angle, distance_pixels);
    }
    SendDataSerial(bno_angle, ball_distance, ball_angle, goal_angle, distance_pixels);
}

#include "Transmission.h"

// Function to parse a string from the serial port and update the ball_distance, ball_angle, goal_angle, and distance_pixels variables from the Camera
void ParseCamString(float &ball_distance, float &ball_angle, float &goal_angle, float &distance_pixels)
{
    String camString = Serial.readStringUntil('\n');
    ball_distance = camString.toFloat();
    ball_angle = camString.substring(camString.indexOf(' ') + 1).toFloat();
    goal_angle = camString.substring(camString.indexOf(' ', camString.indexOf(' ') + 1) + 1, camString.lastIndexOf(' ')).toFloat();
    distance_pixels = camString.substring(camString.lastIndexOf(' ') + 1).toFloat();
}

// Function to send data over the serial port into the Raspberry Pi Pico
void SendDataSerial(float filtered_angle, float ball_distance, float ball_angle, float goal_angle, float distance_pixels)
{
    filtered_angle = String(filtered_angle);
    ball_distance = String(ball_distance);
    ball_angle = String(ball_angle);
    goal_angle = String(goal_angle);
    distance_pixels = String(distance_pixels);
    Serial2.print(filtered_angle);
    Serial2.print(",");
    Serial2.print(ball_distance);
    Serial2.print(",");
    Serial2.print(ball_angle);
    Serial2.print(",");
    Serial2.print(goal_angle);
    Serial2.print(",");
    Serial2.print(distance_pixels);
    Serial2.println();
}
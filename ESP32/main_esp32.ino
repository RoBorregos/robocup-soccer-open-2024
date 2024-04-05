#include <Arduino.h>
#include "bno.h"
#define RXD2 16
#define TXD2 17

BNO055 MyBno;

float angle = 0;
float cam_angle = 10;
float difference_angle = 0;
float target_angle = 0;

void setup()
{
    Serial.begin(9600);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    MyBno.InitializeBNO();
}

void loop()
{
    if (Serial.available())
    {
        String angle_string = Serial.readStringUntil('\n');
        cam_angle = angle_string.toFloat();
    }
    MyBno.GetBNOData();
    angle = MyBno.GetYaw();
    Send();
}

/* This method calculates the angle to which the robot should turn to face the ball
it needs the angle of the camera and the angle of the robot obtained from BNO */
void CalculateAngle()
{
    difference_angle = angle - cam_angle;
    target_angle = 2 * M_PI - difference_angle;
}

void Send()
{
    if (Serial2.available())
    {
        char data = Serial2.read();
        if (data == 's')
        {
            float temp;
            uint8_t tempArray[4];
            union u_tag
            {
                byte b[4];
                float angle;
            } u;
            u.angle = angle;
            tempArray[0] = u.b[0];
            tempArray[1] = u.b[1];
            tempArray[2] = u.b[2];
            tempArray[3] = u.b[3];
            Serial2.write(tempArray, 4);
            Serial2.flush();
        }
        if (data == 'c')
        {
            float temp;
            uint8_t tempArray[4];
            union u_tag
            {
                byte b[4];
                float angle;
            } u;
            u.angle = cam_angle;
            tempArray[0] = u.b[0];
            tempArray[1] = u.b[1];
            tempArray[2] = u.b[2];
            tempArray[3] = u.b[3];
            Serial2.write(tempArray, 4);
            Serial2.flush();
        }
    }
}

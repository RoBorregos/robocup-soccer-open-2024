//#include <Arduino.h>
#include "Bno.h"
#define RXD2 16
#define TXD2 17

BNO055 myBNO;

float angle = 0;
float ballAngle = 10;
float ballDistance = 10;
float goalAngle = 0;
float goalDistance = 0;
void setup()
{
    Serial.begin(9600);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    myBNO.InitializeBNO();
}

void loop()
{
    if (Serial.available())
    {
        String camString = Serial.readStringUntil('\n');
        ballDistance = camString.toFloat();
        ballAngle = camString.substring(camString.indexOf(' ') + 1).toFloat();
        goalAngle = camString.substring(camString.indexOf(' ', camString.indexOf(' ') + 1) + 1, camString.lastIndexOf(' ')).toFloat();
        goalDistance = camString.substring(camString.lastIndexOf(' ') + 1).toFloat();
    }
    myBNO.getBNOData();
    angle = myBNO.getYaw();
    //Serial.println(angle);
    send();
}

void send()
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
        u.angle = ballAngle;
        tempArray[0] = u.b[0];
        tempArray[1] = u.b[1];
        tempArray[2] = u.b[2];
        tempArray[3] = u.b[3];
        Serial2.write(tempArray, 4);
        Serial2.flush();
    }
    if (data == 'd')
    {
        float temp;
        uint8_t tempArray[4];
        union u_tag
        {
            byte b[4];
            float angle;
        } u;
        u.angle = ballDistance;
        tempArray[0] = u.b[0];
        tempArray[1] = u.b[1];
        tempArray[2] = u.b[2];
        tempArray[3] = u.b[3];
        Serial2.write(tempArray, 4);
        Serial2.flush();
      }
    if (data == 'g'){
        float temp;
        uint8_t tempArray[4];
        union u_tag
        {
            byte b[4];
            float angle;
        } u;
        u.angle = goalAngle;
        tempArray[0] = u.b[0];
        tempArray[1] = u.b[1];
        tempArray[2] = u.b[2];
        tempArray[3] = u.b[3];
        Serial2.write(tempArray, 4);
        Serial2.flush();
    }
    if (data == 'k'){
        float temp;
        uint8_t tempArray[4];
        union u_tag
        {
            byte b[4];
            float angle;
        } u;
        u.angle = goalDistance;
        tempArray[0] = u.b[0];
        tempArray[1] = u.b[1];
        tempArray[2] = u.b[2];
        tempArray[3] = u.b[3];
        Serial2.write(tempArray, 4);
        Serial2.flush();
    }

  }
}


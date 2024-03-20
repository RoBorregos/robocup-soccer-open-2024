//#include <Arduino.h>
#include "Bno.h"
#define RXD2 16
#define TXD2 17

BNO055 myBNO;

float angle = 0;
float camAngle = 10;
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
        String angleString = Serial.readStringUntil('\n');
        camAngle = angleString.toFloat();
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
        u.angle = camAngle;
        tempArray[0] = u.b[0];
        tempArray[1] = u.b[1];
        tempArray[2] = u.b[2];
        tempArray[3] = u.b[3];
        Serial2.write(tempArray, 4);
        Serial2.flush();
    }
    }
}


#include <Arduino.h>
#include "Bno.h"
#define RXD2 16
#define TXD2 17

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
}

void loop() {
  //Serial.println("Hello");
   while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }

  if (stringComplete) {
    int distance, angle;
    sscanf(inputString.c_str(), "%f %f", &distance, &angle);
    Serial2.write(distance);
    //Serial2.write(angle);
    inputString = "";
    stringComplete = false;
  }
  
}

/*BNO055 myBNO;

float angle = 0;
void setup()
{
    Serial.begin(9600);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    myBNO.InitializeBNO();
}

void loop()
{
    myBNO.getBNOData();
    angle = myBNO.getYaw();
    sendAngle();
}

void sendAngle()
{
    if (Serial2.read() == 's')
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
}*/
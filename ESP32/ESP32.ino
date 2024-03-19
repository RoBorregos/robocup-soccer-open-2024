#include <Arduino.h>
#include "Bno.h"
#define RXD2 16
#define TXD2 17

String inputString = "";
bool stringComplete = false;
char temp; 

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(25, HIGH);
  digitalWrite(26, HIGH);
  digitalWrite(4, HIGH);
}
float distance, angle;

void loop() {
  if (Serial2.available()) {
    temp = Serial2.read();
  }

  


  //Serial.println("Hello");
  /* while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }*/

  /*if (stringComplete) {
    sscanf(inputString.c_str(), "%f %f", &distance, &angle);
    inputString = "";
    stringComplete = false;
  }*/

  if(angle != 17.03){
    digitalWrite(25, LOW);
    digitalWrite(26, LOW);
    digitalWrite(4, LOW);
  }

  sendDistance();
  sendAngle();
  
}

void sendDistance()
{
  if(temp == 'e')
  {
    uint8_t tempArray[4];
    union u_tag
    {
      byte b[4];
      float distance;
    } u;
    u.distance = distance;
    tempArray[0] = u.b[0];
    tempArray[1] = u.b[1];
    tempArray[2] = u.b[2];
    tempArray[3] = u.b[3];
    Serial2.flush();
    Serial2.write(tempArray, 4);
    
  }
}

void sendAngle(){
  if(temp == 's')
  {
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
    Serial2.flush();
    Serial2.write(tempArray, 4);
    
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
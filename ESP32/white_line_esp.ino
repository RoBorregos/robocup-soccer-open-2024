#include <Arduino.h>
//#include <Adafruit_ADS1X15.h>
#include "bno.h"
#define RXD2 16
#define TXD2 17

BNO055 myBNO;
//Adafruit_ADS1115 ads_left;
//Adafruit_ADS1115 ads_right;
//Adafruit_ADS1115 ads_back;


/*int line_threshold_285 = 16070; //left 0
int line_threshold_276 = 13255; //left 1
int line_threshold_265 = 14287; //left 2
int line_threshold_254 = 18857; //left 3


int line_threshold_105 = 14999; //right 0
int line_threshold_96 = 17373; //right 1
int line_threshold_85 = 16700; //right 2
int line_threshold_74 = 20371; //right 3

int line_threshold_195 = 13903; //back 0
int line_threshold_186 = 16415; //back 1
int line_threshold_175 = 15417; //back 2
int line_threshold_164 = 14290; //back 3

int field_threshold_285 = 14885;
int field_threshold_276 = 11541;
int field_threshold_265 = 12852;
int field_threshold_254 = 17763;

int field_threshold_105 = 13816;
int field_threshold_96 = 16317;
int field_threshold_85 = 15797;
int field_threshold_74 = 19656;

int field_threshold_195 = 12657;
int field_threshold_186 = 15386;
int field_threshold_175 = 14555;
int field_threshold_164 = 13536;*/

//int umbralesLinea[] = {14885, 11541, 12852, 17763, 13816, 16317, 15797, 19656, 12657, 15386, 14555, 13536};
//int umbralesPista[] = {16070, 13255, 14287, 18857, 14999, 17373, 16700, 20371, 13903, 16415, 15417, 14290};

float angle = 0;
float ballAngle = 10;
float ballDistance = 10;
float goalAngle = 0;
float goalDistance = 0;
float averageLeft = 0;
float averageRight = 0;
float averageBack = 0;
float moveAngle = 0;
float lecture = 0;
float on_line = 0;
float on_field = 0; 

void setup(){
    Serial.begin(9600);
    myBNO.InitializeBNO();
   /*ads_left.begin(0x48);
   ads_right.begin(0x49);
   ads_back.begin(0x4A);
   ads_left.setGain(GAIN_FOUR);
   ads_right.setGain(GAIN_FOUR);
   ads_back.setGain(GAIN_FOUR);*/
}

void loop(){
    if (Serial.available())
    {
        String camString = Serial.readStringUntil('\n');
        ballDistance = camString.toFloat();
        ballAngle = camString.substring(camString.indexOf(' ') + 1).toFloat();
        goalAngle = camString.substring(camString.indexOf(' ', camString.indexOf(' ') + 1) + 1, camString.lastIndexOf(' ')).toFloat();
        goalDistance = camString.substring(camString.lastIndexOf(' ') + 1).toFloat();
    }
    myBNO.GetBNOData();
    angle = myBNO.GetYaw();

    /*int sensorLinea[] = {
        ads_left.readADC_SingleEnded(0),
        ads_left.readADC_SingleEnded(1),
        ads_left.readADC_SingleEnded(2),
        ads_left.readADC_SingleEnded(3),
        ads_right.readADC_SingleEnded(0),
        ads_right.readADC_SingleEnded(1),
        ads_right.readADC_SingleEnded(2),
        ads_right.readADC_SingleEnded(3),
        ads_back.readADC_SingleEnded(0),
        ads_back.readADC_SingleEnded(1),
        ads_back.readADC_SingleEnded(2),
        ads_back.readADC_SingleEnded(3)
    };*/
    /*for (int i = 0; i < 12; ++i) {
        if (sensorLinea[i] > umbralesLinea[i] && sensorLinea[i] > umbralesPista[i]) {
            on_line = 1.0;
            //return (i);
        }else{
            on_line = 0.0;
        }
        
    }*/
    //return 0;
    //on_line = 1.0;
    Send();
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
    /*if(data == 'a'){
        float temp;
        uint8_t tempArray[4];
        union u_tag
        {
            byte b[4];
            float angle;
        } u;
        u.angle = on_line;
        tempArray[0] = u.b[0];
        tempArray[1] = u.b[1];
        tempArray[2] = u.b[2];
        tempArray[3] = u.b[3];
        Serial2.write(tempArray, 4);
        Serial2.flush();
    }*/
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
    if (data == 'r'){
        float temp;
        uint8_t tempArray[4];
        union u_tag
        {
            byte b[4];
            float angle;
        } u;
        u.angle = lecture;
        tempArray[0] = u.b[0];
        tempArray[1] = u.b[1];
        tempArray[2] = u.b[2];
        tempArray[3] = u.b[3];
        Serial2.write(tempArray, 4);
        Serial2.flush();
    }
  }
}
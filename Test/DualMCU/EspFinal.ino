#include <iostream>
#include "Bno.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads_left; // Array to hold instances of ADS1115
Adafruit_ADS1115 ads_right;
Adafruit_ADS1115 ads_back;

int last_ads_value = 0;
int current_ads_value = 0;
float bno_angle = 0;
float ball_angle = 0;
float ball_distance = 0;
float goal_angle = 0;
float goal_distance = 0;
double last_time = 0;
double current_time = 0;

struct PhotoValues
{
    int adc285;
    int adc276;
    int adc265;
    int adc254;
    int adc105;
    int adc96;
    int adc85;
    int adc74;
    int adc195;
    int adc186;
    int adc175;
    int adc164;
};

int angle_values[] = {285, 276, 265, 254, 105, 96, 85, 74, 195, 186, 175, 164};
int last_ads_values[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define RXD2 16
#define TXD2 17

BNO055 myBNO;

float angle = 0;
int moveAngle = 0;
int avg = 0;
int fleft = 0;
int fright = 0;

void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
    myBNO.InitializeBNO();
    ads_left.begin(0x48);        
    ads_left.setGain(GAIN_FOUR); 
    ads_right.begin(0x49);       
    ads_right.setGain(GAIN_FOUR); 
    ads_back.begin(0x4A);        
    ads_back.setGain(GAIN_FOUR);  
}

void loop()
{
    fleft = 0;
    fright = 0;
    moveAngle = 0;
    avg = 0;
    angle = myBNO.GetAngle();
    if (Serial.available())
    {
        String camString = Serial.readStringUntil('\n');
        ball_distance = camString.toFloat();
        ball_angle = camString.substring(camString.indexOf(' ') + 1).toFloat();
        goal_angle = camString.substring(camString.indexOf(' ', camString.indexOf(' ') + 1) + 1, camString.lastIndexOf(' ')).toFloat();
        goal_distance = camString.substring(camString.lastIndexOf(' ') + 1).toFloat();
    }
    // LEFT
    PhotoValues photo_values = {
        .adc285 = ads_left.readADC_SingleEnded(0),
        .adc276 = ads_left.readADC_SingleEnded(1),
        .adc265 = ads_left.readADC_SingleEnded(2),
        .adc254 = ads_left.readADC_SingleEnded(3),
        // RIGHT
        .adc105 = ads_right.readADC_SingleEnded(0),
        .adc96 = ads_right.readADC_SingleEnded(1),
        .adc85 = ads_right.readADC_SingleEnded(2),
        .adc74 = ads_right.readADC_SingleEnded(3),
        // BACK
        .adc195 = ads_back.readADC_SingleEnded(0),
        .adc186 = ads_back.readADC_SingleEnded(1),
        .adc175 = ads_back.readADC_SingleEnded(2),
        .adc164 = ads_back.readADC_SingleEnded(3)};
    for (int i = 0; i < 12; i++)
    {
        switch (i)
        {
        case 0:
            current_ads_value = photo_values.adc285;
            break;
        case 1:
            current_ads_value = photo_values.adc276;
            break;
        case 2:
            current_ads_value = photo_values.adc265;
            break;
        case 3:
            current_ads_value = photo_values.adc254;
            break;
        case 4:
            current_ads_value = photo_values.adc105;
            break;
        case 5:
            current_ads_value = photo_values.adc96;
            break;
        case 6:
            current_ads_value = photo_values.adc85;
            break;
        case 7:
            current_ads_value = photo_values.adc74;
            break;
        case 8:
            current_ads_value = photo_values.adc195;
            break;
        case 9:
            current_ads_value = photo_values.adc186;
            break;
        case 10:
            current_ads_value = photo_values.adc175;
            break;
        case 11:
            current_ads_value = photo_values.adc164;
            break;
        }

        if (current_ads_value > last_ads_values[i] + 410)
        {
            moveAngle += angle_values[i];
            avg++; /*
             if(i >= 0 && i <= 3){
               fright++;
             }
             if(i >= 8 && i <= 11){
               fleft++;
             }*/
        }
        last_ads_values[i] = current_ads_value;
    }
    /*
        if(fright > 0 && fleft > 0){
            moveAngle = 180;
        }   else if (avg > 0){
            moveAngle = (moveAngle / avg) + 180;
        }else {
            moveAngle = -1;
        }*/
    String angleString = String(angle);
    String ballDistance = String(ball_distance);
    String ballAngle = String(ball_angle);
    String goalAngle = String(goal_angle);
    String goalDistance = String(goal_distance);
    String moveAngle = String(moveAngle);
    Serial2.print(angleString);
    Serial2.print(",");
    Serial2.print(ballDistance);
    Serial2.print(",");
    Serial2.print(ballAngle);
    Serial2.print(",");
    Serial2.print(goalAngle);
    Serial2.print(",");
    Serial2.print(goalDistance);
    Serial2.print(",");
    /*Serial2.print(moveAngle);
    Serial2.println();*/
}

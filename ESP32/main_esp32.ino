#include <Arduino.h>
#include "bno.h"
#include "Photo.h"
#define RXD2 16
#define TXD2 17

BNO055 MyBno;
Photo photo;

float angle = 0;
float cam_angle = 10;
float difference_angle = 0;
float target_angle = 0;
float ads_init = 0;
float ads_values = 0;
float individual_left_0 = 0; 
float individual_left_1 = 0;
float individual_left_2 = 0; 
float individual_left_3 = 0;
float individual_right_0 = 0;
float individual_right_1 = 0;
float individual_right_2 = 0;
float individual_right_3 = 0;
float individual_back_0 = 0;
float individual_back_1 = 0;
float individual_back_2 = 0;
float individual_back_3 = 0;


void setup()
{
    Serial.begin(9600);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    MyBno.InitializeBNO();
    if(photo.InitializeADS())
    {
        ads_init = 1;
    }
    else
    {
        ads_init = 0;
    }
}

void loop()
{
    
    individual_back_0 = photo.GetADSBackIndividual(0);
    individual_back_1 = photo.GetADSBackIndividual(1);
    individual_back_2 = photo.GetADSBackIndividual(2);
    individual_back_3 = photo.GetADSBackIndividual(3);
    individual_left_0 = photo.GetADSLeftIndividual(0);
    individual_left_1 = photo.GetADSLeftIndividual(1);
    individual_left_2 = photo.GetADSLeftIndividual(2);
    individual_left_3 = photo.GetADSLeftIndividual(3);
    individual_right_0 = photo.GetADSRightIndividual(0);
    individual_right_1 = photo.GetADSRightIndividual(1);
    individual_right_2 = photo.GetADSRightIndividual(2);
    individual_right_3 = photo.GetADSRightIndividual(3);
    
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
        
        if(data == 'a'){
            float temp;
            uint8_t tempArray[4];
            union u_tag
            {
                byte b[4];
                float angle;
            } u;
            u.angle = individual_back_0;
            tempArray[0] = u.b[0];
            tempArray[1] = u.b[1];
            tempArray[2] = u.b[2];
            tempArray[3] = u.b[3];
            Serial2.write(tempArray, 4);
            Serial2.flush();
        }
        if(data == 'b'){
            float temp;
            uint8_t tempArray[4];
            union u_tag
            {
                byte b[4];
                float angle;
            } u;
            u.angle = individual_back_1;
            tempArray[0] = u.b[0];
            tempArrafy[1] = u.b[1];
            tempArray[2] = u.b[2];
            tempArray[3] = u.b[3];
            Serial2.write(tempArray, 4);
            Serial2.flush();
        }
        if(data == 'c'){
            float temp;
            uint8_t tempArray[4];
            union u_tag
            {
                byte b[4];
                float angle;
            } u;
            u.angle = individual_back_2;
            tempArray[0] = u.b[0];
            tempArray[1] = u.b[1];
            tempArray[2] = u.b[2];
            tempArray[3] = u.b[3];
            Serial2.write(tempArray, 4);
            Serial2.flush();
        }
        if(data == 'd'){
            float temp;
            uint8_t tempArray[4];
            union u_tag
            {
                byte b[4];
                float angle;
            } u;
            u.angle = individual_back_3;
            tempArray[0] = u.b[0];
            tempArray[1] = u.b[1];
            tempArray[2] = u.b[2];
            tempArray[3] = u.b[3];
            Serial2.write(tempArray, 4);
            Serial2.flush();
        }
        if(data == 'e'){
            float temp;
            uint8_t tempArray[4];
            union u_tag
            {
                byte b[4];
                float angle;
            } u;
            u.angle = individual_left_0;
            tempArray[0] = u.b[0];
            tempArray[1] = u.b[1];
            tempArray[2] = u.b[2];
            tempArray[3] = u.b[3];
            Serial2.write(tempArray, 4);
            Serial2.flush();
        }
        if(data == 'f'){
            float temp;
            uint8_t tempArray[4];
            union u_tag
            {
                byte b[4];
                float angle;
            } u;
            u.angle = individual_left_1;
            tempArray[0] = u.b[0];
            tempArray[1] = u.b[1];
            tempArray[2] = u.b[2];
            tempArray[3] = u.b[3];
            Serial2.write(tempArray, 4);
            Serial2.flush();
        }
        if(data == 'g'){
            float temp;
            uint8_t tempArray[4];
            union u_tag
            {
                byte b[4];
                float angle;
            } u;
            u.angle = individual_left_2;
            tempArray[0] = u.b[0];
            tempArray[1] = u.b[1];
            tempArray[2] = u.b[2];
            tempArray[3] = u.b[3];
            Serial2.write(tempArray, 4);
            Serial2.flush();
        }
        if(data == 'h'){
            float temp;
            uint8_t tempArray[4];
            union u_tag
            {
                byte b[4];
                float angle;
            } u;
            u.angle = individual_left_3;
            tempArray[0] = u.b[0];
            tempArray[1] = u.b[1];
            tempArray[2] = u.b[2];
            tempArray[3] = u.b[3];
            Serial2.write(tempArray, 4);
            Serial2.flush();
        }
        if(data == 'i'){
            float temp;
            uint8_t tempArray[4];
            union u_tag
            {
                byte b[4];
                float angle;
            } u;
            u.angle = individual_right_0;
            tempArray[0] = u.b[0];
            tempArray[1] = u.b[1];
            tempArray[2] = u.b[2];
            tempArray[3] = u.b[3];
            Serial2.write(tempArray, 4);
            Serial2.flush();
        }
        if(data == 'j'){
            float temp;
            uint8_t tempArray[4];
            union u_tag
            {
                byte b[4];
                float angle;
            } u;
            u.angle = individual_right_1;
            tempArray[0] = u.b[0];
            tempArray[1] = u.b[1];
            tempArray[2] = u.b[2];
            tempArray[3] = u.b[3];
            Serial2.write(tempArray, 4);
            Serial2.flush();
        }
        if(data == 'k'){
            float temp;
            uint8_t tempArray[4];
            union u_tag
            {
                byte b[4];
                float angle;
            } u;
            u.angle = individual_right_2;
            tempArray[0] = u.b[0];
            tempArray[1] = u.b[1];
            tempArray[2] = u.b[2];
            tempArray[3] = u.b[3];
            Serial2.write(tempArray, 4);
            Serial2.flush();
        }
        if(data == 'l'){
            float temp;
            uint8_t tempArray[4];
            union u_tag
            {
                byte b[4];
                float angle;
            } u;
            u.angle = individual_right_3;
            tempArray[0] = u.b[0];
            tempArray[1] = u.b[1];
            tempArray[2] = u.b[2];
            tempArray[3] = u.b[3];
            Serial2.write(tempArray, 4);
            Serial2.flush();
        }
        if(data == 'm'){
            float temp;
            uint8_t tempArray[4];
            union u_tag
            {
                byte b[4];
                float angle;
            } u;
            u.angle = ads_init;
            tempArray[0] = u.b[0];
            tempArray[1] = u.b[1];
            tempArray[2] = u.b[2];
            tempArray[3] = u.b[3];
            Serial2.write(tempArray, 4);
            Serial2.flush();
        }
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
        if (data == 'z')
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

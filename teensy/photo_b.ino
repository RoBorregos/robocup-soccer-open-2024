#include <iostream>
#include <Wire.h>
#include "Bno.h"
#include <Pixy2SPI_SS.h>
#include <Arduino.h>
#include "PID.h"
#include "Motors.h"
#include <typeinfo>

float angle = 0; 
double ball_distance = 0;
double ball_angle = 0;
double goal_angle = 0;
double distance_pixels = 0;
int photo_value_right1;
int photo_value_right2;
int photo_value_left;
int photo_value_back1;
int photo_value_back2;
int photo_value_front1;
int photo_value_front2;
int photo_value_front3;
int speed_photos = 0; 

BNO055 myBNO;
Motors myMotors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);

PID pid_w(0.6, 0.01, 6, 200);

void setup(){
 //Serial1.begin(115200);
    Serial.begin(9600);
    myMotors.InitializeMotors();
    myBNO.InitializeBNO();
    analogReadResolution(12);
    Serial.print("INIT");
}

void timeLoop (long int startMillis, long int interval){ // the delay function

    // this loops until 2 milliseconds has passed since the function began
    while(millis() - startMillis < interval){} 
}

void loop(){
   myBNO.GetBNOData();
    angle = myBNO.GetYaw();
    //A2 A7 
    double speed_w = pid_w.Calculate(0, angle);
    speed_photos = 200;
  photo_value_right1 = analogRead(A2);
  photo_value_right2 = analogRead(A7);
  photo_value_left = analogRead(A3);
  photo_value_front1 = analogRead(A10);
  photo_value_front2 = analogRead(A11);
  photo_value_back1 = analogRead(A8);
  photo_value_back2 = analogRead(A9);


     Serial.print("PHOTO A2: ");
      Serial.println(photo_value_right1);//90°
     Serial.print("PHOTO A7: ");
     Serial.println(photo_value_right2);//96°
    Serial.print("PHOTO A3: ");
    Serial.println(photo_value_left);//256°
    Serial.print("PHOTO A12: ");
    Serial.println(photo_value_front1);//180°
    Serial.print("PHOTO A13: ");
    Serial.println(photo_value_front2);//173°
     Serial.print("PHOTO A14: ");
    Serial.println(photo_value_front3);//166°


    /* Serial.print("PHOTO A8: ");
    Serial.println(photo_value_back1);
     Serial.print("PHOTO A9: ");
    Serial.println(photo_value_back2);*/

  if (speed_w != 0)
  {
    //----------------------- Photoresistors detection ---------------------------//
    if (photo_value_front1 > 3600 || photo_value_front2 > 3600)
  {
    myMotors.MoveMotorsImu(0, speed_photos, speed_w);
    timeLoop(millis(), 300);
    Serial.println("Adelante");
  }
  else if (photo_value_right1 > 2300 || photo_value_right2 > 2300)
  { 
    myMotors.MoveMotorsImu(90, speed_photos, speed_w);
    timeLoop(millis(), 300);
    Serial.println("DERECHA");
  }
  else if (photo_value_back1 > 2300 || photo_value_back2 > 1700)
  {
    myMotors.MoveMotorsImu(180, speed_photos, speed_w);
    timeLoop(millis(), 300);
    Serial.println("ATRAS");
  }
  else if (photo_value_left > 2500)
  {
    myMotors.MoveMotorsImu(270, speed_photos, speed_w);
    timeLoop(millis(), 300);
    Serial.println("Izquierda");
  }
    else {
      myMotors.MoveMotorsImu(0, 0, 0);
      //Serial.println("stop");
    }
  }
}
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

void loop(){
    //A2 A7 
    int photoValue = analogRead(A2);
    int photoValue1 = analogRead(A7);
    int photoValue2 = analogRead(A3); //no
    const int photoValue3 = analogRead(A12);
    const int photoValue4 = analogRead(A13);
    const int photoValue5 = analogRead(A14);
    const int analogPin1 = analogRead(A8);
   const int analogPin2 = analogRead(A9);
   const int photo_value5 = analogRead(A15);
    const int photo_value6 = analogRead(A17);
     const int photo_value7 = analogRead(A6);
    Serial.print("PHOTO 3: ");
    Serial.println(photoValue3); 
    Serial.print("PHOTO 4: "); 
    Serial.println(photoValue4);
    Serial.print("PHOTO 5: ");
    Serial.println(photoValue5);
    myBNO.GetBNOData();
    angle = myBNO.GetYaw();
    //Serial.println(angle);
    double speed_w = pid_w.Calculate(0, angle);
    if(speed_w!=0){
      if (photoValue > 3500 || photoValue1 > 2200) {
      myMotors.MoveMotorsImu(0, 200, speed_w);
      delay(300);
      Serial.println("Adelante");
    }else if (photoValue3 > 2800 || photoValue5 > 3500) {
      myMotors.MoveMotorsImu(270, 200, speed_w);
      delay(300);
      Serial.println("DERECHA");
    } 
    else if (analogPin1 > 1300 || analogPin2 > 2500 ) {
      myMotors.MoveMotorsImu(180, 200, speed_w);
      delay(300);
      Serial.println("ATRAS");
    }
    else if(photo_value5 > 3700 || photo_value6 > 3700 || photo_value7 > 3700){
      myMotors.MoveMotorsImu(90, 200, speed_w);
      delay(300);
      Serial.println("Izquierda");
    
    }
    else {
      myMotors.MoveMotorsImu(0, 0, 0);
      //Serial.println("stop");
    }
  }
}
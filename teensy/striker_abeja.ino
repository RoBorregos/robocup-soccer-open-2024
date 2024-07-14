#include <iostream>
#include <Wire.h>
#include "Bno.h"
#include <cmath>
#include "PID.h"
#include <Pixy2SPI_SS.h>
#include "Motors.h"
#include <Servo.h>

/*
Pixy camera resolution: 316 x 208
pixy.ccc.blocks[i].m_x The x location of the center of the detected object (0 to 316)
pixy.ccc.blocks[i].m_y The y location of the center of the detected object (0 to 208)
*/

double goal_angle_opposite = 0;
unsigned long start_millis;
unsigned long current_millis;
float bno_angle = 0;
bool ball_seen_openmv = false;
double target_angle = 0;
float ball_angle_180 = 0;
float distance_pixels = 0;
float distance_pixels_protect = 0;
float ball_angle = 0;
float ball_distance = 0;
float goal_angle = 0;
bool ball_found = false;
double ponderated_angle = 0;
double angle_degrees = 0;
float goal_distance = 0;
double last_time = 0;
double current_time = 0;
double last_ball_angle = 0;
int goal_threshold = 15;
const int FRAME_HEIGHT = 104;
const int FRAME_WIDTH = 158;
const int FRAME_ROBOT = 20;
const int FRAME_CIRCLE = 150;
const unsigned long period = 100;
const int delay_time = 3000;
const int min_speed = 1000;
const int mid_speed = 1500;
const int max_speed = 2000;
const int esc_pin = 6;
const int kicker = 32;

int counterball = 0;

  int photoValue = analogRead(A2);
    int photoValue1 = analogRead(A7);
    int photoValue2 = analogRead(A3); //no
     int photoValue3 = analogRead(A12);
     int photoValue4 = analogRead(A13);
     int photoValue5 = analogRead(A14);
     int analogPin1 = analogRead(A8);
    int analogPin2 = analogRead(A9);
    int photo_value5 = analogRead(A15);
     int photo_value6 = analogRead(A17);
      int photo_value7 = analogRead(A6);

Pixy2SPI_SS pixy;
BNO055 my_bno;
Servo esc;

PID pid_w(0.6, 0.00735, 45, 200);
Motors myMotors(
  MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
  MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
  MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
  MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);

void setup() {
  esc.attach(esc_pin);
  Serial1.begin(115200);
  Serial.begin(9600);
  esc.writeMicroseconds(min_speed);
  pixy.init();
  my_bno.InitializeBNO();
    pinMode(kicker, OUTPUT);
  analogReadResolution(12);
  Serial.println("Arming ESC...");
  start_millis = millis();
}

double radiansToDegrees(double radians) {
  return radians * (180.0 / M_PI);
}

void timeLoop(long int startMillis, long int interval)
{
  while (millis() - startMillis < interval)
  {
  }
}

void loop() {

  // ----------------- Gather data from OpenMV camera via UART ----------------- //
  my_bno.GetBNOData();
  bno_angle = my_bno.GetYaw();
  if (Serial1.available()) {
    String camString = Serial1.readStringUntil('\n');
    ball_distance = camString.toFloat();
    ball_angle = camString.substring(camString.indexOf(' ') + 1).toFloat();
    goal_angle = camString.substring(camString.indexOf(' ', camString.indexOf(' ') + 1) + 1, camString.lastIndexOf(' ')).toFloat();
    distance_pixels = camString.substring(camString.lastIndexOf(' ') + 1).toFloat();
    ball_seen_openmv = (ball_distance != 0 || ball_angle != 0);
    Serial.println(goal_angle);
  }

  // ----------------- Gather data from Pixy2 camera via SPI ----------------- //

  int i;
  pixy.ccc.getBlocks();
  int pixy_blocks = pixy.ccc.numBlocks;
  bool ball_seen_pixy = false;

  if (pixy.ccc.numBlocks) {
    for (i = 0; i < pixy.ccc.numBlocks; i++) {
      double cx = pixy.ccc.blocks[i].m_x;
      double cy = pixy.ccc.blocks[i].m_y;
      double relative_cx = cx - 158;
      double relative_cy = cy - 208;
      double angle_radians = atan2(relative_cy, relative_cx);
      angle_degrees = radiansToDegrees(angle_radians);

      angle_degrees += 95;
      if (angle_degrees < 0) {
        angle_degrees += 360;
      }
      if (angle_degrees >= 360) {
        angle_degrees -= 360;
      }

      if (pixy.ccc.blocks[i].m_signature == 1) {
        ball_seen_pixy = true;
      } else {
        ball_seen_pixy = false;
      }
    }
  }

  //----------------------- Calculate PIDS-----------------------------//

  double speed_w = pid_w.Calculate(target_angle, bno_angle);
  double speed_t_goal = 150;
  double speed_t_ball = 150;
     photoValue = analogRead(A2);
     photoValue1 = analogRead(A7);
     photoValue2 = analogRead(A3); //no
      photoValue3 = analogRead(A12);
      photoValue4 = analogRead(A13);
      photoValue5 = analogRead(A14);
      analogPin1 = analogRead(A8);
     analogPin2 = analogRead(A9);
     photo_value5 = analogRead(A15);
    photo_value6 = analogRead(A17);
       photo_value7 = analogRead(A6);


  if (speed_w != 0)
  {
      /*Serial.print("PHOTO VALUE 1: ");
      Serial.println(photo_value);
      Serial.print("PHOTO VALUE 2: ");
      Serial.println(photo_value1);
      Serial.print("PHOTO VALUE 3: ");
      Serial.println(photo_value2);
      Serial.print("PHOTO VALUE 4: ");
      Serial.println(photo_value3);
      Serial.print("PHOTO VALUE 5: ");
      Serial.println(photo_value4);*/

    //----------------------- Photoresistors detection ---------------------------//
    if (photoValue > 3500 || photoValue1 > 2100) {
      myMotors.MoveMotorsImu(0, 200, speed_w);
      delay(300);
      Serial.println("Adelante");
    }else if (photoValue2 > 2200) {
      myMotors.MoveMotorsImu(270, 200, speed_w);
      delay(300);
      Serial.println("DERECHA");
    } 
    else if (analogPin1 > 1300 || analogPin2 > 2100) {
      myMotors.MoveMotorsImu(180, 200, speed_w);
      delay(300);
      Serial.println("ATRAS");
    }
    else if(photo_value5 > 3500 || photo_value6 > 3100 || photo_value7 > 3600){
      myMotors.MoveMotorsImu(90, 200, speed_w);
      delay(300);
      Serial.println("Izquierda");
    
    } else {
      //------------------ Angle normalization ------------------//
      if (ball_angle < 180) {
        ball_angle_180 = -ball_angle;
      } else if (ball_angle > 180) {
        ball_angle_180 = 360 - ball_angle;
      }
      ball_angle_180 = ball_angle_180 * (-1);

      //------------------ Camera detection cases ------------------//
      if (ball_seen_pixy && ball_seen_openmv) {
        counterball=0;
        last_ball_angle = ball_angle;
        if (distance_pixels < 85) {
          myMotors.MoveMotorsImu(goal_angle, speed_t_ball, speed_w);
          Serial.println("Both cameras see the ball and shoots");
           digitalWrite(kicker, HIGH);
            delay(20);
            digitalWrite(kicker, LOW);
        } else {
          counterball=0;
          myMotors.MoveMotorsImu(angle_degrees, speed_t_ball, speed_w);
          Serial.println("Both cameras see the ball");
        }
      } else if (ball_seen_pixy && !ball_seen_openmv) {
        counterball=0;
        last_ball_angle = ball_angle;
        if (distance_pixels < 85) {
          myMotors.MoveMotorsImu(goal_angle, speed_t_ball, speed_w);
          esc.writeMicroseconds(mid_speed);
          Serial.println("Only pixy sees the ball and shoots");
           digitalWrite(kicker, HIGH);
            delay(20);
            digitalWrite(kicker, LOW);
        } else {
          myMotors.MoveMotorsImu(angle_degrees, speed_t_ball, speed_w);
          Serial.println("Only pixy sees the ball");
        }
      } else if (!ball_seen_pixy && ball_seen_openmv) {
        counterball=0;
        last_ball_angle = ball_angle;
        double differential = ball_angle_180 * 0.15;
        ponderated_angle = ball_angle + differential;
        myMotors.MoveMotorsImu(ponderated_angle, abs(speed_t_ball), speed_w);
        Serial.println("Only OpenMV sees the ball");
      }
      //---------------------- Center robot in goal ----------------------//
      /*else if (goal_angle > 0)
      {
        if (goal_angle < (185 - goal_threshold) && ball_found == false)
        {
          myMotors.MoveMotorsImu(90, abs(speed_t_goal), speed_w);
        }
        else if (goal_angle > (185 + goal_threshold) && ball_found == false)
        {
          myMotors.MoveMotorsImu(270, abs(speed_t_goal), speed_w);
        }
        else
        {
          myMotors.MoveMotorsImu(ball_angle, 0, speed_w);
        }
      }*/
      //-------------------- Move to last known position --------------------//
      

      else if (counterball <=1) {
        Serial.print("COUNTERBALL1: ");
        Serial.println(counterball);
        if(last_ball_angle > 180){
        myMotors.MoveMotorsImu(90, 200, speed_w);
        timeLoop(millis(), 200);
      }else if(last_ball_angle < 180){
        myMotors.MoveMotorsImu(270, 200, speed_w);
        timeLoop(millis(), 200);
      }
        
        counterball++;
      }
      else if(counterball == 3 ){
        Serial.print("COUNTERBALL2: ");
        Serial.println(counterball);
        myMotors.MoveMotorsImu(180, speed_t_ball, speed_w);
        timeLoop(millis(), 170);
        counterball++;
      }
      else if(counterball == 5 ){
        
           Serial.print("COUNTERBALL3: ");
        Serial.println(counterball);
         if(last_ball_angle > 180){
        myMotors.MoveMotorsImu(90, 200, speed_w);
        timeLoop(millis(), 200);
      }else if(last_ball_angle < 180){
        myMotors.MoveMotorsImu(270, 200, speed_w);
        timeLoop(millis(), 200);
      }
        counterball++;
      }
      else{
        myMotors.MoveMotorsImu(0,0,0);
        timeLoop(millis(), 150);
        counterball++;
      }
    }
  }
}

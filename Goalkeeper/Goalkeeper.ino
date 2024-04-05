/*Main code for the goalkeeper robot*/
//Work in progress. Still defining objectives and requirements
// The robot moves in its own axis to follow the ball and moves right or left to keep the ball in the center of the camera using PID controller for the camera tracking and Bang Bang for the traslation
#include <Arduino.h>
#include <motors.h>
#include <serial.h>
#include <PID.h>
#include <constants.h>
#include <typeinfo>

#define PIN_SERIAL1_TX (0u)
#define PIN_SERIAL1_RX (1u)

SerialCommunication serialComm(Serial1);

float bno_angle = 0;
float cam_angle = 0;
float diff_angle = 0;
float distance = 0;

PID pid(1.05, 3.2, 0.2, 30);

//PID
double target_angle = 0; //frame
double traslation_angle = 0;
double last_error = 0;
double all_error = 0;

// Bang Bang / histéresis / On-off
float bang_bang_threshold = 7.5;

//Sampling Time
double last_time = 0;
double current_time = 0;
double delta_time = 0;

//spd
int speed_traslational = 0;


Motors myMotors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);



void setup()
{
    myMotors.InitializeMotors();
    Serial.begin(115200);
    while (!Serial && millis() < 10000UL);
    Serial.println("Started");
    Serial1.begin(9600);
}

void loop()
{
    // Receive Data
    bno_angle = serialComm.Receive(RECEIVE_BNO);
    cam_angle = serialComm.Receive(RECEIVE_BALL_ANGLE);
    distance = serialComm.Receive(RECEIVE_BALL_DISTANCE);
    Serial.print("BNO:");Serial.print(bno_angle); Serial.print(" ");
    Serial.print("CAM:");Serial.print(cam_angle); Serial.print(" ");

    //Separate target angle 180 and -180
    if (cam_angle == 0){
        
    }else if(cam_angle == 10){
      cam_angle = 0;
    }
    else if (cam_angle < 180)
    {
        cam_angle = -cam_angle;
    }
    else if (cam_angle > 180)
    {
        cam_angle = 360 - cam_angle;
    }

    Serial.print(" Target Angle: "); Serial.println(target_angle);
   // PID & Motor Control
    double speed_w = pid.Calculate(target_angle, bno_angle);
    
    //Traslation
    if(cam_angle < (0 - bang_bang_threshold)){
        myMotors.MoveMotorsImu(270, 190, speed_w);
        Serial.println("LEFT");
    }
    else if(cam_angle > (0 + bang_bang_threshold)){
        myMotors.MoveMotorsImu(90, 190, speed_w);
        Serial.println("RIGHT");
    } else {
        myMotors.MoveMotorsImu(0, 0, speed_w);
        Serial.println("Stop");
    }
    delay(20);
}



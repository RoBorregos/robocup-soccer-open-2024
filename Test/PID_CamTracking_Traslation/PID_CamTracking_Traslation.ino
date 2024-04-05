/*WORK IN PROGRESS*/
/**/

#include <Arduino.h>
#include <Motors.h>
#include <typeinfo>

#define PIN_SERIAL1_TX (0u)
#define PIN_SERIAL1_RX (1u)

uint8_t motor3In1 = 23;
uint8_t motor3In2 = 22;
uint8_t motor3PWM = 15;

uint8_t motor2In1 = 25;
uint8_t motor2In2 = 6;
uint8_t motor2PWM = 20;

uint8_t motor4In1 = 9;
uint8_t motor4In2 = 8;
uint8_t motor4PWM = 10;

uint8_t motorIn1 = 14;
uint8_t motorIn2 = 11;
uint8_t motorPWM = 21;

const uint8_t receive_bno = 's';
const uint8_t receive_cam = 'c';
const uint8_t receive_distance = 'd';
float bno_angle = 0;
float cam_angle = 0;
float diff_angle = 0;
float distance = 0;

//PID
double kp = 1.05;
double ki = 3.2;
double kd = 0.2;
double target_angle = 0; //frame
double traslation_angle = 0;
double last_error = 0;
double all_error = 0;
double max_error = 30;

//Sampling Time
double last_time = 0;
double current_time = 0;
double delta_time = 0;

//spd
int speed_traslational = 0;


Motors myMotors(
    motorPWM, motorIn1, motorIn2,
    motor2PWM, motor2In1, motor2In2,
    motor3PWM, motor3In1, motor3In2,
    motor4PWM, motor4In1, motor4In2);



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
    bno_angle = receive(receive_bno);
    cam_angle = receive(receive_cam);
    distance = receive(receive_distance);
    Serial.print("BNO:");Serial.print(bno_angle); Serial.print(" ");
    Serial.print("CAM:");Serial.print(cam_angle); Serial.print(" ");
    //Serial.println(distance);
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
    //Calculate difference angle
     if (cam_angle != 0)
    {
        Serial.print(" Angle difference: "); 
        diff_angle = cam_angle;
        Serial.print(diff_angle);
    } else {
        diff_angle = 0;
    }
    //Set target angle
    if (diff_angle != 0)
    {
        target_angle = bno_angle + diff_angle;
    }
    else
    {
        target_angle = 0;
    }
    Serial.print(" Target Angle: "); Serial.println(target_angle);
   // PID & Motor Control
    current_time = millis();
    delta_time = current_time - last_time;
    double new_error = target_angle - bno_angle;
    all_error = new_error + last_error;
    if (all_error > max_error)
    {
        all_error = max_error;
    }
    else if (all_error < -max_error)
    {
        all_error = -max_error;
    }
    double sample_time = delta_time/1000.0;
    double proportional = kp * new_error;
    double integer = ki * all_error * sample_time;
    double derivative = kd * (new_error - last_error) / sample_time;
    double speed_w = proportional + integer + derivative;
    last_error = new_error;
    last_time = current_time;
    
    //Traslation
    if(target_angle > 0){
        myMotors.moveMotorsImu(90, 200, speed_w);
        Serial.println("Left");
    }
    else if(target_angle < 0){
        myMotors.moveMotorsImu(270, 200, speed_w);
        Serial.println("Right");
    } else {
        myMotors.moveMotorsImu(0, 0, speed_w);
        Serial.println("Stop");
    }
    //myMotors.moveMotorsImu(traslation_angle, speed_traslational, speed_w);
    delay(20);
}


float receive (uint8_t signal){
     Serial1.write(signal);
    while (!Serial1.available())
    {
        continue;
    }
    delay(10);
    float temp;
    uint8_t tempArray[4];
    union u_tag
    {
        byte b[4];
        float angle;
    } u;
    u.b[0] = Serial1.read();
    u.b[1] = Serial1.read();
    u.b[2] = Serial1.read();
    u.b[3] = Serial1.read();
    return u.angle;
} 

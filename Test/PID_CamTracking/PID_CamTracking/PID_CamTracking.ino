/*Code for testing camera and ball angle*/
/*The robot follows the ball in its own axis with a PID controller for the track of the ball*/

#include <Arduino.h>
#include <Motors.h>
#include <typeinfo>

#define PIN_SERIAL1_TX (0u)
#define PIN_SERIAL1_RX (1u)

uint8_t motor4In1 = 23;
uint8_t motor4In2 = 22;
uint8_t motor4PWM = 15;

uint8_t motor3In1 = 25;
uint8_t motor3In2 = 6;
uint8_t motor3PWM = 20;

uint8_t motorIn1 = 8;
uint8_t motorIn2 = 9;
uint8_t motorPWM = 10;

uint8_t motor2In1 = 14;
uint8_t motor2In2 = 11;
uint8_t motor2PWM = 21;

const uint8_t receive_bno = 's';
const uint8_t receive_cam = 'c';
const uint8_t receive_distance = 'd';
float angle = 0;
float cam_angle = 0;
float distance = 0;

//PID
double kp = 1.05;
double ki = 2.8;
double kd = 0.38;
double target_angle = 0;
double last_error = 0;
double all_error = 0;
double max_error = 30;

//Sampling Time
double last_time = 0;
double current_time = 0;
double delta_time = 0;

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
    Serial.println("started");
    Serial1.begin(9600);
    Serial.println("Hello World");
}

void loop()
{
    // Receive Data
    angle = receive(receive_bno);
    cam_angle = receive(receive_cam);
    distance = receive(receive_distance);
    //Serial.print(angle);
    //Serial.print(" ");
    Serial.print(cam_angle);
    //Serial.print(" ");
    //Serial.println(distance);

    if (cam_angle == 0){

    }
    else if (cam_angle < 180)
    {
        cam_angle = -cam_angle;
    }
    else if (cam_angle > 180)
    {
        cam_angle = 360 - cam_angle;
    }

    Serial.print(" "); Serial.println(cam_angle);

   // PID & Motor Control
    current_time = millis();
    delta_time = current_time - last_time;
    double new_error = target_angle - cam_angle;
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
    double speed = proportional + integer + derivative;
    last_error = new_error;
    last_time = current_time;

    //
    if (speed < 0)
    {
        speed = abs(speed);
        myMotors.setSpeed(motorPWM, speed);
        myMotors.setSpeed(motor2PWM, speed);
        myMotors.setSpeed(motor3PWM, speed);
        myMotors.setSpeed(motor4PWM, speed);
        myMotors.moveRight();
        Serial.println("Right");
    }
    else if (speed > 0)
    {
        myMotors.setSpeed(motorPWM, speed);
        myMotors.setSpeed(motor2PWM, speed);
        myMotors.setSpeed(motor3PWM, speed);
        myMotors.setSpeed(motor4PWM, speed);
        myMotors.moveLeft();
        Serial.print("Left  ");
    }
    else
    {
        myMotors.setSpeed(motorPWM, speed);
        myMotors.setSpeed(motor2PWM, speed);
        myMotors.setSpeed(motor3PWM, speed);
        myMotors.setSpeed(motor4PWM, speed);
        myMotors.stopMotors();
        Serial.println("Stop");
    }
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

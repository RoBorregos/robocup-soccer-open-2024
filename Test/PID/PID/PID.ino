/*Test of PID variables*/
/*The robot moves on its own axis*/

#include <Arduino.h>
#include <motors.h>
#include <typeinfo>
#include <PID.h>
#include <serial.h>
#include <constants.h>


SerialCommunication serialComm(Serial1);

float angle = 0;
float cam_angle = 0;
float distance = 0;

// Parameters include kp, ki, kd, max_error
PID pid(1.05, 3.2, 0.2, 1000);

double target_angle = 0;
double last_error = 0;
double all_error = 0;

// Sampling Time
double last_time = 0;
double current_time = 0;
double delta_time = 0;

Motors myMotors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);

void setup()
{
    myMotors.InitializeMotors();
    Serial.begin(115200);
    while (!Serial && millis() < 10000UL)
        ;
    Serial.println("started");
    Serial1.begin(9600);
}

void loop()
{
    // Receive Data using serial class encapsulation
    angle = serialComm.Receive(RECEIVE_BNO);
    cam_angle = serialComm.Receive(RECEIVE_CAM);
    distance = serialComm.Receive(RECEIVE_DISTANCE);
    Serial.print(angle);
    Serial.print(" ");
    Serial.print(cam_angle);
    Serial.print(" ");
    Serial.println(distance);

    // Method receives a setpoint and an input to produce an output
    double speed = pid.Calculate(target_angle, angle);

    if (speed > 0)
    {
        myMotors.SetAllSpeeds(speed);
        myMotors.MoveRight();
        Serial.println("Right");
    }
    else if (speed < 0)
    {
        speed = abs(speed);
        myMotors.SetAllSpeeds(speed);
        myMotors.MoveLeft();
        Serial.print("Left");
    }
    else
    {
        myMotors.SetAllSpeeds(speed);
        myMotors.StopMotors();
        Serial.println("Stop");
    }
    delay(20);
}



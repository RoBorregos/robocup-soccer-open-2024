#include <iostream>
#include <Wire.h>
#include "Bno.h"
#include <Pixy2SPI_SS.h>
#include <Arduino.h>
#include "PID.h"
#include "Motors.h"
#include <typeinfo>
#include <Servo.h>

float angle = 0; 

BNO055 myBNO;
Motors myMotors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);

PID pid_w(1, 0, 0, 200);
const int kicker = 32;

// Define the pin for the ESC signal wire
Servo esc;
const int escPin = 6;

// Define speed levels
const int minSpeed = 1000; // Minimum speed (1000 microseconds)
const int midSpeed = 1500; // Mid speed (1500 microseconds)
const int maxSpeed = 2000; // Maximum speed (2000 microseconds)

// Define delay times
const int delayTime = 3000; // Delay time in milliseconds

void setup() {
    Serial.begin(9600);
    esc.attach(escPin);
    myMotors.InitializeMotors();
    myBNO.InitializeBNO();
    pinMode(kicker, OUTPUT);
    analogReadResolution(12);
    Serial.print("INIT");
    esc.writeMicroseconds(minSpeed);
    Serial.println("Arming ESC...");

    // Wait for a few seconds to allow the ESC to arm
    delay(3000);

    // Confirm ESC is armed
    Serial.println("ESC Armed. Ready to control the motor.");
}

void loop() {
    myBNO.GetBNOData();
    angle = myBNO.GetYaw();
    
    if (Serial.available() > 0) {
        String inputString = Serial.readString();
        int inputValue = inputString.toInt();
        
        if (inputValue >= minSpeed && inputValue <= maxSpeed) {
            esc.writeMicroseconds(inputValue);
            Serial.print("ESC speed set to: ");
            Serial.println(inputValue);
        }
        // Check for 'a' input to activate kicker
        else if (inputValue == 1) {
            esc.writeMicroseconds(1000);
            delay(20);
            digitalWrite(kicker, HIGH);
            delay(20);
            digitalWrite(kicker, LOW);
        }
        else {
            Serial.println("Invalid speed. Please enter a value between 1000 and 2000.");
        }
    }

    double speed_w = pid_w.Calculate(0, angle);
    if (speed_w != 0) {
        myMotors.MoveMotorsImu(0, 0, speed_w);
    }
}
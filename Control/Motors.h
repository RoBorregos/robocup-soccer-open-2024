#ifndef Motors_h
#define Motors_h

#pragma once
#include "Arduino.h"
#include "Motor.h"

class Motors{
    public: 
        Motor motor1;
        Motor motor2;
        Motor motor3;
        Motor motor4;

        Motors(uint8_t speed1, uint8_t in1_1, uint8_t in2_1, uint8_t stby1, uint8_t encoder1, uint8_t speed2, uint8_t in1_2, uint8_t in2_2, uint8_t stby2, uint8_t encoder2, uint8_t speed3, uint8_t in1_3, uint8_t in2_3, uint8_t stby3, uint8_t encoder3, uint8_t speed4, uint8_t in1_4, uint8_t in2_4, uint8_t stby4, uint8_t encoder4);
        void InitializeMotors();
        void InitializeDriver();
        void setSpeed(uint8_t pwm, uint8_t speed);
        void stopMotors();
        void moveForward();
        void moveBackward();
        void moveMotor1();
        void moveMotor2();
        void moveMotor3();
        void moveMotor4();
        void moveMotors(int degree, uint8_t speed);
        void individualMotor(uint8_t motor, uint8_t speed);
        void getAllSpeeds();
        void moveOneMotor(uint8_t motor, uint8_t speed);
        void moveMotorsImu(double targetAngle, uint8_t speed);
        void moveMotorsProportional(double targetAngle, uint8_t setpoint);
        
};

#endif



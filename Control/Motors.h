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

        Motors(int speed1, int in1_1, int in2_1, int stby1, int speed2, int in1_2, int in2_2, int stby2, int speed3, int in1_3, int in2_3, int stby3, int speed4, int in1_4, int in2_4, int stby4);
        void InitializeMotors();
        void InitializeDriver();
        void setSpeed(int pwm, int speed);
        void stopMotors();
        void moveForward();
        void moveBackward();
        void moveMotor1();
        void moveMotor2();
        void moveMotor3();
        void moveMotor4();
        void moveMotors(int degree, int speed);
        
};

#endif



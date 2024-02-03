#ifndef Motor_h
#define Motor_h

#include <Arduino.h>
#include "Encoder.h"
#pragma once

class Motor {
    public:
        Motor(int encoderPin);
        void set(int encoderPin, int speed, int in1, int in2, int stby);
        void InitializeMotor();
        void moveForward();
        void moveBackward();
        void stopMotor();
        void InitializeDriver();
        void setSpeed(int pwm, int speed);
        int getSpeed();
        int getIn1();
        int getIn2();
        void moveMotor1();
        void moveMotor2();
        void moveMotor3();
        void moveMotor4();
        float getRPM();
        float getAngularVelocity();
        float getDegree();


    private:
        int _speed;
        int _in1;
        int _in2;
        int _stby;
        Encoder _encoder;
    };

#endif

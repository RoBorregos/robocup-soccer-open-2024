#ifndef Motor_h
#define Motor_h

#include <Arduino.h>
#include "Encoder.h"
#pragma once

class Motor {
    public:
        Motor(uint8_t encoderPin, uint8_t speed, uint8_t in1, uint8_t in2, uint8_t stby);
        void InitializeMotor();
        void moveForward();
        void moveBackward();
        void stopMotor();
        void InitializeDriver();
        void setSpeed(uint8_t pwm, uint8_t speed);
        uint8_t getSpeed();
        uint8_t getIn1();
        uint8_t getIn2();
        void moveMotor1();
        void moveMotor2();
        void moveMotor3();
        void moveMotor4();
        float getRPM();
        float getAngularVelocity();
        float getDegree();


    private:
        uint8_t _encoder;
        uint8_t _speed;
        uint8_t _in1;
        uint8_t _in2;
        uint8_t _stby;
        Encoder _encoder;
    };

#endif

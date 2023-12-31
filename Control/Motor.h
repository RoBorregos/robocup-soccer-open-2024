#ifndef Motor_h
#define Motor_h
#pragma once

class Motor {
    public:
        Motor();
        void set(int speed, int in1, int in2);
        void InitializeMotor();
        void moveForward();
        void moveBackward();
        void stopMotor();
        int getSpeed();
        int getIn1();
        int getIn2();

    private:
        int _speed;
        int _in1;
        int _in2;
    };

#endif

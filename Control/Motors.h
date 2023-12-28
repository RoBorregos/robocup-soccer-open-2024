#ifndef MOTORS_H
#define MOTORS_H

#include "Arduino.h"
#include "stdint.h"
#include "math.h"
#undef B1

class Motors
{ 
    public:
        // Variables for Omni with 4 wheels
        bool Omni4;
        int MotorLeftF1;
        int MotorLeftF2;
        int MotorRightF1;
        int MotorRightF2;
        int MotorRightB1;
        int MotorRightB2;
        int MotorLeftB1;
        int MotorLeftB2;
        
        
        Motors(int A1, int A2, int B1, int B2, int C1, int C2, int D1, int D2);
        // Constructor for Omni4.
        void InitializeMotors();
        // Include in setup ti Initialize Motors.
        void turnLeft(int speed);
        // Turn Left at certain speed.
        void turnRight(int speed);
        // Turn Right at certain speed.
        void stopMotors();
        // Stop Motors.
        void moveMotors(int degree, int speed);
        // Move motors at certain degree with certain speed. 
};

#endif
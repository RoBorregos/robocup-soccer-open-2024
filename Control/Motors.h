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

        Motors(int speed1, int in1_1, int in2_1, int speed2, int in1_2, int in2_2, int speed3, int in1_3, int in2_3, int speed4, int in1_4, int in2_4);
        void InitializeMotors();
        void stopMotors();
        void moveForward();
        void moveBackward();
        void moveMotor1();
        void moveMotor2();
        void moveMotor3();
        void moveMotor4();
        void moveMotors(int degree, int speed);
        
}

#endif


/*#ifndef MOTORS_H
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
        int MotorLeftF_PWM;
        int MotorLeftF_CNT;
        int MotorRightF1;
        int MotorRightF2;
        int MotorRightF_PWM;
        int MotorRightF_CNT;
        int MotorRightB1;
        int MotorRightB2;
        int MotorRightB_PWM;
        int MotorRightB_CNT;
        int MotorLeftB1;
        int MotorLeftB2;
        int MotorLeftB_PWM;
        int MotorLeftB_CNT;
        
        
        Motors(int A1, int A2, int A3, int A4, int B1, int B2, int B3, int B4, int C1, int C2, int C3, int C4, int D1, int D2, int D3, int D4);
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

*/;
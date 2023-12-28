#include "Motors.h"


Motors::Motors(int A1, int A2, int A3, int A4, int B1, int B2, int B3, int B4, int C1, int C2, int C3, int C4, int D1, int D2, int D3, int D4)
{
    Omni4 = true;

    //First set of motors
    MotorLeftF1 = A1;
    MotorLeftF2 = A2;
    MotorLeftF_PWM = A3;
    MotorLeftF_CNT = A4;

    //Second set of motors
    MotorRightF1 = B1;
    MotorRightF2 = B2;
    MotorRightF_PWM = B3;
    MotorRightF_CNT = B4;

    //Third set of motors
    MotorRightB1 = C1; 
    MotorRightB2 = C2;
    MotorRightB_PWM = C3;
    MotorRightB_CNT = C4;

    //Fourth set of motors
    MotorLeftB1 = D1;
    MotorLeftB2 = D2;
    MotorLeftB_PWM = D3;
    MotorLeftB_CNT = D4;
} 

void Motors::InitializeMotors()
{
    if (Omni4){
        //Set motor pins as outputs
        pinMode(MotorLeftF1, OUTPUT);
        pinMode(MotorLeftF2, OUTPUT);
        pinMode(MotorLeftF_PWM, OUTPUT);
        pinMode(MotorLeftF_CNT, OUTPUT);
        pinMode(MotorRightF1, OUTPUT);
        pinMode(MotorRightF2, OUTPUT);
        pinMode(MotorRightF_PWM, OUTPUT);
        pinMode(MotorRightF_CNT, OUTPUT);
        pinMode(MotorRightB1, OUTPUT);
        pinMode(MotorRightB2, OUTPUT);
        pinMode(MotorRightB_PWM, OUTPUT);
        pinMode(MotorRightB_CNT, OUTPUT);
        pinMode(MotorLeftB1, OUTPUT);
        pinMode(MotorLeftB2, OUTPUT);
        pinMode(MotorLeftB_PWM, OUTPUT);
        pinMode(MotorLeftB_CNT, OUTPUT);

        //Enable the motor driver
        digitalWrite(MotorLeftF_CNT, HIGH);
        digitalWrite(MotorRightF_CNT, HIGH);
        digitalWrite(MotorRightB_CNT, HIGH);
        digitalWrite(MotorLeftB_CNT, HIGH);

        //Set PWM value (0 to 255)
        analogWrite(MotorLeftF_PWM, 128); 
        analogWrite(MotorRightF_PWM, 128);
        analogWrite(MotorRightB_PWM, 128); 
        analogWrite(MotorLeftB_PWM, 128); 
    }
}

void Motors::turnLeft(int speed)
{
    if (Omni4){
        analogWrite(MotorLeftF1, speed);
        analogWrite(MotorLeftF2, 0);
        analogWrite(MotorRightF1, speed);
        analogWrite(MotorRightF2, 0);
        analogWrite(MotorRightB1, speed);
        analogWrite(MotorRightB2, 0);
        analogWrite(MotorLeftB1, speed);
        analogWrite(MotorLeftB2, 0);
    }
}

void Motors::turnRight(int speed)
{
    if (Omni4){
        analogWrite(MotorLeftF1, 0);
        analogWrite(MotorLeftF2, speed);
        analogWrite(MotorRightF1, 0);
        analogWrite(MotorRightF2, speed);
        analogWrite(MotorRightB1, 0);
        analogWrite(MotorRightB2, speed);
        analogWrite(MotorLeftB1, 0);
        analogWrite(MotorLeftB2, speed); 
    }
}

void Motors::stopMotors()
{
    
    if(Omni4){
        analogWrite(MotorLeftF1, 0);
        analogWrite(MotorLeftF2, 0);
        analogWrite(MotorRightF1, 0);
        analogWrite(MotorRightF2, 0);
        analogWrite(MotorRightB1, 0);
        analogWrite(MotorRightB2, 0);
        analogWrite(MotorLeftB1, 0);
        analogWrite(MotorLeftB2, 0);
    }
}

void Motors::moveMotors(int degree, int speed)
{
    if(Omni4){
        float m1 = -cos(((45-degree) * M_PI / 180));
        float m2 = -cos(((135-degree) * M_PI / 180));
        float m3 = -cos(((225-degree) * M_PI / 180));
        float m4 = -cos(((315-degree) * M_PI / 180));
        int speedA = int(m1*speed);
        int speedB = int(m2*speed);
        int speedC = int(m3*speed);
        int speedD = int(m4*speed);

        if (m1 >= 0){
            analogWrite(MotorLeftF1, speedA);
            analogWrite(MotorLeftF2, 0);
        }
        else {
            analogWrite(MotorLeftF1, 0);
            analogWrite(MotorLeftF2, -speedA);
        }
        if (m2 >= 0){
            analogWrite(MotorRightF1, speedB);
            analogWrite(MotorRightF2, 0);
        }
        else {
            analogWrite(MotorRightF1, 0);
            analogWrite(MotorRightF2, -speedB);
        }
        if (m4 >= 0){
            analogWrite(MotorLeftB1, speedD);
            analogWrite(MotorLeftB2, 0);
        }
        else {
            analogWrite(MotorLeftB1, 0);
            analogWrite(MotorLeftB2, -speedD);
        }
    }
}
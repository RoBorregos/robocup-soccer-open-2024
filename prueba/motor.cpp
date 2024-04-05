#include "Arduino.h"
#include "motor.h"

Motor::Motor(uint8_t speed, uint8_t in1, uint8_t in2)
{
    speed_ = speed;
    in1_ = in1;
    in2_ = in2;
}

void Motor::InitializeMotor()
{
    pinMode(in1_, OUTPUT);
    pinMode(in2_, OUTPUT);
    pinMode(speed_, OUTPUT);
};

void Motor::SetSpeed(uint8_t pwm, uint8_t speed)
{
    analogWrite(pwm, speed);
};

void Motor::MoveForward()
{
    digitalWrite(in1_, HIGH);
    digitalWrite(in2_, LOW);
};

void Motor::MoveBackward()
{
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, HIGH);
};

void Motor::StopMotor()
{
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);
};

uint8_t Motor::GetSpeed()
{
    return speed_;
};

uint8_t Motor::GetIn1()
{
    return in1_;
};

uint8_t Motor::GetIn2()
{
    return in2_;
};
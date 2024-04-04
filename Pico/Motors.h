#ifndef motors_h
#define motors_h

#include "Arduino.h"
#include "motor.h"
#include "constants.h"

class Motors
{
public:
    Motor motor1;
    Motor motor2;
    Motor motor3;
    Motor motor4;

    Motors(uint8_t speed1, uint8_t in1_1, uint8_t in2_1,
           uint8_t speed2, uint8_t in1_2, uint8_t in2_2,
           uint8_t speed3, uint8_t in1_3, uint8_t in2_3,
           uint8_t speed4, uint8_t in1_4, uint8_t in2_4);

    void InitializeMotors();
    void InitializeDriver();
    void SetSpeed(uint8_t pwm, uint8_t speed);
    void SetAllSpeeds(uint8_t speed);
    void StopMotors();
    void MoveForward();
    void MoveBackward();
    void MoveRight();
    void MoveLeft();
    void MoveMotor1();
    void MoveMotor2();
    void MoveMotor3();
    void MoveMotor4();
    void MoveMotors(int degree, uint8_t speed);
    void IndividualMotor(uint8_t motor, uint8_t speed);
    void GetAllSpeeds();
    void MoveOneMotor(uint8_t motor, uint8_t speed);
    void MoveMotorsImu(double target_angle, uint8_t speed, double speed_w);
    void MoveMotorsProportional(double target_angle, uint8_t setpoint);
};

#endif
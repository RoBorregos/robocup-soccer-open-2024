#include "motors.h"
#include "Arduino.h"
#include "constants.h"

Motors::Motors(uint8_t speed1, uint8_t in1_1, uint8_t in2_1, uint8_t speed2, uint8_t in1_2, uint8_t in2_2, uint8_t speed3, uint8_t in1_3, uint8_t in2_3, uint8_t speed4, uint8_t in1_4, uint8_t in2_4) 
: motor1(speed1, in1_1, in2_1),
  motor2(speed2, in1_2, in2_2),
  motor3(speed3, in1_3, in2_3),
  motor4(speed4, in1_4, in2_4)
{};

void Motors::InitializeMotors()
{
    motor1.InitializeMotor();
    motor2.InitializeMotor();
    motor3.InitializeMotor();
    motor4.InitializeMotor();
};

void Motors::InitializeDriver()
{
    motor1.InitializeDriver();
    motor2.InitializeDriver();
    motor3.InitializeDriver();
    motor4.InitializeDriver();
};

void Motors::SetSpeed(uint8_t pwm, uint8_t speed)
{
    motor1.SetSpeed(pwm, speed);
    motor2.SetSpeed(pwm, speed);
    motor3.SetSpeed(pwm, speed);
    motor4.SetSpeed(pwm, speed);
};

void Motors::SetAllSpeeds(uint8_t speed)
{
    motor1.SetSpeed(MOTOR1_PWM, speed);
    motor2.SetSpeed(MOTOR2_PWM, speed);
    motor3.SetSpeed(MOTOR3_PWM, speed);
    motor4.SetSpeed(MOTOR4_PWM, speed);
};

void Motors::GetAllSpeeds()
{
    Serial.print("Motor 1: ");
    Serial.println(motor1.GetSpeed());
    Serial.print("Motor 2: ");
    Serial.println(motor2.GetSpeed());
    Serial.print("Motor 3: ");
    Serial.println(motor3.GetSpeed());
    Serial.print("Motor 4: ");
    Serial.println(motor4.GetSpeed());
};

void Motors::StopMotors()
{
    motor1.StopMotor();
    motor2.StopMotor();
    motor3.StopMotor();
    motor4.StopMotor();
};

void Motors::MoveForward()
{
    motor1.MoveForward();
    motor2.MoveForward();
    motor3.MoveForward();
    motor4.MoveForward();
};

void Motors::MoveRight()
{
    StopMotors();
    motor1.MoveForward();
    motor2.MoveBackward();
    motor3.MoveBackward();
    motor4.MoveBackward();
};

void Motors::MoveLeft()
{
    StopMotors();
    motor1.MoveBackward();
    motor2.MoveForward();
    motor3.MoveForward();
    motor4.MoveForward();
};

void Motors::MoveBackward()
{
    StopMotors();
    motor2.MoveBackward();
    motor3.MoveForward();
};

void Motors::MoveMotor1()
{
    motor1.MoveForward();
};

void Motors::MoveMotor2()
{
    motor2.MoveForward();
};

void Motors::MoveMotor3()
{
    motor3.MoveForward();
};

void Motors::MoveMotor4()
{
    motor4.MoveForward();
};

void Motors::MoveMotors(int degree, uint8_t speed)
{
    float m1 = cos(((45 + degree) * PI / 180));
    float m2 = cos(((135 + degree) * PI / 180));
    float m3 = cos(((225 + degree) * PI / 180));
    float m4 = cos(((315 + degree) * PI / 180));
    int speedA = abs(int(m1 * speed));
    int speedB = abs(int(m2 * speed));
    int speedC = abs(int(m3 * speed));
    int speedD = abs(int(m4 * speed));

    analogWrite(motor1.GetSpeed(), speedA);
    analogWrite(motor2.GetSpeed(), speedB);
    analogWrite(motor3.GetSpeed(), speedC);
    analogWrite(motor4.GetSpeed(), speedD);

    if (m1 >= 0)
    {
        motor1.MoveForward();
    }
    else
    {
        motor1.MoveBackward();
    }
    if (m2 >= 0)
    {
        motor2.MoveForward();
    }
    else
    {
        motor2.MoveBackward();
    }
    if (m3 >= 0)
    {
        motor3.MoveForward();
    }
    else
    {
        motor3.MoveBackward();
    }
    if (m4 >= 0)
    {
        motor4.MoveForward();
    }
    else
    {
        motor4.MoveBackward();
    }
};

void Motors::MoveMotorsImu(double degree, uint8_t speed, double speed_w)
{
    float m2 = cos(((45 + degree) * PI / 180)) * speed + speed_w;
    float m3 = cos(((135 + degree) * PI / 180)) * speed + speed_w;
    float m4 = cos(((225 + degree) * PI / 180)) * speed + speed_w;
    float m1 = cos(((315 + degree) * PI / 180)) * speed + speed_w;
    int speedA = abs(int(m1));
    int speedB = abs(int(m2));
    int speedC = abs(int(m3));
    int speedD = abs(int(m4));



    analogWrite(motor1.GetSpeed(), speedA);
    analogWrite(motor2.GetSpeed(), speedB);
    analogWrite(motor3.GetSpeed(), speedC);
    analogWrite(motor4.GetSpeed(), speedD);

    if (m1 >= 0)
    {
        motor1.MoveForward();
    }
    else
    {
        motor1.MoveBackward();
    }
    if (m2 >= 0)
    {
        motor2.MoveForward();
    }
    else
    {
        motor2.MoveBackward();
    }
    if (m3 >= 0)
    {
        motor3.MoveForward();
    }
    else
    {
        motor3.MoveBackward();
    }
    if (m4 >= 0)
    {
        motor4.MoveForward();
    }
    else
    {
        motor4.MoveBackward();
    }
};
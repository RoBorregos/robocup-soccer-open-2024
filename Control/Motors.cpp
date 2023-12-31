#include "Motors.h"
#include "Arduino.h"

Motors::Motors(int speed1, int in1_1, int in2_1, int stby1, int speed2, int in1_2, int in2_2, int stby2, int speed3, int in1_3, int in2_3, int stby3, int speed4, int in1_4, int in2_4, int stby4) {
    motor1.set(speed1, in1_1, in2_1, stby1);
    motor2.set(speed2, in1_2, in2_2, stby2);
    motor3.set(speed3, in1_3, in2_3, stby3);
    motor4.set(speed4, in1_4, in2_4, stby4);
};

void Motors::InitializeMotors() {
    motor1.InitializeMotor();
    motor2.InitializeMotor();
    motor3.InitializeMotor();
    motor4.InitializeMotor();
};

void Motors::InitializeDriver() {
    motor1.InitializeDriver();
    motor2.InitializeDriver();
    motor3.InitializeDriver();
    motor4.InitializeDriver();
};

void Motors::setSpeed(int pwm, int speed) {
    motor1.setSpeed(pwm, speed);
    motor2.setSpeed(pwm, speed);
    motor3.setSpeed(pwm, speed);
    motor4.setSpeed(pwm, speed);
};

void Motors::stopMotors() {
    motor1.stopMotor();
    motor2.stopMotor();
    motor3.stopMotor();
    motor4.stopMotor();
};

void Motors::moveForward() {
    stopMotors();
    motor2.moveForward();
    motor3.moveForward();
};

void Motors::moveBackward() {
    stopMotors();
    motor2.moveBackward();
    motor3.moveBackward();
};

void Motors::moveMotor1() {
    motor1.moveForward();
};

void Motors::moveMotor2() {
    motor2.moveForward();
};

void Motors::moveMotor3() {
    motor3.moveForward();
};

void Motors::moveMotor4() {
    motor4.moveForward();
};

void Motors::moveMotors(int degree, int speed) {
    float m1 = -cos(((45-degree) * M_PI / 180));
    float m2 = -cos(((135-degree) * M_PI / 180));
    float m3 = -cos(((225-degree) * M_PI / 180));
    float m4 = -cos(((315-degree) * M_PI / 180));
    int speedA = int(m1*speed);
    int speedB = int(m2*speed);
    int speedC = int(m3*speed);
    int speedD = int(m4*speed);

    if (m1 >= 0){
        motor1.setSpeed(motor1.getSpeed(), speedA);
        motor1.moveForward();
    }
    else {
        motor1.setSpeed(motor1.getSpeed(), -1*speedA);
        motor1.moveBackward();
    }
    if (m2 >= 0){
        motor2.setSpeed(motor2.getSpeed(), speedB);
        motor2.moveForward();
    }
    else {
        motor2.setSpeed(motor2.getSpeed(), -1*speedB);
        motor2.moveBackward();
    }
    if (m3 >= 0){
        motor3.setSpeed(motor3.getSpeed(), speedC);
        motor3.moveForward();
    }
    else {
        motor3.setSpeed(motor3.getSpeed(), -1*speedC);
        motor3.moveBackward();
    }
    if (m4 >= 0){
        motor4.setSpeed(motor4.getSpeed(), speedD);
        motor4.moveForward();
    }
    else {
        motor4.setSpeed(motor4.getSpeed(), -1*speedD);
        motor4.moveBackward();
    }
};
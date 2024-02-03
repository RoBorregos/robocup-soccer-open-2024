#include "Motors.h"
#include "Arduino.h"

Motors::Motors(int speed1, int in1_1, int in2_1, int stby1, int encoder1, int speed2, int in1_2, int in2_2, int stby2, int encoder2, int speed3, int in1_3, int in2_3, int stby3, int encoder3, int speed4, int in1_4, int in2_4, int stby4, int encoder4) {
    motor1.set(encoder1, speed1, in1_1, in2_1, stby1);
    motor2.set(encoder2, speed2, in1_2, in2_2, stby2);
    motor3.set(encoder3, speed3, in1_3, in2_3, stby3);
    motor4.set(encoder4, speed4, in1_4, in2_4, stby4);
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

void Motors::getAllSpeeds() {
    Serial.print("Motor 1: ");
    Serial.println(motor1.getSpeed());
    Serial.print("Motor 2: ");
    Serial.println(motor2.getSpeed());
    Serial.print("Motor 3: ");
    Serial.println(motor3.getSpeed());
    Serial.print("Motor 4: ");
    Serial.println(motor4.getSpeed());
};

void Motors::stopMotors() {
    motor1.stopMotor();
    motor2.stopMotor();
    motor3.stopMotor();
    motor4.stopMotor();
};

void Motors::moveForward() {
    motor1.moveForward();
    motor1.stopMotor();
    motor2.moveForward();
    motor2.stopMotor();
    motor3.moveForward();
    motor3.stopMotor();
    motor4.moveForward();
    motor4.stopMotor();
};

void Motors::moveForward() {
    stopMotors();
    motor2.moveForward();
    motor3.moveBackward();
};

void Motors::moveBackward() {
    stopMotors();
    motor2.moveBackward();
    motor3.moveForward();
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

// kinematic equations for robot movement
void Motors::moveMotors(int degree, int speed) {
    float m1 = cos(((45+degree) * PI / 180));
    float m2 = cos(((135+degree) * PI / 180));
    float m3 = cos(((225+degree) * PI / 180));
    float m4 = cos(((315+degree) * PI / 180));
    int speedA = abs(int(m1*speed));
    int speedB = abs(int(m2*speed));
    int speedC = abs(int(m3*speed));
    int speedD = abs(int(m4*speed));

    analogWrite(motor1.getSpeed(), speedA);
    analogWrite(motor2.getSpeed(), speedB);
    analogWrite(motor3.getSpeed(), speedC);
    analogWrite(motor4.getSpeed(), speedD);

    if (m1 >= 0){
        motor1.moveForward();
    }
    else {
        motor1.moveBackward();
    }
    if (m2 >= 0){
        motor2.moveForward();
    }
    else {
        motor2.moveBackward();
    }
    if (m3 >= 0){
        motor3.moveForward();
    }
    else {
        motor3.moveBackward();
    }
    if (m4 >= 0){
        motor4.moveForward();
    }
    else {
        motor4.moveBackward();
    }
};

void Motors::moveMotorsImu(double degree, double speed) {
    float m1 = cos(((45+degree) * PI / 180));
    float m2 = cos(((135+degree) * PI / 180));
    float m3 = cos(((225+degree) * PI / 180));
    float m4 = cos(((315+degree) * PI / 180));
    int speedA = abs(int(m1*speed));
    int speedB = abs(int(m2*speed));
    int speedC = abs(int(m3*speed));
    int speedD = abs(int(m4*speed));

    analogWrite(motor1.getSpeed(), speedA);
    analogWrite(motor2.getSpeed(), speedB);
    analogWrite(motor3.getSpeed(), speedC);
    analogWrite(motor4.getSpeed(), speedD);

    if (m1 >= 0){
        motor1.moveForward();
    }
    else {
        motor1.moveBackward();
    }
    if (m2 >= 0){
        motor2.moveForward();
    }
    else {
        motor2.moveBackward();
    }
    if (m3 >= 0){
        motor3.moveForward();
    }
    else {
        motor3.moveBackward();
    }
    if (m4 >= 0){
        motor4.moveForward();
    }
    else {
        motor4.moveBackward();
    }
};

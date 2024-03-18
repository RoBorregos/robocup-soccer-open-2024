#include "Motors.h"
//#include "PID.h"
#include "Arduino.h"

Motors::Motors(uint8_t speed1, uint8_t in1_1, uint8_t in2_1, uint8_t speed2, uint8_t in1_2, uint8_t in2_2, uint8_t speed3, uint8_t in1_3, uint8_t in2_3, uint8_t speed4, uint8_t in1_4, uint8_t in2_4) 
: motor1(speed1, in1_1, in2_1),
  motor2(speed2, in1_2, in2_2),
  motor3(speed3, in1_3, in2_3),
  motor4(speed4, in1_4, in2_4)
{};

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

void Motors::setSpeed(uint8_t pwm, uint8_t speed) {
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

/*void Motors::moveForward() {
    stopMotors();
    motor2.moveForward();
    motor3.moveBackward();
};*/

void Motors::moveRight() {
    stopMotors();
    motor1.moveForward();
    motor2.moveBackward();
    motor3.moveBackward();
    motor4.moveForward();
};

void Motors::moveLeft() {
    stopMotors();
    motor1.moveBackward();
    motor2.moveForward();
    motor3.moveForward();
    motor4.moveBackward();
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
void Motors::moveMotors(int degree, uint8_t speed) {
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

void Motors::moveMotorsImu(double degree, uint8_t speed) {
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

/*double KP = 1.0;
PID pid1(KP);
PID pid2(KP);
PID pid3(KP);
PID pid4(KP);

void Motors::moveMotorsProportional(double degree, uint8_t setpoint) {
    float m1 = cos(((45+degree) * PI / 180));
    float m2 = cos(((135+degree) * PI / 180));
    float m3 = cos(((225+degree) * PI / 180));
    float m4 = cos(((315+degree) * PI / 180));

    pid1.setSetpoint(setpoint);
    pid2.setSetpoint(setpoint);
    pid3.setSetpoint(setpoint);
    pid4.setSetpoint(setpoint);

    int speedA = pid1.computeP(abs(int(m1*setpoint)));
    int speedB = pid2.computeP(abs(int(m2*setpoint)));
    int speedC = pid3.computeP(abs(int(m3*setpoint)));
    int speedD = pid4.computeP(abs(int(m4*setpoint)));

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
}
*/
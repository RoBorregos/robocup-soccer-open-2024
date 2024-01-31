#include "Arduino.h"
#include "Motor.h"
#include "Encoder.h"


void Motor::set(int encoderPin, int speed, int in1, int in2, int stby) {
    _encoder = Encoder(encoderPin);
    _speed = speed;
    _in1 = in1;
    _in2 = in2;
    _stby = stby;
};

void Motor::InitializeMotor() {
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    pinMode(_speed, OUTPUT);
    pinMode(_stby, OUTPUT);
    _encoder.initialize();
};

void Motor::InitializeDriver() {
    digitalWrite(_stby, HIGH);
};

void Motor::setSpeed(int pwm, int speed) {
    analogWrite(pwm, speed);
};

void Motor::moveForward() {
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, LOW);
};

void Motor::moveBackward() {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, HIGH);
};

void Motor::stopMotor() {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
};


int Motor::getSpeed() {
    return _speed;
};

int Motor::getIn1() {
    return _in1;
};

int Motor::getIn2() {
    return _in2;
};

float Motor::getRPM() {
    _encoder.update();
    return _encoder.getRPM();
}

float Motor::getAngularVelocity() {
    _encoder.update();
    return _encoder.getAngularVelocity();
}

float Motor::getDegree() {
    _encoder.update();
    return _encoder.getDegree();
}


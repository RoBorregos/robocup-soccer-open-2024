#include "Arduino.h"
#include "Motor.h"
#include "Encoder.h"


Motor::Motor(uint8_t encoderPin, uint8_t speed, uint8_t in1, uint8_t in2, uint8_t stby) : _encoder(encoderPin) {
    _speed = speed;
    _in1 = in1;
    _in2 = in2;
    _stby = stby;
}

void Motor::InitializeMotor() {
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    pinMode(_speed, OUTPUT);
    pinMode(_stby, OUTPUT);
    //_encoder.initialize();
};

void Motor::InitializeDriver() {
    digitalWrite(_stby, HIGH);
};

void Motor::setSpeed(uint8_t pwm, uint8_t speed) {
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


uint8_t Motor::getSpeed() {
    return _speed;
};


uint8_t Motor::getIn1() {
    return _in1;
};

uint8_t Motor::getIn2() {
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


#include "Encoder.h"

volatile float Encoder::_right_wheel_pulse_count = 0;

Encoder::Encoder(int pin) {
  _pin = pin;
  _previousMillis = 0;
  _currentMillis = 0;
  _interval = 1000;
  _rpm_right = 0;
  _ang_velocity_right = 0;
}

void Encoder::initialize() {
  pinMode(_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(_pin), Encoder::right_wheel_pulse, RISING);
}

void Encoder::update() {
  _currentMillis = millis();
  if (_currentMillis - _previousMillis > _interval) {
    _previousMillis = _currentMillis;
    _rpm_right = (_right_wheel_pulse_count * 60 / ENC_COUNT_REV);
    _ang_velocity_right = (_rpm_right * 2 * PI) / 60;
  }
}

float Encoder::getRPM() {
  return _rpm_right;
}

float Encoder::getAngularVelocity() {
  return _ang_velocity_right;
}

float Encoder::getDegree() {
  return (_right_wheel_pulse_count / ENC_COUNT_REV) * 360;
}

void Encoder::right_wheel_pulse() {
  _right_wheel_pulse_count++;
}

void Encoder::reset() {
  _right_wheel_pulse_count = 0;
}

float Encoder::getPulses() {
  return _right_wheel_pulse_count;
}
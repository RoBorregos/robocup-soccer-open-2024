#include "Encoder.h"

volatile float Encoder::_wheel_pulse_count = 0;

Encoder::Encoder(uint8_t pin) {
  _pin = pin;
  _previousMillis = 0;
  _currentMillis = 0;
  _interval = 1000;
}

void Encoder::initialize() {
  pinMode(_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(_pin), Encoder::_wheel_pulse, RISING);
}

void Encoder::update() {
    _rpm = (_wheel_pulse_count * 60 / ENC_COUNT_REV);
    _ang_velocity = (_rpm * 2 * PI) / 60;
}

float Encoder::getDegree() {
  return (_wheel_pulse_count / ENC_COUNT_REV) * 360;
}

float Encoder::getRPM() {
  return _rpm;
}

float Encoder::getAngularVelocity() {
  return _ang_velocity;
}

void Encoder::_wheel_pulse() {
  _wheel_pulse_count++;
}

void Encoder::reset() {
  _wheel_pulse_count = 0;
}

float Encoder::getPulses() {
  return _wheel_pulse_count;

}
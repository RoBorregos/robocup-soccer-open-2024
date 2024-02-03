#ifndef Encoder_h
#define Encoder_h
#pragma once

#include <Arduino.h>

// Encoder ticks can vary on a range of 60-66 
#define ENC_COUNT_REV 63

class Encoder {
  public:
    Encoder(int pin);
    void initialize();
    void update();
    void reset();
    float getRPM();
    float getAngularVelocity();
    float getDegree();
    float getPulses();
    static void right_wheel_pulse();

  private:
    int _pin;
    volatile static float _right_wheel_pulse_count;
    long _previousMillis;
    long _currentMillis;
    int _interval;
    float _rpm_right;
    float _ang_velocity_right;
};

#endif
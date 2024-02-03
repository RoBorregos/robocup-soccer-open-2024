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
    float getPulses();
    static void _wheel_pulse();
    float getDegree();
    float getRPM();
    float getAngularVelocity();

  private:
    int _pin;
    volatile static float _wheel_pulse_count;
    long _previousMillis;
    long _currentMillis;
    int _interval;
    float _rpm;
    float _ang_velocity;
};

#endif
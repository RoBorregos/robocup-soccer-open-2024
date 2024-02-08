#ifndef PID_h
#define PID_h
#pragma once

#include <Arduino.h>

class PID{
    public:
        PID(float kp);
        double computeP(double inp);
        void setConstants(float kp);
        void setSetpoint(float setpoint);
        void setOutputLimits(float min, float max);


    private:
        float _kp;
        float _setpoint;
        float _error;
        float _lastError;
        float _output;
        float _min;
        float _max;

};

#endif
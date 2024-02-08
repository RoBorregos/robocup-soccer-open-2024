#include "PID.h"

PID::PID(float kp){
    _kp = kp;
    _setpoint = 0;
    _error = 0;
    _lastError = 0;
    _output = 0;
    _min = 0;
    _max = 0;
}

double PID::computeP(double inp){
    _error = _setpoint - inp;
    _output = _kp * _error;
    _lastError = _error;
    return _output;
}

void PID::setConstants(float kp){
    _kp = kp;
}

void PID::setSetpoint(float setpoint){
    _setpoint = setpoint;
}

void PID::setOutputLimits(float min, float max){
    _min = min;
    _max = max;
}


#include "PID.h"
#include "Arduino.h"

PID::PID(double kp, double ki, double kd, double max_error)
    : kp_(kp), ki_(ki), kd_(kd), max_error_(max_error), last_error_(0), last_time_(millis()) {}

double PID::Calculate(double setpoint, double input) {
    unsigned long current_time = millis();
    double delta_time = (current_time - last_time_) / 1000.0;  
    double error = setpoint - input;
    double total_error = error + last_error_;
    if (total_error > max_error_) {
        total_error = max_error_;
    } else if (total_error < -max_error_) {
        total_error = -max_error_;
    }
    double proportional = kp_ * error;
    double integral = ki_ * total_error * delta_time;
    double derivative = kd_ * (error - last_error_) / delta_time;
    double output = proportional + integral + derivative;
    last_error_ = error;
    last_time_ = current_time;
    return output;
}
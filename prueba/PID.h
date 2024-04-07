#ifndef PID_h
#define PID_h

class PID {
    public:
        PID(double kp, double ki, double kd, double max_error);
        double Calculate(double setpoint, double input);

    private:
        double kp_;
        double ki_;
        double kd_;
        double max_error_;
        double last_error_;
        double last_time_;
};

#endif
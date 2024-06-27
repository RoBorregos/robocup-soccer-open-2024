#ifndef Kicker_h
#define Kicker_h

#include <Arduino.h>
#include <Servo.h>

class Kicker{
    public:
        Kicker(uint8_t pin);
        void setSpeed(int speed);
        void initialize();
        void stop();
    private:
        Servo kicker;
        uint8_t pin;
    };

#endif
#ifndef Dribbler_h
#define Dribbler_h

#include <Arduino.h>
#include <Servo.h>

class Dribbler{
    public:
        Dribbler(uint8_t pin);
        void setSpeed(int speed);
        void initialize();
        void stop();
    private:
        Servo dribbler;
        uint8_t pin;
    };
    
#endif
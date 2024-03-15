#ifndef Uart_h
#define Uart_h

#include "Arduino.h"

class Uart {
public:
    Uart();
    void initializeCommUart();
    void update();
    float getDistance();
    float getAngle();

private:
    String inputString;
    bool stringComplete;
    float distance;
    float angle;
};

#endif
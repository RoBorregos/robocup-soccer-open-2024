#ifndef serial_h
#define serial_h

#include <Arduino.h>

class SerialCommunication {
    public:
        SerialCommunication(HardwareSerial& serial);
        float Receive(uint8_t signal);

    private:
        HardwareSerial& serial_;
};

#endif

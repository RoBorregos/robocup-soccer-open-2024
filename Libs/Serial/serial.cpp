#include "serial.h"

SerialCommunication::SerialCommunication(HardwareSerial& serial) : serial_(serial) {}

// Serial comunication to receive data from any microcontroller, at the moment it is not being used, it is 
// implemented on past versions of the codebase
float SerialCommunication::Receive(uint8_t signal) {
    serial_.write(signal);
    while (!serial_.available()) {
        continue;
    }
    delay(10);
    float temp; 
    uint8_t tempArray[4];
    union u_tag {
        byte b[4];
        float value;
    } u;
    u.b[0] = serial_.read();
    u.b[1] = serial_.read();
    u.b[2] = serial_.read();
    u.b[3] = serial_.read();
    return u.value;
}
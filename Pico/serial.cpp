#include "serial.h"

SerialCommunication::SerialCommunication(HardwareSerial& serial) : serial_(serial) {}

float SerialCommunication::Receive(uint8_t signal) {
    serial_.write(signal);
    while (!serial_.available()) {
        continue;
    }
    delay(10);
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
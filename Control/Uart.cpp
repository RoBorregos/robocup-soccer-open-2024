#include "Arduino.h"
#include "Uart.h"

Uart::Uart() : stringComplete(false) {
}

void Uart::initializeCommUart() {
    Serial.begin(9600);
}

void Uart::update() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        inputString += inChar;
        if (inChar == '\n') {
            stringComplete = true;
        }
    }

    if (stringComplete) {
        sscanf(inputString.c_str(), "%f %f", &distance, &angle);
        inputString = "";
        stringComplete = false;
    }
}

float Uart::getDistance() {
    return distance;
}

float Uart::getAngle() {
    return angle;
}
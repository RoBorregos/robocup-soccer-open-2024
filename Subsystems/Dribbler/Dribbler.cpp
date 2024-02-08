#include "Arduino.h"
#include "Dribbler.h"
#include <Servo.h>

Dribbler::Dribbler(uint8_t pin){
    this->pin = pin;
}

void Dribbler::initialize(){
    dribbler.attach(pin);
    dribbler.writeMicroseconds(1500);
    delay(1000);
}

void Dribbler::setSpeed(int speed){
    dribbler.writeMicroseconds(speed);
}

void Dribbler::stop(){
    dribbler.writeMicroseconds(200);
}
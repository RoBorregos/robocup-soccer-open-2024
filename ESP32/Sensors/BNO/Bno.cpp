#include "Arduino.h"
#include "Bno.h"
#include "Uart.h"
#include "cmath"

//https://www.allaboutcircuits.com/projects/bosch-absolute-orientation-sensor-bno055/

BNO055::BNO055() {
    yaw = 0;
    target_angle = 0;
    difference_angle = 0;
}

void BNO055::InitializeBNO() {
    
    Serial.println("Initializing BNO055...");
    if(!bno.begin()) {
        Serial.println("BNO055 not detected. Check wiring or I2C ADDR!");
        while(1);
    }
    delay(1000);
    bno.setExtCrystalUse(true);
    Serial.println("BNO055 initialized.");
}

void BNO055::getBNOData() {
    Uart uart;
    uart.update();
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    yaw = euler.x();
    if(yaw > 180){
        yaw = -1 * (360 - yaw);
    }
    difference_angle = yaw - uart.getAngle();
    target_angle = 2*M_PI - difference_angle;
}

double BNO055::getYaw() {
    return yaw;
}

void BNO055::setYaw(double y) {
    yaw = y;
}

double BNO055::getTargetAngle() {
    return target_angle;
}
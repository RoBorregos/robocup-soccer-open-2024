#include "Arduino.h"
#include "BNO055.h"


//https://www.allaboutcircuits.com/projects/bosch-absolute-orientation-sensor-bno055/

BNO055::BNO055() {
    yaw = 0;
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
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    yaw = euler.x();
}

double BNO055::getYaw() {
    return yaw;
}
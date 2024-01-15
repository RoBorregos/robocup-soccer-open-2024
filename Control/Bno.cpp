#include "Arduino.h"
#include "Bno.h"

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

void BNO055::setYaw(double y) {
    yaw = y;
}

void BNO055::moveWithBNO(double targetAngle, double speed) {
    double currentAngle = getYaw();
    double angleDifference = targetAngle - currentAngle;
    if (angleDifference > 180) {
        angleDifference -= 360;
    }
    else if (angleDifference < -180) {
        angleDifference += 360;
    }
    if (angleDifference > 0) {
        //turn right
        Serial.println("Turning right");
    }
    else if (angleDifference < 0) {
        //turn left
        Serial.println("Turning left");
    }
    else {
        //do nothing
        Serial.println("Not turning");
    }
}
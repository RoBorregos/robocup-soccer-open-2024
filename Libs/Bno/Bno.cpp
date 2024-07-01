#include "Arduino.h"
#include "Bno.h"
#include "cmath"

// https://www.allaboutcircuits.com/projects/bosch-absolute-orientation-sensor-bno055/

BNO055::BNO055()
{
    yaw_ = 0;
    target_angle_ = 0;
    difference_angle_ = 0;
}

// Start retrieving data from the BNO055 sensor, disable the external crystal in case errors occur
void BNO055::InitializeBNO()
{

    Serial.println("Initializing BNO055...");
    if (!bno_.begin())
    {
        Serial.println("BNO055 not detected. Check wiring or I2C ADDR!");
        while (1)
            ;
    }
    delay(1000);
    bno_.setExtCrystalUse(true);
    Serial.println("BNO055 initialized.");
}

// Retrieve the yaw angle (x) from the BNO055 sensor using quaternion data from the euler vector
void BNO055::GetBNOData()
{
    imu::Vector<3> euler = bno_.getVector(Adafruit_BNO055::VECTOR_EULER);
    yaw_ = euler.x();
    if (yaw_ > 180)
    {
        yaw_ = -1 * (360 - yaw_);
    }
}

double BNO055::GetYaw()
{
    return yaw_;
}

void BNO055::SetYaw(double yaw)
{
    yaw = yaw_;
}
#ifndef imu_h
#define imu_h
#pragma once

#include "ICM_20948.h"

#define WIRE_PORT Wire
#define SERIAL_PORT Serial
#define AD0_VAL 9

class Imu
{
private:
    double yaw;
    ICM_20948_I2C imu;
    double yawOffset;
    double pitch;
    double roll;

public:
    Imu();
    void InitializeImu();
    void getImuData();
    double getYaw();
    void setYaw(double y);
    void moveWithImu(double targetAngle, double speed);
    double getPitch();
    double getRoll();
    void setYawOffset(double y);
};

#endif

#ifndef Imu_h
#define Imu_h

#include "ICM_20948.h"

#define WIRE_PORT Wire
#define AD0_VAL 0

class Imu{
    private:
        double yaw;
        ICM_20948_I2C imu;
        double yawOffset;
        double pitch;
        double roll;

    public: 
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

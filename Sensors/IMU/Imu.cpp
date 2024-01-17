#include "Arduino.h"
#include "Motors.h"
#include "Imu.h"

Imu::Imu()
{

    yaw = 0;
    yawOffset = 0;
    pitch = 0;
    roll = 0;
}

void Imu::InitializeImu()
{
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000); // use 400 kHz I2C

    bool initialized = false;

    // Initialize with the specified I2C port and address until successful
    while (!initialized)
    {
        imu.begin(WIRE_PORT, AD0_VAL);
        initialized = (imu.status == ICM_20948_Stat_Ok);
    }

    bool success = true; // Use success to show if the DMP configuration was successful

    // Initialization of Digital Motion Processor (DMP)
    success &= (imu.initializeDMP() == ICM_20948_Stat_Ok);

    // Enables specific game rotation vector sensor
    // Its purpose is to calculate the orientation of the device and provide this information in the form of a quaternion
    success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

    // Set the output data rate (ODR) of the sensor
    success &= (imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum

    // Enable first in first out (FIFO) buffer that stores data before being read by microcontroller
    success &= (imu.enableFIFO() == ICM_20948_Stat_Ok);

    // Responsible for offloading sensor fusion calculations from the microcontroller
    success &= (imu.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP and FIFO to ensure clean data collection
    success &= (imu.resetDMP() == ICM_20948_Stat_Ok && imu.resetFIFO() == ICM_20948_Stat_Ok);

    // Check success
    if (!success)
    {
        while (1)
            Serial.println("An error has occurred");
    }
}

void Imu::getImuData(){
    icm_20948_DMP_data_t data;
    imu.readDMPdataFromFIFO(&data);

    if((imu.status == ICM_20948_Stat_Ok) || (imu.status == ICM_20948_Stat_FIFOMoreDataAvail)){
        if(data.header & DMP_header_bitmap_Quat6){
            double q1 = (double)data.Quat6.Data.Q1 / (double)(1 << 30);
            double q2 = (double)data.Quat6.Data.Q2 / (double)(1 << 30);
            double q3 = (double)data.Quat6.Data.Q3 / (double)(1 << 30);

            double q0 = sqrt(1 - q1*q1 - q2*q2 - q3*q3);
            double qTo2 = q2*q2;

            yaw = atan2(2*(q1*q2 + q0*q3), q0*q0 + q1*q1 - qTo2 - q3*q3);
            pitch = -asin(2*(q1*q3 - q0*q2));
            roll = atan2(2*(q2*q3 + q0*q1), q0*q0 - q1*q1 - qTo2 + q3*q3);
        }
    }
}

double Imu::getYaw(){
    return yaw;
}

void Imu::setYaw(double y){
    yaw = y;
}

double Imu::getPitch(){
    return pitch;
}

double Imu::getRoll(){
    return roll;
}

void Imu::setYawOffset(double y){
    yawOffset = y;
}


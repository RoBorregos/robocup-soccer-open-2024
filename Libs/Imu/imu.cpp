#include "Arduino.h"
#include "Motors.h"
#include "imu.h"

Imu::Imu() {}

void Imu::InitializeImu()
{

    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);

    bool initialized = false;
    while (!initialized)
    {

        imu.begin(WIRE_PORT, AD0_VAL);

        if (imu.status != ICM_20948_Stat_Ok)
        {
        }
        else
        {
            initialized = true;
        }
    }

    bool success = true; // Use success to show if the DMP configuration was successful

    success &= (imu.initializeDMP() == ICM_20948_Stat_Ok);
    success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
    success &= (imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum

    // Enable the FIFO
    success &= (imu.enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (imu.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (imu.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (imu.resetFIFO() == ICM_20948_Stat_Ok);

    // Check success
    if (success)
    {
    }
    else
    {
        while (1)
            Serial.println("fail"); // Do nothing more
    }
}

void Imu::getImuData()
{
    icm_20948_DMP_data_t data;
    imu.readDMPdataFromFIFO(&data);

    if ((imu.status == ICM_20948_Stat_Ok) || (imu.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
    {

        if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
        {
            double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
            double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
            double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

            // Convert the quaternions to Euler angles (roll, pitch, yaw)
            // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

            double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

            double q2sqr = q2 * q2;

            // roll (x-axis rotation)
            double t0 = +2.0 * (q0 * q1 + q2 * q3);
            double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
            double roll = atan2(t0, t1) * 180.0 / PI;

            // pitch (y-axis rotation)
            double t2 = +2.0 * (q0 * q2 - q3 * q1);
            t2 = t2 > 1.0 ? 1.0 : t2;
            t2 = t2 < -1.0 ? -1.0 : t2;
            double pitch = asin(t2) * 180.0 / PI;

            // yaw (z-axis rotation)
            double t3 = +2.0 * (q0 * q3 + q1 * q2);
            double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
            double yaw = atan2(t3, t4) * 180.0 / PI;
            Serial.println(yaw);
        }
    }

    if (imu.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
    {
        delay(10);
    }
}

double Imu::getYaw()
{
    return yaw;
}

void Imu::setYaw(double y)
{
    yaw = y;
}

double Imu::getPitch()
{
    return pitch;
}

double Imu::getRoll()
{
    return roll;
}

void Imu::setYawOffset(double y)
{
    yawOffset = y;
}

#include "Arduino.h"
#include "Motors.h"
#include "Imu.h"

Imu::Imu()
{

}

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
  if (success) {
  } else {
    while (1)
      Serial.println("fail") ; // Do nothing more
  }
}

// Convert the quaternions to Euler angles (roll, pitch, yaw)
// https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2
void Imu::getImuData()
{
    icm_20948_DMP_data_t data;
  imu.readDMPdataFromFIFO(&data);

  if ((imu.status == ICM_20948_Stat_Ok) || (imu.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );

    if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

      // Scale to +/- 1
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

      /*
      SERIAL_PORT.print(F("Q1:"));
      SERIAL_PORT.print(q1, 3);
      SERIAL_PORT.print(F(" Q2:"));
      SERIAL_PORT.print(q2, 3);
      SERIAL_PORT.print(F(" Q3:"));
      SERIAL_PORT.println(q3, 3);
*/

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

    }}

  if (imu.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
}

//move with imu using robot movement from move robot


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

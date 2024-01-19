

#include "Imu.h"
Imu myImu;
void setup()
{
  Serial.begin(115200);
  myImu.InitializeImu();
}

void loop()
{
  myImu.getImuData();
  delay(100);
  Serial.println(myImu.getYaw());
}
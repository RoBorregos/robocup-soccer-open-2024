#ifndef Bno_h
#define Bno_h

#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_Sensor.h"

class BNO055
{
private:
    double yaw_;
    double target_angle_;
    double difference_angle_;
    Adafruit_BNO055 bno_ = Adafruit_BNO055(55, 0x28, &Wire);

public:
    BNO055();
    void InitializeBNO();
    void GetBNOData();
    double GetYaw();
    void SetYaw(double yaw);
    void MoveWithBNO(double target_angle, double speed);
};

#endif

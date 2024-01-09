#ifndef bno_h
#define bno_h

#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_Sensor.h"

class BNO055 {
    private:
    double yaw;
    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

    public:
    BNO055();
    void InitializeBNO();
    void getBNOData();
    double getYaw();
};

#endif

#ifndef bno_h
#define bno_h

#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_Sensor.h"

class bno {
    private:
    double yaw;
    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

    public:
    bno();
    void InitializeBNO();
    void getBNOData();
    double getYaw();
};

#endif

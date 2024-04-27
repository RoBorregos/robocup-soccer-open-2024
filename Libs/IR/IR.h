#ifndef IR_H
#define IR_H

#include <Adafruit_ADS1X15.h> 

class IR {
  public:
    IR();
    void begin(); 
    int readIndividualSensor(int sensor);
    int readAllSensors();
    bool isOnLine();
    String setI2CAddress(int address);
  private:
    Adafruit_ADS1115 ads;
    int threshold = 1000; 
};

#endif

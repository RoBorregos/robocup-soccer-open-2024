#ifndef Photo_H
#define Photo_H

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>

class Photo {
  public:
    Photo();
    void InitPhoto();
    void PrintSensorValues(Adafruit_ADS1115 sensor);
    int ReadSensors(Adafruit_ADS1115 ads_left, Adafruit_ADS1115 ads_right, Adafruit_ADS1115 ads_back);
    int ReadIndividualSensor(Adafruit_ADS1115 sensor, int index);
  private:
    Adafruit_ADS1115 _ads_left[3];
    Adafruit_ADS1115 _ads_right[3];
    Adafruit_ADS1115 _ads_back[3];
    int _field_threshold = 1000;
    int _line_threshold = 2000;
};

#endif






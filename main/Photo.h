#ifndef Photo_H
#define Photo_H

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

class Photo {
  private:
    Adafruit_ADS1115 ads_left;
    Adafruit_ADS1115 ads_right;
    Adafruit_ADS1115 ads_back;
    int field_threshold = 4159;
    int line_threshold = 3683;

  public:
    Photo();
    bool InitializeADS();
    double GetAverageBackValues();
    double GetAverageLeftValues();
    double GetAverageRightValues();
    boolean isOnLeftLine();
    boolean isOnRightLine();
    boolean isOnBackLine();
    void moveComplementary(boolean left, boolean right, boolean back);
    int16_t GetADSLeftIndividual(int channel);
    int16_t GetADSRightIndividual(int channel);
    int16_t GetADSBackIndividual(int channel);
};

#endif

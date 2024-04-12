

//0x48
//0x49
//0x4A
#include <Adafruit_ADS1X15.h>
#include "Photo.h"

Photo::Photo() {
}

void Photo::InitPhoto() {
  for (int i = 0; i < 3; i++) {
    _ads_left[i].begin(0x48);
    _ads_left[i].setGain(GAIN_FOUR);
  }

  for (int i = 0; i < 3; i++) {
    _ads_right[i].begin(0x49);
    _ads_right[i].setGain(GAIN_FOUR);
  }

  for (int i = 0; i < 3; i++) {
    _ads_back[i].begin(0x4A);
    _ads_back[i].setGain(GAIN_FOUR);
  }
}

void Photo::PrintSensorValues(Adafruit_ADS1115 sensor) {
  int16_t adc0 = sensor.readADC_SingleEnded(0);
  int16_t adc1 = sensor.readADC_SingleEnded(1);
  int16_t adc2 = sensor.readADC_SingleEnded(2);
  int16_t adc3 = sensor.readADC_SingleEnded(3);

  Serial.print("Channel 0: "); Serial.println(adc0);
  Serial.print("Channel 1: "); Serial.println(adc1);
  Serial.print("Channel 2: "); Serial.println(adc2);
  Serial.print("Channel 3: "); Serial.println(adc3);
}

int Photo::ReadSensors(Adafruit_ADS1115 ads_left, Adafruit_ADS1115 ads_right, Adafruit_ADS1115 ads_back) {
  int16_t adc0 = ads_left.readADC_SingleEnded(0);
  int16_t adc1 = ads_right.readADC_SingleEnded(0);
  int16_t adc2 = ads_back.readADC_SingleEnded(0);

  return adc0 + adc1 + adc2;
}

int Photo::ReadIndividualSensor(Adafruit_ADS1115 sensor, int index) {
  return sensor.readADC_SingleEnded(index);
}










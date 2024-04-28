#include "Photo.h"

Photo::Photo() : ads_left(), ads_right(), ads_back() {
}

bool Photo::InitializeADS() {
  ads_left.setGain(GAIN_FOUR);
  ads_right.setGain(GAIN_FOUR);
  ads_back.setGain(GAIN_FOUR);
  if(!ads_left.begin(0x48) || !ads_right.begin(0x49) || !ads_back.begin(0x4A)) {
    return false;
  }
  return true;
}

double Photo::GetAverageRightValues() {
  int16_t adc5 = ads_right.readADC_SingleEnded(0);
  int16_t adc6 = ads_right.readADC_SingleEnded(1);
  int16_t adc7 = ads_right.readADC_SingleEnded(2);
  int16_t adc8 = ads_right.readADC_SingleEnded(3);

  double average = (adc5 + adc6 + adc7 + adc8) / 4.0;

  return average;
}

double Photo::GetAverageLeftValues() {
  int16_t adc0 = ads_left.readADC_SingleEnded(0);
  int16_t adc1 = ads_left.readADC_SingleEnded(1);
  int16_t adc2 = ads_left.readADC_SingleEnded(2);
  int16_t adc3 = ads_left.readADC_SingleEnded(3);

  double average = (adc0 + adc1 + adc2 + adc3) / 4.0;

  return average;
}

double Photo::GetAverageBackValues() {
  int16_t adc9 = ads_back.readADC_SingleEnded(0);
  int16_t adc10 = ads_back.readADC_SingleEnded(1);
  int16_t adc11 = ads_back.readADC_SingleEnded(2);
  int16_t adc12 = ads_back.readADC_SingleEnded(3);

  double average = (adc9 + adc10 + adc11 + adc12) / 4.0;

  return average;
}

int16_t Photo::GetADSLeftIndividual(int channel) {
  return ads_left.readADC_SingleEnded(channel);
}

int16_t Photo::GetADSRightIndividual(int channel) {
  return ads_right.readADC_SingleEnded(channel);
}

int16_t Photo::GetADSBackIndividual(int channel) {
  return ads_back.readADC_SingleEnded(channel);
}

boolean Photo::isOnLeftLine() {
  double left = GetAverageLeftValues();
  return left < line_threshold;
}

boolean Photo::isOnRightLine() {
  double right = GetAverageRightValues();
  return right < line_threshold;
}

boolean Photo::isOnBackLine() {
  double back = GetAverageBackValues();
  return back < line_threshold;
}

void Photo::moveComplementary(boolean left, boolean right, boolean back) {
  if (left) {
    Serial.println("Left");
  } else if (right) {
    Serial.println("Right");
  } else if (back) {
    Serial.println("Back");
  } 
}

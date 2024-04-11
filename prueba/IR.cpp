#include <Arduino.h>

#include "IR.h"

IR::IR() {
  
}

void IR::begin() {
  if(!ads.begin()) {
    Serial.println("Failed to initialize ADS1115");
    while(1);
  }
}

/*String IR::setI2CAddress(int address) {
  ads.setGain(GAIN_ONE);
  ads.begin();
  //ads.setI2CAddress(address);
  return "I2C Address set to " + String(address);
}

int IR::readIndividualSensor(int sensor) {
  return ads.readADC_SingleEnded(sensor);
}

int IR::readAllSensors() {
  int sensorValues[4];
  for (int i = 0; i < 4; i++) {
    sensorValues[i] = readIndividualSensor(i);
  }
  return sensorValues;
}

bool IR::isOnLine() {
  int sensorValue = readIndividualSensor();
  if (sensorValue < threshold) {
    return true; 
  } else {
    return false; 
  }
}*/

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads_left[3]; // Array to hold instances of ADS1115
Adafruit_ADS1115 ads_right[3];
Adafruit_ADS1115 ads_back[3];

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Hello!");

  for (int i = 0; i < 3; i++) {
    ads1[i].begin(0x48); // Initialize all instances of ADS1115
    ads1[i].setGain(GAIN_FOUR); // Set the gain
  }

   for (int i = 0; i < 3; i++) {
    ads2[i].begin(0x49); // Initialize all instances of ADS1115
    ads2[i].setGain(GAIN_FOUR); // Set the gain
  }

   for (int i = 0; i < 3; i++) {
    ads3[i].begin(0x4A); // Initialize all instances of ADS1115
    ads3[i].setGain(GAIN_FOUR); // Set the gain
  }

  Serial.println("Getting single-ended readings from all ADS1115");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV)");

}

void loop(void)
{
  for (int i = 0; i < 3; i++) { 
    int16_t adc0 = ads1[i].readADC_SingleEnded(0);
    int16_t adc1 = ads1[i].readADC_SingleEnded(1);
    int16_t adc2 = ads1[i].readADC_SingleEnded(2);
    int16_t adc3 = ads1[i].readADC_SingleEnded(3);
    
    Serial.print("ADS1 - Channel 0: "); Serial.println(adc0);
    Serial.print("ADS1 - Channel 1: "); Serial.println(adc1);
    Serial.print("ADS1 - Channel 2: "); Serial.println(adc2);
    Serial.print("ADS1 - Channel 3: "); Serial.println(adc3);
   
  }
   Serial.print("-------------------------------------------");
delay(1000);
  for (int i = 0; i < 3; i++) {
    int16_t adc0 = ads2[i].readADC_SingleEnded(0);
    int16_t adc1 = ads2[i].readADC_SingleEnded(1);
    int16_t adc2 = ads2[i].readADC_SingleEnded(2);
    int16_t adc3 = ads2[i].readADC_SingleEnded(3);
    
    Serial.print("ADS2 - Channel 0: "); Serial.println(adc0);
    Serial.print("ADS2 - Channel 1: "); Serial.println(adc1);
    Serial.print("ADS2 - Channel 2: "); Serial.println(adc2);
    Serial.print("ADS2 - Channel 3: "); Serial.println(adc3);
  }
  Serial.print("-------------------------------------------");
delay(1000);
  for (int i = 0; i < 3; i++) {
    int16_t adc0 = ads3[i].readADC_SingleEnded(0);
    int16_t adc1 = ads3[i].readADC_SingleEnded(1);
    int16_t adc2 = ads3[i].readADC_SingleEnded(2);
    int16_t adc3 = ads3[i].readADC_SingleEnded(3);
    
    Serial.print("ADS3 - Channel 0: "); Serial.println(adc0);
    Serial.print("ADS3 - Channel 1: "); Serial.println(adc1);
    Serial.print("ADS3 - Channel 2: "); Serial.println(adc2);
    Serial.print("ADS3 - Channel 3: "); Serial.println(adc3);
  }
  Serial.print("---------------------------------------------");

delay(1000);
}
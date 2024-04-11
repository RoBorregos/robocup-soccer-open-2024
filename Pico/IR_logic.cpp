

//0x48
//0x49
//0x4A

include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads[3]; // Array to hold instances of ADS1115

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Hello!");

  for (int i = 0; i < 3; i++) {
    ads[i].begin(); // Initialize all instances of ADS1115
    ads[i].setGain(GAIN_TWOTHIRDS); // Set the gain
  }

  Serial.println("Getting single-ended readings from all ADS1115");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV)");

}

void loop(void)
{
  int16_t adc[3][4];
  float volts[3][4];

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      adc[i][j] = ads[i].readADC_SingleEnded(j); // Read all channels of each ADS1115
      volts[i][j] = ads[i].computeVolts(adc[i][j]); // Compute voltage for each reading
    }
  }

  Serial.println("-----------------------------------------------------------");

  // Print the readings for each ADS1115
  for (int i = 0; i < 3; i++) {
    Serial.println("ADS1115 " + String(i));
    for (int j = 0; j < 4; j++) {
      Serial.print("AIN" + String(j) + ": ");
      Serial.print(adc[i][j]);
      Serial.print("  ");
      Serial.print(volts[i][j]);
      Serial.println("V");
    }
  }

 delay(1000);
}
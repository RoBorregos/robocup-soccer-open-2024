#include "Photo.h"

Photo photo;

void setup() {
  Serial.begin(9600);
  photo.InitializeADS();
}

void loop() {
  double averageRightValues = photo.GetAverageRightValues();
  Serial.print("Average Right Values: ");
  Serial.println(averageRightValues);
  delay(1000);
}
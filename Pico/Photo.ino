#include "Photo.h"

Photo photo;

void setup() {
  Serial.begin(9600);  
  photo.InitPhoto();  
}

void loop() {
  for (int i = 0; i < 3; i++) {
    Serial.println("Left sensor " + String(i) + ":");
    photo.PrintSensorValues(i);
    Serial.println("Right sensor " + String(i) + ":");
    photo.PrintSensorValues(i);
    Serial.println("Back sensor " + String(i) + ":");
    photo.PrintSensorValues(i);
  }



 
  int sensorValue = photo.ReadIndividualSensor(0, 0);  // Read the value of the first sensor
  Serial.println("Sensor 0 value: " + String(sensorValue));

  delay(1000);  // Wait for a second
}
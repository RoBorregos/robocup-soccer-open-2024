#include "IR.h"

IR ir;

void setup() {
  Serial.begin(9600);  
  ir.begin();  
  ir.setI2CAddress(0x48);  
}

void loop() {
  int sensorValue = ir.readIndividualSensor(0); 
  Serial.println("Sensor 0 value: " + String(sensorValue));
  bool onLine = ir.isOnLine();
  Serial.println("Is on line: " + String(onLine));

  delay(30);  
}
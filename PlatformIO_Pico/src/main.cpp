#include <Arduino.h>


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Blue");
  delay(5000);
}

void setup1() {
  // put your setup code here, to run once:
   delay(5000);
   Serial.println("second");
}

void loop1() {
  // put your main code here, to run repeatedly:
  Serial.println("s");
  delay(500);
}


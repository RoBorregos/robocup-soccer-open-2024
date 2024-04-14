
#include "Photo.h"
#include "serial.h"
#include <Adafruit_ADS1X15.h>

#define PIN_SERIAL1_TX (0u)
#define PIN_SERIAL1_RX (1u)

SerialCommunication serialComm(Serial1);
float tryy = 0; 
float average = 0; 
float individual = 0; 
void setup(){
  Serial.begin(115200);
    while (!Serial && millis() < 10000UL)
        ;
    Serial.println("Started");
    Serial1.begin(9600);
}
void loop() {
   tryy = serialComm.Receive('j');
   average = serialComm.Receive('q');
   individual = serialComm.Receive('k');

   Serial.println(tryy);
   Serial.println(average);

}
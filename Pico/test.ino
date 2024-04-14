

#include "Photo.h"
#include "serial.h"
#include <Adafruit_ADS1X15.h>

#define PIN_SERIAL1_TX (0u)
#define PIN_SERIAL1_RX (1u)

SerialCommunication serialComm(Serial1);
float init = 0;
float on_line = 0;

void setup(){
  Serial.begin(115200);
    while (!Serial && millis() < 10000UL)
        ;
    Serial.println("Started");
    Serial1.begin(9600);
}
void loop() {
    on_line = serialComm.Receive('a');
    Serial.println(on_line);
    /*init = serialComm.Receive('m');
    individual_back_0 = serialComm.Receive('a');
    individual_back_1 = serialComm.Receive('b');
    individual_back_2 = serialComm.Receive('c');
    individual_back_3 = serialComm.Receive('d');
    individual_left_0 = serialComm.Receive('e');
    individual_left_1 = serialComm.Receive('f');
    individual_left_2 = serialComm.Receive('g');
    individual_left_3 = serialComm.Receive('h');
    individual_right_0 = serialComm.Receive('i');
    individual_right_1 = serialComm.Receive('j');
    individual_right_2 = serialComm.Receive('k');
    individual_right_3 = serialComm.Receive('l');*/

   /*Serial.print("Individual_back_0: ")
   Serial.print(individual_back_0);
   Serial.println(" ");
   Serial.print("Individual_back_1: ")
   Serial.print(individual_back_1);
   Serial.println(" ");
    Serial.print("Individual_back_2: ")
    Serial.print(individual_back_2);
    Serial.println(" ");
    Serial.print("Individual_back_3: ")
    Serial.print(individual_back_3);
    Serial.println(" ");
    Serial.print("Individual_left_0: ")
    Serial.print(individual_left_0);
    Serial.println(" ");
    Serial.print("Individual_left_1: ")
    Serial.print(individual_left_1);
    Serial.println(" ");
    Serial.print("Individual_left_2: ")
    Serial.print(individual_left_2);
    Serial.println(" ");
    Serial.print("Individual_left_3: ")
    Serial.print(individual_left_3);
    Serial.println(" ");
    Serial.print("Individual_right_0: ")
    Serial.print(individual_right_0);
    Serial.println(" ");
    Serial.print("Individual_right_1: ")
    Serial.print(individual_right_1);
    Serial.println(" ");
    Serial.print("Individual_right_2: ")
    Serial.print(individual_right_2);
    Serial.println(" ");
    Serial.print("Individual_right_3: ")
    Serial.print(individual_right_3);
    Serial.println(" ");*/


}
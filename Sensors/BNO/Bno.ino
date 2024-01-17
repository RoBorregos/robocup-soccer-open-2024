#include "Bno.h"
#include "Arduino.h"
#include "Motors.h"
#include <typeinfo> 


void setup(){
    Serial.begin(9600);
    BNO055 myBNO;
    myBNO.InitializeBNO();
    myBNO.getBNOData();
    Serial.println(myBNO.getYaw());     
}


void loop(){
}
#include "Bno.h"


void setup(){
    Serial.begin(9600);
    BNO055 myBNO;
    myBNO.InitializeBNO();
    myBNO.getBNOData();
    Serial.println(myBNO.getYaw());
}

void loop(){
}
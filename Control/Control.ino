#include <Arduino.h>

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println("Hello");
   while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }

  if (stringComplete) {
    float distance, angle;
    sscanf(inputString.c_str(), "%f %f", &distance, &angle);
    Serial.print("Distance: ");
    Serial.println(distance);
    Serial.print("Angle: ");
    Serial.println(angle);
    inputString = "";
    stringComplete = false;
  }
  
}

/*#include "Motors.h"
#include <Arduino.h>

// check motor polarity individually for IN1 and IN2 when robot movement is set to 0 degrees. If not correct, swap IN1 and IN2
//changed motor 1 for motor 4 (remember this)
// encoders: 2, 3, 18, 19
uint8_t motor4In1 = 25;
uint8_t motor4In2 = 6;
uint8_t motor4PWM = 20;

uint8_t motor3In1 = 14;
uint8_t motor3In2 = 11;
uint8_t motor3PWM = 21;

uint8_t motorIn1 = 8;
uint8_t motorIn2 = 9;
uint8_t motorPWM = 10;

uint8_t motor2In1 = 22;
uint8_t motor2In2 = 23;
uint8_t motor2PWM = 15;

void setup()
{

  Serial.begin(9600);

  Motors myMotors(
      motorPWM, motorIn1, motorIn2,
      motor2PWM, motor2In1, motor2In2,
      motor3PWM, motor3In1, motor3In2,
      motor4PWM, motor4In1, motor4In2);

  myMotors.InitializeMotors();
  myMotors.setSpeed(motorPWM, 160);
  myMotors.setSpeed(motor2PWM, 160);
  myMotors.setSpeed(motor3PWM, 160);
  myMotors.setSpeed(motor4PWM, 160);

  myMotors.moveMotors(45, 160);
  delay(1000);
  myMotors.stopMotors();
  myMotors.moveMotors(135, 160);
  delay(1000);
  myMotors.stopMotors();
  myMotors.moveMotors(225, 160);
  delay(1000);
  myMotors.stopMotors();
  myMotors.moveMotors(315, 160);
  delay(1000);
  myMotors.stopMotors();
}

void loop()
{
}
*/

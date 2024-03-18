/*#include <Arduino.h>
#include "Motors.h"
#include <typeinfo>

#define PIN_SERIAL1_TX (0u)
#define PIN_SERIAL1_RX (1u)

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

const uint8_t receive_data = 's';
float angle = 0;

Motors myMotors(
    motorPWM, motorIn1, motorIn2,
    motor2PWM, motor2In1, motor2In2,
    motor3PWM, motor3In1, motor3In2,
    motor4PWM, motor4In1, motor4In2);*/

//#include <Arduino.h>



/*void setup()
{
    myMotors.InitializeMotors();
    Serial.begin(9600);
    Serial1.begin(9600);
}

void loop()
{
    Serial1.write(receive_data);
    while (!Serial1.available())
    {
        continue;
    }
    delay(10);
    float temp;
    uint8_t tempArray[4];
    union u_tag
    {
        byte b[4];
        float angle;
    } u;
    
    u.b[0] = Serial1.read();
    u.b[1] = Serial1.read();
    u.b[2] = Serial1.read();
    u.b[3] = Serial1.read();

    angle = u.angle;
    Serial.println(angle);
    double kp = 4;
    double target_angle = 0;
    double error = target_angle - angle;
    double speed = kp * error;
    
    speed = abs(speed);

    if (speed > 0)
    {
        myMotors.setSpeed(motorPWM, speed);
        myMotors.setSpeed(motor2PWM, speed);
        myMotors.setSpeed(motor3PWM, speed);
        myMotors.setSpeed(motor4PWM, speed);
        myMotors.moveRight();
        Serial.println("Right");
    }
    else if (speed < 0)
    {
        myMotors.setSpeed(motorPWM, speed);
        myMotors.setSpeed(motor2PWM, speed);
        myMotors.setSpeed(motor3PWM, speed);
        myMotors.setSpeed(motor4PWM, speed);
        myMotors.moveLeft();
        Serial.println("Left");
    }
    else
    {
        myMotors.setSpeed(motorPWM, speed);
        myMotors.setSpeed(motor2PWM, speed);
        myMotors.setSpeed(motor3PWM, speed);
        myMotors.setSpeed(motor4PWM, speed);
        myMotors.stopMotors();
        Serial.println("Stop");
    }
}*/

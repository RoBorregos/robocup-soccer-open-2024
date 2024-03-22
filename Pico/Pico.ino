#include <Arduino.h>
#include <Motors.h>
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

const uint8_t receive_bno = 's';
const uint8_t receive_cam = 'c';
float angle = 0;
float cam_angle = 0;
Motors myMotors(
    motorPWM, motorIn1, motorIn2,
    motor2PWM, motor2In1, motor2In2,
    motor3PWM, motor3In1, motor3In2,
    motor4PWM, motor4In1, motor4In2);



void setup()
{
    myMotors.InitializeMotors();
    Serial.begin(115200);
    while (!Serial && millis() < 10000UL);
    Serial.println("started");
    Serial1.begin(9600);
    Serial.println("Hello World");
}

void loop()
{
    static unsigned long prev_millis = 0;
    unsigned long current_millis = millis();
    double elapsed_time = current_millis - prev_millis;
    prev_millis = current_millis;
    angle = receive(receive_bno);
    cam_angle = receive(receive_cam);
    Serial.print(angle);
    Serial.print(" ");
    Serial.println(cam_angle);
   
   // Added PID control
    double kp = 4;
    double ki = 0.1;
    double kd = 0.01;

    // For future instances target angle should be ball angle (cam_angle)
    double target_angle = 0;
    double error = target_angle - angle;

    static double integral = 0;
    static double prev_error = 0;
    integral += error * elapsed_time;
    double derivative = (error - prev_error) / elapsed_time;
    prev_error = error;
    
    
    double speed = kp * error + ki * integral + kd * derivative;
    
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
}

float receive (uint8_t signal){
     Serial1.write(signal);
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
    return u.angle;
} 

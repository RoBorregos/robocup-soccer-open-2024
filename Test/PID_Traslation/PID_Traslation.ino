/*Code for testing the PID TRASLATION USING moveMotorsImu*/
// The robot needs to follow a straight line in any direction always facing target angle (0 degrees)

#include <Arduino.h>
#include <Pico/motors.h>
#include <typeinfo>

#define PIN_SERIAL1_TX (0u)
#define PIN_SERIAL1_RX (1u)

uint8_t motor3In1 = 23;
uint8_t motor3In2 = 22;
uint8_t motor3PWM = 15;

uint8_t motor2In1 = 25;
uint8_t motor2In2 = 6;
uint8_t motor2PWM = 20;

uint8_t motor4In1 = 9;
uint8_t motor4In2 = 8;
uint8_t motor4PWM = 10;

uint8_t motorIn1 = 14;
uint8_t motorIn2 = 11;
uint8_t motorPWM = 21;

const uint8_t receive_bno = 's';
const uint8_t receive_cam = 'c';
const uint8_t receive_distance = 'd';
float angle = 0;
float cam_angle = 0;
float distance = 0;

//PID
double kp = 1.05;
double ki = 3.2;
double kd = 0.2;
double target_angle = 0;
//int plane = -90;
//int frame_angle = 0 + plane;
int frame_angle = 90;
double last_error = 0;
double all_error = 0;
double max_error = 30;

//Sampling Time
double last_time = 0;
double current_time = 0;
double delta_time = 0;

//spd
int speed_traslational = 190;


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
    //analogWrite(motorPWM, 100);
    //analogWrite(motor2PWM, 100);
    //analogWrite(motor3PWM, 100);
    //analogWrite(motor4PWM, 100);
}

void loop()
{
    // Receive Data
    angle = receive(receive_bno);
    cam_angle = receive(receive_cam);
    distance = receive(receive_distance);
    Serial.print(angle);
    Serial.print(" ");
    //Serial.print(cam_angle);
    //Serial.print(" ");
    //Serial.println(distance);

   // PID & Motor Control
    current_time = millis();
    delta_time = current_time - last_time;
    double new_error = target_angle - angle;
    all_error = new_error + last_error;
    if (all_error > max_error)
    {
        all_error = max_error;
    }
    else if (all_error < -max_error)
    {
        all_error = -max_error;
    }
    double sample_time = delta_time/1000.0;
    double proportional = kp * new_error;
    double integer = ki * all_error * sample_time;
    double derivative = kd * (new_error - last_error) / sample_time;
    double speed_w = proportional + integer + derivative;
    last_error = new_error;
    last_time = current_time;
    //
    Serial.println(speed_w);
    //myMotors.moveMotorsImu(target_angle,speed_traslational, speed_w);
    myMotors.MoveMotorsImu(frame_angle, speed_traslational, speed_w);
  
    //myMotors.moveMotors(0, speed_traslational);
    //myMotors.moveForward();
    delay(20);
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

/*Code that only use feedback from the angle of the ball and moves the robot towards it*/

#include <Arduino.h>
#include <Motors.h>
#include <typeinfo>

uint8_t motor4In1 = 23;
uint8_t motor4In2 = 22;
uint8_t motor4PWM = 15;

uint8_t motor3In1 = 25;
uint8_t motor3In2 = 6;
uint8_t motor3PWM = 20;

uint8_t motorIn1 = 8;
uint8_t motorIn2 = 9;
uint8_t motorPWM = 10;

uint8_t motor2In1 = 14;
uint8_t motor2In2 = 11;
uint8_t motor2PWM = 21;

const uint8_t receive_bno = 's';
const uint8_t receive_cam = 'c';
const uint8_t receive_distance = 'd';

int i = 0;

float angle = 0;
float cam_angle = 0;
float average_angle = 0;
float last_angle = 0;
float filter_angle = 0;
float distance = 0;


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
    // Receive Data
    angle = receive(receive_bno);
    cam_angle = receive(receive_cam);
    distance = receive(receive_distance);


    Serial.print(angle);
    Serial.print(" ");
    Serial.print(cam_angle);
    Serial.print(" ");
    Serial.println(distance);




    //Move Motors according to the received data
    //average_angle= (cam_angle+last_angle)/i;
    //last_angle = cam_angle;
    myMotors.moveMotors(cam_angle,200);
    
    if(cam_angle == 0 && distance == 0){
        myMotors.stopMotors();
        Serial.println("Stop");
    }

    
    
    delay(100);
    i += 1;
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

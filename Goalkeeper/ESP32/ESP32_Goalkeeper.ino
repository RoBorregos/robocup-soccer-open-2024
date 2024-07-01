#include <Simple_MPU6050.h>
#include "Transmission.h"
#include <Wire.h>

#define RXD2 16
#define TXD2 17

const int kNumSamples = 10;
int samples[kNumSamples];
int sample_index = 0;
double offset = 0;
double filtered_angle = 0;
double angle = 0;
float ypr[3] = {0, 0, 0};
float xyz[3] = {0, 0, 0};

const float kFilterCoefB0 = 0.000395;
const float kFilterCoefB1 = 0.000791;
const float kFilterCoefB2 = 0.000395;
const float kFilterCoefA1 = -1.972186;
const float kFilterCoefA2 = 0.972613;

float distance_pixels = 0;
float ball_angle = 0;
float ball_distance = 0;
float goal_angle = 0;

Simple_MPU6050 mpu;

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
}

void loop()
{
    if (Serial.available())
    {
        ParseCamString(ball_distance, ball_angle, goal_angle, distance_pixels);
    }
    SendDataSerial(filtered_angle, ball_distance, ball_angle, goal_angle, distance_pixels);
}

// Function to get gyro values and process them to get the filtered angle (yaw)
void GyroValues(int16_t *gyro, int16_t *accel, int32_t *quat)
{
    Quaternion q;
    VectorFloat gravity;
    ypr[3] = {0, 0, 0};
    xyz[3] = {0, 0, 0};
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);
    angle = xyz[0];
    filtered_angle = filter(angle);

    if (Serial.available())
    {
        ParseCamString(ball_distance, ball_angle, goal_angle, distance_pixels);
    }

    SendDataSerial(filtered_angle, ball_distance, ball_angle, goal_angle, distance_pixels);
}

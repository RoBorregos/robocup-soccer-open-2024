#include <Simple_MPU6050.h>
#include <Wire.h>



#define RXD2 16
#define TXD2 17
const int numSamples = 10; // Número de muestras para promediar
int samples[numSamples]; // Arreglo para almacenar las muestras
int sampleIndex = 0; // Índice actual en el arreglo de muestras

// Compass:
Simple_MPU6050 mpu;
double offset = 0;

// Coeficientes del filtro Butterworth (2º orden, fc = 0.1 Hz)
const float b0 = 0.000395;
const float b1 = 0.000791;
const float b2 = 0.000395;
const float a1 = -1.972186;
const float a2 = 0.972613;
float distance_pixels = 0; 
String angleString; 
float ball_angle = 0;
float ball_distance = 0;
float goal_angle = 0;


void gyroValues(int16_t *gyro, int16_t *accel, int32_t *quat) {
    // Calcular ángulos de movimiento:
    Quaternion q;
    VectorFloat gravity;
    float ypr[3] = {0, 0, 0};
    float xyz[3] = {0, 0, 0};
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);
    double angle = xyz[0];
    double filtered_angle = filter(angle);
    Serial.print(angle); Serial.print(" "); Serial.println(filtered_angle);
    angleString = String(filtered_angle);
    if (Serial.available())
    {
        String camString = Serial.readStringUntil('\n');
        ball_distance = camString.toFloat();
        ball_angle = camString.substring(camString.indexOf(' ') + 1).toFloat();
        goal_angle = camString.substring(camString.indexOf(' ', camString.indexOf(' ') + 1) + 1, camString.lastIndexOf(' ')).toFloat();
        distance_pixels = camString.substring(camString.lastIndexOf(' ') + 1).toFloat();
    }
    String ballDistance = String(ball_distance);
    String ballAngle = String(ball_angle);
    String goalAngle = String(goal_angle);
    //String goalDistance = String(goal_distance);
    String distancePixels = String(distance_pixels);
    Serial2.print(angleString);
    Serial2.print(",");
    Serial2.print(ballDistance);
    Serial2.print(",");
    Serial2.print(ballAngle);
    Serial2.print(",");
    Serial2.print(goalAngle);
    Serial2.print(",");
    /*Serial2.print(goalDistance);
    Serial2.print(",");*/
    Serial2.print(distancePixels);
    Serial2.println();
}

/**
 *  Inicio de Arduino, configuraciones:
 *  - Motores
 *  - Comunicacion Serial con Camara
 *  - Giroscopio
 */
void setup() {
    // Calibracion Basica de Arduino:
    Wire.begin();
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
    // Calibracion del Giroscopio:
}

/**
 *  En el Loop solamente se mandara a hacer las lecturas
 *  del giroscopio para actualizar los valores.
 */
void loop() {
  if (Serial.available())
    {
        String camString = Serial.readStringUntil('\n');
        ball_distance = camString.toFloat();
        ball_angle = camString.substring(camString.indexOf(' ') + 1).toFloat();
        goal_angle = camString.substring(camString.indexOf(' ', camString.indexOf(' ') + 1) + 1, camString.lastIndexOf(' ')).toFloat();
        distance_pixels = camString.substring(camString.lastIndexOf(' ') + 1).toFloat();
    }
    String ballDistance = String(ball_distance);
    String ballAngle = String(ball_angle);
    String goalAngle = String(goal_angle);
    //String goalDistance = String(goal_distance);
    String distancePixels = String(distance_pixels);
    Serial2.print(angleString);
    Serial2.print(",");
    Serial2.print(ballDistance);
    Serial2.print(",");
    Serial2.print(ballAngle);
    Serial2.print(",");
    Serial2.print(goalAngle);
    Serial2.print(",");
    /*Serial2.print(goalDistance);
    Serial2.print(",");*/
    Serial2.print(distancePixels);
    Serial2.println();

    
}

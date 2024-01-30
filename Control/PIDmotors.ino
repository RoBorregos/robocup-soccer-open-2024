#include <Arduino.h>

// Define motor control pins
const int motorIn1 = 9;
const int motorIn2 = 10;
const int motorPWM = 5;
const int motorSTBY = 11;

// Variables del controlador PID
double Kp = 2;  // Constante proporcional
double Setpoint = 100;  // Valor deseado

// Variables internas del controlador PID
unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError;
double Output;

void setup() {
  // Set motor control pins as outputs
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorSTBY, OUTPUT);

  // Enable the motor driver
  digitalWrite(motorSTBY, HIGH);

  // Inicializar el controlador PID
  currentTime = millis();
  previousTime = currentTime;

  // Establecer la velocidad inicial de los motores
  analogWrite(motorPWM, 128);  // Establecer velocidad (0 a 255)
}

void loop() {
  // Leer la entrada del sensor (por ejemplo, un potenciómetro)
  double sensorValue = analogRead(A0);

  // Calcular la salida del controlador PID
  Output = computeP(sensorValue);

  // Aplicar la salida del controlador PID al motor
  if (Output > 0) {
    digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);
  } else {
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, HIGH);
  }

  // Ajustar la velocidad del motor según la salida del controlador PID
  analogWrite(motorPWM, abs(Output));

  delay(100);  // Ajusta este valor según sea necesario
}

double computeP(double inp) {
  currentTime = millis();
  elapsedTime = (double)(currentTime - previousTime);

  error = Setpoint - inp;
  double output = Kp * error;

  lastError = error;
  previousTime = currentTime;

  return output;
}

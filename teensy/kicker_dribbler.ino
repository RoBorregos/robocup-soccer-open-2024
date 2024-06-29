#include <Servo.h>

// Create a Servo object to control the ESC
Servo esc;

// Define the pin for the ESC signal wire
const int escPin = 10;

// Define speed levels
const int minSpeed = 1000; // Minimum speed (1000 microseconds)
const int midSpeed = 1000; // Mid speed (1500 microseconds)
const int maxSpeed = 2000; // Maximum speed (2000 microseconds)

// Define delay times
const int delayTime = 5000; // Delay time in milliseconds

void setup() {
  // Attach the ESC to the servo object
  esc.attach(escPin);

  // Initialize serial communication for debugging
  Serial.begin(9600);
  pinMode(8,OUTPUT);
  // Send a low signal to arm the ESC (usually 1000 microseconds)
  esc.writeMicroseconds(minSpeed);
  Serial.println("Arming ESC...");
  
  // Wait for a few seconds to allow the ESC to arm
  delay(3000);
  
  // Confirm ESC is armed
  Serial.println("ESC Armed. Ready to control the motor.");
}

void loop() {

  
  // Set motor to maximum speed
  Serial.println("Setting speed to maximum...");
  esc.writeMicroseconds(1400);
  delay(delayTime);
  esc.writeMicroseconds(1100);
  delay(70);
  digitalWrite(8,HIGH);
  delay(5);
  digitalWrite(8,LOW);
  // Repeat the cycle
}
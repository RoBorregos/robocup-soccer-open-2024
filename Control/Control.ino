#include <Arduino.h>

// Motor encoder output pulses per 360 degree revolution (measured manually)
#define ENC_COUNT_REV 56

// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
#define ENC_IN_RIGHT_A 2

// True = Forward; False = Reverse
boolean Direction_right = true;

// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;

// One-second interval for measurements
int interval = 1000;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

// Variables for RPM and angular velocity measurement
float rpm_right = 0;
float ang_velocity_right = 0;

void setup() {

  // Open the serial port at 9600 bps
  Serial.begin(9600);

  // Set pin state of the encoder
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);

  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);

}

void loop() {

  // Record the time
  currentMillis = millis();

  // If one second has passed, print the number of pulses and calculate RPM and angular velocity
  if (currentMillis - previousMillis > interval) {

    previousMillis = currentMillis;

    // Calculate revolutions per minute
    rpm_right = (float)(right_wheel_pulse_count * 60 / ENC_COUNT_REV);

    // Calculate angular velocity in radians per second
    ang_velocity_right = (rpm_right * 2 * PI) / 60;

    Serial.print(" Pulses: ");
    Serial.println(right_wheel_pulse_count);
    Serial.print(" Speed: ");
    Serial.print(rpm_right);
    Serial.println(" RPM");
    Serial.print(" Angular Velocity: ");
    Serial.print(ang_velocity_right);
    Serial.println(" rad/s");
    Serial.println();

    right_wheel_pulse_count = 0;
  }
}

// Increment the number of pulses by 1
void right_wheel_pulse() {
  // Increment the pulse count
  right_wheel_pulse_count++;
}

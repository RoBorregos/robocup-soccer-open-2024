/*#include <Arduino.h>

// Motor encoder output pulses per 360 degree revolution (measured manually)
#define ENC_COUNT_REV 620

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
*/

//there are 56 pulses per revolution

#include <Arduino.h>

// Motor control pin
#define MOTOR_PWM_PIN_A 24 // Change this to the actual pin connected to your motor driver
#define MOTOR_PWM_PIN_B 26 // Change this to the actual pin connected to your motor driver
#define MOTOR_PWM 7 // Change this to the actual pin connected to your motor driver
#define MOTOR_STBY 22 // Change this to the actual pin connected to your motor driver

// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
#define ENC_IN_RIGHT_A 2

// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;

void setup() {
  // Open the serial port at 9600 bps
  Serial.begin(9600);
  pinMode(MOTOR_PWM_PIN_A, OUTPUT);
  pinMode(MOTOR_PWM_PIN_B, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);

  // Enable the motor driver
  digitalWrite(MOTOR_STBY, HIGH);

  //PWM
  analogWrite(MOTOR_PWM, 128); // Set speed (0 to 255)

  

  // Set pin states of the encoder
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);


  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
}

void loop() {
  digitalWrite(MOTOR_PWM_PIN_A, HIGH);
  digitalWrite(MOTOR_PWM_PIN_B, LOW);
  Serial.print(" Pulses: ");
  Serial.println(right_wheel_pulse_count);
}

// Increment the number of pulses by 1
void right_wheel_pulse() {
  right_wheel_pulse_count++;
}

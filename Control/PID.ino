/*#include <Arduino.h>

const int PIN_INPUT = A0;
const int PIN_OUTPUT = 3;

const int motor4In1 = 45;  // 43
const int motor4In2 = 43;  // 41
const int motor4PWM = 12;  // 
const int motor4STBY = 41; // 

double Kp = 2; 

double Input, Output, Setpoint;
 
unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError;
 
void setup()
{
  Input = analogRead(PIN_INPUT);
  Setpoint = 100;
  //setpoint es el 85% de la entrada de la velocidad maxima 
    pinMode(motor4In1, OUTPUT);
  pinMode(motor4In2, OUTPUT);
  pinMode(motor4PWM, OUTPUT);
  pinMode(motor4STBY, OUTPUT);

  // Enable the motor driver
  digitalWrite(motor4STBY, HIGH);
  //PWM
  analogWrite(motor4PWM, 128); // Set speed (0 to 255)
}    
 
void loop(){
  Input = analogRead(PIN_INPUT);      
  Output = computeP(Input);           
  analogWrite(PIN_OUTPUT, Output);    
}
 
double computeP(double inp){   
  //obtener tiempo actual  
  currentTime = millis();  
  // calcular el tiempo transcurrido                           
  elapsedTime = (double)(currentTime - previousTime); 
  // calcular el error
  error = Setpoint - inp;  
  //calcular la salidad proporcional                            
  double output = Kp * error;                        
  lastError = error;                                    
  previousTime = currentTime;                          
  return output;
}
*/
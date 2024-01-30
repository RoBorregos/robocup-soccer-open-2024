#include <Arduino.h>

const int PIN_INPUT = A0;
const int PIN_OUTPUT = 3;

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
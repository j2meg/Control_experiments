/*Arduino PID implementation
by Juan josé Meza Gutiérrez
*/
#include <Wire.h>//Include the Wire library to talk I2C

//This is the I2C Address of the MCP4725, by default (A0 pulled to GND).
//Please note that this breakout is for the MCP4725A0. 
#define MCP4725_ADDR 0x60   

const int analogInPin = A0; // Analog input pin 
const int analogRefPin= A1;
unsigned int InputValue = 0; // value read from the plant system // equivalent to read
unsigned int OutValue   = 55; // equivalent to lookup
unsigned int SetPoint = 1638; 
// Ganancias de control 
float kp = 0;
float kd = 0;
float ki = 0;
//Errores 
float ep=0, ei=0, ed=0;   // Errores para PID
float ek0=0, ek1=0,ek2=0; // Errores de estados anteriores
// Esfuerzo de estado anterior
float uk1=0;
// Referencia // Tamaño de paso // Esfuerzo calculado por PID
float r= 1,h=0.01,F=0; 

// Tiempo de muestreo // Pendiente añadir timer para definirlo 
//

// Timer1 interruption function
ISR(TIMER1_COMPA_vect)
{
  OCR1A += 16; // Advance The COMPA Register
  // Handle The Timer Interrupt
  //...
   InputValue = analogRead(analogInPin);
   // Esto funcionaria si el setpoint y el InputValue estuvieran en la misma escala, pero uno va a 4096 y el otro a 1024
    ek0 = SetPoint-InputValue; // Error 
  // Esfuerzo de control 

  // Error F y ek0 deberian ser de punto flotante 
  F=  uk1 + (kp+ki*h+kd/h)*ek0 + (-2*kd/h-kp)*ek1 + (kd/h)*ek2;
  uk1 = F;
  ek2 = ek1;
  ek1 = ek0;
  OutValue =F;  
  Wire.beginTransmission(MCP4725_ADDR);
//  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64);     // command to update the DAC
  // Edit next lines to send our variable
  Wire.write(OutValue >> 4);        // the 8 most significant bits...
  Wire.write((OutValue & 15) << 4); // the 4 least significant bits...
  Wire.endTransmission();
  
}
void setup() {
  // Timer1 interruption configuration for a interruption period of 1 microsecond.
  TCCR1A = 0;           // Init Timer1A
  TCCR1B = 0;           // Init Timer1B
  TCCR1B |= B00000001;  // Prescaler = 1
  OCR1A = 16;        // Timer Compare1A Register
  TIMSK1 |= B00000010;  // Enable Timer COMPA Interrupt

  // Initialize serial communications at 115200 bps
//    Serial.begin(115200);
    Wire.begin();
}

// Te void loop is a function that changes the setpoint to 3 and 2 volts (an step) 
void loop() {

SetPoint= 1638; // 2 volts   
delay(2); 
SetPoint= 2457;// 3 volts
delay(2);   

}


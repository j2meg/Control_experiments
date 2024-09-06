/*
This program uses an NodeMCU v1.0 Implement a digital PI control system.
The update of the input and output is driven by an arbitrary sample time defined by a hardware timer. 

Materials list. 
NodeMCU V1.0
MCP4725 (12 bits I2C DAC)
The plant attached to this PI control system is a two stage Low Pass Filter
made with two 15kOhms resistors and 2 100nF capacitors.  


Connections:
NodeMCU Pins  | MCP4725 Pins |  Plant System
    GND       |   GND        |
    3.3V      |   VCC        |
    D2        |   SDA        |
    D1        |   SCL        |
    GND       |   GND        |
    A0        |              |      Output
              |   OUT        |      Input
 
*/

/*Notes about the interruption library handler for ESP8266*/
/****************************************************************************************************************************
  Argument_None.ino
  For ESP8266 boards
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/ESP8266TimerInterrupt
  Licensed under MIT license

  The ESP8266 timers are badly designed, using only 23-bit counter along with maximum 256 prescaler. They're only better than UNO / Mega.
  The ESP8266 has two hardware timers, but timer0 has been used for WiFi and it's not advisable to use. Only timer1 is available.
  The timer1's 23-bit counter terribly can count only up to 8,388,607. So the timer1 maximum interval is very short.
  Using 256 prescaler, maximum timer1 interval is only 26.843542 seconds !!!

  Now with these new 16 ISR-based timers, the maximum interval is practically unlimited (limited only by unsigned long miliseconds)
  The accuracy is nearly perfect compared to software timers. The most important feature is they're ISR-based timers
  Therefore, their executions are not blocked by bad-behaving functions / tasks.
  This important feature is absolutely necessary for mission-critical tasks.
*****************************************************************************************************************************/

/* Notes:
   Special design is necessary to share data between interrupt code and the rest of your program.
   Variables usually need to be "volatile" types. Volatile tells the compiler to avoid optimizations that assume
   variable can not spontaneously change. Because your function may change variables while your program is using them,
   the compiler needs this hint. But volatile alone is often not enough.
   When accessing shared variables, usually interrupts must be disabled. Even with volatile,
   if the interrupt changes a multi-byte variable between a sequence of instructions, it can be read incorrectly.
   If your data is multiple variables, such as an array and a count, usually interrupts need to be disabled
   or the entire sequence of your code which accesses the data.
*/

#if !defined(ESP8266)
  #error This code is designed to run on ESP8266 and ESP8266-based boards! Please check your Tools->Board setting.
#endif

// These define's must be placed at the beginning before #include "ESP8266TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

// Select a Timer Clock
#define USING_TIM_DIV1                false           // for shortest and most accurate timer
#define USING_TIM_DIV16               true           // for medium time and medium accurate timer
#define USING_TIM_DIV256              false            // for longest timer but least accurate. Default

#include "ESP8266TimerInterrupt.h"

#ifndef LED_BUILTIN
  #define LED_BUILTIN       2         // Pin D4 mapped to pin GPIO2/TXD1 of ESP8266, NodeMCU and WeMoS, control on-board LED
#endif

// Library to use the I2C protocol
#include <Wire.h>
#define MCP4725_ADDR 0x60

volatile uint32_t lastMillis = 0;
// Input and output data 
volatile int Input = 0; // A0 input 10 bits ADC values(0,1023)
volatile float fInput =0;
volatile int Out   = 0; // MCP4725 16 bits DAC  values(0,4095)
volatile float ek=0, ek1=0, ek2=0;

volatile float Uk=0; // Esfuerzo de control actual
volatile float Uk1=0; // Esfuerzo de control t anterior
volatile float Uk2=0; // Esfuerzo de control 2 t anterior


/*
float Kp=7; 
float Ki=0;
float Kd=0;
*/

//P
float Kp=     1.2785;  
float Ki=      -1.1176;
float Kd=    0.1164;


//PI
/*
float Kp=Ku*0.45;  
float Ki=Kp/(Pu/1.2); // Ki=Kp/Ti -- Ti=Pu/1.2
float Kd=0;
*/
//PID
/*
float Kp=0.6*Ku; // 
float Ki=Kp/(0.5*Pu); //Ki=Kp/Ti -- Ti=0.5*Pu
float Kd=Kp*Pu/8; // Kd=Kp*Td -- Td=Pu/8
*/


//
float SetPoint       = 0; // Signal to Follow by the driver
const int analogInPin = A0;
uint InputValue=0;
float h=0.0007;
// Interruption handler function
void IRAM_ATTR TimerHandler()
{
  Input=analogRead(analogInPin);
  fInput=Input*(3.3/1024); // Conversion de datos a valores adecuados
  
  ek = SetPoint-fInput; // Error
  
  //Uk= Uk1+Kp*ek+Ki*(ek-ek1); // Esfuerzo de control Integral
  //Uk= Uk1+(kp+ki*h+kd/h)*ek+(-2*kd/h-kp)*ek1+(kd/h)*ek2; 
  //Uk= Uk1+Kp*ek + ki*h*ek + kd/h*ek+(-2*kd/h-kp)*ek1+(kd/h)*ek2; 
  
  Uk= Uk1+(Kp+Ki*h+Kd/h)*ek+(-2*Kd/h-Kp)*ek1+(Kd/h)*ek2; 
  //Uk= Uk1+Kp*ek+Ki*ek1+Kd*ek2; 
  
  Uk1=Uk;
  ek2=ek1;
  ek1=ek;
  // Proteccion del sistema
  if (Uk>3.3)
  {Uk=3.3;}
  if (Uk<0){
    Uk=0;
  }
  Out=Uk*4096/3.3; // Conversión del esfuerzo a unidades adecuadas

  Out=constrain(Out,0,4095);
  // Envio de datos al DAC
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64);
  Wire.write(Out >> 4);
  Wire.write((Out& 15) << 4);
  Wire.endTransmission();
}

#define TIMER_INTERVAL_MS        1
#define TIMER_INTERVAL_uS        700 // Periodo de muestreo en microsegundos


// Init ESP8266 timer 1
ESP8266Timer ITimer;

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);
  
  delay(300);

  Serial.print(F("\nStarting Argument_None on ")); Serial.println(ARDUINO_BOARD);
  Serial.println(ESP8266_TIMER_INTERRUPT_VERSION);
  Serial.print(F("CPU Frequency = ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));
  // Output Initialization 
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64);
  Wire.write(Out >> 4);
  Wire.write((Out& 15) << 4);
  Wire.endTransmission();
  // Interruptions Initialization
  // Interval in microsecs
//  if (ITimer.attachInterruptInterval(TIMER_INTERVAL_MS * 1000, TimerHandler))
  if (ITimer.attachInterruptInterval(TIMER_INTERVAL_uS , TimerHandler))
  {
    lastMillis = millis();
    Serial.print(F("Starting  ITimer OK, millis() = ")); Serial.println(lastMillis);
  }
  else
    Serial.println(F("Can't set ITimer correctly. Select another freq. or interval"));
}

void loop()
{
SetPoint =0;   // 0 volts
delay(35);
/*SetPoint = 256;
delay(500);
*/
SetPoint = 1; // 1.5 volts
delay(35);

//SetPoint = 2;// 3.3 volts
//delay(25);
}

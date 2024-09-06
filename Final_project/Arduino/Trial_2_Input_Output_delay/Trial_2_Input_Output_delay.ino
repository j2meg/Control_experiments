/*
This program uses an NodeMCU v1.0 to see the delay an analog input and a DAC output.
The update of the input and output is driven by an arbitrary sample time defined by a hardware timer. 

Materials list. 
NodeMCU V1.0
MCP4725 (12 bits I2C DAC)

*** Use an external osciloscope and wave generator to view the delay between ADC input and DAC 

Connections:
NodeMCU Pins  | MCP4725 Pins 
    GND       |   GND
    3.3V      |   VCC
    D2        |   SDA
    D1        |   SCL
    GND       |   GND
    A0        |                  wave generator | Osciloscope C2
  |           |   Out            Osciloscope Ch1
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
volatile uint Input = 0; // A0 input 10 bits ADC values(0,1023)
volatile uint Out   = 0; // MCP4725 16 bits DAC  vañues(0,4095)
uint SetPoint       = 1*1024/3; // Signal to Follow by the driver
const int analogInPin = A0;
uint InputValue=0;

void IRAM_ATTR TimerHandler()
{ // Just read ADC and send the data to the DAC output
  Input=analogRead(analogInPin);
  Out=Input*4;
  Out=constrain(Out,0,4095);
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64);
  Wire.write(Out >> 4);
  Wire.write((Out& 15) << 4);
  Wire.endTransmission();
}

#define TIMER_INTERVAL_MS        1
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
  if (ITimer.attachInterruptInterval(TIMER_INTERVAL_MS * 1000, TimerHandler))
  {
    lastMillis = millis();
    Serial.print(F("Starting  ITimer OK, millis() = ")); Serial.println(lastMillis);
  }
  else
    Serial.println(F("Can't set ITimer correctly. Select another freq. or interval"));
}
void loop()
{
  // this space is clean because everything is 
  // done in the interruption handler
}

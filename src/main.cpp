#include <Arduino.h>
#ifdef AVR_DEBUG
#include "avr8-stub.h"
#include "avr_debugger.h"
#endif
#include <Wire.h>
#include "MCP23017.h"

//F5 for debug

#define MCP23017_I2C_ADDRESS 0x27  // I2C address of the MCP23017 IC

//inputs cause interrupts on change
const uint8_t D0 = 0;      // GPA0 (21) of the MCP23017
const uint8_t D1 = 1;      // GPA1 (22) of the MCP23017
const uint8_t D2 = 2;      // GPA2 (23) of the MCP23017
const uint8_t D3 = 3;      // GPA3 (24) of the MCP23017
const uint8_t D4 = 4;      // GPA4 (25) of the MCP23017
const uint8_t D5 = 5;      // GPA5 (26) of the MCP23017
const uint8_t D6 = 6;      // GPA6 (27) of the MCP23017
const uint8_t D7 = 7;      // GPA7 (28) of the MCP23017

//inputs cause interrupts on rising
const uint8_t button0 = 8;   // GPB0 (1) of the MCP23017
const uint8_t button1 = 9;   // GPB1 (2) of the MCP23017
const uint8_t button2 = 10;  // GPB2 (3) of the MCP23017

//inputs cause interrupts on change
const uint8_t echo0 = 13;  // GPB5 (6) of the MCP23017
const uint8_t echo1 = 14;  // GPB6 (7) of the MCP23017

//digital output
const uint8_t trig = 15;  // GPB7 (8) of the MCP23017

const uint8_t MCP23017_INTB = 3;
const uint8_t MCP23017_INTA = 3;

volatile bool buttonPressed = false;

bool startTimer =false; //false = stop timer
                        //true = start timer

MCP23017 mcp23017 = MCP23017(MCP23017_I2C_ADDRESS);  // instance of the connected MCP23017 IC

void mcp23017ChangeDetectedOnPortB();
void configurePinsWithPinMode();
void configureInterrupts();

/* Provide print functions that either use GDB or regular serial port transport */
void print(const char* str) {
#ifdef AVR_DEBUG
  // We can still transport the message via debug messages
  debug_message(str);
#else
  // Not in debug mode? Use regular print.
  Serial.print(str);
#endif
}

void print(String& str) {
  print(str.c_str());
}

void setup() {
#ifndef AVR_DEBUG
   Serial.begin(9600);
#endif
   Wire.begin();     // initialize I2C serial bus
   mcp23017.init();  // initialize MCP23017 IC
   // Configure MCP23017 I/O pins
   configurePinsWithPinMode();  // familiar pinMode() style
   configureInterrupts();
   // Reset MCP23017 ports
   mcp23017.writeRegister(MCP23017Register::GPIO_A, 0x00);
   mcp23017.writeRegister(MCP23017Register::GPIO_B, 0x00);
   // Activate debugging stack if we're in the uno_debug enviornment
#ifdef AVR_DEBUG
   debug_init();
#endif
}
void loop() {
  //mcp23017.clearInterrupts();
  print(String("Port B is:") + mcp23017.readRegister(MCP23017Register::GPIO_B));

  if(buttonPressed ==true)
  {
    uint8_t button = mcp23017.readRegister(MCP23017Register::INTF_B);
    if(button == 0x01)
    {
      print("Button1 Pressed\n");
    }
    else if(button == 0x02)
    {
      print("Button2 Pressed\n");
    }
    else if(button == 0x04)
    {
      print("Button3 Pressed\n");
    }
    buttonPressed= false;
  }

  print("Port B is:" +mcp23017.readRegister(MCP23017Register::GPIO_B));
  mcp23017.clearInterrupts();
  delay(100);
}
void configurePinsWithPinMode() {
   // Configure output pins

   // Configure input pins with internal 100K pull-up resistors
   // Third argument inverts the polarity of the input value when read

  //set buttons to input/internally pulled up & invert polarity of input
  mcp23017.pinMode(button0, INPUT_PULLUP, true);
  mcp23017.pinMode(button1, INPUT_PULLUP, true);
  mcp23017.pinMode(button2, INPUT_PULLUP, true);

  //set pins of ultrasonic proximity sensor
  //trigger/output lines connected and input lines can be read separately
  mcp23017.pinMode(echo0, INPUT_PULLUP, true);
  mcp23017.pinMode(echo1, INPUT_PULLUP, true);
  mcp23017.pinMode(trig, OUTPUT, true);

  //set 8 datalines as inputs with pull-up resistor
  //invert polarity so default it 0xFF
  mcp23017.pinMode(D0, INPUT_PULLUP, true);
  mcp23017.pinMode(D1, INPUT_PULLUP, true);
  mcp23017.pinMode(D2, INPUT_PULLUP, true);
  mcp23017.pinMode(D3, INPUT_PULLUP, true);
  mcp23017.pinMode(D4, INPUT_PULLUP, true);
  mcp23017.pinMode(D5, INPUT_PULLUP, true);
  mcp23017.pinMode(D6, INPUT_PULLUP, true);
  mcp23017.pinMode(D7, INPUT_PULLUP, true);
}

void configureInterrupts() {
   // Configure MCP23017 interrupts
   mcp23017.interruptMode(MCP23017InterruptMode::Separated);  // INTA and INTB act independently
   
   //mcp23017.interrupt(MCP23017Port::A, RISING); //Data lines will interrupt when any pin goes high

   mcp23017.writeRegister(MCP23017Register::GPINTEN_B, 0x67); //setting GPIOB0-2 and GPIOB5-6 to give interrupts
   mcp23017.writeRegister(MCP23017Register::INTCON_B, 0x07); //Buttons will check against DEFVAL to send interrupt
   mcp23017.writeRegister(MCP23017Register::DEFVAL_B, 0x00); //Set default val to 0 to interrupt on rising edge

   mcp23017.clearInterrupts();  // reset interrupt system
   pinMode(MCP23017_INTB, INPUT_PULLUP);  // utilize microprocessor's internal pull-up resistor
   attachInterrupt(digitalPinToInterrupt(MCP23017_INTB), mcp23017ChangeDetectedOnPortB, FALLING);  // INTB is active LOW

}

void mcp23017ChangeDetectedOnPortB() {
  uint8_t interruptPin = mcp23017.readRegister(MCP23017Register::INTF_B);

  //interrupt from Proximity Sensor
  if(interruptPin == 0x40 || interruptPin == 0x20)
  {
    startTimer = !startTimer;
  }
  else if(interruptPin == 0x01 || interruptPin == 0x02 ||interruptPin == 0x04)
  {
    buttonPressed =true;
  }
}

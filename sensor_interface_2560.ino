/*
  sensor_interface_2560.ino - Software to collect and display automotive sensor data on 16x2 LCD. 
  
  Written for use on Arduino MEGA2560, requires additional hardware to run.
  
  GitHub project page: https://github.com/seanauff/classic-car-sensor-interface
  
  Copyright (C) 2014-2015 Sean Auffinger
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// included library            // source
#include <BigNumbersFast.h>    // https://github.com/seanauff/BigNumbers/tree/Fast 
#include <DallasTemperature.h> // http://www.hacktronics.com/Tutorials/arduino-1-wire-tutorial.html 
#include <DS1307RTC.h>         // http://www.pjrc.com/teensy/td_libs_DS1307RTC.html 
#include <EEPROM.h>            // http://arduino.cc/en/Reference/EEPROM 
#include <Encoder.h>           // https://www.pjrc.com/teensy/td_libs_Encoder.html 
#include <LiquidCrystalFast.h> // https://www.pjrc.com/teensy/td_libs_LiquidCrystal.html 
#include <OneWire.h>           // http://www.hacktronics.com/Tutorials/arduino-1-wire-tutorial.html 
#include <Time.h>              // http://www.pjrc.com/teensy/td_libs_Time.html 
#include <Wire.h>              // http://arduino.cc/en/reference/wire 

// EEPROM memory addresses
const int lcdContrastAddress = 1;
const int lcdBrightnessAddress = 2;
const int lcdHueAddress = 3;
const int useSIAddress = 4;
const int lcdAutoDimAddress = 5;
const int lcdBigFontAddress = 6;
const int engineCylindersAddress = 7;
const int refreshIntervalAddress = 8;
const int displacementAddress1 = 9;
const int displacementAddress2 = 10;
const int engineCyclesAddress = 11;

// interrupts
const byte switchInterrupt = 0; // rotary encoder momentary switch on interrupt 0 (digital pin 2)
const byte tachInterrupt = 1; // tach signal on interrupt 1 (digital pin 3)

// digital pins
const byte switchPin = 2; // rotary encoder momentary switch on digital pin 2 (interrupt 0)
const byte tachPin = 3; // tach signal on digital pin 3 (interrupt 1)
const byte lcdContrastPin = 9; // contrast adjust on digital pin 9 (PWM)
const byte encoderPin1 = 18; // A leg of encoder on digital pin 18 (interrupt 5)
const byte encoderPin2 = 19; // B leg of encoder on digital pin 19 (interrupt 4)
const byte wireSDAPin = 20; // I2C SDA
const byte wireSCLPin = 21; // I2C SCL
const byte factoryResetPin = 22; // held HIGH by internal pullup, short to GND during bootup to reset to factory defualts 
const byte oneWirePin = 23; // data pin for 1-Wire devices (DS18B20)
const byte lcdRSPin = 37; // LCD RS pin
const byte lcdRWPin = 38; // LCD RW pin
const byte lcdEPin = 39; // LCD E pin
const byte lcdD4Pin = 40; // LCD D4 pin
const byte lcdD5Pin = 41; // LCD D5 Pin
const byte lcdD6Pin = 42; // LCD D6 Pin
const byte lcdD7Pin = 43; // LCD D7 pin
const byte lcdLEDRedPin = 44; // control for Red LED (PWM)
const byte lcdLEDGreenPin = 45; // control for Green LED (PWM)
const byte lcdLEDBluePin = 46; // control for Blue LED (PWM)

// analog pins
const byte battVoltagePin = A0; // pin for battery voltage
const byte oilPressPin = A1; // pin for oil pressure
const byte fuelLevelPin = A2; // pin for fuel level
const byte coolantTempPin = A3; // pin for coolant temp
const byte autoDimPin = A4; // pin for external brightness control
const byte AFRatioPin = A5; // pin for LSU 4.9 O2 sensor controller linear output
const byte intakePressPin = A6; // pin for intake manifold pressure (vac or boost)

// analog input setup
const float aRef = 5.0; // analog reference for board (Volts)
const float regVoltage = 5.0; // instrument unit voltage regulator output (Volts)
const float oilGaugeOhms = 13.0; // resistance of oil pressure gauge (ohms)
const float fuelGaugeOhms = 13.0; // resistance of fuel level gauge (ohms)
const float coolantGaugeOhms = 13.0; // resistance of coolant temperature gauge (ohms)

// Steinhart–Hart equation parameters for coolant temp sender
// http://en.wikipedia.org/wiki/Thermistor 
const float SHparamA = 1.869336e-3;
const float SHparamB = 2.723037e-4;
const float SHparamC = 2.833889e-7;

// OneWire setup (DS18B20 Temperature Sensors)
OneWire oneWire(oneWirePin); // create OneWire object
DallasTemperature sensors(&oneWire); // create DallasTemperature object and pass OneWire object to it
// define sensor addresses
DeviceAddress insideTempDigital = {0x28, 0xFF, 0x1B, 0x36, 0x2D, 0x04, 0x00, 0xBA}; // inside (cabin) temperature
DeviceAddress outsideTempDigital = {0x28, 0xFF, 0xDF, 0x33, 0x2B, 0x04, 0x00, 0xD7}; // outside (exterior) temperature
DeviceAddress oilTempDigital = {0x28, 0xFF, 0xB5, 0x36, 0x2D, 0x04, 0x00, 0x2B}; // oil temperature
DeviceAddress intakeTempDigital = {0x28, 0xFF, 0xAF, 0x08, 0x2E, 0x04, 0x00, 0x53}; // intake temperature
DeviceAddress transTempDigital = {0x28, 0xFF, 0xD5, 0x33, 0x2B, 0x04, 0x00, 0x6A}; // tranmission temperature

// Encoder setup
Encoder modeSwitch(encoderPin1, encoderPin2); // create Encoder object

// Display modes setup
const byte modeMin = 1;
const byte modeMax = 16; // number of modes

// normal modes - this sets order
const byte modeClock = 1;
const byte modeBattVoltage = 2;
const byte modeOilPress = 3;
const byte modeCoolantTemp = 4;
const byte modeOutsideTemp = 5;
const byte modeInsideTemp = 6;
const byte modeOilTemp = 7;
const byte modeTransTemp = 8;
const byte modeIntakeTemp = 9;
const byte modeTach = 10;
const byte modeAFRatio = 11;
const byte modeIntakePress = 12;
const byte modeMAFR = 13;
const byte modeFuelLevel = 14;
const byte modeLCDSetup = 15;
const byte modeSystemSetup = 16;

// hidden modes - not in normal rotation (inside menus, etc.)
const byte modeEngineCycles = 90;
const byte modeDisplacement = 91;
const byte modeRefreshInterval = 92;
const byte modeUseSI = 93;
const byte modeEngineCylinders = 94;
const byte modeLCDColor = 95;
const byte modeBigFont = 96;
const byte modeLCDBrightness = 97;
const byte modeLCDContrast = 98;
const byte modeLCDAutoDim = 99;

byte mode = modeClock; // mode to start in
byte previousMode = mode; // keep track of last mode to know when the mode changes to enable an immediate screen update
int modeSwitchPosition; // temporary variable for storing the position of the encoder

// LCD setup
LiquidCrystalFast lcd(lcdRSPin, lcdRWPin, lcdEPin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);
// default LCD values (changes stored in EEPROM)
byte lcdContrast = 80; // scale from 0-100, map to 0-255 for analogWrite()
byte lcdBrightness = 100; // scale from 0-100, map to 0-255 for analogWrite()
byte lcdHue = 72; // scale from 1-120, use setRGBFromHue() to set lcdLEDRed, lcdLEDGreen, lcdLEDBlue
// variables for storing display RGB values calulated from setRGBFromHue()
byte lcdLEDRed; // scale from 0-255
byte lcdLEDGreen; // scale from 0-255
byte lcdLEDBlue; // scale from 0-255
BigNumbersFast bigNum(&lcd); // create BigNumbers object for displaying large numbers

// clock setup
// default values to use if RTC not set
byte currentHour = 0;
byte currentMinute = 0;
byte currentSecond = 0;
byte currentMonth = 1;
byte currentDay = 1;
int currentYear = 2014;

// default personalization values (changes stored in EEPROM)
byte useSI = 0; // unit selector: 0 = SAE, 1 = SI
byte lcdAutoDim = 0; // automatic brightness adjust: 0 = OFF, 1 = ON
byte lcdBigFont = 1; // Big Font: 0 = OFF, 1 = ON
byte engineCylinders = 8; // for tach calculation (pulses per revolution = 2 * cylinders / cycles)
byte engineCycles = 4; // for tach calculation
int displacement = 390; // (units of cu in) for MAFR calculations
int refreshInterval = 750; // milliseconds between sensor updates

// timing values
unsigned long previousMillis = 0; // for sensor refresh interval
unsigned long timeSwitchLastPressed = 0; // for switch debounce
byte switchDebounceInterval = 200; // time in milliseconds where switch press will do nothing

// interrupt volatile variables
volatile int RPMpulses = 0;
volatile boolean buttonPressed = false;

// setup method, runs once at boot
void setup()
{
  pinMode(factoryResetPin, INPUT_PULLUP); // enable internal pullup on factory reset pin
  delayMicroseconds(10); // wait for pin to be pulled high
  // if factory reset pin has been pulled LOW, clear EEPROM and set Time and RTC to default
  if(digitalRead(factoryResetPin) == LOW)
  {
    EEPROM.write(lcdContrastAddress, 255);
    EEPROM.write(lcdBrightnessAddress, 255);
    EEPROM.write(lcdHueAddress, 255);
    EEPROM.write(useSIAddress, 255);
    EEPROM.write(lcdAutoDimAddress, 255);
    EEPROM.write(lcdBigFontAddress, 255);
    EEPROM.write(engineCylindersAddress, 255);
    EEPROM.write(refreshIntervalAddress, 255);
    EEPROM.write(displacementAddress1, 255);
    EEPROM.write(displacementAddress2, 255);
    EEPROM.write(engineCyclesAddress, 255);
    setTime(currentHour, currentMinute, currentSecond, currentDay, currentMonth, currentYear);
    RTC.set(now());
  }
  // load values from EEPROM if they have been changed from default, otherwise, defaults will be used
  if (EEPROM.read(lcdContrastAddress) < 255)
  {
    lcdContrast = EEPROM.read(lcdContrastAddress);
  }
  if (EEPROM.read(lcdBrightnessAddress) < 255)
  {
    lcdBrightness = EEPROM.read(lcdBrightnessAddress);
  }
  if (EEPROM.read(lcdHueAddress) < 255)
  {
    lcdHue = EEPROM.read(lcdHueAddress);
  }
  if (EEPROM.read(useSIAddress) < 255)
  {
    useSI = EEPROM.read(useSIAddress);
  }
  if (EEPROM.read(lcdAutoDimAddress) < 255)
  {
    lcdAutoDim = EEPROM.read(lcdAutoDimAddress);
  }
  if (EEPROM.read(lcdBigFontAddress) < 255)
  {
    lcdBigFont = EEPROM.read(lcdBigFontAddress);
  }
  if (EEPROM.read(engineCylindersAddress) < 255)
  {
    engineCylinders = EEPROM.read(engineCylindersAddress);
  }
  if (EEPROM.read(refreshIntervalAddress) < 255)
  {
    refreshInterval = EEPROM.read(refreshIntervalAddress) * 10; // value is stored in centiseconds (0.01 s) to fit into a byte. Multiple by 10 to get value in milliseconds
  }
  if (EEPROM.read(displacementAddress1) < 255 || EEPROM.read(displacementAddress2) < 255)
  {
    displacement = EEPROM.read(displacementAddress1) * 256 + EEPROM.read(displacementAddress2); // int value is stored as 2 bytes
  }
  if (EEPROM.read(engineCyclesAddress) < 255)
  {
    engineCycles = EEPROM.read(engineCyclesAddress);
  }
  
  // set the internal clock from the RTC
  setSyncProvider(RTC.get);
  // set time to default if RTC not set
  if (timeStatus() != timeSet)
  {
    setTime(currentHour, currentMinute, currentSecond, currentDay, currentMonth, currentYear);
  }
  
  // setup digital pins
  pinMode(switchPin, INPUT_PULLUP); // enable internal pullup for encoder switch pin
  pinMode(tachPin, INPUT_PULLUP); // enable internal pullup for tach pin
  pinMode(lcdContrastPin, OUTPUT); // set lcdContrastPin as OUTPUT
  pinMode(lcdLEDRedPin, OUTPUT); // set lcdLEDRedPin as OUTPUT
  pinMode(lcdLEDGreenPin, OUTPUT); // set lcdLEDGreenPin as OUTPUT
  pinMode(lcdLEDBluePin, OUTPUT); // set lcdLEDBluePin as OUTPUT
  
  Serial.begin(115200); // start serial connection at 115200 baud
  
  lcd.begin(16, 2); // set up lcd with 16 columns, 2 rows
  bigNum.begin(); // set up BigNumbersFast
   
  TCCR2B = TCCR2B & 0b11111000 | 0x01; // sets Timer 2 PWM frequency to 31372.55 Hz (affects digital pins 9 (LCD Contrast) and 10 (unused))
  
  // update LCD display values
  setRGBFromHue();
  writeLCDValues(); 
  
  // setup DS18B20 temperature sensors to 9 bit resolution (0.5 deg C resolution, 93.75 millisecond conversion time)
  sensors.begin();
  sensors.setResolution(insideTempDigital, 9);
  sensors.setResolution(outsideTempDigital, 9);
  sensors.setResolution(oilTempDigital, 9);
  sensors.setResolution(intakeTempDigital, 9);
  sensors.setResolution(transTempDigital, 9);
  sensors.requestTemperatures(); // run first temperature conversion on all sensors
  
  attachInterrupt(switchInterrupt, pressButton, FALLING); // attach interrupt to encoder switch pin to listen for switch presses
  
  modeSwitch.write(mode * 4); // write initial mode to Encoder
}

// main loop, runs continuously after setup()
void loop()
{
  // change LCD parameters if button pressed while displaying LCD setup
  if(mode == modeLCDSetup && buttonPressed)
  {
    buttonPressed = false;
    lcd.clear();
    // change color
    int previousBrightness = lcdBrightness; // store last brightness value
    int previousHue = lcdHue; // store last color value
    modeSwitch.write(lcdHue * 4); // write current lcdHue value to Encoder
    // loop until button pressed
    while(!buttonPressed)
    { 
      modeSwitchPosition = modeSwitch.read();
      if(modeSwitchPosition % 4 == 0)
      {
        lcdHue = modeSwitchPosition / 4; // set lcdHue according to encoder position
        // loop around if out of range
        if(lcdHue > 120) // max values is 120
        {
          lcdHue = 1;
          modeSwitch.write(lcdHue * 4);
        }
        else if (lcdHue < 1) // min value is 1
        {
          lcdHue = 120;
          modeSwitch.write(lcdHue * 4);
        }
        setRGBFromHue(); // determine new RGB values
        lcdBrightness = map((lcdLEDRed + lcdLEDGreen + lcdLEDBlue), 255, 510, 100, 50); // normalize brightness due to varying amounts of LEDs used
        writeLCDValues(); // write new values to LCD
        displayInfo(modeLCDColor, lcdBigFont); // update display
      }
    }
    // store new value in EEPROM if it changed
    if (lcdHue != previousHue)
    {
      EEPROM.write(lcdHueAddress, lcdHue);
    }
    lcdBrightness = previousBrightness; // restore brightness to previous value
    buttonPressed = false;
    lcd.clear();
    // auto dim on/off
    int previousAutoDim = lcdAutoDim;
    modeSwitch.write(lcdAutoDim * 4); // write current state to Encoder
    // loop until button pressed
    while(!buttonPressed)
    {
      lcdAutoDim = modeSwitch.read() / 4; // set value according to encoder position
      // don't go past limits
      if(lcdAutoDim > 1) // max value is 1
      {
        lcdAutoDim = 1;
        modeSwitch.write(lcdAutoDim * 4);
      }
      else if (lcdAutoDim < 0) // min value is 0
      {
        lcdAutoDim = 0;
        modeSwitch.write(lcdAutoDim * 4);
      }
      if (!lcdAutoDim)
      {
        lcdBrightness = previousBrightness; // restore brightness to previous value
      }
      setAutoBrightness();
      writeLCDValues();
      displayInfo(modeLCDAutoDim, lcdBigFont); 
    }
    // store new value in EEPROM if it changed
    if (lcdAutoDim != previousAutoDim)
    {
      EEPROM.write(lcdAutoDimAddress, lcdAutoDim);
    }
    buttonPressed = false;
    lcd.clear();
    // manual brightness
    previousBrightness = lcdBrightness;
    modeSwitch.write(lcdBrightness * 4); // write current state to Encoder
    // loop until button pressed
    // can only change manual brightness if Automatic brightness is OFF
    while(!buttonPressed && !lcdAutoDim)
    { 
      lcdBrightness = modeSwitch.read() / 4;
      // don't go past limits
      if(lcdBrightness > 100) // max values is 100
      {
        lcdBrightness = 100;
        modeSwitch.write(lcdBrightness * 4);
      }
      else if (lcdBrightness < 0) // min value is 0
      {
        lcdBrightness = 0;
        modeSwitch.write(lcdBrightness * 4);
      }
      writeLCDValues();
      displayInfo(modeLCDBrightness, lcdBigFont);
    }
    // store new value in EEPROM if it changed
    if(lcdBrightness != previousBrightness)
    {
      EEPROM.write(lcdBrightnessAddress, lcdBrightness);
    }
    buttonPressed = false;
    lcd.clear();
    // change LCD contrast
    byte previousContrast = lcdContrast;
    modeSwitch.write(lcdContrast * 4);
    // loop until button pressed
    while(!buttonPressed)
    { 
      lcdContrast = modeSwitch.read() / 4;
      // don't go past limits
      if(lcdContrast > 100) // max value is 100
      {
        lcdContrast = 100;
        modeSwitch.write(lcdContrast * 4);
      }
      else if (lcdContrast < 0) // min value is 0
      {
        lcdContrast = 0;
        modeSwitch.write(lcdContrast * 4);
      }
      writeLCDValues();
      displayInfo(modeLCDContrast, lcdBigFont);
    }
    // store new value in EEPROM if it changed
    if (lcdContrast != previousContrast)
    {
      EEPROM.write(lcdContrastAddress, lcdContrast);
    }
    buttonPressed = false;
    lcd.clear();
    // big font on/off
    byte previousBigFont = lcdBigFont;
    modeSwitch.write(lcdBigFont * 4);
    // loop until button pressed
    while(!buttonPressed)
    {
      lcdBigFont = modeSwitch.read() / 4;
      if(lcdBigFont > 1)
      {
        lcdBigFont = 0;
        modeSwitch.write(lcdBigFont * 4);
      }
      else if (lcdBigFont < 0)
      {
        lcdBigFont = 1;
        modeSwitch.write(lcdBigFont * 4);
      }
      displayInfo(modeBigFont, lcdBigFont);
    }
    // store new value in EEPROM if it changed
    if (lcdBigFont != previousBigFont)
    {
      EEPROM.write(lcdBigFontAddress, lcdBigFont);
    }
    buttonPressed = false;
    lcd.clear();
    modeSwitch.write(mode * 4); // write current mode back to Encoder
  }
  // change system parameters if button pressed while displaying System setup
  else if (mode == modeSystemSetup && buttonPressed)
  {
    lcd.clear();
    buttonPressed = false;
    // set number of cylinders in increments of 1
    byte previousCylinders = engineCylinders;
    modeSwitch.write(engineCylinders * 4);
    // loop until button pressed
    while(!buttonPressed)
    {
      engineCylinders = modeSwitch.read() / 4;
      if(engineCylinders < 1)
      {
        engineCylinders = 1;
        modeSwitch.write(engineCylinders * 4);
      }
      else if (engineCylinders > 16)
      {
        engineCylinders = 16;
        modeSwitch.write(engineCylinders * 4);
      }
      displayInfo(modeEngineCylinders, lcdBigFont);
    }
    // store new value in EEPROM if it changed
    if (engineCylinders != previousCylinders)
    {
      EEPROM.write(engineCylindersAddress, engineCylinders);
    }
    buttonPressed = false;
    lcd.clear();
    // set number of cycles in increments of 2
    byte previousCycles = engineCycles;
    modeSwitch.write((engineCycles / 2) * 4);
    // loop until button pressed
    while(!buttonPressed)
    {
      engineCycles = 2 * (modeSwitch.read() / 4);
      if(engineCycles < 2)
      {
        engineCycles = 2;
        modeSwitch.write((engineCycles / 2) * 4);
      }
      else if (engineCycles > 4)
      {
        engineCycles = 4;
        modeSwitch.write((engineCycles / 2) * 4);
      }
      displayInfo(modeEngineCycles, lcdBigFont);
    }
    // store new value in EEPROM if it changed
    if (engineCycles != previousCycles)
    {
      EEPROM.write(engineCyclesAddress, engineCycles);
    }
    buttonPressed = false;
    lcd.clear();
    // change displacement
    int previousDisplacement = displacement;
    modeSwitch.write(displacement * 4);
    // loop until button pressed
    while(!buttonPressed)
    {
      displacement = modeSwitch.read() / 4;
      if(displacement < 100)
      {
        displacement = 100;
        modeSwitch.write(displacement * 4);
      }
      else if (displacement > 599)
      {
        displacement = 599;
        modeSwitch.write(displacement * 4);
      }
      displayInfo(modeDisplacement, lcdBigFont);
    }
    // store new value in EEPROM if it changed
    if (displacement != previousDisplacement)
    {
      // break down int into 2 bytes for EEPROM storage
      EEPROM.write(displacementAddress1, displacement / 256);
      EEPROM.write(displacementAddress2, displacement % 256);
    }
    buttonPressed = false;
    lcd.clear();
    // change unit system
    byte previousUseSI = useSI;
    modeSwitch.write(useSI * 4);
    // loop until button pressed
    while(!buttonPressed)
    {
      useSI = modeSwitch.read() / 4;
      if(useSI < 0)
      {
        useSI = 0;
        modeSwitch.write(useSI * 4);
      }
      else if (useSI > 1)
      {
        useSI = 1;
        modeSwitch.write(useSI * 4);
      }
      displayInfo(modeUseSI, lcdBigFont);
    }
    // store new value in EEPROM if it changed
    if (useSI != previousUseSI)
    {
      EEPROM.write(useSIAddress, useSI);
    }
    buttonPressed = false;
    lcd.clear();
    // change update frequency (refreshInterval) in increments of 50 ms
    int previousRefreshInterval = refreshInterval;
    modeSwitch.write((refreshInterval / 50) * 4);
    while(!buttonPressed)
    {
      refreshInterval = 50 * (modeSwitch.read() / 4);
      if(refreshInterval < 100)
      {
        refreshInterval = 100;
        modeSwitch.write((refreshInterval / 50) * 4);
      }
      else if (refreshInterval > 1500)
      {
        refreshInterval = 1500;
        modeSwitch.write((refreshInterval / 50) * 4);
      }
      displayInfo(modeRefreshInterval, lcdBigFont);
    }
    // store new value in EEPROM if it changed
    if (refreshInterval != previousRefreshInterval)
    {
      EEPROM.write(refreshIntervalAddress, refreshInterval / 10); // divide by 10 so value will fit into a byte
    }
    buttonPressed = false;
    lcd.clear();
    modeSwitch.write(mode * 4); // write current mode back to Encoder
  }
  // set clock if button pressed while displaying clock
  else if (mode == modeClock && buttonPressed)
  {
    lcd.clear();
    buttonPressed = false;
    byte instantHour = hour(); // store current hour
    byte instantMinute = minute(); // store current minute
    byte instantSecond = 0; // reset seconds to 0
    byte instantMonth = month(); // store current month
    byte instantDay = day(); // store current day
    int instantYear = year(); // store current year
    // set Hour
    modeSwitch.write(instantHour * 4); // set Hour
    lcd.setCursor(0, 0);
    lcd.print("     ");
    lcd.setCursor(0, 0);
    lcd.print("Hour");
    // loop until button pressed
    while(!buttonPressed)
    {
      instantHour = modeSwitch.read() / 4;
      if (instantHour < 0)
      {
        instantHour = 23;
        modeSwitch.write(instantHour * 4);
      }
      else if (instantHour > 23)
      {
        instantHour = 0;
        modeSwitch.write(instantHour * 4);
      }
      setTime(instantHour, instantMinute, instantSecond, instantDay, instantMonth, instantYear);
      displayInfo(modeClock, false);
    }
    buttonPressed = false;
    // set Minute
    modeSwitch.write(instantMinute * 4);
    lcd.setCursor(0, 0);
    lcd.print("     ");
    lcd.setCursor(0, 0);
    lcd.print("Min");
    // loop until button pressed
    while(!buttonPressed)
    {
      instantMinute = modeSwitch.read() / 4;
      if (instantMinute < 0)
      {
        instantMinute = 59;
        modeSwitch.write(instantMinute * 4);
      }
      else if (instantMinute > 59)
      {
        instantMinute = 0;
        modeSwitch.write(instantMinute * 4);
      }
      setTime(instantHour, instantMinute, instantSecond, instantDay, instantMonth, instantYear);
      displayInfo(modeClock, false);
    }
    buttonPressed = false;
    // set Month
    modeSwitch.write(instantMonth * 4);
    lcd.setCursor(0, 0);
    lcd.print("     ");
    lcd.setCursor(0, 0);
    lcd.print("Month");
    // loop until button pressed
    while(!buttonPressed)
    {
      instantMonth = modeSwitch.read() / 4;
      if (instantMonth < 1)
      {
        instantMonth = 12;
        modeSwitch.write(instantMonth * 4);
      }
      else if (instantMonth > 12)
      {
        instantMonth = 1;
        modeSwitch.write(instantMonth * 4);
      }
      setTime(instantHour, instantMinute, instantSecond, instantDay, instantMonth, instantYear);
      displayInfo(modeClock, false);
    }
    buttonPressed = false;
    // set Day
    modeSwitch.write(instantDay * 4);
    lcd.setCursor(0, 0);
    lcd.print("     ");
    lcd.setCursor(0, 0);
    lcd.print("Day");
    // loop until button pressed
    while(!buttonPressed)
    {
      instantDay = modeSwitch.read() / 4;
      if (instantDay < 1)
      {
        instantDay = 31;
        modeSwitch.write(instantDay * 4);
      }
      else if (instantDay > 31)
      {
        instantDay = 1;
        modeSwitch.write(instantDay * 4);
      }
      setTime(instantHour, instantMinute, instantSecond, instantDay, instantMonth, instantYear);
      displayInfo(modeClock, false);
    }
    buttonPressed = false;
    // set Year
    modeSwitch.write(instantYear * 4);
    lcd.setCursor(0, 0);
    lcd.print("     ");
    lcd.setCursor(0, 0);
    lcd.print("Year");
    // loop until button pressed
    while(!buttonPressed)
    {
      instantYear = modeSwitch.read() / 4;
      if (instantYear < 2000)
      {
        instantYear = 2100;
        modeSwitch.write(instantYear * 4);
      }
      else if (instantYear > 2100)
      {
        instantYear = 2000;
        modeSwitch.write(instantYear * 4);
      }
      setTime(instantHour, instantMinute, instantSecond, instantDay, instantMonth, instantYear);
      displayInfo(modeClock, false);
    }
    RTC.set(now()); // update DS1307 RTC with new Time
    buttonPressed = false;
    lcd.clear();
    modeSwitch.write(mode * 4); // write current mode back to Encoder
  }
  
  // switch temp and pressure units
  else if ((mode==modeCoolantTemp || mode==modeInsideTemp || mode==modeOutsideTemp || mode==modeTransTemp || mode==modeOilTemp || mode==modeIntakeTemp || mode==modeOilPress || mode==modeIntakePress || mode==modeMAFR) && buttonPressed==true)
  {
    previousMillis = 0; // forces an immediate value update
    buttonPressed = false; 
    if(useSI)
    {
      useSI = 0;
    }
    else
    {
      useSI = 1;
    }
    EEPROM.write(useSIAddress, useSI); // store new value to EEPROM
  }
  
  // if no button pressed, read Encoder and change modes
  else
  {
    modeSwitchPosition = modeSwitch.read();
    if(modeSwitchPosition % 4 == 0)
    {
      mode = modeSwitchPosition / 4; // read encoder position and set mode
      // loop around to first mode if reached the end
      if (mode > modeMax)
      {
        mode = modeMin;
        modeSwitch.write(mode * 4);
      }
      // loop around to end mode if reached the beginning
      else if (mode < modeMin)
      {
        mode = modeMax;
        modeSwitch.write(mode * 4);
      }
      // check to see if new display mode is selected
      if(mode != previousMode)
      {
        previousMillis = 0; // this will force immediate refresh instead of waiting for normal sensor update interval
        previousMode = mode; // store new previous mode
        lcd.clear();
        // start or stop counting interrupts for engine speed
        // only count interrrupts while showing the tach or MAFR
        if(mode == modeTach || mode == modeMAFR)
        {
          attachInterrupt(tachInterrupt, countRPM, FALLING);
        }
        else
        {
          detachInterrupt(tachInterrupt);
        }     
        buttonPressed = false; // reset momentary button in case it was not
      }
    }
  }
  
  // refresh display if enough time has passed
  if(millis() - previousMillis > refreshInterval)
  {
    previousMillis = millis();
    setAutoBrightness(); 
    writeLCDValues(); // write LCD values
    // display info depending on mode and font size
    // output data to serial connection
    Serial.println(displayInfo(mode, lcdBigFont));
  }
}

// runs color conversion algorithm to set individual RGB values from single Hue value
// http://en.wikipedia.org/wiki/HSL_and_HSV
void setRGBFromHue()
{
  int hue = lcdHue;
  if(hue <= 20)
  {
    lcdLEDRed = 255;
    lcdLEDGreen = map(hue, 0, 20, 0, 255);
    lcdLEDBlue = 0;
  }
  else if (hue <= 40)
  {
    lcdLEDRed = map(hue, 20, 40, 255, 0);
    lcdLEDGreen = 255;
    lcdLEDBlue = 0;
  }
  else if (hue <= 60)
  {
    lcdLEDRed = 0;
    lcdLEDGreen = 255;
    lcdLEDBlue = map(hue, 40, 60, 0, 255);
  }
  else if (hue <= 80)
  {
    lcdLEDRed = 0;
    lcdLEDGreen = map(hue, 60, 80, 255, 0);
    lcdLEDBlue = 255;
  }
  else if (hue <= 100)
  {
    lcdLEDRed = map(hue, 80, 100, 0, 255);
    lcdLEDGreen = 0;
    lcdLEDBlue = 255;
  }
  else if (hue <= 120)
  {
    lcdLEDRed = 255;
    lcdLEDGreen = 0;
    lcdLEDBlue = map(hue, 100, 120, 255, 0);
  }
}

// reads voltage supplied to instrument panel lights and changes LCD brightness accordingly
void setAutoBrightness()
{
  // only do this if feature is enabled in setttings
  if(lcdAutoDim)
  {
    lcdBrightness = map(analogRead(autoDimPin), 0, 1023, 20, 100);
  }
}

// updates LCD display values
void writeLCDValues()
{
  // contrast
  analogWrite(lcdContrastPin, map(lcdContrast, 0, 100, 255, 0));
  
  // modify RBG values based on current brightness
  int r = map(lcdLEDRed, 0, 255, 0, lcdBrightness);
  int g = map(lcdLEDGreen, 0, 255, 0, lcdBrightness);
  int b = map(lcdLEDBlue, 0, 255, 0, lcdBrightness);
  // set PWM values accordingly
  analogWrite(lcdLEDRedPin, map(r, 0, 100, 0, 255));
  analogWrite(lcdLEDGreenPin, map(g, 0, 100, 0, 200)); // compensate for LED brightness differences
  analogWrite(lcdLEDBluePin, map(b, 0, 100, 0, 200)); // compensate for LED brightness differences
}

// reads input voltage
// returns battery voltage in Volts
// resolution: 0.0176 V = 17.6 mV
float getBattVoltage()
{
  // Voltage divider maps 18 V to 5 V
  float R1 = 100000.0; // value of R1 in voltage divider (ohms)
  float R2 = 38300.0; // value of R2 in voltage divider (ohms)
  // take 10 readings and sum them
  int val = 0;
  for (int i = 1; i <= 10; i++)
  {
    val += analogRead(battVoltagePin);
  }
  val += 5; // allows proper rounding due to using integer math
  val /= 10; // get average value
  float Vout = (val / 1023.0) * aRef; // convert 10-bit value to Voltage
  float Vin = Vout * (R1 + R2) / R2; // solve for input Voltage
  return Vin; // return calculated input Voltage
}

// reads voltage between oil pressure gauage and sensor
// returns oil pressure in psi
// Resolution:
float getOilPress()
{
  // Voltage divider maps 6.1 V to 5 V
  float R1 = 22000.0; // value of R1 in voltage divider (ohms)
  float R2 = 100000.0; // value of R2 in voltage divider (ohms)
  // take 10 readings and sum them
  int val = 0;
  for (int i = 1; i <= 10; i++)
  {
    val += analogRead(oilPressPin);
  }
  val += 5; // allows proper rounding due to using integer math
  val /= 10; // get average value
  float Vout = val / 1023.0 * aRef; // convert 10-bit value to Voltage
  float VI = Vout * (R1 + R2) / R2; // solve for input Voltage
  float Rsender = VI * oilGaugeOhms / (regVoltage - VI); // solve for sensor resistance
  float pressure = 140.75 - 1.0199 * Rsender + 0.0018 * Rsender * Rsender; // solve for pressure based on calibration curve
  pressure = constrain(pressure, 0, 999); // deal with any errors that would cause value to be way outside of normal range
  return pressure; // return pressure in psi
}

// reads voltage between fuel level gauge and sensor
// returns Fuel Level in %
// Resolution:
float getFuelLevel()
{
  // Voltage divider maps 6.1 V to 5 V
  float R1 = 22000.0; // value of R1 in voltage divider (ohms)
  float R2 = 100000.0; // value of R2 in voltage divider (ohms)
  // take 10 readings and sum them
  int val = 0;
  for (int i = 1; i <= 10; i++)
  {
    val += analogRead(oilPressPin);
  }
  val += 5; // allows proper rounding due to using integer math
  val /= 10; // get average value
  float Vout = val / 1023.0 * aRef; // convert 10-bit value to Voltage
  float VI = Vout * (R1 + R2) / R2; // solve for input Voltage
  float Rsender = VI * fuelGaugeOhms / (regVoltage - VI); // solve for sensor resistance
  float level = 108.4 - 0.56 * Rsender; // solve for fuel level based on calibration curve
  level = constrain(level, 0, 100); // constrain level to between 0 and 100 (inclusive)
  return level; // return fuel level in %
}

// reads voltage between coolant temp gauge and sensor
// returns coolant temp in deg C
// Resolution:
float getCoolantTemp()
{
  // Voltage divider maps 6.1 V to 5 V
  float R1 = 22000.0; // value of R1 in voltage divider (ohms)
  float R2 = 100000.0; // value of R2 in voltage divider (ohms)
  // take 10 readings and sum them
  int val = 0;
  for (int i = 1; i <= 10; i++)
  {
    val += analogRead(coolantTempPin);
  }
  val += 5; // allows proper rounding due to using integer math
  val /= 10; // get average value
  val = max(val, 1); // dont let val be 0, which would give infinite temp
  float Vout = val / 1023.0 * aRef; // convert 10-bit value to Voltage
  float VI = Vout * (R1 + R2) / R2; // solve for input Voltage
  float Rsender = VI * coolantGaugeOhms / (regVoltage - VI); // solve for sensor resistance
  float temp = pow(SHparamA + SHparamB * log(Rsender) + SHparamC * pow(log(Rsender), 3), -1) - 273.15; // solve for temperature based on calibration curve based on Steinhart–Hart equation
  temp = constrain(temp, -99, 999); // deal with any errors that would cause value to be way outside of normal range
  return temp; // return temp in deg C
}

// reads analog output of LSU 4.9 O2 sensor driver (0-5 V)
// 0 V = 0.68 lambda
// 5 V = 1.36 lambda
// returns Lambda value (dimensionless)
// resolution: 0.000665 = 0.01 A/F
float getLambda()
{
  // take 10 readings and sum them
  int val = 0;
  for (int i = 1; i <= 10; i++)
  {
    val += analogRead(AFRatioPin);
  }
  val += 5; // allows proper rounding due to using integer math
  val /= 10; // get average value
  float lambda = (map(val, 0, 1023, 680, 1360)) / 1000.0; // calculate lambda value avoiding limitations of map()
  return lambda;
}

// reads analog output of intake manifold pressure sensor
// returns pressure in psi (relative to 1 atm (14.7 psia), boost is positive, vac is negative)
// resolution: Vacuum = 0.142 psi, Boost = 0.071 psi
float getIntakePress()
{
  // take 10 readings and sum them
  int val = 0;
  for (int i = 1; i <= 10; i++)
  {
    val += analogRead(intakePressPin);
  }
  val += 5; // allows proper rounding due to using integer math
  val /= 10; // get average value
  float volts = val / 1023.0 * aRef;
  float pressure = 0;
  if (volts <= 1.0)
  {
    pressure = 29.0 * volts - 29.0; // vac
  }
  else
  {
    pressure = 14.5 * volts - 14.5; // boost
  }
  return pressure;
}

// checks accumulated tach signal pulses and calculates engine speed
// returns engine speed in RPM
// Resolution: 30000 * engineCycles / refreshInterval / engineCylinders RPM (for default values = 20 RPM)
int getRPM()
{
  int RPM = int(RPMpulses * (60000.0 / float(refreshInterval)) * engineCycles / engineCylinders / 2.0 ); // calculate RPM
  RPMpulses = 0; // reset pulse count to 0
  RPM = min(9999, RPM); // don't return value larger than 9999
  return RPM;
}

// gets data needed to calculate mass (air) flow rate: intake temp, intake pressure, RPM, engine displacement
// returns mass (air) flow rate in kg/min
// Resolution:
float getMAFR()
{
  int currentRPM = getRPM();
  float VE = 0.9; // volumetric efficiency (dependent on intake pressure and engine speed, assumed constant for now)
  sensors.requestTemperatures();
  float intakeTempAbsolute = sensors.getTempC(intakeTempDigital) + 273.15; // intake temp in Kelvin
  float coolantTempAbsolute = getCoolantTemp() + 273.15; // coolant temp in Kelvin
  float intakePressureAbsolute = (getIntakePress() + 14.7) * 0.06895; // intake pressure in bar (absolute)
  float airDensity = intakePressureAbsolute * 29.0 / 8.314e-2 / (0.5 * intakeTempAbsolute + 0.5 * coolantTempAbsolute); // calculate air density in g/L. Relative weightings of intake and engine temperature will change with engine speed. Assumed constant 50/50 for now
  float MAFR = currentRPM * displacement / 61.024 / 2.0 * VE * airDensity / 1000; // calculate MAFR in kg/min
  return MAFR;
}

// Interrupt Service Routine
// counts tach pulses
void countRPM()
{
  RPMpulses++;
}

// Interrupt Service Routine
// handles Encoder button press
void pressButton()
{
  // ignores multiple button presses in quick succession (interval defined above: switchDebounceInterval)
  if(millis() - timeSwitchLastPressed > switchDebounceInterval)
  {
    buttonPressed = true;
    timeSwitchLastPressed = millis();
  }
}

// collects and displays sensor data on the LCD
// returns: the value that it displayed
// byte displayMode: the mode to display
// boolean bigFont: sets if small or large font is used
float displayInfo(byte displayMode, boolean bigFont)
{
  switch (displayMode)
  {
    case modeClock: // Clock
    {
      time_t currentTime = now(); // store current time as time_t
      currentHour = hourFormat12(currentTime);
      currentMinute = minute(currentTime);
      currentMonth = month(currentTime);
      currentDay = day(currentTime);
      currentYear = year(currentTime);
      if (bigFont)
      {
        bigNum.displayLargeInt(currentHour, 0 , 2, false);
        lcd.setCursor(6, 0);
        lcd.write(165); // dot symbol
        lcd.setCursor(6, 1);
        lcd.write(165); // dot symbol
        bigNum.displayLargeInt(currentMinute, 7, 2, true);
        lcd.setCursor(14, 1);
        if(isAM(currentTime))
        {
          lcd.write(65); // "A"
        }
        else
        {
          lcd.write(80); // "P"
        }
        lcd.write(77); // "M"
      }
      else
      {
        lcd.setCursor(4, 0);
        if(currentHour < 10)
        {
          lcd.print(" ");
        }
        lcd.print(currentHour);
        lcd.print(":");
        if(currentMinute < 10)
        {
          lcd.print("0");
        }
        lcd.print(currentMinute);
        lcd.print(" ");
        if(isAM(currentTime))
        {
          lcd.write(65); // "A"
        }
        else
        {
          lcd.write(80); // "P"
        }
        lcd.write(77); // "M"
        lcd.setCursor(0, 1);
        lcd.print(dayShortStr(weekday(currentTime)));
        lcd.print(" ");
        lcd.print(monthShortStr(currentMonth));
        lcd.print(" ");
        if(currentDay < 10)
        {
          lcd.print(" ");
        }
        lcd.print(currentDay);
        lcd.print(" ");
        lcd.print(currentYear);
      }
      return(millis()); // return time since boot
      break;
    }
    case modeBattVoltage: // battery voltage
    {
      float battVoltage = getBattVoltage();
      if (bigFont)
      {
        int battVoltageInt = int(battVoltage * 10 + 0.5);
        byte lastDigit = battVoltageInt % 10;
        battVoltageInt /= 10;
        bigNum.displayLargeInt(battVoltageInt, 0, 2, false);
        lcd.setCursor(9, 0);
        lcd.print("Battery");
        lcd.setCursor(6, 1);
        lcd.print(".");
        lcd.print(lastDigit);
        lcd.print(" Volts");
      }
      else
      {
        lcd.setCursor(0, 0);
        lcd.print("Battery Voltage");
        lcd.setCursor(3, 1);
        displaySmallFloat(battVoltage, 1);
        lcd.print(" V");
      }
      return battVoltage;
      break;
    }
    case modeOilPress: // oil pressure
    {
      float oilPressure = getOilPress();
      if (useSI)
      {
        oilPressure *= 0.06895; // convert to bar
      }
      if (bigFont)
      {
        int oilPressureInt = int(oilPressure * 10 + 0.5);
        byte lastDigit = oilPressureInt % 10;
        oilPressureInt /= 10;
        bigNum.displayLargeInt(oilPressureInt, 0, 3, false);
        lcd.setCursor(12, 0);
        lcd.print("Oil");
        lcd.setCursor(9, 1);
        lcd.print(".");
        lcd.print(lastDigit);
      }
      else
      {
        lcd.setCursor(2, 0);
        lcd.print("Oil Pressure");
        lcd.setCursor(3, 1);
        displaySmallFloat(oilPressure, 1);
      }
      if (useSI)
      {
        lcd.print(" bar");
      }
      else
      {
        lcd.print(" psi");
      }
      return oilPressure;
      break;
    }
    case modeCoolantTemp: // coolant temp
    {
      float currentTemp = getCoolantTemp();
      if (!useSI)
      {
        currentTemp = currentTemp * 9.0 / 5.0 + 32.0; // convert to deg F
      }
      if (bigFont)
      {
        boolean isNegative = false;
        if(currentTemp < 0)
        {
          isNegative = true;
          currentTemp = abs(currentTemp);
        }
        int currentTempInt = int(currentTemp + 0.5);
        bigNum.displayLargeInt(currentTempInt, 0, 3, false);
        lcd.setCursor(10, 0);
        lcd.print("Engine");
        lcd.setCursor(10, 1);
        lcd.write(223);
        if (useSI)
        {
          lcd.write(67); // "C"
        }
        else
        {
          lcd.write(70); // "F"
        }
        if (isNegative)
        {
          lcd.setCursor(0, 0);
          if (currentTempInt < 2 && currentTempInt >= 1)
          {
            lcd.print("      ");
          }
          else if (currentTempInt < 10)
          {
            lcd.print("    ");
          }
          else if (currentTemp < 20)
          {
            lcd.print("   ");
          }
          else if (currentTemp < 100)
          {
            lcd.print(" ");
          }
          lcd.print("-");
          return -currentTemp;
        }
      }
      else
      {
        lcd.setCursor(2, 0);
        lcd.print("Coolant Temp");
        lcd.setCursor(4, 1);
        displaySmallFloat(currentTemp, 0);
        lcd.print(" ");
        lcd.write(223); // degree symbol
        if (useSI)
        {
          lcd.write(67); // "C"
        }
        else
        {
          lcd.write(70); // "F"
        }
      }
      return currentTemp;
      break;
    }
    case modeOutsideTemp: // outside temp
    {
      float currentTemp = sensors.getTempC(outsideTempDigital);
      if (!useSI)
      {
        currentTemp = currentTemp * 9.0 / 5.0 + 32.0; // comvert to deg F
      }
      if (bigFont)
      { 
        boolean isNegative = false;
        if(currentTemp < 0)
        {
          isNegative = true;
          currentTemp = abs(currentTemp);
        }
        int currentTempInt = int(currentTemp + 0.5);
        bigNum.displayLargeInt(currentTempInt, 0, 3, false);
        lcd.setCursor(10, 0);
        lcd.print("Out");
        lcd.setCursor(10, 1);
        lcd.write(223);
        if (useSI)
        {
          lcd.write(67); // "C"
        }
        else
        {
          lcd.write(70); // "F"
        }
        if (isNegative)
        {
          lcd.setCursor(0, 0);
          if (currentTempInt < 2 && currentTempInt >= 1)
          {
            lcd.print("      ");
          }
          else if (currentTempInt < 10)
          {
            lcd.print("    ");
          }
          else if (currentTemp < 20)
          {
            lcd.print("   ");
          }
          else if (currentTemp < 100)
          {
            lcd.print(" ");
          }
          lcd.print("-");
          sensors.requestTemperatures(); // request a temperature conversion
          return -currentTemp;
        }
      }
      else
      {
        lcd.setCursor(2, 0);
        lcd.print("Outside Temp");
        lcd.setCursor(4, 1);
        displaySmallFloat(currentTemp, 0);
        lcd.print(" ");
        lcd.write(223); // degree symbol
        if (useSI)
        {
          lcd.write(67); // "C"
        }
        else
        {
          lcd.write(70); // "F"
        }
      }
      sensors.requestTemperatures(); // request a temperature conversion
      return currentTemp;
      break;
    }
    case modeInsideTemp: // inside temp
    {
      float currentTemp = sensors.getTempC(insideTempDigital);
      if (!useSI)
      {
        currentTemp = currentTemp * 9.0 / 5.0 + 32.0;
      }
      if (bigFont)
      {
        boolean isNegative = false;
        if(currentTemp < 0)
        {
          isNegative = true;
          currentTemp = abs(currentTemp);
        }
        int currentTempInt = int(currentTemp + 0.5);
        bigNum.displayLargeInt(currentTempInt, 0, 3, false);
        lcd.setCursor(10, 0);
        lcd.print("Cabin");
        lcd.setCursor(10, 1);
        lcd.write(223);
        if (useSI)
        {
          lcd.write(67); // "C"
        }
        else
        {
          lcd.write(70); // "F"
        }
        if (isNegative)
        {
          lcd.setCursor(0, 0);
          if (currentTempInt < 2 && currentTempInt >= 1)
          {
            lcd.print("      ");
          }
          else if (currentTempInt < 10)
          {
            lcd.print("    ");
          }
          else if (currentTemp < 20)
          {
            lcd.print("   ");
          }
          else if (currentTemp < 100)
          {
            lcd.print(" ");
          }
          lcd.print("-");
          sensors.requestTemperatures(); // request a temperature conversion
          return -currentTemp;
        }
      }
      else
      {
        lcd.setCursor(2, 0);
        lcd.print("Inside Temp");
        lcd.setCursor(4, 1);
        displaySmallFloat(currentTemp, 0);
        lcd.print(" ");
        lcd.write(223); // degree symbol
        if (useSI)
        {
          lcd.write(67); // "C"
        }
        else
        {
          lcd.write(70); // "F"
        }
      }
      sensors.requestTemperatures(); // request a temperature conversion
      return currentTemp;
      break;
    }
    case modeOilTemp: // oil temp
    {
      float currentTemp = sensors.getTempC(oilTempDigital);
      if (!useSI)
      {
        currentTemp = currentTemp * 9.0 / 5.0 + 32.0; // convert to deg F
      }
      if (bigFont)
      {
        boolean isNegative = false;
        if(currentTemp < 0)
        {
          isNegative = true;
          currentTemp = abs(currentTemp);
        }
        int currentTempInt = int(currentTemp + 0.5);
        bigNum.displayLargeInt(currentTempInt, 0, 3, false);
        lcd.setCursor(10, 0);
        lcd.print("Oil");
        lcd.setCursor(10, 1);
        lcd.write(223);
        if (useSI)
        {
          lcd.write(67); // "C"
        }
        else
        {
          lcd.write(70); // "F"
        }
        if (isNegative)
        {
          lcd.setCursor(0, 0);
          if (currentTempInt < 2 && currentTempInt >= 1)
          {
            lcd.print("      ");
          }
          else if (currentTempInt < 10)
          {
            lcd.print("    ");
          }
          else if (currentTemp < 20)
          {
            lcd.print("   ");
          }
          else if (currentTemp < 100)
          {
            lcd.print(" ");
          }
          lcd.print("-");
          sensors.requestTemperatures(); // request a temperature conversion
          return -currentTemp;
        }
      }
      else
      {
        lcd.setCursor(4, 0);
        lcd.print("Oil Temp");
        lcd.setCursor(4, 1);
        displaySmallFloat(currentTemp, 0);
        lcd.print(" ");
        lcd.write(223); // degree symbol
        if (useSI)
        {
          lcd.write(67); // "C"
        }
        else
        {
          lcd.write(70); // "F"
        }
      }
      sensors.requestTemperatures(); // request a temperature conversion
      return currentTemp;
      break;
    }
    case modeTransTemp: // transmission temp
    {
      float currentTemp = sensors.getTempC(transTempDigital);
      if (!useSI)
      {
        currentTemp = currentTemp * 9.0 / 5.0 + 32.0; // convert to deg F
      }
      if (bigFont)
      {
        boolean isNegative = false;
        if(currentTemp < 0)
        {
          isNegative = true;
          currentTemp = abs(currentTemp);
        }
        int currentTempInt = int(currentTemp + 0.5);
        bigNum.displayLargeInt(currentTempInt, 0, 3, false);
        lcd.setCursor(10, 0);
        lcd.print("Trans.");
        lcd.setCursor(10, 1);
        lcd.write(223);
        if (useSI)
        {
          lcd.write(67); // "C"
        }
        else
        {
          lcd.write(70); // "F"
        }
        if (isNegative)
        {
          lcd.setCursor(0, 0);
          if (currentTempInt < 2 && currentTempInt >= 1)
          {
            lcd.print("      ");
          }
          else if (currentTempInt < 10)
          {
            lcd.print("    ");
          }
          else if (currentTemp < 20)
          {
            lcd.print("   ");
          }
          else if (currentTemp < 100)
          {
            lcd.print(" ");
          }
          lcd.print("-");
          sensors.requestTemperatures(); // request a temperature conversion
          return -currentTemp;
        }
      }
      else
      {
        lcd.setCursor(2, 0);
        lcd.print("Trans. Temp");
        lcd.setCursor(4, 1);
        displaySmallFloat(currentTemp, 0);
        lcd.print(" ");
        lcd.write(223); // degree symbol
        if (useSI)
        {
          lcd.write(67); // "C"
        }
        else
        {
          lcd.write(70); // "F"
        }
      }
      sensors.requestTemperatures();
      return currentTemp;
      break;
    }
    case modeIntakeTemp: // intake temp
    {
      float currentTemp = sensors.getTempC(intakeTempDigital);
      if (!useSI)
      {
        currentTemp = currentTemp * 9.0 / 5.0 + 32.0; // convert to deg F
      }
      if (bigFont)
      {
        boolean isNegative = false;
        if(currentTemp < 0)
        {
          isNegative = true;
          currentTemp = abs(currentTemp);
        }
        int currentTempInt = int(currentTemp + 0.5);
        bigNum.displayLargeInt(currentTempInt, 0, 3, false);
        lcd.setCursor(10, 0);
        lcd.print("Intake");
        lcd.setCursor(10, 1);
        lcd.write(223); // degree symbol
        if (useSI)
        {
          lcd.write(67); // "C"
        }
        else
        {
          lcd.write(70); // "F"
        }
        if (isNegative)
        {
          lcd.setCursor(0, 0);
          if (currentTempInt < 2 && currentTempInt >= 1)
          {
            lcd.print("      ");
          }
          else if (currentTempInt < 10)
          {
            lcd.print("    ");
          }
          else if (currentTemp < 20)
          {
            lcd.print("   ");
          }
          else if (currentTemp < 100)
          {
            lcd.print(" ");
          }
          lcd.print("-");
          sensors.requestTemperatures(); // request a temperature conversion
          return -currentTemp;
        }
      }
      else
      {
        lcd.setCursor(2, 0);
        lcd.print("Intake Temp");
        lcd.setCursor(4, 1);
        displaySmallFloat(currentTemp, 0);
        lcd.print(" ");
        lcd.write(223); // degree symbol
        if (useSI)
        {
          lcd.write(67); // "C"
        }
        else
        {
          lcd.write(70); // "F"
        }
      }
      sensors.requestTemperatures(); // request a temperature conversion
      return currentTemp;
      break;
    }
    case modeTach: // engine (angular) speed
    {
      int currentRPM = getRPM();
      if (bigFont)
      {
        lcd.setCursor(13, 1);
        lcd.print("RPM");
        bigNum.displayLargeInt(currentRPM, 0, 4, false);
      }
      else
      {
        lcd.setCursor(2, 0);
        lcd.print("Engine Speed");
        lcd.setCursor(4, 1);
        displaySmallInt(currentRPM);
        lcd.print(" RPM");
      }
      return currentRPM;
      break;
    }
    case modeAFRatio:
    {
      float currentAFRatio = getLambda() * 14.7; // convert to Air/Fuel Mass Ratio (Air/Gasoline)
      if (bigFont)
      {
        int currentAFRatioInt = int(currentAFRatio * 10 + 0.5);
        byte lastDigit = currentAFRatioInt % 10;
        currentAFRatioInt /= 10;
        bigNum.displayLargeInt(currentAFRatioInt, 0, 2, false);
        lcd.setCursor(9, 0);
        lcd.print("A/F");
        lcd.setCursor(6, 1);
        lcd.print(".");
        lcd.print(lastDigit);
        lcd.print(" Ratio");
      }
      else
      {
        lcd.setCursor(3, 0);
        lcd.print("A/F Ratio");
        lcd.setCursor(5, 1);
        lcd.print(currentAFRatio, 1);
      }
      return currentAFRatio;
      break;
    }
    case modeIntakePress: // intake pressure (MAP)
    {
      float currentIntakePress = getIntakePress();
      if (useSI)
      {
        currentIntakePress *= 0.06895; // convert to bar
      }
      if (bigFont)
      {
        boolean isNegative = false;
        if(currentIntakePress < 0.05)
        {
          isNegative = true;
          currentIntakePress = abs(currentIntakePress);
        }
        int currentIntakePressInt = int(currentIntakePress * 10 + 0.5);
        byte lastDigit = currentIntakePressInt % 10;
        currentIntakePressInt /= 10;
        bigNum.displayLargeInt(currentIntakePressInt, 0, 2, false);
        lcd.setCursor(9, 0);
        if (isNegative)
        {
          lcd.print("Vacuum");
        }
        else
        {
          lcd.print("Boost ");
        }
        lcd.setCursor(6, 1);
        lcd.print(".");
        lcd.print(lastDigit);
        if (useSI)
        {
          lcd.print(" bar");
        }
        else
        {
          lcd.print(" psi");
        }
        if (isNegative)
        {
          lcd.setCursor(0, 0);
          if (currentIntakePressInt < 2 && currentIntakePressInt >= 1)
          {
            lcd.print("   ");
          }
          else if (currentIntakePressInt < 10)
          {
            lcd.print(" ");
          }
          lcd.print("-");
          return -currentIntakePress;
        }
      }
      else
      {
        lcd.setCursor(5, 0);
        if (currentIntakePress <= 0)
        {
          lcd.print("Vacuum");
        }
        else
        {
          lcd.print("Boost ");
        }
        lcd.setCursor(3, 1);
        displaySmallFloat(currentIntakePress, 1);
        if (useSI)
        {
          lcd.print(" bar");
        }
        else
        {
          lcd.print(" psi");
        }
      }
      return currentIntakePress;
      break;
    }
    case modeMAFR:
    {
      float currentMAFR = getMAFR();
      if (!useSI)
      {
        currentMAFR *= 2.205; // convert to lb/min
      }
      if (bigFont)
      {
        lcd.setCursor(11, 0);
        lcd.print("MAFR");
        int currentMAFRInt = int(currentMAFR * 10 + 0.5);
        byte lastDigit = currentMAFRInt % 10;
        currentMAFRInt /= 10;
        bigNum.displayLargeInt(currentMAFRInt, 0, 2, false);
        lcd.setCursor(6, 1);
        lcd.print(".");
        lcd.print(lastDigit);
        lcd.print(" "); 
      }
      else
      {
        lcd.setCursor(1, 0);
        lcd.print("Mass air flow");
        lcd.setCursor(2, 1);  
        displaySmallFloat(currentMAFR, 1);
      }
      if (useSI)
      {
        lcd.print(" kg/min");
      }
      else
      {
        lcd.print(" lb/min");
      }
      return currentMAFR;
      break;
    }
    case modeFuelLevel: // Fuel Level
    {
      float fuelLevel = getFuelLevel();
      if (bigFont)
      {
        int fuelLevelInt = int(fuelLevel + 0.5); // convert to int with rounding
        bigNum.displayLargeInt(fuelLevel, 0, 3, false);
        lcd.setCursor(10, 0);
        lcd.print("Fuel");
        lcd.setCursor(9, 1);
      }
      else
      {
        lcd.setCursor(3, 0);
        lcd.print("Fuel Level");
        lcd.setCursor(4, 1);
        displaySmallFloat(fuelLevel, 0);
      }
      lcd.print(" %");
      return fuelLevel;
      break;
    }
    case modeLCDSetup: // Display Settings
    {
      lcd.setCursor(4, 0);
      lcd.print("Display");
      lcd.setCursor(4, 1);
      lcd.print("Settings");
      return 0;
      break;
    }
    case modeSystemSetup: // System Settings
    {
      lcd.setCursor(5, 0);
      lcd.print("System");
      lcd.setCursor(4, 1);
      lcd.print("Settings");
      return 0;
      break;
    }
    case modeEngineCycles:
    {
      if (bigFont)
      {
        lcd.setCursor(8, 0);
        lcd.print("Engine");
        lcd.setCursor(8, 1);
        lcd.print("Cycles");
        bigNum.displayLargeInt(engineCycles, 0, 2, false);
      }
      else
      {
        lcd.setCursor(0, 0);
        lcd.print("Engine Cylinders");
        lcd.setCursor(4, 1);
        displaySmallInt(engineCylinders);
      }
      return engineCycles;
      break;
    }
    case modeDisplacement:
    {
      lcd.setCursor(2, 0);
      lcd.print("Displacement");
      lcd.setCursor(1, 1);
      lcd.print(displacement);
      lcd.print(" ci  ");
      lcd.print(displacement / 61.024, 2);
      lcd.print(" L");
      return displacement;
      break;
    }
    case modeRefreshInterval:
    {
      lcd.setCursor(0, 0);
      lcd.print("Update Interval");
      lcd.setCursor(4, 1);
      displaySmallInt(refreshInterval);
      lcd.print(" ms");
      return refreshInterval;
      break;
    }
    case modeUseSI: // unit system
    {
      lcd.setCursor(2, 0);
      lcd.print("Unit System");
      lcd.setCursor(6, 1);
      if(useSI)
      {
        lcd.print(" SI");
      }
      else
      {
        lcd.print("SAE");
      }
      return useSI;
      break;
    }
    case modeEngineCylinders:
    {
      if (bigFont)
      {
        lcd.setCursor(8, 0);
        lcd.print("Engine");
        lcd.setCursor(7, 1);
        lcd.print("Cylinders");
        bigNum.displayLargeInt(engineCylinders, 0, 2, false);
      }
      else
      {
        lcd.setCursor(0, 0);
        lcd.print("Engine Cylinders");
        lcd.setCursor(4, 1);
        displaySmallInt(engineCylinders);
      }
      return engineCylinders;
      break;
    }
    case modeLCDColor:
    {
      if (bigFont)
      {
        lcd.setCursor(10, 1);
        lcd.print("Color");
        bigNum.displayLargeInt(lcdHue, 0, 3, false);
      }
      else
      {
        lcd.setCursor(3, 0);
        lcd.print("LCD Color");
        lcd.setCursor(5, 1);
        displaySmallInt(lcdHue);
      }
      return lcdHue;
      break;
    }
    case modeBigFont:
    {
      lcd.setCursor(4, 0);
      lcd.print("Big Font ");
      lcd.setCursor(6, 1);
      if(lcdBigFont)
      {
        lcd.print(" ON");
      }
      else
      {
        lcd.print("OFF");
      }
      return lcdBigFont;
      break;
    }
    case modeLCDBrightness: // LCD brightness
    {
      if (bigFont)
      {
        lcd.setCursor(10, 0);
        lcd.print("Bright");
        lcd.setCursor(10, 1);
        lcd.print("ness");
        bigNum.displayLargeInt(lcdBrightness, 0, 3, false);
      }
      else
      {
        lcd.setCursor(1, 0);
        lcd.print("LCD Brightness");
        lcd.setCursor(5, 1);
        displaySmallInt(lcdBrightness);
      }
      return lcdBrightness;
      break;
    }
    case modeLCDContrast: // LCD contrast
    {
      if (bigFont)
      {
        lcd.setCursor(10, 0);
        lcd.print("Con");
        lcd.setCursor(10, 1);
        lcd.print("trast");
        bigNum.displayLargeInt(lcdContrast, 0, 3, false);
      }
      else
      {
        lcd.setCursor(2, 0);
        lcd.print("LCD Contrast");
        lcd.setCursor(5, 1);
        displaySmallInt(lcdContrast);
      }
      return lcdContrast;
      break;
    }
    case modeLCDAutoDim: // LCD AutoDim ON/OFF
    {
      lcd.setCursor(2, 0);
      lcd.print("LCD Auto Dim");
      lcd.setCursor(6, 1);
      if(lcdAutoDim)
      {
        lcd.print(" ON");
      }
      else
      {
        lcd.print("OFF");
      }
      return lcdAutoDim;
      break;
    }
  }
}

// function that displays a float to the LCD. Maintains position of decimal point.
// float n: the number to display (from -999.99 to 9999.99)
// byte digits: number of digits after decimal point to display
void displaySmallFloat(float n, byte digits)
{
  n = constrain(n, -999.99, 9999.99);
  if (n < 10 && n >= 0)
  {
    lcd.print("   ");
  }
  else if ((n < 100 && n >= 10) || (n < 0 && n > -10))
  {
    lcd.print("  ");
  }
  else if ((n < 1000 && n >= 100) || (n <= -10 && n > -100))
  {
    lcd.print(" ");
  }
  lcd.print(n, digits);
}

// function that displays an integer to the LCD. Maintains position of ones place.
// int n: the number to display (from -999to 9999)
void displaySmallInt(int n)
{
  n = constrain(n, -999, 9999);
  if (n < 10 && n >= 0)
  {
    lcd.print("   ");
  }
  else if ((n < 100 && n >= 10) || (n < 0 && n > -10))
  {
    lcd.print("  ");
  }
  else if ((n < 1000 && n >= 100) || (n <= -10 && n > -100))
  {
    lcd.print(" ");
  }
  lcd.print(n);
}

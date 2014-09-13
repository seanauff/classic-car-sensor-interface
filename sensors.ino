#include <OneWire.h>
#include <DallasTemperature.h>
#include <Encoder.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <math.h>
#include <Time.h>
#include <Wire.h>
#include <DS1307RTC.h>

// EEPROM memory addresses
const int lcdContrastAddress = 1;
const int lcdBrightnessAddress = 2;
const int lcdHueAddress = 3;
const int useCelciusAddress = 4;
const int lcdAutoDimAddress = 5;
const int lcdBigFontAddress = 6;
const int engineCylindersAddress = 7;

// digital pins
const int switchPin = 0; // momentary switch on interrupt 0 (digital pin 2)
const int tachPin = 1; // tach signal on interrupt 1 (digital pin 3)
const int ignitionPin = 4; // ignition signal on interrupt 4 (digital pin 19)
const int lcdContrastPin = 4; // contrast adjust on digital pin 4 (PWM)
const int encoderPin1 = 18; // A leg of encoder on digital pin 18 (interrupt 5)
const int encoderPin2 = 19; // B leg of encoder on digital pin 19 (interrupt 4)
const int wireSDAPin = 20;
const int wireSCLPin = 21;
const int igntionPin = 4; // ignition signal on interrupt 4 (digital pin 19)
const int oneWirePin = 24; // data pin for 1-Wire devices
const int lcdD7Pin = 38; // LCD D7 pin
const int lcdD6Pin = 39; // LCD D6 pin
const int lcdD5Pin = 40; // LCD D5 pin
const int lcdD4Pin = 41; // LCD D4 pin
const int lcdEPin = 42; // LCD E Pin
const int lcdRSPin = 43; // LCD RS pin
const int lcdLEDRedPin = 44; // control for Red LED (PWM)
const int lcdLEDGreenPin = 45; // control for Green LED (PWM)
const int lcdLEDBluePin = 46; // control for Blue LED (PWM)

// analog pins
const int battVoltagePin = A0; // pin for battery voltage
const int oilPressPin = A1; // pin for oil pressure
const int fuelLevelPin = A2; // pin for fuel level
const int coolantTempPin = A3; // pin for coolant temp
const int autoDimPin = A4; // pin for external brightness control

// analog input setup
const float regVoltage = 5.9; // insturment unit voltage regulator output
const float oilGaugeOhms = 13.0; // resistance of oil pressure gauge
const float fuelGaugeOhms = 13.0; // resistance of fuel level gauge
const float coolantGaugeOhms = 13.0; // resistance of coolant temperature gauge

// Steinhart–Hart equation paramerters for coolant temp sender
const float SHparamA = 1.459339e-3;
const float SHparamB = 2.329463e-4;
const float SHparamC = 9.355121e-8;

// OneWire setup
OneWire oneWire(oneWirePin);
DallasTemperature sensors(&oneWire);
DeviceAddress insideTempDigital = {0x28, 0xFF, 0x1B, 0x36, 0x2D, 0x04, 0x00, 0xBA};
DeviceAddress outsideTempDigital = {0x28, 0xFF, 0xDF, 0x33, 0x2B, 0x04, 0x00, 0xD7};
DeviceAddress oilTempDigital = {0x28, 0xFF, 0xB5, 0x36, 0x2D, 0x04, 0x00, 0x2B};
DeviceAddress intakeTempDigital = {0x28, 0xFF, 0xAF, 0x08, 0x2E, 0x04, 0x00, 0x53};
DeviceAddress transTempDigital = {0x28, 0xFF, 0xD5, 0x33, 0x2B, 0x04, 0x00, 0x6A};

// rotary encoder setup
Encoder modeSwitch(encoderPin1,encoderPin2);
const int modeMin = 1;
const int modeMax = 13; // number of modes
// modes - this sets order
const int modeClock = 1;
const int modeBattVoltage = 2;
const int modeOilPress = 3;
const int modeCoolantTemp = 4;
const int modeOutsideTemp = 5;
const int modeInsideTemp = 6;
const int modeOilTemp = 7;
const int modeTransTemp = 8;
const int modeIntakeTemp = 9;
const int modeTach = 10;
const int modeFuelLevel = 11;
const int modeLCDSetup = 12;
const int modeSystemSetup = 13;
const int modeEngineCylinders = 94; // hidden mode (not in normal rotation)
const int modeLCDColor = 95; // hidden mode (not in normal rotation)
const int modeBigFont = 96; // hidden mode (not in normal rotation)
const int modeLCDBrightness = 97; // hidden mode (not in normal rotation)
const int modeLCDContrast = 98; // hidden mode (not in normal rotation)
const int modeLCDAutoDim = 99; // hidden mode (not in normal rotation)

int mode = modeClock; // mode to start in
int previousMode = mode;

// lcd setup
LiquidCrystal lcd(lcdRSPin, lcdEPin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);

// custom characters for large font numbers
byte leftSide[8] = 
{
  B00111,
  B01111,
  B01111,
  B01111,
  B01111,
  B01111,
  B01111,
  B00111
};
byte upperBar[8] =
{
  B11111,
  B11111,
  B11111,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};
byte rightSide[8] =
{
  B11100,
  B11110,
  B11110,
  B11110,
  B11110,
  B11110,
  B11110,
  B11100
};
byte leftEnd[8] =
{
  B01111,
  B00111,
  B00000,
  B00000,
  B00000,
  B00000,
  B00011,
  B00111
};
byte lowerBar[8] =
{
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111
};
byte rightEnd[8] =
{
  B11110,
  B11100,
  B00000,
  B00000,
  B00000,
  B00000,
  B11000,
  B11100
};
byte middleBar[8] =
{
  B11111,
  B11111,
  B11111,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111
};
byte lowerEnd[8] = 
{
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00111,
  B01111
};

int lcdContrast = 90; // scale from 0-100, map to 0-255 for analogWrite()
int lcdBrightness = 100; // scale from 0-100
int lcdHue = 45; // scale from 1-120
int lcdLEDRed; // scale from 0-255
int lcdLEDGreen; // scale from 0-255
int lcdLEDBlue; // scale from 0-255

// clock setup
// default values if RTC not set
byte currentHour = 0;
byte currentMinute = 0;
byte currentSecond = 0;
byte currentMonth = 1;
byte currentDay = 1;
int currentYear = 2014;

// personalization
byte useCelcius = 0; // unit selector (deg F/deg C, psi/bar)
byte lcdAutoDim = 0;
byte lcdBigFont = 1;
byte engineCylinders = 8; // for tach calculation
const int refreshInterval = 750; // milliseconds between sensor updates

// timing values
unsigned long previousMillis = 0; // for sensor refresh interval
unsigned long timeSwitchLastPressed = 0; // for switch debounce
int switchDebounceInterval = 200; // time in ms where switch press will do nothing

// interrupt volatile variables
volatile int RPMpulses = 0;
volatile boolean buttonPressed=false;

void setup()
{
  // load values from EEPROM
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
  if (EEPROM.read(useCelciusAddress) < 255)
  {
    useCelcius = EEPROM.read(useCelciusAddress);
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
  // get RTC time
  setSyncProvider(RTC.get);
  // set default time if RTC not set
  if (timeStatus() != timeSet)
  {
    setTime(currentHour,currentMinute,currentSecond,currentDay,currentMonth,currentYear);
  }
  pinMode(2, INPUT_PULLUP);
  pinMode(lcdContrastPin,OUTPUT);
  pinMode(lcdLEDRedPin,OUTPUT);
  pinMode(lcdLEDGreenPin,OUTPUT);
  pinMode(lcdLEDBluePin,OUTPUT);
  lcd.begin(16, 2); //setup lcd
  lcd.createChar(0,leftSide);
  lcd.createChar(1,upperBar);
  lcd.createChar(2,rightSide);
  lcd.createChar(3,leftEnd);
  lcd.createChar(4,lowerBar);
  lcd.createChar(5,rightEnd);
  lcd.createChar(6,middleBar);
  lcd.createChar(7,lowerEnd);
  setRGBFromHue();
  lcdBrightness = map((lcdLEDRed+lcdLEDGreen+lcdLEDBlue),255,510,100,50);
  writeLCDValues(); 
  sensors.begin(); //setup DS18B20 sensors to 9 bit resolution
  sensors.setResolution(insideTempDigital,9);
  sensors.setResolution(outsideTempDigital,9);
  sensors.setResolution(oilTempDigital,9);
  sensors.setResolution(intakeTempDigital,9);
  sensors.setResolution(transTempDigital,9);
  attachInterrupt(switchPin,pressButton,FALLING);
  
  modeSwitch.write((mode-1)*4);
}

void setRGBFromHue()
{
  lcdHue = constrain(lcdHue,1,120);
  int hue = lcdHue;
  if(hue<=20)
  {
    lcdLEDRed = 255;
    lcdLEDGreen = map(hue,1,20,0,255);
    lcdLEDBlue = 0;
  }
  else if (hue<=40)
  {
    lcdLEDRed = map(hue,21,40,255,0);
    lcdLEDGreen = 255;
    lcdLEDBlue = 0;
  }
  else if (hue<=60)
  {
    lcdLEDRed = 0;
    lcdLEDGreen = 255;
    lcdLEDBlue = map(hue,41,60,0,255);
  }
  else if (hue<=80)
  {
    lcdLEDRed = 0;
    lcdLEDGreen = map(hue,61,80,255,0);
    lcdLEDBlue = 255;
  }
  else if (hue<=100)
  {
    lcdLEDRed = map(hue,81,100,0,255);
    lcdLEDGreen = 0;
    lcdLEDBlue = 255;
  }
  else if (hue<=120)
  {
    lcdLEDRed = 255;
    lcdLEDGreen = 0;
    lcdLEDBlue = map(hue,101,120,255,0);
  }
}

void setAutoBrightness()
{
  if(lcdAutoDim==1)
  {
    lcdBrightness=map(analogRead(autoDimPin),0,1023,0,100);
  }
}

void writeLCDValues()
{
  // contrast
  analogWrite(lcdContrastPin,map(lcdContrast,0,100,255,0));
  
  // brightness and color
  int r = map(lcdLEDRed,0,255,0,lcdBrightness);
  int g = map(lcdLEDGreen,0,255,0,lcdBrightness);
  int b = map(lcdLEDBlue,0,255,0,lcdBrightness);
  analogWrite(lcdLEDRedPin,map(r,0,100,0,255));
  analogWrite(lcdLEDGreenPin,map(g,0,100,0,200));  
  analogWrite(lcdLEDBluePin,map(b,0,100,0,200));
}

float getBattVoltage() // returns battery voltage in volts
{
  float R1=99400.0;
  float R2=9900.0;
  int val = analogRead(battVoltagePin);
  float Vout = (val/1023.0)*5.0;
  float Vin = Vout*(R1+R2)/R2;
  return Vin;
}

float getOilPress() // returns Oil pressure in psi
{
  float R1=47000.0;
  float R2=100000.0;
  int val = analogRead(oilPressPin);
  float Vout = val/1023.0*5.0;
  float VI = Vout*(R1+R2)/R2;
  float Rsender = VI*oilGaugeOhms/(regVoltage-VI);
  float pressure = 108.4-0.56*Rsender; // sensor calibration curve
  return pressure;
}

float getFuelLevel() // returns Fuel Level in % 
{
  float R1=47000.0;
  float R2=100000.0;
  int val = analogRead(fuelLevelPin);
  float Vout = val/1023.0*5.0;
  float VI = Vout*(R1+R2)/R2;
  float Rsender = VI*fuelGaugeOhms/(regVoltage-VI);
  float level = 108.4-0.56*Rsender; // sensor calibration curve
  return level;
}

float getCoolantTemp() // returns coolant temp in deg C
{
  float R1=47000.0;
  float R2=100000.0;
  int val = analogRead(coolantTempPin);
  float Vout = val/1023.0*5.0;
  float VI = Vout*(R1+R2)/R2;
  float Rsender = VI*coolantGaugeOhms/(regVoltage-VI);
  float temp = pow(SHparamA+SHparamB*log(Rsender)+SHparamC*pow(log(Rsender),3),-1)-273.15; // sensor calibration curve based on Steinhart–Hart equation
  return temp;
}

void countRPM()
{
  RPMpulses++;
}

void pressButton()
{
  if(millis()-timeSwitchLastPressed>switchDebounceInterval)
  {
    buttonPressed = true;
    timeSwitchLastPressed=millis();
  }
}

void loop()
{
  if(mode==modeLCDSetup && buttonPressed==true) // change lcd parameters
  {
    buttonPressed=false;
    lcd.clear();
    modeSwitch.write((lcdHue-1)*4); // color
    while(buttonPressed==false)
    { 
      lcdHue=modeSwitch.read()/4+1;
      if(lcdHue>120)
      {
        lcdHue=1;
        modeSwitch.write((lcdHue-1)*4);
      }
      else if (lcdHue<1)
      {
        lcdHue=120;
        modeSwitch.write((lcdHue-1)*4);
      }
      setRGBFromHue();
      lcdBrightness = map((lcdLEDRed+lcdLEDGreen+lcdLEDBlue),255,510,100,50);
      writeLCDValues();
      displayInfo(modeLCDColor);
    }
    EEPROM.write(lcdHueAddress,lcdHue);
    buttonPressed=false;
    modeSwitch.write((lcdAutoDim-1)*4); // auto dim on/off
    lcd.clear();
    while(buttonPressed==false)
    {
      lcdAutoDim=modeSwitch.read()/4+1;
      if(lcdAutoDim>1)
      {
        lcdAutoDim=1;
        modeSwitch.write((lcdAutoDim-1)*4);
      }
      else if (lcdAutoDim<0)
      {
        lcdAutoDim=0;
        modeSwitch.write((lcdAutoDim-1)*4);
      }
      setAutoBrightness();
      writeLCDValues();
      displayInfo(modeLCDAutoDim); 
    }
    EEPROM.write(lcdAutoDimAddress,lcdAutoDim);
    buttonPressed=false;
    lcd.clear();
    modeSwitch.write((lcdBrightness-1)*4); // manual brightness
    while(buttonPressed==false && lcdAutoDim==0)
    { 
      lcdBrightness=modeSwitch.read()/4+1;
      if(lcdBrightness>100)
      {
        lcdBrightness=100;
        modeSwitch.write((lcdBrightness-1)*4);
      }
      else if (lcdBrightness<0)
      {
        lcdBrightness=0;
        modeSwitch.write((lcdBrightness-1)*4);
      }
      writeLCDValues();
      displayInfo(modeLCDBrightness);
    }
    EEPROM.write(lcdBrightnessAddress,lcdBrightness);
    lcd.clear();
    buttonPressed=false;
    modeSwitch.write((lcdContrast-1)*4); // contrast
    while(buttonPressed==false)
    { 
      lcdContrast=modeSwitch.read()/4+1;
      if(lcdContrast>100)
      {
        lcdContrast=100;
        modeSwitch.write((lcdContrast-1)*4);
      }
      else if (lcdContrast<0)
      {
        lcdContrast=0;
        modeSwitch.write((lcdContrast-1)*4);
      }
      writeLCDValues();
      displayInfo(modeLCDContrast);
    }
    EEPROM.write(lcdContrastAddress,lcdContrast);
    buttonPressed=false;
    modeSwitch.write((lcdBigFont-1)*4); // big font on/off
    lcd.clear();
    while(buttonPressed==false)
    {
      lcdBigFont=modeSwitch.read()/4+1;
      if(lcdBigFont>1)
      {
        lcdBigFont=1;
        modeSwitch.write((lcdBigFont-1)*4);
      }
      else if (lcdBigFont<0)
      {
        lcdBigFont=1;
        modeSwitch.write((lcdBigFont-1)*4);
      }
      displayInfo(modeBigFont);
    }
    EEPROM.write(lcdBigFontAddress,lcdBigFont);
    buttonPressed = false;
    modeSwitch.write((mode-1)*4);
    lcd.clear();
  }
  else if (mode == modeSystemSetup && buttonPressed == true) // change system parameters
  {
    lcd.clear();
    buttonPressed = false;
    modeSwitch.write(((engineCylinders/2)-1)*4); // set number of cylinders
    while(buttonPressed == false)
    {
      engineCylinders = 2*(modeSwitch.read()/4+1);
      if(engineCylinders < 2)
      {
        engineCylinders = 2;
        modeSwitch.write(((engineCylinders/2)-1)*4);
      }
      else if (engineCylinders > 16)
      {
        engineCylinders = 16;
        modeSwitch.write(((engineCylinders/2)-1)*4);
      }
      displayInfo(modeEngineCylinders);
    }
    EEPROM.write(engineCylindersAddress,engineCylinders);
    buttonPressed = false;
    modeSwitch.write((mode-1)*4);
    lcd.clear();
  }
  else if (mode==modeClock && buttonPressed==true) //  set clock
  {
    lcd.clear();
    buttonPressed = false;
    byte instantHour = hour();
    byte instantMinute = minute();
    byte instantSecond = 0;
    byte instantMonth = month();
    byte instantDay = day();
    int instantYear = year();
    modeSwitch.write((instantHour-1)*4); // set Hour
    lcd.setCursor(0,0);
    lcd.print("     ");
    lcd.setCursor(0,0);
    lcd.print("Hour");
    while(buttonPressed==false)
    {
      instantHour = modeSwitch.read()/4+1;
      if (instantHour<0)
      {
        instantHour=23;
        modeSwitch.write((instantHour-1)*4);
      }
      else if (instantHour>23)
      {
        instantHour=0;
        modeSwitch.write((instantHour-1)*4);
      }
      setTime(instantHour,instantMinute,instantSecond,instantDay,instantMonth,instantYear);
      displayInfo(modeClock);
    }
    buttonPressed=false;
    modeSwitch.write((instantMinute-1)*4); // set Minute
    lcd.setCursor(0,0);
    lcd.print("     ");
    lcd.setCursor(0,0);
    lcd.print("Min");
    while(buttonPressed==false)
    {
      instantMinute = modeSwitch.read()/4+1;
      if (instantMinute<0)
      {
        instantMinute=59;
        modeSwitch.write((instantMinute-1)*4);
      }
      else if (instantMinute>59)
      {
        instantMinute=0;
        modeSwitch.write((instantMinute-1)*4);
      }
      setTime(instantHour,instantMinute,instantSecond,instantDay,instantMonth,instantYear);
      displayInfo(modeClock);
    }
    
    buttonPressed=false;
    modeSwitch.write((instantMonth-1)*4); // set Month
    lcd.setCursor(0,0);
    lcd.print("     ");
    lcd.setCursor(0,0);
    lcd.print("Month");
    while(buttonPressed==false)
    {
      instantMonth = modeSwitch.read()/4+1;
      if (instantMonth<1)
      {
        instantMonth=12;
        modeSwitch.write((instantMonth-1)*4);
      }
      else if (instantMonth>12)
      {
        instantMonth=1;
        modeSwitch.write((instantMonth-1)*4);
      }
      setTime(instantHour,instantMinute,instantSecond,instantDay,instantMonth,instantYear);
      displayInfo(modeClock);
    }
    buttonPressed=false;
    modeSwitch.write((instantDay-1)*4); // set Day
    lcd.setCursor(0,0);
    lcd.print("     ");
    lcd.setCursor(0,0);
    lcd.print("Day");
    while(buttonPressed==false)
    {
      instantDay = modeSwitch.read()/4+1;
      if (instantDay<1)
      {
        instantDay=31;
        modeSwitch.write((instantDay-1)*4);
      }
      else if (instantDay>31)
      {
        instantDay=1;
        modeSwitch.write((instantDay-1)*4);
      }
      setTime(instantHour,instantMinute,instantSecond,instantDay,instantMonth,instantYear);
      displayInfo(modeClock);
    }
    buttonPressed=false;
    modeSwitch.write((instantYear-1)*4); // set Year
    lcd.setCursor(0,0);
    lcd.print("     ");
    lcd.setCursor(0,0);
    lcd.print("Year");
    while(buttonPressed==false)
    {
      instantYear = modeSwitch.read()/4+1;
      if (instantYear<2000)
      {
        instantYear=2100;
        modeSwitch.write((instantYear-1)*4);
      }
      else if (instantYear>2100)
      {
        instantYear=2000;
        modeSwitch.write((instantYear-1)*4);
      }
      setTime(instantHour,instantMinute,instantSecond,instantDay,instantMonth,instantYear);
      displayInfo(modeClock);
    }
    RTC.set(now());
    buttonPressed=false;
    lcd.clear();
    modeSwitch.write((mode-1)*4);
  }
  else
  {
    mode = modeSwitch.read()/4+1;
    
    if (mode>modeMax)
    {
      mode=modeMin;
      modeSwitch.write((mode-1)*4);
    }
    if (mode<modeMin)
    {
      mode=modeMax;
      modeSwitch.write((mode-1)*4);
    }
    
    if(mode!=previousMode)
    {
      previousMillis=0;
      previousMode=mode;
      lcd.clear();
      
      if(mode==modeTach) // start or stop counting interrupts for engine speed
      {
        attachInterrupt(tachPin,countRPM,RISING);
      }
      else
      {
        detachInterrupt(tachPin);
      }     
      buttonPressed=false; // reset momentary button
    }
  }
  
  // switch temp and pressure units
  if ((mode==modeCoolantTemp || mode==modeInsideTemp || mode==modeOutsideTemp || mode==modeTransTemp || mode==modeOilTemp || mode==modeIntakeTemp || mode==modeOilPress) && buttonPressed==true)
  {
    previousMillis=0;
    buttonPressed=false; 
    if(useCelcius==0)
    {
      useCelcius=1;
    }
    else if (useCelcius=1)
    {
      useCelcius=0;
    }
    EEPROM.write(useCelciusAddress,useCelcius);
  }
  
  // sensor readings for display
  // modify screen brightness
  if(millis()-previousMillis>refreshInterval)
  {
    setAutoBrightness();
    writeLCDValues();
    previousMillis=millis();
    if(lcdBigFont == false)
    {
      displayInfo(mode);
    }
    else
    {
      displayInfoLarge(mode);
    }
  }
}

void displayInfo(int displayMode)
{
  switch (displayMode)
  {
    case modeBattVoltage: // battery voltage
    {
      lcd.setCursor(0,0);
      lcd.print("Battery Voltage:");
      float battVoltage = getBattVoltage();
      lcd.setCursor(5,1);
      if (battVoltage<10)
      {
        lcd.print(" ");
      }
      lcd.print(battVoltage,1);
      lcd.print(" V");
      break;
    }
    case modeOilPress: // oil pressure
    {
      lcd.setCursor(1,0);
      lcd.print("Oil Pressure:");
      float oilPressure = getOilPress();
      if (useCelcius == 1)
      {
        oilPressure = oilPressure*0.06895;
      }
      lcd.setCursor(3,1);
      if(oilPressure<10)
      {
        lcd.print("  ");
      }
      else if(oilPressure<100)
      {
        lcd.print(" ");
      }
      lcd.print(oilPressure,1);
      if (useCelcius == 1)
      {
        lcd.print(" bar");
      }
      else
      {
        lcd.print(" psi");
      }
      break;
    }
    case modeCoolantTemp: // coolant temp
    {
      lcd.setCursor(1,0);
      lcd.print("Coolant Temp:");
      float currentTemp = getCoolantTemp();
      if (useCelcius==0)
      {
        currentTemp=currentTemp*9.0/5.0+32.0;
      }
      lcd.setCursor(5,1);
      if ((currentTemp<=-10 && currentTemp>-100) || (currentTemp>=10 && currentTemp<100))
      {
        lcd.print(" ");
      }
      else if (currentTemp<10 && currentTemp>-10)
      {
        lcd.print("  ");
      }
      lcd.print(currentTemp,0);
      lcd.print(" ");
      lcd.write(char(223));
      if (useCelcius==0)
      {
        lcd.print("F");
      }
      else
      {
        lcd.print("C");
      }
      break;
    }
    case modeOutsideTemp: // outside temp
    {
      lcd.setCursor(1,0);
      lcd.print("Outside Temp:");
      float currentTemp = sensors.getTempC(outsideTempDigital);
      if (useCelcius==0)
      {
        currentTemp=currentTemp*9.0/5.0+32.0;
      }
      lcd.setCursor(5,1);
      if ((currentTemp<=-10 && currentTemp>-100) || (currentTemp>=10 && currentTemp<100))
      {
        lcd.print(" ");
      }
      else if (currentTemp<10 && currentTemp>-10)
      {
        lcd.print("  ");
      }
      lcd.print(currentTemp,0);
      lcd.print(" ");
      lcd.write(char(223));
      if (useCelcius==0)
      {
        lcd.print("F");
      }
      else
      {
        lcd.print("C");
      }
      break;
    }
    case modeInsideTemp: // inside temp
    {
      lcd.setCursor(2,0);
      lcd.print("Inside Temp:");
      float currentTemp = sensors.getTempC(insideTempDigital);
      if (useCelcius==0)
      {
        currentTemp=currentTemp*9.0/5.0+32.0;
      }
      lcd.setCursor(5,1);
      if ((currentTemp<=-10 && currentTemp>-100) || (currentTemp>=10 && currentTemp<100))
      {
        lcd.print(" ");
      }
      else if (currentTemp<10 && currentTemp>-10)
      {
        lcd.print("  ");
      }
      lcd.print(currentTemp,0);
      lcd.print(" ");
      lcd.write(char(223));
      if (useCelcius==0)
      {
        lcd.print("F");
      }
      else
      {
        lcd.print("C");
      }
      break;
    }
    case modeOilTemp: // oil temp
    {
      lcd.setCursor(3,0);
      lcd.print("Oil Temp:");
      float currentTemp = sensors.getTempC(oilTempDigital);
      if (useCelcius==0)
      {
        currentTemp=currentTemp*9.0/5.0+32.0;
      }
      lcd.setCursor(5,1);
      if ((currentTemp<=-10 && currentTemp>-100) || (currentTemp>=10 && currentTemp<100))
      {
        lcd.print(" ");
      }
      else if (currentTemp<10 && currentTemp>-10)
      {
        lcd.print("  ");
      }
      lcd.print(currentTemp,0);
      lcd.print(" ");
      lcd.write(char(223));
      if (useCelcius==0)
      {
        lcd.print("F");
      }
      else
      {
        lcd.print("C");
      }
      break;
    }
    case modeTransTemp: // transmission temp
    {
      lcd.setCursor(2,0);
      lcd.print("Trans. Temp:");
      float currentTemp = sensors.getTempC(transTempDigital);
      if (useCelcius==0)
      {
        currentTemp=currentTemp*9.0/5.0+32.0;
      }
      lcd.setCursor(5,1);
      if ((currentTemp<=-10 && currentTemp>-100) || (currentTemp>=10 && currentTemp<100))
      {
        lcd.print(" ");
      }
      else if (currentTemp<10 && currentTemp>-10)
      {
        lcd.print("  ");
      }
      lcd.print(currentTemp,0);
      lcd.print(" ");
      lcd.write(char(223));
      if (useCelcius==0)
      {
        lcd.print("F");
      }
      else
      {
        lcd.print("C");
      }
      break;
    }
    case modeTach: // engine speed
    {
      lcd.setCursor(1,0);
      lcd.print("Engine Speed:");
      float currentRPM = RPMpulses/(engineCylinders/2)*(60000.0/refreshInterval);
      RPMpulses=0;
      lcd.setCursor(4,1);
      if(currentRPM<10)
      {
        lcd.print("   ");
      }
      else if(currentRPM<100)
      {
        lcd.print("  ");
      }
      else if(currentRPM<1000)
      {
        lcd.print(" ");
      }
      lcd.print(currentRPM,0);
      lcd.print(" RPM");
      break;
    }
    case modeIntakeTemp: // intake temp
    {
      lcd.setCursor(2,0);
      lcd.print("Intake Temp:");
      float currentTemp = sensors.getTempC(intakeTempDigital);
      if (useCelcius==0)
      {
        currentTemp=currentTemp*9.0/5.0+32.0;
      }
      lcd.setCursor(5,1);
      if ((currentTemp<=-10 && currentTemp>-100) || (currentTemp>=10 && currentTemp<100))
      {
        lcd.print(" ");
      }
      else if (currentTemp<10 && currentTemp>-10)
      {
        lcd.print("  ");
      }
      lcd.print(currentTemp,0);
      lcd.print(" ");
      lcd.write(char(223));
      if (useCelcius==0)
      {
        lcd.print("F");
      }
      else
      {
        lcd.print("C");
      }
      break;
    }
    case modeLCDSetup: // Display Settings
    {
      lcd.setCursor(4,0);
      lcd.print("Display");
      lcd.setCursor(4,1);
      lcd.print("Settings");
      break;
    }
    case modeSystemSetup: // System Settings
    {
      lcd.setCursor(5,0);
      lcd.print("System");
      lcd.setCursor(4,1);
      lcd.print("Settings");
      break;
    }
    case modeClock: // Clock
    {
      time_t currentTime = now();
      currentHour = hourFormat12(currentTime);
      currentMinute = minute(currentTime);
      // currentSecond = second(currentTime);
      currentMonth = month(currentTime);
      currentDay = day(currentTime);
      currentYear = year(currentTime);
      byte currentDayOfWeek = weekday(currentTime);
      lcd.setCursor(4,0);
      if(currentHour<10)
      {
        lcd.print(" ");
      }
      lcd.print(currentHour);
      lcd.print(":");
      if(currentMinute<10)
      {
        lcd.print("0");
      }
      lcd.print(currentMinute);
      lcd.print(" ");
      if(isAM())
      {
        lcd.print("AM");
      }
      else
      {
        lcd.print("PM");
      }
      lcd.setCursor(0,1);
      switch (currentDayOfWeek)
      {
        case 1:
        {
          lcd.print("Sun");
          break;
        }
        case 2:
        {
          lcd.print("Mon");
          break;
        }
        case 3:
        {
          lcd.print("Tue");
          break;
        }
        case 4:
        {
          lcd.print("Wed");
          break;
        }
        case 5:
        {
          lcd.print("Thu");
          break;
        }
        case 6:
        {
          lcd.print("Fri");
          break;
        }
        case 7:
        {
          lcd.print("Sat");
          break;
        }
      }
      lcd.print(" ");
      switch (currentMonth)
      {
        case 1:
        {
          lcd.print("Jan");
          break;
        }
        case 2:
        {
          lcd.print("Feb");
          break;
        }
        case 3:
        {
          lcd.print("Mar");
          break;
        }
        case 4:
        {
          lcd.print("Apr");
          break;
        }
        case 5:
        {
          lcd.print("May");
          break;
        }
        case 6:
        {
          lcd.print("Jun");
          break;
        }
        case 7:
        {
          lcd.print("Jul");
          break;
        }
        case 8:
        {
          lcd.print("Aug");
          break;
        }
        case 9:
        {
          lcd.print("Sep");
          break;
        }
        case 10:
        {
          lcd.print("Oct");
          break;
        }
        case 11:
        {
          lcd.print("Nov");
          break;
        }
        case 12:
        {
          lcd.print("Dec");
          break;
        }
      }
      lcd.print(" ");
      if(currentDay<10)
      {
        lcd.print(" ");
      }
      lcd.print(currentDay);
      lcd.print(" ");
      lcd.print(currentYear);
      break;
    }
    case modeFuelLevel: //  Fuel Level
    {
      lcd.setCursor(2,0);
      lcd.print("Fuel Level:");
      float fuelLevel = getFuelLevel();
      fuelLevel = constrain(fuelLevel,0,100);
      lcd.setCursor(5,1);
      if (fuelLevel<10)
      {
        lcd.print("  ");
      }
      else if (fuelLevel<100)
      {
        lcd.print(" ");
      }
      lcd.print(fuelLevel,0);
      lcd.print(" %");
      break;
    }
    case modeLCDColor:
    {
      lcd.setCursor(3,0);
      lcd.print("LCD Color:");
      lcd.setCursor(6,2);
      if(lcdHue<10)
      {
        lcd.print("  ");
      }
      else if (lcdHue<100)
      {
        lcd.print(" ");
      }
      lcd.print(lcdHue);
      break;
    }
    case modeBigFont:
    {
      lcd.setCursor(4,0);
      lcd.print("Big Font");
      lcd.setCursor(6,1);
      if(lcdBigFont==0)
      {
        lcd.print("OFF");
      }
      else
      {
        lcd.print(" ON");
      }
      break;
    }
    case modeLCDBrightness: // LCD brightness
    {
      lcd.setCursor(0,0);
      lcd.print("LCD Brightness:");
      lcd.setCursor(6,1);
      if (lcdBrightness<10)
      {
        lcd.print("  ");
      }
      else if (lcdBrightness<100)
      {  
        lcd.print(" ");
      }
      lcd.print(lcdBrightness);
      break;
    }
    case modeLCDContrast: // LCD contrast
    {
      lcd.setCursor(1,0);
      lcd.print("LCD Contrast:");
      lcd.setCursor(6,1);
      if (lcdContrast<10)
      {
        lcd.print("  ");
      }
      else if (lcdContrast<100)
      {  
        lcd.print(" ");
      }
      lcd.print(lcdContrast);
      break;
    }
    case modeLCDAutoDim: // LCD AutoDim ON/OFF
    {
      lcd.setCursor(2,0);
      lcd.print("LCD Auto Dim");
      lcd.setCursor(6,1);
      if(lcdAutoDim==0)
      {
        lcd.print("OFF");
      }
      else
      {
        lcd.print(" ON");
      }
      break;
    }
    case modeEngineCylinders:
    {
      lcd.setCursor(0,0);
      lcd.print("Engine Cylinders");
      lcd.setCursor(7,1);
      if (engineCylinders < 10)
      {
        lcd.print(" ");
      }
      lcd.print(engineCylinders);
      break;
    }
  }
}

void displayInfoLarge(int displayMode)
{
  switch (displayMode)
  {
    case modeTach:
    {
      int currentRPM = RPMpulses/(engineCylinders/2)*(60000.0/refreshInterval);
      RPMpulses=0;
      
      byte RPMString[4];
      RPMString[3] = currentRPM%10;
      currentRPM /= 10;
      RPMString[2] = currentRPM%10;
      currentRPM /= 10;
      RPMString[1] = currentRPM%10;
      currentRPM /= 10;
      RPMString[0] = currentRPM%10;
      boolean significantZero = false;
      for (int i = 0; i < 4; i++)
      {
        if( RPMString[i]==0 && significantZero==false && i<3)
        {
          clearLargeNumber(i*3);
        }
        else
        {
          displayLargeNumber(RPMString[i],i*3);
          significantZero = true;
        }
      }
      lcd.setCursor(13,1);
      lcd.print("RPM");
      break;
    }
    case modeFuelLevel:
    {
      float fuelLevel = getFuelLevel();
      fuelLevel = constrain(fuelLevel,0,100);
      int fuelLevelInt = int(fuelLevel);
      byte fuelString[3];
      fuelString[2] = fuelLevelInt%10;
      fuelLevelInt /= 10;
      fuelString[1] = fuelLevelInt%10;
      fuelLevelInt /= 10;
      fuelString[0] = fuelLevelInt%10;
      boolean significantZero = false;
      for (int i = 0; i < 3; i++)
      {
        if( fuelString[i]==0 && significantZero==false && i<2)
        {
          clearLargeNumber(i*3);
        }
        else
        {
          displayLargeNumber(fuelString[i],i*3);
          significantZero = true;
        }
      }
      lcd.setCursor(10,0);
      lcd.print("Fuel");
      lcd.setCursor(10,1);
      lcd.print("%");
      break;
    }
    case modeBattVoltage: // battery voltage
    {
      float battVoltage = getBattVoltage();
      battVoltage *=10;
      int battVoltageInt = int(battVoltage);
      byte battString[3];
      battString[2] = battVoltageInt%10;
      battVoltageInt /= 10;
      battString[1] = battVoltageInt%10;
      battVoltageInt /= 10;
      battString[0] = battVoltageInt%10;
      boolean significantZero = false;
      for (int i = 0; i < 2; i++)
      {
        if( battString[i]==0 && significantZero==false && i<1)
        {
          clearLargeNumber(i*3);
        }
        else
        {
          displayLargeNumber(battString[i],i*3);
          significantZero = true;
        }
      }
      lcd.setCursor(9,0);
      lcd.print("Battery");
      lcd.setCursor(6,1);
      lcd.print(".");
      lcd.print(battString[2]);
      lcd.print(" Volts");
      break;
    }
    case modeOilPress: // oil pressure
    {
      float oilPressure = getOilPress();
      if (useCelcius == 1)
      {
        oilPressure = oilPressure*0.06895;
      }
      oilPressure *= 10;
      int oilPressureInt = int(oilPressure);
      byte oilString[4];
      oilString[3] = oilPressureInt%10;
      oilPressureInt /= 10;
      oilString[2] = oilPressureInt%10;
      oilPressureInt /= 10;
      oilString[1] = oilPressureInt%10;
      oilPressureInt /= 10;
      oilString[0] = oilPressureInt%10;
      boolean significantZero = false;
      for (int i = 0; i < 3; i++)
      {
        if( oilString[i]==0 && significantZero==false && i<2)
        {
          clearLargeNumber(i*3);
        }
        else
        {
          displayLargeNumber(oilString[i],i*3);
          significantZero = true;
        }
      }
      lcd.setCursor(12,0);
      lcd.print("Oil");
      lcd.setCursor(9,1);
      lcd.print(".");
      lcd.print(oilString[3]);
      if (useCelcius == 1)
      {
        lcd.print(" bar");
      }
      else
      {
        lcd.print(" psi");
      }
      break;
    }
    case modeClock:
    {
      time_t currentTime = now();
      currentHour = hourFormat12(currentTime);
      byte currentHourTemp = currentHour;
      byte hourString[2];
      hourString[1] = currentHourTemp%10;
      currentHourTemp /= 10;
      hourString[0] = currentHourTemp%10;
      currentMinute = minute(currentTime);
      byte currentMinuteTemp = currentMinute;
      byte minuteString[2];
      minuteString[1] = currentMinuteTemp%10;
      currentMinuteTemp /= 10;
      minuteString[0] = currentMinuteTemp%10;
      boolean significantZero = false;
      for (int i = 0; i < 2; i++)
      {
        if( hourString[i]==0 && significantZero==false && i<1)
        {
          clearLargeNumber(i*3);
        }
        else
        {
          displayLargeNumber(hourString[i],i*3);
          significantZero = true;
        }
      }
      lcd.setCursor(6,0);
      lcd.write(char(165));
      lcd.setCursor(6,1);
      lcd.write(char(165));
      for (int i = 0; i < 2; i++)
      {
        displayLargeNumber(minuteString[i],(i*3)+7);
        significantZero = true;
      }
      lcd.setCursor(14,1);
      if(isAM())
      {
        lcd.print("AM");
      }
      else
      {
        lcd.print("PM");
      }
      break;
    }
    case modeIntakeTemp:
    {
      float currentTemp = sensors.getTempC(intakeTempDigital);
      if (useCelcius==0)
      {
        currentTemp=currentTemp*9.0/5.0+32.0;
      }
      boolean isNegative = false;
      if(currentTemp<0)
      {
        isNegative = true;
        currentTemp = abs(currentTemp);
      }
      int currentTempInt = int(currentTemp);
      byte tempString[3];
      tempString[2] = currentTempInt%10;
      currentTempInt /= 10;
      tempString[1] = currentTempInt%10;
      currentTempInt /= 10;
      tempString[0] = currentTempInt%10;
      boolean significantZero = false;
      for (int i = 0; i < 3; i++)
      {
        if( tempString[i]==0 && significantZero==false && i<2)
        {
          clearLargeNumber(i*3);
        }
        else
        {
          displayLargeNumber(tempString[i],i*3);
          significantZero = true;
        }
      }
      lcd.setCursor(10,0);
      lcd.print("Intake");
      lcd.setCursor(10,1);
      lcd.write(char(223));
      if (useCelcius == 0)
      {
        lcd.print("F");
      }
      else
      {
        lcd.print("C");
      }
      if (isNegative)
      {
        lcd.setCursor(0,0);
        lcd.print("-");
      }
      break;
    }
    case modeOilTemp:
    {
      float currentTemp = sensors.getTempC(oilTempDigital);
      if (useCelcius==0)
      {
        currentTemp=currentTemp*9.0/5.0+32.0;
      }
      boolean isNegative = false;
      if(currentTemp<0)
      {
        isNegative = true;
        currentTemp = abs(currentTemp);
      }
      int currentTempInt = int(currentTemp);
      byte tempString[3];
      tempString[2] = currentTempInt%10;
      currentTempInt /= 10;
      tempString[1] = currentTempInt%10;
      currentTempInt /= 10;
      tempString[0] = currentTempInt%10;
      boolean significantZero = false;
      for (int i = 0; i < 3; i++)
      {
        if( tempString[i]==0 && significantZero==false && i<2)
        {
          clearLargeNumber(i*3);
        }
        else
        {
          displayLargeNumber(tempString[i],i*3);
          significantZero = true;
        }
      }
      lcd.setCursor(10,0);
      lcd.print("Oil");
      lcd.setCursor(10,1);
      lcd.write(char(223));
      if (useCelcius == 0)
      {
        lcd.print("F");
      }
      else
      {
        lcd.print("C");
      }
      if (isNegative)
      {
        lcd.setCursor(0,0);
        lcd.print("-");
      }
      break;
    }
    case modeCoolantTemp:
    {
      float currentTemp = getCoolantTemp();
      if (useCelcius==0)
      {
        currentTemp=currentTemp*9.0/5.0+32.0;
      }
      boolean isNegative = false;
      if(currentTemp<0)
      {
        isNegative = true;
        currentTemp = abs(currentTemp);
      }
      int currentTempInt = int(currentTemp);
      byte tempString[3];
      tempString[2] = currentTempInt%10;
      currentTempInt /= 10;
      tempString[1] = currentTempInt%10;
      currentTempInt /= 10;
      tempString[0] = currentTempInt%10;
      boolean significantZero = false;
      for (int i = 0; i < 3; i++)
      {
        if( tempString[i]==0 && significantZero==false && i<2)
        {
          clearLargeNumber(i*3);
        }
        else
        {
          displayLargeNumber(tempString[i],i*3);
          significantZero = true;
        }
      }
      lcd.setCursor(10,0);
      lcd.print("Engine");
      lcd.setCursor(10,1);
      lcd.write(char(223));
      if (useCelcius == 0)
      {
        lcd.print("F");
      }
      else
      {
        lcd.print("C");
      }
      if (isNegative)
      {
        lcd.setCursor(0,0);
        lcd.print("-");
      }
      break;
    }
    case modeInsideTemp:
    {
      float currentTemp = sensors.getTempC(insideTempDigital);
      if (useCelcius==0)
      {
        currentTemp=currentTemp*9.0/5.0+32.0;
      }
      boolean isNegative = false;
      if(currentTemp<0)
      {
        isNegative = true;
        currentTemp = abs(currentTemp);
      }
      int currentTempInt = int(currentTemp);
      byte tempString[3];
      tempString[2] = currentTempInt%10;
      currentTempInt /= 10;
      tempString[1] = currentTempInt%10;
      currentTempInt /= 10;
      tempString[0] = currentTempInt%10;
      boolean significantZero = false;
      for (int i = 0; i < 3; i++)
      {
        if( tempString[i]==0 && significantZero==false && i<2)
        {
          clearLargeNumber(i*3);
        }
        else
        {
          displayLargeNumber(tempString[i],i*3);
          significantZero = true;
        }
      }
      lcd.setCursor(10,0);
      lcd.print("Cabin");
      lcd.setCursor(10,1);
      lcd.write(char(223));
      if (useCelcius == 0)
      {
        lcd.print("F");
      }
      else
      {
        lcd.print("C");
      }
      if (isNegative)
      {
        lcd.setCursor(0,0);
        lcd.print("-");
      }
      break;
    }
    case modeOutsideTemp:
    {
      float currentTemp = sensors.getTempC(outsideTempDigital);
      if (useCelcius==0)
      {
        currentTemp=currentTemp*9.0/5.0+32.0;
      }
      boolean isNegative = false;
      if(currentTemp<0)
      {
        isNegative = true;
        currentTemp = abs(currentTemp);
      }
      int currentTempInt = int(currentTemp);
      byte tempString[3];
      tempString[2] = currentTempInt%10;
      currentTempInt /= 10;
      tempString[1] = currentTempInt%10;
      currentTempInt /= 10;
      tempString[0] = currentTempInt%10;
      boolean significantZero = false;
      for (int i = 0; i < 3; i++)
      {
        if( tempString[i]==0 && significantZero==false && i<2)
        {
          clearLargeNumber(i*3);
        }
        else
        {
          displayLargeNumber(tempString[i],i*3);
          significantZero = true;
        }
      }
      lcd.setCursor(10,0);
      lcd.print("Out");
      lcd.setCursor(10,1);
      lcd.write(char(223));
      if (useCelcius == 0)
      {
        lcd.print("F");
      }
      else
      {
        lcd.print("C");
      }
      if (isNegative)
      {
        lcd.setCursor(0,0);
        lcd.print("-");
      }
      break;
    }
    case modeTransTemp:
    {
      float currentTemp = sensors.getTempC(transTempDigital);
      if (useCelcius==0)
      {
        currentTemp=currentTemp*9.0/5.0+32.0;
      }
      boolean isNegative = false;
      if(currentTemp<0)
      {
        isNegative = true;
        currentTemp = abs(currentTemp);
      }
      int currentTempInt = int(currentTemp);
      byte tempString[3];
      tempString[2] = currentTempInt%10;
      currentTempInt /= 10;
      tempString[1] = currentTempInt%10;
      currentTempInt /= 10;
      tempString[0] = currentTempInt%10;
      boolean significantZero = false;
      for (int i = 0; i < 3; i++)
      {
        if( tempString[i]==0 && significantZero==false && i<2)
        {
          clearLargeNumber(i*3);
        }
        else
        {
          displayLargeNumber(tempString[i],i*3);
          significantZero = true;
        }
      }
      lcd.setCursor(10,0);
      lcd.print("Trans.");
      lcd.setCursor(10,1);
      lcd.write(char(223));
      if (useCelcius == 0)
      {
        lcd.print("F");
      }
      else
      {
        lcd.print("C");
      }
      if (isNegative)
      {
        lcd.setCursor(0,0);
        lcd.print("-");
      }
      break;
    }
    case modeLCDSetup:
    {
      lcd.setCursor(4,0);
      lcd.print("Display");
      lcd.setCursor(4,1);
      lcd.print("Settings");
      break;
    }
    case modeSystemSetup: // System Settings
    {
      lcd.setCursor(5,0);
      lcd.print("System");
      lcd.setCursor(4,1);
      lcd.print("Settings");
      break;
    }
  }
}



void displayLargeNumber(byte n, byte x) // n is number to display, x is column of upper left corner for large character
{
  switch (n)
  {
    case 0:
    {
      lcd.setCursor(x,0);
      lcd.write(byte(0));
      lcd.write(1);
      lcd.write(2);
      lcd.setCursor(x, 1);
      lcd.write(byte(0));
      lcd.write(4);
      lcd.write(2);
      break;
    }
    case 1:
    {
      lcd.setCursor(x,0);
      lcd.write(char(254));
      lcd.write(char(254));
      lcd.write(2);
      lcd.setCursor(x,1);
      lcd.write(char(254));
      lcd.write(char(254));
      lcd.write(2);
      break;
    }
    case 2:
    {
      lcd.setCursor(x,0);
      lcd.write(3);
      lcd.write(6);
      lcd.write(2);
      lcd.setCursor(x, 1);
      lcd.write(byte(0));
      lcd.write(4);
      lcd.write(4);
      break;
    }
    case 3:
    {
      lcd.setCursor(x,0);
      lcd.write(3);
      lcd.write(6);
      lcd.write(2);
      lcd.setCursor(x, 1);
      lcd.write(7);
      lcd.write(4);
      lcd.write(2);
      break;
    }
    case 4:
    {
      lcd.setCursor(x,0);
      lcd.write(byte(0));
      lcd.write(4);
      lcd.write(2);
      lcd.setCursor(x, 1);
      lcd.write(char(254));
      lcd.write(char(254));
      lcd.write(2);
      break;
    }
    case 5:
    {
      lcd.setCursor(x,0);
      lcd.write(byte(0));
      lcd.write(6);
      lcd.write(5);
      lcd.setCursor(x, 1);
      lcd.write(7);
      lcd.write(4);
      lcd.write(2);
      break;
    }
    case 6:
    {
      lcd.setCursor(x,0);
      lcd.write(byte(0));
      lcd.write(6);
      lcd.write(5);
      lcd.setCursor(x, 1);
      lcd.write(byte(0));
      lcd.write(4);
      lcd.write(2);
      break;
    }
    case 7:
    {
      lcd.setCursor(x,0);
      lcd.write(1);
      lcd.write(1);
      lcd.write(2);
      lcd.setCursor(x, 1);
      lcd.write(char(254));
      lcd.write(char(254));
      lcd.write(2);
      break;
    }
    case 8:
    {
      lcd.setCursor(x,0);
      lcd.write(byte(0));
      lcd.write(6);
      lcd.write(2);
      lcd.setCursor(x, 1);
      lcd.write(byte(0));
      lcd.write(4);
      lcd.write(2);
      break;
    }
    case 9:
    {
      lcd.setCursor(x,0);
      lcd.write(byte(0));
      lcd.write(6);
      lcd.write(2);
      lcd.setCursor(x, 1);
      lcd.write(7);
      lcd.write(4);
      lcd.write(2);
      break;
    }
  }
}

void clearLargeNumber(byte x) // x is column of upper left corner for large character
{
  lcd.setCursor(x,0);
  lcd.print("   ");
  lcd.setCursor(x,1); 
  lcd.print("   ");
}

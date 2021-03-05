#include "PololuTB9051FTGSingleBrushedDCDriver.h"

// Constructors ////////////////////////////////////////////////////////////////

PololuTB9051FTGSingleBrushedDCDriver::PololuTB9051FTGSingleBrushedDCDriver()
{
  //Pin map
  _M1EN = 4;
  _M1PWM1 = 5;
  _M1PWM2 = 6;
  _M1DIAG = 7;
  _M1OCM = A0;
  _FWDLIMITSW = 8;
  _REVLIMITSW = 9;

  _flipM1 = false;
  _motorfwd = 0;
  _motorrev = 0;
}

PololuTB9051FTGSingleBrushedDCDriver::PololuTB9051FTGSingleBrushedDCDriver(unsigned char M1EN,
                                                                           unsigned char M1DIR,
                                                                           unsigned char M1PWM1,
                                                                           unsigned char M1PWM2,
                                                                           unsigned char M1DIAG,
                                                                           unsigned char M1OCM,
                                                                           unsigned char FWDLIMITSW,
                                                                           unsigned char REVLIMITSW)
{
  _M1EN = M1EN;
  _M1PWM1 = M1PWM1;
  _M1PWM2 = M1PWM2;
  _M1DIAG = M1DIAG;
  _M1OCM = M1OCM;
  _FWDLIMITSW = FWDLIMITSW;
  _REVLIMITSW = REVLIMITSW;
  _flipM1 = false;
  _motorfwd = 0;
  _motorrev = 0;
}

// Public Methods //////////////////////////////////////////////////////////////

void PololuTB9051FTGSingleBrushedDCDriver::init()
{
  // Define pinMode for the pins and set the frequency for timer1.

  pinMode(_M1EN, OUTPUT);
  pinMode(_M1PWM1, OUTPUT);
  pinMode(_M1PWM2, OUTPUT);
  pinMode(_M1DIAG, INPUT_PULLUP);
  pinMode(_M1OCM, INPUT);
  pinMode(_FWDLIMITSW, INPUT_PULLUP);
  pinMode(_REVLIMITSW, INPUT_PULLUP);
}

// Set speed for motor 1, speed is a number between -400 and 400
void PololuTB9051FTGSingleBrushedDCDriver::setM1Speed(int speed)
{
  if (speed == 0)
  {
    PololuTB9051FTGSingleBrushedDCDriver::motorStop();
    return;
  }
  unsigned char forward = 1;
  if (speed < 0)
  {
    speed = -speed; // Make speed a positive quantity
    forward = 0;    // Preserve the direction
  }
  if (speed > 255) // Max PWM dutycycle
    speed = 255;

  if (forward && PololuTB9051FTGSingleBrushedDCDriver::_FWDLIMITSW)
  {
    analogWrite(_M1PWM1, speed);
    digitalWrite(_M1PWM2, LOW);
    PololuTB9051FTGSingleBrushedDCDriver::_motorrev = 0;
    PololuTB9051FTGSingleBrushedDCDriver::_motorfwd = 1;
  }
  else if (!forward && PololuTB9051FTGSingleBrushedDCDriver::_REVLIMITSW)
  {
    //OCR1B = speed;
    analogWrite(_M1PWM2, speed);
    digitalWrite(_M1PWM1, LOW);
    PololuTB9051FTGSingleBrushedDCDriver::_motorfwd = 0;
    PololuTB9051FTGSingleBrushedDCDriver::_motorrev = 1;
  }
}

void PololuTB9051FTGSingleBrushedDCDriver::motorStop()
{
  digitalWrite(_M1PWM1, LOW);
  digitalWrite(_M1PWM2, LOW);
  PololuTB9051FTGSingleBrushedDCDriver::_motorfwd = 0;
  PololuTB9051FTGSingleBrushedDCDriver::_motorrev = 0;
}
boolean PololuTB9051FTGSingleBrushedDCDriver::getFWDLIMITSW()
{
  if (!digitalRead(_FWDLIMITSW))
  {
    return 0;
  }
  else
  {
    return 1;
  }
}
boolean PololuTB9051FTGSingleBrushedDCDriver::getREVLIMITSW()
{
  if (!digitalRead(_REVLIMITSW))
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

boolean PololuTB9051FTGSingleBrushedDCDriver::getFWDstate()
{
  return PololuTB9051FTGSingleBrushedDCDriver::_motorfwd;
}
boolean PololuTB9051FTGSingleBrushedDCDriver::getREVstate()
{
  return PololuTB9051FTGSingleBrushedDCDriver::_motorrev;
}

// Return error status for motor 1
unsigned char PololuTB9051FTGSingleBrushedDCDriver::getM1Fault()
{
  return !digitalRead(_M1DIAG);
}

// Flip direction for motor 1
void PololuTB9051FTGSingleBrushedDCDriver::flipM1(boolean flip)
{
  PololuTB9051FTGSingleBrushedDCDriver::_flipM1 = flip;
}

// Enable the driver for motor 1
void PololuTB9051FTGSingleBrushedDCDriver::enableM1Driver()
{
  digitalWrite(_M1EN, HIGH);
}

// Disable the driver for motor 1
void PololuTB9051FTGSingleBrushedDCDriver::disableM1Driver()
{
  digitalWrite(_M1EN, LOW);
}

// Return motor 1 current value in milliamps.
unsigned int PololuTB9051FTGSingleBrushedDCDriver::getM1CurrentMilliamps()
{
  // 5V / 1024 ADC counts / 500 mV per A = 10 mA per count
  return analogRead(_M1OCM) * 10;
}

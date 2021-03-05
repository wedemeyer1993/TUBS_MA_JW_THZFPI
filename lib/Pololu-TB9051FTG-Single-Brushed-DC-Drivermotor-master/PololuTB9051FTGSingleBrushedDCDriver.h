#pragma once

#include <Arduino.h>

class PololuTB9051FTGSingleBrushedDCDriver
{
public:
  // CONSTRUCTORS
  // Default pin selection.
  PololuTB9051FTGSingleBrushedDCDriver();
  // User-defined pin selection.
  PololuTB9051FTGSingleBrushedDCDriver(unsigned char M1EN,
                                       unsigned char M1DIR,
                                       unsigned char M1PWM1,
                                       unsigned char M1PWM2,
                                       unsigned char M1DIAG,
                                       unsigned char M1OCM,
                                       unsigned char FWDLIMITSW,
                                       unsigned char REVLIMITSW);

  // PUBLIC METHODS
  void init();                          // Initialize pins and timer1 if applicable.
  void setM1Speed(int speed);           // Set speed for M1.
  unsigned char getM1Fault();           // Get fault reading from M1.
  void flipM1(boolean flip);            // Flip the direction of the speed for M1.
  void enableM1Driver();                // Enable the driver for M1.
  void disableM1Driver();               // Disable the driver for M1.
  unsigned int getM1CurrentMilliamps(); // Get current reading for M1.
  void motorStop();
  boolean getFWDLIMITSW();
  boolean getREVLIMITSW();
  boolean getFWDstate();
  boolean getREVstate();
  void checkLimitSw();

private:
  unsigned char _M1PWM1;
  unsigned char _M1PWM2;
  unsigned char _M1EN;
  unsigned char _M1DIAG;
  unsigned char _M1OCM;
  unsigned char _FWDLIMITSW;
  unsigned char _REVLIMITSW;
  boolean _flipM1;
  boolean _motorfwd;
  boolean _motorrev;
};

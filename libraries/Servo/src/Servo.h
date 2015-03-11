/*
Servo.h library class implementation in x86 to support servo motors 
Copyright (C) 2014 Intel Corporation

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef Servo_h
#define Servo_h
#include "variant.h"

// checking if the communication is done using sysfs or cypress
#if PLATFORM_ID == 0x06
#define SERVO_PWM_WITH_I2C           // CYPRESS IC
#else
#define SERVO_PWM_WITH_SYSFS         // PCA9685 IC        
#endif


#ifdef SERVO_PWM_WITH_SYSFS
#include <interrupt.h>
#include <sysfs.h>      // let's use some sysfs functions
#else
#include "Wire.h"       // for cypress (I2C)
#endif

#include "Arduino.h"
#include <stdio.h>

#define MIN_ANGLE              0      // min angle
#define MAX_ANGLE            180      // max angle


#define MIN_PULSE_WIDTH       544     // the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH      2400     // the longest pulse sent to a servo
#define DEFAULT_PULSE_WIDTH  1500     // default pulse width when servo is attached

#define INVALID_SERVO        255

#define MAX_NUMBER_OF_SERVOS  6   // same number of pins in PWM

#define MY_TRACE_PREFIX "ServoX86Lib"


#ifdef SERVO_PWM_WITH_I2C
// Definitions for Galileo

/* How the pins are connected to cypress in Galileo */
#define CYPRESS_I2C_ADDRESS 0x20  // I2C address
static bool currentPWMis188Hz = false;
#else
// Definitions for Galileo Gen2
#define PWM_50Hz                22000000
#define ERROR_FACTOR            1.1       //measured with oscilloscope
// this function pointer is to avoid the conflict of write C ANSI call
// compared to the Servo method called "write"
static int  (*pointer_write)(  int  handle,  const void  *buffer,  unsigned int  nbyte  ) = write;

#endif // PLATFORM_NAME checking


typedef struct  {
  uint8_t pin;
  bool isActive;             // true if this channel is enabled, pin not pulsed if false
} servoPinData_t;

class Servo
{
public:
  Servo();
  uint8_t attach(int16_t pin, bool force48hz = false);           // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
  uint8_t attach(int pin, int min, int max, bool force48hz = false); // as above but also sets min and max values for writes.
  void detach();
  void write(int val);               // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds
  void writeMicroseconds(int value); // Write pulse width in microseconds
  int read();                        // returns current pulse width as an angle between 0 and 180 degrees
  int readMicroseconds();            // returns current pulse width in microseconds for this servo (was read_us() in first release)
  bool attached();                   // return true if this servo is attached, otherwise false

#ifdef SERVO_PWM_WITH_I2C          // CYPRESS IC
  void set48hz();                    // forces cypress to work in 47.8 hertz
  void set188hz();                   // forces cypress to work in 188 hertz (better angle resolution)
#endif


private:

   int min;                       // minimum is this value times 4 added to MIN_PULSE_WIDTH
   int max;                       // maximum is this value times 4 added to MAX_PULSE_WIDTH

   uint8_t index;                 // servo index

   // this compiler does not accept [] should use [6]
   servoPinData_t pinData[6] = {{3,   false},
                                {5,   false},
                                {6,   false},
                                {9,   false},
                                {10,  false},
                                {11,  false}};
 

   static uint8_t counter;   // only for counting

   void prepare_pin(uint8_t pin);
#ifdef SERVO_PWM_WITH_I2C          // CYPRESS IC
  byte transform_cypress_duty_cycle_byte(int microsecs);
#else
   void setFreqInSysFs(int freq);
   void setDuty(int value_duty);
   void enablePin(bool enable=true);
   void disablePin();
   int handle_duty;
   int handle_enable;
#endif

   int usecs;
   bool isAttached;
   byte pin;
   bool is188hz;
   int lastByteInDuty;          // to avoid jitter caused by analogWrite()

};

#endif

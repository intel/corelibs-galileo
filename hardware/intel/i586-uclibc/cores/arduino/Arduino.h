/*
Arduino.h main header with function prototypes for IDE
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
#ifndef __ARDUINO_H__
#define __ARDUINO_H__

#define ARDUINO_LINUX

/* Target c lib */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Arduino conventions */
#include "binary.h"
#include "variant.h"

/* Language/API declaratives */
#include "AnalogIO.h"
#include "BitsAndBytes.h"
#include "OSAbstract.h"
#include "UtilTime.h"
#include "wiring_digital.h"
#include "interrupt.h"

#ifdef __cplusplus
extern "C"{
#endif

#define INPUT		0x00
#define OUTPUT		0x01
#define INPUT_PULLUP	0x02
#define OUTPUT_FAST	0x03
#define INPUT_FAST	0x04

#define DEFAULT 0

#define true 0x1
#define false 0x0

// values defined by libc on the x86 target
#define PI M_PI
#define HALF_PI M_PI_2
#define TWO_PI (2 * M_PI)
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define SERIAL  0x0
#define DISPLAY 0x1

#define LSBFIRST 0
#define MSBFIRST 1

/*
 * Most of the code shipped with Arduino shield assumes it's running on an AVR
 * target, and calls into the avr-gcc specific PROGMEM facility.
 * We are not AVR, so define the macro away.
 */
#define PROGMEM

// TODO: define in interrupts class and include header
//#define interrupts() sei()
//#define noInterrupts() cli()

// TODO: define timer class and include header
//#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
//#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
//#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

#ifdef ARDUINO_MAIN
#define PA 1
#define PB 2
#define PC 3
#define PD 4
#define PE 5
#define PF 6
#define PG 7
#define PH 8
#define PJ 10
#define PK 11
#define PL 12
#endif

#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define sq(x) ((x)*(x))
#define max(x,y) ((x)>(y)?(x):(y))
#define min(x,y) ((x)<(y)?(x):(y))

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus
#include "WCharacter.h"
#include "WString.h"

void tone(uint8_t _pin, unsigned int frequency, unsigned long duration = 0);
void noTone(uint8_t _pin);

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, int val, uint8_t bits=8, uint8_t del=10);

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout = 1000000);

// due to String class is not scoped using namespace, in order to make the
// compilation works with minimum impact the operator is being added on this
// file in order to keep this operator visible to the sketch context.
String operator + ( const char *cstr, const String &str_arg);
#endif


void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, int val, uint8_t bits, uint8_t del);

#include "WMath.h"

#endif /* __ARDUINO_H__ */

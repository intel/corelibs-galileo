/*
AnalogIO.h definition for analog header implementation
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

#ifndef __ANALOG_IO_H__
#define __ANALOG_IO_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Types used for the tables below */
typedef struct _PwmDescription
{
        uint32_t		ulPWMId;		// Identitiy in PWMLib as a PWM
        uint32_t		ulArduinoId;		// Arduino ID
        int			iHandleEnable;		// Persistent handle - open once - use many times
        int			iHandleDuty;		// Persistent handle - open once - use many times
} PwmDescription;

/* Types used for the tables below */
typedef struct _AdcDescription
{
	uint32_t		ulArduinoId;		// Arduino
 	int			iHandle;		// Persistent handle - open once - use many times
} AdcDescription;

void pwmInit(void);
void adcInit(void);
uint32_t analogRead(uint32_t);
void analogReadResolution(uint32_t res);
void analogReference(uint8_t mode);
void analogWrite(uint32_t, uint32_t);

int pin2pwmhandle_enable(uint8_t pin);
int pin2pwmhandle_duty(uint8_t pin);

#ifdef __cplusplus
}
#endif

#endif /* __ANALOG_IO_H__ */

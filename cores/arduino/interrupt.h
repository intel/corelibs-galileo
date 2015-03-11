/* interrupt.h  Interruption interface
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


/*
 * Provides a pseudo interrupt inteface which is broadly an analoge of the Arduino pin based interrupt
 * callback mechanism
 * We don't support 'real' interrupts from kernel to user-space right now since that's way out of scope
 *
 * Author: Bryan O'Donoghue <bryan.odonoghue@intel.com>
 */

#ifndef __INTERRUPT_H__
#define __INTERRUPT_H__

#include <stdint.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LOW  0x0
#define HIGH 0x1
#define RISING 0x02
#define FALLING 0x04
#define CHANGE	0x08

/**
 * interrupt_init
 *
 * Initialise the interrupt queue
 */
int interrupt_init( void );

/**
 * interrupt_fini
 *
 * Finish the interrupt module
 */
void interrupt_fini( void );

/**
 * Attach a callback to an gpio 'interrupt' asynchronous to sketch loop();
 */
void attachInterrupt(uint32_t pin, void (*callback)(void), uint32_t mode);

/**
 * Detach a callback to an gpio 'interrupt' asynchronous to sketch loop();
 */
void detachInterrupt(uint32_t pin);

/**
 * Attach a callback to an timer asynchronous to sketch loop();
 */
void attachTimerInterrupt(void (*callback)(void), int32_t microseconds);


/**
 * setMinimumTimerFreq
 *
 */
int setMinimumTimerFreq(uint32_t microseconds);

#ifdef __cplusplus
}
#endif

#endif /* __INTERRUPT_H__ */


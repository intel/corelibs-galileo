/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified by Manoel Ramon on 21 Jan 2013: 
     - fixed some bugs during Roma Maker Fair / Italy

  Modified by Dan O'Donovan on 29 April 2014: 
    - Reworked pin mux control logic to include buffer and pull-up control
    - Added Fab-E GPIO and PWM pin mapping and variant-specific #defines
    - Modified PWM code to support Galileo Gen2
    - Modified EEPROM examples to reflect Galileo Gen2 EEPROM size
    - Added Software Serial class for Galileo Gen2
    - Includes support for using system console as a TTY

  Modified by Dan O'Donovan on 20 May 2014: 
  Added macros to provide Galileo-specific fastGpioDigitalWrite*()
    - implementations for Galileo and GalileoGen2 boards for backward
      compatibility.  Supports both south- and north-cluster GPIOs which
      exist on Galileo Gen2 board, while retaining support for pre-existing
      Galileo-specific APIs as much as possible.
    - Need to deprecate fastGpioDigitalLatch() and
      fastGpioDigitalWriteDestructive() as these cannot be scaled easily
      to support the NC GPIO pins on Gen2 boards

 Modified by Dino Tinitigan on 3 Jun 2014:
     - attachInterrupt impacts IO performance even after dettachInterrupt 
       on the same pin
	   
 Modified by Dino Tinitigan on 31 July 2014:
     - added fastDigitalRead() implementation
	 - added fastDigitalWrite() implementation
*/


#ifndef _WIRING_DIGITAL_
#define _WIRING_DIGITAL_

#include <stdint.h>
#include "variant.h"		// For pin description structure
#include "fast_gpio_sc.h"
#include "fast_gpio_nc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NONE					0xFFFFFFFF
#define	FN_GPIO_INPUT_PULLUP			0x01
#define	FN_GPIO_INPUT_PULLDOWN			0x02
#define FN_GPIO_INPUT_HIZ			0x04
#define FN_GPIO_OUTPUT				0x08
#define FN_PWM					0x10
#define FN_I2C					0x20
#define FN_ANALOG				0x40
#define FN_UART					0x80
#define FN_SPI					0x100
#define FN_SWITCH				0x200   // Output-only, for toggling board level mux switches, pull-ups, buffers, leds, etc.
#define FN_RESERVED				0x400	// Reserved - not to be touched by sketch layer

/* Combinations of the above for convenience */
#define FN_GPIO_INPUT (FN_GPIO_INPUT_PULLUP | FN_GPIO_INPUT_PULLDOWN | FN_GPIO_INPUT_HIZ)
#define FN_GPIO (FN_GPIO_INPUT | FN_GPIO_OUTPUT)

#define GPIO_DRIVE_PULLUP			0
#define GPIO_DRIVE_PULLDOWN			1
#define GPIO_DRIVE_STRONG			2
#define GPIO_DRIVE_HIZ				3

#define PIN_EINVAL 0xFFFFFFFF

// Describes an individual MUX setting
typedef struct mux_select {
	uint32_t 		ulGPIOId;	// GPIOLib ID that controls the mux
	uint32_t 		ulValue;	// HIGH or LOW output, or NONE to disable output (HiZ INPUT)
	uint32_t		tFunction;	// Function in the HIGH or LOW state
}mux_sel_t;

#define MAX_GPIO_PATH 0x200

/* Types used for the tables below */
typedef struct _PinDescription
{
	uint32_t		ulGPIOId;		// Identitiy in GPIOLib as a GPIO
	uint32_t		ulGPIOAlias;		// Alias pin
	uint32_t		ulFastIOInfo;		// Information on how to use fast IO for pin
	uint32_t		ulArduinoId;		// Arduino ID if any
	uint32_t		ulInitialMuxFn;		// Initial Mux descriptor to apply to this pin
	uint32_t		ulFixedState;		// Fixed to HIGH, LOW, NONE
	mux_sel_t		*ptMuxDesc;		// Describes possible muxes on this pin
	uint32_t		ulMuxDescEntries;	// sizeof ptMuxDesc
	uint32_t		tCurrentType;		// Current State/function of pin
	int			iHandle;		// Persistent handle - open once - use many times
	int			iExtPullup;		// If set, an external pull-up is available.  Disable internal pull-up.
	int			iAlternate;		// If set, switch to the alternate func (pAlternate)
	struct _PinDescription	*pAlternate;		// Pointer to alternate function (NULL if none)
	char			sPath[MAX_GPIO_PATH];	// Path to GPIO - required to poll/select on a gpio in user-space
} PinDescription;

#define MUX_SIZE(x) sizeof(x)/sizeof(mux_sel_t)

typedef struct _PinState
{
	uint32_t	uCurrentPwm;	/* True if currently used as PWM */
	uint32_t	uCurrentInput;	/* True if currently input */
	uint32_t	uCurrentAdc;	/* True if currently used as ADC */
} PinState;

void pinInit(void);
void pinMode(uint8_t pin, uint8_t mode);
int pinModeIRQ(uint8_t pin, int8_t mode);
void digitalWrite(register uint8_t pin, register uint8_t val);
int digitalRead(uint8_t pin);
uint8_t fastDigitalRead(uint8_t pin);
void fastDigitalWrite(register uint8_t pin, register uint8_t val);
int pinToFastPin(register uint8_t pin);

// interrupt.c dependencies
int pin2handle(uint8_t pin);
int pinHandleReopen(uint8_t index);
int pinGetIndex(uint8_t pin);
char * pin2path(uint8_t pin);
int gpio2gpiohandle(uint32_t gpio);
int pin2gpiohandle(uint8_t pin);

#ifdef __cplusplus
} // extern "C"
#endif

// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.
//
#define digitalPinToPort(P)		( g_APinDescription[P].pPort )
#define digitalPinToBitMask(P)		( g_APinDescription[P].ulPin )
#define digitalPinToTimer(P)		(  )
#define portOutputRegister(port)	( &(port->PIO_ODSR) )
#define portInputRegister(port)		( &(port->PIO_PDSR) )

#endif /* _WIRING_DIGITAL_ */


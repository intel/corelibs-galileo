/*
  wiring_digital.c - digital input and output functions
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2005-2006 David A. Mellis
  Copyright (c) 2014 Intel Corporation

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

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
    - Fixed: attachInterrupt impacts IO performance even after dettachInterrupt 
       on the same pin

  Modified by Dino Tinitigan on 19 June 2014
    - Fixed: pinMode breaks PWM on same pin.
	
  Modified by Dino Tinitigan on 31 July 2014:
    - added fastDigitalRead() implementation
	- added fastDigitalWrite() implementation

*/

#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <trace.h>
#include "Arduino.h"
#include "sysfs.h"
#include "fast_gpio_sc.h"
#include "fast_gpio_nc.h"
#include "Mux.h"

#define MY_TRACE_PREFIX "wiring_digital"

static const int pin2gpio(uint8_t pin)
{
	PinDescription *p = NULL;

	if (pin >= GPIO_TOTAL)
		return PIN_EINVAL;

	p = &g_APinDescription[ardPin2DescIdx[pin]];
	pin2alternate(&p);

	return p->ulGPIOId;
}

static const int pin2gpioX(uint8_t pin)
{
	//do not use alternate pin. Workaround for dettachInterrupt that was attached in "CHANGE" mode
	PinDescription *p = NULL;

	if (pin >= GPIO_TOTAL)
		return PIN_EINVAL;

	p = &g_APinDescription[ardPin2DescIdx[pin]];

	return p->ulGPIOId;
}

// Just trust the input - this is a requirement for running select(); on gpiolib derived handles
int pinHandleReopen(uint8_t index)
{
	PinDescription *p = &g_APinDescription[index];
	pin2alternate(&p);

	close(p->iHandle);
	p->iHandle = open(p->sPath, O_RDWR);

	return p->iHandle;
}

int pinGetIndex(uint8_t pin)
{
	PinDescription *p = NULL;

	if (unlikely(pin >= GPIO_TOTAL))
		return PIN_EINVAL;

	// Scan mappings
	p = &g_APinDescription[ardPin2DescIdx[pin]];
//	pin2alternate(&p);

	return ardPin2DescIdx[p->ulArduinoId];

}

int gpio2gpiohandle(uint32_t gpio)
{
	int i = 0;
	PinDescription *p = NULL;
	for (i = 0; i < sizeof_g_APinDescription; i ++) {
		p = &g_APinDescription[i];
		pin2alternate(&p);
		if (p->ulGPIOId == gpio) {
			return p->iHandle;
		}
	}
	return PIN_EINVAL;
}

int pin2gpiohandle(uint8_t pin)
{
	PinDescription *p = NULL;

	if (pin >= GPIO_TOTAL)
		return PIN_EINVAL;

	// Scan mappings
	p = &g_APinDescription[ardPin2DescIdx[pin]];
	pin2alternate(&p);

	return p->iHandle;
}

char * pin2path(uint8_t pin)
{
	PinDescription *p = NULL;

	if (pin >= GPIO_TOTAL)
		return NULL;

	// Scan mappings
	p = &g_APinDescription[ardPin2DescIdx[pin]];
	pin2alternate(&p);

	return p->sPath;
}

void pinMode(uint8_t pin, uint8_t mode)
{
	int ret = 0;
	uint32_t gpio = 0;
	PinDescription *p = NULL;

	ret = variantPinMode(pin, mode);
	if (ret) {
		trace_error("%s: variantPinMode failed\n", __func__);
		return;
	}

	p = &g_APinDescription[ardPin2DescIdx[pin]];
	gpio = pin2gpio(pin);
	if (gpio == PIN_EINVAL){
		trace_error("%s: pin %d out of range (gpio%d)", __func__, pin, gpio);
		return;
	}

	turnOffPWM(pin);
	trace_debug("%s: pin=%d, mode=%d", __func__, pin, mode);

	switch (mode) {
	case INPUT:
		ret = muxSelect(pin, FN_GPIO_INPUT_HIZ);
		if (ret) {
			trace_error("%s: can't set mux for pin%d\n", __func__, pin);
			return;
		}

		g_APinState[pin].uCurrentInput = 1;

		/* Cover alternate gpios too */
		for (; p; p = p->pAlternate) {
			/* Hi-Z  */
			trace_debug("%s: setting gpio%u to input_hiz",
					__func__, p->ulGPIOId);
			sysfsGpioSetDrive(p->ulGPIOId, GPIO_DRIVE_HIZ);
			sysfsGpioDirection(p->ulGPIOId, 0, NONE);
		}
		break;
	case INPUT_PULLUP:
		ret = muxSelect(pin, FN_GPIO_INPUT_PULLUP);
		if (ret) {
			trace_error("%s: can't set mux for pin%d\n", __func__, pin);
			return;
		}

		g_APinState[pin].uCurrentInput = 1;

		/* Cover alternate gpios too */
		for (; p; p = p->pAlternate) {
			/* Enable pullup  */
			trace_debug("%s: setting gpio%u to input_pullup",
					__func__, p->ulGPIOId);
			if (p->iExtPullup)
				sysfsGpioSetDrive(p->ulGPIOId, GPIO_DRIVE_HIZ);
			else
				sysfsGpioSetDrive(p->ulGPIOId, GPIO_DRIVE_PULLUP);
			sysfsGpioDirection(p->ulGPIOId, 0, NONE);
		}
		break;
	case OUTPUT_FAST:
		g_APinState[pin].uCurrentInput = 0;
		trace_debug("%s: setting gpio%u to output_fast",
				__func__, pin2gpio(pin));
		variantEnableFastGpio(pin);
		/* Ensure that alternate pin selection is done BEFORE doing mux config */
		ret = muxSelect(pin, FN_GPIO_OUTPUT);
		if (ret) {
			trace_error("%s: can't set mux for pin%d\n", __func__, pin);
			return;
		}

		/* Refresh pin2gpio */
		sysfsGpioDirection(pin2gpio(pin), 1, 0);
		break;
	case INPUT_FAST:
		g_APinState[pin].uCurrentInput = 1;
		trace_debug("%s: setting gpio%u to input_fast",
				__func__, pin2gpio(pin));
		variantEnableFastGpio(pin);
		/* Ensure that alternate pin selection is done BEFORE doing mux config */
		ret = muxSelect(pin, FN_GPIO_INPUT_HIZ);
		if (ret) {
			trace_error("%s: can't set mux for pin%d\n", __func__, pin);
			return;
		}

		/* Refresh pin2gpio */
		sysfsGpioDirection(pin2gpio(pin), 0, NONE);
		break;
	default:
		ret = muxSelect(pin, FN_GPIO_OUTPUT);
		if (ret) {
			trace_error("%s: can't set mux for pin%d\n", __func__, pin);
			return;
		}

		g_APinState[pin].uCurrentInput = 0;
		trace_debug("%s: setting gpio%u to output",
					__func__, pin2gpio(pin));
		/* Output.  Strong drive mode, default value to 0  */
		sysfsGpioSetDrive(gpio, GPIO_DRIVE_STRONG);
		sysfsGpioDirection(gpio, 1, 0);
		break;
	}

	g_APinState[pin].uCurrentAdc = 0;
}

/**
 * pinModeIRQ
 *
 * pin - arduino pin
 * mode - one of the modes defined for arudino IRQs LOW , CHANGE, RISING, FALLING, HIGH
 */
int pinModeIRQ(uint8_t pin, int8_t mode)
{
	int ret = 0;
	uint32_t gpio = 0;
	PinDescription *p = NULL;

	ret = variantPinModeIRQ(pin, mode);
	if (ret) {
		trace_error("%s: variantPinMode failed\n", __func__);
		return ret;
	}

	// Configure pin as input
	ret = muxSelect(pin, FN_GPIO_INPUT_HIZ);
	if (ret) {
		trace_error("%s: can't set mux for pin%d\n", __func__, pin);
		return;
	}
	g_APinState[pin].uCurrentInput = 1;
	/* Cover alternate gpios too */
	p = &g_APinDescription[ardPin2DescIdx[pin]];
	for (; p; p = p->pAlternate) {
		/* Hi-Z  */
		trace_debug("%s: setting gpio%u to input_hiz",
			    __func__, p->ulGPIOId);
		sysfsGpioSetDrive(p->ulGPIOId, GPIO_DRIVE_HIZ);
		sysfsGpioDirection(p->ulGPIOId, 0, NONE);
	}

	gpio = pin2gpio(pin);
	if (gpio == PIN_EINVAL){
		trace_error("%s: pin %d out of range (gpio%d)", __func__, pin, gpio);
		return gpio;
	}

	trace_debug("%s: pin=%d, gpio=%d, mode=%d", __func__, pin, gpio, mode);

	// Now set the mode for IRQ granularity
	switch (mode) {
		case NONE:
			//do on both alternate SoC and expander gpio since it defaults to the alternate if available
			//in this case if "CHANGE" is the mode used for attachInterrupt() it would use the expander GPIO
			//but "NONE" defaults to SoC becuase it is not "CHANGE"
			gpio = pin2gpioX(pin);
			ret = sysfsGpioEdgeConfig(gpio, mode);
			gpio = pin2gpio(pin);
			ret = sysfsGpioEdgeConfig(gpio, mode);
			break;
		case CHANGE:
		case RISING:
		case FALLING:
			ret = sysfsGpioEdgeConfig(gpio, mode);
			break;
		case LOW:
		case HIGH:
			// Implies edge config none
			ret = sysfsGpioEdgeConfig(gpio, NONE);
			if ( ret < 0 )
				break;

			// Set to active high or active low
			ret = sysfsGpioLevelConfig(gpio, mode);
			break;
		default:
			ret = PIN_EINVAL;
			break;
	}

	return ret;
}

static void digitalWriteSetVal(register uint32_t idx, register uint8_t pin, register uint8_t val)
{
	int ret = 0;

	PinDescription *p = &g_APinDescription[idx];
	pin2alternate(&p);

	/* perform screaming fast direct register access */
	if (likely(p->ulFastIOInfo)) {
		fastGpioDigitalWrite(p->ulFastIOInfo, val);
		return;
	}

	/* else go through sysfs like good citizens */
	if (unlikely(p->iHandle == PIN_EINVAL)){
		trace_error("%s: pin %d out of range (gpio%d)", __func__, pin, pin2gpio(pin));
		return;
	}

	ret = sysfsGpioSet(p->iHandle, (unsigned int)val);
	if (unlikely(ret)) {
		trace_error( "%s: err=%d", __func__, ret);
	}
}

void digitalWrite(register uint8_t pin, register uint8_t val)
{
	uint32_t idx;

	if (unlikely(pin >= GPIO_TOTAL))
		return;

	if (unlikely(g_APinState[pin].uCurrentInput)) {
		if (val) {
			trace_debug("%s: input pin%u driven high: enabling "
					"pullup", __func__, pin);
			pinMode(pin, INPUT_PULLUP);
		} else {
			trace_error("%s: input pin%u driven low!", __func__,
					pin);
		}
		return;
	}

	if (unlikely(g_APinState[pin].uCurrentPwm)) {
		turnOffPWM(pin);
		muxSelect(pin, FN_GPIO_OUTPUT);
	}

	idx = pinGetIndex(pin);
	digitalWriteSetVal(idx, pin, val);

	// alias - enable w/o on Fab D for waggle of pin 20 to compensate for pin 13 error
	if (unlikely(g_APinDescription[idx].ulGPIOAlias != NONE)){
		idx = pinGetIndex(g_APinDescription[idx].ulGPIOAlias);
		digitalWriteSetVal(idx, g_APinDescription[idx].ulGPIOAlias, val);
	}
	//trace_debug("%s: pin=%d, handle=%d, val=%d", __func__, pin, handle, val);
}

int digitalRead(uint8_t pin)
{
	PinDescription *p = NULL;
	uint32_t idx = 0;
	int ret;
	int handle = pin2gpiohandle(pin);

	if (handle == PIN_EINVAL){
		trace_error("%s: pin %d out of range", __func__, pin);
		return -1;
	}

	idx = pinGetIndex(pin);
	p = &g_APinDescription[idx];
	pin2alternate(&p);
	if (likely(p->ulFastIOInfo)) {
		return fastGpioDigitalRead(p->ulFastIOInfo) ? 1 : 0;
	}

	//trace_debug("%s: pin=%d, handle=%d", __func__, pin, handle);
	ret = sysfsGpioGet(handle);
	if (1 == ret) {
		return 1;
	}
	return 0;
}

//Easy to use wrapper for fastGpioDigitalRead
uint8_t fastDigitalRead(uint8_t pin)
{
	int fastPin;
	fastPin = pinToFastPin(pin);
	if(fastPin)
	{
		return fastGpioDigitalRead(fastPin) ? 1:0;
	}
	else
	{
		//no fast mode for this pin. Use slow mode
		return digitalRead(pin);
	} 
}

//Easy to use wrapper for fastGpioDigitalWrite.
void fastDigitalWrite(register uint8_t pin, register uint8_t val)
{
	register int fastPin;
	fastPin = pinToFastPin(pin);
	if(fastPin)
	{
		return fastGpioDigitalWrite(fastPin, val);
	}
	else
	{
		//no fast mode for this pin. Use slow mode
		return digitalWrite(pin, val);
	} 
	
}

int pinToFastPin(register uint8_t pin)
{
	switch (pin)
    {
	
#if PLATFORM_ID != 0x06
	// this is Galileo Gen 2
	case 0:
		return GPIO_FAST_IO0;
		break;              
	case 1:
		return GPIO_FAST_IO1;
		break;
#endif
	//both Gen1 and Gen2
	case 2:
		return GPIO_FAST_IO2;
		break;
	case 3:
		return GPIO_FAST_IO3;
		break;
#if PLATFORM_ID != 0x06               
	// this is Galileo Gen 2 - no fast I/O for pins 7 and 8
	case 4:
		return GPIO_FAST_IO4;
		break;
	case 5:
		return GPIO_FAST_IO5;
		break;
	case 6:
		return GPIO_FAST_IO6;
		break;
	case 9: 
		return GPIO_FAST_IO9;
		break;
	case 10:
		return GPIO_FAST_IO10;
		break;
	case 11:
		return GPIO_FAST_IO11;
		break;
	case 12:
		return GPIO_FAST_IO12;
		break;
	case 13:
		return GPIO_FAST_IO13;
		break;
#endif               
	default:
		return 0;
		break;
	}
}

/*
 * Initialise the GPIO for execution.
 *
 * 1. Export all the Izmir's GPIOs to sysfs
 * 2. Initialise default pin routing on Arduino shield pins
 *
 */
void pinInit(void)
{
	int i = 0;
	PinDescription *p = NULL;

	memset(&ardPin2DescIdx, -1, sizeof(ardPin2DescIdx));
	/*
	 * Export all GPIOs
	 */
	for (i = 0; i < sizeof_g_APinDescription; i++) {
		for (p = &g_APinDescription[i]; p != NULL; p = p->pAlternate) {
			/* Skip past any system reserved pin */
			if (p->tCurrentType == FN_RESERVED)
				continue;

			p->iHandle = sysfsGpioExport(p->ulGPIOId, p->sPath, sizeof(p->sPath));
			if (p->iHandle < 0){
				trace_error("unable to open GPIO %d", p->ulGPIOId);
				assert(0);
			}
		}

		/* Fill entry in uint32_t ardPin2PinDescIdx */
		if (g_APinDescription[i].ulArduinoId != NONE){
			ardPin2DescIdx[g_APinDescription[i].ulArduinoId] = i;
		}
	}

	/* Loop through array and set GPIO to the initial state indicated by the descriptor array */
	for (i = 0; i < sizeof_g_APinDescription; i++) {
		for (p = &g_APinDescription[i]; p != NULL; p = p->pAlternate) {

			if (p->iHandle < 0 && p->tCurrentType != FN_RESERVED) {
				trace_error("%s: invalid iHandle for gpio%u",
						__func__, p->ulGPIOId);
				continue;
			}

			if (p->tCurrentType & FN_SWITCH) {
				/*
				 * GPIOs not exported to Arduino header, used for switching board-level components.
				 * Default drive mode is output with pullup on, unless no output level specified.
				 */
				if (p->ulFixedState == NONE) {
					/* No output, so switch to HiZ input */
					sysfsGpioSetDrive(p->ulGPIOId, GPIO_DRIVE_HIZ);
					sysfsGpioDirection(p->ulGPIOId, 0, NONE);
				} else {
					/* Output defined as LOW or HIGH */
					sysfsGpioSetDrive(p->ulGPIOId, GPIO_DRIVE_PULLUP); /* For Fab-D backward compatibility */
					sysfsGpioDirection(p->ulGPIOId, 1, p->ulFixedState);
				}
			} else if (p->tCurrentType & FN_GPIO) {
				/*
				 * GPIOs exported on Arduino headers are by default input with no pullup unless specified
				 */
				switch (p->ulFixedState) {
				case HIGH:
					sysfsGpioSetDrive(p->ulGPIOId, GPIO_DRIVE_PULLUP);
					break;
				case LOW:
					sysfsGpioSetDrive(p->ulGPIOId, GPIO_DRIVE_PULLDOWN);
					break;
				default:
					sysfsGpioSetDrive(p->ulGPIOId, GPIO_DRIVE_HIZ);
				}
				sysfsGpioDirection(p->ulGPIOId, 0, NONE);
			} else {
				trace_info("%s gpiolib %d is not a candidate for direction configuration", __func__, p->ulGPIOId);
			}
		}
	}

	/* Now set the inital mux  */
	muxInit();

	/* Initialise fast path to GPIO SC */
	if (fastGpioSCInit() != 0){
			trace_error("Unable to initialise SC fast GPIO mode!");
	}
	/* Finally initialise fast path to GPIO NC */
	if (fastGpioNCInit() != 0){
			trace_error("Unable to initialise NC fast GPIO mode!");
	}
	return;
}


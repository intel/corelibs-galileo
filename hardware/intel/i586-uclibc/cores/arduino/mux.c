/* mux.c high level abstraction for muxing across boards
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

#include <Arduino.h>		// Contains types
#include <errno.h>			// -EINVAL and friends
#include <Mux.h>
#include "variant.h"		// Contains board specific data to drive muxing options
#include "sysfs.h"
#include <trace.h>

#define MY_TRACE_PREFIX "mux"

/* If alternate function is enabled, skip to it */
void pin2alternate(PinDescription **p_ptr)
{
	PinDescription *p = *p_ptr;

	for (; p && p->iAlternate && p->pAlternate; p = p->pAlternate) {
//		trace_debug("%s: arduino_pin=%u: skipping to "
//			    "alternate function (gpio%d)", __func__,
//			    p->pAlternate->ulArduinoId,
//			    p->pAlternate->ulGPIOId);
	}

	*p_ptr = p;
}

/**
 * muxSelect
 *
 * Does the nasty business of iterating through the array matching GPIO ids and setting muxing based on tFunction against
 * given arduino_pin
 */
int muxSelect(uint8_t arduino_pin, uint32_t tFunction)
{
	int i = 0, j = 0, k = 0, matched = 0;
	mux_sel_t sel;
	PinDescription *p = NULL;
	int ret = -EINVAL;

	for (i = 0; i < sizeof_g_APinDescription; i++) {
		p = &g_APinDescription[i];

		/* Find Arduino pin mapping */
		if (p->ulArduinoId == arduino_pin){

			/* If alternate function is enabled, skip to it */
			for (; p && p->iAlternate && p->pAlternate;
			     p = p->pAlternate) {
				trace_debug("%s: arduino_pin=%u: skipping to "
					    "alternate function (gpio%d)",
					    __func__,
					    p->pAlternate->ulArduinoId,
					    p->pAlternate->ulGPIOId);
			}
			if (p->ptMuxDesc == NULL){
				/* No muxing options for this pin */
				return 0;
			}

			/* Find possible mux entries */
			for ( j = 0; j < p->ulMuxDescEntries; j++){

				sel = p->ptMuxDesc[j];
				if (sel.tFunction & tFunction){

					/* Bit mask matched - set state as indicated */
					for (k = 0; k < sizeof_g_APinDescription; k++){
						if (g_APinDescription[k].ulGPIOId == sel.ulGPIOId){
							trace_info("%s: mux_sel gpio%u:=%u (arduino_pin=%u)",
								__func__, sel.ulGPIOId, sel.ulValue, arduino_pin);
							if (sel.ulValue == NONE) {
								/* No output, so switch to HiZ input */
								sysfsGpioDirection(sel.ulGPIOId, 0, NONE);
								sysfsGpioSetDrive(sel.ulGPIOId, GPIO_DRIVE_HIZ);
							} else {
								/* Output defined as LOW or HIGH */
								sysfsGpioDirection(sel.ulGPIOId, 1, sel.ulValue);
								sysfsGpioSetDrive(sel.ulGPIOId, GPIO_DRIVE_STRONG);
							}
							matched = 1;
							break;
						}
					}
				}
			}
			break;
		}
	}
	return matched ? 0 : -EINVAL;
}

int muxSelectAnalogPin(uint8_t pin)
{
	int i = 0;

	if (pin >= NUM_ANALOG_INPUTS) {
		return -EINVAL;
	}

	//trace_debug("%s(%u): gpio%u:=%u", __func__, pin, mux.sel_id, mux.sel_val);

	i = muxSelect(mux_sel_analog[pin], FN_ANALOG);
	if (i < 0)
		return i;

	return 0;
}

int muxSelectUart(uint8_t interface)
{
	int i = 0, ret = 0;

	if (interface >= NUM_UARTS){
		return -EINVAL;
	}

	// Require both pins - to be described
	if (MUX_SEL_NONE == mux_sel_uart[interface][0] && MUX_SEL_NONE == mux_sel_uart[interface][1]) {
		return 0;	// No muxing to be done
	}

	for ( i = 0; i < MUX_DEPTH_UART; i++){
		ret = muxSelect(mux_sel_uart[interface][i], FN_UART);
		if (ret < 0)
			return ret;
	}
	return ret;
}

int muxDeselectUart(uint8_t interface)
{
	int i = 0, ret = 0;

	if (interface >= NUM_UARTS){
		return -EINVAL;
	}

	// Require both pins - to be described
	if (MUX_SEL_NONE == mux_sel_uart[interface][0] && MUX_SEL_NONE == mux_sel_uart[interface][1]) {
		return 0;	// No muxing to be done
	}

	for ( i = 0; i < MUX_DEPTH_UART; i++){
		ret = muxSelect(mux_sel_uart[interface][i], FN_GPIO_INPUT_HIZ);
		if (ret < 0)
			return ret;
	}
	return ret;
}

int muxSelectSpi(uint8_t interface)
{
	int i = 0, ret = -EINVAL;

	if (interface >= NUM_SPI){
		return -EINVAL;
	}

	for ( i = 0; i < MUX_DEPTH_SPI; i++){
		if (mux_sel_spi[interface][i] == MUX_SEL_NONE)
			continue;
		ret = muxSelect(mux_sel_spi[interface][i], FN_SPI);
		if (ret < 0)
			return ret;
	}
	return ret;
}

int muxSelectI2c(uint8_t interface)
{
	/* Selecting SCL will automatically route out SDA too.  */
	return muxSelect(SCL, FN_I2C);
}

int muxSelectPwmPin(uint8_t pin)
{
	return muxSelect(pin, FN_PWM);
}

int muxInit(void)
{
	int i = 0, j = 0;
	mux_sel_t sel;
	PinDescription *p = NULL;

	/*
	 * Setup initial muxes as indicated by controller array
	 */
	for (i = 0; i < sizeof_g_APinDescription; i++) {
		p = &g_APinDescription[i];

		/* If alternate function is enabled, skip to it */
		for (; p && p->iAlternate && p->pAlternate; p = p->pAlternate) {
			trace_debug("%s: arduino_pin=%u: skipping to "
				    "alternate function (gpio%d)", __func__,
				    p->pAlternate->ulArduinoId,
				    p->pAlternate->ulGPIOId);
		}

		/* Skip past any GPIO entry that doesn't have a persistent handle or a relevant mux entry */
		if (p->iHandle >= 0 && p->ulArduinoId != NONE && p->ptMuxDesc != NULL && p->ulInitialMuxFn != NONE) {
			trace_debug("%s: setting up mux for gpio%d", __func__, p->ulGPIOId);
			muxSelect(p->ulArduinoId, p->ulInitialMuxFn);
		}
	}
	return 0;
}

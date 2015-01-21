/*
  Copyright (c) 2011 Arduino.  All right reserved.
  Copyright (c) 2013 Intel Corporation.

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
*/

//////////////////////////// Galileo Fab C ////////////////////////////

#include <Arduino.h>
#include <errno.h>
#include <sysfs.h>
#include "variant.h"

//Bindings to Arduino
#include "RingBuffer.h"
#include "TTYUART.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * MUX description
 *
 * TODO handle the interrupt-capable gpios
 */

int mux_sel_analog[NUM_ANALOG_INPUTS];

int mux_sel_uart[NUM_UARTS][MUX_DEPTH_UART];

int  mux_sel_spi[NUM_SPI][MUX_DEPTH_SPI];

#if 0
/*
 * Pins descriptions
 */
PinDescription g_APinDescription[]=
{
  // TODO:
  // ----------------------
  // 0/1 - UART (Serial)
  { 0, 2, GPIO_CYPRESS, PINTYPE_OPENDRAIN },
  { 1, 7, GPIO_CYPRESS, PINTYPE_OPENDRAIN },
  { 2, 4, GPIO_CYPRESS, PINTYPE_OPENDRAIN },
  { 3, 1, GPIO_CYPRESS, PINTYPE_OPENDRAIN },
  { 4, 0, GPIO_CYPRESS, PINTYPE_OPENDRAIN },
  { 5, 15, GPIO_CYPRESS, PINTYPE_OPENDRAIN },
  { 6, 14, GPIO_CYPRESS, PINTYPE_OPENDRAIN },
  { 7, 13, GPIO_CYPRESS, PINTYPE_OPENDRAIN },
  { 8, 12, GPIO_CYPRESS, PINTYPE_OPENDRAIN },
  { 9, 11, GPIO_CYPRESS, PINTYPE_OPENDRAIN },
  { 10, 10, GPIO_CYPRESS, PINTYPE_OPENDRAIN },
  { 13, 3, GPIO_CYPRESS, PINTYPE_OPENDRAIN },
} ;
#endif

// Sorted by Linux GPIO ID
PinDescription g_APinDescription[]=
{
//	gpiolib	alias	fastinf	ardid	Inital Mux	tFixedState,	ptMuxDesc,	MuxCount	type		Handle
	{ 4, 	NONE,	NULL,	2, 	NONE, 		NONE, 		NULL, 		0,		FN_GPIO,	-1 },
	{ 1, 	NONE,	NULL,	3, 	NONE, 		NONE, 		NULL, 		0,		FN_GPIO,	-1 },
	{ 0, 	NONE,	NULL,	4, 	NONE, 		NONE, 		NULL, 		0,		FN_GPIO,	-1 },
	{ 15, 	NONE,	NULL,	5, 	NONE, 		NONE, 		NULL, 		0,		FN_GPIO,	-1 },
	{ 14, 	NONE,	NULL,	6, 	NONE, 		NONE, 		NULL, 		0,		FN_GPIO,	-1 },
	{ 13, 	NONE,	NULL,	7,	NONE, 		NONE, 		NULL, 		0,		FN_GPIO,	-1 },
	{ 12, 	NONE,	NULL,	8,	NONE, 		NONE, 		NULL, 		0,		FN_GPIO,	-1 },
	{ 11, 	NONE,	NULL,	9, 	NONE, 		NONE, 		NULL,		0,		FN_GPIO,	-1 },
	{ 10, 	NONE,	NULL,	10, 	NONE, 		NONE, 		NULL,		0,		FN_GPIO,	-1 },
	{  3, 	NONE,	NULL,	13, 	NONE, 		NONE, 		NULL,		0,		FN_GPIO,	-1 },
};

uint32_t sizeof_g_APinDescription;
uint32_t ardPin2DescIdx[GPIO_TOTAL];

// Sorted by Linux PWM ID
PwmDescription g_APwmDescription[] = {
};
uint32_t sizeof_g_APwmDescription;

AdcDescription g_AdcDescription[] = {
	{ 0,	-1 },
	{ 1,	-1 },
	{ 2,	-1 },
	{ 3,	-1 },
	{ 4,	-1 },
	{ 5,	-1 },
};
uint32_t sizeof_g_AdcDescription;

// Sorted Arduino Pin ID
PinState g_APinState[]=
{
	/* Note PWM-capable pins are output by default */
	/* uCurrentPwm	uCurrentInput	uCurrentAdc		*/
	{ 0,		1,		0 },	/* 0		*/
	{ 0,		1,		0 },	/* 1		*/
	{ 0,		1,		0 },	/* 2		*/
	{ 0,		1,		0 },	/* 3  - PWM	*/
	{ 0,		1,		0 },	/* 4 		*/
	{ 0,		1,		0 },	/* 5  - PWM 	*/
	{ 0,		1,		0 },	/* 6  - PWM	*/
	{ 0,		1,		0 },	/* 7 		*/
	{ 0,		1,		0 },	/* 8 		*/
	{ 0,		1,		0 },	/* 9  - PWM	*/
	{ 0,		1,		0 },	/* 10 - PWM	*/
	{ 0,		1,		0 },	/* 11 - PMW	*/
	{ 0,		1,		0 },	/* 12		*/
	{ 0,		1,		0 },	/* 13		*/
	{ 0,		1,		0 },	/* 14 - ADC	*/
	{ 0,		1,		0 },	/* 15 - ADC	*/
	{ 0,		1,		0 },	/* 16 - ADC	*/
	{ 0,		1,		0 },	/* 17 - ADC	*/
	{ 0,		1,		0 },	/* 18 - ADC	*/
	{ 0,		1,		0 },	/* 19 - ADC	*/
};
uint32_t sizeof_g_APinState;

#ifdef __cplusplus
}
#endif


RingBuffer rx_buffer1;
RingBuffer rx_buffer2;

TTYUARTClass Serial(&rx_buffer1, 0);		// ttyGS0
TTYUARTClass Serial1(&rx_buffer2, 1);		// ttyQRK0

#if 0
const unsigned mapUnoPinToSoC(uint8_t pin)
{
	if (pin >= NUM_DIGITAL_PINS) {
		return -EINVAL;
	}
	return g_APinDescription[pin].ulPinId;
}

/*
 * Initialise the GPIO for execution.
 *
 * 1. Export all the Izmir's GPIOs to sysfs
 * 2. Initialise default pin routing on Arduino shield
 * 3. TODO reset pin
 *
 * XXX error reporting
 */
static int initGPIO(void)
{
	int i = 0;

	/*
	 * Export all Izmir GPIOs and initialise as input.
	 * Some GPIOs are restricted: let the setup routine fail in this
	 * case.
	 */
	for (i = 0; i < GPIO_TOTAL; i++) {
		sysfsGpioExport(i);
		sysfsGpioDirection(i, 0);
	}

	return 0;
}
#endif

// ----------------------------------------------------------------------------

int variantPinMode(uint8_t pin, uint8_t mode)
{
	return 0;
}

int variantPinModeIRQ(uint8_t pin, uint8_t mode)
{
	return 0;
}

void turnOnPWM(uint8_t pin)
{
}

void turnOffPWM(uint8_t pin)
{
}

void variantEnableFastGpio(int pin)
{
}

void __libc_init_array(void);

#if 0
void init_mux(struct mux_sel ** mux, uint32_t idx, uint32_t jdx)
{
	uint32_t i, j;
	struct mux_sel * pmux = (struct mux_sel*)mux;

	for ( i = 0; i < idx; i++){
		for(j = 0; j <jdx; j++, pmux++){
			pmux->sel_id = MUX_SEL_NONE;
			pmux->sel_val = MUX_SEL_NONE;
		}
	}
}
#endif

void init( int argc, char * argv[] )
{
	// FAB C muxes nothing
	#if 0
	init_mux((struct mux_sel**)&mux_sel_analog, NUM_ANALOG_INPUTS, MUX_DEPTH_ANALOG);
	init_mux((struct mux_sel**)&mux_sel_uart, NUM_UARTS, MUX_DEPTH_UART);
	init_mux((struct mux_sel**)&mux_sel_spi, NUM_SPI, MUX_DEPTH_SPI);
	init_mux((struct mux_sel**)&mux_sel_i2c, NUM_I2C, MUX_DEPTH_I2C);
	#endif

	if(argc > 1)
		if(Serial.init_tty(argv[1]) != 0)
			return;

	if(Serial1.init_tty(LINUX_SERIAL1_TTY) != 0)
		return;

	sizeof_g_APinDescription = sizeof(g_APinDescription)/sizeof(struct _PinDescription);
	pinInit();
	sizeof_g_APwmDescription = sizeof(g_APwmDescription)/sizeof(struct _PwmDescription);
	pwmInit();
	sizeof_g_AdcDescription = sizeof(g_AdcDescription)/sizeof(struct _AdcDescription);
	adcInit();
	sizeof_g_APinState = sizeof(g_APinState)/sizeof(struct _PinState);
}


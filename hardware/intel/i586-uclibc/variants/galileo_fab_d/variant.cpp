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

#include <Arduino.h>
#include <errno.h>
#include <interrupt.h>
#include <Mux.h>
#include <sysfs.h>
#include <trace.h>
#include "variant.h"

//Bindings to Arduino
#include "RingBuffer.h"
#include "TTYUART.h"

#define MY_TRACE_PREFIX "variant"

#ifdef __cplusplus
extern "C" {
#endif

//////////////////////////// Galileo Fab D ////////////////////////////

const int mux_sel_analog[NUM_ANALOG_INPUTS] = {
	MUX_SEL_AD7298_VIN0,
	MUX_SEL_AD7298_VIN1,
	MUX_SEL_AD7298_VIN2,
	MUX_SEL_AD7298_VIN3,
	MUX_SEL_AD7298_VIN4,
	MUX_SEL_AD7298_VIN5,
};

const int mux_sel_uart[NUM_UARTS][MUX_DEPTH_UART] = {
	/* This is auto-indexed (board pinout) */
	{MUX_SEL_NONE, MUX_SEL_NONE},				// ttyGS0 - USB not muxed
	{MUX_SEL_UART0_RXD,	MUX_SEL_UART0_TXD},		// ttyQRK0 - muxed
	{MUX_SEL_NONE, MUX_SEL_NONE},				// ttyQRK1 - not muxed
};

const int  mux_sel_spi[NUM_SPI][MUX_DEPTH_SPI] = {
	{
		MUX_SEL_NONE,
		MUX_SEL_NONE,
		MUX_SEL_NONE
	},
	{
		MUX_SEL_SPI1_MOSI,
		MUX_SEL_SPI1_MISO,
		MUX_SEL_SPI1_SCK
	},
};

#if 0
const int mux_sel_i2c[NUM_I2C][MUX_DEPTH_I2C] = {
	/* No indexing: enabled as a whole */
	{MUX_SEL_I2C, OUTPUT, HIGH},
};
#endif

/* Route pin2 to SoC */
mux_sel_t MuxDesc14[] = {
	{31, LOW, FN_GPIO},
	{0, LOW, FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{0, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
};

/* Route pin3 to SoC */
mux_sel_t MuxDesc15[] = {
	{30, LOW, FN_GPIO},
	{1, LOW, FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{1, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
};

// Describe muxes for pin gpiolib id 16
mux_sel_t MuxDesc16[] = {
	//gpio, value, mode, type
	{42, HIGH, FN_GPIO | FN_PWM},
	{42, LOW, FN_SPI},	// SPI chip-select
};

/* Route pin3 to Cypress */
mux_sel_t MuxDesc18[] = {
	{30, HIGH, FN_GPIO | FN_PWM},
};

mux_sel_t MuxDesc25[] = {
	//gpio, value, type
	{43, HIGH, FN_GPIO | FN_PWM},
	{43, LOW, FN_SPI}
};

/* Route pin2 to Cypress */
mux_sel_t MuxDesc32[] = {
	{31, HIGH, FN_GPIO},
};

mux_sel_t MuxDesc38 [] = {
	//gpio, value, type
	{54, HIGH, FN_GPIO},
	{54, LOW, FN_SPI},
};

mux_sel_t MuxDesc39[] = {
	//gpio, value, type
	{55, HIGH, FN_GPIO | FN_PWM},
	{55, LOW, FN_SPI},
};

mux_sel_t MuxDesc44[] = {
	//gpio, value, type
	{37, HIGH, FN_GPIO},
	{37, LOW, FN_ANALOG},
};

mux_sel_t MuxDesc45[] = {
	//gpio, value, type
	{36, HIGH, FN_GPIO},
	{36, LOW, FN_ANALOG},
};

mux_sel_t MuxDesc46[] = {
	//gpio, value, type
	{23, HIGH, FN_GPIO},
	{23, LOW, FN_ANALOG},
};

mux_sel_t MuxDesc47[] = {
	//gpio, value, type
	{22, HIGH, FN_GPIO},
	{22, LOW, FN_ANALOG},
};

mux_sel_t MuxDesc48[] = {
	//gpio, value, type
	{21, HIGH, FN_GPIO},
	{21, LOW, FN_ANALOG},
	{29, HIGH, FN_ANALOG},
	{29, HIGH, FN_GPIO},
	{29, LOW, FN_I2C},
};

mux_sel_t MuxDesc49[] = {
	//gpio, value, type
	{20, HIGH, FN_GPIO},
	{20, LOW, FN_ANALOG},
	{29, HIGH, FN_ANALOG},
	{29, HIGH, FN_GPIO},
	{29, LOW, FN_I2C},
};

mux_sel_t MuxDesc50[] = {
	//gpio, value, type
	{40, HIGH, FN_GPIO},
	{40, LOW, FN_UART},
};

mux_sel_t MuxDesc51[] = {
	//gpio, value, type
	{41, HIGH, FN_GPIO},
	{41, LOW, FN_UART},
};

PinDescription AltPin2 =
	{ 14,	NONE,	GPIO_FAST_IO2,	2,	NONE,	NONE,	(mux_sel_t*)&MuxDesc14,	MUX_SIZE(MuxDesc14),	FN_GPIO,	-1,	1,	0,	NULL };	// pin2 (when routed to SoC)

PinDescription AltPin3 =
	{ 15,	NONE,	GPIO_FAST_IO3,	3,	NONE,	NONE,	(mux_sel_t*)&MuxDesc15,	MUX_SIZE(MuxDesc15),	FN_GPIO,	-1,	1,	0,	NULL };	// pin3 (when routed to SoC)


// Sorted by Linux GPIO ID
PinDescription g_APinDescription[]=
{
//	gpiolib	alias	fastinf	ardid	Initial			FixdSt	ptMuxDesc,		MuxCount		type		Handle	extPU	iAlt	pAlt		
	{ 0,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1, 	0,	0,	NULL,	 },	// pullup for IO2
	{ 1,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// pullup for IO3
	{ 2,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_RESERVED,	-1,	0,	0,	NULL	 },	// S3 resume input - don't touch
	{ 3,	NONE,	NULL,	20,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// General Purpose LED IO20
	{ 4,	NONE,	NULL,	NONE,	NONE,			HIGH,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Level shifter
	{ 7,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_RESERVED,	-1,	0,	0,	NULL	 },	// Jumper to select i2c address of GPIO expander
	{ 12,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_RESERVED,	-1,	0,	0,	NULL	 },	// Reset line to cypress - owned by Linux driver
	{ 13,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_RESERVED,	-1,	0,	0,	NULL	 },	// S0 interrupt on SOC GPIO<5>
	{ 16,	NONE,	NULL,	10,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc16,	MUX_SIZE(MuxDesc16),	FN_GPIO,	-1,	0,	0,	NULL	 },	// SPI1_SS_B or PWM pin #10/GPIO10
	{ 17,	NONE,	NULL,	5,	NONE,			NONE,	NULL,			0,			FN_GPIO,	-1,	0,	0,	NULL	 },	// Arduino GPIO#5 or PWM pin #3
	//{ 18,	NONE,	NULL,	3,	NONE,			NONE,	NULL,			0,			FN_GPIO,	-1,	0,	0,	NULL	 },	// Arduino GPIO#3 or PWM pin #2
	{ 18,	NONE,	NULL,	3,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc18,	MUX_SIZE(MuxDesc18),	FN_GPIO,	-1,	0,	0,	&AltPin3 },	// pin3 (when routed to Cypress)
	{ 19,	NONE,	NULL,	9,	NONE,			NONE,	NULL,			0,			FN_GPIO,	-1,	0,	0,	NULL	 },	// Arduino GPIO#9 or PWM pin #5
	{ 20,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// GPIO # 20 is responsible to mux GPIO #49 - Arduino ID IO19 - AD7298:VIN5
	{ 21,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// GPIO # 21 is responsible to mux GPIO #48 - Arduino ID IO18 - AD7298:VIN4
	{ 22,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// GPIO # 22 is responsible to mux GPIO #47 - Arduino ID IO17 - AD7298:VIN3
	{ 23,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// GPIO # 23 is responsible to mux GPIO #47 - Arduino ID IO16 - AD7298:VIN2
	{ 24,	NONE,	NULL,	6,	NONE,			NONE,	NULL,			0,			FN_GPIO,	-1,	0,	0,	NULL	 },	// Arduino GPIO#6 or PWM pin #4
	{ 25,	NONE,	NULL,	11,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc25,	MUX_SIZE(MuxDesc25),	FN_GPIO,	-1,	0,	0,	NULL	 },	// gpio - or PWM pin #6
	{ 26,	NONE,	NULL,	8,	NONE,			NONE,	NULL,			0,			FN_GPIO,	-1,	0,	0,	NULL	 },	// Arduino GPIO#8
	{ 27,	NONE,	NULL,	7,	NONE,			NONE,	NULL,			0,			FN_GPIO,	-1,	0,	0,	NULL	 },	// Arduino GPIO#7
	{ 28,	NONE,	NULL,	4,	NONE,			NONE,	NULL,			0,			FN_GPIO,	-1,	0,	0,	NULL	 },	// Arduino GPIO#4
	{ 29,	NONE,	NULL,	NONE,	NONE,			HIGH,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Mux select
	{ 30,	NONE,	NULL,	NONE,	NONE,			HIGH,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Control pin2's routing: SoC or Cypress (default=Cypress)
	{ 31,	NONE,	NULL,	NONE,	NONE,			HIGH,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Control pin3's routing: SoC or Cypress (default=Cypress)
	{ 32,	NONE,	NULL,	2,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc32,	MUX_SIZE(MuxDesc32),	FN_GPIO,	-1,	0,	0,	&AltPin2 },	// pin2 (when routed to Cypress)
	{ 36,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Mux to select VIN0:1 or IO15:1
	{ 37,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Mux to select VIN0:0 or IO14:1
	{ 38,	NONE,	NULL,	12,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc38,	MUX_SIZE(MuxDesc38),	FN_GPIO,	-1,	0,	0,	NULL	 },	// Mux to select IO12 or MOSI
	{ 39,	20,	NULL,	13,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc39,	MUX_SIZE(MuxDesc39),	FN_GPIO,	-1,	0,	0,	NULL	 },	// Mux to select IO13 or SCK
	{ 40,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// GPIO # 40 is responsible to mux GPIO #50 - Arduino ID IO0 - UART0_RXD
	{ 41,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// GPIO # 41 is responsible to mux GPIO #51 - Arduino ID IO1 - UART0_TXD
	{ 42,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// GPIO # 42 is responsible to mux GPIO #16 - Arduino ID IO10 - SPI1_SS_B
	{ 43,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// GPIO # 43 is responsible to mux GPIO #25 - Arduino ID IO11 - SPI1_MOSI
	{ 44,	NONE,	NULL,	14,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc44,	MUX_SIZE(MuxDesc44),	FN_GPIO,	-1,	0,	0,	NULL	 },	// Arduino GPIO#14 or AD7298:VIN0
	{ 45,	NONE,	NULL,	15,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc45,	MUX_SIZE(MuxDesc45),	FN_GPIO,	-1,	0,	0,	NULL	 },	// Arduino GPIO#15 or AD7298:VIN1
	{ 46,	NONE,	NULL,	16,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc46,	MUX_SIZE(MuxDesc46),	FN_GPIO,	-1,	0,	0,	NULL	 },	// Arduino GPIO#16 or AD7298:VIN2
	{ 47,	NONE,	NULL,	17,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc47,	MUX_SIZE(MuxDesc47),	FN_GPIO,	-1,	0,	0,	NULL	 },	// Arduino GPIO#17 or AD7298:VIN3
	{ 48,	NONE,	NULL,	18,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc48,	MUX_SIZE(MuxDesc48),	FN_GPIO,	-1,	0,	0,	NULL	 },	// Arduino GPIO#18 or AD7298:VIN4
	{ 49,	NONE,	NULL,	19,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc49,	MUX_SIZE(MuxDesc49),	FN_GPIO,	-1,	0,	0,	NULL	 },	// Arduino GPIO#19 or AD7298:VIN5
	{ 50,	NONE,	NULL,	0,	FN_UART,		NONE,	(mux_sel_t*)&MuxDesc50,	MUX_SIZE(MuxDesc50),	FN_GPIO,	-1,	0,	0,	NULL	 },	// Arduino GPIO#0 or UART
	{ 51,	NONE,	NULL,	1,	FN_UART,		NONE,	(mux_sel_t*)&MuxDesc51,	MUX_SIZE(MuxDesc51),	FN_GPIO,	-1,	0,	0,	NULL	 },	// Arduino GPIO#1 or UART
	{ 52,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_RESERVED,	-1,	0,	0,	NULL	 },	// Shield reset to s/w
//	{ 52,	NONE,	NULL,	20,	NONE,			NONE,	NULL,			0,			FN_GPIO,	-1,	0,	0,	NULL	 },	// Shield reset to s/w
	{ 53,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_RESERVED,	-1,	0,	0,	NULL	 },	// Reset to h/w
	{ 54,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// GPIO # 54 is responsible to mux GPIO #38 - Arduino ID IO12 - SPI1_MISO
	{ 55,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// GPIO # 55 is responsible to mux GPIO #39 - Arduino ID IO13 - SPI1_SCK
};

uint32_t sizeof_g_APinDescription;

uint32_t ardPin2DescIdx[GPIO_TOTAL];

// Sorted by Linux PWM ID
PwmDescription g_APwmDescription[] = {
	/* 0 is unused  */
	{ 1,	9,	-1,	-1 },
	/* 2 is unused  */
	{ 3,	3,	-1,	-1 },
	{ 4,	11,	-1,	-1 },
	{ 5,	5,	-1,	-1 },
	{ 6,	6,	-1,	-1 },
	{ 7,	10,	-1,	-1 },
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
RingBuffer rx_buffer3;

TTYUARTClass Serial(&rx_buffer1, 0);		// ttyGS0
TTYUARTClass Serial1(&rx_buffer2, 1);		// ttyS0
TTYUARTClass Serial2(&rx_buffer3, 2, true);	// ttyS1 (system console)


// ----------------------------------------------------------------------------

int variantPinMode(uint8_t pin, uint8_t mode)
{
	/*
	 * Pin2 and pin3 can be individually assigned to a SoC or a Cypress
	 * GPIO.
	 *
	 * The pin at this time is always set as Cypress-handled.
	 */

	int ret = 0;
	PinDescription *p = NULL;

	if (pin >= GPIO_TOTAL){
		trace_error("%s: invalid pin%u", __func__, pin);
		return PIN_EINVAL;
	}
	
	/*Nothing to do if not pin0, pin1, pin2, or pin3 */
	if(pin > 3)
	{
		return 0;
	}
	/* Search for entry */
	p = &g_APinDescription[ardPin2DescIdx[pin]];


	/* Cypress is first entry: disable alternate */
	p->iAlternate = 0;
	trace_debug("%s: disable SoC GPIO for pin%u",
		    __func__, pin);

	return 0;
}

int variantPinModeIRQ(uint8_t pin, uint8_t mode)
{
	/*
	 * Pin2 and pin3 can be individually assigned to a SoC or a Cypress
	 * GPIO.
	 *
	 * Assignment depends on the triggering mode:
	 * - CHANGE: Cypress
	 * - remaining: SoC
	 */

	PinDescription *p = NULL;
	int ret = 0;

	if (pin >= GPIO_TOTAL){
		trace_error("%s: invalid pin%u", __func__, pin);
		return PIN_EINVAL;
	}

	/* Nothing to do if it's not pin2 nor pin3 */
	if (2 != pin && 3 != pin) {
		return 0;
	}

	/* Search for entry */
	p = &g_APinDescription[ardPin2DescIdx[pin]];

	if (CHANGE != mode) {
		/* SoC is second entry: enable alternate */
		p->iAlternate = 1;
		trace_debug("%s: enable SoC GPIO for pin%u",
			    __func__, pin);
	} else {
		/* Cypress is first entry: disable alternate */
		p->iAlternate = 0;
		trace_debug("%s: enable Cypress GPIO for pin%u", __func__,
			    pin);
	}

	return 0;
}

void turnOnPWM(uint8_t pin)
{
	uint32_t ret = 0;
	int gpio = 0;

	if (pin >= GPIO_TOTAL){
		trace_error("%s: invalid pin%u", __func__, pin);
		return;
	}

	/* Mark PWM enabled on pin */
	g_APinState[pin].uCurrentPwm = 1;
	g_APinState[pin].uCurrentAdc = 0;

	/* Set strong drive and output direction */
	gpio = g_APinDescription[ardPin2DescIdx[pin]].ulGPIOId;
	ret = sysfsGpioSetDrive(gpio, GPIO_DRIVE_STRONG);
	if (ret) {
		trace_error("%s: can't set strong drive for pin%d", __func__,
			    pin);
		return;
	}
	ret = sysfsGpioDirection(gpio, 1, 1);
	if (ret) {
		trace_error("%s: can't set directon for pin%d", __func__, pin);
		return;
	}

	return;
}

void turnOffPWM(uint8_t pin)
{
	int handle = 0;
	PinDescription *p = NULL;

	if (pin >= GPIO_TOTAL){
		trace_error("%s: invalid pin%u", __func__, pin);
		return;
	}

	// Scan mappings
	p = &g_APinDescription[ardPin2DescIdx[pin]];
	pin2alternate(&p);

	if(p->ulArduinoId == pin) {
		handle = pin2pwmhandle_enable(pin);
		if ((int)PIN_EINVAL == handle) {
			trace_error("%s: bad handle for pin%u",
				    __func__, pin);
			return;
		}
		if (sysfsPwmDisableGen1(handle)) {
			trace_error("%s: couldn't disable pwm "
				    "on pin%u", __func__, pin);
			return;
		}

		/* Mark PWM disabled on pin */
		g_APinState[pin].uCurrentPwm = 0;

		return;
	}

	trace_error("%s: unknown pin%u", __func__, pin);
}

void variantEnableFastGpio(int pin)
{
	int entryno = ardPin2DescIdx[pin];
	PinDescription *p = NULL;
	int ret = 0;

	if (entryno >= sizeof_g_APinDescription) {
		trace_error("%s: ardPin2DescIdx[%d] == %d >= "
			    "sizeof_g_APinDescription", __func__, pin, entryno);
		return;
	}

	/* Enable alternate to route to SoC */
	p = &g_APinDescription[entryno];
	p->iAlternate = 1;
}

void __libc_init_array(void);

void init( int argc, char * argv[] )
{
	if(argc > 1)
		if(Serial.init_tty(argv[1]) != 0)
			return;

	if(Serial1.init_tty(LINUX_SERIAL1_TTY) != 0)
		return;
	if(Serial2.init_tty(LINUX_SERIAL2_TTY) != 0)
		return;

	sizeof_g_APinDescription = sizeof(g_APinDescription)/sizeof(struct _PinDescription);
	pinInit();
	sizeof_g_APwmDescription = sizeof(g_APwmDescription)/sizeof(struct _PwmDescription);
	pwmInit();
	sizeof_g_AdcDescription = sizeof(g_AdcDescription)/sizeof(struct _AdcDescription);
	adcInit();
	sizeof_g_APinState = sizeof(g_APinState)/sizeof(struct _PinState);
}


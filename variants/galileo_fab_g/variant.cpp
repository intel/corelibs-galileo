/*
  Copyright (c) 2011 Arduino.  All right reserved.
  Copyright (c) 2014 Intel Corporation.

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

//////////////////////////// Galileo Gen2 ////////////////////////////

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
	{MUX_SEL_UART0_RXD,	MUX_SEL_UART0_TXD},		// ttyS0 - muxed
	{MUX_SEL_UART1_RXD,	MUX_SEL_UART1_TXD},		// ttyS1 - muxed
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

mux_sel_t MuxDesc0[] = {
	//gpio, value, type
	{32, LOW, FN_GPIO_OUTPUT}, // Output enable
	{32, HIGH, FN_UART | FN_GPIO_INPUT}, // Output disable
	{33, NONE, FN_UART | FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{33, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
};

mux_sel_t MuxDesc1[] = {
	//gpio, value, type
	{45, LOW, FN_GPIO},
	{45, HIGH, FN_UART},
	{28, LOW, FN_UART | FN_GPIO_OUTPUT}, // Output enable
	{28, HIGH, FN_GPIO_INPUT}, // Output disable
	{29, NONE, FN_UART | FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{29, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{29, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
};

mux_sel_t MuxDesc2[] = {
	//gpio, value, type
	{77, LOW, FN_GPIO},
	{77, HIGH, FN_UART},
	{34, LOW, FN_GPIO_OUTPUT}, // Output disable
	{34, HIGH, FN_UART | FN_GPIO_INPUT}, // Output enable
	{35, NONE, FN_UART | FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{35, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{35, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
	{13, NONE, FN_GPIO}, // Disable GPIO #13 output, connected on same pin
	{61, NONE, FN_UART}, // Disable GPIO #61 output, connected on same pin
};

mux_sel_t MuxDesc2A[] = {
	//gpio, value, type
	{77, LOW, FN_GPIO},
	{77, HIGH, FN_UART},
	{34, LOW, FN_GPIO_OUTPUT}, // Output disable
	{34, HIGH, FN_UART | FN_GPIO_INPUT}, // Output enable
	{35, NONE, FN_UART | FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{35, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{35, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
	{61, NONE, FN_GPIO | FN_UART}, // Disable GPIO #61 output, connected on same pin
};

mux_sel_t MuxDesc3[] = {
	//gpio, value, type
	{64, LOW, FN_GPIO},
	{64, HIGH, FN_PWM},
	{76, LOW, FN_GPIO | FN_PWM},
	{76, HIGH, FN_UART},
	{16, LOW, FN_UART | FN_PWM | FN_GPIO_OUTPUT}, // Output enable
	{16, HIGH, FN_GPIO_INPUT}, // Output disable
	{17, NONE, FN_UART | FN_PWM | FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{17, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{17, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
	{14, NONE, FN_GPIO}, // Disable GPIO #14 output, connected on same pin
	{62, NONE, FN_PWM | FN_UART}, // Disable GPIO #62 output, connected on same pin
};

mux_sel_t MuxDesc3A[] = {
	//gpio, value, type
	{64, LOW, FN_GPIO},
	{64, HIGH, FN_PWM},
	{76, LOW, FN_GPIO | FN_PWM},
	{76, HIGH, FN_UART},
	{16, LOW, FN_UART | FN_PWM | FN_GPIO_OUTPUT}, // Output enable
	{16, HIGH, FN_GPIO_INPUT}, // Output disable
	{17, NONE, FN_UART | FN_PWM | FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{17, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{17, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
	{62, NONE, FN_GPIO | FN_PWM | FN_UART}, // Disable GPIO #62 output, connected on same pin
};

mux_sel_t MuxDesc4[] = {
	//gpio, value, type
	{36, LOW, FN_GPIO_OUTPUT}, // Output enable
	{36, HIGH, FN_GPIO_INPUT}, // Output disable
	{37, NONE, FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{37, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{37, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
};

mux_sel_t MuxDesc5[] = {
	//gpio, value, type
	{66, LOW, FN_GPIO},
	{66, HIGH, FN_PWM},
	{18, LOW, FN_PWM | FN_GPIO_OUTPUT}, // Output-enable
	{18, HIGH, FN_GPIO_INPUT}, // Output-disable
	{19, NONE, FN_PWM | FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{19, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{19, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
};

mux_sel_t MuxDesc6[] = {
	//gpio, value, type
	{68, LOW, FN_GPIO},
	{68, HIGH, FN_PWM},
	{20, LOW, FN_PWM | FN_GPIO_OUTPUT}, // Output-enable
	{20, HIGH, FN_GPIO_INPUT}, // Output-disable
	{21, NONE, FN_PWM | FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{21, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{21, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
};

mux_sel_t MuxDesc7[] = {
	//gpio, value, type
	{39, NONE, FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{39, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{39, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
};

mux_sel_t MuxDesc8[] = {
	//gpio, value, type
	{41, NONE, FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{41, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{41, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
};

mux_sel_t MuxDesc9[] = {
	//gpio, value, type
	{70, LOW, FN_GPIO},
	{70, HIGH, FN_PWM},
	{22, LOW, FN_PWM | FN_GPIO_OUTPUT}, // Output-enable
	{22, HIGH, FN_GPIO_INPUT}, // Output-disable
	{23, NONE, FN_PWM | FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{23, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{23, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
};

mux_sel_t MuxDesc10[] = {
	//gpio, value, type
	{74, LOW, FN_GPIO},
	{74, HIGH, FN_PWM},
	{26, LOW, FN_PWM | FN_GPIO_OUTPUT}, // Output enable
	{26, HIGH, FN_GPIO_INPUT}, // Output disable
	{27, NONE, FN_PWM | FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{27, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{27, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
};

mux_sel_t MuxDesc11[] = {
	//gpio, value, type
	{44, LOW, FN_GPIO},
	{44, HIGH, FN_SPI},
	{72, LOW, FN_GPIO},
	{72, LOW, FN_SPI},
	{72, HIGH, FN_PWM},
	{24, LOW, FN_PWM | FN_SPI | FN_GPIO_OUTPUT}, // Output-enable
	{24, HIGH, FN_GPIO_INPUT}, // Output-enable
	{25, NONE, FN_PWM | FN_SPI | FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{25, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{25, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
};

mux_sel_t MuxDesc12[] = {
	//gpio, value, type
	{42, LOW, FN_GPIO_OUTPUT}, // Output enable
	{42, HIGH, FN_SPI | FN_GPIO_INPUT}, // Output disable
	{43, NONE, FN_SPI | FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{43, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{43, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
};

mux_sel_t MuxDesc13[] = {
	//gpio, value, type
	{46, LOW, FN_GPIO},
	{46, HIGH, FN_SPI},
	{30, LOW, FN_SPI | FN_GPIO_OUTPUT}, // Output enable
	{30, HIGH, FN_GPIO_INPUT}, // Output disable
	{31, NONE, FN_SPI | FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{31, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{31, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
};

mux_sel_t MuxDesc14[] = {
	//gpio, value, type
	{48, NONE, FN_ANALOG}, // Prevent leakage current for ADC
	{49, NONE, FN_ANALOG | FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{49, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{49, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
};

mux_sel_t MuxDesc15[] = {
	//gpio, value, type
	{50, NONE, FN_ANALOG}, // Prevent leakage current for ADC
	{51, NONE, FN_ANALOG | FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{51, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{51, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
};

mux_sel_t MuxDesc16[] = {
	//gpio, value, type
	{52, NONE, FN_ANALOG}, // Prevent leakage current for ADC
	{53, NONE, FN_ANALOG | FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{53, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{53, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
};

mux_sel_t MuxDesc17[] = {
	//gpio, value, type
	{54, NONE, FN_ANALOG}, // Prevent leakage current for ADC
	{55, NONE, FN_ANALOG | FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{55, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{55, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
};

mux_sel_t MuxDesc18[] = {
	//gpio, value, type
	{78, LOW, FN_ANALOG},
	{78, HIGH, FN_GPIO},
	{60, LOW, FN_I2C},
	{60, HIGH, FN_ANALOG},
	{60, HIGH, FN_GPIO},
	{56, NONE, FN_ANALOG | FN_I2C}, // Prevent leakage current for ADC
	{57, NONE, FN_ANALOG | FN_I2C | FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{57, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{57, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
};

mux_sel_t MuxDesc19[] = {
	//gpio, value, type
	{79, LOW, FN_ANALOG},
	{79, HIGH, FN_GPIO},
	{60, LOW, FN_I2C},
	{60, HIGH, FN_ANALOG},
	{60, HIGH, FN_GPIO},
	{58, NONE, FN_ANALOG | FN_I2C}, // Prevent leakage current for ADC
	{59, NONE, FN_ANALOG | FN_I2C | FN_GPIO_INPUT_HIZ | FN_GPIO_OUTPUT}, // Pullup disable
	{59, HIGH, FN_GPIO_INPUT_PULLUP}, // Pullup enable
	{59, LOW, FN_GPIO_INPUT_PULLDOWN}, // Pulldown enable
};

/*
 * The following 6 pins are for Fast mode MMIO access. The pins are the same 
 * as slow mode, they just use MMIO register access for fast mode and sysfs for slow mode. 
 */
//	gpiolib	alias	fastinf			ardid	Initial		FixdSt	ptMuxDesc,		MuxCount		type		Handle	extPU	iAlt	pAlt
PinDescription AltPin0 =
	{ 11,	NONE,	GPIO_FAST_IO0,	0,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc0,	MUX_SIZE(MuxDesc0),	FN_GPIO,	-1,	1,	0,	NULL	 };	// Arduino IO0 or UART0_RXD
PinDescription AltPin1 =
	{ 12,	NONE,	GPIO_FAST_IO1,	1,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc1,	MUX_SIZE(MuxDesc1),	FN_GPIO,	-1,	1,	0,	NULL	 };	// Arduino IO1 or UART0_TXD
PinDescription AltPin2 =
	{ 13,	NONE,	GPIO_FAST_IO2,	2,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc2A,	MUX_SIZE(MuxDesc2A),	FN_GPIO,	-1,	1,	0,	NULL	 };	// Arduino IO2 or UART1_RXD
PinDescription AltPin3 =
	{ 14,	NONE,	GPIO_FAST_IO3,	3,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc3A,	MUX_SIZE(MuxDesc3A),	FN_GPIO,	-1,	1,	0,	NULL	 };	// Arduino IO3 or UART1_TXD or PWM pin #1
PinDescription AltPin10 =
	{ 10,	NONE,	GPIO_FAST_IO10,	10,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc10,	MUX_SIZE(MuxDesc10),	FN_GPIO,	-1,	1,	0,	NULL	 };	// Arduino IO10 or SPI1_SS_B or PWM pin #5
PinDescription AltPin12 =
	{ 15,	NONE,	GPIO_FAST_IO12,	12,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc12,	MUX_SIZE(MuxDesc12),	FN_GPIO,	-1,	1,	0,	NULL	 };	// Arduino IO12 or SPI1_MISO

/*
 * The following 8 pins are for Fast mode PIO access. The pins are the same 
 * as slow mode, they just use PIO register access for fast mode and sysfs for slow mode. 
 */
//	gpiolib	alias	fastinf			ardid	Initial		FixdSt	ptMuxDesc,		MuxCount		type		Handle	extPU	iAlt	pAlt
PinDescription AltPin5 =
	{ 0,	NONE,	GPIO_FAST_IO5,	5,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc5,	MUX_SIZE(MuxDesc5),	FN_GPIO,	-1,	1,	0,	NULL	 };	// Arduino IO5 or PWM pin #2
PinDescription AltPin6 =
	{ 1,	NONE,	GPIO_FAST_IO6,	6,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc6,	MUX_SIZE(MuxDesc6),	FN_GPIO,	-1,	1,	0,	NULL	 };	// Arduino IO6 or PWM pin #3
PinDescription AltPin9 =
	{ 4,	NONE,	GPIO_FAST_IO9,	9,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc9,	MUX_SIZE(MuxDesc9),	FN_GPIO,	-1,	1,	0,	NULL	 };	// Arduino IO9 or PWM pin #4
PinDescription AltPin11 =
	{ 5,	NONE,	GPIO_FAST_IO11,	11,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc11,	MUX_SIZE(MuxDesc11),	FN_GPIO,	-1,	1,	0,	NULL	 };	// Arduino IO11 or SPI1_MOSI or PWM pin #6
PinDescription AltPin4 =
	{ 6,	NONE,	GPIO_FAST_IO4,	4,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc4,	MUX_SIZE(MuxDesc4),	FN_GPIO,	-1,	1,	0,	NULL	 };	// Arduino IO4
PinDescription AltPin13 =
	{ 7,	NONE,	GPIO_FAST_IO13,	13,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc13,	MUX_SIZE(MuxDesc13),	FN_GPIO,	-1,	1,	0,	NULL	 };	// Arduino IO13 or SPI1_SCK


// Sorted by Linux GPIO ID
PinDescription g_APinDescription[]=
{
//	gpiolib	alias	fastinf	ardid	Initial			FixdSt	ptMuxDesc,		MuxCount		type		Handle	extPU	iAlt	pAlt
	{ 0,	NONE,	NULL,	5,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc5,	MUX_SIZE(MuxDesc5),	FN_GPIO,	-1,	1,	0,	&AltPin5 },	// Arduino IO5 or PWM pin #2
	{ 1,	NONE,	NULL,	6,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc6,	MUX_SIZE(MuxDesc6),	FN_GPIO,	-1,	1,	0,	&AltPin6 },	// Arduino IO6 or PWM pin #3
	{ 2,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_RESERVED,	-1,	1,	0,	NULL	 },	// PCIe Reset to h/w
	{ 3,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_RESERVED,	-1,	1,	0,	NULL	 },	// Wifi Disable reset to h/w
	{ 4,	NONE,	NULL,	9,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc9,	MUX_SIZE(MuxDesc9),	FN_GPIO,	-1,	1,	0,	&AltPin9 },	// Arduino IO9 or PWM pin #4
	{ 5,	NONE,	NULL,	11,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc11,	MUX_SIZE(MuxDesc11),	FN_GPIO,	-1,	1,	0,	&AltPin11},	// Arduino IO11 or SPI1_MOSI or PWM pin #6
	{ 6,	NONE,	NULL,	4,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc4,	MUX_SIZE(MuxDesc4),	FN_GPIO,	-1,	1,	0,	&AltPin4 },	// Arduino IO4
	{ 7,	NONE,	NULL,	13,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc13,	MUX_SIZE(MuxDesc13),	FN_GPIO,	-1,	1,	0,	&AltPin13},	// Arduino IO13 or SPI1_SCK
	{ 8,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_RESERVED,	-1,	0,	0,	NULL	 },	// owned by Linux driver
	{ 9,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_RESERVED,	-1,	0,	0,	NULL	 },	// owned by Linux driver
	{ 10,	NONE,	NULL,	10,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc10,	MUX_SIZE(MuxDesc10),	FN_GPIO,	-1,	1,	0,	&AltPin10},	// Arduino IO10 or SPI1_SS_B or PWM pin #5
	{ 11,	NONE,	NULL,	0,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc0,	MUX_SIZE(MuxDesc0),	FN_GPIO,	-1,	1,	0,	&AltPin0 },	// Arduino IO0 or UART0_RXD
	{ 12,	NONE,	NULL,	1,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc1,	MUX_SIZE(MuxDesc1),	FN_GPIO,	-1,	1,	0,	&AltPin1 },	// Arduino IO1 or UART0_TXD
//	{ 13,	NONE,	NULL,	2,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc2,	MUX_SIZE(MuxDesc2),	FN_GPIO,	-1,	1,	0,	&AltPin2 },	// Arduino IO2 or UART1_RXD (Using GPIO #61 to allow CHANGE-mode interrupts)
//	{ 14,	NONE,	NULL,	3,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc3,	MUX_SIZE(MuxDesc3),	FN_GPIO,	-1,	1,	0,	&AltPin3 },	// Arduino IO3 or UART1_TXD or PWM pin #1 (Using GPIO #62 to allow CHANGE-mode interrupts)
	{ 15,	NONE,	NULL,	12,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc12,	MUX_SIZE(MuxDesc12),	FN_GPIO,	-1,	1,	0,	&AltPin12},	// Arduino IO12 or SPI1_MISO
	{ 16,	NONE,	NULL,	NONE,	NONE,			HIGH,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Direction Control for GPIO #14 - Arduino ID IO3
	{ 17,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #14 - Arduino ID IO3
	{ 18,	NONE,	NULL,	NONE,	NONE,			HIGH,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Direction Control for GPIO #0 - Arduino ID IO5
	{ 19,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #0 - Arduino ID IO5
	{ 20,	NONE,	NULL,	NONE,	NONE,			HIGH,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Direction Control for GPIO #1 - Arduino ID IO6
	{ 21,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #1 - Arduino ID IO6
	{ 22,	NONE,	NULL,	NONE,	NONE,			HIGH,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Direction Control for GPIO #4 - Arduino ID IO9
	{ 23,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #4 - Arduino ID IO9
	{ 24,	NONE,	NULL,	NONE,	NONE,			HIGH,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Direction Control for GPIO #5 - Arduino ID IO11
	{ 25,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #5 - Arduino ID IO11
	{ 26,	NONE,	NULL,	NONE,	NONE,			HIGH,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Direction Control for GPIO #10 - Arduino ID IO10
	{ 27,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #10 - Arduino ID IO10
	{ 28,	NONE,	NULL,	NONE,	NONE,			HIGH,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Direction Control for GPIO #12 - Arduino ID IO1
	{ 29,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #12 - Arduino ID IO1
	{ 30,	NONE,	NULL,	NONE,	NONE,			HIGH,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Direction Control for GPIO #7 - Arduino ID IO13
	{ 31,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #7 - Arduino ID IO13
	{ 32,	NONE,	NULL,	NONE,	NONE,			HIGH,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Direction Control for GPIO #11 - Arduino ID IO0
	{ 33,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #11 - Arduino ID IO0
	{ 34,	NONE,	NULL,	NONE,	NONE,			HIGH,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Direction Control for GPIO #13 - Arduino ID IO2
	{ 35,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #13 - Arduino ID IO2
	{ 36,	NONE,	NULL,	NONE,	NONE,			HIGH,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Direction Control for GPIO #6 - Arduino ID IO4
	{ 37,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #6 - Arduino ID IO4
	{ 38,	NONE,	NULL,	7,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc7,	MUX_SIZE(MuxDesc7),	FN_GPIO,	-1,	0,	0,	NULL	 },	// Arduino IO7
	{ 39,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #2 - Arduino ID IO7
	{ 40,	NONE,	NULL,	8,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc8,	MUX_SIZE(MuxDesc8),	FN_GPIO,	-1,	0,	0,	NULL	 },	// Arduino IO8
	{ 41,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #3 - Arduino ID IO8
	{ 42,	NONE,	NULL,	NONE,	NONE,			HIGH,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Direction Control for GPIO #15 - Arduino ID IO12
	{ 43,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #15 - Arduino ID IO12
	{ 44,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Mux control for GPIO #14 - Arduino ID IO11 - SPI1_MOSI
	{ 45,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Mux control for GPIO #12 - Arduino ID IO1 - UART0_TXD
	{ 46,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Mux control for GPIO #7 - Arduino ID IO13 - SPI1_SCK
	{ 47,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_RESERVED,	-1,	0,	0,	NULL	 },	// Shield reset to s/w
	{ 48,	NONE,	NULL,	14,	FN_GPIO_INPUT_HIZ,	LOW,	(mux_sel_t*)&MuxDesc14,	MUX_SIZE(MuxDesc14),	FN_GPIO,	-1,	1,	0,	NULL	 },	// Arduino IO14 or AD7298:VIN0
	{ 49,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #48 - Arduino ID IO14
	{ 50,	NONE,	NULL,	15,	FN_GPIO_INPUT_HIZ,	LOW,	(mux_sel_t*)&MuxDesc15,	MUX_SIZE(MuxDesc15),	FN_GPIO,	-1,	1,	0,	NULL	 },	// Arduino IO15 or AD7298:VIN1
	{ 51,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #50 - Arduino ID IO15
	{ 52,	NONE,	NULL,	16,	FN_GPIO_INPUT_HIZ,	LOW,	(mux_sel_t*)&MuxDesc16,	MUX_SIZE(MuxDesc16),	FN_GPIO,	-1,	1,	0,	NULL	 },	// Arduino IO16 or AD7298:VIN2
	{ 53,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #52 - Arduino ID IO16
	{ 54,	NONE,	NULL,	17,	FN_GPIO_INPUT_HIZ,	LOW,	(mux_sel_t*)&MuxDesc17,	MUX_SIZE(MuxDesc17),	FN_GPIO,	-1,	1,	0,	NULL	 },	// Arduino IO17 or AD7298:VIN3
	{ 55,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #54 - Arduino ID IO17
	{ 56,	NONE,	NULL,	18,	FN_GPIO_INPUT_HIZ,	LOW,	(mux_sel_t*)&MuxDesc18,	MUX_SIZE(MuxDesc18),	FN_GPIO,	-1,	1,	0,	NULL	 },	// Arduino IO18 or AD7298:VIN4
	{ 57,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #56 - Arduino ID IO18
	{ 58,	NONE,	NULL,	19,	FN_GPIO_INPUT_HIZ,	LOW,	(mux_sel_t*)&MuxDesc19,	MUX_SIZE(MuxDesc19),	FN_GPIO,	-1,	1,	0,	NULL	 },	// Arduino IO19 or AD7298:VIN5
	{ 59,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Pullup Control for GPIO #58 - Arduino ID IO19
	{ 60,	NONE,	NULL,	NONE,	NONE,			HIGH,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Mux control for GPIO #58 - I2C - Arduino ID IO19/AD7298:VIN5 | Arduino ID IO18/AD7298:VIN4
	{ 61,	NONE,	NULL,	2,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc2,	MUX_SIZE(MuxDesc2),	FN_GPIO,	-1,	1,	0,	&AltPin2 },	// Arduino ID IO2 - Alternative pin for CHANGE interrupts
	{ 62,	NONE,	NULL,	3,	FN_GPIO_INPUT_HIZ,	NONE,	(mux_sel_t*)&MuxDesc3,	MUX_SIZE(MuxDesc3),	FN_GPIO,	-1,	1,	0,	&AltPin3 },	// Arduino ID IO3 - Alternative pin for CHANGE interrupts
	{ 63,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_RESERVED,	-1,	0,	0,	NULL	 },	// Shield Reset to h/w
																		                
	{ 64,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Mux control for GPIO #14 - Arduino ID IO3 - PWM #1
	{ 65,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_RESERVED,	-1,	0,	0,	NULL	 },	// PWM
	{ 66,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Mux control for GPIO #0 - Arduino ID IO5 - PWM #2
	{ 67,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_RESERVED,	-1,	0,	0,	NULL	 },	// PWM
	{ 68,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Mux control for GPIO #1 - Arduino ID IO6 - PWM #3
	{ 69,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_RESERVED,	-1,	0,	0,	NULL	 },	// PWM
	{ 70,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Mux control for GPIO #4 - Arduino ID IO9 - PWM #4
	{ 71,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_RESERVED,	-1,	0,	0,	NULL	 },	// PWM
	{ 72,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Mux control for GPIO #14 - PWM #6 - Arduino ID IO11/SPI1_MOSI
	{ 73,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_RESERVED,	-1,	0,	0,	NULL	 },	// PWM
	{ 74,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Mux control for GPIO #10 - Arduino ID IO10 - PWM #5
	{ 75,	NONE,	NULL,	NONE,	NONE,			NONE,	NULL,			0,			FN_RESERVED,	-1,	0,	0,	NULL	 },	// PWM
	{ 76,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Mux control for GPIO #14 - UART1_TXD - Arduino ID IO3/PWM #1
	{ 77,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Mux control for GPIO #13 - Arduino ID IO2 - UART1_RXD
	{ 78,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Mux control for GPIO #56 - Arduino ID IO18 - AD7298:VIN4
	{ 79,	NONE,	NULL,	NONE,	NONE,			LOW,	NULL,			0,			FN_SWITCH,	-1,	0,	0,	NULL	 },	// Mux control for GPIO #58 - Arduino ID IO19 - AD7298:VIN5
};


uint32_t sizeof_g_APinDescription;

uint32_t ardPin2DescIdx[GPIO_TOTAL];

// Sorted by Linux PWM ID
PwmDescription g_APwmDescription[] = {
	{ 1,	3,	-1,	-1 },
	{ 3,	5,	-1,	-1 },
	{ 5,	6,	-1,	-1 },
	{ 7,	9,	-1,	-1 },
	{ 9,	11,	-1,	-1 },
	{ 11,	10,	-1,	-1 },
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
	{ 0,		1,		0},	/* 0		*/
	{ 0,		1,		0},	/* 1		*/
	{ 0,		1,		0},	/* 2		*/
	{ 0,		1,		0},	/* 3  - PWM	*/
	{ 0,		1,		0},	/* 4 		*/
	{ 0,		1,		0},	/* 5  - PWM 	*/
	{ 0,		1,		0},	/* 6  - PWM	*/
	{ 0,		1,		0},	/* 7 		*/
	{ 0,		1,		0},	/* 8 		*/
	{ 0,		1,		0},	/* 9  - PWM	*/
	{ 0,		1,		0},	/* 10 - PWM	*/
	{ 0,		1,		0},	/* 11 - PMW	*/
	{ 0,		1,		0},	/* 12		*/
	{ 0,		1,		0},	/* 13		*/
	{ 0,		1,		0},	/* 14 - ADC	*/
	{ 0,		1,		0},	/* 15 - ADC	*/
	{ 0,		1,		0},	/* 16 - ADC	*/
	{ 0,		1,		0},	/* 17 - ADC	*/
	{ 0,		1,		0},	/* 18 - ADC	*/
	{ 0,		1,		0},	/* 19 - ADC	*/
};
uint32_t sizeof_g_APinState;

#ifdef __cplusplus
}
#endif


RingBuffer rx_buffer1;
RingBuffer rx_buffer2;
RingBuffer rx_buffer3;

TTYUARTClass Serial(&rx_buffer1, 0, false);	// ttyGS0  (USB serial)
TTYUARTClass Serial1(&rx_buffer2, 1, false);	// ttyS0 (IO0/1)
TTYUARTClass Serial2(&rx_buffer3, 2, true);	// ttyS1 (IO2/3, system console)


// ----------------------------------------------------------------------------

int variantPinMode(uint8_t pin, uint8_t mode)
{
	/*
	 * Standard (sysfs) or fast-mode UIO options are available for some pins
	 *
	 * The pin at this time is set to Fast-mode by default, if available
	 */

	int ret = 0;
	PinDescription *p = NULL;

	if (pin >= GPIO_TOTAL){
		trace_error("%s: invalid pin%u", __func__, pin);
		return PIN_EINVAL;
	}

	/* Search for entry */
	p = &g_APinDescription[ardPin2DescIdx[pin]];


	/* Alternate entries for Fast-Mode GPIO: enable by default if available */
	if (p->pAlternate) {
		p->iAlternate = 1;
		trace_debug("%s: enable Fast-Mode SoC GPIO for pin%u",
			    __func__, pin);
	}

	return 0;
}

int variantPinModeIRQ(uint8_t pin, uint8_t mode)
{
	/*
	 * Pin2 and pin3 can be individually assigned to a SoC or a PCAL9555A
	 * GPIO.
	 *
	 * Assignment depends on the triggering mode:
	 * - CHANGE: PCAL9555A
	 * - remaining: SoC
	 * (NOTE - Level-triggered interrupts currently unsupported via sysfs)
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
		trace_debug("%s: enable PCAL9555A GPIO for pin%u", __func__,
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

	return;
}

void turnOffPWM(uint8_t pin)
{
	int handle = 0, ret = 0;
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
		if (sysfsPwmDisable(handle)) {
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

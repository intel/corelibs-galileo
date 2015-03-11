/*
fast_gpio_sc.h Implement a fast GPIO path for the South-Cluster 
               GPIO pins on the Quark X1000 SoC.

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
 * An explicit contract exists between this code and the in-kernel driver, since we both 'own'
 * the registers in question - user-space undertakes - never - ever to run concurrent data
 * whilst using the fast GPIO driver - in other words - user-space guarantees to never drive
 * traffic that can conflict with the kernel code.
 *
 * Author : Bryan O'Donoghue <bryan.odonoghue@intel.com> 2013
 */

#ifndef __FAST_GPIO_SC_H__
#define __FAST_GPIO_SC_H__

#include "fast_gpio_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// Register offsets for Quark X1000 South-Cluster (GIP) GPIO registers
#define QUARK_SC_GPIO_REG_OUT			0x00
#define QUARK_SC_GPIO_REG_IN			0x50

// Wrapper macro to construct Quark X1000 SC GPIO register descriptor
#define GPIO_FAST_ID_QUARK_SC(mask)		\
	GPIO_FAST_ID(GPIO_FAST_TYPE_QUARK_SC,	\
		     QUARK_SC_GPIO_REG_IN,	\
		     QUARK_SC_GPIO_REG_OUT,	\
		     (mask))

int fastGpioSCInit(void);
void fastGpioSCFini(void);
void fastGpioSCDigitalWrite(uint8_t reg_offset, uint8_t gpio, uint8_t val);
uint8_t fastGpioSCDigitalRead(uint8_t reg_offset, uint8_t gpio);
void fastGpioSCDigitalWriteDestructive(register uint8_t reg_offset, uint8_t gpio);
uint32_t fastGpioSCDigitalLatch(uint8_t reg_offset);

#ifdef __cplusplus
}
#endif

#endif /* __FAST_GPIO_SC_H__ */

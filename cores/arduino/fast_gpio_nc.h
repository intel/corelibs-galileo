/*
fast_gpio_nc.h Implement a fast GPIO path for the North-Cluster 
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
 * Author : David Hunt <dave@emutex.com> 2014
 */

#ifndef __FAST_GPIO_NC_H__
#define __FAST_GPIO_NC_H__

#include "fast_gpio_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// Port offsets for Quark X1000 North-Cluster (Legacy) GPIO registers
#define QUARK_NC_CW_GPIO_REG			0x08
#define QUARK_NC_RW_GPIO_REG			0x28

// Wrapper macro to construct Quark X1000 NC core-well GPIO register descriptor
#define GPIO_FAST_ID_QUARK_NC_CW(mask)		\
	GPIO_FAST_ID(GPIO_FAST_TYPE_QUARK_NC,	\
		     QUARK_NC_CW_GPIO_REG,	\
		     QUARK_NC_CW_GPIO_REG,	\
		     (mask))

// Wrapper macro to construct Quark X1000 NC resume-well GPIO register descriptor
#define GPIO_FAST_ID_QUARK_NC_RW(mask)		\
	GPIO_FAST_ID(GPIO_FAST_TYPE_QUARK_NC,	\
		     QUARK_NC_RW_GPIO_REG,	\
		     QUARK_NC_RW_GPIO_REG,	\
		     (mask))

int fastGpioNCInit(void);
void fastGpioNCFini(void);
void fastGpioNCDigitalWrite(uint8_t reg_offset, uint8_t gpio_mask, uint8_t val);
uint8_t fastGpioNCDigitalRead(uint8_t reg_offset, uint8_t gpio_mask);
uint8_t fastGpioNCDigitalLatch(uint8_t reg_offset);
void fastGpioNCDigitalWriteDestructive(uint8_t reg_offset, uint8_t gpio_levels);

#ifdef __cplusplus
}
#endif

#endif /* __FAST_GPIO_NC_H__ */

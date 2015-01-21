/*
fast_gpio_common.h utilitary functions for fast IO
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
#ifndef __FAST_GPIO_COMMON_H__
#define __FAST_GPIO_COMMON_H__

// Macros to (de-)construct GPIO_FAST_* register descriptors
#define GPIO_FAST_TYPE_NONE			0x00
#define GPIO_FAST_TYPE_QUARK_SC			0x01
#define GPIO_FAST_TYPE_QUARK_NC			0x02
#define GPIO_FAST_ID(type, rd_reg, wr_reg, mask) \
	(0UL | ((type) << 24) | ((rd_reg) << 16) | ((wr_reg) << 8) | (mask))
#define GPIO_FAST_ID_TYPE(id)	(((id) >> 24) & 0xFF)
#define GPIO_FAST_ID_RD_REG(id)	(((id) >> 16) & 0xFF)
#define GPIO_FAST_ID_WR_REG(id)	(((id) >> 8) & 0xFF)
#define GPIO_FAST_ID_MASK(id)	((id) & 0xFF)

int fastGpioFindUioByName(const char *name);

int fastGpioGetInfo(const int uio_num,
		    const int index,
		    char *path_fmt);

#endif // __FAST_GPIO_COMMON_H__

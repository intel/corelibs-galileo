/*
fast_gpio_nc.c Implement a fast GPIO path for the North-Cluster 
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
 *          Dan O'Donovan <dan@emutex.com> 2014
 */



#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <sys/io.h>
#include <sys/mman.h>
#include <sys/types.h>

/* ia32 port */
#include <Arduino.h>
#include <trace.h>
#include "fast_gpio_common.h"

#define UIO_NAME "sch_gpio"
/* Assuming a single IO Port */
#define UIO_PORT 0
#define UIO_START_PATH_FMT "/sys/class/uio/uio%d/portio/port%d/start"
#define UIO_SIZE_PATH_FMT "/sys/class/uio/uio%d/portio/port%d/size"

#define MY_TRACE_PREFIX	"fast_gpio_nc"

/*************************** Static ****************************/

struct fgpio_nc {
	char * regs;
	int uio_handle;

	unsigned baseport;
	unsigned size;
	unsigned uio_num;
};

static struct fgpio_nc fgpio;

/*************************** Global ****************************/

/**
 * fastGpioNCInit
 *
 * Initialise the fast NC GPIO interface
 */
int fastGpioNCInit(void)
{
	int i, ret;
	extern int errno;

	ret = fastGpioFindUioByName(UIO_NAME);
	if (ret < 0) {
		trace_error("Failed to find UIO name '%s': %s\n",
			    UIO_NAME, strerror(ret));
		return ret;
	}
	fgpio.uio_num = ret;

	ret = fastGpioGetInfo(fgpio.uio_num, UIO_PORT, UIO_START_PATH_FMT);
	if (ret < 0) {
		trace_error("Failed to read UIO base port: %s\n",
			    strerror(ret));
		return ret;
	}
	fgpio.baseport = ret;


	ret = fastGpioGetInfo(fgpio.uio_num, UIO_PORT, UIO_SIZE_PATH_FMT);
	if (ret < 0) {
		trace_error("Failed to read UIO size: %s\n", strerror(ret));
		return ret;
	}
	fgpio.size = ret;

	trace_debug("Requesting access to %u I/O ports starting at 0x%04X\n",
		    fgpio.size, fgpio.baseport); fflush(stdout);

	/* Get access to the ports */
	if (ioperm(fgpio.baseport, fgpio.size, 1)) {
		perror("ioperm");
		return errno;
	}

	trace_debug("Initialised PIO on UIO %d size %d baseport 0x%04X\n",
		    fgpio.uio_num, fgpio.size, fgpio.baseport); fflush(stdout);
	return 0;
}

/**
 * fastGpioNCFini
 *
 * Release the NC GPIO interface
 */
void fastGpioNCFini(void)
{
	if (ioperm(fgpio.baseport, fgpio.size, 0)) {
		perror("ioperm");
	}
}

/**
 * fastGpioNCDigitalWrite
 *
 * Do a fast write to the GPIO regs
 * This is safe for input GPIO and completely unsafe for write
 * For write() the contract is either kernel or user-space may write NOT BOTH
 *
 * This version of fast write - does a read/modify/write which ends up
 * approx 1/2 the speed of a destructive write for any given square wave.
 */
void fastGpioNCDigitalWrite(uint8_t reg_offset, uint8_t gpio_mask, uint8_t val)
{
	uint8_t regval;

	regval = inb(fgpio.baseport + reg_offset);
	if (val){
		regval |= gpio_mask;
	}else{
		regval &= ~gpio_mask;
	}
	outb(regval, fgpio.baseport + reg_offset);
}

/**
 * fastGpioNCDigitalWriteDestructive
 *
 * Do a fast write to the GPIO regs - destructively. We assume you know all
 * the values of the bits in question and don't do a read/modify/write.
 *
 */
void fastGpioNCDigitalWriteDestructive(uint8_t reg_offset, uint8_t gpio_levels)
{
	outb(gpio_levels, fgpio.baseport + reg_offset);
}

/**
 * fastGpioNCDigitalRead
 *
 * Do a fast read from the GPIO input regs
 */
uint8_t fastGpioNCDigitalRead(uint8_t reg_offset, uint8_t gpio_mask)
{
	uint8_t regval;
	regval = inb(fgpio.baseport + reg_offset) & gpio_mask;
	return regval;
}
/**
 * fastGpioNCDigitalLatch
 *
 * Read the current state of the GPIO output registers.
 *
 */
uint8_t fastGpioNCDigitalLatch(uint8_t reg_offset)
{
	uint8_t regval;

	regval = inb(fgpio.baseport + reg_offset);

	return regval;
}

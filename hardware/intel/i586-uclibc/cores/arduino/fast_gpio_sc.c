/*
fast_gpio_sc.c Implement a fast GPIO path for the South-Cluster 
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
 *
 * An explicit contract exists between this code and the in-kernel driver, since we both 'own'
 * the registers in question - user-space undertakes - never - ever to run concurrent data
 * whilst using the fast GPIO driver - in other words - user-space guarantees to never drive
 * traffic that can conflict with the kernel code.
 *
 * Author : Bryan O'Donoghue <bryan.odonoghue@intel.com> 2013
 */

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>

#include <Arduino.h>
#include <trace.h>
#include "fast_gpio_common.h"

#define UIO_NAME "gpio uio"
/* Assuming a single mmap region */
#define UIO_MAP 0
#define UIO_MAP_SIZE_PATH_FMT "/sys/class/uio/uio%d/maps/map%d/size"
#define UIO_DEVICE_PATH_FMT "/dev/uio%d"

#define MY_TRACE_PREFIX	"fast_gpio_sc"
#define MAP_SIZE	0x1000			/* PCI bar region size */

/*************************** Static ****************************/

struct fgpio_sc {
	char * regs;
	int uio_handle;
	int uio_num;
	int map_size;
};

static struct fgpio_sc fgpio;

/*************************** Global ****************************/

/**
 * fastGpioSCInit
 *
 * Initialise the fast SC GPIO interface
 */
int fastGpioSCInit(void)
{
	extern int errno;
	char uio_device[32];
	int ret;

	fgpio.regs = NULL;
	fgpio.uio_handle = -1;

	ret = fastGpioFindUioByName(UIO_NAME);
	if (ret < 0) {
		trace_error("Failed to find UIO name '%s': %s\n",
			    UIO_NAME, strerror(ret));
		return ret;
	}
	fgpio.uio_num = ret;
	snprintf(uio_device, sizeof(uio_device), UIO_DEVICE_PATH_FMT, fgpio.uio_num);

	ret = fastGpioGetInfo(fgpio.uio_num, UIO_MAP, UIO_MAP_SIZE_PATH_FMT);
	if (ret < 0) {
		trace_error("Failed to read UIO map size: %s\n", strerror(ret));
		return ret;
	}
	fgpio.map_size = ret;

	/* Get handle to UIO regs */
	fgpio.uio_handle = open(uio_device, O_RDWR);
	if (fgpio.uio_handle < 0){
		trace_error("unable to open %s O_RDWR\n", uio_device);
		return errno;
	}

	/* mmap */
	fgpio.regs = (char*)mmap(NULL, fgpio.map_size,
				 PROT_READ|PROT_WRITE,
				 MAP_FILE|MAP_SHARED,
				 fgpio.uio_handle, 0);
	if (fgpio.regs == MAP_FAILED){
		trace_error("unable to mmap UIO %s @ handle %d",
			    uio_device, fgpio.uio_handle);
		close(fgpio.uio_handle);
		fgpio.uio_handle = -1;
	}

	trace_debug("successfully mapped %s size %d handle %d",
		    uio_device, fgpio.map_size, fgpio.uio_handle);
	return 0;
}

/**
 * fastGpioSCInit
 *
 * Tear down the fast SC GPIO interface
 */
void fastGpioSCFini(void)
{
	if(fgpio.regs != NULL){
		munmap(fgpio.regs, fgpio.map_size);
		fgpio.regs = NULL;
	}

	if (fgpio.uio_handle != -1){
		close(fgpio.uio_handle);
		fgpio.uio_handle = -1;
	}
}


/**
 * fastGpioSCDigitalWrite
 *
 * Do a fast write to the GPIO regs
 * This is safe for input GPIO and completely unsafe for write
 * For write() the contract is either kernel or user-space may write NOT BOTH
 *
 * This version of fast write - does a read/modify/write which ends up approx
 * 1/4 the speed of a destructive write for any given square wave.
 */
void fastGpioSCDigitalWrite(uint8_t reg_offset, uint8_t gpio, uint8_t val)
{
	if (val){
		*(volatile uint32_t*)fgpio.regs |= gpio;
	}else{
		*(volatile uint32_t*)fgpio.regs &= ~gpio;
	}
}

/**
 * fastGpioSCDigitalWriteDestructive
 *
 * Do a fast write to the GPIO regs - destructively. We assume you know all the
 * values of the bits in question and don't do a read/modify/write.
 *
 */
void fastGpioSCDigitalWriteDestructive(uint8_t reg_offset, uint8_t gpio)
{
	*(volatile uint32_t*)fgpio.regs = gpio;
}

/**
 * fastGpioSCDigitalRead
 *
 * Do a fast read from the GPIO input regs
 *
 */
uint8_t fastGpioSCDigitalRead(uint8_t reg_offset, uint8_t gpio)
{
	uint32_t regval;
	regval = *(volatile uint32_t*)(fgpio.regs + reg_offset);
	regval &= gpio;
	return regval;
}

/**
 * fastGpioSCDigitalLatch
 *
 * Read the current state of the GPIO output register.
 *
 */
uint32_t fastGpioSCDigitalLatch(uint8_t reg_offset)
{
    return *(volatile uint32_t*)fgpio.regs;
}

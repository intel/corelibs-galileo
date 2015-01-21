/* interrupt.c  Interruption interface
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
 * Provides a pseudo interrupt inteface which is broadly an analoge of the Arduino pin based interrupt
 * callback mechanism
 * We don't support 'real' interrupts from kernel to user-space right now since that's way out of scope
 *
 * Author: Bryan O'Donoghue <bryan.odonoghue@intel.com>
 */
#define _GNU_SOURCE
#include <assert.h>
#include <errno.h>
#include <fcntl.h>              /* Obtain O_* constant definitions */
#include <netinet/in.h>
#include <mqueue.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <linux/hpet.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>

/* ia32 port */
#include <Arduino.h>
#include <interrupt.h>
#include <trace.h>
#include <wiring_digital.h>

#define __TTYUART_IDX_TX 1
#define __TTYUART_IDX_RX 0

#define MY_TRACE_PREFIX "interrupt"

/*************************** Static ****************************/
struct interrupt {
	int ihandle;
	uint8_t pin;			// for reference
	int pin_index;			// pinGetIndex
	void (*callback)(void);		// function pointer
	char * reopen_path;
	uint32_t mode;
	uint32_t ct_thresh;		// For timers the threshold we trigger at value of zero indicates disabled
	uint32_t ct;			// Current count
	struct interrupt * pnext;
	struct interrupt * pprev;
};

struct interrupt_desc {
	uint32_t irq_count;
	int initialised;
	int pipe_tx_rx[2];		// Transmit message queue
	int hpet_handle;		// Handle to /dev/rtc - the source of our timers
	uint32_t hpet_freq;		// Current hpet freq
	uint32_t hpet_us;		// Period in us
	pthread_t thread;
	pthread_barrier_t barrier;
	pthread_mutex_t mutex;
	struct interrupt *phead;
};

/*
 * From Linux GPIO lib documentation:
 *
 * Sysfs attribute files are pollable.  The idea is that you read
 * the content and then you use 'poll' or 'select' to wait for
 * the content to change.  When the content changes (assuming the
 * manager for the kobject supports notification), poll will
 * return POLLERR|POLLPRI, and select will return the fd whether
 * it is waiting for read, write, or exceptions.
 * Once poll/select indicates that the value has changed, you
 * need to close and re-open the file, as simply seeking and reading
 * again will not get new data, or reset the state of 'poll'.
 * Reminder: this only works for attributes which actively support
 * it, and it is not possible to test an attribute from userspace
 * to see if it supports poll (Nether 'poll' or 'select' return
 * an appropriate error code).  When in doubt, set a suitable timeout value.
*/

static struct interrupt_desc idesc;

/**
 * interrupt_main
 *
 */
static void * interrupt_main ( void * pargs )
{
	int loop = 1, ret = 0, max = 0, data = 0;
	fd_set fdset, fdset_except;
	char dummy;
	extern int errno;
	struct interrupt * pinterrupt = NULL;

	pthread_barrier_wait(&idesc.barrier);

	/* Loop waiting for interrupt or shutdown command */
	while(loop == 1){
		/* zero */
		FD_ZERO(&fdset);
		FD_ZERO(&fdset_except);

		/* Add kick listener */
		FD_SET(idesc.pipe_tx_rx[__TTYUART_IDX_RX], &fdset);
		max = idesc.pipe_tx_rx[__TTYUART_IDX_RX];

		/* Add elements - iterate through linked list of active elements */
		pthread_mutex_lock(&idesc.mutex);

		pinterrupt = idesc.phead;
		while(pinterrupt != NULL){
			if (pinterrupt->ihandle == idesc.hpet_handle){
				/* skipping handle to fdset_except */
				pinterrupt = pinterrupt->pnext;
				continue;
			}

			max = max < pinterrupt->ihandle ? pinterrupt->ihandle : max;
			FD_SET(pinterrupt->ihandle, &fdset_except);

			/* Required for GPIOLib handles before select() - very nasty !*/
			read(pinterrupt->ihandle, &dummy, 1);
			pinterrupt = pinterrupt->pnext;
		}

		/* Add HPET to fdset */
		FD_SET(idesc.hpet_handle, &fdset);
		if (max < idesc.hpet_handle){
			max = idesc.hpet_handle;
		}

		pthread_mutex_unlock(&idesc.mutex);

		/* Select on the FD set - infinite timeout */
		ret = select(1 + max, &fdset, 0, &fdset_except, NULL);

		/* Receive bytes */
		switch(ret){
			case -1:
				fprintf(stderr, "critical fault during select errno=%d", errno);
				loop = 0;
				break;
			case 0:
				/* timeout */
				break;
			default:
				/* Process data */
				#if 0
					trace_debug("%s state change detected!", __func__);
				#endif

				/* pipe */
				if(FD_ISSET(idesc.pipe_tx_rx[__TTYUART_IDX_RX], &fdset)){
					/* Update loop */
					ret = read(idesc.pipe_tx_rx[__TTYUART_IDX_RX],(char*)&loop, sizeof(loop));
					if(ret < 0){
						if((ret =! EAGAIN) && (ret != EWOULDBLOCK)){
							/* Critical - exit out */
							loop = 0;
						}
					}
				}

				pthread_mutex_lock(&idesc.mutex);
				pinterrupt = idesc.phead;

				/* Interrupt */
				while(pinterrupt != NULL){

					if (FD_ISSET(pinterrupt->ihandle, &fdset_except)){
						/* Pin based interrupt */
						pinterrupt->callback();

						/* Need to reopen see documentation in Linux GPIO Lib */
						pinterrupt->ihandle = pinHandleReopen(pinterrupt->pin_index);
						if (pinterrupt->ihandle < 0){
							trace_error("Critical error unable to reopen pin %d handle - as required by gpiolib for select()!", pinterrupt->pin);
							pthread_exit(0);
						}
					}else if(FD_ISSET(pinterrupt->ihandle, &fdset)){
						/* Timer based interrupt */
						trace_debug("%s timer interrupt fired!", __func__);
                                                pinterrupt->ct++;
						if(pinterrupt->ct >= pinterrupt->ct_thresh){
							pinterrupt->ct = 0;
							pinterrupt->callback();
						}
						data = 1;
					}
					pinterrupt = pinterrupt->pnext;
				}

				/* Clear HPET */
				if (data == 1){
					if (read(idesc.hpet_handle, &data, sizeof(data)) != sizeof(data)) {
						trace_error("hpet_poll: read failed\n");
					}
				}

				pthread_mutex_unlock(&idesc.mutex);

				break;
		}
	}

	/* Exit out */
	pthread_exit(NULL);
}

/**
 * addIRQEntry
 *
 * Adds either a pin based or timer based "interrupt" to the linked list of IRQs
 * Calling context must hold pthread_mutex_lock(&idesc.mutex); and subsequently release
 */
static int addIRQEntry(int ihandle, uint32_t pin, void (*callback)(void), uint32_t mode, int ct_thresh)
{
	int ret = 0;
	struct interrupt * pinterrupt = NULL;

	/* add new entry to end of linked list */
	pinterrupt = malloc(sizeof(struct interrupt));
	if (pinterrupt == NULL){
		trace_error("%s oom malloc %d bytes fail!", __func__, sizeof(struct interrupt));
		ret = -ENOMEM;
		goto done;
	}
	pinterrupt->callback = callback;
	pinterrupt->pin = pin;
	pinterrupt->ihandle = ihandle;
	if (ihandle != idesc.hpet_handle){
		pinterrupt->reopen_path = pin2path(pin);
		pinterrupt->pin_index = pinGetIndex(pin);
	}
	pinterrupt->ct_thresh = ct_thresh;
	pinterrupt->ct = 0;
	pinterrupt->mode = mode;
	pinterrupt->pnext = NULL;
	pinterrupt->pprev = NULL;

	if (idesc.phead == NULL){
		idesc.phead = pinterrupt;
	}else{
		pinterrupt->pprev = idesc.phead;

		if(idesc.phead->pnext != NULL){
			pinterrupt->pnext = idesc.phead->pnext;
			pinterrupt->pnext->pprev = pinterrupt;
		}
		idesc.phead->pnext = pinterrupt;
	}
	idesc.irq_count++;
done:
	return ret;
}
/*************************** Global ****************************/

/**
 * Attach a callback to an gpio 'interrupt' asynchronous to sketch loop();
 *
 * pin: the pin number
 * function : the function to call when the interrupt occurs
 * mode: defines when the interrupt should be triggered - LOW , CHANGE, RISING, FALLING, HIGH
 */
void attachInterrupt(uint32_t pin, void (*callback)(void), uint32_t mode)
{
	int ihandle = -1;
	int loop = 1;
	struct interrupt * pinterrupt = idesc.phead;
	int ret = 0;

	/* error check */
	if (callback == NULL){
		trace_error("%s -EINVAL callback is null", __func__);
		return;
	}

	/* lock */
	pthread_mutex_lock(&idesc.mutex);

	/* configure pin mode */
	if (pinModeIRQ(pin, mode) < 0){
		trace_error("%s cannot configure pin %d to mode %d!", __func__, pin, mode);
		goto done;
	}

	/* ensure we have a persistent handle */
	ihandle = pin2gpiohandle(pin);
	if (ihandle < 0){
		trace_error("%s Arduino pin %d has no persistent gpiolib handle", __func__, pin);
		goto done;
	}

	/* check to see if this pin is already configured.. */
	while(pinterrupt != NULL){
		if(pinterrupt->pin == pin){
			trace_error("%s detachInterrupt before trying to configure pin %d again", __func__, pin);
			goto done;
		}
		pinterrupt = pinterrupt->pnext;
	}

	ret = addIRQEntry(ihandle, pin, callback, mode, 0);
done:
	/* unlock */
	pthread_mutex_unlock(&idesc.mutex);

	/* async notify */
	if(write(idesc.pipe_tx_rx[__TTYUART_IDX_TX], (const char*)&loop, sizeof(loop))<0){
		fprintf(stderr, "interrupt error %d\n", errno);
	}

	/* Adding an interrupt failed !*/
	if( ret != 0){
		/* Give time for trace to complete */
		sleep(10);
		exit(ret);
	}
}

/**
 * Detach a callback to an gpio 'interrupt' asynchronous to sketch loop();
 */
void detachInterrupt(uint32_t pin)
{
	struct interrupt * pinterrupt = idesc.phead;
	int kick = 0, loop = 1;

	/* lock */
	pthread_mutex_lock(&idesc.mutex);

	/* check to see if this pin is already configured.. */
	while(pinterrupt != NULL && kick == 0){
		if(pinterrupt->pin == pin){

			/* remove entry */
			if (pinterrupt->pprev != NULL) {
				pinterrupt->pprev->pnext = pinterrupt->pnext;
				if (idesc.phead == pinterrupt)
					idesc.phead = pinterrupt->pprev;
			}
			if(pinterrupt->pnext != NULL) {
				pinterrupt->pnext->pprev = pinterrupt->pprev;
				if (idesc.phead == pinterrupt)
					idesc.phead = pinterrupt->pnext;
			}
			if (idesc.phead == pinterrupt)
				idesc.phead = NULL;

			(void)pinModeIRQ(pinterrupt->pin, (uint8_t)NONE);
			free(pinterrupt);

			pinterrupt = NULL;

			/* kick interrupt thread */
			kick = 1;

		} else {
			pinterrupt = pinterrupt->pnext;
		}
	}

	idesc.irq_count--;

	/* unlock */
	pthread_mutex_unlock(&idesc.mutex);

	/* async notify */
	if (kick == 1)
		if(write(idesc.pipe_tx_rx[__TTYUART_IDX_TX], (const char*)&loop, sizeof(loop))<0){
			fprintf(stderr, "interrupt error %d\n", errno);
		}
}

/**
 * Attach a callback to a timer 'interrupt' asynchronous to sketch loop();
 *
 * function : the function to call when the interrupt occurs
 * microseconds: defines when the interrupt should be triggered in the timing loop. Must be even multiple of idesc.hpet_freq
 */
void attachTimerInterrupt(void (*callback)(void), int32_t microseconds)
{
	unsigned long ratio = 0;
	int ret = 0;
	int loop = 1;

	if (microseconds == 0){
		trace_error("cannot set zero microsecond timer");
		return;
	}

	/* Convert microsecond to Hz */
	if (microseconds == -1){
		ratio = idesc.hpet_freq;
                microseconds = idesc.hpet_us;
	}else{
		ratio = microseconds == 0 ? 0 : microseconds/idesc.hpet_us;
	}

	if ( ratio == 0 || idesc.hpet_freq == 0 || microseconds%idesc.hpet_us != 0){
		trace_error("Cannot add timer with period %d us - hpet @ %d Hz cp %p", microseconds, idesc.hpet_us, callback);
		return;
	}

	/* Give some debug feedback */
	trace_debug("Setting timer interrupt callback @%p hpet @ %lu Hz timer @ %lu Hz",
		callback, idesc.hpet_freq, ratio);

	/* lock */
	pthread_mutex_lock(&idesc.mutex);

	ret = addIRQEntry(idesc.hpet_handle, -1, callback, 0, ratio);

	/* unlock */
	pthread_mutex_unlock(&idesc.mutex);

	/* async notify - not strictly necessary - remove ?*/
	if(write(idesc.pipe_tx_rx[__TTYUART_IDX_TX], (const char*)&loop, sizeof(loop))<0){
		fprintf(stderr, "interrupt error %d\n", errno);
	}

	/* Adding an interrupt failed !*/
	if( ret != 0){
		/* Give time for trace to complete */
		sleep(10);
		exit(ret);
	}
}

/**
 * setMinimumTimerFreq
 *
 */
int setMinimumTimerFreq(uint32_t microseconds)
{
	struct hpet_info info;
	int loop = 1;
	unsigned long freq = 1000000;

	/* Convert microsecond to Hz */
	freq = microseconds == 0 ? 0 : freq/microseconds;

	/* Error check range */
	if ( freq == 0 || freq > MAX_VARIANT_HPET_FREQ_HZ){
		trace_error("Cannot set frequency %lu min 0 max %lu", freq, MAX_VARIANT_HPET_FREQ_HZ);
		return -EINVAL;
	}

	/* Send command to HPET @ new freq for timer2 */
	if (ioctl(idesc.hpet_handle, HPET_IRQFREQ, freq) < 0) {
    		trace_error("hpet set frequency %lx failed", freq);
    		return -EIO;
	}

	if (ioctl(idesc.hpet_handle, HPET_INFO, &info) < 0) {
		trace_error("hpet_info: failed to get info\n");
		return -EIO;
	}

	trace_info("hpet_info: hi_irqfreq 0x%lx hi_flags 0x%lx hi_hpet %d hi_timer %d",
		info.hi_ireqfreq, info.hi_flags, info.hi_hpet, info.hi_timer);

	if (info.hi_flags && (ioctl(idesc.hpet_handle, HPET_EPI, 0) < 0)) {
		trace_error("hpet_poll: HPET_EPI failed\n");
		return -EIO;
	}

	/* This only works once - to-do add logic to only call when disabled */
	if (ioctl(idesc.hpet_handle, HPET_IE_ON, 0) < 0) {
		trace_error("hpet_poll, HPET_IE_ON failed\n");
		//return;
	}

	// Kick thread
	if(write(idesc.pipe_tx_rx[__TTYUART_IDX_TX], (const char*)&loop, sizeof(loop))<0){
		trace_error("interrupt error %d", errno);
	}

	idesc.hpet_freq = freq;
	idesc.hpet_us = microseconds;
	trace_info("set hpet frequency to %lu Hz", freq);

	return 0;
}

/**
 * interrupt_init
 *
 * Initialise the interrupt queue
 */
int interrupt_init( void )
{
	extern int errno;
	int ret = 0, fd;
	struct hpet_info info;
	char buf[0x20];

	/* Setup pipe */
	ret = pipe2(idesc.pipe_tx_rx, O_NONBLOCK);
	if (ret < 0){
		fprintf(stderr, "error making pipe!\n");
		return errno;
	}

	/* Open /dev/hpet - for timing purposes */
	idesc.hpet_handle = open("/dev/hpet", O_RDONLY);
	if (idesc.hpet_handle < 0){
		fprintf(stderr, "unable to open /dev/hpet O_RDONLY\n");
		return errno;
	}

	/* Configure maximum frequency to the frequency specified in MAX_VARIANT_HPET_FREQ_HZ */
	fd = open("/proc/sys/dev/hpet/max-user-freq", O_RDWR);
	if (fd < 0){
		close(idesc.hpet_handle);
		fprintf(stderr, "unable to open path to set HPET max Hz\n");
		return errno;
	}
	memset(buf, 0x00, sizeof(buf));
	ret = snprintf(buf, sizeof(buf),"%d", MAX_VARIANT_HPET_FREQ_HZ);
	if(write(fd, buf, ret) != ret){
		close(fd);
		close(idesc.hpet_handle);
		fprintf(stderr, "Failed to set max HPET Hz value to %s\n", buf);
		return errno;
	}
	close(fd);

	/*
	 * Switch off HPET timer TODO: bring out more than one HPET timer kernel uses 0 - but 1 & 2 should be free
	 * We only use timer #2 right now
	 */
	(void)setMinimumTimerFreq(0);

	if (ioctl(idesc.hpet_handle, HPET_INFO, &info) < 0) {
		fprintf(stderr, "hpet_info: failed to get info\n");
		fflush(stderr);
		return -errno;
	}
	trace_info("hpet_info: hi_irqfreq 0x%lx hi_flags 0x%lx hi_hpet %d hi_timer %d",
		info.hi_ireqfreq, info.hi_flags, info.hi_hpet, info.hi_timer);

	idesc.irq_count = 0;
	idesc.phead = NULL;
	idesc.hpet_freq = 0;

	pthread_mutex_init(&idesc.mutex, 0);

	pthread_barrier_init(&idesc.barrier, 0, 2);

	/* initiate IRQ handler */
	pthread_create(&idesc.thread, NULL, interrupt_main, 0);

	/* sync startup */
	pthread_barrier_wait(&idesc.barrier);

	/* Good to go - start the client thread */
	return 0;
}

/**
 * interrupt_fini
 *
 * Shutdown interrupt services
 */
void interrupt_fini(void)
{
	int loop = 0;

	/* Send interrupt message */
	if(write(idesc.pipe_tx_rx[__TTYUART_IDX_TX], (const char*)&loop, sizeof(loop))<0){
		fprintf(stderr, "interrupt error %d\n", errno);
	}

	/* Join thread */
	pthread_join(idesc.thread, 0);

	if (idesc.pipe_tx_rx[__TTYUART_IDX_TX] != -1){
		close(idesc.pipe_tx_rx[__TTYUART_IDX_TX]);
		idesc.pipe_tx_rx[__TTYUART_IDX_TX] = -1;
	}

	if (idesc.pipe_tx_rx[__TTYUART_IDX_RX] != -1){
		close(idesc.pipe_tx_rx[__TTYUART_IDX_RX]);
		idesc.pipe_tx_rx[__TTYUART_IDX_RX] = -1;
	}
}

/*
trace.c implements block trace facility
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

/*
 * Author: Bryan O'Donoghue <bryan.odonoghue@linux.intel.com>
 *
 * Allows the calling task to schedule a print and then go about
 * it's work.
 *
 * Depending on enabled trace targets - this could go to UART, ETH
 * USB - whatever.
 */
#include <assert.h>
#include <errno.h>
#include <netinet/in.h>
#include <mqueue.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
//#include <sys/stat.h>
#include <sys/time.h>

/* ia32 port */
#include "trace.h"

#define __TTYUART_IDX_TX 1
#define __TTYUART_IDX_RX 0

/* Enabled by default */
int g_TraceEnable = 1;
trace_level_t g_tlevel = TRACE_LEVEL_ERROR;

const char * qname = "/trace";
#define TRACE_MSG_DEPTH 0x0A

/*************************** Static ****************************/

typedef enum {
	TRACE_CMD_TRACE = 0,
	TRACE_CMD_EXIT
}trace_cmd_t;

struct trace_msg{
	trace_cmd_t tcmd;
	trace_level_t tlevel;
	char tbuf[TRACE_BUFSIZE];
};

struct trace_desc {
	int sockfd;
	int trace_target;
	int initialised;
	int pipe_tx_rx[2];			/* Transmit message queue */
	pthread_t thread;
	pthread_barrier_t barrier;
	struct trace_msg tmsg;
	struct sockaddr_in sin;
};

static struct trace_desc tdesc;
static char * trce_msg [] = {
	"error",
	"info",
	"debug"
};

/**
 * trace_main
 *
 */
static void * trace_main ( void * pargs )
{
	int res = 0, loop = 1, ret = 0;
	fd_set fdset;
	struct trace_msg tmsg;
	extern int errno;

	pthread_barrier_wait(&tdesc.barrier);

	/* Loop waiting for trace or shutdown command */
	while(loop == 1){
		/* zero */
		FD_ZERO(&fdset);

		/* Add elements */
		FD_SET(tdesc.pipe_tx_rx[__TTYUART_IDX_RX] , &fdset);

		/* Select on the FD set - infinite timeout */
		ret = select(1 + tdesc.pipe_tx_rx[__TTYUART_IDX_RX], &fdset, 0, 0, NULL);

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
				if(FD_ISSET(tdesc.pipe_tx_rx[__TTYUART_IDX_RX], &fdset)){

					/* Read in frame */
					ret = read(tdesc.pipe_tx_rx[__TTYUART_IDX_RX],(char*)&tmsg, sizeof(tmsg));
					if(ret < 0){
						if((ret =! EAGAIN) && (ret != EWOULDBLOCK)){
							/* Critical - exit out */
							loop = 0;
						}
					}else{
						switch(tmsg.tcmd){
							case TRACE_CMD_TRACE:
								if(tdesc.trace_target & TRACE_TARGET_UART){
										if( tmsg.tlevel == TRACE_LEVEL_ERROR){
											fprintf(stderr, tmsg.tbuf);
											fflush(stderr);
										}else{
											printf(tmsg.tbuf);
											fflush(stdout);
										}
								}
								if(tdesc.trace_target &TRACE_TARGET_ETH){
										sendto(tdesc.sockfd, tmsg.tbuf,
												sizeof(tmsg.tbuf), 0, (const struct sockaddr*)&tdesc.sin, sizeof(tdesc.sin));
								}
								break;
							case TRACE_CMD_EXIT:
									loop = 0;
								break;
							default:
								fprintf(stderr, "Unknown trace message %d\n", tmsg.tcmd);
								fflush(stderr);
								break;
						}
					}
				}
				break;
		}
	}

	/* Exit out */
	pthread_exit(NULL);
}

/**
 * trace_init
 *
 * Initialise the trace queue
 */
int trace_init( trace_level_t new_tlevel, in_addr_t ip_addr)
{

	extern int errno;
	int ret = 0;
	g_tlevel = new_tlevel;

	if(ip_addr != 0){
		tdesc.sockfd = socket(AF_INET, SOCK_DGRAM, 0);
		if(tdesc.sockfd == -1){
			fprintf(stderr, "Unable to open socket in %s\n", __FUNCTION__);
			return -errno;
		}

		tdesc.sin.sin_family = AF_INET;
		tdesc.sin.sin_port = htons(TRACE_UDP_PORT);
		tdesc.sin.sin_addr.s_addr = ip_addr;
	}else{
		tdesc.sockfd = -1;
	}

	/* Setup pipe */
	ret = pipe2(tdesc.pipe_tx_rx, O_NONBLOCK);
	if (ret < 0){
		fprintf(stderr, "error making pipe!\n");
		return errno;
	}

	pthread_barrier_init(&tdesc.barrier, 0, 2);

	// Initiate IRQ handler
	pthread_create(&tdesc.thread, NULL, trace_main, 0);

	// sync startup
	pthread_barrier_wait(&tdesc.barrier);

	/* Good to go - start the client thread */
	return 0;
}

/**
 * trace_fini
 *
 * Shutdown trace services
 */
void trace_fini(void)
{
	struct trace_msg tmsg;

	/* Send trace message */
	tmsg.tcmd = TRACE_CMD_EXIT;
	if(write(tdesc.pipe_tx_rx[__TTYUART_IDX_TX], (const char*)&tmsg, sizeof(tmsg))<0){
		fprintf(stderr, "trace error %d\n", errno);
	}

	/* Join thread */
	pthread_join(tdesc.thread, 0);

	if (tdesc.pipe_tx_rx[__TTYUART_IDX_TX] != -1){
		close(tdesc.pipe_tx_rx[__TTYUART_IDX_TX]);
		tdesc.pipe_tx_rx[__TTYUART_IDX_TX] = -1;
	}

	if (tdesc.pipe_tx_rx[__TTYUART_IDX_RX] != -1){
		close(tdesc.pipe_tx_rx[__TTYUART_IDX_RX]);
		tdesc.pipe_tx_rx[__TTYUART_IDX_RX] = -1;
	}
}

/**
 * trace_target_enable
 *
 * Actives a trace target
 */
void trace_target_enable(trace_target_t target)
{
	switch(target){
		case TRACE_TARGET_UART:
		case TRACE_TARGET_ETH:
		case TRACE_TARGET_SYSLOG:
			tdesc.trace_target |= target;
			break;
		default:
			break;
	}
}

/**
 * trace_set_level
 *
 * Sets the current trace level
 */
void trace_set_level(trace_level_t new_tlevel)
{
	g_tlevel = new_tlevel;
}

/**
 * trace_enable
 *
 * Enable/disable trace
 */
void trace_enable(int enable)
{
	if(enable){
		g_TraceEnable = 1;
	}else{
		g_TraceEnable = 0;
	}
}
/*
 * vTrace
 *
 * Print out a trace string
 */
void trace(trace_level_t tlevel, const char * prefix, const char * fmt, ...)
{
	va_list arg;
	uint32_t len = 0, tlen = TRACE_BUFSIZE-3;
	struct trace_msg tmsg;
	extern int errno;
	//struct tm mytm;

	memset(&tmsg.tbuf, 0x00, TRACE_BUFSIZE);
#if 0
	/* Add timestamp */
	timer_get_ts(&mytm);
	len += snprintf(tmsg.tbuf, tlen, "[%02d/%02d/%02d %02d:%02d:%02d] %s-%s\t: ",
		mytm.tm_mday, mytm.tm_mon, mytm.tm_year,
		mytm.tm_hour, mytm.tm_min, mytm.tm_sec,
		prefix, trce_msg[tlevel]);
	tlen -= len;
#else
	len += snprintf(tmsg.tbuf, tlen, "%s-%s: ", prefix, trce_msg[tlevel]);
#endif

	/* Conjoin user message */
	va_start(arg,fmt);
	len += vsnprintf(tmsg.tbuf+len, tlen, fmt, arg);
	va_end(arg);

	/* Unambiguously append CRLF00 */
	tmsg.tbuf[len++] = '\n';
	tmsg.tbuf[len] = '\0';

	/* Send trace message */
	tmsg.tcmd = TRACE_CMD_TRACE;
	tmsg.tlevel = tlevel;

	if(write(tdesc.pipe_tx_rx[__TTYUART_IDX_TX], (const char*)&tmsg, sizeof(tmsg))<0){
		fprintf(stderr, "trace error %d\n", errno);
		fflush(stderr);
	}
}


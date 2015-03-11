/*
trace.h Provides the trace API to the clanton-bridgetest application
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
 * Author: Bryan O'Donoghue <bryan.odonoghue@linux.intel.com>
 *
 * Allows the calling task to schedule a print and then go about
 * it's work.
 *
 * Depending on enabled trace targets - this could go to UART, ETH
 * USB - whatever.
 */

#ifndef __TRACE_H__
#define __TRACE_H__


#define TRACE_BUFSIZE 0x100
#define TRACE_UDP_PORT	0x1235

#ifndef _WINDOWS
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
	TRACE_TARGET_UART = 1,
	TRACE_TARGET_ETH,
	TRACE_TARGET_SYSLOG,
}trace_target_t;

typedef enum {
	TRACE_LEVEL_ERROR = 0,
	TRACE_LEVEL_INFO,
	TRACE_LEVEL_DEBUG,
}trace_level_t;

/**
 * trace_init
 *
 * Initialise the trace queue
 */
int trace_init( trace_level_t tlevel, in_addr_t trace_ip );

/**
 * trace_fini
 *
 * Finish the trace module
 */
void trace_fini( void );

/**
 * trace_target_enable
 *
 * Actives a trace target
 */
void trace_target_enable(trace_target_t target);

/*
 * vTrace
 *
 * Print out a trace string
 */
void trace(trace_level_t tlevel, const char * prefix, const char * fmt, ...);

/**
 * trace_enable
 *
 * Enable/disable trace
 */
void trace_enable(int enable);

extern int g_TraceEnable;
extern trace_level_t g_tlevel;

/* Trace types */
#define trace_debug(fmt, ...) \
	if(g_TraceEnable == 1 && g_tlevel >= TRACE_LEVEL_DEBUG ){ \
		trace(TRACE_LEVEL_DEBUG, MY_TRACE_PREFIX, fmt, ##__VA_ARGS__); \
	}else{ \
		{} \
	}

#define trace_info(fmt , ...) \
	if(g_TraceEnable == 1 && g_tlevel >= TRACE_LEVEL_INFO){ \
		trace(TRACE_LEVEL_INFO, MY_TRACE_PREFIX, fmt, ##__VA_ARGS__); \
	}else{ \
		{} \
	}

#define trace_error(fmt , ...) \
	if(g_TraceEnable == 1 && g_tlevel >= TRACE_LEVEL_ERROR){ \
		trace(TRACE_LEVEL_ERROR, MY_TRACE_PREFIX, fmt, ##__VA_ARGS__); \
	}else{ \
		{} \
	}

#ifdef __cplusplus
}
#endif


#endif /* _WINDOWS */
#endif /* __TRACE_H__ */


/*
TTYUART.h implementation for TTY communication
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

#ifndef __TTYUART_H__
#define __TTYUART_H__

#include "HardwareSerial.h"
#include "RingBuffer.h"

#include <pthread.h>

class TTYUARTClass : public HardwareSerial
{
private:

	// Data
	RingBuffer *_rx_buffer ;
	int _dwId;				// For mux identification - see variant.cpp for index resolution
	int _tty_fd;
	int _pipe_tx_rx[2];
	char * _tty_name;
	uint32_t _dwBaudRate;
	bool _console;

	// Use threads to support the model of available/peek the Serial class requires
	pthread_t _thread;
	pthread_mutex_t _mutex;
	pthread_barrier_t _barrier;

	int _detach_console( void ) ;
	int _reattach_console( void ) ;

public:
	TTYUARTClass( RingBuffer* pRx_buffer, uint32_t dwId, bool console = false ) ;
	virtual ~TTYUARTClass();

	static void * TTYIrqHandler(void * pargs);	// static has no implicit this
	int init_tty( char * tty_name );
	void begin( const uint32_t dwBaudRate ) ;
	void end( void ) ;
	int available( void ) ;
	bool overflow( void ) ;
	int peek( void ) ;
	int read( void ) ;
	void flush( void ) ;
	size_t write( const uint8_t c ) ;
	using Print::write ; // pull in write(str) and write(buf, size) from Print

	operator bool() { return true; }; // UART always active
};

#endif /* __TTYUART_H__ */

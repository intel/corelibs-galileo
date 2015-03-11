/*
TTYUART.cpp implementation for TTY communication
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

#include "Mux.h"
#include "TTYUART.h"
#undef B0		// termios.h has a B0 (baud rate 0) definition - which collides with arduino definition of B0 00000000 !

#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#define __TTYUART_IDX_TX 1
#define __TTYUART_IDX_RX 0

int TTYUARTClass::_detach_console( void )
{
	/* We need to detach the current users of the system console, which we
	 * assume to be (i) kernel logger and (ii) getty or a command shell
	 * Here's how we're going to do it:
	 * - Change the kernel log level to filter log messages to the console
	 * - Modify inittab to stop it respawning gettys on the console
	 *   - Add a mechanism to restore normal behaviour on next reboot
	 * - Tell init to reload the modified inittab
	 * - Kill any processes still using the console
	 *   - Send them the KILL signal (bash doesn't seem to respond to TERM).
	 */

	int ret = 0;
	char cmd[256];
	char *ttydev;

	if (_tty_name == NULL)
		return -EINVAL;

	// Check if we've been here already
	if (!access("/etc/inittab.restore", F_OK))
		return 0;

	ttydev = strstr(_tty_name, "tty");
	if (!ttydev) {
		fprintf(stderr, "Invalid TTY name %s\n", _tty_name);
		return -EINVAL;
	}

	// Change the kernel log level
	system("export PATH=/bin:/usr/bin; dmesg -n 1");

	// We're about to change inittab, but we need our changes to be rolled
	// back on the next reboot.  So first, we make a backup, then we add a
	// rollback mechanism inside inittab itself.
	system("/bin/cp -f /etc/inittab /etc/inittab.restore");
	snprintf(cmd, sizeof(cmd),
		 "/bin/sed -i -e '/^\\(S.*getty.*%s$\\)/i\\\n"
		 "# Disabling getty to allow Arduino sketch to use %s and\n"
		 "# adding sysinit commands to restore normal behaviour on next boot\n"
		 "' /etc/inittab",
		 ttydev, ttydev);
	system(cmd);
	snprintf(cmd, sizeof(cmd),
		 "/bin/sed -i -e '/^\\(S.*getty.*%s$\\)/i\\\n"
		 "res1:2345:sysinit:/bin/mv -f /etc/inittab.restore /etc/inittab\n"
		 "' /etc/inittab",
		 ttydev);
	system(cmd);
	snprintf(cmd, sizeof(cmd),
		 "/bin/sed -i -e '/^\\(S.*getty.*%s$\\)/i\\\n"
		 "res2:2345:sysinit:/sbin/telinit q\n"
		 "' /etc/inittab",
		 ttydev);
	system(cmd);

	// Modify inittab to stop use of this console device for user log-in
	snprintf(cmd, sizeof(cmd),
		 "/bin/sed -i -e 's/^\\(S.*getty.*%s$\\)/#\\1/' /etc/inittab",
		 ttydev);
	system(cmd);

	// Tell init to reload its configuration
	system("/sbin/telinit q");
 
	// Forcibly kill any processes still using the tty device.
	snprintf(cmd, sizeof(cmd),
		 "export PATH=/bin:/usr/bin; "
		 "lsof | grep '%s' | cut -f 1 | sort -u | xargs -r kill -9",
		 ttydev);
	system(cmd);

	return 0;
}

int TTYUARTClass::_reattach_console( void )
{
	// Restore the kernel log level
	system("/bin/dmesg -n 7");

	// Restore the original inittab and kick-start getty on the TTY
	system("/bin/mv -f /etc/inittab.restore /etc/inittab");
	system("/sbin/telinit q");

	return 0;
}

static ssize_t _read(int fd, void *buf, size_t count)
{
	extern ssize_t read(int fd, void *buf, size_t count);
	return read(fd, buf, count);
}

static ssize_t _write(int fd, const void *buf, size_t count)
{
	extern ssize_t write(int fd, const void *buf, size_t count);
	return write(fd, buf, count);
}

// Constructors ////////////////////////////////////////////////////////////////

TTYUARTClass::TTYUARTClass( RingBuffer* pRx_buffer, uint32_t dwId, bool console )
{
	_rx_buffer = pRx_buffer;
	_tty_name = NULL;
	_tty_fd = -1;
	_pipe_tx_rx[__TTYUART_IDX_TX] = -1;
	_pipe_tx_rx[__TTYUART_IDX_RX] = -1;
	_dwId = dwId;
	_console = console;
	pthread_mutex_init(&_mutex, 0);
}

TTYUARTClass::~TTYUARTClass()
{
	pthread_mutex_destroy(&_mutex);

	if (_tty_name)
		free(_tty_name);
}

// Private methods /////////////////////////////////////////////////////////////

void * TTYUARTClass::TTYIrqHandler(void * pargs)
{
	TTYUARTClass * pTTYUARTClass = (TTYUARTClass*)pargs;
	fd_set fdset;
	char rx;
	unsigned int spin = 1;
	int ret = 0, max = pTTYUARTClass->_pipe_tx_rx[__TTYUART_IDX_RX];
	extern int errno;

	pthread_barrier_wait(&pTTYUARTClass->_barrier);

	/* Get max */
	if ( max < pTTYUARTClass->_tty_fd )
		max = pTTYUARTClass->_tty_fd;

	while(spin == 1){
		/* Init flag */
		rx = 0;

		/* zero */
		FD_ZERO(&fdset);

		/* Add elements */
		FD_SET(pTTYUARTClass->_tty_fd, &fdset);
		FD_SET(pTTYUARTClass->_pipe_tx_rx[__TTYUART_IDX_RX] , &fdset);

		/* Select on the FD set - infinite timeout */
		ret = select(1 + max, &fdset, 0, 0, NULL);

		/* Receive bytes */
		switch(ret){
			case -1:
				fprintf(stderr, "critical fault during select errno=%d", errno);
				spin = 0;
				break;
			case 0:
				/* timeout */
				break;
			default:
				/* Process data */
				if(FD_ISSET(pTTYUARTClass->_tty_fd, &fdset)){

					do {
						/* Read in frame */
						ret = _read(pTTYUARTClass->_tty_fd, &rx, 1);
						if(ret < 0){
							if((ret =! EAGAIN) && (ret != EWOULDBLOCK)){
								/* Critical - exit out */
								spin = 0;
							}
						}else{
							pthread_mutex_lock(&pTTYUARTClass->_mutex);
							pTTYUARTClass->_rx_buffer->store_char( rx ) ;
							pthread_mutex_unlock(&pTTYUARTClass->_mutex);
						}
					}while(ret > 0);
				}

				// signal sent to end spin
				if(FD_ISSET(pTTYUARTClass->_pipe_tx_rx[__TTYUART_IDX_RX] , &fdset)){
					spin = 0;
				}
				break;
		}
	}

	/* Exit out */
	pthread_exit(NULL);

	return NULL;
}

// Public Methods //////////////////////////////////////////////////////////////
int TTYUARTClass::init_tty( char * tty_name )
{
	extern int errno;
	int ret;

	if(tty_name == NULL)
		return -EINVAL;

	if (_tty_name)
		free(_tty_name);

	_tty_name = strdup(tty_name);
	if (_tty_name == NULL)
		return -ENOMEM;

	return 0;
}

void TTYUARTClass::begin( const uint32_t dwBaudRate )
{
	struct termios newtios, oldtermios;
	int ret;

	if (_tty_name == NULL) {
		fprintf(stderr, "tty_name not initialised\n");
		fflush(stderr);
		return;
	}

	if (_console) {
		if (_detach_console()) {
			fprintf(stderr,
				"Failed to detach system console from %s\n",
				_tty_name);
			fflush(stderr);
			return;
		}
	}

	// Open handle to tty
	_tty_fd = open(_tty_name, O_RDWR | O_NONBLOCK);
	if (_tty_fd < 0){
		fprintf(stderr, "unable to open %s rdwr: %s\n",
			_tty_name, strerror(errno));
		fflush(stderr);
		return;
	}

	// Open fifo
	ret = pipe2(_pipe_tx_rx, O_NONBLOCK);
	if (ret < 0){
		fprintf(stderr, "error making pipe: %s\n", strerror(errno));
		fflush(stderr);
		return;
	}

	(void)muxSelectUart(_dwId);	// Mux pins as appropriate for this tty

	switch(dwBaudRate){
		case 50:
			_dwBaudRate = B50;
			break;
		case 75:
			_dwBaudRate = B75;
			break;
		case 110:
			_dwBaudRate = B110;
			break;
		case 134:
			_dwBaudRate = B134;
			break;
		case 150:
			_dwBaudRate = B150;
			break;
		case 200:
			_dwBaudRate = B200;
			break;
		case 300:
			_dwBaudRate = B300;
			break;
		case 600:
			_dwBaudRate = B600;
			break;
		case 1200:
			_dwBaudRate = B1200;
			break;
		case 1800:
			_dwBaudRate = B1800;
			break;
		case 2400:
			_dwBaudRate = B2400;
			break;
		case 4800:
			_dwBaudRate = B4800;
			break;
		case 9600:
			_dwBaudRate = B9600;
			break;
		case 19200:
			_dwBaudRate = B19200;
			break;
		case 38400:
			_dwBaudRate = B38400;
			break;
		case 57600:
			_dwBaudRate = B57600;
			break;
		case 115200:
			_dwBaudRate = B115200;
			break;
		case 230400:
			_dwBaudRate = B230400;
			break;
		case 460800:
			_dwBaudRate = B460800;
			break;
		case 500000:
			_dwBaudRate = B500000;
			break;
		case B576000:
			_dwBaudRate = B576000;
			break;
		case 921600:
			_dwBaudRate = B921600;
			break;
		case B1000000:
			_dwBaudRate = B1000000;
			break;
		case B1152000:
			_dwBaudRate = B1152000;
			break;
		case B1500000:
			_dwBaudRate = B1500000;
			break;
		case B2000000:
			_dwBaudRate = B2000000;
			break;
		case B2500000:
			_dwBaudRate = B2500000;
			break;
		case B3000000:
			_dwBaudRate = B3000000;
			break;
		case B3500000:
			_dwBaudRate = B3500000;
			break;
		case B4000000:
			_dwBaudRate = B4000000;
			break;
		default:
			_dwBaudRate = B115200;
			break;
	}

	// TODO: _dwBaudRate
	//	newtios.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;

	tcgetattr(_tty_fd, &oldtermios); /* save current port settings */

	bzero(&newtios, sizeof(newtios));

	//newtios.c_cflag |= _dwBaudRate | CRTSCTS | CS8 | CLOCAL | CREAD;
	newtios.c_cflag = oldtermios.c_cflag;
	newtios.c_cflag &= ~CBAUD;
	newtios.c_cflag |= _dwBaudRate;
	newtios.c_iflag = IGNPAR;
	newtios.c_oflag = 0;

	/* set input mode (non-canonical, no echo,...) */
	newtios.c_lflag = 0;

	newtios.c_cc[VTIME]    = 0;   /* inter-character timer unused */
	newtios.c_cc[VMIN]     = 1;   /* blocking read until 1 chars received */

	tcflush(_tty_fd, TCIFLUSH);
	tcsetattr(_tty_fd, TCSANOW, &newtios);

	pthread_barrier_init(&_barrier, 0, 2);

	// Initiate IRQ handler
	pthread_create(&_thread, NULL, &TTYIrqHandler, this);

	// sync startup
	pthread_barrier_wait(&_barrier);
}

void TTYUARTClass::end( void )
{
	char c;

	// Write to the fifo_fd_wr to signal termination to 'IRQ' thread
	_write(_pipe_tx_rx[__TTYUART_IDX_TX], &c, 1);

	// pthread_join to sync
	pthread_join(_thread, NULL);

	// destroy barrier
	pthread_barrier_destroy(&_barrier);

	(void)muxDeselectUart(_dwId);

	if (_tty_name && _console)
		_reattach_console();

	if (_tty_fd >= 0){
		close(_tty_fd);
		_tty_fd = -1;
	}

	if (_pipe_tx_rx[__TTYUART_IDX_TX] >= 0){
		close(_pipe_tx_rx[__TTYUART_IDX_TX]);
		_pipe_tx_rx[__TTYUART_IDX_TX] = -1;
	}

	if (_pipe_tx_rx[__TTYUART_IDX_RX] >= 0){
		close(_pipe_tx_rx[__TTYUART_IDX_RX]);
		_pipe_tx_rx[__TTYUART_IDX_RX] = -1;
	}
}

int TTYUARTClass::available( void )
{
	uint32_t ret;

	pthread_mutex_lock(&_mutex);
	ret = (uint32_t)(SERIAL_BUFFER_SIZE + _rx_buffer->_iHead - _rx_buffer->_iTail) % SERIAL_BUFFER_SIZE ;
	pthread_mutex_unlock(&_mutex);

	return ret;
}

int TTYUARTClass::peek( void )
{
	int ret;

	pthread_mutex_lock(&_mutex);
	if ( _rx_buffer->_iHead == _rx_buffer->_iTail )
		ret = -1;
	else
		ret = _rx_buffer->_aucBuffer[_rx_buffer->_iTail] ;
	pthread_mutex_unlock(&_mutex);

	return ret;
}

// Read a sigle byte from TTY device non-blocking - use poll() to determine RX availability
int TTYUARTClass::read( void )
{
	// if the head isn't ahead of the tail, we don't have any characters
	int ret;

	pthread_mutex_lock(&_mutex);
	if ( _rx_buffer->_iHead == _rx_buffer->_iTail ){
		ret = -1;
	}else{
		ret = _rx_buffer->_aucBuffer[_rx_buffer->_iTail] ;
		_rx_buffer->_iTail = (unsigned int)(_rx_buffer->_iTail + 1) % SERIAL_BUFFER_SIZE ;
	}
	pthread_mutex_unlock(&_mutex);

	return ret ;
}

void TTYUARTClass::flush( void )
{
// Wait for transmission to complete
//while ((_pUsart->US_CSR & US_CSR_TXRDY) != US_CSR_TXRDY)
//	;
}

// Write to TTY in blocking mode
size_t TTYUARTClass::write( const uint8_t c )
{
	return _write(_tty_fd, &c, 1);
}

bool TTYUARTClass::overflow( void )
{
	bool overflow;

	pthread_mutex_lock(&_mutex);
	overflow = _rx_buffer->overflow();
	pthread_mutex_unlock(&_mutex);

	return overflow;
}

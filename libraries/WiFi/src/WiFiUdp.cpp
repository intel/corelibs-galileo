/*
 *  Udp.cpp: Library to send/receive UDP packets with the Arduino wifi shield.
 *  This version only offers minimal wrapping of socket.c/socket.h
 *  Drop Udp.h/.cpp into the WiFi library directory at hardware/libraries/WiFi/
 *
 * MIT License:
 * Copyright (c) 2008 Bjoern Hartmann
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * bjoern@cs.stanford.edu 12/30/2008
 *
 * Modified by Denny Yang on 25 Sep 2013:
 *    - modified to support x86 arch
 */


#include <errno.h>		// -EINVAL, -ENODEV
#include <netdb.h>		// gethostbyname
#include <sys/poll.h>
#include <sys/types.h>		// connect
#include <sys/socket.h>		// connect

#include <sys/ioctl.h>		// ioctl

#include "WiFi.h"
#include "Udp.h"
#include "Dns.h"

#include <trace.h>

#define MY_TRACE_PREFIX "WiFiUDP"

/* Constructor */
WiFiUDP::WiFiUDP() : _sock(-1) {}

/* Start WiFiUDP socket, listening at local port PORT */
uint8_t WiFiUDP::begin(uint16_t port)
{

	_port = port;
	_remaining = 0;
	int ret = socket(AF_INET, SOCK_DGRAM, 0);
	if ( ret < 0){
		trace_error("%s fail init UDP socket!", __func__);
		return 0;
	}

	_sock = ret;
	if (listen() != 0){
		trace_error("unable to listen");
		close(_sock);
		return 0;
	}

	int on=1;
	ret = ioctl(_sock, FIONBIO, (char *)&on);
	if (ret < 0) {
		trace_error("ioctl() failed");
		close(_sock);
		return 0;
	}


	return 1;
}

int WiFiUDP::listen()
{
	bzero(&_sin,sizeof(_sin));
	_sin.sin_family = AF_INET;
	_sin.sin_addr.s_addr=htonl(INADDR_ANY);
	_sin.sin_port=htons(_port);

	return bind(_sock,(struct sockaddr *)&_sin,sizeof(_sin));
}

/* return number of bytes available in the current packet,
   will return zero if parsePacket hasn't been called yet */
int WiFiUDP::available() {
  	struct pollfd ufds;
	int ret = 0;
	extern int errno;
	int    timeout = 0;	// milliseconds

	if (_sock == -1)
		return 0;

	ufds.fd = _sock;
	ufds.events = POLLIN;
	ufds.revents = 0;

	ret = poll(&ufds, 1, timeout);
	if ( ret < 0 ){
		trace_error("%s error on poll errno %d", __func__, errno);
		return 0;
	}
	if( ret == 0)
		return 0;

	// only return available if bytes are present to be read
	if(ret > 0 && ufds.revents&POLLIN){
		int bytes = 0;
		ret = ioctl(_sock, FIONREAD, &bytes);
		if ( ret < 0){
				trace_error("ioctl fail on socket!");
				return 0;
		}
		if ( ret == 0 && bytes != 0){
			return bytes;
		}
	}
	return 0;
}

/* Release any resources being used by this WiFiUDP instance */
void WiFiUDP::stop()
{
	if (_sock == -1)
		return;

	close(_sock);
	_sock = -1;

	//WiFiClass::_server_port[_sock] = 0;
	WiFiClass::_server_port[0] = 0;
 	//_sock = MAX_SOCK_NUM;
 	//_sock = -1;
}

int WiFiUDP::beginPacket(const char *host, uint16_t port)
{
	// Look up the host first

	int ret = 0;
	extern int errno;
	struct hostent *hp;

	if (host == NULL || _sock == -1)
		return -EINVAL;

	hp = gethostbyname(host);
	_offset = 0;
	if (hp == NULL){
			trace_error("gethostbyname %s fail!", host);
			return -ENODEV;
	}
	memcpy(&_sin.sin_addr, hp->h_addr, sizeof(_sin.sin_addr));
	_sin.sin_port = htons(port); //PL : probably useful to have this...
	return 0;

}

int WiFiUDP::beginPacket(IPAddress ip, uint16_t port)
{
	_offset = 0 ;

	_sin.sin_addr.s_addr = ip._sin.sin_addr.s_addr;
	_sin.sin_family = AF_INET;
  	_sin.sin_port = htons(port);

  	return 0;
}

int WiFiUDP::endPacket()
{
	if ( _sock == -1 )
		return -1;
	trace_debug("%s called", __func__);
	return sendUDP();
}

int WiFiUDP::sendUDP()
{
	return sendto(_sock, _buffer, _offset, 0, (struct sockaddr*)&_sin, sizeof(_sin));
}

size_t WiFiUDP::write(uint8_t byte)
{
	return write(&byte, 1);
}

int WiFiUDP::bufferData(const uint8_t *buffer, size_t size)
{
	int written_bytes =  0;
	if  (UDP_TX_PACKET_MAX_SIZE - _offset < size) {
		written_bytes =  UDP_TX_PACKET_MAX_SIZE - _offset;
	} else {
		written_bytes = size;
	}
	memcpy(_buffer + _offset, buffer, written_bytes);
	_offset += written_bytes;
}

size_t WiFiUDP::write(const uint8_t *buffer, size_t size)
{
	uint16_t bytes_written = bufferData(buffer, size);
	return bytes_written;
}

int WiFiUDP::parsePacket()
{
	// discard any remaining bytes in the last packet
	flush();
	_remaining = 0;
	_remaining = available();
	return _remaining;
}

int WiFiUDP::read()
{
	uint8_t b;
	if ( recv(_sock, &b, 1,0) > 0 ) {
		return b;
	} else {
		// No data available
		return -1;
	}
}

int WiFiUDP::read(unsigned char* buffer, size_t len)
{

	if (_remaining > 0) {

		int got;

		if (_remaining <= len) {
			// data should fit in the buffer
			got = recvfrom(_sock, buffer, _remaining, 0, NULL, NULL);
		} else {
			// too much data for the buffer,
			// grab as much as will fit
			got = recvfrom(_sock, buffer, len, 0, NULL, NULL);
		}

		if (got > 0) {
			_remaining -= got;
			return got;
		}

	}

	// If we get here, there's no data available or recv failed
	return -1;

}

int WiFiUDP::peek()
{
	return -1;
}

void WiFiUDP::flush()
{
	// could this fail (loop endlessly) if _remaining > 0 and recv in read fails?
	// should only occur if recv fails after telling us the data is there, lets
	// hope the w5100 always behaves :)

	while (_remaining) {
		read();
	}
}


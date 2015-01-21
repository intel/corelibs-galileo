/*
EthernetClient.cpp  Ethernet client implementation for x86
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

// Standard system includes
#include <errno.h>			// -EINVAL, -ENODEV
#include <netdb.h>			// gethostbyname
#include <sys/ioctl.h>		// ioctl
#include <sys/poll.h>		// poll
#include <sys/types.h>		// connect
#include <sys/socket.h>		// connect

// Firmware includes specific to galileo
#include <trace.h>

#define MY_TRACE_PREFIX "EthernetClient"

// Arduino includes
extern "C" {
	#include "string.h"
}
#include "Arduino.h"
#include "Ethernet.h"
#include "EthernetClient.h"
#include "EthernetServer.h"
#include "Dns.h"

uint16_t EthernetClient::_srcport = 1024;

EthernetClient::EthernetClient() : _sock(-1)
{
    	// full client mode - not connected
	connect_true = false;
	_pCloseServer = NULL;
}

EthernetClient::EthernetClient(uint8_t sock) : _sock(sock)
{
	// being explicitely initialised by external logic to _sock = sock
	connect_true = true;
	_pCloseServer = NULL;
}

static int _connect(int _sock, struct sockaddr_in * psin, unsigned int len)
{
	extern int errno;
	int ret;

	trace_debug("%s doing connect() %d target ip %3d.%3d.%3d.%3d",
			__func__, _sock,
	  		(psin->sin_addr.s_addr&0xFF000000)>>24,
			(psin->sin_addr.s_addr&0x00FF0000)>>16,
			(psin->sin_addr.s_addr&0x0000FF00)>>16,
		    	(psin->sin_addr.s_addr&0x000000FF));

	ret = connect(_sock, (struct sockaddr*)psin, len);
	if (ret < 0){
		trace_error("%s connect err %d errno %d fail!", __func__, ret, errno);
		close(_sock);
		return ret;
	}
	return ret;
}

int EthernetClient::connect(const char* host, uint16_t port)
{
	// Look up the host first
	struct hostent *hp;
	int ret = 0;
	extern int errno;

	if (host == NULL || _sock != -1)
		return 0;

	_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (_sock < 0){
		trace_error("unable to open a TCP socket!");
		return 0;
	}

	hp = gethostbyname(host);
	if (hp == NULL){
			trace_error("gethostbyname %s fail!", host);
			return 0;
	}
	memcpy(&_sin.sin_addr, hp->h_addr, sizeof(_sin.sin_addr));
	_sin.sin_family = AF_INET;
	_sin.sin_port = htons(port);

	ret = _connect(_sock, &_sin, sizeof(_sin));
	if (ret < 0){
		trace_error("unable to connect to target host %s", host);
		return 0;
	}

	if ( ret == 0)
		connect_true = true;

	return 1;
}

int EthernetClient::connect(IPAddress ip, uint16_t port)
{
  // Look up the host first
  	int ret = 0;
  	extern int errno;
  	int on = 1;

  	if (_sock != -1)
  		return 0;

  	_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  	if (_sock < 0){
  		trace_error("unable to open a TCP socket!");
  		return 0;
  	}

	trace_debug("%s made socket %d target ip %3d.%3d.%3d.%3d",
		__func__, _sock,
  		(ip._sin.sin_addr.s_addr&0x000000FF),
		(ip._sin.sin_addr.s_addr&0x0000FF00)>>8,
		(ip._sin.sin_addr.s_addr&0x00FF0000)>>16,
		(ip._sin.sin_addr.s_addr&0xFF000000)>>24);

	_sin.sin_addr.s_addr = ip._sin.sin_addr.s_addr;
  	_sin.sin_family = AF_INET;
  	_sin.sin_port = htons(port);

  	ret = _connect(_sock, &_sin, sizeof(_sin));
  	if (ret < 0){
  		trace_error("unable to connect to target ip %3d.%3d.%3d.%3d",
  			(_sin.sin_addr.s_addr&0x000000FF),
		    (_sin.sin_addr.s_addr&0x0000FF00)>>8,
		    (_sin.sin_addr.s_addr&0x00FF0000)>>16,
		    (_sin.sin_addr.s_addr&0xFF000000)>>24);
		return 0;
	}

	// Set non-blocking - required for poll() on this socket to work and for child sockets
	ret = ioctl(_sock, FIONBIO, (char *)&on);
	if (ret < 0) {
		trace_error("ioctl() failed");
		close(_sock);
		return 0;
	}
	if ( ret == 0)
		connect_true = true;

	return 1;
}

size_t EthernetClient::write(uint8_t b) {
  return write(&b, 1);
}

size_t EthernetClient::write(const uint8_t *buf, size_t size)
{
	extern int errno;

 	if (_sock < 0) {
		trace_error("%s cannot write to closed socket", __func__);
		return 0;
	}

	if (send(_sock, buf, size, MSG_NOSIGNAL) < 0) {
		trace_error("%s unable to send on sock %d errno %d", __func__, _sock, errno);
		_sock = -1;
                stop();
    		return 0;
	}

	return size;
}

int EthernetClient::available()
{
	struct pollfd ufds;
	int ret = 0;
	extern int errno;
	int    timeout = 5000;	// milliseconds

	if (_sock == -1){
		return 0;
	}

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
		    	if (_inactive_counter)
		    		*_inactive_counter = 0;
			return 1;
		}else{
			if (_inactive_counter == NULL)
				return 0;

			// Increment inactivity counter
			(*_inactive_counter)++;

			// counter exceeded nuke connection
			if ( *_inactive_counter >= CLIENT_MAX_INACTIVITY_RETRIES){
				trace_info("Inactivity counter on socket reached %d - closing connection", CLIENT_MAX_INACTIVITY_RETRIES);
				stop();
			}
		}
	}

	return 0;
}

int EthernetClient::read()
{
	uint8_t b;
	if ( recv(_sock, &b, 1, 0) > 0 ) {
		return b;
	} else {
		// No data available
		return -1;
	}
}

int EthernetClient::read(uint8_t *buf, size_t size)
{
	return recv(_sock, buf, size, 0);
}

int EthernetClient::peek()
{
        uint8_t b;
       
        if ( ! available() ) return -1;
        if ( recv(_sock, &b, 1, MSG_PEEK) < 1 ) return -1;
       
        return b;

}

void EthernetClient::flush()
{
	while (available())
		read();
}

void EthernetClient::stop()
{
	if (_sock < 0)
		return;

	connect_true = false;
	if (_inactive_counter != NULL)
		*_inactive_counter = 0;
	if(_sock != -1){
		close(_sock);
		_sock = -1;

		// Sketches do a Ethernet client = server.available() - which means a copy constructor is used
		// Hence any server which populated a pclients[idx] - won't actually see the socket closed - since
		// his client was copied - not given a pointer to.... pcleint[] in EthernetServer.cpp _sock != -1 at this point
		// make it so !
		if(_pCloseServer != NULL){
		    	_pCloseServer->closeNotify(this->id);
		}
	}
}

uint8_t EthernetClient::connected()
{
	return connect_true == true;
}

uint8_t EthernetClient::status()
{
	return _sock == -1;
}

// the next function allows us to use the client returned by
// EthernetServer::available() as the condition in an if-statement.

EthernetClient::operator bool() {
  return _sock != -1;
}

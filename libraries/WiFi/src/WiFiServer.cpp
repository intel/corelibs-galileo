/*
WiFiServer.cpp added WiFi server class implementation for x86 
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
#include <assert.h>
#include <errno.h>			// -EINVAL, -ENODEV
#include <netdb.h>			// gethostbyname
#include <sys/poll.h>
#include <sys/types.h>		// connect
#include <sys/socket.h>		// connect

#include <trace.h>

#define MY_TRACE_PREFIX "WiFiServer"

extern "C" {
#include "string.h"
}

#include "WiFi.h"
#include "WiFiClient.h"
#include "WiFiServer.h"

WiFiServer::WiFiServer(uint16_t port)
{
	_port = port;
	_sock = -1;
	_init_ok = false;
	pclients = new WiFiClient[MAX_SOCK_NUM];
	_pcli_inactivity_counter = new int[MAX_SOCK_NUM];
	_scansock = 0;

	if (pclients == NULL){
		trace_error("%s OOM condition", __func__);
		return;
	}
	for(int sock = 0; sock < MAX_SOCK_NUM; sock++){
		_pcli_inactivity_counter[sock] = 0;
		pclients[sock].id = sock;
	}
}

WiFiServer::~WiFiServer()
{
    if (pclients != NULL){
    	delete [] pclients;
    	pclients = NULL;
    }

    if(_pcli_inactivity_counter != NULL){
    	delete [] _pcli_inactivity_counter;
    	_pcli_inactivity_counter = NULL;
    }
}

void WiFiServer::begin()
{
	int ret;
	extern int errno;

	_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (_sock < 0){
		trace_error("unable to open a TCP socket!");
		return;
	}

	_sin.sin_addr.s_addr = INADDR_ANY;
	_sin.sin_family = AF_INET;
  	_sin.sin_port = htons(_port);

	ret = bind(_sock, (struct sockaddr*)&_sin, sizeof(_sin));
	if ( ret < 0){
		trace_error("%s unable to bind port %d", __func__, _port);
		return;
	}

	//socket(sock, SnMR::TCP, _port, 0);
	ret = listen(_sock, MAX_SOCK_NUM);
	if ( ret < 0){
		trace_error("%s unable to listen on port %d", __func__, _port);
		return;
	}

	for(int sock = 0; sock < MAX_SOCK_NUM; sock++)
		WiFiClass::_server_port[sock] = _port;

	// mark as available
	_init_ok = true;
}

static int _accept(int sock, struct sockaddr * psin, socklen_t * psize)
{
	return accept(sock, psin, psize);
}

void WiFiServer::accept()
{
	struct pollfd ufds;
	int ret = 0, size_val, success = 0;
	extern int errno;

	if (_sock == -1)
		return;

	ufds.fd = _sock;
	ufds.events = POLLIN;
	ufds.revents = 0;

	ret = poll(&ufds, 1, 0);
	if ( ret < 0 ){
		trace_error("%s error on poll errno %d", __func__, errno);
		_sock = -1;
		close(_sock);
		return;
	}

	if(ufds.revents&POLLIN){
		//trace_debug("%s in activity on socket %d - calling accept()", __func__, _sock);
		size_val = sizeof(_cli_sin);
		ret = _accept(_sock, (struct sockaddr*)&_cli_sin, (socklen_t*)&size_val);

		if ( ret < 0){
			close(_sock);
			_sock = -1;
			trace_error("%s Fail to accept() sock %d port %d", __func__, _sock, _port);
			return;
		}

		for(int sock = 0; sock < MAX_SOCK_NUM && success == 0; sock++){
			if (pclients[sock]._sock == -1){
				pclients[sock]._sock = ret;
				pclients[sock]._pCloseServer = this;
				pclients[sock].connect_true = true;
				pclients[sock]._inactive_counter = &_pcli_inactivity_counter[sock];
				success = 1;
			}
		}
		if ( success == 0 ){
			trace_error("%s detect connect event - unable to allocate socket slot !", __func__);
		}
	}
}

WiFiClient WiFiServer::available()
{
	accept();

	// Scan for next connection - meaning don't return the same one each time
	for(int sock = 0; sock < MAX_SOCK_NUM; sock++){
		if (pclients[sock]._sock != -1 && sock != _scansock){
			//trace_debug("Returning socket entity %d socket %d", sock, pclients[sock]._sock);
			_scansock = sock;
			return pclients[sock];
		}
	}

	// scan for any connection
	for(int sock = 0; sock < MAX_SOCK_NUM; sock++){
		if (pclients[sock]._sock != -1){
			//trace_debug("Returning socket entity %d socket %d", sock, pclients[sock]._sock);
			_scansock = sock;
			return pclients[sock];
		}
	}

	//trace_debug("%s no client to return", __func__);
	_scansock = 0;
	return pclients[0];

}

size_t WiFiServer::write(uint8_t b)
{
	return write(&b, 1);
}

size_t WiFiServer::write(const uint8_t *buffer, size_t size)
{
	size_t n = 0;

	//accept();

	// This routine writes the given data to all current clients
	for(int sock = 0; sock < MAX_SOCK_NUM; sock++){
		if (pclients[sock]._sock != -1){
		    	n += pclients[sock].write(buffer, size);
		}
	}
	return n;
}

void WiFiServer::closeNotify(int idx)
{
    if (idx < MAX_SOCK_NUM)
    	pclients[idx]._sock = -1;
}

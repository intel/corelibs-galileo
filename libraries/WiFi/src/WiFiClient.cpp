// Modified by Manoel Ramon on 29 May 2014:
//    - Ethernet/WiFi Clients - peek() method do not work
//
// Modified by Denny Yang on 25 Sep 2013:
//    - making to compile in x86 for Intel Galileo Arduino IDE

// Standard system includes
#include <errno.h>			// -EINVAL, -ENODEV
#include <netdb.h>			// gethostbyname
#include <sys/ioctl.h>		// ioctl
#include <sys/poll.h>		// poll
#include <sys/types.h>		// connect
#include <sys/socket.h>		// connect

// Firmware includes specific to galileo
#include <trace.h>

#define MY_TRACE_PREFIX "WiFiClient"

// Arduino includes
extern "C" {
	#include "string.h"
}
#include "Arduino.h"
#include "WiFi.h"
#include "WiFiClient.h"
#include "WiFiServer.h"
#include "Dns.h"

uint16_t WiFiClient::_srcport = 1024;

WiFiClient::WiFiClient() : _sock(-1)
{
    	// full client mode - not connected
	connect_true = false;
	_pCloseServer = NULL;
}

WiFiClient::WiFiClient(uint8_t sock) : _sock(sock)
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

int WiFiClient::connect(const char* host, uint16_t port)
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

int WiFiClient::connect(IPAddress ip, uint16_t port)
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

size_t WiFiClient::write(uint8_t b) {
  return write(&b, 1);
}

size_t WiFiClient::write(const uint8_t *buf, size_t size)
{
	extern int errno;

 	if (_sock < 0) {
		trace_error("%s cannot write to closed socket", __func__);
		return 0;
	}

	if (send(_sock, buf, size, 0) < 0) {
		trace_error("%s unable to send on sock %d errno %d", __func__, _sock, errno);
		_sock = -1;
    		return 0;
	}

	return size;
}

int WiFiClient::available()
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

int WiFiClient::read()
{
	uint8_t b;
	if ( recv(_sock, &b, 1, 0) > 0 ) {
		return b;
	} else {
		// No data available
		return -1;
	}
}

int WiFiClient::read(uint8_t *buf, size_t size)
{
	return recv(_sock, buf, size, 0);
}

int WiFiClient::peek()
{
        uint8_t b;
       
        if ( ! available() ) return -1;
        if ( recv(_sock, &b, 1, MSG_PEEK) < 1 ) return -1;
       
        return b;

}

void WiFiClient::flush()
{
	while (available())
		read();
}

void WiFiClient::stop()
{
	if (_sock < 0)
		return;

	connect_true = false;
	if (_inactive_counter != NULL)
		*_inactive_counter = 0;
	if(_sock != -1){
		close(_sock);
		_sock = -1;

		// Sketches do a WiFi client = server.available() - which means a copy constructor is used
		// Hence any server which populated a pclients[idx] - won't actually see the socket closed - since
		// his client was copied - not given a pointer to.... pcleint[] in WiFiServer.cpp _sock != -1 at this point
		// make it so !
		if(_pCloseServer != NULL){
		    	_pCloseServer->closeNotify(this->id);
		}
	}
}

uint8_t WiFiClient::connected()
{
	return connect_true == true;
}

uint8_t WiFiClient::status()
{
	return _sock == -1;
}

// the next function allows us to use the client returned by
// WiFiServer::available() as the condition in an if-statement.

WiFiClient::operator bool() {
  return _sock != -1;
}

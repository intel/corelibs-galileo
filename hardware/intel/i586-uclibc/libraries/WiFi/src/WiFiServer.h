// Modified by Denny Yang on 25 Sep 2013:
//    - making to compile in x86 for Intel Galileo Arduino IDE

#ifndef wifiserver_h
#define wifiserver_h

#include "Server.h"
#include "WiFi.h"
#include "WiFiClient.h"

class WiFiClient;

class WiFiServer :
public Server {
private:
	uint16_t _port;
	void accept();
	int _sock;
	int _scansock;
	struct sockaddr_in _sin;
	struct sockaddr_in _cli_sin;
	bool _init_ok;
	WiFiClient *pclients;
	int *_pcli_inactivity_counter;
public:
	WiFiServer(uint16_t);
	~WiFiServer();
	WiFiClient available();
	virtual void begin();
	virtual size_t write(uint8_t);
	virtual size_t write(const uint8_t *buf, size_t size);
	void closeNotify(int idx);
	using Print::write;
};

#endif

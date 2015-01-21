#ifndef wificlient_h
#define wificlient_h
#include "Arduino.h"
#include "Print.h"
#include "Client.h"
#include "IPAddress.h"
#include "WiFiServer.h"

class WiFiClient : public Client {

#define CLIENT_MAX_INACTIVITY_RETRIES 100000
public:
	WiFiClient();
	WiFiClient(uint8_t sock);

	uint8_t status();
	virtual int connect(IPAddress ip, uint16_t port);
	virtual int connect(const char *host, uint16_t port);
	virtual size_t write(uint8_t);
	virtual size_t write(const uint8_t *buf, size_t size);
	virtual int available();
	virtual int read();
	virtual int read(uint8_t *buf, size_t size);
	virtual int peek();
	virtual void flush();
	virtual void stop();
	virtual uint8_t connected();
	virtual operator bool();

	friend class WiFiServer;

	using Print::write;

	int _sock;
	int *_inactive_counter;
	class WiFiServer * _pCloseServer;
	bool connect_true;
	int id;
private:
	static uint16_t _srcport;
	struct sockaddr_in _sin;
};

#endif

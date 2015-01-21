#ifndef ethernetserver_h
#define ethernetserver_h

#include "Server.h"
#include "Ethernet.h"
#include "EthernetClient.h"

class EthernetClient;

class EthernetServer :
public Server {
private:
	uint16_t _port;
	void accept();
	int _sock;
	int _scansock;
	struct sockaddr_in _sin;
	struct sockaddr_in _cli_sin;
	bool _init_ok;
	EthernetClient *pclients;
	int *_pcli_inactivity_counter;
public:
	EthernetServer(uint16_t);
	~EthernetServer();
	EthernetClient available();
	virtual void begin();
	virtual size_t write(uint8_t);
	virtual size_t write(const uint8_t *buf, size_t size);
	void closeNotify(int idx);
	using Print::write;
};

#endif

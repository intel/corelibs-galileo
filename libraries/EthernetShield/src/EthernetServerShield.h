#ifndef ethernetserverShield_h
#define ethernetserverShield_h

#include "Server.h"

class EthernetClientShield;

class EthernetServerShield : 
public Server {
private:
  uint16_t _port;
  void accept();
public:
  EthernetServerShield(uint16_t);
  EthernetClientShield available();
  virtual void begin();
  virtual size_t write(uint8_t);
  virtual size_t write(const uint8_t *buf, size_t size);
  using Print::write;
};

#endif

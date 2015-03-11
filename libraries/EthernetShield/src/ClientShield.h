#ifndef clientShield_h
#define clientShield_h
#include "Print.h"
#include "Stream.h"
#include "IPAddressShield.h"

class ClientShield : public Stream {

public:
  virtual int connect(IPAddressShield ip, uint16_t port) =0;
  virtual int connect(const char *host, uint16_t port) =0;
  virtual size_t write(uint8_t) =0;
  virtual size_t write(const uint8_t *buf, size_t size) =0;
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int read(uint8_t *buf, size_t size) = 0;
  virtual int peek() = 0;
  virtual void flush() = 0;
  virtual void stop() = 0;
  virtual uint8_t connected() = 0;
  virtual operator bool() = 0;
protected:
  uint8_t* rawIPAddress(IPAddressShield& addr) { return addr.raw_address(); };
};

#endif

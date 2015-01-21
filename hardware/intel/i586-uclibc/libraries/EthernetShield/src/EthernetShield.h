// Modified for Intel Galileo - dino.tinitigan@intel.com

#ifndef ethernetShield_h
#define ethernetShield_h

#include <inttypes.h>
//#include "w5100.h"
#include "IPAddressShield.h"
#include "EthernetClientShield.h"
#include "EthernetServerShield.h"
#include "DhcpShield.h"

#define MAX_SOCK_NUM 4

class EthernetShieldClass {
private:
  IPAddressShield _dnsServerAddress;
  DhcpShieldClass* _dhcp;
public:
  static uint8_t _state[MAX_SOCK_NUM];
  static uint16_t _server_port[MAX_SOCK_NUM];
  // Initialise the Ethernet shield to use the provided MAC address and gain the rest of the
  // configuration through DHCP.
  // Returns 0 if the DHCP configuration failed, and 1 if it succeeded
  int begin(uint8_t *mac_address);
  void begin(uint8_t *mac_address, IPAddressShield local_ip);
  void begin(uint8_t *mac_address, IPAddressShield local_ip, IPAddressShield dns_server);
  void begin(uint8_t *mac_address, IPAddressShield local_ip, IPAddressShield dns_server, IPAddressShield gateway);
  void begin(uint8_t *mac_address, IPAddressShield local_ip, IPAddressShield dns_server, IPAddressShield gateway, IPAddressShield subnet);
  int maintain();

  IPAddressShield localIP();
  IPAddressShield subnetMask();
  IPAddressShield gatewayIP();
  IPAddressShield dnsServerIP();

  friend class EthernetClientShield;
  friend class EthernetServerShield;
};

extern EthernetShieldClass EthernetShield;

#endif

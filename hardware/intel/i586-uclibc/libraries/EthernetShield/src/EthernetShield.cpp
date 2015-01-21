// Modified for Intel Galileo - dino.tinitigan@intel.com

#include "utility/w5100.h"
#include "EthernetShield.h"
#include "DhcpShield.h"

// XXX: don't make assumptions about the value of MAX_SOCK_NUM.
uint8_t EthernetShieldClass::_state[MAX_SOCK_NUM] = { 
  0, 0, 0, 0 };
uint16_t EthernetShieldClass::_server_port[MAX_SOCK_NUM] = { 
  0, 0, 0, 0 };

int EthernetShieldClass::begin(uint8_t *mac_address)
{
  static DhcpShieldClass s_dhcp;
  _dhcp = &s_dhcp;


  // Initialise the basic info
  W5100.init();
  W5100.setMACAddress(mac_address);
  W5100.setIPAddress(IPAddressShield(0,0,0,0).raw_address());

  // Now try to get our config info from a DHCP server
  int ret = _dhcp->beginWithDHCP(mac_address);
  if(ret == 1)
  {
    // We've successfully found a DHCP server and got our configuration info, so set things
    // accordingly
    W5100.setIPAddress(_dhcp->getLocalIp().raw_address());
    W5100.setGatewayIp(_dhcp->getGatewayIp().raw_address());
    W5100.setSubnetMask(_dhcp->getSubnetMask().raw_address());
    _dnsServerAddress = _dhcp->getDnsServerIp();
  }

  return ret;
}

void EthernetShieldClass::begin(uint8_t *mac_address, IPAddressShield local_ip)
{
  // Assume the DNS server will be the machine on the same network as the local IP
  // but with last octet being '1'
  IPAddressShield dns_server = local_ip;
  dns_server[3] = 1;
  begin(mac_address, local_ip, dns_server);
}

void EthernetShieldClass::begin(uint8_t *mac_address, IPAddressShield local_ip, IPAddressShield dns_server)
{
  // Assume the gateway will be the machine on the same network as the local IP
  // but with last octet being '1'
  IPAddressShield gateway = local_ip;
  gateway[3] = 1;
  begin(mac_address, local_ip, dns_server, gateway);
}

void EthernetShieldClass::begin(uint8_t *mac_address, IPAddressShield local_ip, IPAddressShield dns_server, IPAddressShield gateway)
{
  IPAddressShield subnet(255, 255, 255, 0);
  begin(mac_address, local_ip, dns_server, gateway, subnet);
}

void EthernetShieldClass::begin(uint8_t *mac, IPAddressShield local_ip, IPAddressShield dns_server, IPAddressShield gateway, IPAddressShield subnet)
{
  W5100.init();
  W5100.setMACAddress(mac);
  W5100.setIPAddress(local_ip.raw_address());
  W5100.setGatewayIp(gateway.raw_address());
  W5100.setSubnetMask(subnet.raw_address());
  _dnsServerAddress = dns_server;
}

int EthernetShieldClass::maintain(){
  int rc = DHCP_CHECK_NONE;
  if(_dhcp != NULL){
    //we have a pointer to dhcp, use it
    rc = _dhcp->checkLease();
    switch ( rc ){
      case DHCP_CHECK_NONE:
        //nothing done
        break;
      case DHCP_CHECK_RENEW_OK:
      case DHCP_CHECK_REBIND_OK:
        //we might have got a new IP.
        W5100.setIPAddress(_dhcp->getLocalIp().raw_address());
        W5100.setGatewayIp(_dhcp->getGatewayIp().raw_address());
        W5100.setSubnetMask(_dhcp->getSubnetMask().raw_address());
        _dnsServerAddress = _dhcp->getDnsServerIp();
        break;
      default:
        //this is actually a error, it will retry though
        break;
    }
  }
  return rc;
}

IPAddressShield EthernetShieldClass::localIP()
{
  IPAddressShield ret;
  W5100.getIPAddress(ret.raw_address());
  return ret;
}

IPAddressShield EthernetShieldClass::subnetMask()
{
  IPAddressShield ret;
  W5100.getSubnetMask(ret.raw_address());
  return ret;
}

IPAddressShield EthernetShieldClass::gatewayIP()
{
  IPAddressShield ret;
  W5100.getGatewayIp(ret.raw_address());
  return ret;
}

IPAddressShield EthernetShieldClass::dnsServerIP()
{
  return _dnsServerAddress;
}

EthernetShieldClass EthernetShield;

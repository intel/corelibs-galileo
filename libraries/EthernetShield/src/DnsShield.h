// Arduino DNS client for WizNet5100-based Ethernet shield
// (c) Copyright 2009-2010 MCQN Ltd.
// Released under Apache License, version 2.0
// Modified for Intel Galileo - dino.tinitigan@intel.com

#ifndef DNSClientShield_h
#define DNSClientShield_h

#include <EthernetUdpShield.h>

class DNSClientShield
{
public:
    // ctor
    void begin(const IPAddressShield& aDNSServer);

    /** Convert a numeric IP address string into a four-byte IP address.
        @param aIPAddrString IP address to convert
        @param aResult IPAddress structure to store the returned IP address
        @result 1 if aIPAddrString was successfully converted to an IP address,
                else error code
    */
    int inet_aton(const char *aIPAddrString, IPAddressShield& aResult);

    /** Resolve the given hostname to an IP address.
        @param aHostname Name to be resolved
        @param aResult IPAddress structure to store the returned IP address
        @result 1 if aIPAddrString was successfully converted to an IP address,
                else error code
    */
    int getHostByName(const char* aHostname, IPAddressShield& aResult);

protected:
    uint16_t BuildRequest(const char* aName);
    uint16_t ProcessResponse(uint16_t aTimeout, IPAddressShield& aAddress);

    IPAddressShield iDNSServer;
    uint16_t iRequestId;
    EthernetUDPShield iUdp;
};

#endif

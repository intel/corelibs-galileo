/*
  IPAddressShield.h - Base class that provides IPAddress
  Copyright (c) 2011 Adrian McEwen.  All right reserved.

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
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified for Intel Galileo - dino.tinitigan@intel.com
*/

#ifndef IPAddressShield_h
#define IPAddressShield_h

#include <stdint.h>
#include <Printable.h>

// A class to make it easier to handle and pass around IP addresses

class IPAddressShield : public Printable {
private:
    union {
	uint8_t bytes[4];  // IPv4 address
	uint32_t dword;
    } _address;

    // Access the raw byte array containing the address.  Because this returns a pointer
    // to the internal structure rather than a copy of the address this function should only
    // be used when you know that the usage of the returned uint8_t* will be transient and not
    // stored.
    uint8_t* raw_address() { return _address.bytes; };

public:
    // Constructors
    IPAddressShield();
    IPAddressShield(uint8_t first_octet, uint8_t second_octet, uint8_t third_octet, uint8_t fourth_octet);
    IPAddressShield(uint32_t address);
    IPAddressShield(const uint8_t *address);

    // Overloaded cast operator to allow IPAddress objects to be used where a pointer
    // to a four-byte uint8_t array is expected
    operator uint32_t() const { return _address.dword; };
    bool operator==(const IPAddressShield& addr) const { return _address.dword == addr._address.dword; };
    bool operator==(const uint8_t* addr) const;

    // Overloaded index operator to allow getting and setting individual octets of the address
    uint8_t operator[](int index) const { return _address.bytes[index]; };
    uint8_t& operator[](int index) { return _address.bytes[index]; };

    // Overloaded copy operators to allow initialisation of IPAddressShield objects from other types
    IPAddressShield& operator=(const uint8_t *address);
    IPAddressShield& operator=(uint32_t address);

    virtual size_t printTo(Print& p) const;

    friend class EthernetShieldClass;
    friend class UDPShield;
    friend class ClientShield;
    friend class Server;
    friend class DhcpShieldClass;
    friend class DNSClientShield;
};

const IPAddressShield INADDR_NONE(0,0,0,0);


#endif

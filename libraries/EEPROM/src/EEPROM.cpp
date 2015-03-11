/*
  EEPROM.cpp - EEPROM library
  Copyright (c) 2006 David A. Mellis.  All right reserved.
  Copyright (c) 2013 Intel Corporation.  All right reserved.

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
*/

/******************************************************************************
 * Includes
 ******************************************************************************/

#include <sys/stat.h> 
#include <fcntl.h>

#include "Arduino.h"
#include "trace.h"
#include "EEPROM.h"

#define MY_TRACE_PREFIX "EEPROM"

/******************************************************************************
 * Definitions
 ******************************************************************************/

/******************************************************************************
 * Constructors
 ******************************************************************************/

/******************************************************************************
 * User API
 ******************************************************************************/

uint8_t EEPROMClass::read(int address)
{
	int fd = -1;
	uint8_t value = 0;
	
	fd = open(LINUX_EEPROM, O_RDONLY);
	if (fd < 0) {
		trace_error("Failed to open EEPROM device for reading\n");
		goto exit;
	}
	if (lseek(fd, address, SEEK_SET) < 0) {
		trace_error("Failed to set EEPROM read address %d\n", address);
		goto exit;
	}
	if (::read(fd, &value, 1) < 0) {
		trace_error("Failed to read from EEPROM address %d\n", address);
		goto exit;
	}

exit:
	if (fd >= 0)
		close(fd);

	return value;
}

void EEPROMClass::write(int address, uint8_t value)
{
	int fd = -1;
	
	fd = open(LINUX_EEPROM, O_WRONLY);
	if (fd < 0) {
		trace_error("Failed to open EEPROM device for writing\n");
		goto exit;
	}
	if (lseek(fd, address, SEEK_SET) < 0) {
		trace_error("Failed to set EEPROM write address %d\n", address);
		goto exit;
	}
	if (::write(fd, &value, 1) < 0) {
		trace_error("Failed to write to EEPROM address %d\n", address);
		goto exit;
	}

exit:
	if (fd >= 0)
		close(fd);

	return;
}

EEPROMClass EEPROM;

/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * Copyright (c) 2013 Intel Corporation
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include "Arduino.h"

#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV32 0x06
//#define SPI_CLOCK_DIV64 0x07
#define SPI_CLOCK_DIV1 0x09

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

#define SPI_CLOCK_100K 100000
#define SPI_CLOCK_125K 125000
#define SPI_CLOCK_250K 250000
#define SPI_CLOCK_500K 500000
#define SPI_CLOCK_1M 1000000
#define SPI_CLOCK_2M 2000000
#define SPI_CLOCK_4M 4000000
#define SPI_CLOCK_8M 8000000
#define SPI_CLOCK_10M 10000000
#define SPI_CLOCK_12M 12500000
#define SPI_CLOCK_13M 13500000

class SPIClass {
public:
  SPIClass();  // Constructor

  byte transfer(byte _data);
  void transferBuffer(const uint8_t *, uint8_t *, uint32_t);

  // SPI Configuration methods

  void attachInterrupt();
  void detachInterrupt(); // Default

  void begin(); // Default
  void end();

  void setBitOrder(uint8_t);
  void setDataMode(uint8_t);
  void setClockDivider(uint8_t);
  void setClockSpeed(uint32_t);

private:
  static int fd;
  uint8_t mode;
  uint8_t bitOrder;
  uint8_t clkDiv;
};

extern SPIClass SPI;

#endif

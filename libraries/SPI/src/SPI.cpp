/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * Copyright (c) 2013 Intel Corporation

 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 *
 * Modified by Dino Tinitigan 20 May 2014:
 *       -fix compilation failure with Intel IDE
 *
 */

#include <sys/stat.h> 
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>

#include "Arduino.h"
#include "trace.h"
#include "SPI.h"

#define MY_TRACE_PREFIX "SPI"

/* For Arduino, this is assumed to be 4MHz, using SPI_CLOCK_DEV4
 * Sticking to this for backward compatibility */
#define SPI_CLK_DEFAULT_HZ 4000000

#define SPI_SS_GPIO_PIN   10

SPIClass SPI;
int SPIClass::fd = 0;

/* Constructor - establish defaults */
SPIClass::SPIClass()
{
	/* reflect Arduino's default behaviour where possible */
	this->mode = SPI_MODE0;
	this->bitOrder = MSBFIRST;
	this->clkDiv = SPI_CLOCK_DIV4;
}

void SPIClass::begin()
{
	/* Set the pin mux, for the SCK, MOSI and MISO pins ONLY
	 *
	 * Leave the SS pin in GPIO mode (the application will control it)
	 * but set it's direction to output and initially high
	 */
	pinMode(SPI_SS_GPIO_PIN, OUTPUT);
	digitalWrite(SPI_SS_GPIO_PIN, HIGH);
	muxSelectSpi(1);
	
	if(fd <= 0)
	{
		this->fd = open(LINUX_SPIDEV, O_RDWR);
		if (this->fd < 0) {
			trace_error("Failed to open SPI device\n");
			return;
		}
	}
	/* Load default/last configuration */
	this->setClockDivider(this->clkDiv);
	this->setBitOrder(this->bitOrder);
	this->setDataMode(this->mode);
}


void SPIClass::end()
 {
	if (this->fd >= 0)
		close(this->fd);
	this->fd = 0;
}

void SPIClass::setBitOrder(uint8_t bitOrder)
{
	uint8_t lsbFirst = 0;
	
	if (bitOrder == LSBFIRST) {
		lsbFirst = 1;
	}
	
	if (ioctl (this->fd, SPI_IOC_WR_LSB_FIRST, &lsbFirst) < 0) {
		trace_error("Failed to set SPI bit justification\n");
		return;
	}

	this->bitOrder = bitOrder;
}

void SPIClass::setDataMode(uint8_t mode)
{
	uint8_t linuxSpiMode = 0;

	switch(mode) {
	case SPI_MODE0:
		linuxSpiMode = SPI_MODE_0;
		break;
	case SPI_MODE1:
		linuxSpiMode = SPI_MODE_1;
		break;
	case SPI_MODE2:
		linuxSpiMode = SPI_MODE_2;
		break;
	case SPI_MODE3:
		linuxSpiMode = SPI_MODE_3;
		break;
	default:
		trace_error("Invalid SPI mode specified\n");
		return;
	}

	if (ioctl (this->fd, SPI_IOC_WR_MODE, &linuxSpiMode) < 0) {
		trace_error("Failed to set SPI mode\n");
		return;
	}

	this->mode = mode;
}

void SPIClass::setClockDivider(uint8_t clkDiv)
{
	uint32_t maxSpeedHz = SPI_CLK_DEFAULT_HZ;

	/* Adjust the clock speed relative to the default divider of 4 */
	switch(clkDiv)
	{
	case SPI_CLOCK_DIV2:
		maxSpeedHz = SPI_CLK_DEFAULT_HZ << 1;
		break;
	case SPI_CLOCK_DIV4:
		/* Do nothing */
		break;
	case SPI_CLOCK_DIV8:
		maxSpeedHz = SPI_CLK_DEFAULT_HZ >> 1;
		break;
	case SPI_CLOCK_DIV16:
		maxSpeedHz = SPI_CLK_DEFAULT_HZ >> 2;
		break;
	case SPI_CLOCK_DIV32:
		maxSpeedHz = SPI_CLK_DEFAULT_HZ >> 3;
		break;
	case SPI_CLOCK_DIV64:
		maxSpeedHz = SPI_CLK_DEFAULT_HZ >> 4;
		break;
	case SPI_CLOCK_DIV128:
		maxSpeedHz = SPI_CLK_DEFAULT_HZ >> 5;
		break;
	default:
		trace_error("Invalid SPI mode specified\n");
		return;
	}

	if (ioctl (this->fd, SPI_IOC_WR_MAX_SPEED_HZ, &maxSpeedHz) < 0) {
		trace_error("Failed to set SPI clock speed\n");
		return;
	}

	this->clkDiv = clkDiv;
}

void SPIClass::setClockSpeed(uint32_t clkSpeed)
{
	if(clkSpeed > 16000000)
	{
		clkSpeed = 16000000;
	}
	uint32_t maxSpeedHz = clkSpeed;
	if (ioctl (this->fd, SPI_IOC_WR_MAX_SPEED_HZ, &maxSpeedHz) < 0) {
		trace_error("Failed to set SPI clock speed\n");
		return;
	}
	this->clkDiv = 0x0;
}

uint8_t SPIClass::transfer(uint8_t txData)
{
	uint8_t rxData = 0xFF;
	struct spi_ioc_transfer msg;

	memset(&msg, 0, sizeof(msg));
	
	msg.tx_buf = (__u64) &txData;
	msg.rx_buf = (__u64) &rxData;
	msg.len = sizeof(uint8_t);
	
	if (ioctl (this->fd, SPI_IOC_MESSAGE(1), &msg) < 0)
		trace_error("Failed to execute SPI transfer\n");

	return rxData;
}

void SPIClass::transferBuffer(const uint8_t *txData,
			      uint8_t *rxData,
			      uint32_t len)
{
	struct spi_ioc_transfer msg;

	memset(&msg, 0, sizeof(msg));
	
	msg.tx_buf = (__u64) txData;
	msg.rx_buf = (__u64) rxData;
	msg.len = len;
	
	if (ioctl (this->fd, SPI_IOC_MESSAGE(1), &msg) < 0)
		trace_error("Failed to execute SPI transfer\n");
}

void SPIClass::attachInterrupt() {
	trace_error("SPI slave mode is not currently supported\n");
}

void SPIClass::detachInterrupt() {
	/* Do nothing */
}


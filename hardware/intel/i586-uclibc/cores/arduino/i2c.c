/*
 * i2c.c - the i2c library
 *
 * Copyright (c) 2013 Parav Nagarsheth
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA  02111-1307  USA
 *
 * Modified by Denny Yang on 25 Sept 2013:
 *     -Changed I2C_SLAVE to I2C_SLAVE_FORCE
 *
 */

#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <trace.h>
#include "i2c-dev.h"

#define MAX_BUF 100

#define MY_TRACE_PREFIX "i2c"

struct i2c_msg i2c_buf[2];
unsigned int i2c_buf_count = 0;

int i2c_getadapter(uint32_t i2c_bus_address)
{
	/* Single adapter supported for now.  */
#if 0
	char buf[MAX_BUF];
	DIR *d;
	struct dirent *dir;
	int adapter_nr = -1;
	snprintf(buf, sizeof(buf), "/sys/devices/ocp.2/%x.i2c",
							i2c_bus_address);
	d = opendir(buf);
	if (d) {
		while ((dir = readdir(d)) != NULL) {
			if (!strncmp("i2c", dir->d_name, 3)) {
				sscanf(dir->d_name, "i2c-%d", &adapter_nr);
				closedir(d);
				return adapter_nr;
			}
		}
		closedir(d);
	}
	return -1;
#endif
	return 0;
}

int i2c_openadapter(uint8_t i2c_adapter_nr)
{
	char buf[MAX_BUF];
	int i2c_fd;
	snprintf(buf, sizeof(buf), "/dev/i2c-%d", i2c_adapter_nr);
	if ((i2c_fd = open(buf, O_RDWR)) < 1) {
		perror("Failed to open adapter");
		return -1;
	}
	return i2c_fd;
}

int i2c_setslave(int i2c_fd, uint8_t addr)
{
	if (ioctl(i2c_fd, I2C_SLAVE_FORCE, addr) < 0) {
		fprintf(stderr, "Failed to set slave address %d:", addr);		
		perror("");
		return -1;	
	}
	return 0;
}

int i2c_writebyte(int i2c_fd, uint8_t byte)
{
	if (i2c_smbus_write_byte(i2c_fd, byte ) < 0) {
		perror("Failed to write byte to I2C slave");
		return -1;	
	}
	return 0;
}

int i2c_writebytes(int i2c_fd, uint8_t *bytes, uint8_t length)
{
	if(i2c_smbus_write_i2c_block_data(i2c_fd, bytes[0], length-1, bytes+1) 
									< 0) {
		perror("Failed to write bytes to I2C slave");
		return -1;
	}
	return 0;
}

int i2c_readbyte(int i2c_fd)
{
	int byte;
	if ((byte = i2c_smbus_read_byte(i2c_fd)) < 0) {
		// perror("Failed to read byte from I2C slave");
		return -1;
	}
	return byte;
}

int i2c_readbytes(int i2c_fd, uint8_t *buf, int length)
{
	if (read(i2c_fd, buf, length) == length)
		return length;
	else
		return -1;
}

int i2c_readwrite(int i2c_fd)
{
	struct i2c_rdwr_ioctl_data packets;
	packets.msgs = i2c_buf;
	packets.nmsgs = i2c_buf_count;
	if (ioctl(i2c_fd, I2C_RDWR, &packets) < 0) {
		perror("Unable to send data");
		i2c_buf_count = 0;
		return -1;
	}
	i2c_buf_count = 0;
	return 0;
}

int i2c_add_to_buf(uint8_t addr, uint8_t rw, uint8_t *value, int length)
{
	if(i2c_buf_count < 2) {
		i2c_buf[i2c_buf_count].addr = addr;
		i2c_buf[i2c_buf_count].flags = rw ? I2C_M_RD : 0;
		i2c_buf[i2c_buf_count].len = length;
		i2c_buf[i2c_buf_count].buf = (char *)value;
		return ++i2c_buf_count;
	} else
		return -1;
}

/*
 * i2c.h
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
 */

#ifndef i2c_h
#define i2c_h

#include <inttypes.h>
#ifdef __cplusplus
extern "C"{
#endif

int i2c_getadapter(uint32_t i2c_bus_address);
int i2c_openadapter(uint8_t i2c_adapter_nr);
int i2c_setslave(int i2c_fd, uint8_t addr);
int i2c_writebyte(int i2c_fd, uint8_t byte);
int i2c_writebytes(int i2c_fd, uint8_t *bytes, uint8_t length);
int i2c_readbyte(int i2c_fd);
int i2c_readbytes(int i2c_fd, uint8_t *buf, int length);
int i2c_readwrite(int i2c_fd);
int i2c_add_to_buf(uint8_t addr, uint8_t rw, uint8_t *value, int length);

#ifdef __cplusplus
}
#endif
#endif /* i2c_h */

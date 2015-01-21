/*
sysfs.h
Copyright (c) 2014 Intel Corporation
Copyright (c) 2013 Anuj Deshpande
This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.
This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General
Public License along with this library; if not, write to the
Free Software Foundation, Inc., 59 Temple Place, Suite 330,
Boston, MA 02111-1307 USA
*/

#ifndef __SYSFS_H__
#define __SYSFS_H__

#ifdef __cplusplus
extern "C" {
#endif

int sysfsPwmExport(unsigned pwm, int *handle_enable, int *handle_duty);
int sysfsPwmEnable(int handle_enable, int handle_duty, unsigned int ulValue);
int sysfsPwmDisable(int handle_enable);
int sysfsPwmDisableGen1(int handle_enable);

int sysfsAdcExport(unsigned adc, int *handle);
uint32_t sysfsAdcGet(int ihandle);

int sysfsGpioSet(int ihandle, unsigned int value);
int sysfsGpioGet(int ihandle);
int sysfsGpioExport(unsigned int gpio, char *path, unsigned int max_path);
int sysfsGpioDirection(unsigned int gpio, int output, int outval);
int sysfsGpioEdgeConfig(unsigned int gpio, int mode);
int sysfsGpioLevelConfig(unsigned int gpio, int mode);
int sysfsGpioSetDrive(unsigned int gpio, unsigned int mode);

#ifdef __cplusplus
}
#endif


#endif /* __SYSFS_H__ */

/*
pulseIn.cpp pulseIn library for Intel Galileo Gen 2
Copyright (C) 2014 Intel Corporation

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
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

/* 
 * Author:
 * Dino Tinitigan <dino.tinitigan@intel.com>
 *
 * Port of pulseIn Library for Galileo and Galileo 2.0
 *
*/

#include <Arduino.h>

#define READ(PIN) (fastMode?(fastGpioDigitalRead(fastPin)):(digitalRead(pin)))

unsigned long pulseIn(uint8_t pin, uint8_t value, unsigned long timeout)
{
  unsigned long time_a;
  unsigned long time_b;
  bool fastMode = true;
  bool timeoutFlag = false;
  bool timeBFlag = false;
  int fastPin;
  int highValue = 0;
  int targetValue = 0;

  time_a = micros();

  if(PLATFORM_NAME == "GalileoGen2")
  {
    switch(pin)
    {
    case 0:
      fastPin = GPIO_FAST_ID_QUARK_SC(0x08);
      highValue = 0x08;
      break;
    case 1:
      fastPin = GPIO_FAST_ID_QUARK_SC(0x10);
      highValue = 0x10;
      break;
    case 2:
      fastPin = GPIO_FAST_ID_QUARK_SC(0x20);
      highValue = 0x20;
      break;
    case 3:
      fastPin = GPIO_FAST_ID_QUARK_SC(0x40);
      highValue = 0x40;
      break;
    case 4:
      fastPin = GPIO_FAST_ID_QUARK_NC_RW(0x10);
      highValue = 0x10;
      break;
    case 5:
      fastPin = GPIO_FAST_ID_QUARK_NC_CW(0x01);
      highValue = 0x01;
      break;
    case 6:
      fastPin = GPIO_FAST_ID_QUARK_NC_CW(0x02);
      highValue = 0x02;
      break;
    case 9:
      fastPin = GPIO_FAST_ID_QUARK_NC_RW(0x04);
      highValue = 0x04;
      break;
    case 10:
      fastPin = GPIO_FAST_ID_QUARK_SC(0x04);
      highValue = 0x04;
      break;
    case 11:
      fastPin = GPIO_FAST_ID_QUARK_NC_RW(0x08);
      highValue = 0x08;
      break;
    case 12:
      fastPin = GPIO_FAST_ID_QUARK_SC(0x80);
      highValue = 0x80;
      break;
    case 13:
      fastPin = GPIO_FAST_ID_QUARK_NC_RW(0x20);
      highValue = 0x20;
      break;
    default:
      highValue = 1;
      fastMode = false;
      break;
    }
  }
  else
  {
    switch(pin)
    {
    case 2:
      fastPin = GPIO_FAST_ID_QUARK_SC(0x40);
      highValue = 0x40;
      break;
    case 3:
      fastPin = GPIO_FAST_ID_QUARK_SC(0x80);
      highValue = 0x80;
      break;
    default:
      highValue = 1;
      fastMode = false;
      break;
    }
  }

  if(value>0)
  {
    targetValue = highValue;
  }
  timeoutFlag = false;

  //wait for previous pulse to end or timeout
  while(READ(PIN) == targetValue)
  {
    time_b = micros();
    timeBFlag = true;
    if(time_b >= time_a)
    {
      if((time_b - time_a)>timeout)
      {
        timeoutFlag = true;
        break;
      }
    }
    else
    {//micros() overflow
      unsigned long time_c = (0xFFFFFFFF - time_a) + time_b;
      if(time_c > timeout)
      {
        timeoutFlag = true;
        break;
      }
    }
  }

  //wait for pin to go to target value or timeout
  while((READ(PIN) != targetValue) && !timeoutFlag)
  {
    time_b = micros();
    timeBFlag = true;
    if(time_b >= time_a)
    {
      if((time_b - time_a)>timeout)
      {
        timeoutFlag = true;
        break;
      }
    }
    else
    {//micros() overflow
      unsigned long time_c = (0xFFFFFFFF - time_a) + time_b;
      if(time_c > timeout)
      {
        timeoutFlag = true;
        break;
      }
    }
  }

  //determine pulse length
  if((READ(PIN) == targetValue) && !timeoutFlag)
  {
    if(!timeBFlag)
    {
      time_b = micros();
    }
    time_a = time_b;
    while(READ(PIN) == targetValue)
    {
    }
    time_b = micros();
    if(time_b > time_a)
    {
      return (time_b - time_a - 1);
    }
    else
    {//micros() overflow
      unsigned long time_c = (0xFFFFFFFF - time_a) + time_b - 1;
      return time_c;
    }
  }
  else
  {
    return 0;
  }
}

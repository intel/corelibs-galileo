/*
UtilMisc.cpp utility functions for Intel Galileo
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
 * 
 * Author: Manoel Ramon <manoel.ramon@intel.com>
 *
 *
*/

#include <Arduino.h>

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, int val, uint8_t bits, uint8_t del)
{
   uint8_t i;
   for (i = 0; i < bits; i++)  {
      if (bitOrder == LSBFIRST)
      {
         digitalWrite(dataPin, !!(val & (1 << i)));
      }
      else
      {    
         digitalWrite(dataPin, !!(val & (1 << ((bits - 1 - i)))));
         digitalWrite(clockPin, HIGH);
         delayMicroseconds(del);
         digitalWrite(clockPin, LOW);            
	 }
   }
}


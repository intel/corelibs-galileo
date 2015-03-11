/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified by Daniel Hugo on 11 Mar 2014:
    - Implement c++ random(...) overrides, wrap .h in __cplusplus, tab2spaces
*/

#ifndef _WIRING_MATH_
#define _WIRING_MATH_

#ifndef ARDUINO_LINUX
extern "C" {
  #include <stdint.h>
}
#endif

#ifdef word
#undef word
#endif

#ifdef __cplusplus
long int random( long ) ;
long int random( long, long ) ;
#endif

void randomSeed( uint32_t dwSeed ) ;

long map( long, long, long, long, long ) ;

#ifdef __cplusplus
uint16_t makeWord( uint16_t w) ;
uint16_t makeWord( uint8_t h, uint8_t l ) ;
#define word(...) makeWord(__VA_ARGS__)
#endif

#endif /* _WIRING_MATH_ */

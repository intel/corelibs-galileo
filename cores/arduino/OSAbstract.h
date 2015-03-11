/* OSAbstract.h set of main functions for userpace
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

#ifndef __OS_ABSTRACT_H__
#define __OS_ABSTRACT_H__

void init(int argc, char * argv[]);
void setup(void);
void loop(void);

#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)

// No meaning in user-space
#define noInterrupts()
#define interrupts()

#define word(x, y)	((unsigned long)x>>8 | (unsigned long)y <<8)

#endif /* __OS_ABSTRACT_H__ */

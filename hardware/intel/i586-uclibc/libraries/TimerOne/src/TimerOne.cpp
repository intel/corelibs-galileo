/*
 *  Interrupt and PWM utilities for 16 bit Timer1 on ATmega168/328
 *  Original code by Jesse Tane for http://labs.ideo.com August 2008
 *  Modified March 2009 by Jérôme Despatis and Jesse Tane for ATmega328 support
 *  Modified June 2009 by Michael Polli and Jesse Tane to fix a bug in setPeriod() which caused the timer to stop
 *  Modified June 2011 by Lex Talionis to add a function to read the timer
 *  Modified Oct 2011 by Andrew Richards to avoid certain problems:
 *  - Add (long) assignments and casts to TimerOne::read() to ensure calculations involving tmp, ICR1 and TCNT1 aren't truncated
 *  - Ensure 16 bit registers accesses are atomic - run with interrupts disabled when accessing
 *  - Remove global enable of interrupts (sei())- could be running within an interrupt routine)
 *  - Disable interrupts whilst TCTN1 == 0.  Datasheet vague on this, but experiment shows that overflow interrupt
 *    flag gets set whilst TCNT1 == 0, resulting in a phantom interrupt.  Could just set to 1, but gets inaccurate
 *    at very short durations
 *  - startBottom() added to start counter at 0 and handle all interrupt enabling.
 *  - start() amended to enable interrupts
 *  - restart() amended to point at startBottom()
 * Modiied 7:26 PM Sunday, October 09, 2011 by Lex Talionis
 *  - renamed start() to resume() to reflect it's actual role
 *  - renamed startBottom() to start(). This breaks some old code that expects start to continue counting where it left off
 *
 * Modified and ported to x86. 22/09/2013 - Bryan O'Donoghue Intel
 *
 *  This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  See Google Code project http://code.google.com/p/arduino-timerone/ for latest
 */
#ifndef TIMERONE_cpp
#define TIMERONE_cpp

#include "TimerOne.h"

TimerOne Timer1;              // preinstatiate

void TimerOne::initialize(long microseconds)
{
	(void)setMinimumTimerFreq(microseconds);
}


void TimerOne::setPeriod(long microseconds)		// AR modified for atomic access
{
	(void)setMinimumTimerFreq(microseconds);
}

void TimerOne::setPwmDuty(char pin, int duty)
{
	// PWM - TBD - BOD
	unsigned long dutyCycle = pwmPeriod;
	#if 0
		dutyCycle *= duty;
		dutyCycle >>= 10;

		oldSREG = SREG;
		cli();
		if(pin == 1 || pin == 9)       OCR1A = dutyCycle;
		else if(pin == 2 || pin == 10) OCR1B = dutyCycle;
		SREG = oldSREG;
	#endif
}

void TimerOne::pwm(char pin, int duty, long microseconds)  // expects duty cycle to be 10 bit (1024)
{
	// PWM - TBD - BOD
	#if 0
		if(microseconds > 0) setPeriod(microseconds);
		if(pin == 1 || pin == 9) {
			DDRB |= _BV(PORTB1);                                   // sets data direction register for pwm output pin
			TCCR1A |= _BV(COM1A1);                                 // activates the output pin
		}else if(pin == 2 || pin == 10) {
			DDRB |= _BV(PORTB2);
			TCCR1A |= _BV(COM1B1);
		}
		setPwmDuty(pin, duty);
		resume();			// Lex - make sure the clock is running.  We don't want to restart the count, in case we are starting the second WGM
						// and the first one is in the middle of a cycle
	#endif
}

void TimerOne::disablePwm(char pin)
{
	#if 0
		if(pin == 1 || pin == 9)       TCCR1A &= ~_BV(COM1A1);   // clear the bit that enables pwm on PB1
		else if(pin == 2 || pin == 10) TCCR1A &= ~_BV(COM1B1);   // clear the bit that enables pwm on PB2
	#endif
}

void TimerOne::attachInterrupt(void (*isr)(), long microseconds)
{
	attachTimerInterrupt(isr, microseconds);
}

void TimerOne::detachInterrupt()
{
	#if 0
		TIMSK1 &= ~_BV(TOIE1);                                   // clears the timer overflow interrupt enable bit
	#endif
}

void TimerOne::resume()				// AR suggested
{
	// BOD - TBD
	#if 0
		TCCR1B |= clockSelectBits;
	#endif
}

void TimerOne::restart()		// Depricated - Public interface to start at zero - Lex 10/9/2011
{
	// BOD - TBD
	#if 0
		start();
	#endif
}

void TimerOne::start()	// AR addition, renamed by Lex to reflect it's actual role
{
	unsigned int tcnt1;

	// BOD - TBD
	#if 0
		TIMSK1 &= ~_BV(TOIE1);        // AR added
		GTCCR |= _BV(PSRSYNC);   		// AR added - reset prescaler (NB: shared with all 16 bit timers);

		oldSREG = SREG;				// AR - save status register
		cli();						// AR - Disable interrupts
		TCNT1 = 0;
		SREG = oldSREG;          		// AR - Restore status register

		do {	// Nothing -- wait until timer moved on from zero - otherwise get a phantom interrupt
			oldSREG = SREG;
			cli();
			tcnt1 = TCNT1;
			SREG = oldSREG;
		} while (tcnt1==0);

		//  TIFR1 = 0xff;              		// AR - Clear interrupt flags
	//  TIMSK1 = _BV(TOIE1);              // sets the timer overflow interrupt enable bit
	#endif
}

void TimerOne::stop()
{
	// BOD - TBD
	#if 0
		TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));          // clears all clock selects bits
	#endif
}

unsigned long TimerOne::read()		//returns the value of the timer in microseconds
{									//rember! phase and freq correct mode counts up to then down again
  	unsigned long tmp;				// AR amended to hold more than 65536 (could be nearly double this)
  	unsigned int tcnt1;				// AR added
	// BOD - TBD
#if 0
	oldSREG= SREG;
  	cli();
  	tmp=TCNT1;
	SREG = oldSREG;

	char scale=0;
	switch (clockSelectBits)
	{
	case 1:// no prescalse
		scale=0;
		break;
	case 2:// x8 prescale
		scale=3;
		break;
	case 3:// x64
		scale=6;
		break;
	case 4:// x256
		scale=8;
		break;
	case 5:// x1024
		scale=10;
		break;
	}

	do {	// Nothing -- max delay here is ~1023 cycles.  AR modified
		oldSREG = SREG;
		cli();
		tcnt1 = TCNT1;
		SREG = oldSREG;
	} while (tcnt1==tmp); //if the timer has not ticked yet

	//if we are counting down add the top value to how far we have counted down
	tmp = (  (tcnt1>tmp) ? (tmp) : (long)(ICR1-tcnt1)+(long)ICR1  );		// AR amended to add casts and reuse previous TCNT1
	return ((tmp*1000L)/(F_CPU /1000L))<<scale;
#endif
	return 0;
}

#endif
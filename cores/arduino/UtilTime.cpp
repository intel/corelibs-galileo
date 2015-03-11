/*
UtilTime.cpp provides time functions
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

#include <Arduino.h>
#include <fcntl.h>
#include <unistd.h>
#include <limits.h>
#include <sys/sysinfo.h>

#include <UtilTime.h>
#include <trace.h>

static uint64_t tsc_init = 0;
static float clocks_per_ns = 0;
static float cpufreq = 0;

#define MY_TRACE_PREFIX "time"

#if 0
unsigned long micros2( void )
{

  struct timespec t;
  t.tv_sec = t.tv_nsec = 0;

  /* Considering the system does not suspend CLOCK_REALTIME is welcome.
     However, if in the future our system suspend, we need to replace
     CLOCK_REALTIME by CLOCK_BOOTTIME and apply the proper patches in
     the kernel. */

  clock_gettime(CLOCK_REALTIME, &t);
  return (unsigned long)(t.tv_sec)*1000000L + t.tv_nsec / 1000L ;

}
#endif

static inline uint64_t rdtsc(void)
{
    uint32_t lo, hi;
    uint64_t returnVal;
    /* We cannot use "=A", since this would use %rax on x86_64 */
    __asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
    returnVal = hi;
    returnVal <<= 32;
    returnVal |= lo;

    return returnVal;
}

void delay(unsigned long ms)
{
    usleep(ms * 1000);
}

void delayMicroseconds(unsigned int us)
{
    usleep( us );
    return;

}

unsigned long micros( void )
{
    uint64_t tsc_cur = rdtsc(), diff = 0, divisor = 0;

    /*
    This function returns a 32-bit value representing the microseconds
    since the sketch started running on IA-32 Arduino boards.

    overflow should not be a problem, though for
    correctness it should be accounted for.
    A 64 bit counter at 400mhz will run for about
    1500 years before it overflows. the timestamp
    counter on IA32 is a 64-bit counter that ticks at
    the CPU clock rate, starting at 0.

    However, we have to deal with a loss of numerical representation
    when doing math between 64-bit unsigned long longs and 32 bit floats,
    then returning a 32 bit integer.

    Microseconds representation at 32 bits will recycle about every 4294 seconds at 400mhz.
    */

    divisor = (cpufreq );
    diff = tsc_cur - tsc_init;

    return (unsigned long) (diff / divisor);
}

unsigned long millis( void )
{
    /* similar to the micros() function, it returns ms since sketch start up time.
     The underlying counter is a 64 bit value, but the representation of millis
     as unsigned 32-bits means it recycles in ~ 1190 hours.*/

    uint64_t tsc_cur = rdtsc(), diff = 0, divisor = 0;
    divisor = (cpufreq * 1000);
    diff = tsc_cur - tsc_init;

    return (unsigned long) ( (diff / divisor) );

}



/* TSC snapshot */
int timeInit(void)
{
    int cpufreq_fd, ret;
    char buf[0x400];
    char * str = 0, * str2 = 0;
    char * mhz_str = "cpu MHz\t\t: ";

    /* Grab initial TSC snapshot */
    tsc_init = rdtsc();

    cpufreq_fd = open("/proc/cpuinfo", O_RDONLY);
    if( cpufreq_fd < 0){
        fprintf(stderr, "unable to open /proc/cpuinfo\n");
        return -1;
    }
    memset(buf, 0x00, sizeof(buf));
    ret = read(cpufreq_fd, buf, sizeof(buf));
    if ( ret < 0 ){
        fprintf(stderr, "unable to read cpuinfo !\n");
        close(cpufreq_fd);
        return -1;
    }
    close(cpufreq_fd);
    str = strstr(buf, mhz_str);
    if (!str){
        fprintf(stderr, "Buffer %s does not contain CPU frequency info !\n", buf);
        return -1;
    }

    str += strlen(mhz_str);
    str2 = str;

    while(str2 < buf  + sizeof(buf)-1 && *str2 != '\n'){
        str2++;
    }
    if(str2 == buf + sizeof(buf-1) && *str2 !='\n'){
        fprintf(stderr, "malformed cpufreq string %s\n", str);
        return -1;
    }
    *str2 = '\0';
    cpufreq = atof(str);


    printf("cpufrequency is %f mhz\n", cpufreq);

    /* Calculate nanoseconds per clock */
    clocks_per_ns = 1000/cpufreq;

    printf("nanoseconds per clock %f\n", clocks_per_ns);
}

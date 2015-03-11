/*
Tone.cpp port Tone library for Intel Galileo
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
 * Author: Dino Tinitigan <dino.tinitigan@intel.com>
 *
 * Port of Tone Library for Galileo
 *
*/

#include <pthread.h>
#include <Arduino.h>

struct tone_thread_param
{
  int tonePin;
  unsigned int frequency;
  long duration;
  int threadID;
};

long delayAmount;
long loopTime;
unsigned long time1;
unsigned long time2;
unsigned long toneDuration = 0;
int threadControl[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
pthread_mutex_t pinMutex[14] = {PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER};

void *toneHandler(void *ptr);
void toneNonBlocking(uint8_t _pin, unsigned int frequency, unsigned long duration);
void tone(int _pin, unsigned int frequency, unsigned long duration);
void noTone(uint8_t _pin);

void toneNonBlocking(uint8_t _pin, unsigned int frequency, unsigned long duration)
{
  pthread_mutex_lock(&pinMutex[_pin]);
  int tempThreadControl = ++threadControl[_pin];
  pthread_mutex_unlock(&pinMutex[_pin]);

  pthread_t toneThread;

  struct tone_thread_param *params;
  params=(tone_thread_param *)malloc(sizeof(tone_thread_param));
  params->tonePin = _pin;
  params->frequency = frequency;
  params->duration = duration;
  params->threadID = tempThreadControl;
  int iret1 = pthread_create( &toneThread, NULL, toneHandler, (void*) params);
  pthread_detach(toneThread);
}

void tone(uint8_t _pin, unsigned int frequency, unsigned long duration)
{
  if(duration > 0)
  {
    pinMode(_pin, OUTPUT_FAST);
    if(frequency > 0)
    {
      delayAmount = (long)(1000000/frequency)/2 - 1;
      loopTime = (long)((duration*1000)/(delayAmount*2));
      for (int x=0;x<loopTime;x++)
      {
        time1 = micros();
        time2 = time1;
        fastDigitalWrite(_pin,HIGH);
        while(((long)(time2 - time1)) <= delayAmount)
        {
          time2 = micros();
        }
        time1 = micros();
        fastDigitalWrite(_pin,LOW);
        while(((long)(time2 - time1)) <= delayAmount)
        {
          time2 = micros();
        }
      }
    }
    else
    {
      //no tone generated, basically a pause or rest note
      time1 = micros();
      time2 = time1;
      delayAmount = duration*1000;
      while(((long)(time2 - time1)) <= delayAmount)
      {
        time2 = micros();
      }
    }
  }
  else
  {
    //tone(pin, frequency) is called with two parameters only or duration is set 0
    //generate a constant tone.
    //since this generates a constant tone, it needs to be non-blocking else we would be stuck in an infinite loop
    pthread_mutex_lock(&pinMutex[_pin]);
    int tempThreadControl = ++threadControl[_pin];
    pthread_mutex_unlock(&pinMutex[_pin]);

    pthread_t toneThread;
    struct tone_thread_param *params;

    params=(tone_thread_param *)malloc(sizeof(tone_thread_param));
    params->tonePin = _pin;
    params->frequency = frequency;
    params->duration = duration;
    params->threadID = tempThreadControl;
    int iret1 = pthread_create( &toneThread, NULL, toneHandler, (void*) params);
    pthread_detach(toneThread);
  }
}

void noTone(uint8_t _pin)
{
  pthread_mutex_lock(&pinMutex[_pin]);
  ++threadControl[_pin];  //this would effectively kill other running tone threads assigned to that pin
  pthread_mutex_unlock(&pinMutex[_pin]);
}

void *toneHandler(void *arg)
{
  tone_thread_param *param;
  param = (tone_thread_param*)arg;

  int tPin = param->tonePin;
  pinMode(tPin, OUTPUT_FAST);
  long delayAmount;
  long loopTime;
  unsigned long time1;
  unsigned long time2;
  int currentThreadControlID = param->threadID; //when threadControl[tPin] is modified by another thread it will no longer be equal to currentThreadControlID which exits the loop below allowing it to die

  if((param->frequency) > 0)
  {
    delayAmount = (long)(1000000/(param->frequency))/2 - 1;
    loopTime = (long)((param->duration*1000)/(delayAmount*2));

    if((param->duration) > 0)
    {
      //implementing this way because delayMicroseconds is blocking which makes the generated tone not accurate enough
      for (int x=0;x<loopTime;x++)
      {
        if(currentThreadControlID != threadControl[tPin])
        {
          break;
        }
        time1 = micros();
        time2 = time1;
        fastDigitalWrite(tPin,HIGH);
        while(((long)(time2 - time1)) <= delayAmount)
        {
          time2 = micros();
        }
        time1 = micros();
        fastDigitalWrite(tPin,LOW);
        while(((long)(time2 - time1)) <= delayAmount)
        {
          time2 = micros();
        }
      }
    }
    else
    {
      //infinite loop
      while(true)
      {
        if(currentThreadControlID != threadControl[tPin])
        {
          break;
        }
        time1 = micros();
        time2 = time1;
        fastDigitalWrite(tPin,HIGH);
        while(((long)(time2 - time1)) <= delayAmount)
        {
          time2 = micros();
        }
        time1 = micros();
        fastDigitalWrite(tPin,LOW);
        while(((long)(time2 - time1)) <= delayAmount)
        {
          time2 = micros();
        }
      }
    }
  }
  else
  {
    time1 = micros();
    time2 = time1;
    delayAmount = (param->duration)*1000;
    while(((long)(time2 - time1)) <= delayAmount)
    {
      time2 = micros();
      if(currentThreadControlID != threadControl[tPin])
      {
        break;
      }
    }
  }
}

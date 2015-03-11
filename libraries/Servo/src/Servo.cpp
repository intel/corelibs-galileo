/*
Servo.cpp library class implementation in x86 to support servo motors 
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

#include "Servo.h"
#include <trace.h>
uint8_t Servo::counter = 0;   // init the counter here.. static...


Servo::Servo()
{
  if (counter < MAX_NUMBER_OF_SERVOS) {
    this->index = counter++;                    // assign a servo index to this instance
    lastByteInDuty = -1;
  } else {
    this->index = INVALID_SERVO;  // too many servos
  }

}

#ifdef SERVO_PWM_WITH_SYSFS         // PCA9685 IC
   void Servo::setFreqInSysFs(int freqInNanoSec)
   {
       // using sysfs
       FILE *fp = NULL;
       int ret = 0;
       char export_value[16] = "";
       char fs_path[50] = LINUX_PWM_EXPORT;

       // setting the right frequency
       memset(fs_path, 0x00, sizeof(fs_path));
       snprintf(fs_path, sizeof(fs_path), LINUX_PWM_PERIOD_FMT);

       if (NULL == (fp = fopen(fs_path, "ab"))) {
           trace_error("can't open handle to %s", fs_path);
           return ;
       }
       rewind(fp);

       memset(export_value, 0x0, sizeof(export_value));
       snprintf(export_value, sizeof(export_value), "%u", freqInNanoSec);
       fwrite(&export_value, sizeof(char), sizeof(export_value), fp);

       fclose(fp);
   }

   void Servo::setDuty(int value_duty)
   {

      char value[16] = "";
      int ret;

      value_duty *= ERROR_FACTOR;

      handle_duty = pin2pwmhandle_duty(pin);

      memset(value, 0x0, sizeof(value));
      snprintf(value, sizeof(value), "%u", (unsigned)value_duty);
      lseek(handle_duty, 0, SEEK_SET);
      ret = pointer_write(handle_duty, &value, sizeof(value));
      if (sizeof(value) != ret) {
           trace_error("can't write to duty_cycle\n");
      }
   }

   void Servo::enablePin(bool enable)
   {
       int ret;
       enable = 1;
       //set the mux and open the handles
       analogWrite(pin, 0);

       handle_enable = pin2pwmhandle_enable(pin);

       lseek(handle_enable, 0, SEEK_SET);
       ret = pointer_write(handle_enable, (const void*)&enable, sizeof(enable));
       if (sizeof(enable) != ret) {
           trace_error("can't write to enable\n");
       }

   }

   void Servo::disablePin()
   {
       this->enablePin(false);
   }
#endif
#ifdef SERVO_PWM_WITH_I2C
void Servo::set48hz()
{
    if (this->is188hz)
    {
      // only changes if is different freq
       this->is188hz = false;

       // cypress - I2C
       writeMicroseconds(DEFAULT_PULSE_WIDTH);
    }	
	currentPWMis188Hz = this->is188hz;
}

void Servo::set188hz()
{
    if (!this->is188hz)
    {
      // only changes if is different freq
      this->is188hz = true;
       // cypress - I2C
      writeMicroseconds(DEFAULT_PULSE_WIDTH);
    }
	currentPWMis188Hz = this->is188hz;
}
#endif
uint8_t Servo::attach(int16_t pin, bool force48hz)
{

  return attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, force48hz);
}

uint8_t Servo::attach(int pin, int min, int max, bool force48hz)
{
  // need to validate the pin
  uint8_t list_index = 0;
  bool    is_valid_pin = false;

  // let's check the boundaries
  if (min < MIN_PULSE_WIDTH) min = MIN_PULSE_WIDTH;
  if (max > MAX_PULSE_WIDTH) max = MAX_PULSE_WIDTH;

  trace_debug("******* pin:%d min:%d max:%d\n", pin, min, max);

  for (list_index = 0; list_index < sizeof(pinData)/sizeof(servoPinData_t); list_index++)
  {
      if (pinData[list_index].pin == pin)
      {
          is_valid_pin = true;
          break;
      }
  }

  if (!is_valid_pin)
  {
      trace_error("invalid pin");
      return INVALID_SERVO;
  }

  if (this->index < MAX_NUMBER_OF_SERVOS) {

    // set as active
    pinData[list_index].isActive = true;


    this->pin = pin;
    this->min = min;
    this->max = max;
    this->isAttached = true;

#ifdef SERVO_PWM_WITH_I2C
    if (!force48hz) 
      this->is188hz = true;
    else
      this->is188hz = false;
	  
	currentPWMis188Hz = this->is188hz;
    pinMode(pin, OUTPUT);
    analogWrite(pin, 1);
    writeMicroseconds(DEFAULT_PULSE_WIDTH);
#else
    // sysfs
    this->is188hz = false;
    setFreqInSysFs(PWM_50Hz); // aroung 50hz
    this->enablePin(pin);
    this->setDuty( DEFAULT_PULSE_WIDTH*1000);
#endif
  }

  trace_debug("\nattached ok on pin:%d min:%d max:%d\n",pin, this->min, this->max);

  return this->index;
}


/* The "train" requested by servo motors is
   the minimal of 20ms. Our PWM allow us to have
   41.7Hz that means 23.98 ms.
   However the cypress does not offer a good angle
   resolution on this frequency and the user
   has option to operates the servos in 188Hz */

void Servo::prepare_pin(uint8_t pin)
{

#ifdef SERVO_PWM_WITH_I2C          // CYPRESS IC
  extern TwoWire Wire;

    Wire.begin();

    // let's use this function only to select the bit port
    // the datasheet is a little confusing regading this set

    analogWrite(pin, 1);

    // Select programmable PWM CLK source to 367.7 Hz
    Wire.beginTransmission(CYPRESS_I2C_ADDRESS);
    Wire.write(0x29);
    Wire.write(0x04);
    Wire.endTransmission();


    // Rising edge register
    Wire.beginTransmission(CYPRESS_I2C_ADDRESS);
    Wire.write(0x2a);
    Wire.write(0xff);
    Wire.endTransmission();


    // Set divider to get 47.4Hz freq.
    Wire.beginTransmission(CYPRESS_I2C_ADDRESS);
    Wire.write(0x2C);

	// force the servo assume the last PWM settings
	this->is188hz = currentPWMis188Hz;
    if (this->is188hz)
       Wire.write(0x02);
    else
       Wire.write(0x09);

    Wire.endTransmission();
#else  // SERVO_PWM_WITH_SYSFS
    enablePin(pin);
#endif

}

#ifdef SERVO_PWM_WITH_I2C
byte Servo::transform_cypress_duty_cycle_byte(int microsecs)
{

  /* the max division of 23ms (100% duty cycle) is
     255 units (bytes 0 to 255 according regs 0x2B

     So, the maximum units for 2ms is 22.17

     The principle it is used for 188Hz thar correspond
     a period of 5.319ms.

     So, the max byte available for different frequency
     is in maximum time of 2ms or 2.4ms in 8 bits resolution
     is:

         = max_duty/1000 * (1/freq)/255
         = max_duty*255/(1000/freq)

     */

  int freq =  (this->is188hz) ? 188:43.4;
  int max_byte = MAX_PULSE_WIDTH*255*freq/1000000L;

  if(this->min==1000) trace_debug("maxbyte:%d\n", max_byte);

  byte b_duty = map(microsecs, 0, MAX_PULSE_WIDTH, 0, max_byte);

  return b_duty;
}
#endif

void Servo::writeMicroseconds(int microsecs)
{

#ifdef SERVO_PWM_WITH_I2C
  int byteDuty = transform_cypress_duty_cycle_byte(microsecs);
#else
  int byteDuty = microsecs;
#endif
  if (this->lastByteInDuty == byteDuty)
    return;

  this->lastByteInDuty = byteDuty;

  // checking the boundaries
  if (microsecs < this->min) microsecs = this->min;
  if (microsecs > this->max) microsecs = this->max;

#ifdef SERVO_PWM_WITH_I2C

  prepare_pin(this->pin);

  // Set duty cycle
  Wire.beginTransmission(CYPRESS_I2C_ADDRESS);
  Wire.write(0x2b);
  Wire.write(byteDuty);
  Wire.endTransmission();
#else
  // set in sysfs
  setDuty(microsecs * 1000);
#endif
  // update last microseconds passed
  this->usecs = microsecs;

}

void Servo::write(int val)
{

  // according to Arduino reference lib, if this angle will
  // be bigger than 200, it should be considered as microsenconds

  if (val < MIN_PULSE_WIDTH)
  {
    // yeah.. user is passing angles

    if (val  < 0)
      val  = 0;
    else if (val > 180)
      val = 180;

    trace_debug("it is an angle:%d  this->min:%d  this->max:%d\n", val, this->min, this->max);
    writeMicroseconds(map(val, MIN_ANGLE, MAX_ANGLE, this->min, this->max));
  }
  else
  {
    trace_debug("it is microseconds:%d\n", val);
    // actually angle on this case it is microseconds
    writeMicroseconds(val);
  }
}

int Servo::read()
{
  return map(this->usecs, this->min, this->max, MIN_ANGLE, MAX_ANGLE);
}

int Servo::readMicroseconds()
{
  return this->usecs;
}

bool Servo::attached()
{
  return this->isAttached;
}


void Servo::detach()
{
    if (this->isAttached)
    {
        this->isAttached = false;
                counter--;
#ifdef SERVO_PWM_WITH_I2C
        pinMode(this->pin, OUTPUT);
#endif
        this->lastByteInDuty = -1;
    }
#ifdef SERVO_PWM_WITH_SYSFS
        disablePin();

        if (counter <= 0)
        {
            // restore the original frequency
                // for PWM channels
                setFreqInSysFs(SYSFS_PWM_PERIOD_NS);
        }
#endif
}

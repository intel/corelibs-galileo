/*
 * EEPROM Clear
 *
 * Sets all of the bytes of the EEPROM to 0.
 * This example code is in the public domain.

 */

#include <EEPROM.h>

// static int size=11264; /* Intel Galileo */
static int size=1024;  /* Intel Galileo Gen2 */

void setup()
{
  // write a 0 to all bytes of the EEPROM
  for (int i = 0; i < size; i++)
    EEPROM.write(i, 0);

  // turn the LED on when we're done
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

void loop()
{
}

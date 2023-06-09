/*
 * led.c
 *
 *  Created on: Apr 23, 2023
 *      Author: gns2l
 */



#include "led.h"

#ifdef _USE_HW_LED

bool ledInit(void)
{
  return true;
}

void ledOn(uint8_t ch)
{
  switch(ch)
  {
    case _DEF_LED1:
      break;
  }
}

void ledOff(uint8_t ch)
{
  switch(ch)
  {
    case _DEF_LED1:
      break;
  }
}

void ledToggle(uint8_t ch)
{
  switch(ch)
  {
    case _DEF_LED1:
      if (0)
        ledOn(_DEF_LED1);
      else
        ledOff(_DEF_LED1);
      break;
  }
}

#endif

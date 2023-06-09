/*
 * utils.c
 *
 *  Created on: Apr 23, 2023
 *      Author: gns2l
 */



#include "utils.h"





void delay(uint32_t time_ms)
{
  HAL_Delay(time_ms);
}

uint32_t millis(void)
{
  return HAL_GetTick();
}

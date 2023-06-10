/*
 * bsp.c
 *
 *  Created on: 2023. 6. 10.
 *      Author: gns2l
 */


#include "bsp.h"
#include "uart.h"


bool bspInit(void)
{

  return true;
}



void bspDeInit(void)
{
  HAL_RCC_DeInit();

  // Disable Interrupts
  NVIC->ICER[0] = 0xFFFFFFFF;
  __DSB();
  __ISB();

  SysTick->CTRL = 0;
}


void delay(uint32_t time_ms)
{
  HAL_Delay(time_ms);
}


uint32_t millis(void)
{
  return HAL_GetTick();
}


#ifndef _USE_HW_LOG
void logPrintf(const char *fmt, ...)
{
#ifdef DEBUG
  va_list args;
  int len;
  char buf[256];

  va_start(args, fmt);
  len = vsnprintf(buf, 256, fmt, args);

  uartWrite(HW_LOG_CH, (uint8_t *)buf, len);

  va_end(args);

#endif

}
#endif

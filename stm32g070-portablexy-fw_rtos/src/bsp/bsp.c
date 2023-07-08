/*
 * bsp.c
 *
 *  Created on: Aug 21, 2022
 *      Author: gns2l
 */





#include "bsp.h"
#include "uart.h"
#include "utils.h"

void SystemClock_Config(void);


bool bspInit(void)
{
	HAL_Init();

	SystemClock_Config();

	return true;
}

void delay(uint32_t ms)
{
#ifdef _USE_HW_RTOS
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    osDelay(ms);
  }
  else
  {
    HAL_Delay(ms);
  }
#else
  HAL_Delay(ms);
#endif
}

uint32_t millis(void)
{
  return HAL_GetTick();
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




void logView(const char* file, const char* func, const int line, const char* fmt, ...)
{

#ifdef DEBUG
  const int ARG_TBL_CNT_MAX = 10;
  const uint8_t FILE_STR_MAX = 255;

  char tmp_str[FILE_STR_MAX];
  memset(tmp_str,0, FILE_STR_MAX);
  strcpy(tmp_str, file);
  char* arg_tbl[ARG_TBL_CNT_MAX];
  memset(arg_tbl,0, ARG_TBL_CNT_MAX);
  uint8_t arg_cnt = utilParseArgs(tmp_str, arg_tbl, "/", ARG_TBL_CNT_MAX);

  va_list args;
  char buf[256];
  va_start(args, fmt);
  vsnprintf(buf, 256, fmt, args);
  uartPrintf(HW_LOG_CH, "[%s][%s(%d)][%s] \n", arg_tbl[arg_cnt - 1], func, line, (const char*)buf);
  va_end(args);

#endif
}

void SystemClock_Config(void)
{
	 LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }

  /* HSE configuration and activation */
  LL_RCC_HSE_Enable();
  while(LL_RCC_HSE_IsReady() != 1)
  {
  }

  /* LSI configuration and activation */
  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() != 1)
  {
  }

  LL_PWR_EnableBkUpAccess();
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_1, 16, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(64000000);

  /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  while(1)
  {
  }

}

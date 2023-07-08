/*
 * bsp.h
 *
 *  Created on: Aug 21, 2022
 *      Author: gns2l
 */

#ifndef SRC_BSP_BSP_H_
#define SRC_BSP_BSP_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "def.h"

#include "stm32g0xx_hal.h"

#ifdef _USE_HW_RTOS
#include "rtos.h"
#endif

#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_gpio.h"


  bool bspInit(void);
  void bspDeInit(void);
  void delay(uint32_t time_ms);
  uint32_t millis(void);
  void logPrintf(const char *fmt, ...);
  void logView(const char* file, const char* func, const int line, const char* fmt, ...);
  void Error_Handler(void);
  
  void delayUs(uint32_t us);


#ifndef LOG_PRINT
#define LOG_PRINT(fmt, ...)  logView(__FILE__, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#endif


#ifdef __cplusplus
}
#endif


#endif /* SRC_BSP_BSP_H_ */

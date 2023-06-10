/*
 * bsp.h
 *
 *  Created on: Sep 12, 2022
 *      Author: gns2l
 *
 * for stm32g070 bootloader fw
 * to reduce the file size, the function is simplified.
 * logPrintf  -> (system callback function)print
 */

#ifndef SRC_BSP_BSP_H_
#define SRC_BSP_BSP_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "def.h"

#include "stm32g0xx_hal.h"

#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_gpio.h"

/*
 * Macro that connects logprintf function with printf function
 */
#define LOG_PRINT  1
#if LOG_PRINT
#define logPrintf(fmt, ...)   printf(fmt, ##__VA_ARGS__)
#else
//If you do not use the printf function
#define logPrintf(fmt, ...)
#endif

	bool bspInit(void);
	void bspDeInit(void);
	void delay(uint32_t ms);
	uint32_t millis(void);
	void Error_Handler(void);

// Post a prototype when using the log function.
// void logPrintf(const char *fmt, ...);


#if 0
#define LOG_ERROR_(fmt, ...) \
		logPrintf("%s:%d:error: " fmt "%s\n", __FILE__, __LINE__, __VA_ARGS__)
#define LOG_ERROR(...) LOG_ERROR_(__VA_ARGS__, "")
#else
#define LOG_ERROR(...)
#endif


#ifdef __cplusplus
}
#endif


#endif /* SRC_BSP_BSP_H_ */

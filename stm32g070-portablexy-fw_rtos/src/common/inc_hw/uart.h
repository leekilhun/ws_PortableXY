/*
 * uart.h
 *
 *  Created on: Apr 23, 2023
 *      Author: gns2l
 */

#ifndef INCLUDE_UART_H_
#define INCLUDE_UART_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "hw_def.h"


#ifdef _USE_HW_UART

#define UART_MAX_CH         HW_UART_MAX_CH


bool     uartInit(void);
bool     uartOpen(uint8_t ch, uint32_t baud);
bool     uartClose(uint8_t ch);
uint32_t uartAvailable(uint8_t ch);
bool     uartFlush(uint8_t ch);
uint8_t  uartRead(uint8_t ch);
uint32_t uartWrite(uint8_t ch, uint8_t *p_data, uint32_t length);
uint32_t uartPrintf(uint8_t ch, const char *fmt, ...);
uint32_t uartGetBaud(uint8_t ch);

#endif

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_UART_H_ */

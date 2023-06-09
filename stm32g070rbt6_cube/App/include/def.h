/*
 * def.h
 *
 *  Created on: Apr 23, 2023
 *      Author: gns2l
 */

#ifndef INCLUDE_DEF_H_
#define INCLUDE_DEF_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>


#ifdef __cplusplus
 extern "C" {
#endif


#include "main.h"


#define _DEF_CH1          0
#define _DEF_CH2          1
#define _DEF_CH3          2
#define _DEF_CH4          3


#define UART_CH1    0
#define UART_CH2    1
#define UART_MAX_CH 2


void delay(uint32_t time_ms);
uint32_t millis(void);



#ifdef __cplusplus
 }
#endif




#endif /* INCLUDE_DEF_H_ */

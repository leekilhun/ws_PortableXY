/*
 * sys.h
 *
 *  Created on: 2022. 9. 13.
 *      Author: gns2l
 */

#ifndef SRC_SYS_SYS_H_
#define SRC_SYS_SYS_H_



#ifdef __cplusplus
extern "C" {
#endif


#include "def.h"
#include "util.h"

#define _USE_LOG_PRINT    1

#if _USE_LOG_PRINT
#define logPrintf(fmt, ...)     printf(fmt, ##__VA_ARGS__)
#else
#define logPrintf(fmt, ...)
#endif



void sysInit(void);

void delay(uint32_t ms);
uint32_t millis(void);



#ifdef __cplusplus
}
#endif



#endif /* SRC_SYS_SYS_H_ */

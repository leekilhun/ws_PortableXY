/*
 * rtos.h
 *
 *  Created on: Dec 28, 2021
 *      Author: gns2.lee
 */

#ifndef SRC_BSP_RTOS_H_
#define SRC_BSP_RTOS_H_



#include "def.h"


#ifdef __cplusplus
extern "C" {
#endif


#include "cmsis_os.h"
#include "cpu_utils.h"


void rtosInit(void);



#ifdef __cplusplus
}
#endif

#endif /* SRC_BSP_RTOS_H_ */

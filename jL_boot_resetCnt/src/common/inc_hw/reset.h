/*
 * reset.h
 *
 *  Created on: Jul 10, 2021
 *      Author: gns2l
 */

#ifndef SRC_COMMON_INC_HW_RESET_H_
#define SRC_COMMON_INC_HW_RESET_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw_def.h"


#ifdef _USE_HW_RESET

#define RESET_REG_PARAM         0
#define RESET_REG_COUNT         1

bool resetInit(void);

/*
 * read count data in rtc backup register
 */
uint32_t resetGetCount(void);


/*
 *  When the decreasing time reaches 0, the boot mode is entered.
 */
void resetToBoot(uint32_t timeout);


/*
 * reset to systemboot mode
 * to use dfu mode
 */
void resetToSysBoot(void);

#endif

#ifdef __cplusplus
}
#endif

#endif /* SRC_COMMON_INC_HW_RESET_H_ */

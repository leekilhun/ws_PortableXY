/*
 * button.h
 *
 *  Created on: Jul 11, 2021
 *      Author: gns2l
 */

#ifndef SRC_COMMON_INC_HW_BUTTON_H_
#define SRC_COMMON_INC_HW_BUTTON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw_def.h"


#ifdef _USE_HW_BUTTON

#define BUTTON_MAX_CH         HW_BUTTON_MAX_CH


bool buttonInit(void);
bool buttonGetPressed(uint8_t ch);


#endif

#ifdef __cplusplus
}
#endif

#endif /* SRC_COMMON_INC_HW_BUTTON_H_ */

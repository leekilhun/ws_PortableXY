/*
 * gpio.h
 *
 *  Created on: Nov 27, 2021
 *      Author: gns2l
 */

#ifndef SRC_COMMON_INC_HW_GPIO_H_
#define SRC_COMMON_INC_HW_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw_def.h"


#ifdef _USE_HW_GPIO


#define GPIO_MAX_CH     HW_GPIO_MAX_CH

bool gpioInit(void);
bool gpioPinMode(uint8_t ch, uint8_t mode);
void gpioPinWrite(uint8_t ch, bool value);
bool gpioPinRead(uint8_t ch);
void gpioPinToggle(uint8_t ch);
bool gpioIsOn(uint8_t ch);


#endif

#ifdef __cplusplus
}
#endif

#endif /* SRC_COMMON_INC_HW_GPIO_H_ */

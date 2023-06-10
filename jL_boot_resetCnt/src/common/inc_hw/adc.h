/*
 * adc.h
 *
 *  Created on: Dec 10, 2021
 *      Author: gns2l
 */

#ifndef SRC_COMMON_INC_HW_ADC_H_
#define SRC_COMMON_INC_HW_ADC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw_def.h"

#ifdef _USE_HW_ADC



bool adcInit(void);
bool adcStartDma(uint8_t ch, uint32_t* pData, uint32_t len);
uint32_t adcReadVoltage(uint8_t ch);
uint32_t adcReadCurrent(uint8_t ch);

#endif /*_USE_HW_ADC*/

#ifdef __cplusplus
}
#endif

#endif /* SRC_COMMON_INC_HW_ADC_H_ */

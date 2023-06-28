/*
 * rtc.h
 *
 *  Created on: Jun 26, 2023
 *      Author: gns2l
 */

#ifndef COMMON_INC_HW_RTC_H_
#define COMMON_INC_HW_RTC_H_



#ifdef __cplusplus
extern "C" {
#endif

#include "hw_def.h"


#ifdef _USE_HW_RTC


bool rtcInit(void);

uint32_t rtcBackupRegRead(uint32_t index);
void     rtcBackupRegWrite(uint32_t index, uint32_t data);


#endif

#ifdef __cplusplus
}
#endif


#endif /* COMMON_INC_HW_RTC_H_ */

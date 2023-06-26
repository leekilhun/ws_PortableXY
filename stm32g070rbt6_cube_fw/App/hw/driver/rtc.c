/*
 * rtc.c
 *
 *  Created on: Jun 26, 2023
 *      Author: gns2l
 */





#include "rtc.h"

#ifdef _USE_HW_RTC


extern RTC_HandleTypeDef hrtc;



bool rtcInit(void)
{
  bool ret = true;

  return ret;
}

uint32_t rtcBackupRegRead(uint32_t index)
{
  return HAL_RTCEx_BKUPRead(&hrtc, index);
}

void rtcBackupRegWrite(uint32_t index, uint32_t data)
{
  HAL_RTCEx_BKUPWrite(&hrtc, index, data);
}




#endif /*_USE_HW_RTC*/


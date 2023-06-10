/*
 * rtc.c
 *
 *  Created on: Sep 11, 2022
 *      Author: gns2l
 */






#include "rtc.h"

#ifdef _USE_HW_RTC


static RTC_HandleTypeDef hrtc;



bool rtcInit(void)
{
	bool ret = true;

	/** Initialize RTC Only
	 */
	hrtc.Instance 						= RTC;
	hrtc.Init.HourFormat 			= RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv 		= 127;
	hrtc.Init.SynchPrediv 		= 255;
	hrtc.Init.OutPut 					= RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutRemap 		= RTC_OUTPUT_REMAP_NONE;
	hrtc.Init.OutPutPolarity 	= RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType 			= RTC_OUTPUT_TYPE_OPENDRAIN;
	hrtc.Init.OutPutPullUp 		= RTC_OUTPUT_PULLUP_NONE;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		ret = false;
	}
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




void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

	if(rtcHandle->Instance==RTC)
	{
		if(LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSI)
		{
			LL_RCC_ForceBackupDomainReset();
			LL_RCC_ReleaseBackupDomainReset();
			LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
		}

		/* RTC clock enable */
		__HAL_RCC_RTC_ENABLE();
		__HAL_RCC_RTCAPB_CLK_ENABLE();
	}
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

	if(rtcHandle->Instance==RTC)
	{
		/* USER CODE BEGIN RTC_MspDeInit 0 */

		/* USER CODE END RTC_MspDeInit 0 */
		/* Peripheral clock disable */
	  __HAL_RCC_RTC_DISABLE();
	  __HAL_RCC_RTCAPB_CLK_DISABLE();
		/* USER CODE BEGIN RTC_MspDeInit 1 */

		/* USER CODE END RTC_MspDeInit 1 */
	}
}


#endif /*_USE_HW_RTC*/

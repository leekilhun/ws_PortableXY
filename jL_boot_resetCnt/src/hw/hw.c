/*
 * hw.c
 *
 *  Created on: Aug 21, 2022
 *      Author: gns2l
 */


#include "hw.h"

__attribute__((section(".version"))) firm_version_t boot_ver =
    {
        _DEF_BOOT_VERSION,
        _DEF_BOARD_NAME
    };


bool hwInit(void)
{
	bool ret = true;

	ret &= bspInit();

#ifdef _USE_HW_CLI
	ret &= cliInit();
#endif
	ret &= uartInit();

	ret &=rtcInit();

	ret &=resetInit();

	ret &= ledInit();

	ret &= flashInit();

#ifdef _USE_HW_LOG
	ret &= logInit();
	logOpen(HW_LOG_CH, 115200);
#else // use small size log func
	uartOpen(HW_LOG_CH, 115200);
#endif
	logPrintf("[ boot Begin... ]\r\n");



	return ret;
}




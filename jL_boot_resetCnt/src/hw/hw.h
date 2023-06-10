/*
 * hw.h
 *
 *  Created on: Aug 21, 2022
 *      Author: gns2l
 */


#ifdef __cplusplus
extern "C" {
#endif


#include "hw_def.h"

#include "led.h"
#include "flash.h"
#include "uart.h"
#include "cli.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "log.h"

#include "flash.h"
#include "reset.h"
#include "rtc.h"

#include "buzzer.h"
#include "gpio.h"
#include "cmd.h"


bool hwInit(void);

#ifdef __cplusplus
}
#endif

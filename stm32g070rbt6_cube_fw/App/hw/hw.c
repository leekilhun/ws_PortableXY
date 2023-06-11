/*
 * hw.c
 *
 *  Created on: Jun 9, 2023
 *      Author: gns2l
 */


#include "hw.h"


extern uint32_t _flash_tag_addr;
extern uint32_t __isr_vector_addr;


__attribute__((section(".tag"))) firm_tag_t firm_tag =
{
    .magic_number = 0xAAAA5555,
    //-- fw info
    //
    .addr_tag = (uint32_t)&_flash_tag_addr,
    .addr_fw  = (uint32_t)&__isr_vector_addr,
    .size_tag = 1024,

    //-- tag info
    //
};


__attribute__((section(".version"))) firm_version_t firm_ver =
{
    _DEF_FW_VERSION,
    _DEF_FW_NAME
};


bool hwInit(void)
{
  bool ret = true;

#ifdef _USE_HW_CLI
  ret &= cliInit();
#endif

#ifdef _USE_HW_LED
  ret &= ledInit();
#endif

#ifdef _USE_HW_UART
  ret &= uartInit();
#endif

#ifdef _USE_HW_GPIO
  ret &= gpioInit();
#endif

#ifdef _USE_HW_LOG
  ret &= logInit();

  logOpen(HW_LOG_CH, 115200);
  logPrintf("[ fw Begin... ]\r\n");
#else // use small size log func

  firm_version_t *p_boot_ver = (firm_version_t *)(FLASH_ADDR_BOOT_VER);

  uartOpen(HW_LOG_CH, 115200);
  logPrintf("[ Firmware Begin... ]\r\n");
  logPrintf("Booting..Name \t\t: %s\r\n", p_boot_ver->name);
  logPrintf("Booting..Ver  \t\t: %s\r\n", p_boot_ver->version);
  logPrintf("Firmware.Name \t\t: %s\r\n", _DEF_FW_NAME);
  logPrintf("Firmware.Ver  \t\t: %s\r\n", _DEF_FW_VERSION);
  logPrintf("Core Clock    \t\t: %d Mhz\r\n", SystemCoreClock/1000000);
#endif

#ifdef _USE_HW_I2C
  ret &= i2cInit();
#endif


#ifdef _USE_HW_CAN
  ret &= canInit();
#endif


  return ret;
}


/*
 * hw_def.h
 *
 *  Created on: Aug 21, 2022
 *      Author: gns2l
 */

#ifndef SRC_HW_HW_DEF_H_
#define SRC_HW_HW_DEF_H_



#include "bsp.h"


/****************************************************
   boot/ firmware memory information
 ****************************************************/
#define _DEF_BOOT_VERSION           "B220911R1"
#define _DEF_BOARD_NAME             "jL Bootloader"

#define FLASH_ADDR_TAG              0x08005000
#define FLASH_ADDR_FW               0x08005400
#define FLASH_ADDR_FW_VER           0x08005800

#define FLASH_ADDR_START            0x08005000
#define FLASH_ADDR_END              (FLASH_ADDR_START + (128-20)*1024)

/* ------------------------------------------------- */


#define _USE_HW_FLASH

#define _USE_HW_RTC
#define _USE_HW_RESET


#define _USE_HW_LED
#define      HW_LED_MAX_CH          1

#define _USE_HW_UART
#define      HW_UART_MAX_CH         4
//#define _USE_HW_UART_1_DMA
//#define _USE_HW_UART_3_DMA

//#define _USE_HW_CLI
#define      HW_CLI_CH              _DEF_UART3
#define      HW_CLI_CMD_LIST_MAX    16
#define      HW_CLI_CMD_NAME_MAX    16
#define      HW_CLI_LINE_HIS_MAX    4
#define      HW_CLI_LINE_BUF_MAX    64


#define _USE_HW_FLASH

#define _USE_HW_RESET

//#define _USE_HW_BUZZER

#define _USE_HW_CMD
#define      HW_CMD_MAX_DATA_LENGTH 1024

//#define _USE_HW_LOG
#define      HW_LOG_CH              _DEF_UART2
#define      HW_LOG_BOOT_BUF_MAX    1024
#define      HW_LOG_LIST_BUF_MAX    2048



//#define _USE_HW_CAN
#define      HW_CAN_MAX_CH          1
#define      HW_CAN_MSG_RX_BUF_MAX  16

//#define _USE_HW_SPI
#define      HW_SPI_MAX_CH          1
//#define        _SPI_W25Q16          _DEF_SPI1
//#define        _SPI_SLAVE           _DEF_SPI2


//#define _USE_HW_I2C
#define      HW_I2C_MAX_CH          2





#endif /* SRC_HW_HW_DEF_H_ */

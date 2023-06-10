/*
 * hw_def.h
 *
 *  Created on: Jun 9, 2023
 *      Author: gns2l
 */

#ifndef HW_HW_DEF_H_
#define HW_HW_DEF_H_



#include "bsp.h"


/****************************************************
   boot/ firmware memory information
 ****************************************************/
#define _DEF_FW_VERSION             "V230608R1"
#define _DEF_FW_NAME                "PortableXY_firm"
#define FLASH_ADDR_BOOT_VER         0x08000400
#define FLASH_ADDR_TAG              0x08005000
#define FLASH_ADDR_FW               0x08005400
#define FLASH_ADDR_FW_VER           0x08005800
/* ------------------------------------------------- */



#define _USE_HW_RTC
#define _USE_HW_RESET

#define _USE_HW_LED
#define      HW_LED_MAX_CH          1


#define _USE_HW_BUZZER

#define _USE_HW_GPIO
#define      HW_GPIO_MAX_CH         27
#define        _GPIO_SPI_NCS        0
#define        _GPIO_SPI_NSS        1
#define        _GPIO_OP_SW_START    2
#define        _GPIO_OP_SW_STOP     3
#define        _GPIO_OP_SW_RESET    4
#define        _GPIO_OP_SW_ESTOP    5
#define        _GPIO_OP_LAMP_START  6
#define        _GPIO_OP_LAMP_STOP   7
#define        _GPIO_OP_LAMP_RESET  8
#define        _GPIO_IO_IN_01       9
#define        _GPIO_IO_IN_02       10
#define        _GPIO_IO_IN_03       11
#define        _GPIO_IO_IN_04       12
#define        _GPIO_IO_IN_05       13
#define        _GPIO_IO_IN_06       14
#define        _GPIO_IO_IN_07       15
#define        _GPIO_IO_IN_08       16
#define        _GPIO_IO_OUT_01      17
#define        _GPIO_IO_OUT_02      18
#define        _GPIO_IO_OUT_03      19
#define        _GPIO_IO_OUT_04      20
#define        _GPIO_IO_OUT_05      21
#define        _GPIO_IO_OUT_06      22
#define        _GPIO_IO_OUT_07      23
#define        _GPIO_IO_OUT_08      24
#define        _GPIO_I2C_INTERRUPT  25
#define        _GPIO_SPI_ROM_CS     26



#define _USE_HW_UART
#define      HW_UART_MAX_CH         4
#define      HW_UART_MOTOR          _DEF_UART1
#define      HW_UART_PC             _DEF_UART2
#define      HW_UART_LCD            _DEF_UART3
#define      HW_UART_LOG            _DEF_UART4
#define _USE_HW_UART_1_DMA
#define _USE_HW_UART_2_DMA
#define _USE_HW_UART_3_DMA


//#define _USE_HW_LOG
#define      HW_LOG_CH              _DEF_UART4
#define      HW_LOG_BOOT_BUF_MAX    1024
#define      HW_LOG_LIST_BUF_MAX    2048

#define _USE_HW_CLI
#define      HW_CLI_CMD_LIST_MAX    16
#define      HW_CLI_CMD_NAME_MAX    16
#define      HW_CLI_LINE_HIS_MAX    4
#define      HW_CLI_LINE_BUF_MAX    64

//#define _USE_HW_CAN
#define      HW_CAN_MAX_CH          1
#define      HW_CAN_MSG_RX_BUF_MAX  16

//#define _USE_HW_SPI
#define      HW_SPI_MAX_CH          1
//#define        _SPI_W25Q16          _DEF_SPI1
//#define        _SPI_SLAVE           _DEF_SPI2


#define _USE_HW_I2C
#define      HW_I2C_MAX_CH          2
#define        _I2C_EXT_IO          _DEF_I2C1
#define        _I2C_EEPROM          _DEF_I2C2


#endif /* HW_HW_DEF_H_ */

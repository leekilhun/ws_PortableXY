/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN_RESET_Pin GPIO_PIN_11
#define BTN_RESET_GPIO_Port GPIOC
#define BTN_STOP_Pin GPIO_PIN_12
#define BTN_STOP_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_13
#define BUZZER_GPIO_Port GPIOC
#define STATUS_Pin GPIO_PIN_14
#define STATUS_GPIO_Port GPIOC
#define LAMP_RESET_Pin GPIO_PIN_15
#define LAMP_RESET_GPIO_Port GPIOC
#define SPI2_NSS_Pin GPIO_PIN_0
#define SPI2_NSS_GPIO_Port GPIOC
#define SPI2_CS_Pin GPIO_PIN_1
#define SPI2_CS_GPIO_Port GPIOC
#define TXD3_Pin GPIO_PIN_0
#define TXD3_GPIO_Port GPIOA
#define RXD3_Pin GPIO_PIN_1
#define RXD3_GPIO_Port GPIOA
#define TXD1_Pin GPIO_PIN_2
#define TXD1_GPIO_Port GPIOA
#define RXD1_Pin GPIO_PIN_3
#define RXD1_GPIO_Port GPIOA
#define SPI1_CS_ROM_Pin GPIO_PIN_4
#define SPI1_CS_ROM_GPIO_Port GPIOA
#define TXD0_Pin GPIO_PIN_4
#define TXD0_GPIO_Port GPIOC
#define RXD0_Pin GPIO_PIN_5
#define RXD0_GPIO_Port GPIOC
#define SPI1_CS_Pin GPIO_PIN_1
#define SPI1_CS_GPIO_Port GPIOB
#define TXD2_Pin GPIO_PIN_10
#define TXD2_GPIO_Port GPIOB
#define RXD2_Pin GPIO_PIN_11
#define RXD2_GPIO_Port GPIOB
#define I2C2_INT_Pin GPIO_PIN_15
#define I2C2_INT_GPIO_Port GPIOB
#define IO_OUT7_Pin GPIO_PIN_6
#define IO_OUT7_GPIO_Port GPIOC
#define IO_OUT6_Pin GPIO_PIN_7
#define IO_OUT6_GPIO_Port GPIOC
#define IO_OUT5_Pin GPIO_PIN_8
#define IO_OUT5_GPIO_Port GPIOD
#define IO_OUT4_Pin GPIO_PIN_9
#define IO_OUT4_GPIO_Port GPIOD
#define IO_OUT3_Pin GPIO_PIN_11
#define IO_OUT3_GPIO_Port GPIOA
#define IO_OUT1_Pin GPIO_PIN_15
#define IO_OUT1_GPIO_Port GPIOA
#define IO_OUT0_Pin GPIO_PIN_8
#define IO_OUT0_GPIO_Port GPIOC
#define IO_OUT2_Pin GPIO_PIN_9
#define IO_OUT2_GPIO_Port GPIOC
#define LAMP_STOP_Pin GPIO_PIN_0
#define LAMP_STOP_GPIO_Port GPIOD
#define LAMP_START_Pin GPIO_PIN_2
#define LAMP_START_GPIO_Port GPIOD
#define IO_IN0_Pin GPIO_PIN_3
#define IO_IN0_GPIO_Port GPIOD
#define IO_IN1_Pin GPIO_PIN_4
#define IO_IN1_GPIO_Port GPIOD
#define IO_IN2_Pin GPIO_PIN_5
#define IO_IN2_GPIO_Port GPIOD
#define IO_IN3_Pin GPIO_PIN_6
#define IO_IN3_GPIO_Port GPIOD
#define IO_IN4_Pin GPIO_PIN_3
#define IO_IN4_GPIO_Port GPIOB
#define IO_IN5_Pin GPIO_PIN_4
#define IO_IN5_GPIO_Port GPIOB
#define IO_IN6_Pin GPIO_PIN_5
#define IO_IN6_GPIO_Port GPIOB
#define IO_IN7_Pin GPIO_PIN_6
#define IO_IN7_GPIO_Port GPIOB
#define BTN_ESTOP_Pin GPIO_PIN_7
#define BTN_ESTOP_GPIO_Port GPIOB
#define BTN_START_Pin GPIO_PIN_10
#define BTN_START_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

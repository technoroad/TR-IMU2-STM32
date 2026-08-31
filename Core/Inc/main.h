/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32u5xx_hal.h"
#include "stm32u5xx_ll_iwdg.h"
#include "stm32u5xx_ll_rcc.h"
#include "stm32u5xx_ll_spi.h"
#include "stm32u5xx_ll_tim.h"
#include "stm32u5xx_ll_system.h"
#include "stm32u5xx_ll_gpio.h"
#include "stm32u5xx_ll_exti.h"
#include "stm32u5xx_ll_lpgpio.h"
#include "stm32u5xx_ll_bus.h"
#include "stm32u5xx_ll_cortex.h"
#include "stm32u5xx_ll_utils.h"
#include "stm32u5xx_ll_pwr.h"
#include "stm32u5xx_ll_dma.h"

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
void MX_IWDG_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BRD_Pin LL_GPIO_PIN_2
#define BRD_GPIO_Port GPIOA
#define ADIS_nRST_Pin LL_GPIO_PIN_3
#define ADIS_nRST_GPIO_Port GPIOA
#define SPI1_NSS_Pin LL_GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define ADIS_SYNC_Pin LL_GPIO_PIN_0
#define ADIS_SYNC_GPIO_Port GPIOB
#define ADIS_DR_Pin LL_GPIO_PIN_1
#define ADIS_DR_GPIO_Port GPIOB
#define RS485_DE_Pin LL_GPIO_PIN_2
#define RS485_DE_GPIO_Port GPIOB
#define IN0_Pin LL_GPIO_PIN_10
#define IN0_GPIO_Port GPIOB
#define SPI2_NSS_Pin LL_GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define LD2_Pin LL_GPIO_PIN_8
#define LD2_GPIO_Port GPIOA
#define DP1_Pin LL_GPIO_PIN_3
#define DP1_GPIO_Port GPIOB
#define CAN_nSTBY_Pin LL_GPIO_PIN_5
#define CAN_nSTBY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

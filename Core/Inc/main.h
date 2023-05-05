/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_3
#define VCP_RX_GPIO_Port GPIOA
#define ADC_VOLTS_Pin GPIO_PIN_0
#define ADC_VOLTS_GPIO_Port GPIOB
#define ADC_AMPS_Pin GPIO_PIN_1
#define ADC_AMPS_GPIO_Port GPIOB
#define SPI_MUX_Pin GPIO_PIN_2
#define SPI_MUX_GPIO_Port GPIOB
#define FORCE_RECOVERY_BOOT_Pin GPIO_PIN_15
#define FORCE_RECOVERY_BOOT_GPIO_Port GPIOA
#define VBAT_CONNECTED_Pin GPIO_PIN_3
#define VBAT_CONNECTED_GPIO_Port GPIOB
#define MASTER_RESET_Pin GPIO_PIN_4
#define MASTER_RESET_GPIO_Port GPIOB
#define POWER_BUTTON_Pin GPIO_PIN_5
#define POWER_BUTTON_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define FEATURE_ANALOG
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

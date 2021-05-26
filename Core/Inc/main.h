/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BLE_NSS_Pin GPIO_PIN_4
#define BLE_NSS_GPIO_Port GPIOA
#define BLE_SCK_Pin GPIO_PIN_5
#define BLE_SCK_GPIO_Port GPIOA
#define BLE_MISO_Pin GPIO_PIN_6
#define BLE_MISO_GPIO_Port GPIOA
#define BLE_MOSI_Pin GPIO_PIN_7
#define BLE_MOSI_GPIO_Port GPIOA
#define HGM_Pin GPIO_PIN_4
#define HGM_GPIO_Port GPIOC
#define LNA_EN_Pin GPIO_PIN_0
#define LNA_EN_GPIO_Port GPIOB
#define PA_EN_Pin GPIO_PIN_1
#define PA_EN_GPIO_Port GPIOB
#define EN_HP_Pin GPIO_PIN_10
#define EN_HP_GPIO_Port GPIOB
#define GP_SW_Pin GPIO_PIN_11
#define GP_SW_GPIO_Port GPIOB
#define SWITCH_Pin GPIO_PIN_12
#define SWITCH_GPIO_Port GPIOB
#define CC_CS_Pin GPIO_PIN_8
#define CC_CS_GPIO_Port GPIOC
#define MPU_INT_Pin GPIO_PIN_11
#define MPU_INT_GPIO_Port GPIOA
#define MPU_SS_Pin GPIO_PIN_12
#define MPU_SS_GPIO_Port GPIOA
#define MPU_SCK_Pin GPIO_PIN_10
#define MPU_SCK_GPIO_Port GPIOC
#define MPU_MISO_Pin GPIO_PIN_11
#define MPU_MISO_GPIO_Port GPIOC
#define MPU_MOSI_Pin GPIO_PIN_12
#define MPU_MOSI_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

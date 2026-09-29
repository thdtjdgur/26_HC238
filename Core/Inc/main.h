/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define ST7789_DC_Pin GPIO_PIN_4
#define ST7789_DC_GPIO_Port GPIOA
#define ST7789_CS_Pin GPIO_PIN_6
#define ST7789_CS_GPIO_Port GPIOA
#define ST7789_RST_Pin GPIO_PIN_0
#define ST7789_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* LoRa RFM9x (SX1276) - SPI1 을 ST7789 와 공유, CS 는 PB12
 * CubeMX 에서 이 핀들에 라벨을 달면 위쪽에 자동 생성되므로 아래는 건너뛴다. */
#ifndef LORA_CS_Pin
#define LORA_CS_Pin            GPIO_PIN_12
#define LORA_CS_GPIO_Port      GPIOB
#endif
#ifndef LORA_RST_Pin
#define LORA_RST_Pin           GPIO_PIN_13
#define LORA_RST_GPIO_Port     GPIOB
#endif
#ifndef LORA_DIO0_Pin
#define LORA_DIO0_Pin          GPIO_PIN_14   /* 모듈의 G0 */
#define LORA_DIO0_GPIO_Port    GPIOB
#endif
#ifndef LORA_EN_Pin
#define LORA_EN_Pin            GPIO_PIN_15
#define LORA_EN_GPIO_Port      GPIOB
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

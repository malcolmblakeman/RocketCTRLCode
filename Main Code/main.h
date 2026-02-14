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
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/


void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);


/* Private defines -----------------------------------------------------------*/
#define IMU_INT_1_Pin GPIO_PIN_13
#define IMU_INT_1_GPIO_Port GPIOC
#define IMU_INT_2_Pin GPIO_PIN_14
#define IMU_INT_2_GPIO_Port GPIOC
#define LoRa_INT_1_Pin GPIO_PIN_15
#define LoRa_INT_1_GPIO_Port GPIOC
#define Busy_Pin GPIO_PIN_0
#define Busy_GPIO_Port GPIOH
#define BAROM_INT_Pin GPIO_PIN_1
#define BAROM_INT_GPIO_Port GPIOH
#define RSFW_V2_Pin GPIO_PIN_0
#define RSFW_V2_GPIO_Port GPIOA
#define RSFW_V1_Pin GPIO_PIN_1
#define RSFW_V1_GPIO_Port GPIOA
#define Pyro_CONT_1_Pin GPIO_PIN_6
#define Pyro_CONT_1_GPIO_Port GPIOA
#define Pyro_CONT_2_Pin GPIO_PIN_7
#define Pyro_CONT_2_GPIO_Port GPIOA
#define Pyro_CONT_3_Pin GPIO_PIN_4
#define Pyro_CONT_3_GPIO_Port GPIOC
#define Pyro_CONT_4_Pin GPIO_PIN_5
#define Pyro_CONT_4_GPIO_Port GPIOC
#define GPS_FIX_Pin GPIO_PIN_0
#define GPS_FIX_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_12
#define LED4_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_13
#define LED3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOB
#define Pyro_CTRL_4_Pin GPIO_PIN_6
#define Pyro_CTRL_4_GPIO_Port GPIOC
#define Pyro_CTRL_3_Pin GPIO_PIN_7
#define Pyro_CTRL_3_GPIO_Port GPIOC
#define Pyro_CTRL_2_Pin GPIO_PIN_8
#define Pyro_CTRL_2_GPIO_Port GPIOC
#define Pyro_CTRL_1_Pin GPIO_PIN_9
#define Pyro_CTRL_1_GPIO_Port GPIOC
#define Pyro_CTRL_Master_Pin GPIO_PIN_8
#define Pyro_CTRL_Master_GPIO_Port GPIOA
#define FLASH_WP_Pin GPIO_PIN_15
#define FLASH_WP_GPIO_Port GPIOA
#define SPI1_CS_RF_Pin GPIO_PIN_10
#define SPI1_CS_RF_GPIO_Port GPIOC
#define SPI2_CS_Flash_Pin GPIO_PIN_11
#define SPI2_CS_Flash_GPIO_Port GPIOC
#define SPI1_CS_BAROM_Pin GPIO_PIN_12
#define SPI1_CS_BAROM_GPIO_Port GPIOC
#define SPI1_CS_IMU_Pin GPIO_PIN_2
#define SPI1_CS_IMU_GPIO_Port GPIOD


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

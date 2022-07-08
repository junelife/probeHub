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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>

#define DEBUG_STATE
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
#define EXTI_ADDR_HEARTBEAT_DEBUG_Pin GPIO_PIN_8
#define EXTI_ADDR_HEARTBEAT_DEBUG_GPIO_Port GPIOB
#define EXTI_ADDR_HEARTBEAT_DEBUG_EXTI_IRQn EXTI4_15_IRQn
#define RS485_RECEIVER_EN_Pin GPIO_PIN_14
#define RS485_RECEIVER_EN_GPIO_Port GPIOC
#define RS485_DRIVER_EN_Pin GPIO_PIN_15
#define RS485_DRIVER_EN_GPIO_Port GPIOC
#define AN_PROBE_A1_Pin GPIO_PIN_0
#define AN_PROBE_A1_GPIO_Port GPIOA
#define AN_PROBE_A2_Pin GPIO_PIN_1
#define AN_PROBE_A2_GPIO_Port GPIOA
#define AN_PROBE_A3_Pin GPIO_PIN_2
#define AN_PROBE_A3_GPIO_Port GPIOA
#define EXTI_PROBE_A_DETECT_Pin GPIO_PIN_3
#define EXTI_PROBE_A_DETECT_GPIO_Port GPIOA
#define EXTI_PROBE_A_DETECT_EXTI_IRQn EXTI2_3_IRQn
#define AN_PROBE_B1_Pin GPIO_PIN_4
#define AN_PROBE_B1_GPIO_Port GPIOA
#define AN_PROBE_B2_Pin GPIO_PIN_5
#define AN_PROBE_B2_GPIO_Port GPIOA
#define AN_PROBE_B3_Pin GPIO_PIN_6
#define AN_PROBE_B3_GPIO_Port GPIOA
#define EXTI_PROBE_B_DETECT_Pin GPIO_PIN_7
#define EXTI_PROBE_B_DETECT_GPIO_Port GPIOA
#define EXTI_PROBE_B_DETECT_EXTI_IRQn EXTI4_15_IRQn
#define PWM_LED1_Pin GPIO_PIN_0
#define PWM_LED1_GPIO_Port GPIOB
#define USART_TX_Pin GPIO_PIN_9
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_10
#define USART_RX_GPIO_Port GPIOA
#define PWM_LED2_Pin GPIO_PIN_4
#define PWM_LED2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "common.h"
struct {
	GPIO_PinState heartBeat;
	GPIO_PinState probeA;
	GPIO_PinState probeB;
	GPIO_PinState rs485ReceiverEnable;
	GPIO_PinState rs485DriverEnable;
} gpio;
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = EXTI_ADDR_HEARTBEAT_DEBUG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXTI_ADDR_HEARTBEAT_DEBUG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = EXTI_PROBE_A_DETECT_Pin|EXTI_PROBE_B_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 2 */
void GPIO_Init_Read(void) {
	gpio.heartBeat = HAL_GPIO_ReadPin(EXTI_ADDR_HEARTBEAT_DEBUG_GPIO_Port, EXTI_ADDR_HEARTBEAT_DEBUG_Pin);
	gpio.probeA = HAL_GPIO_ReadPin(EXTI_PROBE_A_DETECT_GPIO_Port, EXTI_PROBE_A_DETECT_Pin);
	gpio.probeB = HAL_GPIO_ReadPin(EXTI_PROBE_B_DETECT_GPIO_Port, EXTI_PROBE_B_DETECT_Pin);

	// Turn on RS485 chip
//	HAL_GPIO_WritePin(RS485_RECEIVER_EN_GPIO_Port, RS485_RECEIVER_EN_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(RS485_DRIVER_EN_GPIO_Port, RS485_DRIVER_EN_Pin, GPIO_PIN_SET);
}
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
		case EXTI_ADDR_HEARTBEAT_DEBUG_Pin:
			gpio.heartBeat = GPIO_PIN_SET;
			break;
		case EXTI_PROBE_A_DETECT_Pin:
			gpio.probeA = GPIO_PIN_SET;
			break;
		case EXTI_PROBE_B_DETECT_Pin:
			gpio.probeB = GPIO_PIN_SET;
			break;
		default:
			// Future TO DO: Set Error state undefined GPIO EXTI interrupt
			break;
			Error_Handler();
	}
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
		case EXTI_ADDR_HEARTBEAT_DEBUG_Pin:
			gpio.heartBeat = GPIO_PIN_RESET;
			break;
		case EXTI_PROBE_A_DETECT_Pin:
			gpio.probeA = GPIO_PIN_RESET;
			break;
		case EXTI_PROBE_B_DETECT_Pin:
			gpio.probeB = GPIO_PIN_RESET;
			break;
		default:
			// Future TO DO: Set Error state undefined GPIO EXTI interrupt
			Error_Handler();
	}
}
/* USER CODE END 2 */

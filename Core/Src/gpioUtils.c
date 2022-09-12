/*
 * gpioUtils.c
 *
 *  Created on: Aug 16, 2022
 *      Author: ronso
 */
#include "common.h"
#include "gpioUtils.h"



void configGpioNoPull(GPIO_TypeDef  *port, uint16_t pin,  uint32_t mode, uint32_t altFunction)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  GPIO_InitStruct.Pin = pin;
	  GPIO_InitStruct.Mode = mode;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = altFunction;
	  HAL_GPIO_Init(port, &GPIO_InitStruct);
}

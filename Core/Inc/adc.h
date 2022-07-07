/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */
/* USER CODE BEGIN Private defines */
/* ADC channel states, each channel can have 1 of the 5 states by ADC_Config_Single()
 *
 */
typedef enum {
    ADC_OFFLINE = 0,
    ADC_ACTIVE,
    ADC_ACTIVE_UNSOLICITED,
} adcStateToken;

/* enumerated channel names
 *
 */
typedef enum {
	ADC_PROBE_A1 = 0,
	ADC_PROBE_A2,
	ADC_PROBE_A3,
	ADC_PROBE_B1,
	ADC_PROBE_B2,
	ADC_PROBE_B3,
	ADC_INTERNAL_TEMP,
	ADC_INTERNAL_VREF,

	ADC_END_OF_LIST,
} adcToken;
/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */
adcStateToken ADC_Config_Single(adcToken target, adcStateToken state);
uint16_t ADC_Value_Last_Get(adcToken target);
uint16_t ADC_Ripple_Last_Get(adcToken target);
adcStateToken ADC_Status_Single_Get(adcToken target);

// ADC Round Robin Task Function
bool ADC_Task(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */


/*
 * gpioXlate.h
 *
 * Copyright © Weber Stephen Products LLC., 2022 All Rights Reserved No part of this file may be
 * distributed, reproduced, or used by entities other than Stephen Products LLC. without express
 * written permission. All Rights Reserved UNPUBLISHED, LICENSED SOFTWARE IS CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Weber Stephen Products LLC.
 *
 *  Created on: July 22, 2022
 *      Author: ronso
 */

#ifndef INC_GPIOXLATE_H_
#define INC_GPIOXLATE_H_

//#define REV1_BOARD 1

#ifdef REV1_BOARD
  #define EXTI_PROBE_B_DETECT_GPIO_Port        GPIOA
  #define EXTI_PROBE_B_DETECT_Pin                   GPIO_PIN_7
  #define PWM_LED2_GPIO_Port                   GPIOC
  #define PWM_LED2_Pin                              GPIO_PIN_15
#else
  #define EXTI_PROBE_B_DETECT_GPIO_Port        GPIOB
  #define EXTI_PROBE_B_DETECT_Pin                   GPIO_PIN_9
  #define PWM_LED2_GPIO_Port                   GPIOA
  #define PWM_LED2_Pin                              GPIO_PIN_7
#endif

#define AN_PROBE_A1_GPIO_Port                GPIOA
#define AN_PROBE_A1_Pin                           GPIO_PIN_0
#define AN_PROBE_A2_GPIO_Port                GPIOA
#define AN_PROBE_A2_Pin                           GPIO_PIN_1
#define AN_PROBE_A3_GPIO_Port                GPIOA
#define AN_PROBE_A3_Pin                           GPIO_PIN_2
#define EXTI_PROBE_A_DETECT_GPIO_Port        GPIOA
#define EXTI_PROBE_A_DETECT_Pin                   GPIO_PIN_3
#define AN_PROBE_B1_GPIO_Port                GPIOA
#define AN_PROBE_B1_Pin                           GPIO_PIN_4
#define AN_PROBE_B2_GPIO_Port                GPIOA
#define AN_PROBE_B2_Pin                           GPIO_PIN_5
#define AN_PROBE_B3_GPIO_Port                GPIOA
#define AN_PROBE_B3_Pin                           GPIO_PIN_6
#define PWM_LED1_GPIO_Port                   GPIOB
#define PWM_LED1_Pin                              GPIO_PIN_0
#define RS485_TX_PORT                        GPIOA
#define RS485_TX_PIN                              GPIO_PIN_9
#define RS485_RX_PORT                        GPIOA
#define RS485_RX_PIN                              GPIO_PIN_10
#define RS485_DE_PORT                        GPIOB
#define RS485_DE_PIN                              GPIO_PIN_3

//#define EXTI_ADDR_HEARTBEAT_DEBUG_GPIO_Port  GPIOB
//#define EXTI_ADDR_HEARTBEAT_DEBUG_Pin             GPIO_PIN_8

//Interrupt USAGE
//#define EXTI_ADDR_HEARTBEAT_DEBUG_EXTI_IRQn  EXTI4_15_IRQn
#define EXTI_PROBE_A_DETECT_EXTI_IRQn        EXTI2_3_IRQn
#define EXTI_PROBE_B_DETECT_EXTI_IRQn        EXTI4_15_IRQn



#endif /* INC_GPIOXLATE_H_ */

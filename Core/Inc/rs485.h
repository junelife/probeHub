/*
 * rs485.h
 *
 * Copyright © Weber Stephen Products LLC., 2022 All Rights Reserved No part of this file may be
 * distributed, reproduced, or used by entities other than Stephen Products LLC. without express
 * written permission. All Rights Reserved UNPUBLISHED, LICENSED SOFTWARE IS CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Weber Stephen Products LLC.
 *
 *  Created on: Apr 14, 2022
 *      Author: ronso
 */

#ifndef INC_RS485_H_
#define INC_RS485_H_

#include "uart.h"

extern UART_HandleTypeDef rs485uart;

#define RS485_INSTANCE              USART1
#define RS485_BAUDRATE              115200;
#define RS485_WORDLENGTH            0x00000000U   /*!< 8-bit long UART frame */
#define RS485_STOPBITS              0x00000000U   /*!< UART frame with 1 stop bit    */
#define RS485_PARITY                0x00000000U
#define RS485_UART_MODE             UART_MODE_TX_RX
#define RS485_HW_FLOW_CTL           0x00000000U  /*!< No hardware control       */
#define RS485_OVERSAMPLING          0x00000000U  /*!< Oversampling by 16 */
#define RS485_UART_ONE_BIT_SAMPLING 0x00000000U  /*!< One-bit sampling disable */
#define RS485_CLK_PRESCALER         0x00000000U  /*!< fclk_pres = fclk     */
#define RS485_ADV_FEATURE           0x00000000U  /*!< No advanced feature initialization       */
#define RS485_UART_CLK_ENABLE()     __HAL_RCC_USART2_CLK_ENABLE()
#define RS485_INTERRUPT             USART2_IRQn


void initializeRs485(void);
extern void rs485RxInterrupt(UART_HandleTypeDef *);
uint16_t getRs485RxbufSize(void);
uint8_t getNextCharFromRs485rxBuf(void);
void UART1_IRQHandler(UART_HandleTypeDef *);
HAL_StatusTypeDef UART_Transmit_IT(UART_HandleTypeDef *, const uint8_t *, uint16_t);
#endif

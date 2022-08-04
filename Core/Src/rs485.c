/*
 * rs485.c
 *
 * Copyright © Weber Stephen Products LLC., 2022 All Rights Reserved No part of this file may be
 * distributed, reproduced, or used by entities other than Stephen Products LLC. without express
 * written permission. All Rights Reserved UNPUBLISHED, LICENSED SOFTWARE IS CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Weber Stephen Products LLC.
 *
 *  Created on: Apr 14, 2022
 *      Author: ronso
 *
 *  This file contains functions and the interrupt to handle transmit and receive data over
 *  the RS-485 Uart.
 */

#include "common.h"
#include "rs485.h"

#define UART_UNLOCKED 0x00U
#define UART_LOCKED 0x01U

#define UART_STATE_READY   0x00000020U
#define UART_STATE_BUSY    0x00000024U
#define UART_STATE_RESET   0x00000000U
#define UART_ERROR_NE      (0x00000002U)    /*!< Noise error             */

static void configureRs455RxInterrupt(UART_HandleTypeDef *);
static void initializeUart(UART_HandleTypeDef *);
static void UARTEx_SetTxFifoThreshold(UART_HandleTypeDef *, uint32_t);
static void SetRxFifoThreshold(UART_HandleTypeDef *, uint32_t);
static void rs485TxInterrupt(UART_HandleTypeDef *huart);
static void UARTEx_EnableFifoMode(UART_HandleTypeDef *);
static void rs485uartSetConfig(UART_HandleTypeDef *);


UART_HandleTypeDef rs485uart;
#ifndef UART_GETCLOCKSOURCE
const uint16_t UARTPrescTable[12] = {1U, 2U, 4U, 6U, 8U, 10U, 12U, 16U, 32U, 64U, 128U, 256U};
#endif
/*    RS485  USART1 GPIO Configuration
 *     PC4   ------> USART1_TX
 *     PC5   ------> USART1_RX
 *     PB3   ------> USART1_DE
 *
 */
void initializeRs485(void)
{

    rs485uart.Instance = RS485_INSTANCE;
    rs485uart.Init.BaudRate = RS485_BAUDRATE;
    rs485uart.Init.WordLength = RS485_WORDLENGTH;
    rs485uart.Init.StopBits = RS485_STOPBITS;
    rs485uart.Init.Parity = RS485_PARITY;
    rs485uart.Init.Mode = RS485_UART_MODE;
    rs485uart.Init.HwFlowCtl = RS485_HW_FLOW_CTL;
    rs485uart.Init.OverSampling = RS485_OVERSAMPLING;
    rs485uart.Init.OneBitSampling = RS485_UART_ONE_BIT_SAMPLING;
    rs485uart.Init.ClockPrescaler = RS485_CLK_PRESCALER;
    rs485uart.AdvancedInit.AdvFeatureInit = RS485_ADV_FEATURE;

    initializeUart(&rs485uart);
    UARTEx_SetTxFifoThreshold(&rs485uart, UART_TXFIFO_THRESHOLD_1_8);
    SetRxFifoThreshold(&rs485uart, UART_RXFIFO_THRESHOLD_1_8);

    UARTEx_EnableFifoMode(&rs485uart);


    // rs485uart.TxXferSize  = 0;
   // rs485uart.TxXferCount = 0;


   rs485uart.RxISR       = rs485RxInterrupt;
   rs485uart.TxISR       = rs485TxInterrupt;



    configureRs455RxInterrupt(&rs485uart);


    __HAL_UART_ENABLE_IT(&rs485uart,UART_IT_RXNE);

}



void UARTEx_EnableFifoMode(UART_HandleTypeDef *huart)
{
  uint32_t tmpcr1;

  /* Process Locked */
  huart->Lock = UART_LOCKED;

  huart->gState = UART_STATE_BUSY;

  /* Save actual UART configuration */
  tmpcr1 = READ_REG(huart->Instance->CR1);

  /* Disable UART */
  __HAL_UART_DISABLE(huart);

  /* Enable FIFO mode */
  SET_BIT(tmpcr1, USART_CR1_FIFOEN);
  huart->FifoMode = UART_FIFOMODE_ENABLE;

  /* Restore UART configuration */
  WRITE_REG(huart->Instance->CR1, tmpcr1);

  huart->gState = UART_STATE_READY;

  /* Process Unlocked */
  huart->Lock = UART_UNLOCKED;
}


void SetRxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold)
{
  uint32_t tmpcr1;

  /* Process Locked */
  huart->Lock = UART_LOCKED;

  huart->gState = UART_STATE_BUSY;

  /* Save actual UART configuration */
  tmpcr1 = READ_REG(huart->Instance->CR1);

  /* Disable UART */
  __HAL_UART_DISABLE(huart);

  /* Update RX threshold configuration */
  MODIFY_REG(huart->Instance->CR3, USART_CR3_RXFTCFG, Threshold);

  /* Restore UART configuration */
  WRITE_REG(huart->Instance->CR1, tmpcr1);

  huart->gState = UART_STATE_READY;

  /* Process Unlocked */
  huart->Lock = UART_UNLOCKED;
}

void initializeUart(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    if (huart->gState == UART_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        huart->Lock = UART_UNLOCKED;

        /* Init the low level hardware : GPIO, CLOCK, CORTEX */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
        PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;

        /* Configure the USART1 clock source */
        __HAL_RCC_USART1_CONFIG(PeriphClkInit.Usart1ClockSelection);

        __HAL_RCC_USART1_CLK_ENABLE();

        GPIO_InitStruct.Pin = RS485_TX_PIN | RS485_RX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
        HAL_GPIO_Init(RS485_TX_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = RS485_DE_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
        HAL_GPIO_Init(RS485_DE_PORT, &GPIO_InitStruct);

        /* USART1 interrupt Init */
        HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }

    huart->gState = HAL_UART_STATE_BUSY;

    __HAL_UART_DISABLE(huart);  /* Disable the Peripheral */

    rs485uartSetConfig(huart);  /* Set the UART Communication parameters */
    SET_BIT(huart->Instance->CR3, USART_CR3_DEM);  /* Enable the Driver Enable mode by setting the DEM bit in the CR3 register */
    MODIFY_REG(huart->Instance->CR3, USART_CR3_DEP, UART_DE_POLARITY_HIGH); /* Set the Driver Enable polarity */
    MODIFY_REG(huart->Instance->CR1, (USART_CR1_DEDT | USART_CR1_DEAT), 0); /* Set the Driver Enable assertion and deassertion times */

    __HAL_UART_ENABLE(huart); /* Enable the Peripheral */
    /* Initialize the UART State */
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->gState = HAL_UART_STATE_READY;
    huart->RxState = HAL_UART_STATE_READY;
    huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
    huart->RxEventType = HAL_UART_RXEVENT_TC;

    __HAL_UNLOCK(huart);

}


void rs485uartSetConfig(UART_HandleTypeDef *huart)
{
  uint32_t tmpreg;
  //UART_ClockSourceTypeDef clocksource;
  uint32_t usartdiv;
  uint32_t pclk;

  /* Check the parameters */
  assert_param(IS_UART_BAUDRATE(huart->Init.BaudRate));
  assert_param(IS_UART_WORD_LENGTH(huart->Init.WordLength));
  assert_param(IS_UART_STOPBITS(huart->Init.StopBits));
  assert_param(IS_UART_ONE_BIT_SAMPLE(huart->Init.OneBitSampling));


  assert_param(IS_UART_PARITY(huart->Init.Parity));
  assert_param(IS_UART_MODE(huart->Init.Mode));
  assert_param(IS_UART_HARDWARE_FLOW_CONTROL(huart->Init.HwFlowCtl));
  assert_param(IS_UART_OVERSAMPLING(huart->Init.OverSampling));
  assert_param(IS_UART_PRESCALER(huart->Init.ClockPrescaler));

  /*-------------------------- USART CR1 Configuration -----------------------*/
  /* Clear M, PCE, PS, TE, RE and OVER8 bits and configure
  *  the UART Word Length, Parity, Mode and oversampling:
  *  set the M bits according to huart->Init.WordLength value
  *  set PCE and PS bits according to huart->Init.Parity value
  *  set TE and RE bits according to huart->Init.Mode value
  *  set OVER8 bit according to huart->Init.OverSampling value */
  tmpreg = (uint32_t)huart->Init.WordLength | huart->Init.Parity | huart->Init.Mode | huart->Init.OverSampling ;
  MODIFY_REG(huart->Instance->CR1, USART_CR1_FIELDS, tmpreg);

  /*-------------------------- USART CR2 Configuration -----------------------*/
  /* Configure the UART Stop Bits: Set STOP[13:12] bits according
  * to huart->Init.StopBits value */
  MODIFY_REG(huart->Instance->CR2, USART_CR2_STOP, huart->Init.StopBits);

  /*-------------------------- USART CR3 Configuration -----------------------*/
  /* Configure
  * - UART HardWare Flow Control: set CTSE and RTSE bits according
  *   to huart->Init.HwFlowCtl value
  * - one-bit sampling method versus three samples' majority rule according
  *   to huart->Init.OneBitSampling (not applicable to LPUART) */
  tmpreg = (uint32_t)huart->Init.HwFlowCtl;

  MODIFY_REG(huart->Instance->CR3, USART_CR3_FIELDS, tmpreg);

  /*-------------------------- USART PRESC Configuration -----------------------*/
  /* Configure
  * - UART Clock Prescaler : set PRESCALER according to huart->Init.ClockPrescaler value */
  MODIFY_REG(huart->Instance->PRESC, USART_PRESC_PRESCALER, huart->Init.ClockPrescaler);

  /*-------------------------- USART BRR Configuration -----------------------*/
  //UART_GETCLOCKSOURCE(huart, clocksource);
#if 0
    switch (clocksource)
    {
      case UART_CLOCKSOURCE_PCLK1:
        pclk = HAL_RCC_GetPCLK1Freq();
        break;
      case UART_CLOCKSOURCE_HSI:
        pclk = (uint32_t) HSI_VALUE;
        break;
      case UART_CLOCKSOURCE_SYSCLK:
        pclk = HAL_RCC_GetSysClockFreq();
        break;
      case UART_CLOCKSOURCE_LSE:
        pclk = (uint32_t) LSE_VALUE;
        break;
      default:
        pclk = 0U;
        break;
    }
#endif
    pclk = HAL_RCC_GetPCLK1Freq();
    if (pclk != 0U)
    {
      /* USARTDIV must be greater than or equal to 0d16 */
      usartdiv = (uint32_t)(UART_DIV_SAMPLING16(pclk, huart->Init.BaudRate, huart->Init.ClockPrescaler));
      if ((usartdiv >= UART_BRR_MIN) && (usartdiv <= UART_BRR_MAX))
      {
        huart->Instance->BRR = (uint16_t)usartdiv;
      }
  }

  /* Initialize the number of data to process during RX/TX ISR execution */
  huart->NbTxDataToProcess = 1;
  huart->NbRxDataToProcess = 1;
}



void UARTEx_SetTxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold)
{
  uint32_t tmpcr1;

  /* Process Locked */
  huart->Lock = UART_LOCKED;

  huart->gState = HAL_UART_STATE_BUSY;

  /* Save actual UART configuration */
  tmpcr1 = READ_REG(huart->Instance->CR1);

  /* Disable UART */
  __HAL_UART_DISABLE(huart);

  /* Update TX threshold configuration */
  MODIFY_REG(huart->Instance->CR3, USART_CR3_TXFTCFG, Threshold);

  /* Set the number of data to process during RX/TX ISR execution */
  huart->NbTxDataToProcess = 1U;
  huart->NbRxDataToProcess = 1U;

  /* Restore UART configuration */
  WRITE_REG(huart->Instance->CR1, tmpcr1);

  huart->gState = UART_STATE_READY;

  /* Process Unlocked */
  huart->Lock = UART_UNLOCKED;
}


#define RS485_BUF_SIZE 64
uint8_t rs485rxBuf[RS485_BUF_SIZE];
uint8_t *rs485rxBufOutPtr;

static void configureRs455RxInterrupt(UART_HandleTypeDef *huart)
{
    huart->Lock = UART_LOCKED;
    huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
    huart->pRxBuffPtr  = rs485rxBuf;
    huart->RxXferSize  = RS485_BUF_SIZE;
    huart->RxXferCount = 0;
    huart->RxISR       = rs485RxInterrupt;
    huart->TxISR       = rs485TxInterrupt;
    rs485rxBufOutPtr   = rs485rxBuf;

    UART_MASK_COMPUTATION(huart);  // Computation of UART mask to apply to RDR register

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState = HAL_UART_STATE_BUSY_RX;

    ATOMIC_SET_BIT(huart->Instance->CR3, USART_CR3_EIE);  // Enable the UART Error Interrupt: (Frame error, noise error, overrun error)
    huart->Lock = UART_UNLOCKED;

    ATOMIC_SET_BIT(huart->Instance->CR3, USART_CR3_RXFTIE);
}


/* Main Uart 1 (RS-485) interrupt
 *   This is the first stop from the interrupt vector table
 *   Uart 1 is the 43 entry in the vector.
 */
void USART1_IRQHandler(void)
{
    UART1_IRQHandler(&rs485uart);
}



/*********************************************************************************************/
/*********************************************************************************************/
/*                        INTERUPT FUNCTIONS FOR RS-485 RX AND TX                            */
/*********************************************************************************************/
/*********************************************************************************************/

/* Both RX and TX interrupts are serviced here
 *   Some of this code should be removed, but for now, it works
 */
void UART1_IRQHandler(UART_HandleTypeDef *huart)
{
  uint32_t isrflags   = READ_REG(huart->Instance->ISR);
  uint32_t cr1its     = READ_REG(huart->Instance->CR1);
  uint32_t cr3its     = READ_REG(huart->Instance->CR3);

  uint32_t errorflags;
  uint32_t errorcode;

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE | USART_ISR_RTOF));
  if (errorflags == 0U)
  {
    /* UART in mode Receiver ---------------------------------------------------*/
    if (((isrflags & USART_ISR_RXNE_RXFNE) != 0U)
        && (((cr1its & USART_CR1_RXNEIE_RXFNEIE) != 0U)
            || ((cr3its & USART_CR3_RXFTIE) != 0U)))
    {
        if (huart->RxISR != NULL)
        {
          huart->RxISR(huart);
        }
        return;
    }
  }

  /* If errors occur */
  if ((errorflags != 0U)
      && ((((cr3its & (USART_CR3_RXFTIE | USART_CR3_EIE)) != 0U)
           || ((cr1its & (USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE | USART_CR1_RTOIE)) != 0U))))
  {
    /* UART parity error interrupt occurred -------------------------------------*/
    if (((isrflags & USART_ISR_PE) != 0U) && ((cr1its & USART_CR1_PEIE) != 0U))
    {
        huart->Instance->ICR = UART_CLEAR_PEF;
        huart->ErrorCode |= HAL_UART_ERROR_PE;
    }
    /* UART frame error interrupt occurred --------------------------------------*/
    if (((isrflags & USART_ISR_FE) != 0U) && ((cr3its & USART_CR3_EIE) != 0U))
    {
        huart->Instance->ICR = UART_CLEAR_FEF;
        huart->ErrorCode |= HAL_UART_ERROR_FE;
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if (((isrflags & USART_ISR_NE) != 0U) && ((cr3its & USART_CR3_EIE) != 0U))
    {
        huart->Instance->ICR = UART_CLEAR_NEF;
        huart->ErrorCode |= HAL_UART_ERROR_NE;
    }
    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if (((isrflags & USART_ISR_ORE) != 0U)
        && (((cr1its & USART_CR1_RXNEIE_RXFNEIE) != 0U) ||
            ((cr3its & (USART_CR3_RXFTIE | USART_CR3_EIE)) != 0U)))
    {
        huart->Instance->ICR = UART_CLEAR_OREF;
        huart->ErrorCode |= HAL_UART_ERROR_ORE;
    }
    /* UART Receiver Timeout interrupt occurred ---------------------------------*/
    if (((isrflags & USART_ISR_RTOF) != 0U) && ((cr1its & USART_CR1_RTOIE) != 0U))
    {
        huart->Instance->ICR = UART_CLEAR_RTOF;
        huart->ErrorCode |= HAL_UART_ERROR_RTO;
    }

    if (huart->ErrorCode != HAL_UART_ERROR_NONE)
    {
      /* UART in mode Receiver --------------------------------------------------*/
      if (((isrflags & USART_ISR_RXNE_RXFNE) != 0U)
          && (((cr1its & USART_CR1_RXNEIE_RXFNEIE) != 0U)
              || ((cr3its & USART_CR3_RXFTIE) != 0U)))
      {
        if (huart->RxISR != NULL)
        {
          huart->RxISR(huart);
        }
      }
      /* If Error is to be considered as blocking :
          - Receiver Timeout error in Reception
          - Overrun error in Reception
          - any error occurs in DMA mode reception
      */
      errorcode = huart->ErrorCode;
      if ((HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR)) ||
          ((errorcode & (HAL_UART_ERROR_RTO | HAL_UART_ERROR_ORE)) != 0U))
      {
        /* Blocking error : transfer is aborted
           Set the UART state ready to be able to start again the process,
           Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
          /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
          ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE));
          ATOMIC_CLEAR_BIT(huart->Instance->CR3, (USART_CR3_EIE | USART_CR3_RXFTIE));

          /* In case of reception waiting for IDLE event, disable also the IDLE IE interrupt source */
          if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
          {
            ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
          }
          /* At end of Rx process, restore huart->RxState to Ready */
          huart->RxState = HAL_UART_STATE_READY;
          huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;

          /* Reset RxIsr function pointer */
          //huart->RxISR = NULL;
      }
    }
    return;

  } /* End if  error occurs */

  /* Check current reception Mode :
     If Reception till IDLE event has been selected : */
  if ((huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
      && ((isrflags & USART_ISR_IDLE) != 0U)
      && ((cr1its & USART_ISR_IDLE) != 0U))
  {
      huart->Instance->ICR = UART_CLEAR_IDLEF;

    /* Check if DMA mode is enabled in UART */
    if (HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR))
    {
      /* DMA mode enabled */
      /* Check received length : If all expected data are received, do nothing,
         (DMA cplt callback will be called).
         Otherwise, if at least one data has already been received, IDLE event is to be notified to user */
      uint16_t nb_remaining_rx_data = (uint16_t) __HAL_DMA_GET_COUNTER(huart->hdmarx);
      if ((nb_remaining_rx_data > 0U)
          && (nb_remaining_rx_data < huart->RxXferSize))
      {
        /* Reception is not complete */
        huart->RxXferCount = nb_remaining_rx_data;

        /* In Normal mode, end DMA xfer and HAL UART Rx process*/
        if (HAL_IS_BIT_CLR(huart->hdmarx->Instance->CCR, DMA_CCR_CIRC))
        {
          /* Disable PE and ERR (Frame error, noise error, overrun error) interrupts */
          ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_PEIE);
          ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

          /* Disable the DMA transfer for the receiver request by resetting the DMAR bit
             in the UART CR3 register */
          ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

          /* At end of Rx process, restore huart->RxState to Ready */
          huart->RxState = HAL_UART_STATE_READY;
          huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;

          ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);

          (void)HAL_DMA_Abort(huart->hdmarx);   /* Last bytes received, so no need as the abort is immediate */
        }
      }
      return;
    }
    else
    {
      /* DMA mode not enabled */
      /* Check received length : If all expected data are received, do nothing.
         Otherwise, if at least one data has already been received, IDLE event is to be notified to user */
      uint16_t nb_rx_data = huart->RxXferSize - huart->RxXferCount;
      if ((huart->RxXferCount > 0U)
          && (nb_rx_data > 0U))
      {
        /* Disable the UART Parity Error Interrupt and RXNE interrupts */
        ATOMIC_CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE));

        /* Disable the UART Error Interrupt:(Frame error, noise error, overrun error) and RX FIFO Threshold interrupt */
        ATOMIC_CLEAR_BIT(huart->Instance->CR3, (USART_CR3_EIE | USART_CR3_RXFTIE));

        /* Rx process is completed, restore huart->RxState to Ready */
        huart->RxState = HAL_UART_STATE_READY;
        huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
        //huart->RxISR = NULL;   /* Clear RxISR function pointer */
        ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
      }
      return;
    }
  }

  /* UART wakeup from Stop mode interrupt occurred ---------------------------*/
  if (((isrflags & USART_ISR_WUF) != 0U) && ((cr3its & USART_CR3_WUFIE) != 0U))
  {
      huart->Instance->ICR = UART_CLEAR_WUF;
      return;
  }
  /* UART in mode Transmitter ------------------------------------------------*/
  if (((isrflags & USART_ISR_TXE_TXFNF) != 0U)
      && (((cr1its & USART_CR1_TXEIE_TXFNFIE) != 0U)
          || ((cr3its & USART_CR3_TXFTIE) != 0U)))
  {
    if (huart->TxISR != NULL)
    {
      huart->TxISR(huart);
    }
    return;
  }

  /* UART in mode Transmitter (transmission end) -----------------------------*/
  if (((isrflags & USART_ISR_TC) != 0U) && ((cr1its & USART_CR1_TCIE) != 0U))
  {
      ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_TCIE);  /* Disable the UART Transmit Complete Interrupt */
      huart->gState = HAL_UART_STATE_READY;  /* Tx process is ended, restore huart->gState to Ready */
      //huart->TxISR = NULL;   /* Cleat TxISR function pointer */
      return;
  }
}




/*********************************************************************************************/
/*********************************************************************************************/
/*                        INTERUPT FUNCTIONS FOR RS-485 FOR TX                               */
/*********************************************************************************************/
/*********************************************************************************************/

/**
  * @brief TX interrupt handler for RS-485 with FIFO mode enabled.
  * @note   Function is called under interruption only, once
  *         interruptions have been enabled.
  * @param huart UART handle.
  * @retval None
  */
static void rs485TxInterrupt(UART_HandleTypeDef *huart)
{
//  uint16_t  nb_tx_data;

  /* Check that a Tx process is ongoing */
  if (huart->gState == HAL_UART_STATE_BUSY_TX)
  {
      if (huart->TxXferCount == 0U)
      {
          /* Disable the UART Transmit Data Register Empty Interrupt */
          ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE_TXFNFIE);

          /* Enable the UART Transmit Complete Interrupt */
          ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_TCIE);
      }
      else if (READ_BIT(huart->Instance->ISR, USART_ISR_TXE_TXFNF) != 0U)
      {
        huart->Instance->TDR = (uint8_t)(*huart->pTxBuffPtr & (uint8_t)0xFF);
        huart->pTxBuffPtr++;
        huart->TxXferCount--;
      }
      else
      {
        /* Nothing to do */
      }
  }
}



/**
  * This function starts the transmit opperation
  *   In general, the first character is sent here, and the remaining characters get sent
  *   by the transmit interrupt.
  */
HAL_StatusTypeDef UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
  /* Check that a Tx process is not already ongoing */
  if (huart->gState == HAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return HAL_ERROR;
    }

    __HAL_LOCK(huart);

    huart->pTxBuffPtr  = pData;
    huart->TxXferSize  = Size;
    huart->TxXferCount = Size;
    //huart->TxISR       = NULL;

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->gState = HAL_UART_STATE_BUSY_TX;

    __HAL_UNLOCK(huart);

    /* Enable the Transmit Data Register Empty interrupt */
    ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_TXEIE_TXFNFIE);
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

bool isRs485txReady(void)
{
   bool retVal = false;
   if (rs485uart.gState == HAL_UART_STATE_READY)
   {
       retVal = true;
   }
   return(retVal);
}



/*********************************************************************************************/
/*********************************************************************************************/
/*                        INTERUPT FUNCTIONS FOR RS-485 FOR RX                               */
/*********************************************************************************************/
/*********************************************************************************************/

/* This is the RX interrupt, called from the main Uart 1 interrupt
 *
 */
void rs485RxInterrupt(UART_HandleTypeDef *huart)
{
    uint16_t  uhMask = huart->Mask;
    uint16_t  uhdata;
    uint32_t  isrflags = READ_REG(huart->Instance->ISR);
    uint32_t  cr1its   = READ_REG(huart->Instance->CR1);
    uint32_t  cr3its   = READ_REG(huart->Instance->CR3);

    /* Check that a Rx process is ongoing */
    if (huart->RxState == HAL_UART_STATE_BUSY_RX)
    {
        while ((isrflags & USART_ISR_RXNE_RXFNE) != 0U)
        {
            uhdata = (uint16_t) READ_REG(huart->Instance->RDR);
            if(huart->RxXferCount < (huart->RxXferSize - 1))
            {  // Only add to the buffer if it is not full
                *huart->pRxBuffPtr++ = (uint8_t)(uhdata & (uint8_t)uhMask);
                if(huart->pRxBuffPtr >= (rs485rxBuf + RS485_BUF_SIZE))
                {
                    huart->pRxBuffPtr = rs485rxBuf;
                }
                huart->RxXferCount++;
            }
            isrflags = READ_REG(huart->Instance->ISR);

            /* If some non blocking errors occurred */
            if ((isrflags & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE)) != 0U)
            {
                /* UART parity error interrupt occurred -------------------------------------*/
                if (((isrflags & USART_ISR_PE) != 0U) && ((cr1its & USART_CR1_PEIE) != 0U))
                {
                    huart->Instance->ICR = UART_CLEAR_PEF;
                    huart->ErrorCode |= HAL_UART_ERROR_PE;
                }

                /* UART frame error interrupt occurred --------------------------------------*/
                if (((isrflags & USART_ISR_FE) != 0U) && ((cr3its & USART_CR3_EIE) != 0U))
                {
                    huart->Instance->ICR = UART_CLEAR_FEF;
                    huart->ErrorCode |= HAL_UART_ERROR_FE;
                }

                /* UART noise error interrupt occurred --------------------------------------*/
                if (((isrflags & USART_ISR_NE) != 0U) && ((cr3its & USART_CR3_EIE) != 0U))
                {
                    huart->Instance->ICR = UART_CLEAR_NEF;
                    huart->ErrorCode |= UART_ERROR_NE;
                }
            }
        }

    }
    else
    {
        /* Clear RXNE interrupt flag */
        __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    }
}


/* This function is called to find out how many characters are in the
 *  receive buffer
 */
uint16_t getRs485RxbufSize(void)
{
    return(rs485uart.RxXferCount);
}


/* Get one character from the RS485 RX buffet
 *   Call this function unly if you know there is at least
 *   one character in the buffer.
 */
uint8_t getNextCharFromRs485rxBuf(void)
{
    uint8_t retVal = 0;

    if(rs485uart.RxXferCount > 0)
    {
        __HAL_UART_DISABLE_IT(&rs485uart, UART_IT_RXNE);
        retVal = *rs485rxBufOutPtr++;
        if(rs485rxBufOutPtr >= (rs485rxBuf + RS485_BUF_SIZE))
        {
            rs485rxBufOutPtr = rs485rxBuf;
        }
        rs485uart.RxXferCount--;
        __HAL_UART_ENABLE_IT(&rs485uart, UART_IT_RXNE);
    }
    return(retVal);
}



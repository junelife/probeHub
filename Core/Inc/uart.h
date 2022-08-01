/*
 * uart.h
 *
 * Copyright © Weber Stephen Products LLC., 2022 All Rights Reserved No part of this file may be
 * distributed, reproduced, or used by entities other than Stephen Products LLC. without express
 * written permission. All Rights Reserved UNPUBLISHED, LICENSED SOFTWARE IS CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Weber Stephen Products LLC.
 *
 *  Created on: Jun 9, 2022
 *      Author: ronso
 *
 *  This file contains definitions used in the uart for RS-485 code.
 *    The definitions are mostly direct copies of items that were included using CUBEMX
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#define UART_IT_MASK                        0x001FU  /*!< UART interruptions flags mask */


/** @brief  Disable the specified UART interrupt.
  * @param  __HANDLE__ specifies the UART Handle.
  * @param  __INTERRUPT__ specifies the UART interrupt source to disable.
  *          This parameter can be one of the following values:
  *            @arg @ref UART_IT_RXFF  RXFIFO Full interrupt
  *            @arg @ref UART_IT_TXFE  TXFIFO Empty interrupt
  *            @arg @ref UART_IT_RXFT  RXFIFO threshold interrupt
  *            @arg @ref UART_IT_TXFT  TXFIFO threshold interrupt
  *            @arg @ref UART_IT_WUF   Wakeup from stop mode interrupt
  *            @arg @ref UART_IT_CM    Character match interrupt
  *            @arg @ref UART_IT_CTS   CTS change interrupt
  *            @arg @ref UART_IT_LBD   LIN Break detection interrupt
  *            @arg @ref UART_IT_TXE   Transmit Data Register empty interrupt
  *            @arg @ref UART_IT_TXFNF TX FIFO not full interrupt
  *            @arg @ref UART_IT_TC    Transmission complete interrupt
  *            @arg @ref UART_IT_RXNE  Receive Data register not empty interrupt
  *            @arg @ref UART_IT_RXFNE RXFIFO not empty interrupt
  *            @arg @ref UART_IT_RTO   Receive Timeout interrupt
  *            @arg @ref UART_IT_IDLE  Idle line detection interrupt
  *            @arg @ref UART_IT_PE    Parity Error interrupt
  *            @arg @ref UART_IT_ERR   Error interrupt (Frame error, noise error, overrun error)
  * @retval None
  */
#define __HAL_UART_DISABLE_IT(__HANDLE__, __INTERRUPT__)  (\
                                                           ((((uint8_t)(__INTERRUPT__)) >> 5U) == 1U)?\
                                                           ((__HANDLE__)->Instance->CR1 &= ~ (1U <<\
                                                               ((__INTERRUPT__) & UART_IT_MASK))): \
                                                           ((((uint8_t)(__INTERRUPT__)) >> 5U) == 2U)?\
                                                           ((__HANDLE__)->Instance->CR2 &= ~ (1U <<\
                                                               ((__INTERRUPT__) & UART_IT_MASK))): \
                                                           ((__HANDLE__)->Instance->CR3 &= ~ (1U <<\
                                                               ((__INTERRUPT__) & UART_IT_MASK))))

#define __HAL_UART_ENABLE_IT(__HANDLE__, __INTERRUPT__)   (\
                                                           ((((uint8_t)(__INTERRUPT__)) >> 5U) == 1U)?\
                                                           ((__HANDLE__)->Instance->CR1 |= (1U <<\
                                                               ((__INTERRUPT__) & UART_IT_MASK))): \
                                                           ((((uint8_t)(__INTERRUPT__)) >> 5U) == 2U)?\
                                                           ((__HANDLE__)->Instance->CR2 |= (1U <<\
                                                               ((__INTERRUPT__) & UART_IT_MASK))): \
                                                           ((__HANDLE__)->Instance->CR3 |= (1U <<\
                                                               ((__INTERRUPT__) & UART_IT_MASK))))

/** @brief  Enable UART.
  * @param  __HANDLE__ specifies the UART Handle.
  * @retval None
  */
#define __HAL_UART_ENABLE(__HANDLE__)                   ((__HANDLE__)->Instance->CR1 |= USART_CR1_UE)

/** @brief  Disable UART.
  * @param  __HANDLE__ specifies the UART Handle.
  * @retval None
  */
#define __HAL_UART_DISABLE(__HANDLE__)                  ((__HANDLE__)->Instance->CR1 &= ~USART_CR1_UE)

/** @brief  Set a specific UART request flag.
  * @param  __HANDLE__ specifies the UART Handle.
  * @param  __REQ__ specifies the request flag to set
  *          This parameter can be one of the following values:
  *            @arg @ref UART_AUTOBAUD_REQUEST Auto-Baud Rate Request
  *            @arg @ref UART_SENDBREAK_REQUEST Send Break Request
  *            @arg @ref UART_MUTE_MODE_REQUEST Mute Mode Request
  *            @arg @ref UART_RXDATA_FLUSH_REQUEST Receive Data flush Request
  *            @arg @ref UART_TXDATA_FLUSH_REQUEST Transmit data flush Request
  * @retval None
  */
#define __HAL_UART_SEND_REQ(__HANDLE__, __REQ__) ((__HANDLE__)->Instance->RQR |= (uint16_t)(__REQ__))


/** @brief  Report the UART mask to apply to retrieve the received data
  *         according to the word length and to the parity bits activation.
  * @note   If PCE = 1, the parity bit is not included in the data extracted
  *         by the reception API().
  *         This masking operation is not carried out in the case of
  *         DMA transfers.
  * @param  __HANDLE__ specifies the UART Handle.
  * @retval None, the mask to apply to UART RDR register is stored in (__HANDLE__)->Mask field.
  */


/** @defgroup UART_Error_Definition   UART Error Definition
  * @{
  */
#define  HAL_UART_ERROR_NONE             (0x00000000U)    /*!< No error                */
#define  HAL_UART_ERROR_PE               (0x00000001U)    /*!< Parity error            */
#define  HAL_UART_ERROR_NE               (0x00000002U)    /*!< Noise error             */
#define  HAL_UART_ERROR_FE               (0x00000004U)    /*!< Frame error             */
#define  HAL_UART_ERROR_ORE              (0x00000008U)    /*!< Overrun error           */
#define  HAL_UART_ERROR_DMA              (0x00000010U)    /*!< DMA transfer error      */
#define  HAL_UART_ERROR_RTO              (0x00000020U)    /*!< Receiver Timeout error  */

/** @defgroup UART_IT_CLEAR_Flags  UART Interruption Clear Flags
  * @{
  */
#define UART_CLEAR_PEF                       USART_ICR_PECF            /*!< Parity Error Clear Flag           */
#define UART_CLEAR_FEF                       USART_ICR_FECF            /*!< Framing Error Clear Flag          */
#define UART_CLEAR_NEF                       USART_ICR_NECF            /*!< Noise Error detected Clear Flag   */
#define UART_CLEAR_OREF                      USART_ICR_ORECF           /*!< Overrun Error Clear Flag          */
#define UART_CLEAR_IDLEF                     USART_ICR_IDLECF          /*!< IDLE line detected Clear Flag     */
#define UART_CLEAR_TXFECF                    USART_ICR_TXFECF          /*!< TXFIFO empty clear flag           */
#define UART_CLEAR_TCF                       USART_ICR_TCCF            /*!< Transmission Complete Clear Flag  */
#define UART_CLEAR_LBDF                      USART_ICR_LBDCF           /*!< LIN Break Detection Clear Flag    */
#define UART_CLEAR_CTSF                      USART_ICR_CTSCF           /*!< CTS Interrupt Clear Flag          */
#define UART_CLEAR_CMF                       USART_ICR_CMCF            /*!< Character Match Clear Flag        */
#define UART_CLEAR_WUF                       USART_ICR_WUCF            /*!< Wake Up from stop mode Clear Flag */
#define UART_CLEAR_RTOF                      USART_ICR_RTOCF           /*!< UART receiver timeout clear flag  */

/** @defgroup UART_RECEPTION_TYPE_Values  UART Reception type values
  * @{
  */
#define HAL_UART_RECEPTION_STANDARD          (0x00000000U)             /*!< Standard reception                       */
#define HAL_UART_RECEPTION_TOIDLE            (0x00000001U)             /*!< Reception till completion or IDLE event  */
#define HAL_UART_RECEPTION_TORTO             (0x00000002U)             /*!< Reception till completion or RTO event   */
#define HAL_UART_RECEPTION_TOCHARMATCH       (0x00000003U)             /*!< Reception till completion or CM event    */


#define USART_CR1_FIELDS  ((uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE | \
                                      USART_CR1_OVER8 | USART_CR1_FIFOEN)) /*!< UART or USART CR1 fields of parameters set by UART_SetConfig API */

#define USART_CR3_FIELDS  ((uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_ONEBIT | USART_CR3_TXFTCFG | \
                                      USART_CR3_RXFTCFG)) /*!< UART or USART CR3 fields of parameters set by UART_SetConfig API */

/**
  * @brief UART clock sources definition
  */
#ifndef UART_GETCLOCKSOURCE
typedef enum
{
  UART_CLOCKSOURCE_PCLK1      = 0x00U,    /*!< PCLK1 clock source  */
  UART_CLOCKSOURCE_HSI        = 0x02U,    /*!< HSI clock source    */
  UART_CLOCKSOURCE_SYSCLK     = 0x04U,    /*!< SYSCLK clock source */
  UART_CLOCKSOURCE_LSE        = 0x08U,    /*!< LSE clock source       */
  UART_CLOCKSOURCE_UNDEFINED  = 0x10U     /*!< Undefined clock source */
} UART_ClockSourceTypeDef;
#endif

/** @defgroup UART_State_Definition UART State Code Definition
  * @{
  */
#define  HAL_UART_STATE_RESET         0x00000000U    /*!< Peripheral is not initialized
                                                          Value is allowed for gState and RxState */
#define  HAL_UART_STATE_READY         0x00000020U    /*!< Peripheral Initialized and ready for use
                                                          Value is allowed for gState and RxState */
#define  HAL_UART_STATE_BUSY          0x00000024U    /*!< an internal process is ongoing
                                                          Value is allowed for gState only */
#define  HAL_UART_STATE_BUSY_TX       0x00000021U    /*!< Data Transmission process is ongoing
                                                          Value is allowed for gState only */
#define  HAL_UART_STATE_BUSY_RX       0x00000022U    /*!< Data Reception process is ongoing
                                                          Value is allowed for RxState only */
#define  HAL_UART_STATE_BUSY_TX_RX    0x00000023U    /*!< Data Transmission and Reception process is ongoing
                                                          Not to be used for neither gState nor RxState.Value is result
                                                          of combination (Or) between gState and RxState values */
#define  HAL_UART_STATE_TIMEOUT       0x000000A0U    /*!< Timeout state
                                                          Value is allowed for gState only */
#define  HAL_UART_STATE_ERROR         0x000000E0U    /*!< Error */


/** @defgroup UART_DriverEnable_Polarity      UART DriverEnable Polarity
  *
  */
#define UART_DE_POLARITY_HIGH               0x00000000U             /*!< Driver enable signal is active high */
#define UART_DE_POLARITY_LOW                USART_CR3_DEP           /*!< Driver enable signal is active low  */

#define UART_FIFOMODE_DISABLE       0x00000000U       /*!< FIFO mode disable */
#define UART_FIFOMODE_ENABLE        USART_CR1_FIFOEN  /*!< FIFO mode enable  */

/** @defgroup UART_Interrupt_definition   UART Interrupts Definition
  *        Elements values convention: 000ZZZZZ0XXYYYYYb
  *           - YYYYY  : Interrupt source position in the XX register (5bits)
  *           - XX  : Interrupt source register (2bits)
  *                 - 01: CR1 register
  *                 - 10: CR2 register
  *                 - 11: CR3 register
  *           - ZZZZZ  : Flag position in the ISR register(5bits)
  *        Elements values convention: 000000000XXYYYYYb
  *           - YYYYY  : Interrupt source position in the XX register (5bits)
  *           - XX  : Interrupt source register (2bits)
  *                 - 01: CR1 register
  *                 - 10: CR2 register
  *                 - 11: CR3 register
  *        Elements values convention: 0000ZZZZ00000000b
  *           - ZZZZ  : Flag position in the ISR register(4bits)
  * @{
  */
#define UART_IT_PE                          0x0028U              /*!< UART parity error interruption                 */
#define UART_IT_TXE                         0x0727U              /*!< UART transmit data register empty interruption */
#define UART_IT_TXFNF                       0x0727U              /*!< UART TX FIFO not full interruption             */
#define UART_IT_TC                          0x0626U              /*!< UART transmission complete interruption        */
#define UART_IT_RXNE                        0x0525U              /*!< UART read data register not empty interruption */
#define UART_IT_RXFNE                       0x0525U              /*!< UART RXFIFO not empty interruption             */
#define UART_IT_IDLE                        0x0424U              /*!< UART idle interruption                         */
#define UART_IT_LBD                         0x0846U              /*!< UART LIN break detection interruption          */
#define UART_IT_CTS                         0x096AU              /*!< UART CTS interruption                          */
#define UART_IT_CM                          0x112EU              /*!< UART character match interruption              */
#define UART_IT_WUF                         0x1476U              /*!< UART wake-up from stop mode interruption       */
#define UART_IT_RXFF                        0x183FU              /*!< UART RXFIFO full interruption                  */
#define UART_IT_TXFE                        0x173EU              /*!< UART TXFIFO empty interruption                 */
#define UART_IT_RXFT                        0x1A7CU              /*!< UART RXFIFO threshold reached interruption     */
#define UART_IT_TXFT                        0x1B77U              /*!< UART TXFIFO threshold reached interruption     */
#define UART_IT_RTO                         0x0B3AU              /*!< UART receiver timeout interruption             */

#define UART_IT_ERR                         0x0060U              /*!< UART error interruption                        */

#define UART_IT_ORE                         0x0300U              /*!< UART overrun error interruption                */
#define UART_IT_NE                          0x0200U              /*!< UART noise error interruption                  */
#define UART_IT_FE                          0x0100U              /*!< UART frame error interruption                  */


/** @defgroup UART_Request_Parameters UART Request Parameters
  * @{
  */
#define UART_AUTOBAUD_REQUEST               USART_RQR_ABRRQ        /*!< Auto-Baud Rate Request      */
#define UART_SENDBREAK_REQUEST              USART_RQR_SBKRQ        /*!< Send Break Request          */
#define UART_MUTE_MODE_REQUEST              USART_RQR_MMRQ         /*!< Mute Mode Request           */
#define UART_RXDATA_FLUSH_REQUEST           USART_RQR_RXFRQ        /*!< Receive Data flush Request  */
#define UART_TXDATA_FLUSH_REQUEST           USART_RQR_TXFRQ        /*!< Transmit data flush Request */

/** @defgroup UARTEx_TXFIFO_threshold_level UARTEx TXFIFO threshold level
  * @brief    UART TXFIFO threshold level
  * @{
  */
#define UART_TXFIFO_THRESHOLD_1_8   0x00000000U                               /*!< TX FIFO reaches 1/8 of its depth */
#define UART_TXFIFO_THRESHOLD_1_4   USART_CR3_TXFTCFG_0                       /*!< TX FIFO reaches 1/4 of its depth */
#define UART_TXFIFO_THRESHOLD_1_2   USART_CR3_TXFTCFG_1                       /*!< TX FIFO reaches 1/2 of its depth */
#define UART_TXFIFO_THRESHOLD_3_4   (USART_CR3_TXFTCFG_0|USART_CR3_TXFTCFG_1) /*!< TX FIFO reaches 3/4 of its depth */
#define UART_TXFIFO_THRESHOLD_7_8   USART_CR3_TXFTCFG_2                       /*!< TX FIFO reaches 7/8 of its depth */
#define UART_TXFIFO_THRESHOLD_8_8   (USART_CR3_TXFTCFG_2|USART_CR3_TXFTCFG_0) /*!< TX FIFO becomes empty            */

/** @defgroup UARTEx_RXFIFO_threshold_level UARTEx RXFIFO threshold level
  * @brief    UART RXFIFO threshold level
  * @{
  */
#define UART_RXFIFO_THRESHOLD_1_8   0x00000000U                               /*!< RX FIFO reaches 1/8 of its depth */
#define UART_RXFIFO_THRESHOLD_1_4   USART_CR3_RXFTCFG_0                       /*!< RX FIFO reaches 1/4 of its depth */
#define UART_RXFIFO_THRESHOLD_1_2   USART_CR3_RXFTCFG_1                       /*!< RX FIFO reaches 1/2 of its depth */
#define UART_RXFIFO_THRESHOLD_3_4   (USART_CR3_RXFTCFG_0|USART_CR3_RXFTCFG_1) /*!< RX FIFO reaches 3/4 of its depth */
#define UART_RXFIFO_THRESHOLD_7_8   USART_CR3_RXFTCFG_2                       /*!< RX FIFO reaches 7/8 of its depth */
#define UART_RXFIFO_THRESHOLD_8_8   (USART_CR3_RXFTCFG_2|USART_CR3_RXFTCFG_0) /*!< RX FIFO becomes full             */

/** @defgroup UART_Mode UART Transfer Mode
  * @{
  */
#define UART_MODE_RX                        USART_CR1_RE                    /*!< RX mode        */
#define UART_MODE_TX                        USART_CR1_TE                    /*!< TX mode        */
#define UART_MODE_TX_RX                     (USART_CR1_TE |USART_CR1_RE)    /*!< RX and TX mode */
#define UART_BRR_MIN    0x10U        /* UART BRR minimum authorized value */
#define UART_BRR_MAX    0x0000FFFFU  /* UART BRR maximum authorized value */

#ifndef UART_GETCLOCKSOURCE

#define UART_MASK_COMPUTATION(__HANDLE__)                             \
  do {                                                                \
        (__HANDLE__)->Mask = 0x00FFU ;                                \
                                                                      \
  } while(0U)

/** @brief  BRR division operation to set BRR register in 16-bit oversampling mode.
  * @param  __PCLK__ UART clock.
  * @param  __BAUD__ Baud rate set by the user.
  * @param  __CLOCKPRESCALER__ UART prescaler value.
  * @retval Division result
  */

#define UART_DIV_SAMPLING16(__PCLK__, __BAUD__, __CLOCKPRESCALER__)                       \
  ((((__PCLK__)/UARTPrescTable[(__CLOCKPRESCALER__)]) + ((__BAUD__)/2U)) / (__BAUD__))


#define UART_GETCLOCKSOURCE(__HANDLE__,__CLOCKSOURCE__)       \
  do {                                                        \
      switch(__HAL_RCC_GET_USART1_SOURCE())                   \
      {                                                       \
        case RCC_USART1CLKSOURCE_PCLK1:                       \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_PCLK1;         \
          break;                                              \
        case RCC_USART1CLKSOURCE_HSI:                         \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_HSI;           \
          break;                                              \
        case RCC_USART1CLKSOURCE_SYSCLK:                      \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_SYSCLK;        \
          break;                                              \
        case RCC_USART1CLKSOURCE_LSE:                         \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_LSE;           \
          break;                                              \
        default:                                              \
          (__CLOCKSOURCE__) = UART_CLOCKSOURCE_UNDEFINED;     \
          break;                                              \
      }                                                       \
  } while(0U)
#endif


#ifndef UART_GETCLOCKSOURCE
/**
  * @brief UART Init Structure definition
  */
typedef struct
{
  uint32_t BaudRate;                /*!< This member configures the UART communication baud rate.
                                         The baud rate register is computed using the following formula:
                                         LPUART:
                                         =======
                                         Baud Rate Register = ((256 * lpuart_ker_ckpres) / ((huart->Init.BaudRate)))
                                         where lpuart_ker_ck_pres is the UART input clock divided by a prescaler
                                         UART:
                                         =====
                                         - If oversampling is 16 or in LIN mode,
                                            Baud Rate Register = ((uart_ker_ckpres) / ((huart->Init.BaudRate)))
                                         - If oversampling is 8,
                                            Baud Rate Register[15:4] = ((2 * uart_ker_ckpres) /
                                            ((huart->Init.BaudRate)))[15:4]
                                            Baud Rate Register[3] =  0
                                            Baud Rate Register[2:0] =  (((2 * uart_ker_ckpres) /
                                            ((huart->Init.BaudRate)))[3:0]) >> 1
                                         where uart_ker_ck_pres is the UART input clock divided by a prescaler */

  uint32_t WordLength;              /*!< Specifies the number of data bits transmitted or received in a frame.
                                         This parameter can be a value of @ref UARTEx_Word_Length. */

  uint32_t StopBits;                /*!< Specifies the number of stop bits transmitted.
                                         This parameter can be a value of @ref UART_Stop_Bits. */

  uint32_t Parity;                  /*!< Specifies the parity mode.
                                         This parameter can be a value of @ref UART_Parity
                                         @note When parity is enabled, the computed parity is inserted
                                               at the MSB position of the transmitted data (9th bit when
                                               the word length is set to 9 data bits; 8th bit when the
                                               word length is set to 8 data bits). */

  uint32_t Mode;                    /*!< Specifies whether the Receive or Transmit mode is enabled or disabled.
                                         This parameter can be a value of @ref UART_Mode. */

  uint32_t HwFlowCtl;               /*!< Specifies whether the hardware flow control mode is enabled
                                         or disabled.
                                         This parameter can be a value of @ref UART_Hardware_Flow_Control. */

  uint32_t OverSampling;            /*!< Specifies whether the Over sampling 8 is enabled or disabled,
                                         to achieve higher speed (up to f_PCLK/8).
                                         This parameter can be a value of @ref UART_Over_Sampling. */

  uint32_t OneBitSampling;          /*!< Specifies whether a single sample or three samples' majority vote is selected.
                                         Selecting the single sample method increases the receiver tolerance to clock
                                         deviations. This parameter can be a value of @ref UART_OneBit_Sampling. */

  uint32_t ClockPrescaler;          /*!< Specifies the prescaler value used to divide the UART clock source.
                                         This parameter can be a value of @ref UART_ClockPrescaler. */

} UART_InitTypeDef;



typedef struct __UART_HandleTypeDef
{
  USART_TypeDef            *Instance;                /*!< UART registers base address        */

  UART_InitTypeDef         Init;                     /*!< UART communication parameters      */

  uint32_t                 AdvancedInit;           /*!< UART Advanced Features initialization parameters */

  const uint8_t            *pTxBuffPtr;              /*!< Pointer to UART Tx transfer Buffer */

  uint16_t                 TxXferSize;               /*!< UART Tx Transfer size              */

  volatile uint16_t            TxXferCount;              /*!< UART Tx Transfer Counter           */

  uint8_t                  *pRxBuffPtr;              /*!< Pointer to UART Rx transfer Buffer */

  uint16_t                 RxXferSize;               /*!< UART Rx Transfer size              */

  volatile uint16_t            RxXferCount;              /*!< UART Rx Transfer Counter           */

  uint16_t                 Mask;                     /*!< UART Rx RDR register mask          */

  uint32_t                 FifoMode;                 /*!< Specifies if the FIFO mode is being used.
                                                          This parameter can be a value of @ref UARTEx_FIFO_mode. */

  uint16_t                 NbRxDataToProcess;        /*!< Number of data to process during RX ISR execution */

  uint16_t                 NbTxDataToProcess;        /*!< Number of data to process during TX ISR execution */

  volatile uint32_t        ReceptionType;         /*!< Type of ongoing reception          */

  void (*RxISR)(struct __UART_HandleTypeDef *huart); /*!< Function pointer on Rx IRQ handler */

  void (*TxISR)(struct __UART_HandleTypeDef *huart); /*!< Function pointer on Tx IRQ handler */

  DMA_HandleTypeDef        *hdmatx;                  /*!< UART Tx DMA Handle parameters      */

  DMA_HandleTypeDef        *hdmarx;                  /*!< UART Rx DMA Handle parameters      */

  HAL_LockTypeDef           Lock;                    /*!< Locking object                     */

  volatile uint32_t        gState;              /*!< UART state information related to global Handle management
                                                          and also related to Tx operations. This parameter
                                                          can be a value of @ref HAL_UART_StateTypeDef */

  volatile uint32_t        RxState;             /*!< UART state information related to Rx operations. This
                                                          parameter can be a value of @ref HAL_UART_StateTypeDef */

  volatile uint32_t                 ErrorCode;           /*!< UART Error code                    */

} UART_HandleTypeDef;

#endif

#endif /* INC_UART_H_ */

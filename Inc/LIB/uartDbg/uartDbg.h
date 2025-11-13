/**
* @file        uartDbg.h
*
* @brief       Uart for debug  - Definition -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: uartDbg.h 701 2025-02-20 16:05:12Z luca $
*
*     $Revision: 701 $
*
*     $Author: luca $
*
*     $Date: 2025-02-20 17:05:12 +0100 (gio, 20 feb 2025) $
*
*
* @copyright
*       Copyright (C) 2016 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

#ifndef __UART_DBG_H 
#define __UART_DBG_H

/************************************************************
 * Include
 ************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include "cmsis_os.h"
#ifdef GD32F4xx 
#include "stm32f4xx_periph_init.h"
#else
#include "stm32h5xx_periph_init.h"
#endif

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#ifndef TRUE
#define TRUE            ((uint8_t)1)
#endif

#ifndef FALSE
#define FALSE           ((uint8_t)0)
#endif

/* Definition for USART DBG:  We use UART6 PC6=Tx and PC7=Rx in case of STM32F4xx */
/* and UART7 PE7=Rx and PE8=Tx in case of STM32F7xx                              */

#ifdef GD32F4xx

#define UART_DBG                           USART6
#define UART_DBG_HANDLE                    huart6
#define UART_DBG_DEFAULT_BR                ((uint32_t)115200)
#define UART_DBB_CLK_ENABLE()              __HAL_RCC_USART6_CLK_ENABLE() 
#define UART_DBB_CLK_DISABLE()             __HAL_RCC_USART6_CLK_ENABLE() 
#define UART_DBG_RX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()
#define UART_DBG_TX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()
#define DMAx_DBG_CLK_ENABLE()              __DMA1_CLK_ENABLE()

/* Definition for USARTx Pins */
#define UART_DBG_TX_PIN                    GPIO_PIN_6
#define UART_DBG_TX_GPIO_PORT              GPIOC
#define UART_DBG_TX_AF                     GPIO_AF8_USART6
#define UART_DBG_RX_PIN                    GPIO_PIN_7
#define UART_DBG_RX_GPIO_PORT              GPIOC
#define UART_DBG_RX_AF                     GPIO_AF8_USART6

/* Definition for USARTx's DMA */
#define UART_DBG_TX_DMA_STREAM              DMA2_Stream6
#define UART_DBG_RX_DMA_STREAM              DMA2_Stream2
#define UART_DBG_RX_DMA_TC_FLAG             DMA_LISR_TCIF2


#define UART_DBG_TX_DMA_CHANNEL             DMA_CHANNEL_5
#define UART_DBG_RX_DMA_CHANNEL             DMA_CHANNEL_5

/* Definition for USARTx's NVIC */
#define UART_DBG_DMA                        DMA2
/* Stream for Rx */
#define UART_DBG_DMA_RX_IRQn                DMA2_Stream2_IRQn
#define UART_DBG_DMA_RX_IRQHandler          DMA2_Stream2_IRQHandler
/* Stream for Tx */
#define UART_DBG_DMA_TX_IRQn                DMA2_Stream6_IRQn
#define UART_DBG_DMA_TX_IRQHandler          DMA2_Stream6_IRQHandler

/* Definition for USARTx's NVIC */
#define UART_DBG_IRQn                       USART6_IRQn
#define UART_DBG_IRQHandler                 USART6_IRQHandler

#else

#define UART_DBG                           USART6
#define UART_DBG_HANDLE                    huart6
#define UART_DBG_DEFAULT_BR                ((uint32_t)115200)
#define UART_DBB_CLK_ENABLE()              __HAL_RCC_USART6_CLK_ENABLE() 
#define UART_DBB_CLK_DISABLE()             __HAL_RCC_USART6_CLK_DISABLE() 
#define UART_DBG_RX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()
#define UART_DBG_TX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()


/* Definition for USARTx Pins */
#define UART_DBG_TX_PIN                    GPIO_PIN_6
#define UART_DBG_TX_GPIO_PORT              GPIOC
#define UART_DBG_RX_PIN                    GPIO_PIN_7
#define UART_DBG_RX_GPIO_PORT              GPIOC

/* Definition for USARTx's DMA */
#define UART_DBG_TX_DMA_STREAM              GPDMA2_Channel5
#define UART_DBG_RX_DMA_STREAM              GPDMA2_Channel2
#define UART_DBG_RX_DMA_TC_FLAG             DMA_LISR_TCIF3


/* Definition for USARTx's NVIC */
#define UART_DBG_DMA                        GPDMA2
/* Stream for Rx */
#define UART_DBG_DMA_RX_IRQn                GPDMA2_Channel2_IRQn
#define UART_DBG_DMA_RX_IRQHandler          GPDMA2_Channel2_IRQHandler
/* Stream for Tx */
#define UART_DBG_DMA_TX_IRQn                GPDMA2_Channel5_IRQn
#define UART_DBG_DMA_TX_IRQHandler          GPDMA2_Channel5_IRQHandler

/* Definition for USARTx's NVIC */
#define UART_DBG_IRQn                       USART6_IRQn
#define UART_DBG_IRQHandler                 USART6_IRQHandler

#endif

/*** default baude rate   ***/
#define UART_DBG_BR_DEFAULT			((uint32_t)115200)




/* ----------- numero buffer nella coda di ricezione  -------------*/
#define   NUM_BUFF_DBG_RX                   ((uint16_t)8)      

/* ----------- Define per automa di ricezione console seriale -------------*/
typedef enum
{
    WAIT_END1 = 0,
    WAIT_END2,
    ERROR_DBG_STATE = 0xFFFF
} stateDbg_e;


 
typedef __packed struct
{
  stateDbg_e      cRxStatus;
  uint16_t        rxIndex;
  uint16_t        nLengthRx;
} msgBurstDbgRx_st;

/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
xQueueHandle      getDbgQueueHandle                 (void);
void              HAL_UART_DBG_RxCpltCallback       (UART_HandleTypeDef *huart);
void              HAL_UART_DBG_TxCpltCallback       (UART_HandleTypeDef *huart);
void              MX_DBG_UART_Init                  (void);
void              HAL_UART_DBG_Rx_Ena_IT            (void);
void              debug_IRQHandler                  (void);
void              rxDbgBurstActivation              (void);
void              UART_DBG_RxCpltCallback_DMA       (void); 
void              UART_DBG_DMA_Tx                   (uint8_t* pTxBuffer, uint16_t len); 
void              burstReceptionDbgMng              (uint8_t* pRxChar, uint16_t numBytes);
void              rxDBGBurstActivation              (void);
void              MX_DBG_UART_DeInit                (void);
HAL_StatusTypeDef UART_DBG_IDLE_Tx                  (void); 
void              UART_DBG_DMA_IRQHandler           (void); 

void              UART_EM_DMA_Tx                    (uint8_t* pTxBuffer, uint16_t len); 
void              UART1_PROT_DMA_Tx                 (uint8_t* pTxBuffer, uint16_t len); 
void              set_DMA2_s7_hdmatx                (uint8_t intf);
void              UART2_PROT_IRQHandler             (void); 

#endif //  __UART_DBG_H


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
*     $Id: Em_uart.h 646 2024-12-11 10:06:21Z npiergi $
*
*     $Revision: 646 $
*
*     $Author: npiergi $
*
*     $Date: 2024-12-11 11:06:21 +0100 (mer, 11 dic 2024) $
*
*
* @copyright
*       Copyright (C) 2016 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

#ifndef __EM_UART_H 
#define __EM_UART_H

/************************************************************
 * Include
 ************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include "cmsis_os.h"
 
 
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

/********* -----------  USART2 -- Energy Meter -------- ***************************/
/* Definition for USART PI2  clock resources We use UART2 PD6=Rx and PD5=Tx  PD4=DE*/
#define UART_EM                            USART2
#define UART_EM_HANDLE                     huart2
#define UART_EM_DEFAULT_BR                 ((uint32_t)19200)
#define UART_EM_MINIMUM_BR                 ((uint32_t)9600)
#define UART_EM_CLK_ENABLE()               __HAL_RCC_USART2_CLK_ENABLE() 
#define UART_EM_RX_GPIO_CLK_ENABLE()       __GPIOD_CLK_ENABLE()
#define UART_EM_TX_GPIO_CLK_ENABLE()       __GPIOD_CLK_ENABLE()
#define DMAx_EM_CLK_ENABLE()               __DMA1_CLK_ENABLE()

/* Definition for USARTx Pins */
#define UART_EM_TX_PIN                     GPIO_PIN_6
#define UART_EM_TX_GPIO_PORT               GPIOD
#define UART_EM_TX_AF                      GPIO_AF7_USART2
#define UART_EM_RX_PIN                     GPIO_PIN_5
#define UART_EM_RX_GPIO_PORT               GPIOD
#define UART_EM_RX_AF                      GPIO_AF7_USART2
#define UART_EM_DE_PIN                     GPIO_PIN_4
#define UART_EM_DE_GPIO_PORT               GPIOD
#define UART_EM_DE_AF                      GPIO_AF7_USART2

/* Definition for USARTx's DMA */
#ifdef GD32F4xx
#define UART_EM_TX_DMA_STREAM               DMA1_Stream6
#define UART_EM_RX_DMA_STREAM               DMA1_Stream5
#define UART_EM_TX_DMA_CHANNEL              DMA_CHANNEL_4
#define UART_EM_RX_DMA_CHANNEL              DMA_CHANNEL_4
#else
#define UART_EM_TX_DMA_STREAM               GPDMA1_Channel5    
#define UART_EM_RX_DMA_STREAM               GPDMA1_Channel3    
#endif

/* Definition for USARTx's NVIC */
#define UART_EM_DMA                         DMA1
#define UART_EM_DMA_STREAM                  DMA1_Stream5
#define UART_EM_DMA_TX_IRQn                 DMA1_Stream6_IRQn
#define UART_EM_DMA_RX_IRQn                 DMA1_Stream5_IRQn
#define UART_EM_DMA_TX_IRQHandler           DMA1_Stream6_IRQHandler
#define UART_EM_DMA_RX_IRQHandler           DMA1_Stream5_IRQHandler

/* Definition for USARTx's NVIC */
#define UART_EM_IRQn                        USART2_IRQn
#define UART_EM_IRQHandler                  USART2_IRQHandler

/*** default baude rate   ***/
#define UART_EM_BR_DEFAULT			    ((uint32_t)9600)
#define RS485_EM_ASSERTION_TIME     ((uint32_t)8)
#define RS485_EM_DEASSERTION_TIME   ((uint32_t)4)



/* ----------- numero buffer nella coda di ricezione  -------------*/
#define   NUM_BUFF_EM_RX                    ((uint16_t)2)      
#define   NUM_BUFF_SIM_ALGO                 ((uint16_t)2) 

#ifdef GD32F4xx

/** @defgroup UART_TimeOut_Value    UART polling-based communications time-out value
  * @{
  */
#define HAL_UART_TIMEOUT_VALUE              0x1FFU  /*!< UART polling-based communications time-out value */

#endif

/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
uint8_t*          getBuffRxEm                       (void);
void              em_IRQHandler                     (void); 
void              UART_EM_DMA_Tx                    (uint8_t* pTxBuffer, uint16_t len); 
void              UART_EM_IRQHandler_DMA_IRQHandler (void); 
void              em_IRQHandler                     (void); 

#endif //  __EM_UART_H

/*************** END OF FILE ******************************************************************************************/



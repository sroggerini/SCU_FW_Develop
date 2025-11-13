/**
* @file        sbcUart.h
*
* @brief       Uart for SBC communication  - Definition -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: sbcUart.h 636 2024-11-22 14:49:00Z stefano $
*
*     $Revision: 636 $
*
*     $Author: stefano $
*
*     $Date: 2024-11-22 15:49:00 +0100 (ven, 22 nov 2024) $
*
*
* @copyright
*       Copyright (C) 2016 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

#ifndef __SBC_UART_H 
#define __SBC_UART_H

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

/********* -----------  USART5 -- SBC  -------- ***************************/
/* Definition for USART SBC  clock resources We use UART5 PD2=Rx and PC12=Tx  */
#ifdef GD32F4xx
#define UART_SBC                           UART5
#define UART_SBC_HANDLE                    huart5
#else
#define UART_SBC                           USART6
#define UART_SBC_HANDLE                    huart6
#endif
#define UART_SBC_DEFAULT_BR                ((uint32_t)19200)
//#define UART_SEM_SAME_BR                   1
//#define UART_SBC_SEM_BR                    ((uint32_t)230400) /* ((uint32_t)115200) è inutile avere velocità divers rispetto a quella del download */
#define UART_SBC_SEM_BR                    ((uint32_t)115200) /* mi tocca avere una velocità divers rispetto a quella del download */
#define UART_SBC_DWLD_BR                   ((uint32_t)230400) /* versione ufficiale: il download del FW avviene a questa velocità */

//#define UART_SBC_SEM_BR                    ((uint32_t)460800) /* funziona anche q questa velocità, ma per precauzione... */
#define UART_SBC_SEM_DWNL_BR               ((uint32_t)116300)
#define UART_SBC_CLK_ENABLE()              __HAL_RCC_UART5_CLK_ENABLE() 
#define UART_SBC_RX_GPIO_CLK_ENABLE()      __GPIOD_CLK_ENABLE()
#define UART_SBC_TX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()
#define DMAx_SBC_CLK_ENABLE()              __DMA1_CLK_ENABLE()

/* Definition for USARTx Pins */
#define UART_SBC_TX_PIN                    GPIO_PIN_12
#define UART_SBC_TX_GPIO_PORT              GPIOC
#define UART_SBC_TX_AF                     GPIO_AF8_UART5
#define UART_SBC_RX_PIN                    GPIO_PIN_2
#define UART_SBC_RX_GPIO_PORT              GPIOD
#define UART_SBC_RX_AF                     GPIO_AF8_UART5
 
/* Definition for USARTx's DMA */
#ifdef GD32F4xx
#define UART_SBC_TX_DMA_STREAM              DMA1_Stream7
#define UART_SBC_RX_DMA_STREAM              DMA1_Stream0
#else
#define UART_SBC_TX_DMA_STREAM              GPDMA2_Channel5
#define UART_SBC_RX_DMA_STREAM              GPDMA2_Channel2
#endif  

#define UART_SBC_TX_DMA_CHANNEL             DMA_CHANNEL_4
#define UART_SBC_RX_DMA_CHANNEL             DMA_CHANNEL_4



/* Definition for USARTx's NVIC */
#define UART_SBC_DMA                        DMA1
#define UART_SBC_DMA_TX_IRQn                DMA1_Stream7_IRQn
#define UART_SBC_DMA_RX_IRQn                DMA1_Stream0_IRQn
#define UART_SBC_DMA_TX_IRQHandler          DMA1_Stream7_IRQHandler
#define UART_SBC_DMA_RX_IRQHandler          DMA1_Stream0_IRQHandler

/* Definition for USARTx's NVIC */
#define UART_SBC_IRQn                       UART5_IRQn
#define UART_SBC_IRQHandler                 UART5_IRQHandler






/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
void              UART_SBC_DMA_Tx                     (uint8_t* pTxBuffer, uint16_t len); 
void              UART_SBC_IRQHandler_DMA_IRQHandler  (void); 
void              sbc_IRQHandler                      (void); 
uint8_t*          getBuffRxSbc                        (void);

#endif //  __SBC_UART_H

/*************** END OF FILE ******************************************************************************************/



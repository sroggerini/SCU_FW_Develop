/**
* @file        uartDbg.h
*
* @brief       Legacy definitions for UART in case of STM32F4xx device.
*              This to fit with STM32F74x register definitions.
*
* @author
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: uart_Legacy.h 
*
*     $Revision: 
*
*     $Author:
*
*     $Date:
*
*
* @copyright
*       Copyright (C) 2022 SCAME S.p.A. All rights reserved.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

#ifndef __UART_LEGACY_H 
#define __UART_LEGACY_H

/*
******************************************************************************
 *                              Include
******************************************************************************/
 

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/

/* Register and flag redefinition for Legacy purpose */

#ifdef GD32F4xx

/* Register redefinitionfor STM32F4xx */
#define UART_EM_ISR        (UART_EM->SR)
#define UART_EM_RDR        (UART_EM->DR)
#define UART_SBC_ISR       (UART_SBC->SR)
#define UART_SBC_RDR       (UART_SBC->DR)
#define UART_SBC_TDR       (UART_SBC->DR)
#define UART_DBG_ISR       (UART_DBG->SR)
#define UART_DBG_RDR       (UART_DBG->DR)
#define UART_SCU_ISR       (UART_SCU->SR)
#define UART_SCU_RDR       (UART_SCU->DR)
/* Flag redefinitionfor STM32F4 */
#define USART_FLAG_IDLE     UART_FLAG_IDLE
#define USART_FLAG_ORE      UART_FLAG_ORE    
#define USART_FLAG_FE       UART_FLAG_FE
#define USART_FLAG_PE       UART_FLAG_PE
#define USART_FLAG_NE       UART_FLAG_NE
#define USART_FLAG_RXNE     UART_FLAG_RXNE

extern HAL_StatusTypeDef UART_CheckIdleState(UART_HandleTypeDef *huart);

#else

/* Register redefinitionfor STM32F7xx */
#define UART_EM_ISR        (UART_EM->ISR)
#define UART_EM_RDR        (UART_EM->RDR)
#define UART_SBC_ISR       (UART_SBC->ISR)
#define UART_SBC_RDR       (UART_SBC->RDR)
#define UART_SBC_TDR       (UART_SBC->TDR)
#define UART_DBG_ISR       (UART_DBG->ISR)
#define UART_DBG_RDR       (UART_DBG->RDR)
#define UART_SCU_ISR       (UART_SCU->ISR)
#define UART_SCU_RDR       (UART_SCU->RDR)

#endif

#endif //  __UART_LEGACY_H


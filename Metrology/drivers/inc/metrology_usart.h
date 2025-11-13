/**
*   @file      handler_Modbus.h
*   @author    SCAME developers
*   @date      09 December 2021
*   @brief     Implements routines to handle Modbus protocol through SCU board
*   @note      
*
* @attention
*
*/

#ifndef METROLOGY_USART_H
#define METROLOGY_USART_H

/*******************************************************************************
* INCLUDE FILES:
*******************************************************************************/

#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#else
#include "stm32h5xx_hal.h"
#endif

/*******************************************************************************
* MACROS & CONSTANTS:
*******************************************************************************/

#define RS485_EM_ASSERTION_TIME     ((uint32_t)8)
#define RS485_EM_DEASSERTION_TIME   ((uint32_t)4)

/* USART for EM management definitions (Modbus)  */
#define UART_EM                      USART2
#define UART_EM_HANDLE               huart2

/* DMA definitions for EM management (Modbus)    */
#define UART_EM_DMA                  DMA1

/* Max lenght of the message received via Modbus  */
#define EM_DATA_MAX_LEN                 ((uint16_t)128)
#define EM_MSG_MAX_LEN                  ((uint16_t)(EM_DATA_MAX_LEN + (uint16_t)8))
#define DMA_RX_EM_BUFFER_SIZE           EM_MSG_MAX_LEN

/*******************************************************************************
* TYPES:
*******************************************************************************/

/*******************************************************************************
* FUNCTIONS EXPORTED:
*******************************************************************************/
void UART_EM_IRQHandler_UART(void);

/*******************************************************************************
* GLOBAL VARIABLES:
*******************************************************************************/

#endif /* METROLOGY_USART_H */

/*******************************************************************************
* FUNCTION EXPORTED:
*******************************************************************************/

/**
  * @}
  */

/* End Of File */

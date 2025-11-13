/**
* @file        prot_OnUsart.h
*
* @brief       protocol managet on USART - Definition -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: prot_OnUsart.h 599 2024-09-26 07:03:24Z stefano $
*
*     $Revision: 599 $
*
*     $Author: stefano $
*
*     $Date: 2024-09-26 09:03:24 +0200 (gio, 26 set 2024) $
*
*
* @copyright
*       Copyright (C) 2020 SCAME S.p.A. All rights reserved. This file is copyrighted and the property of Aesys
*       S.p.A.. It contains confidential and proprietary information. Any copies of this file (in whole or in part)
*       made by any method must also include a copy of this legend. Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

#ifndef __PROT_ONUSART_H 
#define __PROT_ONUSART_H

/************************************************************
 * Include
 ************************************************************/
#include <stdint.h>
#include <stdlib.h>
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#else
#include "stm32h5xx_hal.h"
#endif
#include "cmsis_os.h"

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/

#define   NUM_PROT_USART      ((uint8_t)4)

#define   PROT_UART_SCU       ((uint8_t)0)
#define   PROT_UART_EM        ((uint8_t)1)
#define   PROT_UART_SBC       ((uint8_t)2)
#define   PROT_UART_DBG       ((uint8_t)3)

#define   NUM_BUFF_USART_RX   ((uint8_t)2)
#define   TIMER_TICK_50       ((uint16_t)50)

#define   UART_SCU_BUFF_LEN   ((uint16_t)256)
#define   UART_SBC_BUFF_LEN   ((uint16_t)256)
#define   UART_DBG_BUFF_LEN   ((uint16_t)256)

/*
 * Strutture buffer in ricezione e trasmissione
 */
typedef struct
{
  unsigned char	  Flag;
  unsigned short	PBuffer;
  unsigned short  NumDLE;
  unsigned short	Checksum;
  unsigned int	  Buffer;
  unsigned short	Trans;
  unsigned short	payloadLen;
} RxStruct_t;

typedef struct
{
  unsigned short	PBuffer;
  unsigned short	Checksum;
  unsigned int	  Buffer;
  unsigned short	Length;
  unsigned short	Trans;
  unsigned char   NoAnswerChannel;
} TxStruct_t;
	

typedef __packed struct
{
  uint32_t      	    baudeRate;
  uint32_t       	    wordLen;
  uint32_t      	    parity;
  uint32_t      	    stopBit;
  USART_TypeDef*      uartIstance;
  UART_HandleTypeDef* pHuart;
  uint16_t	          PBuffer;
  uint16_t	          Checksum;
  uint32_t    	      Buffer;
  uint16_t      	    Length;
  uint16_t            intf;
	RxStruct_t*         pBuffRxSerial;
	TxStruct_t*         pBuffTxSerial;
	TxStruct_t*         pBuffTxTempSerial;
} usartStruct_st;


typedef void          (*vFuncVoid)(void);

/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart5;


/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
void              protOnUartTask                (void * pvParameters);
xQueueHandle      getSitosQueueHandle           (void);
usartStruct_st*   getIntfStructPtr              (uint8_t intf);
void              MX_PROT_UART_Init             (uint8_t intf);
void              MX_PROT_UART_DeInit           (uint8_t intf);
uint8_t           getProtTypeFromIstance        (UART_HandleTypeDef* uHandle);
uint8_t*          getDMAptr                     (uint8_t intf);
xQueueHandle      getUartxQueueHandle           (uint8_t intf);
uint32_t          getIntfBaudeRate              (uint8_t intf);
HAL_StatusTypeDef MX_PROT_UART_ChangeBaudeRate  (uint8_t intf, uint32_t br);
void              reInitSbcUart                 (void);
void              reInitScuUart                 (void);
void              fastInitScuUart               (void);
void              disableRS485                  (void);
void              startSemSbcUartTask           (void);
void              deleteEmTask                  (void);
void              activeSemTask                 (void);

#endif //  __PROT_ONUSART_H 

/*************** END OF FILE ******************************************************************************************/


/**
* @file        dbg_Task.h
*
* @brief       Uart debug  protocol - Definition -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: dbg_Task.h 694 2025-02-18 14:06:30Z stefano $
*
*     $Revision: 694 $
*
*     $Author: stefano $
*
*     $Date: 2025-02-18 15:06:30 +0100 (mar, 18 feb 2025) $
*
*
* @copyright
*       Copyright (C) 2016 SCAME S.p.A. All rights reserved. This file is copyrighted and the property of Aesys
*       S.p.A.. It contains confidential and proprietary information. Any copies of this file (in whole or in part)
*       made by any method must also include a copy of this legend. Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

#ifndef __DATI_DBG_H 
#define __DATI_DBG_H

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
#include "uartDbg.h"
#include "DataLink_dbg.h"
#include "httpserver-socket.h"

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#define   MIN_SETUP_ANSWER_TIME     ((uint32_t)5)
#define   MAX_SETUP_ANSWER_TIME     ((uint32_t)30)
#define   MIDDLE_SETUP_ANSWER_TIME  ((uint32_t)((MIN_SETUP_ANSWER_TIME + MAX_SETUP_ANSWER_TIME)/2))

#define   DEFAULT_STATUS_LEN         ((uint16_t)6)

/* Dimensione massimo campo dati del messaggio   */
#define DBG_DATA_MAX_LEN             ((uint16_t)256)
#define DBG_MSG_MAX_LEN              ((uint16_t)(DBG_DATA_MAX_LEN + (uint16_t)6))
//#define WIFI_MSG_MAX_LEN             ((uint16_t)262)
#define WIFI_MSG_LEN                 (PACKET_FW_LEN + 96) 
//#define WIFI_MSG_MAX_LEN             (2144) /*1024 + 6 + 13 di header + 32 secondo messaggio */
#define WIFI_MSG_MAX_LEN             (WIFI_MSG_LEN) /*1024 + 6 + 13 di header + 32 secondo messaggio */
//#define WIFI_MSG_MAX_LEN             ((uint16_t)2500)

#define WIFI_UDP_MSG_MAX_LEN         ((uint16_t)50)

#define WIFI_UPDATE_FW_GUARD_MSG_MAX_LEN   ((uint16_t)5)

//#define DMA_RX_DBG_BUFFER_SIZE       DBG_MSG_MAX_LEN
//#define DMA_RX_DBG_BUFFER_SIZE       256
#define DMA_RX_DBG_BUFFER_SIZE       1100

typedef enum
{
  TX_TO_HOST          = 0x00,       /* Request to send a msg to Siemens Outdoor station SST     */ 
  RX_FROM_HOST        = 0x01,       /* Request to take a msg from Siemens Outdoor station SST   */ 
  WIFI_EVENT_INIT,                  /* wifi start signal                                        */ 
  WIFI_EVENT_RX_DATA,               /* wifi receive data signal                                 */ 
  WIFI_EVENT_TX_DATA,               /* wifi transmit data signal                                */
  WIFI_EVENT_ADVERTISING,           /* wifi timeout due to advertising message                  */
  WIFI_EVENT_FORCE_RELEASE,         /* wifi forcing release semaphore UDP                       */
  DBG_HOST_ERROR      = 0xFFFF      /* Error on uart                                            */ 
} dbgMsgDir_e;


typedef enum
{
  DBG_START   = 0x68,     /* Start Long Telegram Type 1   */ 
  DBG_END1    = 0x0A,     /* Start Long Telegram Type 2   */ 
  DBG_END2    = 0x0D      /* Start Long Telegram Type 2   */  
} flagMsg_e;


/* in the following structure, keep alòigned the head position !!! Nick 27/03/2023 */
typedef __packed struct 
{
  uint16_t            payloadLen;
  dbgMsgDir_e         dbgMsgDir;
  uint8_t             infoRxDbg[DBG_MSG_MAX_LEN];
  //uint8_t             infoRxDbg[WIFI_MSG_MAX_LEN];
} frameDbg_st;


typedef __packed struct
{
  uint16_t            payloadLen;
  dbgMsgDir_e         dbgMsgDir;
  uint8_t             infoRxDbg[WIFI_MSG_MAX_LEN];
  //uint8_t             infoRxDbg[DBG_MSG_MAX_LEN];
} WififrameDbg_st;


typedef __packed struct
{
  uint16_t            payloadLen;
  dbgMsgDir_e         dbgMsgDir;
  uint8_t             infoRxDbg[WIFI_UDP_MSG_MAX_LEN];
} WifiUdpframeDbg_st;


typedef __packed struct
{
  uint16_t            payloadLen;
  uint8_t             infoRxDbg[WIFI_UPDATE_FW_GUARD_MSG_MAX_LEN];
} WifiUpdateFwGuardframe_st;


/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
void            dbgGestTask                   (void * pvParameters);
xQueueHandle    getDbgQueueHandle             (void);
void            HAL_UART_DBG_RxCpltCallback   (UART_HandleTypeDef *huart);
void            HAL_UART_DBG_TxCpltCallback   (UART_HandleTypeDef *huart);
void            UART_DBG_DMA_IRQHandler       (void) ;

extern uint8_t  newDbgMessageReceived         (frameDbg_st* pNewMessage);

#endif //  __DATI_DBG_H


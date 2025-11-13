/**
* @file        DataLink_dbg.c
*
* @brief       Data link Layer  for SIemens Traffic Outdoor Station protocol - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: DataLink_dbg.c 662 2025-01-24 13:42:36Z luca $
*
*     $Revision: 662 $
*
*     $Author: luca $
*
*     $Date: 2025-01-24 14:42:36 +0100 (ven, 24 gen 2025) $
*
*
* @copyright
*       Copyright (C) 2017 SCAME S.p.A. All rights reserved. This file is copyrighted and the property of Aesys
*       S.p.A.. It contains confidential and proprietary information. Any copies of this file (in whole or in part)
*       made by any method must also include a copy of this legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

/************************************************************
 * Include
 ************************************************************/
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#else
#include "stm32h5xx_hal.h"
#endif
#include "cmsis_os.h"
#include "dbg_Task.h"
#include "DataLink_dbg.h"

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local const                                   **
**                                                                          **
****************************************************************************** 
*/ 

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 
/*  DEBUG  data link queue  declaration */
xQueueHandle dlDbgRxQueue = NULL;
xQueueHandle dlDbgTxQueue = NULL;

/* Buffer used for reception and transmission */
frameDbg_st          dbgMsgDLrx_IT;
frameDbg_st          dbgMsgDLtx_IT;

/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/


/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Function                             **
**                                                                          **
******************************************************************************
*/

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
static void     processDbgDl                (frameDbg_st* pDbgDlMsg);

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/


/**
*
* @brief       Gestione del protocollo UART : rx side
*
* @param [in]  (void const *: not used  
*  
* @retval      none 
*  
****************************************************************/
void dl_DbgRxProcess (void * pvParameters)
{
  frameDbg_st         dbgDlMsg; 

  /*-------- Creates an empty mailbox for uart  messages --------------------------*/
  dlDbgRxQueue = xQueueCreate(NUM_BUFF_DBG_DL_RX, sizeof(frameDbg_st));
  configASSERT(dlDbgRxQueue != NULL);
  /* We want this queue to be viewable in a RTOS kernel aware debugger, so register it. */
  vQueueAddToRegistry( dlDbgRxQueue, "dbgDlQueueRx" ); 


  for (;;)
  {
    /* Wait for some event from Rx uart RS485 (typically UART2 */
    if (xQueueReceive(dlDbgRxQueue, (void *)&dbgDlMsg, portMAX_DELAY) == pdPASS)
    {
      dbgDlMsg.dbgMsgDir = RX_FROM_HOST;
      processDbgDl (&dbgDlMsg); 
    }
    else
    {
      continue;
    }
  }
}

/**
*
* @brief       Gestione del protocollo UART: tx side
*
* @param [in]  (void const *: not used  
*  
* @retval      none 
*  
****************************************************************/
void dl_DbgTxProcess (void * pvParameters)
{
  frameDbg_st    dbgDlMsg; 
  frameDbg_st*   pLocFrameDbg;

  /*-------- Creates an empty mailbox for uart  messages --------------------------*/
  dlDbgTxQueue = xQueueCreate(NUM_BUFF_DBG_DL_TX, sizeof(frameDbg_st));
  configASSERT(dlDbgTxQueue != NULL);

  for (;;)
  {
    /* Wait for some event from Tx uart RS485 (typically UART2 */
    if (xQueueReceive(dlDbgTxQueue, (void *)&dbgDlMsg, portMAX_DELAY) == pdPASS)
    {
      pLocFrameDbg = (frameDbg_st*)calloc(sizeof(frameDbg_st), 1);
      if ((pLocFrameDbg != NULL) && (dbgDlMsg.payloadLen != 0))
      {
        dbgDlMsg.dbgMsgDir = TX_TO_HOST;
        memcpy((void*)pLocFrameDbg, (void*)&dbgDlMsg, sizeof(frameDbg_st));
        processDbgDl (pLocFrameDbg);
        free((void *)pLocFrameDbg);
      }
    }
    else
    {
      continue;
    }
  }
}

/**
*
* @brief       Gestione del protocollo debug
*
* @param [in]  PACKET_MS*: puntatore al messaggio ricevuto  
*  
* @retval      none 
*  
****************************************************************/
static void processDbgDl(frameDbg_st* pDbgDlMsg)
{

  if (pDbgDlMsg->dbgMsgDir == RX_FROM_HOST)   // message coming from HOST
  {
    burstReceptionDbgMng((uint8_t*)pDbgDlMsg->infoRxDbg, pDbgDlMsg->payloadLen);
  }
  else
  {
    /* check for send message to HOST */
    if (pDbgDlMsg->dbgMsgDir == TX_TO_HOST)   // message to be send to HOST
    {
      while (UART_DBG_IDLE_Tx() != HAL_OK)
      {
        osDelay(10);
      }
      /* UART7 Tx is free: start transmission on UART debug  using DMA */
      UART_DBG_DMA_Tx((uint8_t*)pDbgDlMsg->infoRxDbg, (uint16_t)pDbgDlMsg->payloadLen);           
    }
    else
    {
      /* check for error */
      if (pDbgDlMsg->dbgMsgDir == DBG_HOST_ERROR)   // uart problem to comunicate with  HOST 
      {
        /*----- Initialization UART FOR UART HOST      -------------------------------------------*/
        MX_DBG_UART_Init();

        /*----- attivo sempre la ricezione: input from uart2   -------------*/
        HAL_UART_DBG_Rx_Ena_IT();
        rxDbgBurstActivation();
      }
    }
  }
  return;
}

/**
*
* @brief        Get the pointer to Debug Rx queue
*
* @param [in]   none
*
* @retval       xQueueHandle: pointer to defined Rx data link Rx queue
*
***********************************************************************************************************************/
xQueueHandle getDbgDlRxQueueHandle(void)
{
   return(dlDbgRxQueue);
}

/**
*
* @brief        Get the pointer to Debug Tx queue
*
* @param [in]   none
*
* @retval       xQueueHandle: pointer to defined  data link Tx queue
*
***********************************************************************************************************************/
xQueueHandle getDbgDlTxQueueHandle(void)
{
   return(dlDbgTxQueue);
}


/*************** END OF FILE ******************************************************************************************/


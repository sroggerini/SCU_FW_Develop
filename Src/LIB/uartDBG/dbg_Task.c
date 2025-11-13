/**
* @file        dbg_Task.c
*
* @brief       uart debug protocol - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: dbg_Task.c 652 2025-01-14 09:45:36Z stefano $
*
*     $Revision: 652 $
*
*     $Author: stefano $
*
*     $Date: 2025-01-14 10:45:36 +0100 (mar, 14 gen 2025) $
*
*
* @copyright
*       Copyright (C) 2017 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

/************************************************************
 * Include
 ************************************************************/
//#include <main.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_rcc.h"
#else
#include "stm32h5xx_hal.h"
#include "stm32h5xx_ll_rcc.h"
#endif   
#include "cmsis_os.h"
#include "dbg_Task.h"
#include "telnet.h"
#include "fatMng.h"
#include "rtcApi.h"
#include "wrapper.h"

/*
***********************************SCAME**************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/

#define     TIMER_TICK_500                ((uint16_t)500)

/*
***********************************SCAME**************************************
**                                                                          **
**                            Local Const                                   **
**                                                                          **
****************************************************************************** 
*/ 
/*
***********************************SCAME**************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 

static WififrameDbg_st    dbgMsg;  

/*
***********************************SCAME**************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 


/* DEBUG   queue  declaration */
xQueueHandle dbgQueue = NULL;


/*
***********************************SCAME**************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/


/*
***********************************SCAME**************************************
**                                                                          **
**                            External Function                             **
**                                                                          **
******************************************************************************
*/
extern void startConsole (uint8_t* pBuff);
#ifdef WR_SIMUL
extern void       setWrSimulation     (unsigned int status);
#endif

/*
***********************************SCAME**************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
static void           Process_dbg_Request     (frameDbg_st* pDbgMsg);

/*
***********************************SCAME**************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/

void dbgGestTask (void * pvParameters)
{
  uint32_t       timeTick;
  frameDbg_st*   pLocFrameDbg;
  
  /*-------- Creates an empty mailbox for uart  messages --------------------------*/
  //dbgQueue = xQueueCreate(NUM_BUFF_DBG_RX, sizeof(frameDbg_st));
  dbgQueue = xQueueCreate(8, sizeof(WififrameDbg_st));
  configASSERT(dbgQueue != NULL);
  /* We want this queue to be viewable in a RTOS kernel aware debugger, so register it. */
  vQueueAddToRegistry( dbgQueue, "dbgQueueRx" );

  /*----- Initialization UART FOR DEBUG      -------------------------------------------*/
	/* end initialization */

  MX_DBG_UART_DeInit();

  MX_DBG_UART_Init();

  /*----- attivo sempre la ricezione: input from uart6   -------------*/
  HAL_UART_DBG_Rx_Ena_IT();
    
  rxDbgBurstActivation();

  timeTick = pdMS_TO_TICKS(TIMER_TICK_500);

  for (;;)
  {
    /* Wait for some event from Rx/Tx uart RS485 (typically UART2 */
    if (xQueueReceive(dbgQueue, (void *)&dbgMsg, timeTick) == pdPASS)
    {
      pLocFrameDbg = (frameDbg_st*)calloc(sizeof(frameDbg_st), 1);
      if (pLocFrameDbg != NULL)
      {
        memcpy((void*)pLocFrameDbg, (void*)&dbgMsg, sizeof(frameDbg_st));
        Process_dbg_Request(pLocFrameDbg);
        free((void *)pLocFrameDbg);
      }
    }
    else
    {
      setLegalPeriod((uint32_t)0); /* al reset controllo la condizione dell'ora legale  */
      /* timeout */
      timeTick =  portMAX_DELAY;
      setConsoleIdle();
#ifdef SIMULATORE_CARICHI
      setReleCarico((uint8_t*)"00");
#endif
      startConsole((uint8_t*)"m\r\n");
       /* start EvsMngTask */
      send_to_evs(EVS_EVENT_START);            
    }
  }
}
 
/**
*
* @brief       Gestione del protocollo 
*
* @param [in]  PACKET_MS*: puntatore al messaggio ricevuto  
*  
* @retval      none 
*  
****************************************************************/
static void Process_dbg_Request(frameDbg_st* pDbgMsg)
{
  uint16_t len;

  do
  {
    len = (uint16_t)uxQueueMessagesWaiting(getDbgDlTxQueueHandle());
    if (len != 0)
    {
      osDelay(20);
    }
  } while (len != 0);

  configASSERT(xQueueSendToBack(getDbgDlTxQueueHandle(), (void *)pDbgMsg, portMAX_DELAY) == pdPASS);  // processDbgDl()
}

/**
*
* @brief        Get the pointer to master/slave Rx queue
*
* @param [in]   none
*
* @retval       xQueueHandle: pointer to defined Rx MS queue
*
***********************************************************************************************************************/
xQueueHandle getDbgQueueHandle(void)
{
   return(dbgQueue);
}

/*************** END OF FILE ******************************************************************************************/


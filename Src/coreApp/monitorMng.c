/******************* (C) COPYRIGHT 2009 STMicroelectronics *********/
/**
* @file        monitorMng.c
*
* @brief       Monitor task as supervisor for activities  - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: monitorMng.c 599 2024-09-26 07:03:24Z stefano $
*
*     $Revision: 599 $
*
*     $Author: stefano $
*
*     $Date: 2024-09-26 09:03:24 +0200 (gio, 26 set 2024) $
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
#include <main.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <ctype.h>
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_rcc.h"
#else
#include "stm32h5xx_hal.h"
#include "stm32h5xx_ll_rcc.h"
#endif   
#include "cmsis_os.h"
#include "main.h"
#include "monitorMng.h"
#include "wrapper.h"


/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#define     TIMER_SETUP_500                ((uint16_t)500)

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 
static   monMng_st        monMng[MON_EV_NUM];
static   monMngMsg_st     monMngMsg;
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
/*  Input manager queue  declaration */
xQueueHandle mngMonQueue = NULL;

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
extern  uint8_t               allTaskAreOperative           (void);
extern  void                  activeImmediateReset          (void);
extern  void                  resetInOperative              (void);

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
static uint32_t monManager            (monMngMsg_st* pMsg);

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/


/**
*
* @brief       Monitor task for activities 
*
* @param [in]  (void const *: not used  
*  
* @retval      none 
*  
****************************************************************/
void monMngTask (void * pvParameters)
{
  uint32_t       timeTick;
  uint16_t       cnt;

  /*-------- Creates an empty mailbox for monitor messages --------------------------*/
  mngMonQueue = xQueueCreate(MON_MAX_MESSAGE_NUM, sizeof(monMngMsg_st));
  configASSERT(mngMonQueue != NULL);

  for (cnt = 0; cnt < MON_EV_NUM; cnt++)
  {
    monMng[cnt].stato = MON_MNG_STATE_IDLE;
    monMng[cnt].monTimeout = (uint16_t)0;
  }

  /* tick di campionamento tipico 500ms quando attivo */
  timeTick = portMAX_DELAY;
  

  /* task could be call printf function so it starts only when console task is initializated see: startConsole()*/

  for (;;)
  {
    /* Wait for some event from SW for monitoring  */
    if (xQueueReceive(mngMonQueue, (void *)&monMngMsg, timeTick) == pdPASS)
    {
      timeTick = monManager((monMngMsg_st *)&monMngMsg);
    }
    else
    {
      monMngMsg.taskEv = MON_EVENT_POLLING;
      timeTick = monManager((monMngMsg_st *)&monMngMsg);
    }
  }
}



/**
*
* @brief        monitor on activities 
*
* @param [in]   monMngMsg_st*: puntatore al messaggio da eseguire
*
* @retval       none
*
***********************************************************************************************************************/
static uint32_t monManager (monMngMsg_st* pMsg)
{
  uint16_t    cnt, activeMon;
  uint32_t    localTimeTick;

  localTimeTick = MON_MNG_TIME_TICK;  // now tick is 500ms

  /* start processo */

  if (pMsg->taskEv == MON_EVENT_POLLING)
  {
    for (cnt = 0, activeMon = 0; cnt < MON_EV_NUM; cnt++)
    {
      switch (monMng[cnt].stato)
      {
        case MON_MNG_STATE_IDLE:
          break;

        case MON_MNG_STATE_HTTP:
          activeMon++;
          monMng[cnt].monTimeout++;
          toggleHeartLed();
          if (monMng[cnt].monTimeout >= monMng[cnt].monTimeoutTh)
          {
            if (cnt == (uint16_t)MON_EV_HTTP)
            {
              setFlagForNvic();
              /* problem on HTTP: reset immediatly */
              /* Generate a software reset */
              NVIC_SystemReset();
            }
          }
          break;

        case MON_MNG_STATE_SEM:
          activeMon++;
          if (allTaskAreOperative() == TRUE)
          {
            monMng[cnt].monTimeout = 0;
          }
          else
          {
            monMng[cnt].monTimeout++;
          }
          if (monMng[cnt].monTimeout >= monMng[cnt].monTimeoutTh)
          {
#ifdef ACTIVE_RESET
            tPrintf( "Error on Sbc Task!\r\n");
            activeImmediateReset();
            monMng[MON_SEM_TASK].stato = MON_MNG_STATE_IDLE;
#else

            monMng[MON_SEM_TASK].stato = MON_MNG_STATE_SEM;
            monMng[MON_SEM_TASK].monTimeout = (uint16_t)0;
            monMng[MON_SEM_TASK].monTimeoutTh = TIMEOUT_SEM;
            resetInOperative();
#endif
          }
          break;

        default:
          break;
      }
    }
    if (activeMon == 0) localTimeTick = portMAX_DELAY;
  }
  else
  {
    switch (pMsg->taskEv)
    {
      case MON_EV_HTTP:
        switch (monMng[MON_EV_HTTP].stato)
        {
          case MON_MNG_STATE_HTTP:
          case MON_MNG_STATE_IDLE:
            if (pMsg->taskValue == MON_START)
            {
              toggleHeartLed();
              monMng[MON_EV_HTTP].stato = MON_MNG_STATE_HTTP;
              monMng[MON_EV_HTTP].monTimeout = (uint16_t)0;
              monMng[MON_EV_HTTP].monTimeoutTh = TIMEOUT_HTTP;
            }
            else
            {
              if (pMsg->taskValue == MON_STOP)
              {
                monMng[MON_EV_HTTP].monTimeout = (uint16_t)0;
                /* lascio armato il watchdog: se tutto OK il SW si dovrebbe resettare autonomamente prima del timeout */
                //monMng[MON_EV_HTTP].stato = MON_MNG_STATE_IDLE;
              }
            }
            break;

          default:
            break;
        }
        break;

      case MON_SEM_TASK:
        switch (monMng[MON_SEM_TASK].stato)
        {
          case MON_MNG_STATE_SEM:
          case MON_MNG_STATE_IDLE:
            if (pMsg->taskValue == MON_START)
            {
              monMng[MON_SEM_TASK].stato = MON_MNG_STATE_SEM;
              monMng[MON_SEM_TASK].monTimeout = (uint16_t)0;
              monMng[MON_SEM_TASK].monTimeoutTh = TIMEOUT_SEM;
            }
            else
            {
              if (pMsg->taskValue == MON_STOP)
              {
                monMng[MON_SEM_TASK].monTimeout = (uint16_t)0;
              }
            }
            break;

          default:
            break;
        }
        break;

      default:
        break;
    }
  }
  return (localTimeTick);
}

/**
*
* @brief        Get the pointer to monitor manager queue
*
* @param [in]   none
*
* @retval       xQueueHandle: pointer to defined fat mng queue
*
***********************************************************************************************************************/
xQueueHandle getMonMngQueueHandle(void)
{
   return(mngMonQueue);
}


/**
*
* @brief        Send a message to the task 
*
* @param [in]   monMngEv_e: event type
* @param [in]   monMngValue_e: value
*
* @retval       none
*
***********************************************************************************************************************/
void sendMonMngMsg(monMngEv_e inp_event, monMngValue_e inp_value)
{
  monMngMsg_st tempMonMngMsg;

  tempMonMngMsg.taskEv = (monMngEv_e)inp_event;
  tempMonMngMsg.taskValue = (monMngValue_e)inp_value;
  configASSERT(xQueueSendToBack(mngMonQueue, (void *)&tempMonMngMsg, portMAX_DELAY) == pdPASS);
}


/*************** END OF FILE ******************************************************************************************/


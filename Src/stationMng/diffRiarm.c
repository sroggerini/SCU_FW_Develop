/******************* (C) COPYRIGHT 2009 STMicroelectronics *********/
/**
* @file        diffRiarm.c
*
* @brief       Check and management for retriggerable differential switch  - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: diffRiarm.c 79 2023-01-19 09:15:04Z npiergi $
*
*     $Revision: 79 $
*
*     $Author: npiergi $
*
*     $Date: 2023-01-19 10:15:04 +0100 (gio, 19 gen 2023) $
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
#include "ioExp.h"
#include "eeprom.h"
#include "diffRiarm.h"


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
static   diffRiarm_st     diffRiarm;
static   diffRiarmMsg_st  diffRiarmMsg;
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
xQueueHandle diffRiarmQueue = NULL;

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
static uint32_t diffRiarmManager            (diffRiarmMsg_st* pMsg);
static void     sendPulse                   (ioOut_TypeDef_e eventType);

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
void diffRiarmTask (void * pvParameters)
{
  uint32_t        timeTick;

  /*-------- Creates an empty mailbox for monitor messages --------------------------*/
  diffRiarmQueue = xQueueCreate(DIFF_RIARM_MAX_MESSAGE_NUM, sizeof(diffRiarmMsg_st));
  configASSERT(diffRiarmQueue != NULL);

    diffRiarm.stato = DIFF_RIARM_STATE_IDLE;
    timeTick = portMAX_DELAY;

  for (;;)
  {
    /* Wait for some event from SW for monitoring  */
    if (xQueueReceive(diffRiarmQueue, (void *)&diffRiarmMsg, timeTick) == pdPASS)
    {
      timeTick = diffRiarmManager((diffRiarmMsg_st *)&diffRiarmMsg);
    }
    else
    {
      diffRiarmMsg.taskEv = DIFF_RIARM_EV_TIMEOUT;
      timeTick = diffRiarmManager((diffRiarmMsg_st *)&diffRiarmMsg);
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
static uint32_t diffRiarmManager (diffRiarmMsg_st* pMsg)
{
  uint32_t      localTimeTick;
  uint8_t         diffStatus;

  localTimeTick = portMAX_DELAY;

if (pMsg->taskEv == DIFF_RIARM_EVENT_STOP)
	diffRiarm.stato = DIFF_RIARM_STATE_IDLE;

  /* start processo */
  switch (diffRiarm.stato)
  {
    case DIFF_RIARM_STATE_IDLE:
      switch (pMsg->taskEv)
      {
      	case DIFF_RIARM_EVENT_POLLING:
            eeprom_param_get(LCD_TYPE_EADD, (uint8_t *)&diffStatus, 1);
            diffStatus &= DIRI_MASK;
            diffRiarm.diffMode = (diriMode_e)diffStatus;
            if (diffRiarm.diffMode == DIRI_ON)
            {
              diffRiarm.stato = DIFF_RIARM_STATE_NORM_OP;
              localTimeTick = DIFF_RIARM_CHECK_RCBO;
            }
      	  break;

        default:
          break;
      }
      break;

    case DIFF_RIARM_STATE_CHECK:
      switch (pMsg->taskEv)
      {
        case DIFF_RIARM_EV_TIMEOUT:
          if (REM_ACT_STATUS_INPUT == GPIO_PIN_RESET)
          {
            tPrintf("Diff. Riarmabile Doepke presente!\n\r");
            localTimeTick = DIFF_RIARM_TO_CHECK_ON;
            /* retriggerable diff is present --> put it in ON state */
            diffRiarm.stato = DIFF_RIARM_STATE_GO_ON;
          }
          else
          {
            /* retriggerable diff isn't present --> suspend the task */
            diffRiarm.stato = DIFF_RIARM_STATE_DUMMY;
          }
          break;

        default:
          break;
      }
      break;

    case DIFF_RIARM_STATE_GO_ON:
      switch (pMsg->taskEv)
      {
        case DIFF_RIARM_EV_TIMEOUT:
          sendPulse(REM_ACT_ON);
          localTimeTick = DIFF_RIARM_TO_START_CHECK;
          diffRiarm.stato = DIFF_RIARM_STATE_NORM_OP;
          break;

        default:
          break;
      }
      break;

    case DIFF_RIARM_STATE_NORM_OP:
      switch (pMsg->taskEv)
      {
        case DIFF_RIARM_EV_TIMEOUT:
          if (REM_ACT_STATUS_INPUT == GPIO_PIN_SET)
          {
            localTimeTick = DIFF_RIARM_CHECK_RCBO;
          }
          else
          {
            /* TO DO: to send RCBO alarm to Vania */
            send_to_evs(EVS_RCBO_CLOSE);
            /* wait message to try retrigger the differential  */
            diffRiarm.stato = DIFF_RIARM_STATE_WAIT_TRG;
            localTimeTick = portMAX_DELAY;
          }
          break;

        case DIFF_RIARM_EV_SET_OFF:
          sendPulse(REM_ACT_OFF);
          break;

        case DIFF_RIARM_EVENT_POLLING:
          localTimeTick = DIFF_RIARM_CHECK_RCBO;
          break;

        default:
          break;
      }
      break;

    case DIFF_RIARM_STATE_WAIT_TRG:
      switch (pMsg->taskEv)
      {
        case DIFF_RIARM_EV_SET_OFF:
          sendPulse(REM_ACT_OFF);
          break;

        case DIFF_RIARM_EV_SET_ON:
          sendPulse(REM_ACT_ON);
          localTimeTick = DIFF_RIARM_SETUP_ON_TIME;
          diffRiarm.stato = DIFF_RIARM_STATE_NORM_OP;
          break;

        default:
          break;
      }
      break;

    default:
      break;
  }

  return (localTimeTick);
}

/**
*
* @brief        Get the pointer to task queue
*
* @param [in]   none
*
* @retval       xQueueHandle: pointer to defined  queue
*
***********************************************************************************************************************/
xQueueHandle getDiffRiarmQueueHandle(void)
{
   return(diffRiarmQueue);
}

/**
*
* @brief        Creates an activation/Deactivation pulse   REM_ACT_ON / REM_ACT_OFF
*
* @param [in]   ioEvents_e: 
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
#ifndef GD32F4xx

static void sendPulse(ioOut_TypeDef_e eventType)
{
  ioMngMsg_st ioExpMsg;

  /* pulse to activation diff retriggerable */
  ioExpMsg.val = 1;
  ioExpMsg.taskEv = IO_EVENT_IO_WRITING;
  ioExpMsg.outRegId = eventType;
  while(xQueueSendToBack(getIoMngQueueHandle(), (void *)&ioExpMsg, ( TickType_t ) 100) != pdPASS)
  {
    ; // no message sent within 100msec; retry
  }
  osDelay(200);
  ioExpMsg.val = 0;
  while(xQueueSendToBack(getIoMngQueueHandle(), (void *)&ioExpMsg, ( TickType_t ) 100) != pdPASS)
  {
    ; // no message sent within 100msec; retry
  }
}

#else

static void sendPulse(ioOut_TypeDef_e eventType)
{
  /* Select the event to manage */
  switch (eventType)
  {
      case REM_ACT_ON:
        /* Generate a 200ms pulse on REM_ACTON pin: set REM_ACTON pin to LOW level -->  +24V on R_ACT_OFF */
        HAL_GPIO_WritePin(REM_ACTON_GPIO_Port, REM_ACTON_Pin, GPIO_PIN_RESET);
        /* 200ms impulse duration */
        osDelay(200);
        /* Set REM_ACTON pin to LOW level  --> 0V on R_ACT_OFF */
        HAL_GPIO_WritePin(REM_ACTON_GPIO_Port, REM_ACTON_Pin, GPIO_PIN_SET);
        break;
        
      case REM_ACT_OFF:
        /* Generate a 200ms pulse on REM_ACTOFF pin: set REM_ACTOFF pin to LOW level --> +24V on R_ACT_OFF */
        HAL_GPIO_WritePin(REM_ACTOFF_GPIO_Port, REM_ACTOFF_Pin, GPIO_PIN_RESET);
        /* 200ms impulse duration */
        osDelay(200);
        /* Set REM_ACTOFF pin to LOW level --> 0V on R_ACT_OFF */
        HAL_GPIO_WritePin(REM_ACTOFF_GPIO_Port, REM_ACTOFF_Pin, GPIO_PIN_SET);        
        break;
        
      default:
        break;
  }  
  
}

#endif

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
void sendDiffRiarmMsg(diffRiarmEv_e inp_event)
{
#ifndef HW_MP28947  
  diffRiarmMsg_st tempdiffRiarmMsg;

  tempdiffRiarmMsg.taskEv = inp_event;
  configASSERT(xQueueSendToBack(diffRiarmQueue, (void *)&tempdiffRiarmMsg, portMAX_DELAY) == pdPASS);
#endif  
}

/*************** END OF FILE ******************************************************************************************/


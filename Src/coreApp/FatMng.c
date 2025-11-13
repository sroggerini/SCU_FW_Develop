/******************* (C) COPYRIGHT 2009 STMicroelectronics *********/
/**
* @file        FatMng.c
*
* @brief       Small FAT manager - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: FatMng.c 599 2024-09-26 07:03:24Z stefano $
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
#include "ff.h"
#include "telnet.h"
#include "fatfs.h"
#include "flashFat.h"
#include "fatMng.h"
#include "InputsMng.h"
#include "ethInitTask.h"

#include "EvsMng.h"

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
static   fatMng_t         fatMng;
static   fatMngMsg_st     fatMngMsg;

static   osSemaphoreId    fatMngSemaphore;

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
/*  FAT manager queue  declaration */
xQueueHandle fatMngQueue = NULL;

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
static void fatManager (fatMngMsg_st* pMsg);

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/


/**
*
* @brief       Gestione della FAT su Flash
*
* @param [in]  (void const *: not used  
*  
* @retval      none 
*  
****************************************************************/
void fatMngTask (void * pvParameters)
{
  /*-------- Creates an empty mailbox for uart SITOS messages --------------------------*/
  fatMngQueue = xQueueCreate(FAT_MAX_MESSAGE_NUM, sizeof(fatMngMsg_st));
  configASSERT(fatMngQueue != NULL);

  /* create a binary semaphore used for manager access */
  fatMngSemaphore = osSemaphoreNew(1, 1, NULL);
  configASSERT(fatMngSemaphore != NULL);
  /* init task state */
  fatMng.stato = FAT_STATE_IDLE;

  /* task could be call printf function so it starts only when console task is initializated see: startConsole()*/

  for (;;)
  {
    /* Wait for some event from SW to be transmitted on telnet */
    if (xQueueReceive(fatMngQueue, (void *)&fatMngMsg, portMAX_DELAY) == pdPASS)
    {
      fatManager((fatMngMsg_st *)&fatMngMsg);
    }
    else
    {
      continue;
    }
  }
}



/**
*
* @brief        esegue gestione comandi sulla FAT
*
* @param [in]   fatMngMsg_st*: puntatore al messaggio da eseguire
*
* @retval       none
*
***********************************************************************************************************************/
static void fatManager (fatMngMsg_st* pMsg)
{
  char*           pPath;
  osStatus_t      val;
  //inpMngMsg_st    inpMngMsg;
  FRESULT         resMount, resOpen, resFormat;

  /* start processo */
  switch (fatMng.stato)
  {
    case FAT_STATE_IDLE:
      switch (pMsg->taskEv)
      {
        case FAT_EVENT_INIT:
          val = osSemaphoreAcquire(fatMngSemaphore, portMAX_DELAY);
          if(val == osOK)
          {
            /* check for dir structures and new files */
            pPath = getFatDevicePath (FAT_DEVICE_FLASH);
            if (f_chdrive(pPath) == FR_OK) 
            {
              // Register the file system object to the FatFs module ##############
              resMount = f_mount(&USERFatFS, (TCHAR const*)pPath, 1);
              resOpen  = f_opendir(&locDir, (TCHAR*)VER1_DIR_SUFF); 
              if ((resMount != FR_OK) || (resOpen != FR_OK))
              {
                /* new flash unprogrammed: need to create the dir structure and welcome file */
                resFormat = deviceFatFormat(FAT_DEVICE_FLASH);
                if (resFormat != FR_OK)
                {
                  setFlagForNvic();
                  /* hard problem: try an software restart  */
                  NVIC_SystemReset();
                }
              } 
              else
              {
                f_closedir(&locDir);
              }
            }
            fatMng.stato = FAT_STATE_OPERATIVE;;
            // Rilascio il semaforo
            osSemaphoreRelease(fatMngSemaphore);          
//            /* start Input Management  management */
//            inpMngMsg.taskEv = INP_EVENT_START_POLLING;
//            configASSERT(xQueueSendToBack(getInptMngQueueHandle(), (void *)&inpMngMsg, portMAX_DELAY) == pdPASS); //inputMngTask

            /* start EvsMngTask */
            send_to_evs(EVS_EVENT_START);

#ifdef SINAPSI_RSE
            if (getSinapsiEepromEn() == ENABLED)
            {
              startSinapsiIom2G(); 
            }
#endif

            if (fatMng.action == FAT_EVENT_FW_FILE_DELETE)
            {
              fatMng.action = FAT_EVENT_DUMMY;
              fatMngMsg_st   tmpFatMngMsg;
              tmpFatMngMsg.taskEv = FAT_EVENT_FW_FILE_DELETE;
              configASSERT(xQueueSendToBack(fatMngQueue, (void *)&tmpFatMngMsg, portMAX_DELAY) == pdPASS); //fatMngTask
            }

          }
          break;

        default:
          break;
      }
      break;

    case FAT_STATE_OPERATIVE:
      switch (pMsg->taskEv)
      {
        case FAT_EVENT_WRITE:
          break;

        case FAT_EVENT_FW_FILE_DELETE:
          break;

        default:
          break;
      }
      break;

    default:
      break;
  }
}


/**
*
* @brief        Get the pointer to FAT memager queue
*
* @param [in]   none
*
* @retval       xQueueHandle: pointer to defined fat mng queue
*
***********************************************************************************************************************/
xQueueHandle getFatMngQueueHandle(void)
{
   return(fatMngQueue);
}

/**
*
* @brief        Set an action for the task 
*
* @param [in]   uint16_t: action code (or event)
*
* @retval       none 
*
***********************************************************************************************************************/
void  setActionForFatMng(uint16_t action )
{
   fatMng.action = action;
}



/*************** END OF FILE ******************************************************************************************/


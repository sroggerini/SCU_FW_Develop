/******************* (C) COPYRIGHT 2009 STMicroelectronics *********/
/**
* @file        ethInitTask.c
*
* @brief       starts eth enviroment - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: ethInitTask.c 764 2025-06-09 08:51:23Z stefano $
*
*     $Revision: 764 $
*
*     $Author: stefano $
*
*     $Date: 2025-06-09 10:51:23 +0200 (lun, 09 giu 2025) $
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
#include "lwip.h"
#include "main.h"
#include "telnet.h"
#include "ftps.h"
#include "ethInitTask.h"


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
static   ethInitMsg_t     ethInitMsg;
static   ethState_st      ethState;

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
**                            Global  const                                 **
**                                                                          **
****************************************************************************** 
*/ 
/* Definitions for telnet Task  */
osThreadId_t telnetTaskHandle;
const osThreadAttr_t telnetTask_attributes = {
  .name = "TELNET_TASK",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 10        
};
#ifdef FTP_SERVER
/* Definitions for FTP server Task  */
osThreadId_t ftpServerTaskHandle;
const osThreadAttr_t ftpServerTask_attributes = {
  .name = "FTP_SERVER_TASK",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 4
};
#endif
/*
*********************************** SCAME ************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 
/*  ethernet initialization queue  declaration */
xQueueHandle ethInitQueue = NULL;

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

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/

#ifdef HW_MP28947
/**
*
* @brief       Activate oscillator for ETH functionality
*
* @param [in]  none
*  
* @retval      none 
*  
****************************************************************/

void ETH_Activate_Osc (void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  GPIO_InitStruct.Pin = ETH_REF_CK_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/**
*
* @brief       Activate oscillator for ETH functionality
*
* @param [in]  none
*  
* @retval      none 
*  
****************************************************************/

void ETH_Disactivate_Osc (void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  HAL_GPIO_WritePin(GPIOE, ETH_REF_CK_EN_Pin, GPIO_PIN_RESET);
  
  GPIO_InitStruct.Pin = ETH_REF_CK_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}
#endif

/**
*
* @brief       Gestione input digitali
*
* @param [in]  (void const *: not used  
*  
* @retval      none 
*  
****************************************************************/
void ethIniProcess (void * pvParameters)
{
  uint32_t       timeTick;

  /*-------- Creates an empty mailbox for ethernet init messages --------------------------*/
  ethInitQueue = xQueueCreate(ETH_INIT_MAX_MESSAGE_NUM, sizeof(ethInitMsg_t));
  configASSERT(ethInitQueue != NULL);

  ethState.ethStatus = ETH_STATE_INIT;

  timeTick = portMAX_DELAY;
  for (;;)
  {
    /* Wait for init event from SW to start ethernet */
    if (xQueueReceive(ethInitQueue, (void *)&ethInitMsg, timeTick) == pdPASS)
    {
      if (ethInitMsg.taskEv == ETH_EVENT_INIT)
      {    

#if defined HW_MP28947 
#if !defined EXCLUDE_ETHERNET         
        ETH_Activate_Osc();
        /* init code for LWIP */
        MX_LWIP_Init();
        
        /* start polling to check ethernet status */
        timeTick = (uint32_t)ETH_POLLING_TIME_TICK;

#endif 
        /* Initilaize the telnet module */
        /* definition and creation of telnet Task */
        telnetTaskHandle = osThreadNew(telnetProcess, NULL, &telnetTask_attributes); 
#else
        /* init code for LWIP */
        MX_LWIP_Init();
        
        /* start polling to check ethernet status */
        timeTick = (uint32_t)ETH_POLLING_TIME_TICK;

        /* Initilaize the telnet module */
        /* definition and creation of telnet Task */
        telnetTaskHandle = osThreadNew(telnetProcess, NULL, &telnetTask_attributes);        
#endif
        
#ifdef FTP_SERVER
        /* definition and creation of FTP Server Task */
        ftpServerTaskHandle = osThreadNew(ftp_server, NULL, &ftpServerTask_attributes); 
#endif
        
      }
      else
      {
        if (ethInitMsg.taskEv == ETH_EVENT_SUSPEND)
        {
          timeTick = portMAX_DELAY;
        }
        else
        {
          if (ethInitMsg.taskEv == ETH_EVENT_RESUME)
          {
            /* restart polling to check ethernet status */
            timeTick = (uint32_t)ETH_POLLING_TIME_TICK;
          }         
          else
          {
            if (ethInitMsg.taskEv == ETH_EVENT_SUSPEND_TIME)
            {
              /* suspend the ethernet for a time to contrast brute force attack  */
              timeTick = (uint32_t)ETH_SUSP_RESUME_TIME_TICK;
              ethPowerDown();
              ethState.ethStatus = ETH_STATE_TEMP_SUSPEND;
            }
            else if (ethInitMsg.taskEv == ETH_EVENT_ETH_RESUME)
            {
#if defined HW_MP28947                         
              ETH_Activate_Osc(); 
              /* init code for LWIP */
              MX_LWIP_Init();
              /* start polling to check ethernet status */
              timeTick = (uint32_t)ETH_POLLING_TIME_TICK;
#endif              
            }
          }
        }
      }
    }
    else
    {
      switch (ethState.ethStatus)
      {
        case ETH_STATE_TEMP_SUSPEND:
          // Riattivo chip fisico della Ethernet 
          ethPowerUp();
          /* restart polling to check ethernet status */
          timeTick = (uint32_t)ETH_POLLING_TIME_TICK;
          ethState.ethStatus = ETH_STATE_INIT;
          break;
        default:
          // Controllo se è cambiato lo stato del link del fisico ETH
          updateEthLinkStatus();
          break;
      }
    }
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
xQueueHandle getEthInitQueueHandle(void)
{
   return(ethInitQueue);
}

/**
*
* @brief        send message to suspend polling check 
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void suspendEthPollingCheck(ethEvents_e ethEvents)
{
  ethInitMsg_t    ethInitMsg;

  /* start Ethernet Init */
  ethInitMsg.taskEv = ethEvents;
  configASSERT(xQueueSendToBack(ethInitQueue, (void *)&ethInitMsg, portMAX_DELAY) == pdPASS); //ethIniProcess
}

/**
*
* @brief        send message to resume polling check 
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void resumeEthPollingCheck(void)
{
  ethInitMsg_t    ethInitMsg;

  /* start Ethernet Init */
  ethInitMsg.taskEv = ETH_EVENT_RESUME;
  configASSERT(xQueueSendToBack(ethInitQueue, (void *)&ethInitMsg, portMAX_DELAY) == pdPASS); //ethIniProcess
}

/**
*
* @brief        send message to activate ethernet
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void Eth_send_Activate_Event(void)
{
  ethInitMsg_t    ethInitMsg;

  /* Ethernet resume */
  ethInitMsg.taskEv = ETH_EVENT_ETH_RESUME;
  configASSERT(xQueueSendToBack(ethInitQueue, (void *)&ethInitMsg, portMAX_DELAY) == pdPASS); //ethIniProcess
}

/**
*
* @brief        send message to start ethernet initialization 
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void ethSendEventInit(void)
{
  ethInitMsg_t    ethInitMsg;

  /* start Ethernet Init */
  ethInitMsg.taskEv = ETH_EVENT_INIT;
  configASSERT(xQueueSendToBack(ethInitQueue, (void *)&ethInitMsg, portMAX_DELAY) == pdPASS); //ethIniProcess
}



/*************** END OF FILE ******************************************************************************************/


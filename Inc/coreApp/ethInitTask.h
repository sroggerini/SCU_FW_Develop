/******************* (C) COPYRIGHT 2009 STMicroelectronics *********/
/**
* @file        ethInitTask.h
*
* @brief       starts eth enviroment - Definition -
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: ethInitTask.h 408 2024-02-19 09:29:00Z npiergi $
*
*     $Revision: 408 $
*
*     $Author: npiergi $
*
*     $Date: 2024-02-19 10:29:00 +0100 (lun, 19 feb 2024) $
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ETH_INIT_H
#define __ETH_INIT_H

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>

/*
*********************************** BM ***************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/

/* input  message size   */
#define   ETH_INIT_MAX_MESSAGE_NUM     ((uint8_t)1)

/* polling period to check etehrnet status default is 200msec    */
#define   ETH_POLLING_TIME_TICK        pdMS_TO_TICKS((uint16_t)200)
#define   ETH_SUSP_RESUME_TIME_TICK    pdMS_TO_TICKS((uint16_t)20000)


typedef enum
{
  ETH_EVENT_INIT              = 0x00,      /* init event                     */ 
  ETH_EVENT_SUSPEND           = 0x01,      /* suspend polling check  event   */ 
  ETH_EVENT_RESUME            = 0x02,      /* resume polling check  event    */ 
  ETH_EVENT_SUSPEND_TIME      = 0x03,      /* suspend just a bit  event      */ 
  ETH_EVENT_ETH_RESUME        = 0x04       /* ethernet resume event          */ 
} ethEvents_e;

typedef struct
{
  ethEvents_e     taskEv;
} ethInitMsg_t;

typedef enum
{
  ETH_STATE_INIT              = 0x00,      /* init event                     */ 
  ETH_STATE_TEMP_SUSPEND      = 0x01,      /* temporary suspension           */ 
} ethState_e;


typedef struct
{
  ethState_e     ethStatus;
} ethState_st;



/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
void            ethIniProcess           (void * pvParameters);
xQueueHandle    getEthInitQueueHandle   (void);
void            suspendEthPollingCheck  (ethEvents_e ethEvents);
void            ethSendEventInit        (void);
void            resumeEthPollingCheck   (void);
void            Eth_send_Activate_Event (void);
void            ETH_Disactivate_Osc     (void);
void            ETH_Activate_Osc        (void);

/**
  * @}
  */



#ifdef __cplusplus
}
#endif

#endif /* __ETH_INIT_H */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/


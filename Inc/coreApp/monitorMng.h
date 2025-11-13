/******************* (C) COPYRIGHT 2009 STMicroelectronics *********/
/**
* @file        monitorMng.h
*
* @brief       Monitor task as supervisor for activities - Definition -
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: monitorMng.h 581 2024-07-26 11:36:33Z npiergi $
*
*     $Revision: 581 $
*
*     $Author: npiergi $
*
*     $Date: 2024-07-26 13:36:33 +0200 (ven, 26 lug 2024) $
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
#ifndef __MON_MNG_H
#define __MON_MNG_H

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>
#include "telnet.h"

/*
*********************************** BM ***************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#define   TASK_TICK                      ((uint16_t)500)
#define   MON_MNG_TIME_TICK               pdMS_TO_TICKS(TASK_TICK)

#define   TIMEOUT_HTTP                   ((uint16_t)20000/TASK_TICK)
#define   TIMEOUT_SEM                    ((uint16_t)4000/TASK_TICK)

/* input  message size   */
#define   MON_MAX_MESSAGE_NUM   ((uint8_t)2)


typedef enum
{
  MON_MNG_STATE_IDLE         = 0x0000,       /* initial state    */ 
  MON_MNG_STATE_HTTP         = 0x0001,       /* monitor HTTP     */ 
  MON_MNG_STATE_SEM          = 0x0002,       /* monitor SEM      */ 
  MON_MNG_STATE_DUMMY        = 0xFFFF
} monMngStates_e;

typedef enum
{
  MON_EV_HTTP               = 0x00,      /* monitor HTTP activity                     */ 
  MON_SEM_TASK,                          /* monitor SEM activities                    */
  MON_EV_NUM,                            /* num activity to be monitored              */ 
  MON_EVENT_POLLING,
  MON_EV_DUMMY = 0xFFFF
} monMngEv_e;

typedef enum
{
  MON_START               = 0x00,      /* start monitor  activity              */ 
  MON_STOP,                            /* stop  monitor  activity              */ 
  MON_SUSPEND,
  MON_VALUE_DUMMY = 0xFFFF
} monMngValue_e;

typedef struct
{
  monMngEv_e      taskEv;
  monMngValue_e   taskValue;
} monMngMsg_st;

typedef struct
{
  monMngStates_e    stato;
  uint16_t          monTimeout;
  uint16_t          monTimeoutTh;
} monMng_st;


/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
void            monMngTask              (void * pvParameters);
xQueueHandle    getMonMngQueueHandle    (void);
void            sendMonMngMsg           (uint16_t inp_event, uint16_t inp_value);

/**
  * @}
  */



#ifdef __cplusplus
}
#endif

#endif /* __MON_MNG_H */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/


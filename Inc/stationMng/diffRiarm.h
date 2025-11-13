/******************* (C) COPYRIGHT 2009 STMicroelectronics *********/
/**
* @file        diffRiarm.h
*
* @brief       Check and management for retriggerable differential switch  - Definition -
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: diffRiarm.h 79 2023-01-19 09:15:04Z npiergi $
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DIFFRIARM_MNG_H
#define __DIFFRIARM_MNG_H

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>
#include "telnet.h"
#include "EnergyMng.h"

/*
*********************************** BM ***************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#define   TASK_TICK                      ((uint16_t)500)
#define   DIFF_RIARM_TIME_TICK           pdMS_TO_TICKS(TASK_TICK)
#define   DIFF_RIARM_TO_START_CHECK      pdMS_TO_TICKS(3000)
#define   DIFF_RIARM_TO_CHECK_ON         pdMS_TO_TICKS(3000)
#define   DIFF_RIARM_CHECK_RCBO          pdMS_TO_TICKS(1000)
#define   DIFF_RIARM_SETUP_ON_TIME       pdMS_TO_TICKS(3000)
#define   DIFF_RIARM_CHECK_IDLE          pdMS_TO_TICKS(2000)

#define   TIMEOUT_DIFF_RIARM             ((uint16_t)5000/TASK_TICK)

/* input  message size   */
#define   DIFF_RIARM_MAX_MESSAGE_NUM   ((uint8_t)2)

#ifndef GD32F4xx            
#define REM_ACT_STATUS_INPUT  getIoExpInput(IN_REM_ACT_STATUS)
#else
#define REM_ACT_STATUS_INPUT  HAL_GPIO_ReadPin(RACT_STATUS_GPIO_Port, RACT_STATUS_Pin)
#endif            
   
   
   
typedef enum
{
  DIFF_RIARM_STATE_IDLE         = 0x0000,       /* initial state    */ 
  DIFF_RIARM_STATE_CHECK        = 0x0001,       /* check if diff DOEKPE is present      */ 
  DIFF_RIARM_STATE_GO_ON        = 0x0002,
  DIFF_RIARM_STATE_GO_OFF       = 0x0003,
  DIFF_RIARM_STATE_NORM_OP      = 0x0004,
  DIFF_RIARM_STATE_WAIT_TRG     = 0x0005,
  DIFF_RIARM_STATE_DUMMY        = 0xFFFF
} diffRiarmStates_e;

typedef enum
{
  DIFF_RIARM_EV_TIMEOUT          = 0x00,      /* strart diff DOEKPE hunt phase             */ 
  DIFF_RIARM_EV_SET_ON,                       /* event to set ON                           */ 
  DIFF_RIARM_EV_SET_OFF,                      /* event to set OFF                          */ 
  DIFF_RIARM_EVENT_POLLING,
  DIFF_RIARM_EVENT_STOP,
  DIFF_RIARM_EV_DUMMY = 0xFFFF
} diffRiarmEv_e;

typedef enum
{
  DIFF_RIARM_START               = 0x00,      /* start monitor  activity              */ 
  DIFF_RIARM_STOP,                            /* stop  monitor  activity              */ 
  DIFF_RIARM_SUSPEND,
  DIFF_RIARM_VALUE_DUMMY = 0xFFFF
} diffRiarmValue_e;

typedef struct
{
  diffRiarmEv_e      taskEv;
  diffRiarmValue_e   taskValue;
} diffRiarmMsg_st;

typedef struct
{
  diffRiarmStates_e stato;
  uint16_t          diffRiarmTimeout;
  uint16_t          diffRiarmTimeoutTh;
  diriMode_e        diffMode;
} diffRiarm_st;


/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
void            diffRiarmTask              (void * pvParameters);
xQueueHandle    getDiffRiarmQueueHandle    (void);
void            sendDiffRiarmMsg            (diffRiarmEv_e inp_event);

/**
  * @}
  */



#ifdef __cplusplus
}
#endif

#endif /* __DIFFRIARM_MNG_H */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/


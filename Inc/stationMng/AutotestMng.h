/**
* @file        dbg_Task.h
*
* @brief       Autotest management - Definition -
*
* @author      Stefano
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: AutotestMng.h 79 2023-01-19 09:15:04Z npiergi $
*
*     $Revision: 79 $
*
*     $Author: npiergi $
*
*     $Date: 2023-01-19 10:15:04 +0100 (gio, 19 gen 2023) $
*
*
* @copyright
*       Copyright (C) 2016 SCAME S.p.A. All rights reserved. This file is copyrighted and the property of SCAME
*       S.p.A.. It contains confidential and proprietary information. Any copies of this file (in whole or in part)
*       made by any method must also include a copy of this legend. Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

#ifndef __AUTOTEST_MNG_H 
#define __AUTOTEST_MNG_H

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

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/

// ---------------------------------------- global define -----------------------------------------------------//

#define NUM_BUFF_AUTOTEST 2
#define TIMER_TICK_5000   ((uint16_t)5000)
#define TIMER_TICK_10000  ((uint16_t)10000)
#define TEST_TIMEOUT      TIMER_TICK_10000

#define PWM_DC_10PC       ((uint16_t)900)
#define PWM_DC_20PC       ((uint16_t)800)
#define PWM_DC_30PC       ((uint16_t)700)

/* Notify the autotest phase result */
#define NOTIFY_AUTOTEST_RESULT(TEST, RESULT)             \
  {                                                      \
     if (Autotest)                                       \
     {                                                   \
         AutotestMngMsg_st   AutotestMngMsg;             \
         static uint8_t Test_Digital_IC_Done = FALSE;    \
         AutotestMngMsg.Result = RESULT;                 \
         AutotestMngMsg.AutotestMngEvent = TEST;         \
         switch (TEST)                                   \
         {                                               \
            case AUTOTEST_EVENT_DIGITAL_IC:              \
              if (!Test_Digital_IC_Done)                 \
              {                                          \
                Test_Digital_IC_Done = TRUE;             \
                configASSERT(xQueueSendToBack(getAutotestMngQueueHandle(), (void *)&AutotestMngMsg, portMAX_DELAY) == pdPASS); \
              }                                          \
              break;                                     \
            default:                                     \
              configASSERT(xQueueSendToBack(getAutotestMngQueueHandle(), (void *)&AutotestMngMsg, portMAX_DELAY) == pdPASS); \
              break;                                     \
         }                                               \
     }                                                   \
  }

// ---------------------------------------- global typedef -----------------------------------------------------//

typedef enum
{
  AUTOTEST_STATE_IC = 0,
  AUTOTEST_STATE_MIRROR,
  AUTOTEST_STATE_MOTOR_1,
  AUTOTEST_STATE_MOTOR_2,
  AUTOTEST_STATE_MOTOR_3,
} AutotestState_en;

typedef enum
{
  AUTOTEST_EVENT_NULL = 0,
  AUTOTEST_EVENT_DIGITAL_IC,
  AUTOTEST_EVENT_MIRROR,
  AUTOTEST_EVENT_MOTOR_1,
  AUTOTEST_EVENT_MOTOR_2,
  AUTOTEST_EVENT_MOTOR_3,
} AutotestMngEvent_en;

typedef enum
{
  TEST_FAILED = 0,
  TEST_PASSED,
} TestResult_en;

/* queue info structure */
typedef __packed struct
{
  TestResult_en            Result;
  AutotestMngEvent_en      AutotestMngEvent;
} AutotestMngMsg_st;

// ---------------------------------------- global functions -----------------------------------------------------//

void AutotestMngTask (void * pvParameters);

// ---------------------------------------- external variables -----------------------------------------------------//

extern uint8_t Autotest;

// ---------------------------------------- external functions -----------------------------------------------------//

extern void         startPwmOnCP              (uint16_t dc);
extern xQueueHandle getAutotestMngQueueHandle (void);
extern void         Check_for_Autotest        (void);
extern void         notifyAutotestResult      (AutotestMngEvent_en eventType, TestResult_en res);
extern uint8_t      getAutotestStatus         (void);


/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/

#endif //  __AUTOTEST_MNG_H


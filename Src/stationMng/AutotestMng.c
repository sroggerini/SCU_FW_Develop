/**
* @file        AutotestMng.c
*
* @brief       Autotest implementation -
*
* @author      Stefano
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: AutotestMng.c 79 2023-01-19 09:15:04Z npiergi $
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
*       This file is copyrighted and the property of SCAME S.p.A. It contains confidential and proprietary
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
#include "main.h"
#include "AutotestMng.h"
#include "wrapper.h"

/*
***********************************SCAME**************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/


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

static AutotestMngMsg_st    AutotestMngMsg;  
static AutotestState_en     AutotestState;

/*
***********************************SCAME**************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 

/* Queue  declaration */
xQueueHandle AutotestMngQueue;
uint8_t Autotest;


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


/*
***********************************SCAME**************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/

/*
***********************************SCAME**************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/


/**
  * @brief  Check for Autotest functionality
  *         
  * @param  None
  * @param  None
  * @retval None
  */
void Check_for_Autotest (void)
{
  uint8_t cnt = 0;
  
  if (getSW1flagAndReset() == (uint8_t)TRUE)
  {
    /* Coming from a long pression on SW1 --> no autotest is necessary */
    Autotest = FALSE;
    return;
  }
     
  /* Check the button status for 200ms */
  for (cnt = 0; cnt < 20; cnt++)
  {
    /* Wait 10ms */
    HAL_Delay (10);
    /* Check SW PROG button if pressed or not */
    if (HAL_GPIO_ReadPin(SW_PROG_GPIO_Port, SW_PROG_Pin) == GPIO_PIN_SET)
    {
      /* Detected as not pressed --> procedure entry is NOT VALID */
      Autotest = FALSE;
      return;
    }
  }
  
  /* Button detected pressed for at least 200ms --> the procedure entry is VALID */
  Autotest = TRUE;
  return;
}

/**
  * @brief  Process the autotest phase
  *         
  * @param  pointer to the message received
  * @param  None
  * @retval None
  */
void ProcessAutotestPhase(AutotestMngMsg_st* ptr)
{
  /* Get the autotest phase */
  switch (AutotestState)
  {
    case AUTOTEST_STATE_IC:         /* Check of digital ICs (EEPROM, Flash, U40, U23, U25) */
      
      /* Check if the test is passed */
      if ((ptr->AutotestMngEvent == AUTOTEST_EVENT_DIGITAL_IC) && (ptr->Result == TEST_PASSED))
      {
         /* PWM signal to 10% */
         startPwmOnCP(PWM_DC_10PC);
         /* Move to the next test session */
         //AutotestState = AUTOTEST_STATE_MIRROR;
         /* because there a problem when SPEA do this test we have decidead to skip it */
         AutotestState = AUTOTEST_STATE_MOTOR_1;
      } 
      
      break;
      
    case AUTOTEST_STATE_MIRROR:     /* Check of mirror functionality */
      
      /* Check if the test is passed */
      if ((ptr->AutotestMngEvent == AUTOTEST_EVENT_MIRROR) && (ptr->Result == TEST_PASSED))
      {
         /* PWM signal to 20% */
         startPwmOnCP(PWM_DC_20PC);
         /* Move to the next test session */
         AutotestState = AUTOTEST_STATE_MOTOR_1;
      } 
      
      break;
      
    case AUTOTEST_STATE_MOTOR_1:      /* Check of motor functionality (1st part) */
      
      /* Check if the test is passed */
      if ((ptr->AutotestMngEvent == AUTOTEST_EVENT_MOTOR_1) && (ptr->Result == TEST_PASSED))
      {
        setOutBL1_P();
         /* Move to the next test session */
         AutotestState = AUTOTEST_STATE_MOTOR_2;
      }
      
      break;
      
    case AUTOTEST_STATE_MOTOR_2:      /* Check of motor functionality (2nd part) */
      
      /* Check if the test is passed */
      if ((ptr->AutotestMngEvent == AUTOTEST_EVENT_MOTOR_2) && (ptr->Result == TEST_PASSED))
      {
        setOutBL1_M();
         /* Move to the next test session */
         AutotestState = AUTOTEST_STATE_MOTOR_3;
      }
      
      break;

    case AUTOTEST_STATE_MOTOR_3:      /* Check of motor functionality (3rd part) */
      
      /* Check if the test is passed */
      if ((ptr->AutotestMngEvent == AUTOTEST_EVENT_MOTOR_3) && (ptr->Result == TEST_PASSED))
      {
        brakePhase();
         /* PWM signal to 30% */
         startPwmOnCP(PWM_DC_30PC);
         /* Move to the next test session */
         AutotestState = AUTOTEST_STATE_IC;
      } 
      
      break;
      
    default:
      break;
  }
  
}

/**
*
* @brief       Manage the Autotest procedure 
*
* @param [in]  pvParameters
*  
* @retval      none 
*  
****************************************************************/

void AutotestMngTask (void * pvParameters)
{
  
  uint32_t              timeTick;

  /*-------- Creates an empty mailbox for uart  messages --------------------------*/
  AutotestMngQueue = xQueueCreate(NUM_BUFF_AUTOTEST, sizeof(AutotestMngMsg_st));
  configASSERT(AutotestMngQueue != NULL);
  /* We want this queue to be viewable in a RTOS kernel aware debugger, so register it. */
  vQueueAddToRegistry( AutotestMngQueue, "AutotestMngQueue" );

  timeTick = pdMS_TO_TICKS(TEST_TIMEOUT);
  AutotestState = AUTOTEST_STATE_IC;

  for (;;)
  {
    /* Wait for some event from different processes */
    if (xQueueReceive(AutotestMngQueue, (void *)&AutotestMngMsg, timeTick) == pdPASS)
       /* Process the message */
       ProcessAutotestPhase(&AutotestMngMsg);
    else
       /* Something wrong happens during the test */
       /* Stop PWM signal */
       stopPwmOnCP();   
  } 
  
}
 

/**
*
* @brief        Get the pointer to the queue
*
* @param [in]   none
*
* @retval       AutotestMngQueue: pointer to defined queue
*
***********************************************************************************************************************/
xQueueHandle getAutotestMngQueueHandle(void)
{
   return(AutotestMngQueue);
}

/**
*
* @brief        give the notify for autotest 
*
* @param [in]   AutotestMngEvent_en: type of event
* @param [in]   TestResult_en: result type
*
* @retval       none
*
***********************************************************************************************************************/
void notifyAutotestResult (AutotestMngEvent_en eventType, TestResult_en res)
{
  AutotestMngMsg_st   AutotestMngMsg;             
   if (Autotest)                                       
   {                                                   
     AutotestMngMsg.Result = res;                 
     AutotestMngMsg.AutotestMngEvent = eventType;         
     configASSERT(xQueueSendToBack(AutotestMngQueue, (void *)&AutotestMngMsg, portMAX_DELAY) == pdPASS); 
   }                                                   
}

/**
*
* @brief        get current autotest status         
*
* @param [in]   none
*
* @retval       uint8_t: != 0 for autotest running   
*
***********************************************************************************************************************/
uint8_t getAutotestStatus (void)
{
  return(Autotest); 
}


/*************** END OF FILE ******************************************************************************************/


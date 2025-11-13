/******************* (C) COPYRIGHT 2009 STMicroelectronics *********/
/**
* @file        ledMng.c
*
* @brief       Manager led RGB  - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: ledMng.c 599 2024-09-26 07:03:24Z stefano $
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
#include "ledMng.h"
#include "displayPin.h"
#include "eeprom.h"


/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#define     TIMER_SETUP_500                ((uint16_t)500)

/* set for strip with 6 led */
#define     PWMLED_C_06_DC                 ((uint32_t)30) /* LED_C_RED   */
#define     PWMLED_B_06_DC                 ((uint32_t)30) /* LED_B_GREEN */
#define     PWMLED_A_06_DC                 ((uint32_t)32) /* LED_A_BLU   */
/* set for strip with 9 led */
#define     PWMLED_C_09_DC                 ((uint32_t)45)
#define     PWMLED_B_09_DC                 ((uint32_t)46)
#define     PWMLED_A_09_DC                 ((uint32_t)47)
/* set for strip with 12 led */
#define     PWMLED_C_12_DC                 ((uint32_t)50)
#define     PWMLED_B_12_DC                 ((uint32_t)51)
#define     PWMLED_A_12_DC                 ((uint32_t)52)
/* set for strip with 18 led */
#define     PWMLED_C_18_DC                 ((uint32_t)55)
#define     PWMLED_B_18_DC                 ((uint32_t)56)
#define     PWMLED_A_18_DC                 ((uint32_t)57)

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 
static    ledMng_t          ledMng[NUM_LED];
static    ledMngMsg_st      ledMngMsg;
static    TimerHandle_t     xLedTimers[NUM_LED];


/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local const                                   **
**                                                                          **
****************************************************************************** 
*/ 
const uint16_t pwmPercentileDef[NUM_LED] = {(uint16_t)PWMLED_C_MIN_DC, (uint16_t)PWMLED_B_MIN_DC, (uint16_t)PWMLED_A_MIN_DC};

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 
/*  Input manager queue  declaration */
xQueueHandle ledMngQueue = NULL;

uint8_t      ledCurrentSet[NUM_LED];

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
static void     ledManager          (ledMngMsg_st* pMsg);
static void     ledTimCallBack      (TimerHandle_t pxTimer);
static void     setLedStripCurrent  (void);

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/


/**
*
* @brief       Gestione input digitali
*
* @param [in]  (void const *: not used  
*  
* @retval      none 
*  
****************************************************************/
void ledMngTask (void * pvParameters)
{
  uint8_t         ix;

  /*-------- Creates an empty mailbox for uart SITOS messages --------------------------*/
  ledMngQueue = xQueueCreate(LED_MAX_MESSAGE_NUM, sizeof(ledMngMsg_st));
  configASSERT(ledMngQueue != NULL);

  ledCurrentSet[LED_C_RED] = (uint8_t)0;

  for (ix = 0; ix < NUM_LED; ix++)
  {
    ledMng[ix].stato = LED_STATE_OFF;
    ledMng[ix].timerId = (uint16_t)LED_C_RED + (uint16_t)ix;
    ledMng[ix].pwmPercentile = pwmPercentileDef[ix];
  }

  /*-------- Creates all timer for the led task  --------------------------*/
  for (ix = (uint8_t)0; ix < (uint8_t)NUM_LED; ix++)
  {
    /* in this case we use the auto-reload features */
    xLedTimers[ix] = xTimerCreate("TimLed", DEFAULT_BLINK_TIME, pdTRUE, (void*)(ix), ledTimCallBack);
    configASSERT(xLedTimers[ix] != NULL);
  }

  for (;;)
  {
    /* Wait for some event from SW to change led status */
    if (xQueueReceive(ledMngQueue, (void *)&ledMngMsg, portMAX_DELAY) == pdPASS)
    {
      setLedStripCurrent();
      ledManager((ledMngMsg_st *)&ledMngMsg);
    }
    else
    {
      /* timeout: not used  */
    }
  }
}



/**
*
* @brief        esegue gestione comandi sui led RGB
*
* @param [in]   fatMngMsg_st*: puntatore al messaggio da eseguire
*
* @retval       none
*
***********************************************************************************************************************/
static void ledManager (ledMngMsg_st* pMsg)
{
  uint32_t  period;
  uint8_t   ix;



  switch (pMsg->ledEvents)
  {
    case LED_EVENT_OFF:
      if ((ledMng[pMsg->ledId].stato == LED_STATE_BLINK_ON) || (ledMng[pMsg->ledId].stato == LED_STATE_BLINK_OFF))
      {
        /* stop blink timer */
         xTimerStop(xLedTimers[pMsg->ledId], TIMER_GARD_TIME);
      }
      setPWM_Ledx ((ledIdx_e)pMsg->ledId, (uint8_t)0);
      ledMng[pMsg->ledId].pwmPercentile = (uint8_t)0;
      ledMng[pMsg->ledId].stato = LED_STATE_OFF;
      break;

    case LED_EVENT_OFF_ALL:
      for (ix = 0; ix < NUM_LED; ix++)
      {
        if ((ledMng[ix].stato == LED_STATE_BLINK_ON) || (ledMng[ix].stato == LED_STATE_BLINK_OFF) || 
            (ledMng[ix].stato == LED_STATE_SOFT_ON)  || (ledMng[ix].stato == LED_STATE_FORCE_OFF))
        {
          /* stop blink timer */
           xTimerStop(xLedTimers[ix], TIMER_GARD_TIME);
        }
        setPWM_Ledx ((ledIdx_e)ix, (uint8_t)0);
        ledMng[ix].pwmPercentile = (uint8_t)0;
        ledMng[ix].stato = LED_STATE_OFF;
      }
      break;

    case LED_EVENT_ON:
      if ((ledMng[pMsg->ledId].stato == LED_STATE_BLINK_ON) || (ledMng[pMsg->ledId].stato == LED_STATE_BLINK_OFF))
      {
        /* stop blink timer */
         xTimerStop(xLedTimers[pMsg->ledId], TIMER_GARD_TIME);
      }
      if (ledMng[pMsg->ledId].pwmPercentile != pMsg->percentile)
      {
        ledMng[pMsg->ledId].pwmPercentile = pMsg->percentile;
      }
      setPWM_Ledx ((ledIdx_e)pMsg->ledId, (uint8_t)ledMng[pMsg->ledId].pwmPercentile);
//      ledMng[pMsg->ledId].stato = LED_STATE_OFF;
      ledMng[pMsg->ledId].stato = LED_STATE_ON;
     break;
    
    case LED_EVENT_BLINK:
      /* save blink time */
      ledMng[pMsg->ledId].blinkTime = pMsg->data;
      /* start blink timer */
      while ((xTimerChangePeriod (xLedTimers[pMsg->ledId], ledMng[pMsg->ledId].blinkTime, TIMER_GARD_TIME) != pdPASS)); 
      if (ledMng[pMsg->ledId].pwmPercentile != pMsg->percentile)
      {
        ledMng[pMsg->ledId].pwmPercentile = pMsg->percentile;
      }
      setPWM_Ledx ((ledIdx_e)pMsg->ledId, (uint8_t)ledMng[pMsg->ledId].pwmPercentile);
      ledMng[pMsg->ledId].stato = LED_STATE_BLINK_ON;
      break;

    case LED_EVENT_SOFT_LIGHT:
      /* save soft light parameter */
      ledMng[pMsg->ledId].iMaxCharge = pMsg->iMaxCharge;
      ledMng[pMsg->ledId].iRealCharge = pMsg->iRealCharge;

      if (ledMng[pMsg->ledId].iMaxCharge <= I_MIN_CHARGE)
      {
        period = T_MAX_SOFT_LIGHT;
      }
      else
      {
        period = T_MAX_SOFT_LIGHT - ((DELTA_T_SOFT_LIGHT * (ledMng[pMsg->ledId].iRealCharge - I_MIN_CHARGE)) / (ledMng[pMsg->ledId].iMaxCharge - I_MIN_CHARGE));
      }
      ledMng[pMsg->ledId].tickToIset =  (uint16_t)(period / TICK_SOFT_LIGHT);
      /* now set the step of pwm increment for every tick (100msec) */
      ledMng[pMsg->ledId].stepPwm = (pMsg->percentile / ledMng[pMsg->ledId].tickToIset);
      if (ledMng[pMsg->ledId].stepPwm == 0)
      {
        /* set minimum step */
        ledMng[pMsg->ledId].stepPwm = 1;
      }
      ledMng[pMsg->ledId].tickCurrent = ledMng[pMsg->ledId].tickMinimum = MIN_STEP_PWM / ledMng[pMsg->ledId].stepPwm;
      ledMng[pMsg->ledId].pendenza = 0;

      ledMng[pMsg->ledId].blinkTime = TICK_SOFT_LIGHT;
      while ((xTimerChangePeriod (xLedTimers[pMsg->ledId], ledMng[pMsg->ledId].blinkTime, TIMER_GARD_TIME) != pdPASS)); 
      if (ledMng[pMsg->ledId].pwmPercentile != pMsg->percentile)
      {
        ledMng[pMsg->ledId].tickCurrent++;
        if (ledMng[pMsg->ledId].tickCurrent > ledMng[pMsg->ledId].tickToIset)
        {
          ledMng[pMsg->ledId].tickCurrent = 0;
        }
        ledMng[pMsg->ledId].pwmPercentile = pMsg->percentile;
      }
      ledMng[pMsg->ledId].stato = LED_STATE_SOFT_ON;
      break;

    case LED_EVENT_BLINK_ALL:
      for (ix = 0; ix < NUM_LED; ix++)
      {
        /* save blink time */
        ledMng[ix].blinkTime = pMsg->data;
        /* start blink timer */
        while ((xTimerChangePeriod (xLedTimers[ix], ledMng[ix].blinkTime, TIMER_GARD_TIME) != pdPASS)); 
        if (ledMng[ix].pwmPercentile != pMsg->percentile)
        {
          ledMng[ix].pwmPercentile = pMsg->percentile;
        }
        setPWM_Ledx ((ledIdx_e)ix, (uint8_t)ledMng[ix].pwmPercentile);
        ledMng[ix].stato = LED_STATE_BLINK_ON;
      }
      break;
  }
}


/**
*
* @brief        Get the pointer to Led manager queue
*
* @param [in]   none
*
* @retval       xQueueHandle: pointer to defined led mng queue
*
***********************************************************************************************************************/
xQueueHandle getLedMngQueueHandle(void)
{
   return(ledMngQueue);
}

/**
*
* @brief        callback to manager timers   
*
* @param [in]   TimerHandle_t: the elapsed timer 
*
* @retval       none
*
***********************************************************************************************************************/
static void ledTimCallBack (TimerHandle_t pxTimer)
{
  uint32_t          timer_id;

  /* find the led  which the timer is referred */
  timer_id = (uint32_t)pvTimerGetTimerID(pxTimer);

  /* check if timer linked to led colour exist  */
  if ((timer_id == (uint32_t)TIM_TR) || (timer_id == (uint32_t)TIM_TG) || (timer_id == (uint32_t)TIM_TB))
  {
    /* reload timer for next blink */
    while ((xTimerChangePeriod (xLedTimers[timer_id], ledMng[timer_id].blinkTime, TIMER_GARD_TIME) != pdPASS)); 
    if (ledMng[timer_id].stato == LED_STATE_BLINK_ON)
    {
      setPWM_Ledx ((ledIdx_e)timer_id, (uint8_t)0);
      ledMng[timer_id].stato = LED_STATE_BLINK_OFF;
    }
    else
    {
      if (ledMng[timer_id].stato == LED_STATE_SOFT_ON)
      {
        if (ledMng[timer_id].pendenza == 0)
        {
          ledMng[timer_id].tickCurrent++;
          if (ledMng[timer_id].tickCurrent > ledMng[timer_id].tickToIset)
          {
            ledMng[timer_id].pendenza = 1;
          }
        }
        else
        {
          ledMng[timer_id].tickCurrent--;
          if (ledMng[timer_id].tickCurrent < ledMng[timer_id].tickMinimum)
          {
            ledMng[timer_id].pendenza = 0;
          }
        }
        ledMng[timer_id].pwmPercentile = ledMng[timer_id].tickCurrent * ledMng[timer_id].stepPwm;
        setPWM_Ledx ((ledIdx_e)timer_id, (uint8_t)ledMng[timer_id].pwmPercentile);
      }
      else
      {
        setPWM_Ledx ((ledIdx_e)timer_id, (uint8_t)ledMng[timer_id].pwmPercentile);
        ledMng[timer_id].stato = LED_STATE_BLINK_ON;
      }
    }
  }
}

/**
*
* @brief        API for led    
*
* @param [in]   ledIdx_e: led colour index 
* @param [in]   ledEvents_e:  LED_EVENT_OFF / LED_EVENT_ON / LED_EVENT_BLINK / LED_EVENT_OFF_ALL / LED_EVENT_BLINK_ALL  
* @param [in]   uint16_t:  Blinking time in msec (used only with LED_EVENT_BLINK and LED_EVENT_BLINK_ALL)  
* @param [in]   uint8_t:  0 for default percentile, other value is PWM percentile i.e led current value  
*
* @retval       none
*
***********************************************************************************************************************/
void setLed (ledIdx_e led, ledEvents_e ledEvent, uint16_t blinkTime, uint8_t percentValue)
{
  ledMngMsg_st  ledTmpMsg;
    
  ledTmpMsg.ledId = led;
  ledTmpMsg.ledEvents = ledEvent;
  ledTmpMsg.data = blinkTime;
  if (percentValue == (uint8_t)0)
  {
    ledTmpMsg.percentile = pwmPercentileDef[led];
  }
  else
  {
    ledTmpMsg.percentile = (uint16_t)percentValue;
  }
  configASSERT(xQueueSendToBack(getLedMngQueueHandle(), (void *)&ledTmpMsg, portMAX_DELAY) == pdPASS);
}
/**
*
* @brief        API for led to have "soft start light" during charge phase   
*
* @param [in]   ledIdx_e: led colour index 
* @param [in]   ledEvents_e:  LED_EVENT_OFF / LED_EVENT_ON / LED_EVENT_BLINK / LED_EVENT_OFF_ALL / LED_EVENT_BLINK_ALL  
* @param [in]   uint16_t:  Blinking time in msec (used only with LED_EVENT_BLINK and LED_EVENT_BLINK_ALL)  
* @param [in]   uint8_t:  0 for default percentile, other value is PWM percentile i.e led current value  
* @param [in]   uint16_t: 0 current I charge in Amp * 10  
* @param [in]   uint16_t: 0 max I in current configuration    
*
* @retval       none
*
***********************************************************************************************************************/
void setLedSoft (ledIdx_e led, ledEvents_e ledEvent, uint16_t blinkTime, uint8_t percentValue, uint16_t iCurr, uint16_t iMax)
{
  ledMngMsg_st  ledTmpMsg;

  ledTmpMsg.ledId = led;
  ledTmpMsg.ledEvents = ledEvent;
  ledTmpMsg.data = blinkTime;
  ledTmpMsg.iMaxCharge = iMax;
  ledTmpMsg.iRealCharge = iCurr;
  if (percentValue == (uint8_t)0)
  {
    ledTmpMsg.percentile = pwmPercentileDef[led];
  }
  else
  {
    ledTmpMsg.percentile = (uint16_t)percentValue;
  }
  configASSERT(xQueueSendToBack(getLedMngQueueHandle(), (void *)&ledTmpMsg, portMAX_DELAY) == pdPASS);
}

/**
*
* @brief        set led current in according to strip led size    
*
* @param [in]   none 
*
* @retval       none
*
***********************************************************************************************************************/
static void setLedStripCurrent (void)
{
  uint8_t   ix;

  if (ledCurrentSet[LED_C_RED] == 0)
  {
    /* the current led must be set in according to led strip size */
    eeprom_param_get(STRIP_LED_TYPE_EADD, (uint8_t *)&ix, 1);
    switch (ix)
    {
      case LED_STRIP_18:
        ledCurrentSet[LED_C_RED] = PWMLED_C_18_DC;
        ledCurrentSet[LED_B_GREEN] = PWMLED_B_18_DC;
        ledCurrentSet[LED_A_BLU] = PWMLED_A_18_DC;
        break;
      case LED_STRIP_09:   
        ledCurrentSet[LED_C_RED] = PWMLED_C_09_DC;
        ledCurrentSet[LED_B_GREEN] = PWMLED_B_09_DC;
        ledCurrentSet[LED_A_BLU] = PWMLED_A_09_DC;
        break;
      case LED_STRIP_12:
        ledCurrentSet[LED_C_RED] = PWMLED_C_12_DC;
        ledCurrentSet[LED_B_GREEN] = PWMLED_B_12_DC;
        ledCurrentSet[LED_A_BLU] = PWMLED_A_12_DC;
        break;
      default: 
      case LED_STRIP_06: 
        ledCurrentSet[LED_C_RED] = PWMLED_C_06_DC;
        ledCurrentSet[LED_B_GREEN] = PWMLED_B_06_DC;
        ledCurrentSet[LED_A_BLU] = PWMLED_A_06_DC;
        break;
    }
  }
}

/**
*
* @brief        set new level for led     
*
* @param [in]   none 
*
* @retval       none
*
***********************************************************************************************************************/
void setNewCurrentLed (void)
{
  uint8_t   ix;

  /* reset status and set new reference led current level */
  ledCurrentSet[LED_C_RED] = 0;
  setLedStripCurrent();

  /* upgrading current level */
  for (ix = 0; ix < NUM_LED; ix++)
  {
    if (ledMng[ix].pwmPercentile != 0)
    {
      ledMng[ix].pwmPercentile = (uint16_t)ledCurrentSet[ix];
      setPWM_Ledx ((ledIdx_e)ix, (uint8_t)ledMng[ix].pwmPercentile);
    }
  }
}

/*************** END OF FILE ******************************************************************************************/


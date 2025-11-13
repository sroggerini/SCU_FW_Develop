/******************* (C) COPYRIGHT 2009 STMicroelectronics *********/
/**
* @file        ledMng.h
*
* @brief       Manager led RGB  - Definition -
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: ledMng.h 157 2022-11-02 15:02:56Z stefano $
*
*     $Revision: 157 $
*
*     $Author: stefano $
*
*     $Date: 2022-11-02 16:02:56 +0100 (mer, 02 nov 2022) $
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
#ifndef __LEDMNG_H
#define __LEDMNG_H

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>
#include "wrapper.h"
#include "telnet.h"
#include "i2c.h"

/*
*********************************** BM ***************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#define   TIMER_GARD_TIME                pdMS_TO_TICKS((uint16_t)500)
#define   DEFAULT_BLINK_TIME             pdMS_TO_TICKS((uint16_t)1000)

#define   I_MIN_CHARGE                   ((uint32_t)60)     /* in A*10 ossi a 6A */
#define   T_MAX_SOFT_LIGHT               ((uint32_t)2000)   /* 4sec di durata massima effetto luce crescente */
#define   T_MIN_SOFT_LIGHT               ((uint32_t)1000)   /* 1sec di durata minima effetto luce crescente */
#define   DELTA_T_SOFT_LIGHT             ((uint32_t)(T_MAX_SOFT_LIGHT - T_MIN_SOFT_LIGHT))  
#define   TICK_SOFT_LIGHT                ((uint32_t)50)     /* tick di 100msec per cambiare step di luminosità */
#define   MIN_STEP_PWM                   ((uint32_t)14)      /* tick per luminosità minima */

/* input  message size   */
//#define   LED_MAX_MESSAGE_NUM     ((uint8_t)2)
#define   LED_MAX_MESSAGE_NUM     ((uint8_t)9)


/* timer for led blinking   */
#define   TIM_TR                    ((uint16_t)LED_C_RED)        
#define   TIM_TG                    ((uint16_t)LED_B_GREEN)        
#define   TIM_TB                    ((uint16_t)LED_A_BLU)        

typedef enum
{
  LED_STATE_IDLE        = 0x0000,       /* initial state          */ 
  LED_STATE_OFF,                        /* Led state OFF          */ 
  LED_STATE_ON,                         /* Led state ON           */ 
  LED_STATE_BLINK_ON,                   /* Led state Blinking On  */ 
  LED_STATE_BLINK_OFF,                  /* Led state Blinking Off */ 
  LED_STATE_SOFT_ON,                    /* Led state soft start from min to Max  */ 
  LED_STATE_FORCE_OFF                   /* Force LED off due to V230 absence */
} ledStates_e;

typedef struct
{
  ledIdx_e        ledId;
  ledEvents_e     ledEvents;
  uint16_t        percentile;
  uint16_t        data;
  uint16_t        iMaxCharge;
  uint16_t        iRealCharge;
} ledMngMsg_st;


typedef struct
{
  ledStates_e   stato;
  uint16_t      timerId;
  uint32_t      blinkTime;
  uint16_t      pwmPercentile;
  uint16_t      iMaxCharge;
  uint16_t      iRealCharge;
  uint16_t      tickToIset;
  uint16_t      stepPwm;
  uint16_t      tickCurrent;
  uint16_t      tickMinimum;
  uint16_t      pendenza;
} ledMng_t;

typedef enum
{
  LED_STRIP_06        = 0x06,    /* strip with 6 leds           */     
  LED_STRIP_09        = 0x09,    /* strip with 9 leds           */     
  LED_STRIP_12        = 0x12,    /* strip with 12 leds          */     
  LED_STRIP_18        = 0x18,    /* strip with 18 leds          */     
  LED_STRIP_NUM       = 4        /* strip with 6 leds           */     
}ledStrip_e;

/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
void            ledMngTask              (void * pvParameters);
xQueueHandle    getLedMngQueueHandle    (void);
void            setNewCurrentLed        (void);

/**
  * @}
  */



#ifdef __cplusplus
}
#endif

#endif /* __LEDMNG_H */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/


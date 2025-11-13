/******************* (C) COPYRIGHT 2009 STMicroelectronics *********/
/**
* @file        InputsMng.c
*
* @brief       polling and filtering on digital inputs - Definition -
* @file        FatMng.c
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: inputsMng.h 587 2024-08-02 12:29:01Z stefano $
*
*     $Revision: 587 $
*
*     $Author: stefano $
*
*     $Date: 2024-08-02 14:29:01 +0200 (ven, 02 ago 2024) $
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
#ifndef __INPUTSMNG_H
#define __INPUTSMNG_H

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>
#include "telnet.h"
#include "wrapper.h"

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#define     SAMPLE_TICK                ((uint16_t)10)

/* input  message size   */
#define   INP_MAX_MESSAGE_NUM     ((uint8_t)2)

/* filtering parameters */
#define   DEFAULT_THRESHOLD       ((uint16_t)2)

typedef enum
{
  INP_STATE_IDLE        = 0x0000,       /* initial state   */ 
  INP_STATE_DISABLED    = 0x0001,       /* input disabled    */ 
  INP_STATE_RESET       = 0x0002,       /* input in reset state "0"  */ 
  INP_STATE_SET         = 0x0003,       /* input in set state "1"  */ 
  INP_STATE_GOING_LOW   = 0x0004,       /* input in transition from state "1" to state "0"  */ 
  INP_STATE_GOING_HIGH  = 0x0005,       /* input in transition from state "0" to state "1"  */ 
  INP_STATE_DUMMY       = 0xFFFF
} inpStates_e;

typedef enum
{
  INP_EVENT_START_POLLING = 0x00,          /* start the polling               */ 
  INP_EVENT_CHANGE_TIM_FILTER,             /* change filtering time           */ 
  INP_EVENT_POLLING_ON_TO,                 /* new samplig on timeout          */ 
  INP_EVENT_START_FILTERING,               /* start filtering                 */ 
  INP_EVENT_STOP_FILTERING,                /* stop filtering                  */ 
  INP_EVENT_DUMMY = 0xFFFF
} inpEvents_e;

typedef struct
{
  uint16_t      idInp;
  inpEvents_e   taskEv;
} inpMngMsg_st;

typedef struct
{
  inpStates_e   stato;
  uint16_t      currValInp;
  uint16_t      newValInp;
  uint16_t      currFilteringInp;
  uint16_t      filteringThreshold;
  uint16_t      posPulseCnt;
  uint16_t      negPulseCnt;
  uint16_t      alarmOn;
} inpMng_t;





/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
void            inputMngTask            (void * pvParameters);
xQueueHandle    getInptMngQueueHandle   (void);
GPIO_PinState   getInput                (dIn_TypeDef pinId);
void			      send_to_inp				      (uint8_t inp_event);
void            stopCompletePolling     (void);
void            startCompletePolling    (void);
uint32_t        readDigitalInput        (void);
void            getDigitalInput         (void);
char*           getTimePtr              (void);
inpStates_e     getInputStato			      (dIn_TypeDef pinId);

extern          inpMng_t     inpMng[NUM_INPUT_SCU];

/**
  * @}
  */



#ifdef __cplusplus
}
#endif

#endif /* __INPUTSMNG_H */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/


/******************* (C) COPYRIGHT 2009 STMicroelectronics *********/
/**
* @file        sinapsi.h
*
* @brief       Manager chain 2 protocol (SINAPSI - RSE)  - Definition -
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: sinapsi.h 327 2023-10-17 17:15:38Z npiergi $
*
*     $Revision: 327 $
*
*     $Author: npiergi $
*
*     $Date: 2023-10-17 19:15:38 +0200 (mar, 17 ott 2023) $
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
#ifndef __SINAPSI_H
#define __SINAPSI_H

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
#define   SINAPSI_TIMER_GARD_TIME        pdMS_TO_TICKS((uint16_t)500)

#define   I_MIN_CHARGE                   ((uint32_t)60)     /* in A*10 ossi a 6A */

#define   STEP_DEC_POWER                 ((uint16_t)500)           
#define   STEP_DEC_TIME                  ((uint16_t)10)

/* input  message size   */
#define   SINAPSI_MAX_MESSAGE_NUM     ((uint8_t)2)


/* timer for warning RSE message   */
#define   TIM_PMAX                  ((uint16_t)TIMER_PMAX_T)        
#define   TIM_SUSP                  ((uint16_t)TIMER_SUSP_T)        
#define   TIM_DIST                  ((uint16_t)TIMER_DIST_T)        

typedef enum
{
  SINAPSI_STATE_IDLE          = 0x0000,       /* initial state                                  */ 
  SINAPSI_STATE_WAIT_MODULES,                 /* attesa connessione con i moduli IOM2G e PM     */ 
  SINAPSI_STATE_FULL_CONN                     /* connesso sia con IOM2G che con Power Management  */ 
} sinapsiStates_e;

typedef enum
{
  IOM2G_ACTIVE_MASK        = 0x0001,        /* modulo IOM2G attivo                              */ 
  PM_ACTIVE_MASK           = 0x0002,        /* modulo power management attivo                   */ 
} activeModuleMask_e;

typedef struct
{
  sinapsiEvents_e     sinapsiEvents;
} sinapsiMngMsg_st;

typedef __packed struct
{
  rseSetReg_st    rseSetRegRAM;
  uint32_t        checsumInfo;               // Checsum = Sum of all bytes
} rseSetRegEEPROM_st;                                    


typedef struct
{
  sinapsiStates_e     stato;
  uint16_t            activeModule;
  rseSetRegEEPROM_st  rseSetRegEEPROM;
  sinapsiSetReg_st    sinapsiSetReg;
} sinapsiMng_t;


/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
void            sinapsiMngTask              (void * pvParameters);
xQueueHandle    getSinapsiMngQueueHandle    (void);
void            setSinapsiPinRS485          (void);
void            setSinapsiM1PdDefault       (uint16_t p1Val);
uint16_t        getSinapsiM1PdDefault       (void);
void            setSinapsiChain2Status      (uint16_t p1Val);
uint16_t        getSinapsiChain2Status      (void);


/**
  * @}
  */



#ifdef __cplusplus
}
#endif

#endif /* __SINAPSI_H */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/


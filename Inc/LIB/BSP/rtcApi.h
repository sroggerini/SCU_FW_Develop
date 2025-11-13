/**
* @file        rtcApi.h
*
* @brief       API for RTC   - Definition -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: rtcApi.h 599 2024-09-26 07:03:24Z stefano $
*
*     $Revision: 599 $
*
*     $Author: stefano $
*
*     $Date: 2024-09-26 09:03:24 +0200 (gio, 26 set 2024) $
*
*
* @copyright
*       Copyright (C) 2016 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

#ifndef __RTCAPI_H 
#define __RTCAPI_H

/************************************************************
 * Include
 ************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#else
#include "stm32h5xx_hal.h"
#endif
#include "cmsis_os.h"
 

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#define  MIN_TIME_BETWEEN_CALIB   ((uint32_t)172800)  /* tempo minimo per la calibrazione 2gg = 172800 sec */

#define  CAL_BLOCK_LEN            ((uint16_t)32)
#define  FREQ_RTC                 ((float)32768.0)
#define  MAX_CALM_VALUE           ((uint32_t)512)
#define  MIN_UNIX_TIME_VAL        ((uint32_t)1622675670)

/* STM data and time structure */
struct stmDataAndTime_t
{
  RTC_DateTypeDef   RTC_DateType;
  RTC_TimeTypeDef   RTC_TimeType;
};

/*
 * Struttura di lettura/scrittura
 * di DATA e ORA
 */
__packed struct DataAndTime_t
{
  unsigned char	  Hour;            
  unsigned char	  Minute;
  unsigned char	  Second;
  unsigned char	  Day;
  unsigned char	  Month;    //1..12
  unsigned short  Year;
  unsigned char	  DayWeek;  // 1..7 Monday...Sunday
  char            dstFlag;
};

typedef enum
{
  DOMENICA  = 0x0000,
  LUNEDI    = 0x0001,
  MARTEDI   = 0x0002,
  MERCOLEDI = 0x0003,
  GIOVEDI   = 0x0004,
  VENERDI   = 0x0005,
  SABATO    = 0x0006,
  NUM_DAY   = SABATO + 1,
  DUMMY_DAY = 0xFFFF
} dayOfWeek_e;

typedef struct
{
  uint8_t         oraLegFlag;
  uint8_t         dstStatus;
  time_t          startDstUnixTime;
  time_t          endDstUnixTime;
  int8_t          tZone;
} legalPeriod_st;


/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/
extern struct DataAndTime_t GlobalDT;
extern RTC_DateTypeDef sTamperDate;  
extern RTC_TimeTypeDef sTamperTime;
extern uint8_t TamperDetected;

extern void setDateTimeFromUnixT (uint32_t NewUT);  /* Ticket SCU-100 */

/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
uint8_t                 getDayOfWeek          (struct DataAndTime_t * pLocDateTime);
void                    DateTimeGet           (struct DataAndTime_t *DT_Get);
void                    DateTimeSet           (struct DataAndTime_t *DT_Set);
void                    UpdateGlobalDT        (void);
time_t                  getCurrentUnixTime    (void);
void                    setDateTimeFromUnixT  (uint32_t currUT);
struct DataAndTime_t*   getCurrentLocalTime   (uint32_t* pDTstring);
void                    checkLegalPeriod      (uint32_t locUT);
void                    setLegalPeriod        (uint32_t locUT);
char*                   getHmsStr             (void);

#endif //  __RTCAPI_H


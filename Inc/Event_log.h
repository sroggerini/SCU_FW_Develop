 /**
* @file        Event_log.h
*
* @brief       Event log management - Definition -
*
* @author      Roggerini S
*
* @riskClass   C 
*
* @moduleID  
*
* @vcsInfo
*     $Id: Event_log.h $
*
*     $Revision: $
*
*     $Author: $
*
*     $Date: $
*
*
* @copyright
*       Copyright (C) 2025 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

#ifndef __EVENT_LOG_H
	#define __EVENT_LOG_H

	/* Includes ------------------------------------------------------------------*/

#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#else
#include "stm32h5xx_hal.h"
#endif
#include "wrapper.h"
#include "time.h"

/* Event log is located in the last block (block 31) of the external flash 
   It takes about 8KB of space, every single entry is 32byte and so the capability is of 256 entries */

/* It's located in the Block 62 of the external flash and it takes 16KB of space */

#define EVLOG_EXTFLASH_ADDR_START     0x1F0000 
#define EVLOG_EXTFLASH_ADDR_END       0x1F3E80 
     
#define EVLOG_SIZE                    250
#define EVLOG_N_SECTOR                4

#define MAX_CHAR_FOR_LOGTEXT          48

#define FIRST_ENTRY_TO_STORE           0
#define FIRST_ENTRY_TO_READ            1
#define LAST_ENTRY_TO_READ             2

typedef enum {
  
  EV_ERROR,
  EV_ERROR_SEM,
  EV_ERROR_SIN,  
  EV_WARNING,
  EV_INFO,
  EV_INFO_SEM,
  EV_INFO_SIN,
  EV_DEBUG,  
  
} EventType;

/* Event log entry structure (59 byte lenght) */

typedef struct {
  
  time_t       TimeStamp; 
  uint16_t     Millisecond;
  EventType    EvType;
  char         EvDescription[MAX_CHAR_FOR_LOGTEXT];  
  
} EvLog_Entry_t;

/* Macro definition */

/* Global function definition */
void EVLOG_Message (EventType Type, const char *format, ...);

/* External variable definition */
extern EvLog_Entry_t EvLog_Entry;
extern void EVLOG_Message (EventType Type, const char *format, ...);
extern void EVLOG_Read (void);
extern void EVLOG_Erase_All (void);

#endif  /* __EVENT_LOG_H */

/**
* @file        Event log
*
* @brief       Event log - Implementation -
*
* @author      Roggerini S
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: Event_log.c 599 2024-09-26 07:03:24Z stefano $
*
*     $Revision: 1 $
*
*     $Author: stefano $
*
*     $Date: 2024-09-26 09:03:24 +0200 (gio, 26 set 2024) $
*
*
* @copyright
*       Copyright (C) 2025 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

/************************************************************
 * Include
 ************************************************************/
#include "Event_log.h"
#include "ExtFlash.h"
#include "rtcApi.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <telnet.h>

/********************** Variable definition *************************/

EvLog_Entry_t EvLog_Entry;

char EvLog_Entry_Type_Text[8][12] = 
{
   "EV_ERROR",
   "EV_ERROR_SEM",
   "EV_ERROR_SIN",  
   "EV_WARNING",
   "EV_INFO",
   "EV_INFO_SEM",
   "EV_INFO_SIN",
   "EV_DEBUG"
};

/********************** Function definition *************************/

/********************** External Function *************************/

extern sFLASH_Info sFLASH_Information;
extern infoStation_t  infoStation;

/**
  * @brief  Init Event log
  * @param  None
  * @retval None
  */

uint8_t is_Entry_NULL (uint8_t *pEvLog_Entry)
{
    uint8_t cnt;
    
    for (cnt = 0; cnt < sizeof (EvLog_Entry_t); cnt++, pEvLog_Entry++)
      if (*pEvLog_Entry != 0xFF)
        return FALSE;
    
    return TRUE;
}

void EVLOG_Init(void)
{
      /* IT'S EMPTY AT THE MOMENT */     
}

/**
  * @brief  Event log find entry address
  * @param  None
  * @retval Flash address where the entry must be stored
  */

uint32_t EVLOG_Find_Addr_Entry (uint8_t Type)
{
  
  EvLog_Entry_t  *pEvLog_Entry = &EvLog_Entry;  
  uint32_t       FlashAddr, FirstEntryFree;  
    
  switch (Type)
  {
      
    case FIRST_ENTRY_TO_STORE:
    case LAST_ENTRY_TO_READ:
      /* Find the first location free where is possible to save the log entry */
      for (FlashAddr = EVLOG_EXTFLASH_ADDR_START; FlashAddr < EVLOG_EXTFLASH_ADDR_END; FlashAddr += sizeof (EvLog_Entry_t))
      {
        /* Read the entry starting from the first location of the log */
        raw_sFLASH_Read ((uint8_t *)pEvLog_Entry, FlashAddr, sizeof (EvLog_Entry_t));
        /* Check if the entry is NULL --> is the LAST */
        if (is_Entry_NULL((uint8_t *)pEvLog_Entry) == TRUE)
          break;
      }
      
      /* Check if at the end of the log and if there is the need to erase a block */
      if (Type == FIRST_ENTRY_TO_STORE)
      {
        /* If the log is FULL, roll over */
        if (FlashAddr >= EVLOG_EXTFLASH_ADDR_END)
        {
          FlashAddr = EVLOG_EXTFLASH_ADDR_START;
          raw_sFLASH_Erase_SubBlock(FlashAddr);
        }
        /* if next location is at the beginning of the block, erase it */
        if ((FlashAddr + sizeof (EvLog_Entry_t)) % sFLASH_Information.EraseSectorSize == 0) 
          raw_sFLASH_Erase_SubBlock(FlashAddr + sizeof (EvLog_Entry_t));
      }      
      break;
      
    case FIRST_ENTRY_TO_READ:
      /* Find the first entry free */
      for (FlashAddr = EVLOG_EXTFLASH_ADDR_START; FlashAddr < EVLOG_EXTFLASH_ADDR_END; FlashAddr += sizeof (EvLog_Entry_t))
      {
        /* Read the entry starting from the first location of the log */
        raw_sFLASH_Read ((uint8_t *)pEvLog_Entry, FlashAddr, sizeof (EvLog_Entry_t));
        /* Check if the entry is NULL */
        if (is_Entry_NULL((uint8_t *)pEvLog_Entry) == TRUE)
          break;
      }
      
      FirstEntryFree = FlashAddr;
        
      /* Find the first entry written */
      while (1)
      {        
        /* Read the entry starting from the first location of the log */
        raw_sFLASH_Read ((uint8_t *)pEvLog_Entry, FlashAddr, sizeof (EvLog_Entry_t));
        /* Check if the entry is NOT NULL --> is the first */
        if (is_Entry_NULL((uint8_t *)pEvLog_Entry) == FALSE)          
          break; 
        
        FlashAddr += sizeof (EvLog_Entry_t);
        /* if at the end of the log area , */
        if (FlashAddr >= EVLOG_EXTFLASH_ADDR_END)
        {
          FlashAddr = EVLOG_EXTFLASH_ADDR_START;
          if (FirstEntryFree == EVLOG_EXTFLASH_ADDR_START)
            break;
        }
      }      
          
      break;
      
  }
  
  /* Return address */
  return FlashAddr;
}

/**
  * @brief  Event log message formatter
  * @param  Event log type, format of the string ......
  * @retval None
  */
void EVLOG_Message (EventType Type, const char *format, ...)
{
   
  va_list       args;  
  uint32_t      FlashAddr;  
  EvLog_Entry_t *pEvLog_Entry = &EvLog_Entry;
  
  /* Find free slot where the entry must be stored */
  FlashAddr = EVLOG_Find_Addr_Entry (FIRST_ENTRY_TO_STORE);
    
  /* Manage function with a variable number of arguments */
  va_start (args, format);   
  /* Copy parameters into the event log entry struct */
  /* Get timestamp */
  pEvLog_Entry->TimeStamp = getCurrentUnixTime();
  /* Save millisecond */
  pEvLog_Entry->Millisecond = HAL_GetTick() % 1000;
  /* Set event type */
  pEvLog_Entry->EvType = Type;
  /* Write the message into the entry */
  vsnprintf (pEvLog_Entry->EvDescription, MAX_CHAR_FOR_LOGTEXT, format, args);   
  /* Save the entry into the ext flash */
  raw_sFLASH_Write ((uint8_t *)pEvLog_Entry, FlashAddr, sizeof (EvLog_Entry_t));
  
}

/**
  * @brief  Show the entry of EVLOG (on Teraterm at the moment)
  * @param  None
  * @retval None
  */

void EVLOG_Show_Entry (uint8_t ID, EvLog_Entry_t *EvLog_Entry)
{
  struct tm   *pTime;
   
  /* Update globat date and time structure  **/
  pTime = localtime (&EvLog_Entry->TimeStamp);
  
  switch (EvLog_Entry->EvType)
  {
    case EV_ERROR:
    case EV_ERROR_SEM:
    case EV_ERROR_SIN:
      /* Print entry informations */
      tPrintf("#%03d [%04d-%02d-%02d %02d:%02d:%02d:%03d]   \033[31m%-12s\033[0m %-49s\r\n", 
              ID, pTime->tm_year + 1900, pTime->tm_mon, pTime->tm_mday, pTime->tm_hour, 
              pTime->tm_min, pTime->tm_sec, EvLog_Entry->Millisecond, EvLog_Entry_Type_Text[EvLog_Entry->EvType], EvLog_Entry->EvDescription);      
     break;
     
    default:
      /* Print entry informations */
      tPrintf("#%03d [%04d-%02d-%02d %02d:%02d:%02d:%03d]   %-12s %-49s\r\n", 
              ID, pTime->tm_year + 1900, pTime->tm_mon, pTime->tm_mday, pTime->tm_hour, 
              pTime->tm_min, pTime->tm_sec, EvLog_Entry->Millisecond, EvLog_Entry_Type_Text[EvLog_Entry->EvType], EvLog_Entry->EvDescription);      
     break;
  }
  
}

/**
  * @brief  Read the entire EVLOG
  * @param  None
  * @retval None
  */
void EVLOG_Read (void)
{
  uint8_t       ID = 0;
  uint32_t      LastEntry_addr, FirstEntry_addr, CurrentEntry_Addr;  
  EvLog_Entry_t *pEvLog_Entry = &EvLog_Entry;

  /* Find the 1st entry stored --> the first after a NULL entry */
  FirstEntry_addr = EVLOG_Find_Addr_Entry (FIRST_ENTRY_TO_READ);
  /* Find the last entry stored --> the last before a NULL entry */
  LastEntry_addr = EVLOG_Find_Addr_Entry (LAST_ENTRY_TO_READ);
  
  /* Start form the first entry */
  CurrentEntry_Addr = FirstEntry_addr;
  
  /* Show on Teraterm at the moment */
  tPrintf ("----------------------------------------------------------------------------------\r\n");
  tPrintf (" EVENT LOG for SCU SN %s8\r\n", infoStation.productSn);
  tPrintf ("----------------------------------------------------------------------------------\r\n");
  tPrintf ("  #      TIMESTAMP                TYPE                     DESCRIPTION            \r\n\n");
  
  /* if the First entry addr is equivalent to the Last entry addr */
  if (CurrentEntry_Addr == LastEntry_addr)
  {
    tPrintf ("                           LOG IS EMPTY!!                                       \r\n");
    return;
  }
  
  /* while the last entry is found */
  while (CurrentEntry_Addr != LastEntry_addr)
  {      
    /* Read the entry at the flash address  */
    raw_sFLASH_Read ((uint8_t *)pEvLog_Entry, CurrentEntry_Addr, sizeof (EvLog_Entry_t));
    /* Show the log on Teraterm */
    EVLOG_Show_Entry (++ID, pEvLog_Entry);
    /* go to the next entry to show */
    CurrentEntry_Addr += sizeof (EvLog_Entry_t);
    /* if at the end of the log, start from the beginning of the flash area */
    if (CurrentEntry_Addr >= EVLOG_EXTFLASH_ADDR_END)
      CurrentEntry_Addr = EVLOG_EXTFLASH_ADDR_START;
  }

  tPrintf ("--------------------------- END OF LOG -------------------------------------------\r\n");
  
}

/**
  * @brief  Erase all the event log
  * @param  None
  * @retval None
  */
void EVLOG_Erase_All (void)
{
  
  uint32_t FlashAddr;  
  
  /* Erase the 4 blocks used to manage the LOG */
  for (FlashAddr = EVLOG_EXTFLASH_ADDR_START; FlashAddr < EVLOG_EXTFLASH_ADDR_END; FlashAddr += sFLASH_Information.EraseSectorSize)
  {
     /* Erase 4K sector */
     if (raw_sFLASH_Erase_SubBlock (FlashAddr) != HAL_OK)
       return;
  } 
  
  tPrintf ("Event log erased!!\r\n");     
  
}




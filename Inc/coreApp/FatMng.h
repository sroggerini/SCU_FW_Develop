/******************* (C) COPYRIGHT 2009 STMicroelectronics *********/
/**
* @file        FatMng.c
*
* @brief       Small FAT manager - Definition -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: FatMng.h 76 2022-06-20 09:46:05Z npiergi $
*
*     $Revision: 76 $
*
*     $Author: npiergi $
*
*     $Date: 2022-06-20 11:46:05 +0200 (lun, 20 giu 2022) $
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
#ifndef __FATMNG_H
#define __FATMNG_H

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>
#include "ff.h"

/*
*********************************** BM ***************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
/* Telenet Task message size   */
#define   FAT_MAX_MESSAGE_NUM     ((uint8_t)2)
#define   FAT_MSG_SIZE            ((uint16_t)8)

typedef enum
{
  FAT_STATE_IDLE   = 0x00,       /* initial state   */ 
  FAT_STATE_NOT_FOUND,           /* state when u SD not found     */ 
  FAT_STATE_READY,               /* ready to process the first message   */ 
  FAT_STATE_FILE_CHECK,          /* check the precence particular files   */ 
  FAT_STATE_FILE_PROCESS,        /* wait for file processing              */ 
  FAT_STATE_OPERATIVE,           /* normal state   */ 
  FAT_NUM_STATE,
  FAT_STATE_DUMMY = 0xFFFF
} fatStates_e;

typedef enum
{
  FAT_EVENT_TIMEOUT = 0x00,      /* Timeout event           */ 
  FAT_EVENT_INIT,                /* initialization event    */ 
  FAT_EVENT_WRITE,               /* WRITE file event        */ 
  FAT_EVENT_READ,                /* READ file event         */ 
  FAT_EVENT_FW_FILE_DELETE,      /* Erase FW bin file       */ 
  FAT_EVENT_DUMMY = 0xFFFF
} fatEvents_e;


typedef struct
{
  uint8_t       bufferMsg[FAT_MSG_SIZE];
  uint16_t      lenMsg;
  fatEvents_e   taskEv;
} fatMngMsg_st;

typedef struct
{
  fatStates_e   stato;
  uint16_t      ev;
  uint16_t      action;
} fatMng_t;


/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
void          fatMngTask            (void * pvParameters);
xQueueHandle  getFatMngQueueHandle  (void);
void          setActionForFatMng    (uint16_t action);

/**
  * @}
  */



#ifdef __cplusplus
}
#endif

#endif /* __TELNET_H */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/


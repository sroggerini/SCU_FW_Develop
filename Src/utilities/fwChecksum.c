/**
* @file        fwCheckSum.c
*
* @brief       File where will be stored the FW checksum - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: fwChecksum.c 599 2024-09-26 07:03:24Z stefano $
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
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#ifdef GD32F4xx
#include "stm32f4xx_hal.h" 
#else
#include "stm32h5xx_hal.h" 
#endif
#include "main.h" 
    
/*
***********************************Aesys**************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
/*
***********************************Aesys**************************************
**                                                                          **
**                            Local Const                                   **
**                                                                          **
****************************************************************************** 
*/ 
#ifdef FULL_DEBUG
const infoFw_u fwInfo @ INFO_FOR_FW_UPDATE_ADDR = {.infoNewFw = {(uint32_t)0x0000ABCD, FW_CHECK_STR, FW_VERSION, (uint32_t)FW_OFFSET_ADDRESS, (uint32_t)123456}};
/*                                                                            
__root const uint32_t checkVal  @ CHECKSUM_ADDRESS       = (uint32_t)0x0000ABCD; 
__root const uint32_t tagStr1   @ (CHECKSUM_ADDRESS+4)   = (uint32_t)0x4D414353; 
__root const uint32_t tagStr2   @ (CHECKSUM_ADDRESS+8)   = (uint32_t)0x57465F45; 
__root const uint32_t fwTotLen  @ (CHECKSUM_ADDRESS+40)  = (uint32_t)123456; 
*/ 
#else

const infoFw_u fwInfo @ INFO_FOR_FW_UPDATE_ADDR = {.infoNewFw = {(uint32_t)0, FW_CHECK_STR, FW_VERSION, (uint32_t)FW_OFFSET_ADDRESS, (uint32_t)0}};
//const uint8_t fwVersion[16] @ FW_VERSION_ADDRESS = FW_VERSION; 
#endif

/*************** END OF FILE ******************************************************************************************/
 

/**
* @file        DataLink_dbg.h
*
* @brief       Data Link layer SIemens Traffic Outdoor Station protocol - Definition -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: DataLink_dbg.h 133 2022-09-19 13:37:22Z npiergi $
*
*     $Revision: 133 $
*
*     $Author: npiergi $
*
*     $Date: 2022-09-19 15:37:22 +0200 (lun, 19 set 2022) $
*
*
* @copyright
*       Copyright (C) 2016 SCAME S.p.A. All rights reserved. This file is copyrighted and the property of Aesys
*       S.p.A.. It contains confidential and proprietary information. Any copies of this file (in whole or in part)
*       made by any method must also include a copy of this legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

#ifndef __DATALINK_DBG_H 
#define __DATALINK_DBG_H

/************************************************************
 * Include
 ************************************************************/
#include <stdint.h>
#include <stdlib.h>
 

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/

/* Definition for USART   clock resources We use UART2 PD6=Rx and PD5=Tx*/

/* ----------- numero buffer nella coda di ricezione  -------------*/
#define   NUM_BUFF_DBG_DL_RX             ((uint16_t)8)   
//#define   NUM_BUFF_DBG_DL_RX             ((uint16_t)1)   
#define   NUM_BUFF_DBG_DL_TX             ((uint16_t)1)      



/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
void          dl_DbgRxProcess          (void * pvParameters);
void          dl_DbgTxProcess          (void * pvParameters);
xQueueHandle  getDbgDlRxQueueHandle    (void);
xQueueHandle  getDbgDlTxQueueHandle    (void);

#endif //  __DATALINK_DBG_H


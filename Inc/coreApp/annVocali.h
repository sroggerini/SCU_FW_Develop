/******************* (C) COPYRIGHT 2009 STMicroelectronics *********/
/**
* @file        annVocali.h
*
* @brief       Dati per annunci vocali - Definition -
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: annVocali.h 76 2022-06-20 09:46:05Z npiergi $
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
#ifndef __ANN_VOCALI_H
#define __ANN_VOCALI_H

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>
#include "wrapper.h"

/*
*********************************** BM ***************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/


typedef enum
{
  ANN_GP = 0,      
  ANN_ESTRARRE_COPERCHIO,              
  ANN_SOLLEVARE_LA_LEVA,
  NUM_ANN        
} annIdx_e;


typedef struct
{
  uint16_t        sizeAnn;
  uint8_t*        pAnn;
} annVoce_st;




/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/

/**
  * @}
  */



#ifdef __cplusplus
}
#endif

#endif /* __ANN_VOCALI_H */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/


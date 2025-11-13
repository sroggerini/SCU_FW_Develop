/******************* (C) COPYRIGHT 2025 Scame Parre spa *********/
/**
* @file        homeplugdev.h
*
* @brief       Management of ISO15118 protocol  - Definition -
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: homeplugdev.h 
*
*     $Revision:  $
*
*     $Author: $
*
*     $Date: 
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
#ifndef __ISO15118_MNG_H
#define __ISO15118_MNG_H

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>
#include "cmsis_os.h"
#include "homeplugdev_spi.h"
   
/*
*********************************** BM ***************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
//#define ISO15118_TASK_TICK   (pdMS_TO_TICKS(60))
#define ISO15118_TASK_TICK   (pdMS_TO_TICKS(20))


/* input  message size   */
#define ISO15118_MAX_MESSAGE_NUM   ((uint8_t)2)
   
#define DB_STRING_LEN 64

typedef enum
{
  ISO15118_EV_TIMEOUT  = 0x00,      
  ISO15118_EV_START,                
  ISO15118_EVENT_POLLING,
  ISO15118_EVENT_STOP,
  ISO15118_EV_DUMMY    = 0xFFFF
} ISO15118Ev_e;
  
      
typedef enum
{
  ISO15118_START       = 0x00,      /* start activity              */ 
  ISO15118_STOP,                    /* stop  activity              */ 
  ISO15118_SUSPEND,
  ISO15118_VALUE_DUMMY = 0xFFFF
} ISO15118Value_e;

typedef struct
{
  ISO15118Ev_e      taskEv;
  ISO15118Value_e   taskValue;
} ISO15118Msg_st;

typedef enum timeouts {
	timerCCSHomeplug,
	timerHPGP,
	timerLast
} chademo_timeout_t;

typedef enum {
	ccs2StateA = 0U, /* The EV is not connected to the Charging Station (Unmated) */
	ccs2StateB1,     /* The EV is connected to the Charging Station (Mated) */
	ccs2StateB2,     /* The EV is connected to the Charging Station and PWM is 5% (initialize) */
	ccs2StateC1,     /* The EV is connected to the Charging Station and ready for charging (no ventilation requested) */
	ccs2StateC2,     /* The EV is connected to the Charging Station and ready for charging (no ventilation requested) */
	ccs2StateD2,     /* The EV is connected to the Charging Station and ready for charging (ventilation requested) */
	ccs2StateE,      /* Problem with the grid or no grid connection */
	ccs2StateF       /* Charging Station is not available. */
} ccs2_cp_state_t;

enum info_host_param_sel {
    ISO15118_MAXCURRENT,
    ISO15118_STOPEVSE,
    ISO15118_STATECP,
    ISI15118_PAUSEEVSE,
    ISO15118_EMERGENCY
};

typedef struct
{
  
  uint8_t       State_openv2g;
  uint8_t       Term_openv2g;
  uint8_t       State_Slac;
  uint8_t       State_SDP;
  uint8_t       Error_Code;
  uint8_t       evccId[EVCCID_STRING_LEN];
  uint32_t      ev_Departure_Time;
  uint16_t      ev_Max_Current_Limit;
  uint16_t      ev_Min_Current;
  uint16_t      ev_Max_Voltage_Limit;
  uint16_t      ev_Energy_Request;
  uint8_t       ev_Protocol;
  uint8_t       serviceIdReq;
  
} ISO15118_Info_Device_st;

/*************    ONLY FOR DEBUG START  ****************/
typedef struct
{
  uint16_t      maxCurrent;
  uint16_t      maxPower;
  uint8_t   	freeService;
} DEBUG_ISO15118_st;
/*************    ONLY FOR DEBUG END  ****************/

typedef struct
{
  uint8_t  board_check_presence;
  uint8_t  board_detected;
  uint8_t  check_communication;
  uint8_t  comm_ongoing;
  uint8_t  tracement;
} ISO15118_Status_st;

typedef char     string_t[DB_STRING_LEN];
typedef uint8_t  byte_t;

/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
void            ISO15118Task                        (void * pvParameters);
void            ISO15118_homeplugdev_info_host_set  (uint8_t info_sel, uint16_t val);


/**
  * @}
  */

/* Variable exported by this module */    
extern uint32_t timerCounter[timerLast];
extern ISO15118_Info_Device_st  ISO15118_Info_Device;
extern ISO15118_Status_st       ISO15118_Status;
extern uint8_t is_EvccId_NULL (void);

/* Function exported by this module */
extern void   ISO15118_Task_SendEv (ISO15118Ev_e Event);
//extern ccs2_cp_state_t  ISO15118_get_StateCP(void);
//extern uint8_t  ISO15118_get_StopEVSE(void);

#ifdef __cplusplus
}
#endif

#endif /* __ISO15118_MNG_H */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/


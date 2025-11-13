/**
* @file        PwmMng.c
*
* @brief       Power Management  - Definition -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: PwmMng.h 528 2024-06-19 12:36:17Z vania $
*
*     $Revision: 528 $
*
*     $Author: vania $
*
*     $Date: 2024-06-19 14:36:17 +0200 (mer, 19 giu 2024) $
*
*
* @copyright
*       Copyright (C) 2016 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#ifndef _PWMMNG_H
#define _PWMMNG_H
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global include ------------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global defines ------------------------------------------------------------------------------------------------------------------------- //
#define M3T_CURRENT_MIN     (uint16_t)(60)     // corrente minima evs in modo 3 standard [A*10]
#define M3T_CURRENT_MAX     (uint16_t)(630)    // corrente massima evs in modo 3 standard [A*10]
#define M3S_CURRENT_MIN     (uint16_t)(60)     // corrente minima evs in modo 3 semplificato [A*10]
#define M3S_CURRENT_MAX     (uint16_t)(160)    // corrente massima evs in modo 3 semplificato [A*10]
#define EVS_CURRENT_MIN     (uint16_t)(60)     // corrente minima evs da norma
#define EVS_CURRENT_MAX     (uint16_t)(630)    // corrente massima evs in modo 3 standard [A*10]

#define EVS_START_CURRENT   (uint16_t)(320)    // massimo pwm all'avvio [espresso in corrente] per evitare errate letture sulla presenza del diodo
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global typedef ------------------------------------------------------------------------------------------------------------------------- //
typedef enum
{
PWM_EVENT_NULL = 0,
PWM_OUTPUT_DC_START,
PWM_OUTPUT_HIGH,
PWM_OUTPUT_LOW,
PWM_CONTROL_TIMEOUT,
PWM_ISO15118_TIMEOUT,
PWM_INTERNAL_EM_GOOD,
PWM_INTERNAL_EM_FAIL,
PWM_EXTERNAL_EM_GOOD,
PWM_EXTERNAL_EM_FAIL,
PWM_SINAPSI_UPDATE, // update info from Sinapsi (M1 chain 2)
PWM_MDB_POWER_UPDATE
}PwmMngEvent_en;

/* queue info structure */
typedef __packed struct
{
PwmMngEvent_en    PwmMngEvent;
}PwmMngMsg_st;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global function prototypes ------------------------------------------------------------------------------------------------------------- //
void send_to_pwm(uint8_t pwm_event);
void pwm_currents_byte_get(uint8_t *dst_ptr);
void pwm_currents_get(uint8_t *dst_ptr);
void active_phases_num_set(uint8_t val);
void pmng_pid_enable_set(uint8_t val);
uint8_t pmng_pid_enable_get(void);
void pwm_iso15118_set(void);
void pwm_iso15118_clr(void);
void pmng_suspending_enable_set(void);
void pmng_sem_power_set(uint16_t available_power, uint8_t sem_update);
uint8_t pmng_suspending_enable_get(void);
void pmng_ext_active_power_update(void);
uint8_t pmng_sinapsi_mode_get(void);
void PwmMngTask(void *pvParameters);
uint8_t pmng_app_schedule_mode_get(void);
void pmng_app_schedule_mode_set(uint8_t mode);
unsigned int getAvailablePower_Flag(uint8_t* pFlagPM, int16_t* pidError, uint16_t *pPotMinRestart);
void PM_Mdb_to_EEprom_Translate (uint16_t rAddr);
uint8_t is_pwm_CP_Active (void);


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#endif

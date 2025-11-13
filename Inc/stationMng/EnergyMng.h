/**
* @file        EnergyMng.h
*
* @brief       Energy manager - Definition -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: EnergyMng.h 485 2024-05-28 09:27:37Z npiergi $
*
*     $Revision: 485 $
*
*     $Author: npiergi $
*
*     $Date: 2024-05-28 11:27:37 +0200 (mar, 28 mag 2024) $
*
*
* @copyright
*       Copyright (C) 2016 SCAME S.p.A. All rights reserved. This file is copyrighted and the property of Aesys
*       S.p.A.. It contains confidential and proprietary information. Any copies of this file (in whole or in part)
*       made by any method must also include a copy of this legend. Developed by:  SCAME S.p.A.
***********************************************************************************************************************/


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#ifndef _ENERGYMNG_H
#define _ENERGYMNG_H
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global include ------------------------------------------------------------------------------------------------------------------------- //
#include "Em_Task.h"
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global defines ------------------------------------------------------------------------------------------------------------------------- //

/* LIVE measures array */
static int32_t                energy_array[EM_READ_REG_NUM];

#define EMETER_THREE_PH     (uint8_t)(0x80)

/* -------------------------------------------------------------------------- */
/*                 LIVE measures definition                                   */
/* -------------------------------------------------------------------------- */

#define SYS_VOLTAGE             energy_array[EM_SYS_VOLTAGE]            /*  0 */
#define SYS_PH1_VOLTAGE         energy_array[EM_SYS_PH1_VOLTAGE]        /*  1 */
#define CURRENT_L1              energy_array[EM_CURRENT_L1]             /*  2 */
#define CURRENT_L2              energy_array[EM_CURRENT_L2]             /*  3 */
#define CURRENT_L3              energy_array[EM_CURRENT_L3]             /*  4 */
#define CURRENT_L               energy_array[EM_CURRENT_L]              /*  5 */
#define COS_PHI                 energy_array[EM_COS_PHI]                /*  6 */
#define COS_PH1_PHI             energy_array[EM_COS_PH1_PHI]            /*  7 */
#define ACTIVE_POWER            energy_array[EM_ACTIVE_POWER]           /*  8 */
#define REACTIVE_POWER          energy_array[EM_REACTIVE_POWER]         /*  9 */
#define TOT_ACTIVE_ENERGY       energy_array[EM_TOT_ACTIVE_ENERGY]      /* 10 */
#define TOT_REACT_ENERGY        energy_array[EM_TOT_REACT_ENERGY]       /* 11 */
#define SES_ACTIVE_ENERGY       energy_array[EM_SES_ACTIVE_ENERGY]      /* 12 */
#define EXT_ACTIVE_POWER        energy_array[EM_EXT_ACTIVE_POWER]       /* 13 */
#define EXT_L1_ACTIVE_POWER     energy_array[EM_EXT_L1_ACTIVE_POWER]    /* 14 */ 
#define EXT_L2_ACTIVE_POWER     energy_array[EM_EXT_L2_ACTIVE_POWER]    /* 15 */ 
#define EXT_L3_ACTIVE_POWER     energy_array[EM_EXT_L3_ACTIVE_POWER]    /* 16 */ 
#define EXT_SIGN_POWER          energy_array[EM_EXT_SIGN_POWER]         /* 17 */ 


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global typedef ------------------------------------------------------------------------------------------------------------------------- //
typedef enum    // possible value of EmeterType
{
EMETER_TYPE_NULL         = (uint8_t)(0x00),
EMETER_TAMP              = (uint8_t)(0x01),
EMETER_TAMP_3            = (uint8_t)(EMETER_THREE_PH | 0x01),
EMETER_MONO_PH_GAVAZZI   = (uint8_t)(0x04),
EMETER_MONO_PH_ALGO2     = (uint8_t)(0x14),
EMETER_MONO_PH_SCAME     = (uint8_t)(0x20),
EMETER_MONO_PH_LOVATO    = (uint8_t)(0x40),
EMETER_SINAPSI           = (uint8_t)(0x60),
EMETER_THREE_PH_GAVAZZI  = (uint8_t)(EMETER_THREE_PH | EMETER_MONO_PH_GAVAZZI),  // 0x80 + 0x04 = 0x84
EMETER_THREE_PH_ALGO2    = (uint8_t)(EMETER_THREE_PH | EMETER_MONO_PH_ALGO2),    // 0x80 + 0x14 = 0x94
EMETER_THREE_PH_SCAME    = (uint8_t)(EMETER_THREE_PH | EMETER_MONO_PH_SCAME),    // 0x80 + 0x20 = 0xA0
EMETER_THREE_PH_LOVATO   = (uint8_t)(EMETER_THREE_PH | EMETER_MONO_PH_LOVATO),   // 0x80 + 0x40 = 0xC0
EMETER_MONO_PH_PA775     = (uint8_t)(0x25),
EMETER_THREE_PH_PA775    = (uint8_t)(EMETER_THREE_PH | EMETER_MONO_PH_PA775),
}EmeterType_en;

#define EMETER_TYPE_MIN    (EMETER_TYPE_NULL)
#define EMETER_TYPE_MAX    (EMETER_THREE_PH_ALGO2) 

typedef enum
{
ENERGY_EVENT_NULL = 0,
ENERGY_CONTROL_START,
ENERGY_CONTROL_STOP,
ENERGY_INTERNAL_EM_GOOD,
ENERGY_INTERNAL_EM_FAIL,
ENERGY_EXTERNAL_EM_GOOD,
ENERGY_EXTERNAL_EM_FAIL,
ENERGY_CONTROL_TIMEOUT
}EnergyMngEvent_en;

/* queue info structure */
typedef __packed struct
{
EnergyMngEvent_en   EnergyMngEvent;
}EnergyMngMsg_st;

typedef enum    // possible value of LCD Type
{
  LCD_TYPE_NULL         = (uint8_t)(0x00),
  LCD_2X20,
  LCD_TYPE_NUM,
  LCD_TYPE_MASK         = (uint8_t)(0x03)
}lcdType_e;

typedef enum    // possible value of wifi type --> reference in eeprom: LCD_TYPE_EADD bit 2
{
  WIFI_OFF         = (uint8_t)(0x00),
  WIFI_ON          = (uint8_t)(0x04),
  WIFI_MASK        = (uint8_t)(0x04)
}wifiMode_e;


typedef enum    // possible value of differ. Riarm type --> reference in eeprom: LCD_TYPE_EADD bit 3
{
  DIRI_OFF         = (uint8_t)(0x00),
  DIRI_ON          = (uint8_t)(0x08),
  DIRI_MASK        = (uint8_t)(0x08)
}diriMode_e;

typedef enum    // possible value of wifi&SBC --> : LCD_TYPE_EADD bit 4
{
  SBC_WIFI_OFF     = (uint8_t)(0x00),
  SBC_WIFI_ON      = (uint8_t)(0x10),
  SBC_WIFI_MASK    = (uint8_t)(0x10)
}wifiSbcMode_e;

typedef enum    // possible value of Access Point / router mode --> : LCD_TYPE_EADD bit 5
{
  WIFI_AP          = (uint8_t)(0x00),
  WIFI_ROUTER      = (uint8_t)(0x20),
  WIFI_MODE_MASK   = (uint8_t)(0x20)
}wifiFuncMode_e;

typedef enum    // possible value of socket Position in EEPROM
{
  SKT_HIHG_DX         = (uint8_t)(0x00),
  SKT_HIHG_SX         = (uint8_t)(0x40),
  SKT_LOW_DX          = (uint8_t)(0x80),
  SKT_LOW_SX          = (uint8_t)(0xC0),
  SKT_POS_MASK        = (uint8_t)(0xC0)
}sktPos_e;

typedef enum    // possible value of socket Position in MODBUS MAP reg ADDR_CONNECTOR_N_RW 0x59
{
  SKT_HIHG_DX_MB      = (uint8_t)(0x02),
  SKT_HIHG_SX_MB      = (uint8_t)(0x01),
  SKT_LOW_DX_MB       = (uint8_t)(0x04),
  SKT_LOW_SX_MB       = (uint8_t)(0x03),
}sktPosMb_e;



// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global function prototypes ------------------------------------------------------------------------------------------------------------- //
void send_to_energy(EnergyMngEvent_en energy_event);
void energy_current_update(uint32_t current);
void energy_param_get(emReadReg_e energy_parameter, int32_t *dst_ptr, uint8_t num);
void energy_to_gsy_get(emReadReg_e energy_parameter, int32_t *dst_ptr, uint8_t num);
uint8_t energymng_running_get(void);
void EnergyMngTask(void *pvParameters);
void i_parameters_reset(void);

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#endif

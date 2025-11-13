/**
* @file        lcd.c
*
* @brief       API and Managerdisplay LCD 2x20 / 4x 40 - Definition -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: lcd.h 636 2024-11-22 14:49:00Z stefano $
*
*     $Revision: 636 $
*
*     $Author: stefano $
*
*     $Date: 2024-11-22 15:49:00 +0100 (ven, 22 nov 2024) $
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

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#ifndef _LCD_H
#define _LCD_H
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global include ------------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global defines ------------------------------------------------------------------------------------------------------------------------- //
#define     TIMER_TICK_TO_INIT          ((uint16_t)100)
#define     TIMER_TICK_TO_HOLD          ((uint16_t)10000)

#define     MIN_TIME_EN_INI_VAL         ((uint16_t)300)   

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global typedef ------------------------------------------------------------------------------------------------------------------------- //

typedef enum
{
  LCD_INIT = 0,
  LCD_OPERATIVE,
  LCD_FAIL_CHECK,
  LCD_TIMEOUT,
  LCD_NUM_EVENTS
}lowLevelEvent_en;

typedef enum
{
  LCD_STATE_IDLE = 0,
  LCD_STATE_OPERATIVE,
  LCD_STATE_FAIL,
  LCD_STATE_HOLD_ON_FAIL
}lowLevelState_en;

/* queue info structure */
typedef __packed struct
{
  lowLevelState_en    state;
}lowLevelLcd_st;

/* queue info structure */
typedef __packed struct
{
  lowLevelEvent_en    lowLevelEvent;
}lowLevelLcdMsg_st;

typedef enum 
{
  AS_STANDARD_MODE,
  AS_BACKUP_MODE
}LcdInitState;

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global function prototypes ------------------------------------------------------------------------------------------------------------- //
void          lowLevelLcdMngTask      (void *pvParameters);
void          sendMessageToLcd        (lowLevelEvent_en alarmLcd);
void          setUpgradeLcd           (upgLcd_e status);

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

extern        lcdCntt_e          cnttChange;
extern        void               Lcd2x20FastInit     (void);
extern        void               BACKUP_Manage_Progress_Bar(uint16_t Vin);
extern        void               triggerVINConfig    (void);
extern        uint8_t            LCD_Check_StatusBar_Presence(void);

#endif  // LCD.H

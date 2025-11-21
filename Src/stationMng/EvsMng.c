/**
* @file        evsMng.c
*
* @brief       Event manager for charging protocol   - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: EvsMng.c 762 2025-05-30 15:11:39Z stefano $
*
*     $Revision: 762 $
*
*     $Author: stefano $
*
*     $Date: 2025-05-30 17:11:39 +0200 (ven, 30 mag 2025) $
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
//
//  File:           EvsMng.c
//  Author:         Vania
//  Date:           06/11/2020
//
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local include -------------------------------------------------------------------------------------------------------------------------- //
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_rcc.h"
#else
#include "stm32h5xx_hal.h"
#include "stm32h5xx_ll_rcc.h"
#endif
#include "cmsis_os.h"

#include <string.h>

#include "adcTask.h"
#include "eeprom.h"
#include "wrapper.h"

#include "BlockMng.h"
#include "ContactMng.h"
#include "EnergyMng.h"
#include "EvsMng.h"
#include "EvsTimeMng.h" 
#include "ExtInpMng.h"
#include "InputsMng.h"
#include "ioExp.h"
#include "LcdMng.h"
#include "PersMng.h"
#include "PilotMng.h"
#include "PwmMng.h"
#include "RfidMng.h"

#include "sbcGsy.h"
#include "sinapsi.h"

#include "sbcSem.h"
#include "scuMdb.h"
#include "diffRiarm.h"

#ifndef HW_MP28947
#include "homeplugdev_spi.h"
#include "homeplugdev.h"
#endif

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local defines -------------------------------------------------------------------------------------------------------------------------- //
#define RENAULT_EV_READY

#define EVS_WAIT_DEF_TIME           pdMS_TO_TICKS((uint32_t)(60000))
#define EVS_GARD_TIME               pdMS_TO_TICKS((uint32_t)(100))
#define EVS_SEC_TIME                pdMS_TO_TICKS((uint32_t)(1000))
#define CPSET_STATE_TIME            pdMS_TO_TICKS((uint32_t)(100))
#define DELAY_STATE_TIME            pdMS_TO_TICKS((uint32_t)(2000))
#define PLUG_CHECK_TIME             pdMS_TO_TICKS((uint32_t)(30000))
#define BLOCK_UP_DELAY_TIME         pdMS_TO_TICKS((uint32_t)(1500))
#define BLOCK_DOWN_DELAY_TIME       pdMS_TO_TICKS((uint32_t)(500))
#define CONTACT_CLOSE_DELAY_TIME    pdMS_TO_TICKS((uint32_t)(1000))
#define WAKEUP_STATE_A_TIME         pdMS_TO_TICKS((uint32_t)(3000))
#define WAKEUP_STATE_B_TIME         pdMS_TO_TICKS((uint32_t)(750))
#define EVSTATE_WAITING_10S         pdMS_TO_TICKS((uint32_t)(10000))
#define EVSTATE_WAITING_30S         pdMS_TO_TICKS((uint32_t)(30000))
#define EVSTATE_WAITING_60S         pdMS_TO_TICKS((uint32_t)(60000))
#define INTERRUPTING_CHARGE_TIME    pdMS_TO_TICKS((uint32_t)(7000))
#define EVS_LID_TIME                pdMS_TO_TICKS((uint32_t)(1000))
#define ERROR_WAIT_TIME             pdMS_TO_TICKS((uint32_t)(3000))
#define CPPP_WAIT_TIME              pdMS_TO_TICKS((uint32_t)(20000))
#define RESETTABLE_ERROR_TIME       pdMS_TO_TICKS((uint32_t)(2500))
#define F_STATE_ERROR_TIME          pdMS_TO_TICKS((uint32_t)(3000))
#define X1_STATE_ERROR_TIME         pdMS_TO_TICKS((uint32_t)(500))
#define F_X1_RES_ERROR_TIME         pdMS_TO_TICKS((uint32_t)(1500))
#define EVS_TPRINT_TIME             pdMS_TO_TICKS((uint32_t)(500))
#define EVS_UID_ERROR_TIME          pdMS_TO_TICKS((uint32_t)(3000))
#define AUTH_MISSED_TIME            pdMS_TO_TICKS((uint32_t)(2500))
#define EVS_MODE_CHECK_TIME         pdMS_TO_TICKS((uint32_t)(1000))
#define EMETER_INT_ANOM1_TIME       pdMS_TO_TICKS((uint32_t)(2500))
#define SBC_GSY_TIME                pdMS_TO_TICKS((uint32_t)(12000))
#define EVS_TIMESTAMP_TIME          pdMS_TO_TICKS((uint32_t)(1000000))
#define LID_ERROR_DELAY_TIME        pdMS_TO_TICKS((uint32_t)(4000))
#define ISO15118_WAIT_TIME          pdMS_TO_TICKS((uint32_t)(40000))
#define ISO15118_COUNT_TIME         pdMS_TO_TICKS((uint32_t)(3000))
#define ISO15118_F_TIME             pdMS_TO_TICKS((uint32_t)(4000))
#define ISO15118_B_TIME             pdMS_TO_TICKS((uint32_t)(3000))

#define LED_RED_DEFAULT             ledCurrentSet[LED_C_RED]    /* (uint8_t)(15) */
#define LED_GREEN_DEFAULT           ledCurrentSet[LED_B_GREEN]  /* (uint8_t)(15) */
#define LED_BLUE_DEFAULT            ledCurrentSet[LED_A_BLU]    /* (uint8_t)(15) */

#define EVS_ISO15118_TEST_MAX       1
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local typedef -------------------------------------------------------------------------------------------------------------------------- //
typedef enum // possible value for evs_lid_error
{
NO_PLUG_CONSISTENCY = 0,
PLUG_IN_CONSISTENCY
} consistency_en;

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local constats ------------------------------------------------------------------------------------------------------------------------- //
static const char          tPrintf_evs_null[]               = "EVSTATE_NULL\n\r";
static const char          tPrintf_evs_idle[]               = "EVSTATE_IDLE\n\r";
static const char          tPrintf_evs_init[]               = "EVSTATE_INIT\n\r";
static const char          tPrintf_evs_cpset[]              = "EVSTATE_CPSET\n\r";
static const char          tPrintf_evs_mode_sel[]           = "EVSTATE_MODE_SEL\n\r";
static const char          tPrintf_evs_disabled[]           = "EVSTATE_DISABLED\n\r";
static const char          tPrintf_evs_auth_wait[]          = "EVSTATE_AUTH_WAIT\n\r";
static const char          tPrintf_evs_socket_available[]   = "EVSTATE_SOCKET_AVAILABLE\n\r";
static const char          tPrintf_evs_auth_pending[]       = "EVSTATE_AUTH_PENDING\n\r";
static const char          tPrintf_evs_plug_wait[]          = "EVSTATE_PLUG_WAIT\n\r";
static const char          tPrintf_evs_plug_check[]         = "EVSTATE_PLUG_CHECK\n\r";
static const char          tPrintf_evs_socket_check[]       = "EVSTATE_SOCKET_CHECK\n\r";
static const char          tPrintf_evs_block_up_delay[]     = "EVSTATE_BLOCK_UP_DELAY\n\r";
static const char          tPrintf_evs_block_down_delay[]   = "EVSTATE_BLOCK_DOWN_DELAY\n\r";
static const char          tPrintf_evs_m3s_m3t_detect[]     = "EVSTATE_M3S_M3T_DETECT\n\r";
static const char          tPrintf_evs_rectifier_check[]    = "EVSTATE_RECTIFIER_CHECK\n\r";
static const char          tPrintf_evs_suspending[]         = "EVSTATE_SUSPENDING\n\r";
static const char          tPrintf_evs_wakeup[]             = "EVSTATE_WAKEUP\n\r";
static const char          tPrintf_evs_s2_waiting[]         = "EVSTATE_S2_WAITING\n\r";
static const char          tPrintf_contact_close_delay[]    = "EVSTATE_CONTACT_CLOSE_DELAY\n\r";
static const char          tPrintf_evs_charging[]           = "EVSTATE_CHARGING\n\r";
static const char          tPrintf_evs_interrupting[]       = "EVSTATE_INTERRUPTING\n\r";
static const char          tPrintf_evs_res_error[]          = "EVSTATE_RES_ERROR\n\r";
static const char          tPrintf_evs_plug_out[]           = "EVSTATE_PLUG_OUT\n\r";
static const char          tPrintf_evs_lid_error[]          = "EVSTATE_LID_ERROR\n\r";
static const char          tPrintf_evs_close_lid[]          = "EVSTATE_CLOSE_LID\n\r";
static const char          tPrintf_evs_auth_neg[]           = "EVSTATE_AUTH_NEG\n\r";
static const char          tPrintf_evs_error_wait[]         = "EVSTATE_ERROR_WAIT\n\r";

static const char          *tPrintf_evs_state_array[] = {tPrintf_evs_null, tPrintf_evs_idle, tPrintf_evs_init, tPrintf_evs_cpset, tPrintf_evs_mode_sel,
                                                         tPrintf_evs_disabled, tPrintf_evs_auth_wait, tPrintf_evs_socket_available, tPrintf_evs_auth_pending,
                                                         tPrintf_evs_plug_wait, tPrintf_evs_plug_check, tPrintf_evs_socket_check, tPrintf_evs_block_up_delay,
                                                         tPrintf_evs_block_down_delay, tPrintf_evs_m3s_m3t_detect, tPrintf_evs_rectifier_check, tPrintf_evs_suspending,
                                                         tPrintf_evs_wakeup, tPrintf_evs_s2_waiting, tPrintf_contact_close_delay, tPrintf_evs_charging,
                                                         tPrintf_evs_interrupting, tPrintf_evs_res_error, tPrintf_evs_plug_out, tPrintf_evs_lid_error,
                                                         tPrintf_evs_close_lid, tPrintf_evs_auth_neg, tPrintf_evs_error_wait};

static const char          tPrintf_error_rcdm[]             = "rcdm_anom0\n\r";
static const char          tPrintf_error_lid[]              = "lid_anom0\n\r";
static const char          tPrintf_error_vent[]             = "vent_anom0\n\r";
static const char          tPrintf_error_block[]            = "block_anom0\n\r";
static const char          tPrintf_error_remote[]           = "remote_stat0\n\r";
static const char          tPrintf_error_puls[]             = "puls_stat0\n\r";
static const char          tPrintf_error_mirror[]           = "mirror_anom0\n\r";
static const char          tPrintf_error_rcbo[]             = "rcbo_anom0\n\r";
static const char          tPrintf_error_cpshort[]          = "cpshort_anom1\n\r";
static const char          tPrintf_error_ppshort[]          = "ppshort_anom1\n\r";
static const char          tPrintf_error_cplost[]           = "cplost_anom1\n\r";
static const char          tPrintf_error_pplost[]           = "pplost_anom1\n\r";
static const char          tPrintf_error_vbus[]             = "vbus_anom1\n\r";
static const char          tPrintf_error_mifare[]           = "mifare_anom1\n\r";
static const char          tPrintf_error_emeter_int[]       = "emeter_int_anom1\n\r";
static const char          tPrintf_error_overcurrent[]      = "overcurrent_anom1\n\r";
static const char          tPrintf_error_rectifier[]        = "rectifier_anom2\n\r";
static const char          tPrintf_error_pmng_emeter[]      = "pmng_emeter_anom2\n\r";
static const char          tPrintf_error_nu22[]             = "nu22_anom2\n\r";
static const char          tPrintf_error_nu23[]             = "nu23_anom2\n\r";
static const char          tPrintf_error_nu24[]             = "nu24_anom2\n\r";
static const char          tPrintf_error_nu25[]             = "nu25_anom2\n\r";
static const char          tPrintf_error_nu26[]             = "nu26_anom2\n\r";
static const char          tPrintf_error_nu27[]             = "nu27_anom2\n\r";

static const char* const   tPrintf_error_array[] = {tPrintf_error_rcdm, tPrintf_error_lid, tPrintf_error_vent, tPrintf_error_block, tPrintf_error_remote,
                                                     tPrintf_error_puls, tPrintf_error_mirror, tPrintf_error_rcbo, tPrintf_error_cpshort, tPrintf_error_ppshort,
                                                     tPrintf_error_cplost, tPrintf_error_pplost, tPrintf_error_vbus, tPrintf_error_mifare, tPrintf_error_emeter_int,
                                                     tPrintf_error_overcurrent, tPrintf_error_rectifier, tPrintf_error_pmng_emeter, tPrintf_error_nu22,
                                                     tPrintf_error_nu23, tPrintf_error_nu24, tPrintf_error_nu25, tPrintf_error_nu26, tPrintf_error_nu27};

static uint8_t evState_str[38][27] = 
{ 
  
    "EVSTATE_NULL",               
    "EVSTATE_IDLE",                   
    "EVSTATE_INIT",                   
    "EVSTATE_CPSET",                  
    "EVSTATE_DELAY",                  
    "EVSTATE_MODE_SEL",               
    "EVSTATE_DISABLED",               
    "EVSTATE_AUTH_WAIT",              
    "EVSTATE_SOCKET_AVAILABLE",       
    "EVSTATE_AUTH_PENDING",           
    "EVSTATE_AUTH_MISSED",            
    "EVSTATE_PLUG_WAIT",              
    "EVSTATE_LID_WAIT",               
    "EVSTATE_PLUG_CHECK",             
    "EVSTATE_SOCKET_CHECK",           
    "EVSTATE_BLOCK_UP_DELAY",           
    "EVSTATE_BLOCK_DOWN_DELAY",         
    "EVSTATE_M3S_M3T_DETECT",           
    "EVSTATE_RECTIFIER_CHECK",          
    "EVSTATE_SUSPENDING",               
    "EVSTATE_WAKEUP",                   
    "EVSTATE_S2_WAITING",               
    "EVSTATE_CONTACT_CLOSE_DELAY",      
    "EVSTATE_CHARGING",                 
    "EVSTATE_INTERRUPTING",             
    "EVSTATE_F_STATE_ERROR",            
    "EVSTATE_X1_STATE_ERROR",           
    "EVSTATE_RES_ERROR",                
    "EVSTATE_PLUG_OUT",                 
    "EVSTATE_LID_ERROR",                
    "EVSTATE_PAUT_LID",                 
    "EVSTATE_CLOSE_LID",                
    "EVSTATE_AUTH_NEG",                 
    "EVSTATE_ERROR_WAIT",               
    "EVSTATE_PEN_STATE",                
    "EVSTATE_V230_SUSPEND",             
    "EVSTATE_POWER_OFF",                
    "EVSTATE_LID_ERROR_DELAY"  
  
};

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local references ----------------------------------------------------------------------------------------------------------------------- //
xQueueHandle EvsMngQueue =  NULL;

static EvsMngMsg_st         EvsMngMsg;

static TimerHandle_t        xEvsMngTimers[EVS_NUM_TIMER];

static evs_state_en         evs_state;
static evs_state_en         evs_state_old;
static evs_state_en         evs_state_next;
static evs_state_en         evs_state_resume;
       evs_modbus_state_en  evsModbusState;
static evs_modbus_state_en  evsModbusState_old;

static uint8_t              evs_mode;
static uint8_t              evs_mode_update;

static charging_mode_en     charging_mode;
static charging_mode_en     pen_charging_mode;

static uint8_t              ev_suspending;
static uint8_t              suspending_save;
static uint8_t              suspending_enable;
static suspending_en        suspending_request;
static suspending_en        suspending_set;

static uint8_t              pwm_low_output;

static uint8_t              evs_sec;

static uint8_t              evs_error_array[EVS_ERROR_ARRAY_SIZE];
static uint8_t              evs_error_save[EVS_ERROR_ARRAY_SIZE];

static uint8_t              tPrintf_error;
static uint8_t              tPrintf_error_old;

static uint8_t              control_byte_old[CONTROL_BYTE_NUM];

static uint8_t              plug_wait_lcd;

static uint8_t              snp3_error_save;

static uint8_t              pen_state_set;

static uint8_t              evstate_plug_out_post;

static uint8_t              suspending_enable_set;
static uint8_t              post_suspending_num;
static uint8_t              post_suspending;
static uint32_t             post_suspending_time;

static sinapsiSetReg_st     *evs_sinapsi_ptr;

static LcdMngEvent_en       evs_LcdMngEvent;

static uint8_t              suspending_set_pen_save;

static uint8_t              getInputState_enable;
static uint8_t              emeter_int_anom1_save;

static uint8_t              vbus_anom1_save;
static uint8_t              wifi_antenna_error;

static uint8_t              evs_charging_resume;
static uint8_t              evs_charging_resume_sem;

static uint8_t              evs_gost_plug;
static uint8_t              evs_gost_lid;

static uint8_t              evs_timestamp_state;
static uint8_t              evs_timestamp_timeout;
static uint32_t             evs_timestamp_value;

static uint8_t              sinapsi_inst_status;
static uint8_t              chn2_error_enable;
static uint8_t              internal_chn2_error_enable;

static uint8_t              initEmParameter_enb;

static uint8_t              mdb_suspending_ev_hide;
static uint8_t              mdb_suspending_ev_hide_enable;
static uint8_t              mdb_ev_connectedd_hide;

static uint8_t              sem_charging_time_reset;

static uint8_t              evsModbusState_force;

static uint8_t              evs_reserved;

static uint8_t              lcd_msg_update_num;

static uint8_t              sem_time_valid;
static uint32_t             sem_wait_time;
static uint32_t             evs_wait_time;

uint8_t                     startPersonal = PERSONAL_BY_CARD;
uint8_t                     rfidTaskStatus = RFID_TASK_ACTIVE;

static uint8_t              evs_iso15118_run;
static uint8_t              evs_iso15118_num_test;
static uint8_t              evs_iso15118_data_req;
static uint8_t              evs_iso15118_info;
static uint8_t              evs_iso15118_count;
static uint8_t              evs_iso15118_tim_enable;
static iso15118_state_en    evs_iso15118_state;
static uint8_t              evs_iso15118_s2;
static uint8_t              iso15118_wakeup_step;



#ifdef HW_MP28947
ISO15118_Info_Device_st  ISO15118_Info_Device;
#endif


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local function prototypes -------------------------------------------------------------------------------------------------------------- //
static xQueueHandle getEvsMngQueueHandle(void);
static void EvsMngTimCallBack(TimerHandle_t pxTimer);
static void evs_set_timer(EvsMngTim_en timer, uint32_t set_time);
static void evs_tPrintf_state(EvsMngMsg_st *pMsg);
static void evs_tPrintf_error(uint8_t *src_ptr);
static void evs_rtc_backup_set(uint32_t reg, uint32_t val);
uint32_t evs_rtc_backup_get(uint32_t reg);
static uint8_t evs_error_wait_capture(EvsMngMsg_st *pMsg, evs_state_en state);
static uint8_t evs_res_error_capture(EvsMngMsg_st *pMsg, evs_state_en state);
static uint8_t evs_lid_error(consistency_en consistency, uint8_t gsy_update);
static void evs_error_update(void);
static void evs_state_set(evs_state_en *state, evs_state_en set_state);
static void evs_event_save(EvsMngMsg_st *pMsg);
static void evs_timestamp_manager(void);
static void EvsManager_init(void);
static void EvsManager(EvsMngMsg_st *pMsg);
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
extern uint8_t        ledCurrentSet[NUM_LED];
//extern osThreadId_t   RfidTaskHandle;
extern infoStation_t  infoStation;

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  getEvsMngQueueHandle
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static xQueueHandle getEvsMngQueueHandle(void)
{
return(EvsMngQueue);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  EvsMngTimCallBack
//
//  DESCRIPTION:    callback to manager timers
//
//  INPUT:          TimerHandle_t: the elapsed timer
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void EvsMngTimCallBack(TimerHandle_t pxTimer)
{
uint32_t    timer_id;

timer_id = (uint32_t)pvTimerGetTimerID(pxTimer);                                    // find the led  which the timer is referred

switch (timer_id)
    {
    case EVS_ISO15118_TIM:
        {
        if (evs_iso15118_tim_enable == 1)
            send_to_evs(EVS_ISO15118_TIM_EXPIRED);
        }
        break;

    case LID_ERROR_DELAY_TIM:
        {
        send_to_evs(LID_ERROR_DELAY_TIM_EXPIRED);
        }
        break;

    case EVS_TIMESTAMP_TIM:
        {
        evs_timestamp_timeout = 1;
        send_to_evs(EVS_TIMESTAMP_TIM_EXPIRED);
        }
        break;

    case EMETER_INT_ANOM1_TIM:
        {
        send_to_evs(EVS_STATE_TIM_EXPIRED);

        if (emeter_int_anom1_save == 1)
            emeter_int_anom1_save = 2;
        }
        break;

    case EVS_SUSPENDING_TIM:
        {
        send_to_evs(EVS_SUSPENDING_TIM_EXPIRED);
        suspending_enable_set = 0;
        }
        break;

    case CPSET_STATE_TIM:
        {
        if (evs_state == EVSTATE_CPSET)
            evs_set_timer(CPSET_STATE_TIM, CPSET_STATE_TIME);                       // reload timer
    
        send_to_evs(CPSET_TIM_EXPIRED);
        }
        break;

    case EVS_WAIT_TIM:
        {
        if (evs_sec > 0)
            {
            evs_sec --;
            evs_set_timer(EVS_WAIT_TIM, evs_wait_time);                             // reload timer
            send_to_lcd(EVS_TIME_EXPIRED);
            }
        else
            {
            send_to_evs(EVS_SEC_TIM_EXPIRED);

            if (rfidTaskStatus == RFID_TASK_SUSPENDED)
                {
                vTaskResume(RfidTaskHandle);
                rfidTaskStatus = RFID_TASK_ACTIVE;
                }
            }
        }
        break;

    case EVS_SEC_TIM:
        {
        if (evs_sec > 0)
            {
            evs_sec --;
            evs_set_timer(EVS_SEC_TIM, EVS_SEC_TIME);                             // reload timer
            send_to_lcd(EVS_TIME_EXPIRED);
            }
        else
            {
            send_to_evs(EVS_SEC_TIM_EXPIRED);

            if (rfidTaskStatus == RFID_TASK_SUSPENDED)
                {
                vTaskResume(RfidTaskHandle);
                rfidTaskStatus = RFID_TASK_ACTIVE;
                }
            }
        }
        break;

    case ERROR_WAIT_TIM:
        {
        if ((evs_state == EVSTATE_ERROR_WAIT) || (evs_state == EVSTATE_AUTH_NEG))
            {
            evs_set_timer(ERROR_WAIT_TIM, EVS_SEC_TIME);                            // reload timer
            send_to_evs(ERROR_WAIT_TIM_EXPIRED);
            }
        }
        break;

    case CPPP_WAIT_TIM:
        {
        if (evs_state == EVSTATE_SOCKET_CHECK)
            {
            evs_set_timer(CPPP_WAIT_TIM, CPPP_WAIT_TIME);                           // reload timer
            send_to_evs(CPPP_WAIT_TIM_EXPIRED);
            }
        }
        break;

    case EVS_TPRINT_TIM:
        {
        evs_set_timer(EVS_TPRINT_TIM, EVS_TPRINT_TIME);                             // reload timer
        send_to_evs(EVS_EVENT_PRINT);
        }
        break;

    case EVS_STATE_TIM:
        {
        send_to_evs(EVS_STATE_TIM_EXPIRED);
        }
        break;

    case EVS_POST_SUSPENDING_TIM:
        {
        send_to_evs(EVS_POST_SUSPENDING_EXPIRED);
        }
        break;

    case EVS_UID_ERROR_TIM:
        {
        send_to_lcd(evs_LcdMngEvent);
        }
        break;

    case SBC_GSY_TIM:
        {
        gsy_enable_set();
        }
        break;

    default:
        break;
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_set_timer
//
//  DESCRIPTION:    imposta il timer selezionato
//
//  INPUT:          timer da aggiornare: timer; tempo da impostare: set_time
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void evs_set_timer(EvsMngTim_en timer, uint32_t set_time)
{
  while ((xTimerChangePeriod (xEvsMngTimers[timer], set_time, EVS_GARD_TIME) != pdPASS));    // set timer
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  send_to_evs
//
//  DESCRIPTION:    impacchetta l'evento da inviare a EvsMngTask
//
//  INPUT:          valore di EvsMngEvent
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void send_to_evs(uint8_t evs_event)
{
  EvsMngMsg_st    msgEvsSend;

  msgEvsSend.EvsMngEvent = (EvsMngEvent_en)(evs_event);
  configASSERT(xQueueSendToBack(getEvsMngQueueHandle(), (void *)&msgEvsSend, portMAX_DELAY) == pdPASS);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_charging_mode_get
//
//  DESCRIPTION:    ritorna la modalità di ricarica standard / semlificata
//
//  INPUT:          none
//
//  OUTPUT:         charging_mode
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
charging_mode_en evs_charging_mode_get(void)
{
return charging_mode;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  -
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t vbus_anom1_save_get(void)
{
return vbus_anom1_save;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_reserved_set
//
//  DESCRIPTION:    -
//
//  INPUT:          next evs_reserved
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void evs_reserved_set(uint8_t val)
{
evs_reserved = val;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_reserved_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         evs_reserved
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t evs_reserved_get(void)
{
return evs_reserved;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_error_set
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void evs_error_set(uint8_t error_byte, uint8_t error_bit, uint8_t error_val)
{
uint8_t control_enable;

eeprom_param_get((CONTROL_BYTE0_EADD + error_byte), &control_enable, 1);

if (error_val == 1)
    {
    evs_error_array[error_byte] |= (control_enable & error_bit);
    evs_error_save[error_byte] |= error_bit;
    }
else 
    {
    evs_error_array[error_byte] &=~ error_bit;
    evs_error_save[error_byte] &=~ error_bit;
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_error_get
//
//  DESCRIPTION:    ritorna l'array degli errori presenti
//
//  INPUT:          puntatore destinatario: dst_ptr; nasconde errore mifare: mifare_error_hide 
//
//  OUTPUT:         1 = ci sono errori presenti; 0 = nessun errore
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t evs_error_get(uint8_t *dst_ptr, uint8_t mifare_error_hide, uint8_t lid_error_hide, uint8_t emex_error_hide)
{
uint8_t i, ret = 0;

*(dst_ptr + 0) = evs_error_array[0];
*(dst_ptr + 1) = evs_error_array[1];
*(dst_ptr + 2) = evs_error_array[2];

if (lid_error_hide == 1)
    *dst_ptr &=~ LID_ANOM0;

if ((mifare_error_hide == 1) && (evs_mode == EVS_FREE_MODE))
    *(dst_ptr + 1) &=~ MIFARE_ANOM1;

if (emex_error_hide & 0x02)
    *(dst_ptr + 2) &=~ EMETER_EXT_ANOM2;

if (emex_error_hide & 0x04)
    *(dst_ptr + 2) &=~ SINAPSI_CHN2_ANOM2;

for (i=0; i<EVS_ERROR_ARRAY_SIZE; i++)
    {
    if (*(dst_ptr + i) != 0)
        ret = 1;
    }

return ret;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //


/**
*
* @brief        funzione che esegue la decodifica degli errori GSY e la codifica secondo 
*               Modbus v20 con scrittura nei due registri EVSE_ERROR1_RO e EVSE_ERROR2_RO
*
* @param [in]   uint8_t*: puntatore all'array di errori ricevuto
* @param [in]   uint8_t: flag to force update 
*
* @retval       none
*
***********************************************************************************************************************/
void updateModbusErrorRegisters (uint8_t* errorArray, uint8_t forceUpd)
{
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;
  uint16_t              error1 = 0;
  uint16_t              TmpError1;
  uint16_t              error2 = 0;
  uint16_t              TmpError2;
  
  mdbAddr = getLogicalMdbAddrSem();
  pRoRegs = getRoMdbRegs(mdbAddr);
  
  /* RCDM error for EVSE --> bit n.0 for ERROR1 modbus v20 */
  if((errorArray[0] & 0x01) == 0x01)
  {
    error1 |= ERROR1_RCDM;
  }
  
  /* LIDE error for EVSE --> bit n.1 ERROR1 for modbus v20 */
  if((errorArray[0] & 0x02) == 0x02)
  {
    error1 |= ERROR1_LIDE;
  }
  
  /* VENT error for EVSE --> bit n.2 ERROR1 for modbus v20 */
  if((errorArray[0] & 0x04) == 0x04)
  {
    error1 |= ERROR1_VENT;
  }

  /* BLOCK error for EVSE --> bit n.3 for ERROR1 modbus v20 */
  if((errorArray[0] & 0x08) == 0x08)
  {
    error1 |= ERROR1_BLCK;
  }  
  
  /* Bit n.4 = BLE_GUASTO and bit n.5 = WIFI_GUASTO */

  /* MIRR error for EVSE --> bit n.6 for ERROR1 modbus v20 */
  if((errorArray[0] & 0x40) == 0x40)
  {
    error1 |= ERROR1_MIRR;
  }
  
  /* RCBO error for EVSE --> bit n.7 for ERROR1 modbus v20 */
  if((errorArray[0] & 0x80) == 0x80)
  {
    error1 |= ERROR1_RCBO;
  }  

  /* CPSE error for EVSE --> bit n.8 for ERROR1 modbus v20 */
  if((errorArray[1] & 0x01) == 0x01)
  {
    error1 |= ERROR1_CPSE;
  }    
  
  /* PPSE error for EVSE --> bit n.9 for ERROR1 modbus v20 */
  if((errorArray[1] & 0x02) == 0x02)
  {
    error1 |= ERROR1_PPSE;
  }
  
  /* CPLS error for EVSE --> bit n.10 for ERROR1 modbus v20 */
  if((errorArray[1] & 0x04) == 0x04)
  {
    error1 |= ERROR1_CPLS;
  }    
  
  /* PPLS error for EVSE --> bit n.11 for ERROR1 modbus v20 */
  if((errorArray[1] & 0x08) == 0x08)
  {
    error1 |= ERROR1_PPLS;
  }  
  
  /* VBUS error for EVSE --> bit n.12 for ERROR1 modbus v20 */
  if((errorArray[1] & 0x10) == 0x10)
  {
    error1 |= ERROR1_VBUS;
  }    
  
  /* MFRE error for EVSE --> bit n.13 for ERROR1 modbus v20 */
  if((errorArray[1] & 0x20) == 0x20)
  {
    error1 |= ERROR1_MFRE;
  }  
  
  /* EMTR error for EVSE --> bit n.14 for ERROR1 modbus v20 */
  if((errorArray[1] & 0x40) == 0x40)
  {
    error1 |= ERROR1_EMTR;
  }  
  
  /* EMTR error for EVSE --> bit n.14 for ERROR1 modbus v20 */
  if((errorArray[1] & 0x80) == 0x80)
  {
    error1 |= ERROR1_OVCE;
  }  
  
  /* RCTE error for EVSE --> bit n.0 for ERROR2 modbus v20 */
  if((errorArray[2] & 0x01) == 0x01)
  {
    error2 |= ERROR2_RCTE;
  }
  
  /* RCTE error for EVSE --> bit n.1 for ERROR2 modbus v20 */
  if((errorArray[2] & 0x02) == 0x02)
  {
    error2 |= ERROR2_EMEX;
  }
  
  /* CHAIN2 error for EVSE --> bit n.1 for ERROR2 modbus v20 */
  if((errorArray[2] & 0x04) == 0x04)
  {
    error2 |= ERROR2_CHN2;
  }

  if (getScuTypeMode() != SCU_SEM_STAND_ALONE)
  {
    /* Create a temporary copy of the error in order to detect changes */
    /* for ERROR1 */
    TmpError1 = pRoRegs->scuMapRegNotify.ntfErr1;
    pRoRegs->scuMapRegNotify.ntfErr1 = error1;
    /* for ERROR1 */
    TmpError2 = pRoRegs->scuMapRegNotify.ntfErr2;
    pRoRegs->scuMapRegNotify.ntfErr2 = error2;  
      
    /* In SEM mode, notify the errors */
    if (isSemMode() == TRUE)
    {    
      if ((error1 != TmpError1) || (forceUpd == TRUE))
        upgradeModbusReg(ADDR_EVSE_ERROR1_RO);   
      
      if ((error2 != TmpError2) || (forceUpd == TRUE))
        upgradeModbusReg(ADDR_EVSE_ERROR2_RO);      
    }
  }
  
  
}


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_suspending_set_get
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t evs_suspending_set_get(void)
{
return (uint8_t)(suspending_set);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_sec_get
//
//  DESCRIPTION:    ritorna il valore dei secondi corrente per il timeout di stato di evs_state
//
//  INPUT:          puntatore destinatario: dst_ptr
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void evs_sec_get(uint8_t *dst_ptr)
{
*dst_ptr = evs_sec;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_tPrintf_state
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void evs_tPrintf_state(EvsMngMsg_st *pMsg)
{
if (pMsg->EvsMngEvent == EVS_EVENT_PRINT)
    {
    if (evs_state != evs_state_old)
        {
        tPrintf(" - evs_state = ");
        tPrintf(tPrintf_evs_state_array[evs_state]);
        evs_state_old = evs_state;
        }
    else if (tPrintf_error != tPrintf_error_old)
        {
        tPrintf(" - evs_error = ");
        tPrintf(tPrintf_error_array[tPrintf_error]);
        tPrintf_error_old = tPrintf_error;
        }

    evs_set_timer(EVS_TPRINT_TIM, EVS_TPRINT_TIME);
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  internal_chn2_error_enable_get
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         internal_chn2_error_enable
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t internal_chn2_error_enable_get(void)
{
return internal_chn2_error_enable;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  internal_chn2_error_enable_reset
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void internal_chn2_error_enable_reset(void)
{
internal_chn2_error_enable = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  chn2_error_enable_get
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         chn2_error_enable
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t chn2_error_enable_get(void)
{
return chn2_error_enable;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  chn2_error_enable_set
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void chn2_error_enable_set(void)
{
chn2_error_enable = 1;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_tPrintf_error
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void evs_tPrintf_error(uint8_t *src_ptr)
{
uint8_t i, j, error, error_array[EVS_ERROR_ARRAY_SIZE];

memcpy(error_array, src_ptr, (size_t)EVS_ERROR_ARRAY_SIZE);

for (i=0; i<EVS_ERROR_ARRAY_SIZE; i++)
    {
    for (j=0; j<8; j++)
        {
        error = j + (i * 8);

        if ((error_array[i] & 0x01) && (error != tPrintf_error_old))
            {
            tPrintf_error = error;
            i = EVS_ERROR_ARRAY_SIZE;
            break;
            }

        error_array[i] >>= 1;
        }
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_iso15118_run_get
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t evs_iso15118_run_get(void)
{
return evs_iso15118_run;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_rtc_backup_set
//
//  DESCRIPTION:    salva parametri della ricarica per la gestione degli stati di low power mode
//
//  INPUT:          status
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void evs_rtc_backup_set(uint32_t reg, uint32_t val)
{
HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), reg, val);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_rtc_backup_get
//
//  DESCRIPTION:    legge parametri della ricarica per la gestione degli stati di low power mode
//
//  INPUT:          status
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint32_t evs_rtc_backup_get(uint32_t reg)
{
return HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)getHandleRtc(), reg);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_error_wait_capture
//
//  DESCRIPTION:    controlla lo stato degli errori che forzano la macchina verso lo stato EVSTATE_ERROR_WAIT
//
//  INPUT:          puntatore EvsMngEvent: pMsg; stato della stazione: state
//
//  OUTPUT:         1 = errore presente
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t evs_error_wait_capture(EvsMngMsg_st *pMsg, evs_state_en state)
{
uint8_t ret, pmng_enable, control_enable, emeter_type;

ret = 0;

eeprom_param_get(EMETER_INT_EADD, &emeter_type, 1);
eeprom_param_get(CONTROL_BYTE1_EADD, &control_enable, 1);

if (emeter_int_anom1_save == 2)
    {
    evs_error_set(CONTROL_BYTE_1, EMETER_INT_ANOM1, 1);
    send_to_energy(ENERGY_INTERNAL_EM_FAIL);
    send_to_pwm(PWM_INTERNAL_EM_FAIL);
    emeter_int_anom1_save = 0;
    ret = 1;
    }

if ((pMsg->EvsMngEvent == EVS_EXTERNAL_EM_GOOD) || (pMsg->EvsMngEvent == EVS_SINAPSI_CHN2_GOOD))
    {
    if (pMsg->EvsMngEvent == EVS_EXTERNAL_EM_GOOD)
        {
        evs_error_set(CONTROL_BYTE_2, EMETER_EXT_ANOM2, 0);
        suspending_request &=~ EMEX_SUSPENDING;
        }
    else // if (pMsg->EvsMngEvent == EVS_SINAPSI_CHN2_GOOD)
        {
        if (sinapsi_inst_status != 1)
            {
            sinapsi_inst_status = 1;
            SCU_InfoStation_Set ((uint8_t *)&infoStation.Sinapsi_Installed, &sinapsi_inst_status, 1);  /* ex SINAPSI_INST_EADD */
            }

        chn2_error_enable = 1;
        internal_chn2_error_enable = 0;
        evs_error_set(CONTROL_BYTE_2, SINAPSI_CHN2_ANOM2, 0);
        suspending_request &=~ CHN2_SUSPENDING;
        }

    lcd_external_em_set(1);
    send_to_pwm(PWM_EXTERNAL_EM_GOOD);
    }
else if (pMsg->EvsMngEvent == EVS_EXTERNAL_EM_UPDATE)
    {
    eeprom_param_get(CONTROL_BYTE2_EADD, &control_enable, 1);
    lcd_external_em_set(1);

    eeprom_param_get(HIDDEN_MENU_ENB_EADD, &pmng_enable, 1);            // legge power management enable
    pmng_enable &= HIDDEN_MENU_PMNG_ENB;
            
    if (((control_enable & EMETER_EXT_CRL2) == 0) || (pmng_enable == 0))
        {
        evs_error_set(CONTROL_BYTE_2, (EMETER_EXT_ANOM2 | SINAPSI_CHN2_ANOM2), 0);
        suspending_request &=~ EMEX_SUSPENDING;
        }
    }
else if ((state == EVSTATE_AUTH_WAIT) || (state == EVSTATE_SOCKET_AVAILABLE) || (state == EVSTATE_ERROR_WAIT) || (state <= EVSTATE_CPSET)
 || (pMsg->EvsMngEvent == EVS_INTERNAL_EM_UPDATE) || (pMsg->EvsMngEvent == EVS_INTERNAL_EM_GOOD))
    {
    if ((emeter_type == EMETER_TAMP) || (emeter_type == EMETER_TAMP_3) || (emeter_type == EMETER_TYPE_NULL)) /*RIVEDERE */
        {
        if (evs_error_array[1] & EMETER_INT_ANOM1)
            {
            evs_error_set(CONTROL_BYTE_1, EMETER_INT_ANOM1, 0);
            send_to_energy(ENERGY_INTERNAL_EM_GOOD);
            send_to_pwm(PWM_INTERNAL_EM_GOOD);
            }
        }

    if ((state == EVSTATE_AUTH_WAIT) && (pMsg->EvsMngEvent == EVS_RFID_ANOM1_UPDATE))
        {
        rfid_anom1_update();
            
        if (evs_error_array[1] & MIFARE_ANOM1)
            {
            evs_tPrintf_error(evs_error_array);
            ret = 1;
            }
        }
    else if ((emeter_type != EMETER_TAMP) && (emeter_type != EMETER_TAMP_3) && (emeter_type != EMETER_TYPE_NULL))
        {
        if ((pMsg->EvsMngEvent == EVS_INTERNAL_EM_GOOD) || (pMsg->EvsMngEvent == EVS_INTERNAL_EM_UPDATE))
            {
            if (((control_enable & EMETER_INT_CRL1) == 0) || (get_emeter_type(INTERNAL_EM) == emeter_type))
                {
                evs_error_set(CONTROL_BYTE_1, EMETER_INT_ANOM1, 0);
                send_to_energy(ENERGY_INTERNAL_EM_GOOD);
                send_to_pwm(PWM_INTERNAL_EM_GOOD);
                }
            else if (get_emeter_type(INTERNAL_EM) != emeter_type)
                {
                if ((control_enable & EMETER_INT_CRL1) == EMETER_INT_CRL1)
                    {
                    if  ((evs_state > EVSTATE_MODE_SEL) && (emeter_int_anom1_save == 0))
                        {
                        evs_error_set(CONTROL_BYTE_1, EMETER_INT_ANOM1, 1);
                        send_to_energy(ENERGY_INTERNAL_EM_FAIL);
                        send_to_pwm(PWM_INTERNAL_EM_FAIL);
                        evs_tPrintf_error(evs_error_array);
                        ret = 1;
                        }
                    else
                        {
                        emeter_int_anom1_save = 1;
                        evs_tPrintf_error(evs_error_array);
                        }
                    }
                }
            }
        else if (pMsg->EvsMngEvent == EVS_EXTERNAL_EM_UPDATE)
            {
            eeprom_param_get(CONTROL_BYTE2_EADD, &control_enable, 1);
            lcd_external_em_set(1);

            eeprom_param_get(HIDDEN_MENU_ENB_EADD, &pmng_enable, 1);            // legge power management enable
            pmng_enable &= HIDDEN_MENU_PMNG_ENB;
                    
            if (((control_enable & EMETER_EXT_CRL2) == 0) || (pmng_enable == 0))
                {
                evs_error_set(CONTROL_BYTE_2, (EMETER_EXT_ANOM2 | SINAPSI_CHN2_ANOM2), 0);
                suspending_request &=~ EMEX_SUSPENDING;
//                send_to_pwm(PWM_EXTERNAL_EM_GOOD);
                }
            }
        else if ((pMsg->EvsMngEvent == EVS_INTERNAL_EM_FAIL) && (snp3_error_save == 0))
            {
//            evs_error_set(CONTROL_BYTE_1, EMETER_INT_ANOM1, 1);
    
            if ((control_enable & EMETER_INT_CRL1) == EMETER_INT_CRL1)
                {
                if ((evs_state > EVSTATE_MODE_SEL) && (emeter_int_anom1_save == 0))
                    {
                    evs_error_set(CONTROL_BYTE_1, EMETER_INT_ANOM1, 1);
                    send_to_energy(ENERGY_INTERNAL_EM_FAIL);
                    send_to_pwm(PWM_INTERNAL_EM_FAIL);
                    evs_tPrintf_error(evs_error_array);
                    ret = 1;
                    }
                else
                    emeter_int_anom1_save = 1;
                }
            }
        else if (((pMsg->EvsMngEvent == EVS_EXTERNAL_EM_FAIL) || (pMsg->EvsMngEvent == EVS_SINAPSI_CHN2_FAIL)) && (snp3_error_save == 0))
            {
            if (hidden_menu_sel_get() == HIDDEN_IDLE)
                {
                eeprom_param_get(HIDDEN_MENU_ENB_EADD, &pmng_enable, 1);            // legge power management enable
                pmng_enable &= HIDDEN_MENU_PMNG_ENB;
                    
                if (pmng_enable == HIDDEN_MENU_PMNG_ENB)
                     {
                    if (pMsg->EvsMngEvent == EVS_EXTERNAL_EM_FAIL)
                        evs_error_set(CONTROL_BYTE_2, EMETER_EXT_ANOM2, 1);
                    else    // if (pMsg->EvsMngEvent == EVS_SINAPSI_CHN2_FAIL)
                        evs_error_set(CONTROL_BYTE_2, SINAPSI_CHN2_ANOM2, 1);

                     eeprom_param_get(CONTROL_BYTE2_EADD, &control_enable, 1);
                     lcd_external_em_set(0);

                     if (((control_enable & EMETER_EXT_CRL2) == EMETER_EXT_CRL2)
                       && ((pMsg->EvsMngEvent == EVS_EXTERNAL_EM_FAIL) || ((pMsg->EvsMngEvent == EVS_SINAPSI_CHN2_FAIL) && (chn2_error_enable == 1))))
                         {
                         send_to_pwm(PWM_EXTERNAL_EM_FAIL);
                         evs_tPrintf_error(evs_error_array);
                         ret = 1;
                         }
                    }
                }
            }
        }
    }
else if (((emeter_type != EMETER_TAMP) && (emeter_type != EMETER_TAMP_3)) && (pMsg->EvsMngEvent == EVS_INTERNAL_EM_FAIL) && (snp3_error_save == 0))
    {
    if ((control_enable & EMETER_INT_CRL1) == EMETER_INT_CRL1)
        {
        if  ((evs_state > EVSTATE_MODE_SEL) && (emeter_int_anom1_save == 0))
            {
            send_to_pwm(PWM_INTERNAL_EM_FAIL);
            evs_error_set(CONTROL_BYTE_1, EMETER_INT_ANOM1, 1);
            }
        else
            emeter_int_anom1_save = 1;
        }

    if (evs_error_array[1] & EMETER_INT_ANOM1)
        evs_tPrintf_error(evs_error_array);
    }
else if (((emeter_type != EMETER_TAMP) && (emeter_type != EMETER_TAMP_3))
       && ((pMsg->EvsMngEvent == EVS_EXTERNAL_EM_FAIL) || ((pMsg->EvsMngEvent == EVS_SINAPSI_CHN2_FAIL) && (chn2_error_enable == 1))) && (snp3_error_save == 0))
    {
    eeprom_param_get(HIDDEN_MENU_ENB_EADD, &pmng_enable, 1);            // legge power management enable
    pmng_enable &= HIDDEN_MENU_PMNG_ENB;
                    
    if (pmng_enable == HIDDEN_MENU_PMNG_ENB)
        {
        if (pMsg->EvsMngEvent == EVS_EXTERNAL_EM_FAIL)
            evs_error_set(CONTROL_BYTE_2, EMETER_EXT_ANOM2, 1);
        else // if (pMsg->EvsMngEvent == EVS_SINAPSI_CHN2_FAIL)
            evs_error_set(CONTROL_BYTE_2, SINAPSI_CHN2_ANOM2, 1);

        lcd_external_em_set(0);

        if (evs_error_array[2] & (EMETER_EXT_ANOM2 | SINAPSI_CHN2_ANOM2))
            {
            send_to_pwm(PWM_EXTERNAL_EM_FAIL);
            
            if (evs_error_array[2] & EMETER_EXT_ANOM2)
                suspending_request |= EMEX_SUSPENDING;
            else // if (evs_error_array[2] & SINAPSI_CHN2_ANOM2)
                suspending_request |= CHN2_SUSPENDING;

            evs_tPrintf_error(evs_error_array);
            }
        }
    }
else if ((pMsg->EvsMngEvent == EVS_INTERNAL_EM_GOOD) && (get_emeter_type(INTERNAL_EM) == emeter_type))
    {
    evs_error_set(CONTROL_BYTE_1, EMETER_INT_ANOM1, 0);
    
    if (energymng_running_get() == 0)
        send_to_energy(ENERGY_INTERNAL_EM_GOOD);

    send_to_pwm(PWM_INTERNAL_EM_GOOD);
    }
return ret;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_res_error_capture
//
//  DESCRIPTION:    controlla lo stato degli errori che forzano la macchina verso lo stato EVSTATE_RES_ERROR
//
//  INPUT:          puntatore EvsMngEvent: pMsg; stato della stazione: state
//
//  OUTPUT:         1 = errore presente
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t evs_res_error_capture(EvsMngMsg_st *pMsg, evs_state_en state)
{
uint8_t ret, pmng_enable, control_enable, actuator_enable, error_capture_array[EVS_ERROR_ARRAY_SIZE];

eeprom_param_get(ACTUATORS_EADD, &actuator_enable, 1);

eeprom_param_get(CONTROL_BYTE0_EADD, &control_enable, 1);

ret = 0;

if ((pMsg->EvsMngEvent != EVS_ERROR_CAPTURE)
&& ((state == EVSTATE_RES_ERROR)/* || (state == EVSTATE_PLUG_OUT)*/ || (state == EVSTATE_ERROR_WAIT)
#ifdef HW_MP28947    
 || (state == EVSTATE_F_STATE_ERROR) || (state == EVSTATE_X1_STATE_ERROR) || (state < EVSTATE_INIT)))    
#else
 || (state == EVSTATE_F_STATE_ERROR) || (state == EVSTATE_X1_STATE_ERROR)|| (state < EVSTATE_MODE_SEL)))
#endif
    return ret;

evs_error_get(error_capture_array, 0, 0, 0);

if (pMsg->EvsMngEvent == CPPP_WAIT_TIM_EXPIRED)
    {
    if (plug_presence_get() & (PP_FILTERED | PP_INSTANT))
        pilot_lost_set(CPLOST_ANOM1, 1);
    else    // if (plug_presence_get() & (CP_FILTERED | CP_INSTANT))
        pilot_lost_set(PPLOST_ANOM1, 1);
    }

if ((pMsg->EvsMngEvent == EVS_ERROR_CAPTURE) || (state <= EVSTATE_SOCKET_AVAILABLE))
    {
    if ((snp3_error_save == 0) && (getInputState_enable == 1) && (getInput(IN4_RCBO_EXP0) == GPIO_PIN_RESET) && (control_enable & RCBO_CRL0))
        evs_error_set(CONTROL_BYTE_0, RCBO_ANOM0, 1);

    if ((snp3_error_save == 0) && (getInputState_enable == 1) && (getInput(RCDM_IN_UP_PIN_UP) == GPIO_PIN_SET))
        evs_error_set(CONTROL_BYTE_0, RCDM_ANOM0, 1);
    }

pilot_anom1_update();
rfid_anom1_update();

if (pMsg->EvsMngEvent == EVS_RCDM_OPEN)
    evs_error_set(CONTROL_BYTE_0, RCDM_ANOM0, 1);

if ((snp3_error_save == 0) && (pMsg->EvsMngEvent == EVS_MIRROR_ERROR))
    evs_error_set(CONTROL_BYTE_0, MIRROR_ANOM0, 1);

if ((snp3_error_save == 0) && (pMsg->EvsMngEvent == EVS_RCBO_CLOSE))
    evs_error_set(CONTROL_BYTE_0, RCBO_ANOM0, 1);

if (evs_error_array[1] & MIFARE_ANOM1)              // l'errore MIFARE_ANOM1 nel ciclo di confronto sotto non origina ret = 1 in uscita [evs_error_wait_capture]
    error_capture_array[1] |= MIFARE_ANOM1;
else
    error_capture_array[1] &=~ MIFARE_ANOM1;

if (evs_error_array[1] & EMETER_INT_ANOM1)          // l'errore EMETER_INT_ANOM1 nel ciclo di confronto sotto non origina ret = 1 in uscita [evs_error_wait_capture]
    error_capture_array[1] |= EMETER_INT_ANOM1;
else
    error_capture_array[1] &=~ EMETER_INT_ANOM1;

if (evs_error_array[2] & EMETER_EXT_ANOM2)          // l'errore EMETER_EXT_ANOM2 nel ciclo di confronto sotto non origina ret = 1 in uscita [evs_error_wait_capture]
    {
    eeprom_param_get(HIDDEN_MENU_ENB_EADD, &pmng_enable, 1);            // legge power management enable
    pmng_enable &= HIDDEN_MENU_PMNG_ENB;
    
    if (pmng_enable == HIDDEN_MENU_PMNG_ENB)
        error_capture_array[2] |= EMETER_EXT_ANOM2;
    }
else
    error_capture_array[2] &=~ EMETER_EXT_ANOM2;

if (evs_error_array[2] & SINAPSI_CHN2_ANOM2)        // l'errore SINAPSI_CHN2_ANOM2 nel ciclo di confronto sotto non origina ret = 1 in uscita [evs_error_wait_capture]
    {
    eeprom_param_get(HIDDEN_MENU_ENB_EADD, &pmng_enable, 1);            // legge power management enable
    pmng_enable &= HIDDEN_MENU_PMNG_ENB;
    
    if (pmng_enable == HIDDEN_MENU_PMNG_ENB)
        error_capture_array[2] |= SINAPSI_CHN2_ANOM2;
    }
else
    error_capture_array[2] &=~ SINAPSI_CHN2_ANOM2;

if (memcmp(evs_error_array, error_capture_array, (size_t)EVS_ERROR_ARRAY_SIZE) != 0)
    {
    evs_tPrintf_error(error_capture_array);
    ret = 1;
    }

return ret;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_lid_error
//
//  DESCRIPTION:    controlla lo stato del coperchio in base alla configurazione della presa
//
//  INPUT:          -
//
//  OUTPUT:         1 = errore: posizione del coperchio incoerente con lo stato
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t evs_lid_error(consistency_en consistency, uint8_t gsy_update)
{
uint8_t control_enable, socket_type;

eeprom_param_get(CONTROL_BYTE0_EADD, &control_enable, 1);
eeprom_param_get(SOCKET_TYPE_EADD, &socket_type, 1);

evs_error_set(CONTROL_BYTE_0, LID_ANOM0, 0);

if ((getInputState(IN8_LID_EXP0) == GPIO_PIN_SET) && ((consistency == NO_PLUG_CONSISTENCY) || ((consistency == PLUG_IN_CONSISTENCY) && (socket_type & LID_CLOSE_IN_CHARGE))))
    {
    if ((evs_state == EVSTATE_PLUG_OUT) || (gsy_update == 0))
        return 1;
    else
        evs_error_set(CONTROL_BYTE_0, LID_ANOM0, 1);
    }

if (evs_error_array[0] & LID_ANOM0)
    return 1;
else
    return 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_error_update
//
//  DESCRIPTION:    reset errori resettabili con l'estrazione della spina; controlla la persistenza di altri errori che inibiscono la stazione
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void evs_error_update(void)
{
uint8_t control_enable;

eeprom_param_get(CONTROL_BYTE0_EADD, &control_enable, 1);

evs_error_set(CONTROL_BYTE_0, EVS_RESETTABLE_ERROR0, 0);
evs_error_set(CONTROL_BYTE_1, EVS_RESETTABLE_ERROR1, 0);
evs_error_set(CONTROL_BYTE_2, EVS_RESETTABLE_ERROR2, 0);

pilot_error_reset();
block_error_reset();

contact_anom0_update();
rfid_anom1_update();

if ((getInputState(RCDM_IN_UP_PIN_UP) == GPIO_PIN_RESET) || ((control_enable & RCDM_CRL0) == 0))
    evs_error_set(CONTROL_BYTE_0, RCDM_ANOM0, 0);
else if ((snp3_error_save == 0) && (getInputState_enable == 1))
    evs_error_set(CONTROL_BYTE_0, RCDM_ANOM0, 1);

if (((getInputState(IN4_RCBO_EXP0) == GPIO_PIN_SET) && (REM_ACT_STATUS_INPUT == GPIO_PIN_SET)) || ((control_enable & RCBO_CRL0) == 0))
    evs_error_set(CONTROL_BYTE_0, RCBO_ANOM0, 0);
else if ((snp3_error_save == 0) && (getInputState_enable == 1))
    evs_error_set(CONTROL_BYTE_0, RCBO_ANOM0, 1);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_control_save
//
//  DESCRIPTION:    salva in flash la corrente configurazione di abilitazione dei controlli
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void evs_control_save(void)
{
eeprom_param_get(CONTROL_BYTE0_EADD, control_byte_old, CONTROL_BYTE_NUM);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_chn2_init
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void evs_chn2_init(void)
{
uint8_t control_enable;

eeprom_param_get(CONTROL_BYTE2_EADD, &control_enable, 1);

if ((control_enable & EMETER_EXT_CRL2) && ((control_enable & SINAPSI_CHN2_CRL2) == 0))
    {
    control_enable |= SINAPSI_CHN2_CRL2;
    SCU_InfoStation_Set ((uint8_t *)&infoStation.controlByte.Byte.Byte2, &control_enable, 1);       /* ex CONTROL_BYTE2_EADD */
    }
else if (((control_enable & EMETER_EXT_CRL2) == 0) && (control_enable & SINAPSI_CHN2_CRL2))
    {
    control_enable &=~ SINAPSI_CHN2_CRL2;
    SCU_InfoStation_Set ((uint8_t *)&infoStation.controlByte.Byte.Byte2, &control_enable, 1);       /* ex CONTROL_BYTE2_EADD */
    }

eeprom_param_get(SINAPSI_INST_EADD, &sinapsi_inst_status, 1);

chn2_error_enable = 0;
internal_chn2_error_enable = 0;

if (sinapsi_inst_status == 1)   // installazione avvenuta con successo [all'accensione precedente]
    internal_chn2_error_enable = 1;
else
    chn2_error_enable = 1;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_error_control
//
//  DESCRIPTION:    a seguito di un aggiornamento dei controlli abilitati azzera o attiva gli errori presenti
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void evs_error_control(uint8_t *control, uint16_t add, uint8_t num)
{
uint8_t control_bit, control_byte;
uint16_t i;

for (control_byte=0; control_byte<num; control_byte++)
    {
    control_bit = 0x01;

    for (i=0; i<8; i++)
        {
        if (((*(control + control_byte) & control_bit) == control_bit) && ((control_byte_old[control_byte] & control_bit) == 0))
            send_to_evs(EVS_ERROR_CAPTURE);

        control_bit <<= 1;
        }
    }

for (i=0; i<num; i++)
    evs_error_array[(i + (add - CONTROL_BYTE0_EADD))] &= *(control + i);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_mode_get
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         evs_state
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t evs_mode_get(void)
{
return evs_mode;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_state_get
//
//  DESCRIPTION:    ritorna lo stato della stazione
//
//  INPUT:          none
//
//  OUTPUT:         evs_state
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
evs_state_en evs_state_get(void)
{
return evs_state;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_charging_resume_get
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t evs_charging_resume_get(void)
{
return evs_charging_resume;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_charging_resume_sem_get
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t evs_charging_resume_sem_get(void)
{
return evs_charging_resume_sem;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_charging_resume_reset
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void evs_charging_resume_reset(void)
{
evs_charging_resume = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_charging_resume_sem_reset
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void evs_charging_resume_sem_reset(void)
{
evs_charging_resume_sem = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_gost_param_get
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t evs_gost_param_get(void)
{
return (evs_gost_plug | evs_gost_lid);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_state_set
//
//  DESCRIPTION:    imposta i parametri necessari allo stato da impostare
//
//  INPUT:          puntatore stato: state_ptr; stato da impostare: set_state
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void evs_state_set(evs_state_en *state_ptr, evs_state_en set_state)
{
uint8_t         reserved, socket_enable, control_enable, battery_config, actuator_mode, error_array[EVS_ERROR_ARRAY_SIZE];
evs_state_en    destination_state = set_state;
int32_t         activeSessionEnergy;

eeprom_param_get(SOCKET_ENABLE_EADD, &reserved, 1);

eeprom_param_get(BATTERY_CONFIG_EADD, &battery_config, 1);

setLed((ledIdx_e)0, LED_EVENT_OFF_ALL, (uint16_t)(0), (uint8_t)(0));

if (destination_state == EVSTATE_MODE_SEL)
    {
    send_to_evstime(EVSTIME_SEM_CHARGING_STOP);
    sem_charging_time_reset = 1;
    activeSessionEnergy = measureSck.Etot;
    restoreOperativeState();

    evs_sec = 0;
    xp_short_enable_set(1);
    xp_presence_enable_set(1);
    xp_presence_freeze_set(0);

    s2_control_set(0);
    send_to_evstime(EVSTIME_BUSY_STOP);

    active_phases_num_set(0);
    hidden_menu_enable_set(1);
    
    snp3_error_save = 0;
    evstate_plug_out_post = 0;
    
    post_suspending = 0;
    post_suspending_num = 1;
    suspending_enable = 0;
    charging_mode = CHARGING_MODE_NULL;
    
    eeprom_param_get(EVS_MODE_EADD, &evs_mode, 1);
    
    eeprom_param_get(SOCKET_ENABLE_EADD, &socket_enable, 1);
    socket_enable &= EVS_MODE_AVAILABLE;

    eeprom_param_get(CONTROL_BYTE1_EADD, &control_enable, 1);

    send_to_rfid(RFID_CONTROL_START);                                       // start RfidMngTask
    send_to_pers(PERS_START);                                               // start PersMngTask
    sendDiffRiarmMsg(DIFF_RIARM_EVENT_POLLING);
    send_to_evs(EVS_MIRROR_READ);

#ifndef HW_MP28947    
    if (ISO15118_Status.board_detected)
        evs_iso15118_run = 1;
#endif

    ISO15118_homeplugdev_info_host_set(ISO15118_STOPEVSE, 0);
    evs_iso15118_state = ISO15118_NULL;
    evs_iso15118_num_test = 0;
    pwm_iso15118_clr();

    if (socket_enable == 0)
        {
        gsy_enable_set();
        destination_state = EVSTATE_DISABLED;
        }
    else if (evs_mode == EVS_FREE_MODE)
        {
        gsy_enable_set();
        destination_state = EVSTATE_SOCKET_AVAILABLE;
        }
    else
        {
        if ((evs_state == EVSTATE_DELAY) && ((evs_rtc_backup_get(BACKUP_CHARGE_STATUS) & AUTH_START_SAVE) == AUTH_START_SAVE))
            {
            evs_set_timer(SBC_GSY_TIM, SBC_GSY_TIME);
            destination_state = EVSTATE_AUTH_WAIT;
            }
        else
            {
            gsy_enable_set();
            destination_state = EVSTATE_AUTH_WAIT;
            }
        }

    evs_mode_update = 0;
    evs_set_timer(EMETER_INT_ANOM1_TIM, EMETER_INT_ANOM1_TIME);

    if ((evs_rtc_backup_get(BACKUP_CHARGE_STATUS) & AUTH_START_SAVE) == AUTH_START_SAVE)
    	initEmParameter(EVT_START_ENRG_ACT_SESS_MEAS);
    else // if ((evs_rtc_backup_get(BACKUP_CHARGE_STATUS) & AUTH_START_SAVE) == NULL_CHARGE_SAVE)
    	{
        stopNewTransaction(activeSessionEnergy);
        initEmParameter(EVT_STOP_ENRG_ACT_SESS_MEAS);
        measureSck.duration = 0;
        initEmParameter_enb = 1;
    	}
    }
else
    hidden_menu_init();

switch (destination_state)
    {
    case EVSTATE_ISO15118_WAIT:
        {
        evs_iso15118_state = ISO15118_WAIT;
        evs_iso15118_tim_enable = 1;
        evs_set_timer(EVS_ISO15118_TIM, ISO15118_WAIT_TIME);
        }
        break;
    
    case EVSTATE_ISO15118_WAKEUP:
        {
        xp_short_enable_set(0);
        iso15118_wakeup_step = 0;
        send_to_pwm(PWM_OUTPUT_LOW);
        evs_iso15118_tim_enable = 1;
        evs_set_timer(EVS_ISO15118_TIM, ISO15118_F_TIME);
        }
        break;
    
    case EVSTATE_INIT:
        {
        setLed(LED_A_BLU, LED_EVENT_BLINK, (uint16_t)(1000), LED_BLUE_DEFAULT);
        gsy_quick_polling_update(BUSY_OUTLET, 1);
        send_to_pwm(PWM_OUTPUT_HIGH);                                               // start PwmMngTask

#ifndef HW_MP28947        
        send_to_inp(IO_EVENT_START_IOEXP_POLLING);                                  // start inputMngTask        
        getInputState_enable = 1;
#else
        startCompletePolling();
#endif
        
        evsModbusState = MDBSTATE_NULL_STATE; // MDBSTATE_STARTING;
        }
        break;

    case EVSTATE_DELAY:  // No user allowed
        {
        evs_set_timer(EVS_STATE_TIM, DELAY_STATE_TIME);
        send_to_pilot(PILOT_START);
        evsModbusState = MDBSTATE_REBOOTING; 
        }
        break;

    case EVSTATE_CPSET:
        {
        setLed(LED_A_BLU, LED_EVENT_BLINK, (uint16_t)(1000), LED_BLUE_DEFAULT);
        evs_set_timer(CPSET_STATE_TIM, CPSET_STATE_TIME);

        evsModbusState = MDBSTATE_NULL_STATE; // MDBSTATE_STARTING;
        }
        break;

    case EVSTATE_DISABLED:
        {
        setLed(LED_C_RED, LED_EVENT_ON, (uint16_t)(0), LED_RED_DEFAULT);
        setLed(LED_B_GREEN, LED_EVENT_ON, (uint16_t)(0), LED_GREEN_DEFAULT);

        send_to_block(BLOCK_UP_REQ);
        send_to_lcd(LCD_EVS_DISABLED);

        evsModbusState = MDBSTATE_UNAVAILABLE;
        }
        break;

    case EVSTATE_AUTH_WAIT:
        {
        if (evs_reserved == 1)
            {
            setLedForReservedMode();
	        evsModbusState = MDBSTATE_RESERVED;
            }
        else
            {
            setLed(LED_B_GREEN, LED_EVENT_ON, (uint16_t)(0), LED_GREEN_DEFAULT);
	        evsModbusState = MDBSTATE_AVAILABLE;
	        }

        eeprom_param_get(ACTUATORS_EADD, &actuator_mode, 1);
        actuator_mode &= PAUT_ATT0;

        if ((vbus_anom1_save == 0) && (actuator_mode == 0))
            send_to_block(BLOCK_UP_REQ);
        else
            send_to_block(BLOCK_DOWN_REQ);

        send_to_lcd(LCD_CARD_WAIT);
        lcd_msg_update_num = 3;
        }
        break;

    case EVSTATE_PAUT_LID:
        {
        setLed(LED_A_BLU, LED_EVENT_BLINK, (uint16_t)(1000), LED_BLUE_DEFAULT);
        evs_gost_lid = 1;
        evs_sec = DISP_WAIT_POINTS;

        if ((isSemMode() == TRUE) && (sem_time_valid == 1))
            evs_wait_time = sem_wait_time;
        else
            evs_wait_time = EVS_WAIT_DEF_TIME;

        evs_sec = DISP_WAIT_POINTS;
        evs_wait_time = ((evs_wait_time + 10)/20);
        evs_set_timer(EVS_WAIT_TIM, evs_wait_time);

        send_to_lcd(LCD_CARD_PENDING);

        if (evs_reserved == 1)
	        evsModbusState = MDBSTATE_RESERVED;
        else
	        evsModbusState = MDBSTATE_AVAILABLE;
        }
        break;

    case EVSTATE_LID_ERROR:
    case EVSTATE_CLOSE_LID:
        {
        setLed(LED_C_RED, LED_EVENT_ON, (uint16_t)(0), LED_RED_DEFAULT);
        evs_set_timer(EVS_STATE_TIM, EVS_LID_TIME);
        send_to_block(BLOCK_DOWN_REQ);
        send_to_lcd(LCD_CLOSE_LID);

        evsModbusState = MDBSTATE_FAULTED;
        }
        break;

    case EVSTATE_SOCKET_AVAILABLE:
        {
        setLed(LED_B_GREEN, LED_EVENT_ON, (uint16_t)(0), LED_GREEN_DEFAULT);
        send_to_block(BLOCK_DOWN_REQ);
        send_to_lcd(LCD_SOCKET_AVAILABLE);

        lcd_msg_update_num = 3;
        evsModbusState = MDBSTATE_AVAILABLE;
        }
        break;

    case EVSTATE_AUTH_PENDING:
        {
        setLed(LED_A_BLU, LED_EVENT_BLINK, (uint16_t)(1000), LED_BLUE_DEFAULT);

        if (evs_gost_plug == 1)
            {
            if ((isSemMode() == TRUE) && (sem_time_valid == 1))
                evs_wait_time = ((sem_wait_time + 10)/20);
            else
                evs_wait_time = 3000;
            }
        else
            evs_wait_time = 1000;

        evs_sec = DISP_WAIT_POINTS;
        evs_set_timer(EVS_WAIT_TIM, evs_wait_time);

        eeprom_param_get(ACTUATORS_EADD, &actuator_mode, 1);
        actuator_mode &= PAUT_ATT0;

        if ((actuator_mode & PAUT_ATT0) && (evs_gost_plug == 1))
	        evsModbusState = MDBSTATE_PREPARING;
        else if (evs_reserved == 1)
	        evsModbusState = MDBSTATE_RESERVED;
        else
	        evsModbusState = MDBSTATE_AVAILABLE;
        }
        break;

    case EVSTATE_AUTH_MISSED:
        {
        setLed(LED_A_BLU, LED_EVENT_BLINK, (uint16_t)(1000), LED_BLUE_DEFAULT);
        gsy_quick_polling_update((BUSY_OUTLET | RFID_PENDING), 0);
        evs_set_timer(EVS_STATE_TIM, AUTH_MISSED_TIME);
        send_to_lcd(LCD_AUTH_MISSED);

        if (evs_reserved == 1)
	        evsModbusState = MDBSTATE_RESERVED;
        else
	        evsModbusState = MDBSTATE_AVAILABLE;
        }
        break;

    case EVSTATE_PLUG_WAIT:
        {
        setLed(LED_A_BLU, LED_EVENT_BLINK, (uint16_t)(1000), LED_BLUE_DEFAULT);
        plug_wait_lcd = 1;
        evs_sec = DISP_WAIT_POINTS;
        gsy_quick_polling_update(BUSY_OUTLET, 1);
        send_to_block(BLOCK_DOWN_REQ);

//        if ((evs_mode == EVS_PERS_MODE) && (battery_config != 0))
        if (battery_config != 0)
            {
            if ((evs_rtc_backup_get(BACKUP_CHARGE_STATUS) & AUTH_START_SAVE) == AUTH_START_SAVE)
                {
                send_to_pers(PERS_RTC_BACKUP_START);
                send_to_evstime(EVSTIME_RTC_BACKUP_START);

                if (busy_time_run_get() == 0)
                  send_to_evstime(EVSTIME_BUSY_START);
                }
            else
                {
                send_to_pers(PERS_RTC_BACKUP_SAVE);
                evs_rtc_backup_set(BACKUP_CHARGE_STATUS, AUTH_START_SAVE);
                }
            }
        else
            evs_rtc_backup_set(BACKUP_CHARGE_STATUS, AUTH_RESET_SAVE);

        evsModbusState = MDBSTATE_PREPARING;
        }
        break;

    case EVSTATE_LID_WAIT:
        {
        send_to_lcd(LCD_CLOSE_LID);

        evsModbusState = MDBSTATE_FAULTED;
        }
        break;
    
    case EVSTATE_PLUG_CHECK:
        {
        setLed(LED_A_BLU, LED_EVENT_BLINK, (uint16_t)(1000), LED_BLUE_DEFAULT);
        gsy_quick_polling_update(BUSY_OUTLET, 1);
        evs_set_timer(EVS_STATE_TIM, PLUG_CHECK_TIME);
        send_to_lcd(LCD_PLUG_CHECK);

        evsModbusState = MDBSTATE_PREPARING;
        }
        break;

    case EVSTATE_BLOCK_UP_DELAY:
        {
        setLed(LED_A_BLU, LED_EVENT_BLINK, (uint16_t)(1000), LED_BLUE_DEFAULT);
        gsy_quick_polling_update(BUSY_OUTLET, 1);
        evs_set_timer(EVS_STATE_TIM, BLOCK_UP_DELAY_TIME);
        send_to_lcd(LCD_PLUG_CHECK);

        evsModbusState = MDBSTATE_PREPARING;
        }
        break;

    case EVSTATE_SOCKET_CHECK:
        {
        evs_gost_plug = 0;        
        setLed(LED_A_BLU, LED_EVENT_BLINK, (uint16_t)(1000), LED_BLUE_DEFAULT);

        if ((isSemMode() == TRUE) && (sem_time_valid == 1))
            evs_wait_time = sem_wait_time;
        else
            evs_wait_time = EVS_WAIT_DEF_TIME;

//        evs_wait_time = ((evs_wait_time + 10)/20);
        evs_set_timer(CPPP_WAIT_TIM, evs_wait_time);

        xp_short_enable_set(1);
        send_to_block(BLOCK_UP_REQ);
        send_to_lcd(LCD_PLUG_CHECK);

        evsModbusState = MDBSTATE_PREPARING;
        }
        break;

    case EVSTATE_M3S_M3T_DETECT:
        {
        setLed(LED_A_BLU, LED_EVENT_BLINK, (uint16_t)(1000), LED_BLUE_DEFAULT);
        #ifndef RENAULT_EV_READY
        suspending_enable = 1;
        #endif
        
        if (evs_mode == EVS_FREE_MODE)
            startNewTransaction(UID_TRANSACTION_FREE);                                  // notifica start transaction x bluetooth
        else if (evs_mode == EVS_PERS_MODE)
        {
          if (startPersonal == PERSONAL_BY_CARD)
          {
            startNewTransaction(UID_TRANSACTION_PERS_BY_CARD);
          }
          else
          {
            startNewTransaction(UID_TRANSACTION_PERS_BY_APP);
          }
        }
        else if (evs_mode == EVS_NET_MODE)
            startNewTransaction(UID_TRANSACTION_PERS_BY_APP);
        else if (evs_mode == EVS_OCPP_MODE)
            startNewTransaction(UID_TRANSACTION_PERS_BY_APP);

        s2_control_set(1);
        send_to_lcd(LCD_S2_WAIT);

        if (mdb_ev_connectedd_hide == 0)
            evsModbusState = MDBSTATE_EV_CONNECTED;
        
        evs_iso15118_s2 = 0;
        }
        break;

    case EVSTATE_RECTIFIER_CHECK:
        {
        setLed(LED_A_BLU, LED_EVENT_BLINK, (uint16_t)(1000), LED_BLUE_DEFAULT);
        rect_enable_set(1);

        if (mdb_ev_connectedd_hide == 0)
            evsModbusState = MDBSTATE_EV_CONNECTED;

        mdb_ev_connectedd_hide = 0;
        }
        break;

    case EVSTATE_SUSPENDING:
        {
        setLed(LED_A_BLU, LED_EVENT_BLINK, (uint16_t)(1000), LED_BLUE_DEFAULT);
        ev_suspending = 0;
        s2_control_set(0);
        send_to_contact(CONTACT_OPEN_REQ);
        send_to_pwm(PWM_OUTPUT_HIGH);

        if (busy_time_run_get() == 0)
            send_to_evstime(EVSTIME_BUSY_START);

        if (suspending_enable_set == 0)
            evs_set_timer(EVS_SUSPENDING_TIM, EVSTATE_WAITING_30S);

        suspending_enable_set = 1;
        
        eeprom_param_get(BATTERY_CONFIG_EADD, &control_enable, 1);
        
        evsModbusState = MDBSTATE_SUSPENDING_EVSE;

        if (suspending_set & PEN_SUSPENDING)
            {
            suspending_set_pen_save = 1;
            send_to_lcd(LCD_PENWAITNG);
            }
        else if (suspending_set & (UART_SUSPENDING | REMOTE_SUSPENDING | SINAPSI_SUSPENDING | APP_SUSPENDING))
            send_to_lcd(LCD_RMWAITNG);
        else if (suspending_set & HTS_SUSPENDING)
            send_to_lcd(LCD_HTSWAITNG);
        else if (suspending_set & PMNG_SUSPENDING)
            {
            send_to_lcd(LCD_NOPOWER);
            evsModbusState = MDBSTATE_SUSPENDED_EVSE_NOPOWER;
            }
        else // if ((suspending_set & EMEX_SUSPENDING) || (suspending_set & CHN2_SUSPENDING))
            send_to_lcd(LCD_EMWAITING);
        }
        break;

    case EVSTATE_WAKEUP:
        {
        setLed(LED_A_BLU, LED_EVENT_BLINK, (uint16_t)(1000), LED_BLUE_DEFAULT);
        #ifndef RENAULT_EV_READY
        suspending_enable = 1;
        #endif
        pwm_low_output = 1;
        xp_short_enable_set(0);
        send_to_pwm(PWM_OUTPUT_LOW);
        evs_set_timer(EVS_STATE_TIM, WAKEUP_STATE_A_TIME);
        send_to_lcd(LCD_S2_WAIT);

        if (sem_charging_time_reset == 1)
            send_to_evstime(EVSTIME_SEM_CHARGING_RESET);

        sem_charging_time_reset = 0;

        evsModbusState = MDBSTATE_SUSPENDING_EV;
        }
        break;

    case EVSTATE_S2_WAITING:
        {
        setLed(LED_A_BLU, LED_EVENT_BLINK, (uint16_t)(1000), LED_BLUE_DEFAULT);
          evs_set_timer(EVS_STATE_TIM, EVSTATE_WAITING_30S);
        
        s2_control_set(1);
        send_to_contact(CONTACT_OPEN_REQ);

        if (sem_charging_time_reset == 1)
            send_to_evstime(EVSTIME_SEM_CHARGING_RESET);

        sem_charging_time_reset = 0;

        if (busy_time_run_get() == 0)
            send_to_evstime(EVSTIME_BUSY_START);

        if (ev_suspending == 1)
            send_to_lcd(LCD_SUSPENDING);
        else
            send_to_lcd(LCD_S2_WAIT);

        if (mdb_suspending_ev_hide == 0)        
            evsModbusState = MDBSTATE_SUSPENDING_EV;

        mdb_suspending_ev_hide = 0;
        
        evs_iso15118_count = 0;
        }
        break;

    case EVSTATE_CONTACT_CLOSE_DELAY:
        {
        setLed(LED_A_BLU, LED_EVENT_ON, (uint16_t)(0), LED_BLUE_DEFAULT);
        evs_set_timer(EVS_STATE_TIM, CONTACT_CLOSE_DELAY_TIME);
        ev_suspending = 0;
        
        if (busy_time_run_get() == 0)
            send_to_evstime(EVSTIME_BUSY_START);

        if (sem_charging_time_reset == 1)
            send_to_evstime(EVSTIME_SEM_CHARGING_RESET);

        sem_charging_time_reset = 0;
        
        send_to_lcd(LCD_CHARGING);

        evsModbusState = MDBSTATE_CHARGING;
        }
        break;

    case EVSTATE_CHARGING:
        {
//        setLed(LED_A_BLU, LED_EVENT_SOFT_LIGHT, (uint16_t)(0), LED_BLUE_DEFAULT);
        setLed(LED_A_BLU, LED_EVENT_ON, (uint16_t)(0), LED_BLUE_DEFAULT);

        if (suspending_enable == 0)
            evs_set_timer(EVS_SUSPENDING_TIM, EVSTATE_WAITING_30S);

        suspending_enable_set = 1;
        send_to_contact(CONTACT_CLOSE_REQ);
        
        eeprom_param_get(BATTERY_CONFIG_EADD, &control_enable, 1);
        
        if (charging_time_run_get() == 0)
            send_to_evstime(EVSTIME_CHARGING_START);

        suspending_save = 0;
        send_to_lcd(LCD_CHARGING);
        send_to_pwm(PWM_MDB_POWER_UPDATE);
        send_to_evstime(EVSTIME_SEM_CHARGING_START);

        evsModbusState = MDBSTATE_CHARGING;
        }
        break;

    case EVSTATE_INTERRUPTING:
        {
        if (evs_error_get(error_array, 1, 1, 6) == 1)
            setLed(LED_C_RED, LED_EVENT_BLINK, (uint16_t)(1000), LED_RED_DEFAULT);
        else
            setLed(LED_A_BLU, LED_EVENT_BLINK, (uint16_t)(1000), LED_BLUE_DEFAULT);

        ev_suspending = 0;
        evs_set_timer(EVS_STATE_TIM, INTERRUPTING_CHARGE_TIME);
        
        send_to_pwm(PWM_OUTPUT_HIGH);
        
        eeprom_param_get(BATTERY_CONFIG_EADD, &control_enable, 1);

        send_to_lcd(LCD_INTERRUPTING_CHARGE);
        }
        break;

    case EVSTATE_BLOCK_DOWN_DELAY:
        {
        setLed(LED_A_BLU, LED_EVENT_BLINK, (uint16_t)(1000), LED_BLUE_DEFAULT);
        evs_set_timer(EVS_STATE_TIM, BLOCK_DOWN_DELAY_TIME);
        send_to_contact(CONTACT_OPEN_REQ);
        send_to_lcd(LCD_INTERRUPTING_CHARGE);
        }
        break;

    case EVSTATE_RES_ERROR:
    case EVSTATE_F_STATE_ERROR:
        {
        setLed(LED_C_RED, LED_EVENT_BLINK, (uint16_t)(1000), LED_RED_DEFAULT);

        ev_suspending = 0;
        send_to_contact(CONTACT_OPEN_REQ);
        send_to_evstime(EVSTIME_CHARGING_STOP);
        pmng_pid_enable_set(0);
        
        if (destination_state == EVSTATE_RES_ERROR)
            {
            evs_set_timer(EVS_STATE_TIM, RESETTABLE_ERROR_TIME);
            send_to_pwm(PWM_OUTPUT_HIGH);
            }
        else    // if (destination_state == EVSTATE_F_STATE_ERROR)
            {
            xp_short_enable_set(0);
            xp_presence_enable_set(0);
            evs_set_timer(EVS_STATE_TIM, F_STATE_ERROR_TIME);
            send_to_pwm(PWM_OUTPUT_LOW);
            }

        gsy_quick_polling_update(BUSY_OUTLET, 1);
        evs_rtc_backup_set(BACKUP_CHARGE_STATUS, AUTH_RESET_SAVE);
        send_to_lcd(LCD_EVS_ERROR);
        
        evsModbusState = MDBSTATE_FAULTED;
        }
        break;

    case EVSTATE_X1_STATE_ERROR:
        {
        send_to_pwm(PWM_OUTPUT_HIGH);
        evs_set_timer(EVS_STATE_TIM, X1_STATE_ERROR_TIME);
        
        evsModbusState = MDBSTATE_FAULTED;
        }
        break;
    
    case EVSTATE_PLUG_OUT:
        {
        setLed(LED_A_BLU, LED_EVENT_BLINK, (uint16_t)(1000), LED_GREEN_DEFAULT);
        ev_suspending = 0;
        gsy_quick_polling_update(BUSY_OUTLET, 1);
        send_to_block(BLOCK_DOWN_REQ);
        send_to_evstime(EVSTIME_CHARGING_STOP);
        send_to_pwm(PWM_OUTPUT_HIGH);
        evs_set_timer(EVS_STATE_TIM, pdMS_TO_TICKS((uint32_t)(500)));
        evs_rtc_backup_set(BACKUP_CHARGE_STATUS, AUTH_RESET_SAVE);
        send_to_lcd(LCD_PLUG_OUT);

        evsModbusState = MDBSTATE_END_CHARGE;
        }
        break;

    case EVSTATE_LID_ERROR_DELAY:
        {
        evs_set_timer(LID_ERROR_DELAY_TIM, LID_ERROR_DELAY_TIME);
        }
        break;

    case EVSTATE_AUTH_NEG:      // authorization denied
    case EVSTATE_ERROR_WAIT:
        {
        if ((evs_error_array[0] & RCDM_ANOM0) || (evs_error_array[0] & MIRROR_ANOM0) || (evs_error_array[0] & RCBO_ANOM0)
         || (evs_error_array[1] & (MIFARE_ANOM1 | EMETER_INT_ANOM1)) || (wifi_antenna_error == 1))
            setLed(LED_C_RED, LED_EVENT_ON, (uint16_t)(0), LED_RED_DEFAULT);
        else
            {
            setLed(LED_C_RED, LED_EVENT_BLINK, (uint16_t)(1000), LED_RED_DEFAULT);
//            setLed(LED_B_GREEN, LED_EVENT_BLINK, (uint16_t)(1000), LED_GREEN_DEFAULT);
            }
        
        if (destination_state == EVSTATE_AUTH_NEG)        
            {
            gsy_quick_polling_update(BUSY_OUTLET, 0);

            eeprom_param_get(ACTUATORS_EADD, &actuator_mode, 1);
            actuator_mode &= PAUT_ATT0;

            if ((actuator_mode & PAUT_ATT0) && (evs_gost_plug == 1))
                {
                }
            else if (evs_reserved == 1)
    	        evsModbusState = MDBSTATE_RESERVED;
            else
    	        evsModbusState = MDBSTATE_AVAILABLE;
            }
        else    // if (destination_state == EVSTATE_ERROR_WAIT)
            {
            hidden_menu_enable_set(1);
            gsy_quick_polling_update(BUSY_OUTLET, 1);

            evsModbusState = MDBSTATE_FAULTED;
            }

        evs_set_timer(ERROR_WAIT_TIM, ERROR_WAIT_TIME);
        send_to_evstime(EVSTIME_CHARGING_STOP);
        evs_rtc_backup_set(BACKUP_CHARGE_STATUS, AUTH_RESET_SAVE);
        
        if (wifi_antenna_error == 1)
          send_to_lcd(LCD_ANTENNA_WIFI_ERROR);
        else if (destination_state == EVSTATE_AUTH_NEG)
            {
            if (gsy_auth_neg_type_get() == EXPIRED_CARD)
                send_to_lcd(LCD_EXPIRED_CARD);
            else if (gsy_auth_neg_type_get() == CREDIT_EXHAUSTED)
                send_to_lcd(LCD_CREDIT_EXHAUSTED);
            else
                send_to_lcd(LCD_UID_ERROR);
            }
        else
            {
            if (evs_mode > EVS_FREE_MODE)
                send_to_block(BLOCK_UP_REQ);

            send_to_lcd(LCD_EVS_ERROR);
            }
        }
        break;

    case EVSTATE_PEN_STATE:
        {
//        setLed(LED_C_RED, LED_EVENT_BLINK, (uint16_t)(1000), LED_RED_DEFAULT);
//        setLed(LED_B_GREEN, LED_EVENT_BLINK, (uint16_t)(1000), LED_GREEN_DEFAULT);
//        setLed(LED_A_BLU, LED_EVENT_OFF, (uint16_t)(1000), LED_BLUE_DEFAULT);
        setLed(LED_C_RED, LED_EVENT_ON, (uint16_t)(0), LED_BLUE_DEFAULT);
        gsy_enable_set();
        pen_state_set = 0;
        pen_charging_mode = charging_mode;
        
        if (*state_ptr != EVSTATE_LID_ERROR)
            {
            if (*state_ptr == EVSTATE_INTERRUPTING)
                evs_state_resume = EVSTATE_WAKEUP;
            else
                evs_state_resume = *state_ptr;
            }

        send_to_contact(CONTACT_OPEN_REQ);        
        send_to_pwm(PWM_OUTPUT_HIGH);
        send_to_lcd(LCD_PENWAITNG);
        }
        break;

    case EVSTATE_V230_SUSPEND:
    case EVSTATE_POWER_OFF:
        {
        if (destination_state == EVSTATE_POWER_OFF)
            {
            evs_rtc_backup_set(BACKUP_CHARGE_STATUS, AUTH_RESET_SAVE);

            if (block_state_get() != BLOCK_DOWN_STATE)
                send_to_block(BLOCK_DOWN_REQ);
            }

        xp_presence_freeze_set(1);

        evs_state_resume = *state_ptr;

        send_to_contact(CONTACT_OPEN_REQ);
        send_to_rfid(RFID_CONTROL_STOP);
        
        send_to_pwm(PWM_OUTPUT_HIGH);
        xp_short_enable_set(0);
        xp_presence_enable_set(0);

        evs_error_set(CONTROL_BYTE_1, VBUS_ANOM1, 1);
        gsy_quick_polling_update(VBUS_ANOM1, 1);
        
        sendDiffRiarmMsg(DIFF_RIARM_EVENT_STOP);
        
        send_to_lcd(LCD_POWER_OFF);
        }
        break;

    default:
        break;
    }

if (evsModbusState != evsModbusState_old)
    {
/*    if (((evs_rtc_backup_get(BACKUP_CHARGE_STATUS) & AUTH_START_SAVE) == AUTH_START_SAVE) && (evsModbusState_force == 0)
        && ((evsModbusState == MDBSTATE_PREPARING) || (evsModbusState == MDBSTATE_REBOOTING) || (evsModbusState == MDBSTATE_EV_CONNECTED) || (evsModbusState == MDBSTATE_AVAILABLE)))
      {
      }
    else*/
      {
        if (getScuTypeMode() != SCU_SEM_STAND_ALONE)
          sendChangeStatusToSemMng(evsModbusState, evsModbusState_force);
      }

    evsModbusState_old = evsModbusState;
    }

*state_ptr = destination_state;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_event_save
//
//  DESCRIPTION:    si aggiornano variabili dipendenti da eventi eventualmente non gestiti nello stato corrente
//
//  INPUT:          puntatore EvsMngEvent: pMsg
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void evs_event_save(EvsMngMsg_st *pMsg)
{
switch (pMsg->EvsMngEvent)
    {
    case EVS_BLOCK_STEADY:
        {
        block_anom0_update();
        }
        break;

    case EVS_AUTORIZATION_MODE:
    case SEM_AUTORIZATION_MODE:
        {
        evs_mode_update = 1;
        }
        break;

    case EVS_MIRROR_READ:
        {
        send_to_contact(CONTACT_CONTROL_START);                                 // start ContactMngTask
        }
        break;

    case EVS_V230_SUSPEND:  // main power off and backup active ---> Start emergency phase and suspend charging
        {
        evs_state_set(&evs_state, EVSTATE_V230_SUSPEND);
        }
        break;
    
    case EVS_V230_OFF:  // main power off ---> Start emergency phase with immediate stop charging 
        {
        if  ((evs_state > EVSTATE_MODE_SEL) && (vbus_anom1_save == 0))
            evs_state_set(&evs_state, EVSTATE_POWER_OFF);
        else
            vbus_anom1_save = 1;
        }
        break;

    case EVS_V230_ON:   // main power on ---> come back from emergency phase
        {
        evs_error_set(CONTROL_BYTE_1, VBUS_ANOM1, 0);
        gsy_quick_polling_update(VBUS_ANOM1, 0);
        }
        break;
    
    case EVS_PEN_ALM_ON:    // PEN alarm ON                                      
        {
        if (evs_state == EVSTATE_CHARGING)
            {
            pen_state_set = 1;
            suspending_enable = 1;
            suspending_request |= PEN_SUSPENDING;
            }
        else if (evs_state == EVSTATE_INTERRUPTING)
            pen_state_set = 1;
        else
            evs_state_set(&evs_state, EVSTATE_PEN_STATE);

            suspending_enable = 1;
            suspending_request |= PEN_SUSPENDING; /* RIVEDERE */
        }
        break;
    
    case EVS_SINAPSI_UPDATE:
        {
        evs_sinapsi_ptr = getIom2Ginfo();
        
        if (evs_sinapsi_ptr->m1SospRicTlimBool == 1)
            suspending_request |= SINAPSI_SUSPENDING;
        else
            suspending_request &=~ SINAPSI_SUSPENDING;
        }
        break;
    
    default:
        {
        }
        break;
    }
   
if (pMsg->EvsMngEvent == EVS_PEN_ALM_OFF)
    suspending_request &=~ PEN_SUSPENDING;

if (pMsg->EvsMngEvent == EVS_REMOTE_SUSPENDING)
    suspending_request |= REMOTE_SUSPENDING;

if (pMsg->EvsMngEvent == EVS_REMOTE_RELEASE)
    suspending_request &=~ REMOTE_SUSPENDING;

if (pMsg->EvsMngEvent == EVS_APP_SUSPENDING)
    suspending_request |= APP_SUSPENDING;

if (pMsg->EvsMngEvent == EVS_APP_RELEASE)
    suspending_request &=~ APP_SUSPENDING;

if (pMsg->EvsMngEvent == EVS_UART_SUSPENDING)
    suspending_request |= UART_SUSPENDING;

if (pMsg->EvsMngEvent == EVS_UART_RELEASE)
    suspending_request &=~ UART_SUSPENDING;

if (pMsg->EvsMngEvent == EVS_PMNG_SUSPENDING)
    suspending_request |= PMNG_SUSPENDING;

if (pMsg->EvsMngEvent == EVS_PMNG_RELEASE)
    suspending_request &=~ PMNG_SUSPENDING;

if (pMsg->EvsMngEvent == EVS_HTS_SUSPENDING)
    suspending_request |= HTS_SUSPENDING;

if (pMsg->EvsMngEvent == EVS_HTS_RELEASE)
    suspending_request &=~ HTS_SUSPENDING;

if ((suspending_enable == 1) && (pmng_suspending_enable_get() == 0))
    pmng_suspending_enable_set();

if ((suspending_enable == 1) && (suspending_request != SUSPENDING_NULL))
    suspending_set = suspending_request;
else
    suspending_set = SUSPENDING_NULL;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evs_timestamp_manager
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void evs_timestamp_manager(void)
{
switch (evs_timestamp_state)
    {
    case 0:
        {
        if ((getSinapsiEepromEn() == ENABLED) && (getSinapsiStatusPlc() == 3))
            {
            evs_timestamp_value = getSinapsiActivePowerTS();
            evs_timestamp_timeout = 0;
            evs_timestamp_state = 1;
            evs_set_timer(EVS_TIMESTAMP_TIM, EVS_TIMESTAMP_TIME);
            }
        }
        break;

    case 1:
        {
        if (getSinapsiEepromEn() == DISABLED)
            evs_timestamp_state = 0;
        else if (evs_timestamp_value != getSinapsiActivePowerTS())
            {
            evs_timestamp_value = getSinapsiActivePowerTS();
            evs_set_timer(EVS_TIMESTAMP_TIM, EVS_TIMESTAMP_TIME);
            }
        else if (evs_timestamp_timeout == 1)
            {
            chn2_error_enable_set();
            send_to_evs(EVS_SINAPSI_CHN2_FAIL);
            evs_timestamp_state = 2;
            }
        }
        break;

    case 2:
        {
        if (getSinapsiEepromEn() == DISABLED)
            evs_timestamp_state = 0;
        else if (evs_timestamp_value != getSinapsiActivePowerTS())
            {
            send_to_evs(EVS_SINAPSI_CHN2_GOOD);
            evs_timestamp_value = getSinapsiActivePowerTS();
            evs_timestamp_timeout = 0;
            evs_timestamp_state = 1;
            evs_set_timer(EVS_TIMESTAMP_TIM, EVS_TIMESTAMP_TIME);
            }
        }
        break;
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  EvsManager_init
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void EvsManager_init(void)
{
uint8_t i;

sem_time_valid = 0;
evs_wait_time = 3000;
sem_wait_time = EVS_WAIT_DEF_TIME;

for (i=0; i<EVS_ERROR_ARRAY_SIZE; i++)
    {
    evs_error_array[i] = 0x00;
    evs_error_save[i] = 0x00;
    }

evs_gost_plug = 0;
evs_gost_lid = 0;

sem_charging_time_reset = 0;

evs_charging_resume = 0;
evs_charging_resume_sem = 0;

tPrintf_error = 0xFF;
tPrintf_error_old = 0xFF;

evs_state_resume = EVSTATE_NULL;

pen_state_set = 0;
evstate_plug_out_post = 0;

pen_charging_mode = CHARGING_MODE_NULL;

post_suspending = 0;
post_suspending_num = 1;
post_suspending_time = EVSTATE_WAITING_10S;
suspending_enable_set = 0;
suspending_set_pen_save = 0;

getInputState_enable = 0;
emeter_int_anom1_save = 0;

vbus_anom1_save = 0;
wifi_antenna_error = 0;

evs_timestamp_state = 0;

suspending_request = SUSPENDING_NULL;
suspending_set = SUSPENDING_NULL;
evs_state_old = EVSTATE_IDLE;
evs_state = EVSTATE_IDLE;


evsModbusState = MDBSTATE_NULL_STATE;
evsModbusState_old = MDBSTATE_NULL_STATE;
mdb_suspending_ev_hide = 0;
mdb_suspending_ev_hide_enable = 1;
mdb_ev_connectedd_hide = 0;

evsModbusState_force = 0;

lcd_msg_update_num = 0;

evs_reserved = 0;                            
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  EvsManager
//
//  DESCRIPTION:    -
//
//  INPUT:          puntatore EvsMngEvent: pMsg
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void EvsManager(EvsMngMsg_st *pMsg)
{
uint8_t     val, emeter_type, control_enable, control, actuator_mode, battery_config, socket_type, error_array[EVS_ERROR_ARRAY_SIZE];
uint32_t    rtc_reg_save;

eeprom_param_get(BATTERY_CONFIG_EADD, &battery_config, 1);

eeprom_param_get(SOCKET_TYPE_EADD, &socket_type, 1);
evs_tPrintf_state(pMsg);
evs_event_save(pMsg);

evs_timestamp_manager();

if (evs_state != EVSTATE_V230_SUSPEND)
    {
    if (evs_res_error_capture(pMsg, evs_state) == 1)
        {
        if (evs_error_array[0] & (RCBO_ANOM0 | MIRROR_ANOM0))
            {
            if (evs_error_array[0] & MIRROR_ANOM0)
                snp3_error_save = MIRROR_ANOM0;
            else
                snp3_error_save = RCBO_ANOM0;
    
            send_to_energy(ENERGY_CONTROL_STOP);
            }
    
        if (((evs_state == EVSTATE_CHARGING) && (evs_error_array[0] & (MIRROR_ANOM0 | RCBO_ANOM0)))
        || ((evs_state == EVSTATE_S2_WAITING) && (evs_error_array[0] & (MIRROR_ANOM0 | RCBO_ANOM0))))
            evs_state_set(&evs_state, EVSTATE_F_STATE_ERROR);
        else if (evs_error_array[1] & VBUS_ANOM1)
            evs_state_set(&evs_state, EVSTATE_V230_SUSPEND);
        else
            evs_state_set(&evs_state, EVSTATE_RES_ERROR);
        
        post_suspending = 0;
        post_suspending_num = 1;
        }
    else if (evs_error_wait_capture(pMsg, evs_state) == 1)
        {
        if ((evs_state != EVSTATE_CHARGING) && (evs_state != EVSTATE_PLUG_OUT) && (evs_state != EVSTATE_ERROR_WAIT))
            {
            post_suspending = 0;
            post_suspending_num = 1;
            evs_state_set(&evs_state, EVSTATE_ERROR_WAIT);
            }
        }
    }

if (pMsg->EvsMngEvent == EVS_AUTH_NEG)
    {
    if ((evs_mode > EVS_PERS_MODE) && (evs_state != EVSTATE_AUTH_WAIT) && (evs_state != EVSTATE_AUTH_PENDING)
     && (evs_state != EVSTATE_POWER_OFF) && (evs_state != EVSTATE_V230_SUSPEND) && (evs_state != EVSTATE_LID_ERROR_DELAY))
        {
        evs_LcdMngEvent = LcdMngMsg_Old_get();
        send_to_lcd(LCD_UID_ERROR);
        evs_set_timer(EVS_UID_ERROR_TIM, EVS_UID_ERROR_TIME);
        }
    }

switch (evs_state)
    {
    case EVSTATE_IDLE:    // idle evs_state
        {
        if (pMsg->EvsMngEvent == EVS_EVENT_START)                                  // eventuali altri eventi sono ignorati
            {
            evs_state_set(&evs_state, EVSTATE_INIT);
            send_to_lcd(LCD_WRITE_SCAME);
            }
        }
        break;

    case EVSTATE_INIT: // first power on state from hw/fw reset: wait for block movements control [BlockMngTask]
        {
        if (pMsg->EvsMngEvent == EVS_BLOCK_STEADY)                                  // eventuali altri eventi sono ignorati
            evs_state_set(&evs_state, EVSTATE_CPSET);
        }
        break;

    case EVSTATE_CPSET:   // wait stable volatage after CP set [+12V]
        {
        if ((pMsg->EvsMngEvent == CPSET_TIM_EXPIRED) && (getADCvalid() == 1))
            {
            eeprom_param_get(EMETER_INT_EADD, &emeter_type, 1);

            if ((emeter_type == EMETER_TAMP) || (emeter_type == EMETER_TAMP_3))
                {
                sendMsgStartTa();
                send_to_energy(ENERGY_INTERNAL_EM_GOOD);
                }

            if (getInput(RCDM_IN_UP_PIN_UP) == GPIO_PIN_SET)
                send_to_evs(EVS_RCDM_OPEN);

            evs_set_timer(EVS_TPRINT_TIM, EVS_TPRINT_TIME);
            evs_state_set(&evs_state, EVSTATE_DELAY);
            }
        }
        break;
        
    case EVSTATE_DELAY:  // No user allowed
        {
        if (pMsg->EvsMngEvent == EVS_STATE_TIM_EXPIRED)
            evs_state_set(&evs_state, EVSTATE_MODE_SEL);
        }
        break;

    case EVSTATE_DISABLED:  // No user allowed
        {
        if (plug_presence_get() == 0)
            gsy_quick_polling_update((BUSY_OUTLET | PLUGGED_OUTLET), 0);

        if (evs_mode_update == 1)
            evs_state_set(&evs_state, EVSTATE_MODE_SEL);                            // attuazione del cambio modalità [evs_mode]
        else if (pMsg->EvsMngEvent == EVS_PULS_STOP)
            lcd_language_scroll();                                                  // aggiornamento lingua
        else if ((pMsg->EvsMngEvent == EVS_PLUG_DETECTED) || (pMsg->EvsMngEvent == EVS_PLUG_INSERTED))
            evs_state_set(&evs_state, EVSTATE_PLUG_OUT);
        else if (pMsg->EvsMngEvent == EVS_LID_UPDATE)
            {
            if (evs_lid_error(NO_PLUG_CONSISTENCY, 1) == 1)
                evs_state_set(&evs_state, EVSTATE_LID_ERROR);
            else
                evs_set_timer(EVS_STATE_TIM, EVS_LID_TIME);
            }
        }
        break;

    case EVSTATE_AUTH_WAIT: // wait for authorization / remote supervisor action
        {
        eeprom_param_get(CONTROL_BYTE0_EADD, &control_enable, 1);
        eeprom_param_get(ACTUATORS_EADD, &actuator_mode, 1);
        actuator_mode &= PAUT_ATT0;

        if (lcd_msg_update_num)
            {
            send_to_lcd(LCD_CARD_WAIT);
            lcd_msg_update_num --;
            }

        if (plug_presence_get() == 0)
            gsy_quick_polling_update((BUSY_OUTLET | PLUGGED_OUTLET), 0);

        if (pMsg->EvsMngEvent == EVS_WIFI_ANTENNA_ERROR)
            {
            wifi_antenna_error = 1;
            evs_state_set(&evs_state, EVSTATE_ERROR_WAIT);
            }
        else if (vbus_anom1_save == 1)
            {
            send_to_evs(EVS_V230_OFF);
            send_to_block(BLOCK_DOWN_REQ);
            vbus_anom1_save = 0;
            }
        else if (evs_mode_update == 1)
            evs_state_set(&evs_state, EVSTATE_MODE_SEL);                            // attuazione del cambio modalità [evs_mode]
        else if (pMsg->EvsMngEvent == EVS_PULS_STOP)
            lcd_language_scroll();
        else if (pMsg->EvsMngEvent == EVS_AUTH_NEG)                                 // è stata negata l'autorizzazione alla ricarica
            evs_state_set(&evs_state, EVSTATE_AUTH_NEG);
        else if ((pMsg->EvsMngEvent == EVS_AUTH_START) || ((evs_rtc_backup_get(BACKUP_CHARGE_STATUS) & AUTH_START_SAVE) == AUTH_START_SAVE))
            {                                                                       // è stata autorizzata la ricarica
            if ((evs_rtc_backup_get(BACKUP_CHARGE_STATUS) & AUTH_START_SAVE) == AUTH_START_SAVE)
                {
                evs_charging_resume = 1;
                evs_charging_resume_sem = 1;
                mdb_suspending_ev_hide_enable = 0;
                evsModbusState_force = 0;
                }
            else  // if (pMsg->EvsMngEvent == EVS_AUTH_START)
                evsModbusState_force = 1;

            evs_state_set(&evs_state, EVSTATE_PLUG_WAIT);
            }
        else if (pMsg->EvsMngEvent == EVS_AUTH_REQUIRED)                            // si richiede autorizzazione alla ricarica
            {
            evsModbusState_force = 1;
            evs_state_set(&evs_state, EVSTATE_AUTH_PENDING);
            gsy_quick_polling_update((BUSY_OUTLET | RFID_PENDING), 1);
            send_to_lcd(LCD_AUTH_PENDING);
            }
        else if ((pMsg->EvsMngEvent == EVS_PLUG_DETECTED) || (pMsg->EvsMngEvent == EVS_PLUG_INSERTED) || (plug_state_get() != PLUG_NOT_PRESENT))
            {                                                                       // è stata inserita una spina senza autorizzazione
            if (actuator_mode & PAUT_ATT0)
                {
                evs_gost_plug = 1;
                evsModbusState_force = 1;
                evs_state_set(&evs_state, EVSTATE_AUTH_PENDING);
                send_to_lcd(LCD_CARD_PENDING);
                }
            else
                {
                if ((block_state_get() == BLOCK_DRIVE_MOVING) || (block_state_get() == BLOCK_DRIVE_COOLING))
                    evstate_plug_out_post = 1;
                else
                    evs_state_set(&evs_state, EVSTATE_PLUG_OUT);
                }
            }
        else if ((pMsg->EvsMngEvent == EVS_LID_UPDATE) && (evs_lid_error(NO_PLUG_CONSISTENCY, 0) == 1))
            {
            if (actuator_mode & PAUT_ATT0)
                evs_state_set(&evs_state, EVSTATE_PAUT_LID);
            else if (evs_lid_error(NO_PLUG_CONSISTENCY, 1) == 1)
                evs_state_set(&evs_state, EVSTATE_LID_ERROR);
            }
        else if (pMsg->EvsMngEvent == EVS_BLOCK_STEADY)
            {
            block_error_reset();
            evs_error_set(CONTROL_BYTE_0, BLOCK_ANOM0, 0);
            
            if (evstate_plug_out_post == 1)
                evs_state_set(&evs_state, EVSTATE_PLUG_OUT);                        // è stata inserita una spina senza autorizzazione
            }
        else if (evs_lid_error(NO_PLUG_CONSISTENCY, 0) == 1)
            {
            if (((control_enable & LID_ANOM0) == 0) &&
                ((socket_type == SOCKET_T2_NO_LID) || (socket_type == SOCKET_3C_NO_LID) || (socket_type == SOCKET_3A_NO_LID)
              || (socket_type == SOCKET_SK_NO_LID)|| (socket_type == SOCKET_T1_TETHERED)|| (socket_type == SOCKET_T2_TETHERED)))
                {
                }
            else
                {
                if (actuator_mode & PAUT_ATT0)
                    evs_state_set(&evs_state, EVSTATE_PAUT_LID);
                else if (evs_lid_error(NO_PLUG_CONSISTENCY, 1) == 1)
                    evs_state_set(&evs_state, EVSTATE_LID_ERROR);
                }
            }
        }
        break;

    case EVSTATE_LID_ERROR:  // wait for lid closing: plug not inserted
        {
        if (pMsg->EvsMngEvent == EVS_LID_UPDATE)
            evs_set_timer(EVS_STATE_TIM, EVS_LID_TIME);
        else if ((pMsg->EvsMngEvent == EVS_PLUG_DETECTED) || (pMsg->EvsMngEvent == EVS_PLUG_INSERTED))
            {
            evs_error_set(CONTROL_BYTE_0, LID_ANOM0, 0);
            evs_state_set(&evs_state, EVSTATE_PLUG_OUT);                            // è stata inserita una spina senza autorizzazione
            }
        else if (pMsg->EvsMngEvent == EVS_STATE_TIM_EXPIRED)
            {
            if (evs_lid_error(NO_PLUG_CONSISTENCY, 1) == 1)
                evs_set_timer(EVS_STATE_TIM, EVS_LID_TIME);
            else
                {
                if (suspending_set & PEN_SUSPENDING)
                    {
                    if (evs_state_resume == EVSTATE_NULL)
                        evs_state = EVSTATE_MODE_SEL;            // assegnazione per evs_state_resume
    
                    evs_state_set(&evs_state, EVSTATE_PEN_STATE);
                    
                    if (evs_mode > EVS_FREE_MODE)
                        send_to_block(BLOCK_UP_REQ);
                    }
                else if (evs_error_get(error_array, 1, 1, 6) == 1)
                    evs_state_set(&evs_state, EVSTATE_ERROR_WAIT);
                else
                    evs_state_set(&evs_state, EVSTATE_MODE_SEL);
                }
            }
        }
        break;

    case EVSTATE_AUTH_PENDING:  // user authorization required
        {
        if ((pMsg->EvsMngEvent == EVS_AUTH_STOP) || (pMsg->EvsMngEvent == EVS_SEC_TIM_EXPIRED))
            {
            if (evs_gost_plug == 1)
                {
                gsy_quick_polling_update(PLUGGED_OUTLET, 1);
                evs_state_set(&evs_state, EVSTATE_PLUG_OUT);
                }
            else
                evs_state_set(&evs_state, EVSTATE_AUTH_MISSED);
            }
        else if (pMsg->EvsMngEvent == EVS_AUTH_NEG)                                     // è stata negata l'autorizzazione alla ricarica
            {
            if (evs_gost_plug == 1)
                gsy_quick_polling_update(PLUGGED_OUTLET, 1);

            evs_state_set(&evs_state, EVSTATE_AUTH_NEG);
            }
        else if (pMsg->EvsMngEvent == EVS_AUTH_START)                                   // è stata autorizzata la ricarica
            {
            if (plug_state_get() == PLUG_NOT_PRESENT)
                evs_state_set(&evs_state, EVSTATE_PLUG_WAIT);
            else
                {
                gsy_quick_polling_update(PLUGGED_OUTLET, 1);
                evs_gost_plug = 0;
                
                if (evs_lid_error(PLUG_IN_CONSISTENCY, 1) == 1)
                    evs_state_set(&evs_state, EVSTATE_LID_WAIT);
                else
                    {
                    if (battery_config != 0)
                        {
                        if ((evs_rtc_backup_get(BACKUP_CHARGE_STATUS) & AUTH_START_SAVE) == AUTH_START_SAVE)
                            {
                            send_to_pers(PERS_RTC_BACKUP_START);
                            send_to_evstime(EVSTIME_RTC_BACKUP_START);

                            if (busy_time_run_get() == 0)
                                send_to_evstime(EVSTIME_BUSY_START);
                            }
                        else
                            {
                            send_to_pers(PERS_RTC_BACKUP_SAVE);
                            evs_rtc_backup_set(BACKUP_CHARGE_STATUS, AUTH_START_SAVE);
                            }
                        }
                    else
                        evs_rtc_backup_set(BACKUP_CHARGE_STATUS, AUTH_RESET_SAVE);

                    evs_state_set(&evs_state, EVSTATE_BLOCK_UP_DELAY);
                    }
                }
            }
        }
        break;

    case EVSTATE_AUTH_MISSED:   // user authorization required failed
        {
        if (pMsg->EvsMngEvent == EVS_STATE_TIM_EXPIRED)
            {
            if (plug_state_get() != PLUG_NOT_PRESENT)
                {
                if (evs_gost_plug == 1)
                    gsy_quick_polling_update(PLUGGED_OUTLET, 1);

                evs_state_set(&evs_state, EVSTATE_PLUG_OUT);
                }
            else
                evs_state_set(&evs_state, EVSTATE_MODE_SEL);
            }
        }
        break;

    case EVSTATE_PLUG_WAIT: // evs_mode = EVS_PERS_MODE, EVS_NET_MODE o EVS_OCPP_MODE
        {
        if (pMsg->EvsMngEvent == EVS_AUTH_REQUIRED)                         		    // è stata passata una carta
            gsy_quick_polling_update((BUSY_OUTLET | RFID_PENDING), 1);
        else if ((pMsg->EvsMngEvent == EVS_AUTH_STOP) || (pMsg->EvsMngEvent == EVS_SEC_TIM_EXPIRED))
            {
            if (evs_lid_error(PLUG_IN_CONSISTENCY, 1) == 1)
                evs_state_set(&evs_state, EVSTATE_LID_ERROR);
            else
                {
    	        evs_rtc_backup_set(BACKUP_CHARGE_STATUS, AUTH_RESET_SAVE);
                evs_state_set(&evs_state, EVSTATE_MODE_SEL);
                send_to_pers(PERS_UID_RELEASE);
	        }
            }
        else if ((pMsg->EvsMngEvent == EVS_PLUG_INSERTED) || (pMsg->EvsMngEvent == EVS_PLUG_DETECTED))
            {
            if (evs_lid_error(PLUG_IN_CONSISTENCY, 1) == 1)
                evs_state_set(&evs_state, EVSTATE_LID_WAIT);
            else
                evs_state_set(&evs_state, EVSTATE_BLOCK_UP_DELAY);
            }
        else if ((pMsg->EvsMngEvent == EVS_BLOCK_STEADY) && (plug_wait_lcd == 1)) // si richiede l'inserimento della spina dopo che i blocchi si sono fermati
            {
            if (evs_error_array[0] & BLOCK_ANOM0)
                evs_state_set(&evs_state, EVSTATE_RES_ERROR);
            else
                {
                plug_wait_lcd = 0;

                if ((isSemMode() == TRUE) && (sem_time_valid == 1))
                    evs_wait_time = ((sem_wait_time + 10)/20);
                else
                    evs_wait_time = 3000;

                evs_set_timer(EVS_WAIT_TIM, evs_wait_time);
                send_to_lcd(LCD_PLUG_WAIT);
                }
            }
        }
        break;

    case EVSTATE_LID_WAIT:  // evs_mode = EVS_PERS_MODE, EVS_NET_MODE o EVS_OCPP_MODE [from and back EVSTATE_PLUG_WAIT]
        {
        if ((pMsg->EvsMngEvent == EVS_LID_UPDATE) && (evs_lid_error(PLUG_IN_CONSISTENCY, 1) == 0))
            evs_state_set(&evs_state, EVSTATE_BLOCK_UP_DELAY);
        }
        break;
    
    case EVSTATE_SOCKET_AVAILABLE:  // evs_mode = EVS_FREE_MODE
        {
        if (lcd_msg_update_num)
            {
            send_to_lcd(LCD_SOCKET_AVAILABLE);
            lcd_msg_update_num --;
            }

        if (plug_presence_get() == 0)
            gsy_quick_polling_update((BUSY_OUTLET | PLUGGED_OUTLET), 0);

        if (pMsg->EvsMngEvent == EVS_WIFI_ANTENNA_ERROR)
            {
            wifi_antenna_error = 1;
            evs_state_set(&evs_state, EVSTATE_ERROR_WAIT);
            }
        else if (vbus_anom1_save == 1)
            {
            send_to_evs(EVS_V230_OFF);
            send_to_block(BLOCK_DOWN_REQ);
            vbus_anom1_save = 0;
            }
        else if (evs_mode_update == 1)
            evs_state_set(&evs_state, EVSTATE_MODE_SEL);                            // attuazione del cambio modalità [evs_mode]
        else if (pMsg->EvsMngEvent == EVS_PULS_STOP)
            lcd_language_scroll();                                                  // aggiornamento lingua
        else if ((pMsg->EvsMngEvent == EVS_PLUG_INSERTED) || (pMsg->EvsMngEvent == EVS_PLUG_DETECTED) || (((socket_type & EVS_TETHERED) == 0) && (plug_presence_get() != 0)))
            {
            if (evs_lid_error(PLUG_IN_CONSISTENCY, 1) == 1)
                evs_state_set(&evs_state, EVSTATE_LID_WAIT);
            else
                {
                if ((evs_rtc_backup_get(BACKUP_CHARGE_STATUS) & FREE_START_SAVE) == FREE_START_SAVE)
                    {
                    evs_charging_resume_sem = 1;
                    mdb_suspending_ev_hide_enable = 0;
                    }

                if ((evs_rtc_backup_get(BACKUP_CHARGE_STATUS) & FREE_START_SAVE) != FREE_START_SAVE)
                    evs_rtc_backup_set(BACKUP_CHARGE_STATUS, FREE_START_SAVE);

                evs_state_set(&evs_state, EVSTATE_BLOCK_UP_DELAY);
                }
            }
        else if (pMsg->EvsMngEvent == EVS_STATE_TIM_EXPIRED)
            {
            if (evs_mode != EVS_FREE_MODE)
                {
                evs_set_timer(EVS_STATE_TIM, portMAX_DELAY);
                evs_state_set(&evs_state, EVSTATE_MODE_SEL);
                }
            }
        }
        break;

    case EVSTATE_PLUG_CHECK:    // wait for plug_presence = plug_configuration [see: PlugManager]
        {
        if (pMsg->EvsMngEvent == EVS_AUTH_REQUIRED)                         		    // è stata passata una carta
            gsy_quick_polling_update((BUSY_OUTLET | RFID_PENDING), 1);
        else if ((pMsg->EvsMngEvent == EVS_AUTH_STOP) || (pMsg->EvsMngEvent == EVS_STATE_TIM_EXPIRED))
            evs_state_set(&evs_state, EVSTATE_PLUG_OUT);
        else if (pMsg->EvsMngEvent == EVS_PLUG_INSERTED)
            evs_state_set(&evs_state, EVSTATE_BLOCK_UP_DELAY);
        }
        break;

    case EVSTATE_BLOCK_UP_DELAY:    // wait timeout before drive block up
        {
        if (pMsg->EvsMngEvent == EVS_STATE_TIM_EXPIRED)
            evs_state_set(&evs_state, EVSTATE_SOCKET_CHECK);
        }
        break;

    case EVSTATE_SOCKET_CHECK:  // check socket safety condition
        {
        if (pMsg->EvsMngEvent == EVS_BLOCK_STEADY)
            {
            if (evs_error_array[0] & BLOCK_ANOM0)
                evs_state_set(&evs_state, EVSTATE_RES_ERROR);
            else if (evs_lid_error(PLUG_IN_CONSISTENCY, 1) == 1)
                evs_state_set(&evs_state, EVSTATE_CLOSE_LID);
            else if (plug_state_get() == PLUG_INSERTED) // && (block_state_get() == BLOCK_UP_STATE))
                evs_state_set(&evs_state, EVSTATE_M3S_M3T_DETECT);
            }
        else if ((pMsg->EvsMngEvent == EVS_PLUG_INSERTED) && (block_state_get() == BLOCK_UP_STATE))
            evs_state_set(&evs_state, EVSTATE_M3S_M3T_DETECT);
        else if (pMsg->EvsMngEvent == EVS_AUTH_REQUIRED)                         		    // è stata passata una carta
            gsy_quick_polling_update((BUSY_OUTLET | RFID_PENDING), 1);
        else if ((pMsg->EvsMngEvent == EVS_PULS_STOP) || (pMsg->EvsMngEvent == EVS_AUTH_STOP))
            evs_state_set(&evs_state, EVSTATE_PLUG_OUT);
        }
        break;

    case EVSTATE_CLOSE_LID:  // wait for lid closing: plug inserted
        {
        if (pMsg->EvsMngEvent == EVS_LID_UPDATE)
            evs_set_timer(EVS_STATE_TIM, EVS_LID_TIME);
        else if (pMsg->EvsMngEvent == EVS_STATE_TIM_EXPIRED)
            {
            if (evs_lid_error(PLUG_IN_CONSISTENCY, 1) == 1)
                evs_set_timer(EVS_STATE_TIM, EVS_LID_TIME);
            else
                evs_state_set(&evs_state, EVSTATE_SOCKET_CHECK);
            }
        }
        break;

    case EVSTATE_SUSPENDING: // forced suspension
        {
        if (pMsg->EvsMngEvent == EVS_POST_SUSPENDING_EXPIRED)
            post_suspending = 1;

        if (pMsg->EvsMngEvent == EVS_AUTH_REQUIRED)                         		    // è stata passata una carta
            gsy_quick_polling_update((BUSY_OUTLET | RFID_PENDING), 1);
        else if (pMsg->EvsMngEvent == EVS_AUTH_STOP)
            evs_state_set(&evs_state, EVSTATE_PLUG_OUT);
        else if (pMsg->EvsMngEvent == EVS_PULS_STOP)
            {
            if (evs_mode == EVS_FREE_MODE)
                evs_state_set(&evs_state, EVSTATE_PLUG_OUT);
            }
        else if ((post_suspending != 2) && ((suspending_set == SUSPENDING_NULL) || ((suspending_set_pen_save == 1) && ((suspending_set & PEN_SUSPENDING) == 0))))
            {
            if (suspending_set_pen_save == 1)
                {
                suspending_enable = 0;
                evs_set_timer(EVS_SUSPENDING_TIM, EVSTATE_WAITING_30S);
                suspending_enable_set = 1;
                }

            suspending_set_pen_save = 0;
            evs_state_set(&evs_state, EVSTATE_WAKEUP);
            }
        else if (pMsg->EvsMngEvent == EVS_LID_UPDATE)
            {
            if (evs_lid_error(NO_PLUG_CONSISTENCY, 1) == 1)
                evs_state_set(&evs_state, EVSTATE_LID_ERROR);
            }
        }
        break;

    case EVSTATE_WAKEUP:  // ev wakeup transition
        {
        if (pMsg->EvsMngEvent == EVS_STATE_TIM_EXPIRED)
            {
            if (pwm_low_output == 1)
                {
                pwm_low_output = 0;
                evs_set_timer(EVS_STATE_TIM, WAKEUP_STATE_B_TIME);
                send_to_pwm(PWM_OUTPUT_HIGH);
                }
            else
                {
                send_to_pwm(PWM_OUTPUT_DC_START);

                if ((evs_rtc_backup_get(BACKUP_CHARGE_STATUS) & M3S_CHARGE_SAVE) == M3S_CHARGE_SAVE)
                    {
                    if ((s2_state_get() == S2_STATE_D) || (s2_state_get() == S2_STATE_C))
                        charging_mode = M3S_CHARGING_MODE;
                    else
                        charging_mode = CHARGING_MODE_NULL;
                    }
                else if ((evs_rtc_backup_get(BACKUP_CHARGE_STATUS) & M3T_CHARGE_SAVE) == M3T_CHARGE_SAVE)
                  charging_mode = M3T_CHARGING_MODE;

                if (charging_mode == M3S_CHARGING_MODE)
                    {
                    eeprom_param_get(CONTROL_BYTE2_EADD, &control_enable, 1);
                    control = (control_enable & RECTIFIER_CRL2);

                    if (control == RECTIFIER_CRL2)
                        evs_state_set(&evs_state, EVSTATE_RECTIFIER_CHECK);
                    else
                        {
                        evs_state_set(&evs_state, EVSTATE_CHARGING);
                        mdb_ev_connectedd_hide = 0;
                        }
                    }
                else if (charging_mode == M3T_CHARGING_MODE)
                    {
                    eeprom_param_get(CONTROL_BYTE0_EADD, &control_enable, 1);
                    control = (control_enable & VENT_CRL0);
        
                    if ((s2_state_get() == S2_STATE_D) && (getInput(IN7_VENT_EXP0) == GPIO_PIN_SET) && (control == VENT_CRL0))
                        {
                        evs_error_set(CONTROL_BYTE_0, VENT_ANOM0, 1);
                        evs_state_set(&evs_state, EVSTATE_RES_ERROR);
                        }
                    else if ((s2_state_get() == S2_STATE_D) || (s2_state_get() == S2_STATE_C))
                        evs_state_set(&evs_state, EVSTATE_CHARGING);
                    else
                        evs_state_set(&evs_state, EVSTATE_S2_WAITING);
                    }
                else
                    evs_state_set(&evs_state, EVSTATE_M3S_M3T_DETECT);

                xp_short_enable_set(1);
                }
            }
        }
        break;

    case EVSTATE_M3S_M3T_DETECT:    // detect M3 standard / simplified
        {
        if ((pMsg->EvsMngEvent == EVS_S2_STATE_UPDATE) || (pen_charging_mode != CHARGING_MODE_NULL)
         || (charging_mode != CHARGING_MODE_NULL) || (evs_state_resume == EVSTATE_M3S_M3T_DETECT)
         || (pMsg->EvsMngEvent == EVS_ISO15118_INFO_DEVICE_UPDATE))
            {
            if (pMsg->EvsMngEvent != EVS_ISO15118_INFO_DEVICE_UPDATE)
                {
                eeprom_param_get(BATTERY_CONFIG_EADD, &control_enable, 1);
                
                if ((evs_rtc_backup_get(BACKUP_CHARGE_STATUS) & M3S_CHARGE_SAVE) == M3S_CHARGE_SAVE)    // recupero charging_mode da scenario salvato [asssenza tensione, fault, ecc.]
                    {
                    if ((s2_state_get() == S2_STATE_D) || (s2_state_get() == S2_STATE_C))
                        charging_mode = M3S_CHARGING_MODE;
                    }
                else if ((evs_rtc_backup_get(BACKUP_CHARGE_STATUS) & M3T_CHARGE_SAVE) == M3T_CHARGE_SAVE)    // recupero charging_mode da scenario salvato [asssenza tensione, fault, ecc.]
                    charging_mode = M3T_CHARGING_MODE;
                
                if (pen_charging_mode != CHARGING_MODE_NULL)    // recupero charging_mode da scenario salvato [asssenza tensione, fault, ecc.]
                    {
                    if ((pen_charging_mode == M3S_CHARGING_MODE) && (s2_state_get() == S2_STATE_B))
                        charging_mode = CHARGING_MODE_NULL;
                    else
                        charging_mode = pen_charging_mode;
                
                    pen_charging_mode = CHARGING_MODE_NULL;
                    }
                else if (charging_mode == CHARGING_MODE_NULL)    // determinazione charging_mode in assenza di scenario salvato
                    {
	                if ((s2_state_get() == S2_STATE_D) || (s2_state_get() == S2_STATE_C))   // ISO15118 sicuramente assente
	                    {
	                    charging_mode = M3S_CHARGING_MODE;                                  // set modalità di carica = Modo3 semplificato
	                    rtc_reg_save = evs_rtc_backup_get(BACKUP_CHARGE_STATUS);
	                    rtc_reg_save &=~ M3T_CHARGE_SAVE;
	                    rtc_reg_save |= (NULL_CHARGE_SAVE | M3S_CHARGE_SAVE);
	                    evs_rtc_backup_set(BACKUP_CHARGE_STATUS, rtc_reg_save);
	                    }
	                else    // if (s2_state_get() == S2_STATE_B)                            // ISO15118 potrebbe essere presente
	                    {
	                    charging_mode = M3T_CHARGING_MODE;                                  // set modalità di carica = Modo3 standard
	                    rtc_reg_save = evs_rtc_backup_get(BACKUP_CHARGE_STATUS);
	                    rtc_reg_save &=~ M3S_CHARGE_SAVE;
	                    rtc_reg_save |= (NULL_CHARGE_SAVE | M3T_CHARGE_SAVE);
	                    evs_rtc_backup_set(BACKUP_CHARGE_STATUS, rtc_reg_save);
	                    }
                    }
                }

            eeprom_param_get(CONTROL_BYTE0_EADD, &control_enable, 1);
            control = (control_enable & VENT_CRL0);

            if ((s2_state_get() == S2_STATE_D) && (getInput(IN7_VENT_EXP0) == GPIO_PIN_SET) && (control == VENT_CRL0))
                {
                evs_error_set(CONTROL_BYTE_0, VENT_ANOM0, 1);
                evs_state_set(&evs_state, EVSTATE_RES_ERROR);
                break;
                }
            else if ((evs_iso15118_run == 1) && (evs_iso15118_state == ISO15118_NULL))
                {
                pwm_iso15118_set();
                send_to_pwm(PWM_OUTPUT_DC_START);
                evs_state_next = EVSTATE_M3S_M3T_DETECT;
                evs_iso15118_data_req = ISO15118_CHARGING_START;
                evs_state_set(&evs_state, EVSTATE_ISO15118_WAIT);
                break;
                }
            else if ((evs_iso15118_run == 1) && (evs_iso15118_state == ISO15118_WAIT))
                break;
            else if ((evs_iso15118_run == 1) && (evs_iso15118_state == ISO15118_NOK))
                {
                evs_iso15118_run = 0;
                evs_iso15118_state = ISO15118_NULL;
                evs_state_set(&evs_state, EVSTATE_WAKEUP);
                break;
                }

            evs_iso15118_state = ISO15118_NULL;
            
            if (evs_iso15118_run == 0)
                send_to_pwm(PWM_OUTPUT_DC_START);

            eeprom_param_get(CONTROL_BYTE2_EADD, &control_enable, 1);
            control = (control_enable & RECTIFIER_CRL2);
            
            if (control == RECTIFIER_CRL2)
                evs_state_set(&evs_state, EVSTATE_RECTIFIER_CHECK);
            else if (charging_mode == M3S_CHARGING_MODE)
                {
                evs_state_set(&evs_state, EVSTATE_CONTACT_CLOSE_DELAY);
                mdb_ev_connectedd_hide = 0;
                }
            else    // if (charging_mode == M3T_CHARGING_MODE)
                {
                if ((mdb_suspending_ev_hide_enable == 1) && (evs_state_resume != EVSTATE_M3S_M3T_DETECT))
                    mdb_suspending_ev_hide = 1;
            
                mdb_suspending_ev_hide_enable = 1;                
                mdb_ev_connectedd_hide = 0;
            
                evs_state_set(&evs_state, EVSTATE_S2_WAITING);
                }
            }
        }
        break;

    case EVSTATE_ISO15118_WAIT:
        {
        if (pMsg->EvsMngEvent == EVS_ISO15118_INFO_DEVICE_UPDATE)
            {
            switch (evs_iso15118_data_req)
                {
                case ISO15118_CHARGING_STOP:
                case ISO15118_CHARGING_START:
                case ISO15118_CHARGING_PAUSE:
                    evs_iso15118_info = ISO15118_Info_Device.serviceIdReq;
                    break;
                case ISO15118_V2GSEQUENCEEND:
                    evs_iso15118_info = ISO15118_Info_Device.State_openv2g;
                    break;
                }

            if (evs_iso15118_info == evs_iso15118_data_req)
                {
                evs_iso15118_tim_enable = 0;
                evs_state = evs_state_next;
                evs_state_next = EVSTATE_NULL;
                evs_iso15118_state = ISO15118_OK;
                send_to_evs(EVS_ISO15118_INFO_DEVICE_UPDATE);
                }
            }
        else if (pMsg->EvsMngEvent == EVS_ISO15118_TIM_EXPIRED)
            {
            evs_iso15118_num_test ++;

            if (evs_iso15118_num_test >= EVS_ISO15118_TEST_MAX)
                {
                evs_iso15118_tim_enable = 0;
                evs_iso15118_num_test = 0;
                evs_iso15118_run = 0;
                pwm_iso15118_clr();
                evs_state = evs_state_next;
                evs_state_next = EVSTATE_NULL;
                evs_iso15118_state = ISO15118_NOK;
                send_to_evs(EVS_ISO15118_INFO_DEVICE_UPDATE);
                }
            else    /*  RIVEDERE */ // gestire tentativi di comunicazione successivi
                evs_set_timer(EVS_ISO15118_TIM, ISO15118_WAIT_TIME);
            }
        }
        break;
    
    case EVSTATE_RECTIFIER_CHECK:   // check rectifier presence
        {
        if (pMsg->EvsMngEvent == EVS_RECT_UPDATE)
            {
            pilot_anom2_update();

            if (evs_error_array[2] & RECTIFIER_ANOM2)
                evs_state_set(&evs_state, EVSTATE_RES_ERROR);
            else if (charging_mode == M3S_CHARGING_MODE)
                evs_state_set(&evs_state, EVSTATE_CONTACT_CLOSE_DELAY);
            else    // if (charging_mode == M3T_CHARGING_MODE)
                {
                if ((mdb_suspending_ev_hide_enable == 1) && (evs_state_resume != EVSTATE_M3S_M3T_DETECT))
                    mdb_suspending_ev_hide = 1;

                mdb_suspending_ev_hide_enable = 1;                
                evs_state_set(&evs_state, EVSTATE_S2_WAITING);
                }

            rect_enable_set(0);
            }
        }
        break;

    case EVSTATE_S2_WAITING:  // wait for S2 closing
        {
          
        if (((evs_iso15118_run == 1)/* && (evs_iso15118_count == 0)*/)
         && ((ISO15118_Info_Device.State_openv2g == ISO15118_V2GSEQUENCEPAUSE) || (ISO15118_Info_Device.State_openv2g == ISO15118_V2GSEQUENCEEND))
         && (is_pwm_CP_Active() == TRUE))
            send_to_pwm(PWM_OUTPUT_HIGH);
        
        eeprom_param_get(CONTROL_BYTE0_EADD, &control_enable, 1);
        control = (control_enable & VENT_CRL0);

        if (pMsg->EvsMngEvent == EVS_AUTH_REQUIRED)                         		    // è stata passata una carta
            gsy_quick_polling_update((BUSY_OUTLET | RFID_PENDING), 1);
        else if (pMsg->EvsMngEvent == EVS_AUTH_STOP)
            evs_state_set(&evs_state, EVSTATE_PLUG_OUT);
        else if (pMsg->EvsMngEvent == EVS_PULS_STOP)
            {
            if (evs_mode == EVS_FREE_MODE)
                evs_state_set(&evs_state, EVSTATE_PLUG_OUT);
            }
        else if ((pMsg->EvsMngEvent == EVS_S2_STATE_UPDATE) || (evs_state_resume == EVSTATE_M3S_M3T_DETECT) || (pMsg->EvsMngEvent == EVS_ISO15118_TIM_EXPIRED))
            {
            if ((plug_state_get() == PLUG_INSERTED) || (evs_state_resume == EVSTATE_M3S_M3T_DETECT) || (pMsg->EvsMngEvent == EVS_ISO15118_TIM_EXPIRED))
                {
                if ((s2_state_get() == S2_STATE_D) && (getInput(IN7_VENT_EXP0) == GPIO_PIN_SET) && (control == VENT_CRL0))
                    {
                    evs_error_set(CONTROL_BYTE_0, VENT_ANOM0, 1);
                    evs_state_set(&evs_state, EVSTATE_RES_ERROR);
                    }
                else if ((evs_iso15118_run == 1) && (evs_iso15118_s2 == 1))
                    {
                    evs_iso15118_count ++;
                    evs_iso15118_tim_enable = 1;
                    
                    if ((evs_iso15118_count == 3) || (pMsg->EvsMngEvent == EVS_ISO15118_TIM_EXPIRED))
                        evs_state_set(&evs_state, EVSTATE_ISO15118_WAKEUP);
                     else
                        evs_set_timer(EVS_ISO15118_TIM, ISO15118_COUNT_TIME);
                    }
                else if ((s2_state_get() == S2_STATE_D) || (s2_state_get() == S2_STATE_C))
                    {
                    eeprom_param_get(POST_SUSP_TIME_EADD, &val, 1);
                    
                    if ((val > 0) && (val <= 9) && (post_suspending == 0))  // 15/05/25 Nick: portato a 9 
                        {
                        if (post_suspending_num == 1)
                            post_suspending_num --;
                        else if ((val > 0) && (val <= 9) && (post_suspending == 0)) // 15/05/25 Nick: portato a 9
                            {
                            eeprom_param_get(POST_SUSP_TIME_EADD, (uint8_t*)&val, 1);
                        
                            post_suspending_time = (uint32_t)(val) * EVSTATE_WAITING_10S;
                            evs_set_timer(EVS_POST_SUSPENDING_TIM, post_suspending_time);
                            }
                        }

                    evs_state_set(&evs_state, EVSTATE_CHARGING);
                    }
                else if (evs_state_resume == EVSTATE_M3S_M3T_DETECT)    // (s2_state_get() == S2_STATE_B)
                    evs_set_timer(EVS_STATE_TIM, EVSTATE_WAITING_30S);
                }
            else
                {
                }

            evs_state_resume = EVSTATE_NULL;
            }
        else if (pMsg->EvsMngEvent == EVS_STATE_TIM_EXPIRED)
            {
            if (ev_suspending == 0)
                evs_state_set(&evs_state, EVSTATE_WAKEUP);
            }
        else if (pMsg->EvsMngEvent == EVS_SUSPENDING_TIM_EXPIRED)
            {
            suspending_enable = 1;

            if (suspending_request == REMOTE_SUSPENDING)
                {
                suspending_set = suspending_request;
                evs_state_set(&evs_state, EVSTATE_SUSPENDING);
                }
            }
        else if (pMsg->EvsMngEvent == EVS_REMOTE_SUSPENDING)
            {
            if (suspending_enable == 1)
                evs_state_set(&evs_state, EVSTATE_SUSPENDING);
            }
        else if (((evs_rtc_backup_get(BACKUP_CHARGE_STATUS) & M3S_CHARGE_SAVE) == M3S_CHARGE_SAVE) || ((evs_rtc_backup_get(BACKUP_CHARGE_STATUS) & M3T_CHARGE_SAVE) == M3T_CHARGE_SAVE))
            {
            if ((s2_state_get() == S2_STATE_D) && (getInput(IN7_VENT_EXP0) == GPIO_PIN_SET) && (control == VENT_CRL0))
                {
                evs_error_set(CONTROL_BYTE_0, VENT_ANOM0, 1);
                evs_state_set(&evs_state, EVSTATE_RES_ERROR);
                }
            else if ((evs_iso15118_s2 == 0) && ((s2_state_get() == S2_STATE_D) || (s2_state_get() == S2_STATE_C)))
                evs_state_set(&evs_state, EVSTATE_CHARGING);
            }
        }
        break;

    case EVSTATE_ISO15118_WAKEUP:
        {
        if (pMsg->EvsMngEvent == EVS_ISO15118_TIM_EXPIRED)
            {
            if (iso15118_wakeup_step == 0)
                {
                send_to_pwm(PWM_OUTPUT_HIGH);
                evs_iso15118_tim_enable = 1;
                evs_set_timer(EVS_ISO15118_TIM, ISO15118_B_TIME);
                }
            else
                {
                xp_short_enable_set(1);
                evs_state_set(&evs_state, EVSTATE_M3S_M3T_DETECT);
                }
            
            iso15118_wakeup_step ++;
            }
        }
        break;
    
    case EVSTATE_CONTACT_CLOSE_DELAY:   // wait timeout before drive contact close
        {
        if (pMsg->EvsMngEvent == EVS_STATE_TIM_EXPIRED)
            evs_state_set(&evs_state, EVSTATE_CHARGING);
        }
        break;

    case EVSTATE_CHARGING:  // EV in charge
        {
        if (initEmParameter_enb == 1)
            initEmParameter(EVT_START_ENRG_ACT_SESS_MEAS);

        initEmParameter_enb = 0;

        if (pMsg->EvsMngEvent == EVS_OVERCURRENT_ERROR)
            {
                {
                setLed(LED_C_RED, LED_EVENT_BLINK, (uint16_t)(1000), LED_RED_DEFAULT);
                evs_error_set(CONTROL_BYTE_1, OVERCURRENT_ANOM1, 1);
                evs_state_set(&evs_state, EVSTATE_INTERRUPTING);
                post_suspending = 0;
                post_suspending_num = 1;
                }
            }
        else if (pMsg->EvsMngEvent == EVS_AUTH_REQUIRED)                         		    // è stata passata una carta
            gsy_quick_polling_update((BUSY_OUTLET | RFID_PENDING), 1);
        else if ((pMsg->EvsMngEvent == EVS_AUTH_STOP) || ((evs_mode == EVS_FREE_MODE) && (pMsg->EvsMngEvent == EVS_PULS_STOP))
              || (pMsg->EvsMngEvent == EVS_ISO15118_INFO_DEVICE_UPDATE) && (evs_iso15118_state != ISO15118_NULL))
            {
            if ((pMsg->EvsMngEvent == EVS_AUTH_STOP) || ((evs_mode == EVS_FREE_MODE) && (pMsg->EvsMngEvent == EVS_PULS_STOP)))
                {
                post_suspending = 0;
                post_suspending_num = 1;
                suspending_set = SUSPENDING_NULL;
                }
                
            if ((evs_iso15118_run == 1) && (evs_iso15118_state == ISO15118_NULL))
                {
                ISO15118_homeplugdev_info_host_set(ISO15118_STOPEVSE, 1);
                evs_state_next = EVSTATE_CHARGING;
                evs_iso15118_data_req = ISO15118_CHARGING_STOP;
                evs_state_set(&evs_state, EVSTATE_ISO15118_WAIT);
                break;
                }
            else if ((evs_iso15118_run == 1) && (evs_iso15118_state == ISO15118_OK) && (contact_state_get() == CONTACT_STATE_CLOSE))
                {
                send_to_contact(CONTACT_OPEN_REQ);
                evs_state_next = EVSTATE_CHARGING;
                evs_iso15118_data_req = ISO15118_V2GSEQUENCEEND;
                evs_state_set(&evs_state, EVSTATE_ISO15118_WAIT);
                break;
                }
            else
            	{
            	evs_iso15118_state = ISO15118_NULL;
                evs_state_set(&evs_state, EVSTATE_INTERRUPTING);
                }
            }
        else if ((suspending_set != SUSPENDING_NULL) || ((pMsg->EvsMngEvent == EVS_POST_SUSPENDING_EXPIRED) && (post_suspending == 0)))
            {
            suspending_save = 1;

            if (pMsg->EvsMngEvent == EVS_POST_SUSPENDING_EXPIRED)
                post_suspending = 1;
            else
               post_suspending = 0;

            evs_state_set(&evs_state, EVSTATE_INTERRUPTING);
            }
        else if (pMsg->EvsMngEvent == EVS_VENT_UPDATE)
            {
            eeprom_param_get(CONTROL_BYTE0_EADD, &control_enable, 1);
            control = (control_enable & VENT_CRL0);

            if ((s2_state_get() == S2_STATE_D) && (getInput(IN7_VENT_EXP0) == GPIO_PIN_SET) && (control == VENT_CRL0))
                {
                post_suspending = 0;
                post_suspending_num = 1;
                evs_error_set(CONTROL_BYTE_0, VENT_ANOM0, 1);
                evs_state_set(&evs_state, EVSTATE_INTERRUPTING);
                }
            }
        else if (pMsg->EvsMngEvent == EVS_S2_STATE_UPDATE)
            {
            eeprom_param_get(CONTROL_BYTE0_EADD, &control_enable, 1);
            control = (control_enable & VENT_CRL0);

            if ((s2_state_get() == S2_STATE_D) && (getInput(IN7_VENT_EXP0) == GPIO_PIN_SET) && (control == VENT_CRL0))
                {
                evs_error_set(CONTROL_BYTE_0, VENT_ANOM0, 1);
                evs_state_set(&evs_state, EVSTATE_INTERRUPTING);
                }
            else if (s2_state_get() == S2_STATE_B)
                {
                ev_suspending = 1;
                charging_mode = M3T_CHARGING_MODE;                                  // set modalità di carica = Modo3 standard

                eeprom_param_get(BATTERY_CONFIG_EADD, &control_enable, 1);

                post_suspending = 0;
                evs_iso15118_s2 = 1;
                evs_state_set(&evs_state, EVSTATE_S2_WAITING);
                }
            }
        else if (pMsg->EvsMngEvent == EVS_SUSPENDING_TIM_EXPIRED)
            suspending_enable = 1;
        }
        break;

    case EVSTATE_INTERRUPTING:   // wait for S2 opening or timeout
        {
        if (((pMsg->EvsMngEvent == EVS_S2_STATE_UPDATE) && (s2_state_get() == S2_STATE_B))
          || (pMsg->EvsMngEvent == EVS_STATE_TIM_EXPIRED)
          || (pMsg->EvsMngEvent == EVS_ISO15118_INFO_DEVICE_UPDATE))
            {
            if (pen_state_set == 1)
                evs_state_set(&evs_state, EVSTATE_PEN_STATE);
            else
                {
                if ((s2_state_get() == S2_STATE_B) && (pMsg->EvsMngEvent == EVS_S2_STATE_UPDATE))
                    charging_mode = M3T_CHARGING_MODE;                                  // set modalità di carica = Modo3 standard
            
                eeprom_param_get(BATTERY_CONFIG_EADD, &control_enable, 1);
            
                if (evs_error_get(error_array, 1, 1, 6) == 1)
                    evs_state_set(&evs_state, EVSTATE_RES_ERROR);
                else if ((suspending_set != SUSPENDING_NULL) || (suspending_save == 1) || (post_suspending == 1))
                    {
                    if (post_suspending == 1)
                        {
                        post_suspending = 2;
                        evs_set_timer(EVS_POST_SUSPENDING_TIM, EVSTATE_WAITING_10S);
                        }
            
                    evs_state_set(&evs_state, EVSTATE_SUSPENDING);
                    }
                else
                    evs_state_set(&evs_state, EVSTATE_BLOCK_DOWN_DELAY);
                }
            }
        else if (pMsg->EvsMngEvent == EVS_PLUG_OUT)
            evs_set_timer(EVS_STATE_TIM, EVS_SEC_TIME);
        }
        break;

    case EVSTATE_BLOCK_DOWN_DELAY:    // wait timeout before drive block down
        {
        if (pMsg->EvsMngEvent == EVS_STATE_TIM_EXPIRED)
            evs_state_set(&evs_state, EVSTATE_PLUG_OUT);
        }
        break;

    case EVSTATE_F_STATE_ERROR:
        {
        if (pMsg->EvsMngEvent == EVS_STATE_TIM_EXPIRED)
            evs_state_set(&evs_state, EVSTATE_X1_STATE_ERROR);
        }
        break;

    case EVSTATE_X1_STATE_ERROR:
        {
        if (pMsg->EvsMngEvent == EVS_STATE_TIM_EXPIRED)
            {
            evs_set_timer(EVS_STATE_TIM, F_X1_RES_ERROR_TIME);
//            send_to_pwm(PWM_OUTPUT_HIGH);
            xp_short_enable_set(1);
            xp_presence_enable_set(1);
            evs_state = EVSTATE_RES_ERROR;
            }
        }
        break;

    case EVSTATE_RES_ERROR:  // show evs error
        {
        if (pMsg->EvsMngEvent == EVS_STATE_TIM_EXPIRED)
            {
            if (plug_state_get() == PLUG_NOT_PRESENT)
                {
                evs_gost_plug = 0;
                evs_error_update();

                if ((evs_mode > EVS_FREE_MODE) && (evs_lid_error(NO_PLUG_CONSISTENCY, 1) == 1))
                    evs_state_set(&evs_state, EVSTATE_LID_ERROR);
                else if (suspending_set & PEN_SUSPENDING)
                    evs_state_set(&evs_state, EVSTATE_PEN_STATE);
                else if (evs_error_get(error_array, 1, 1, 6) == 1)
                    evs_state_set(&evs_state, EVSTATE_ERROR_WAIT);
                else
                    {
                    if (snp3_error_save != 0)
                        send_to_energy(ENERGY_INTERNAL_EM_GOOD);

                    gsy_quick_polling_update(BUSY_OUTLET, 0);
                    evs_state_set(&evs_state, EVSTATE_MODE_SEL);
                    }

                send_to_pers(PERS_UID_RELEASE);
                }
            else
                evs_state_set(&evs_state, EVSTATE_PLUG_OUT);
            }
        }
        break;

    case EVSTATE_PLUG_OUT:  // wait for EVS_PLUG_OUT
        {
        if (pMsg->EvsMngEvent == EVS_STATE_TIM_EXPIRED)
        	send_to_block(BLOCK_DOWN_REQ);
        else if (pMsg->EvsMngEvent == EVS_PLUG_OUT)
            {
            evs_gost_plug = 0;
            gsy_quick_polling_update(PLUGGED_OUTLET, 0);
            evs_error_update();

            if ((evs_mode > EVS_FREE_MODE) && (evs_lid_error(NO_PLUG_CONSISTENCY, 0) == 1))
                evs_state_set(&evs_state, EVSTATE_LID_ERROR_DELAY);
            else if (suspending_set & PEN_SUSPENDING)
                {
                evs_state_set(&evs_state, EVSTATE_PEN_STATE);

                if (evs_mode == EVS_FREE_MODE)
                    evs_state_resume = EVSTATE_SOCKET_AVAILABLE;
                else
                    evs_state_resume = EVSTATE_AUTH_WAIT;
                }
            else if (evs_error_get(error_array, 1, 1, 6) == 1)
                evs_state_set(&evs_state, EVSTATE_ERROR_WAIT);
            else
                {
                if (snp3_error_save != 0)
                    send_to_energy(ENERGY_INTERNAL_EM_GOOD);

                gsy_quick_polling_update(BUSY_OUTLET, 0);
                evs_state_set(&evs_state, EVSTATE_MODE_SEL);
                }

            send_to_pers(PERS_UID_RELEASE);
            }
        }
        break;

    case EVSTATE_LID_ERROR_DELAY:
        {
        if (pMsg->EvsMngEvent == LID_ERROR_DELAY_TIM_EXPIRED)
            {
            if ((evs_mode > EVS_FREE_MODE) && (evs_lid_error(NO_PLUG_CONSISTENCY, 1) == 1))
                evs_state_set(&evs_state, EVSTATE_LID_ERROR);
            else if (suspending_set & PEN_SUSPENDING)
                {
                evs_state_set(&evs_state, EVSTATE_PEN_STATE);

                if (evs_mode == EVS_FREE_MODE)
                    evs_state_resume = EVSTATE_SOCKET_AVAILABLE;
                else
                    evs_state_resume = EVSTATE_AUTH_WAIT;
                }
            else if (evs_error_get(error_array, 1, 1, 6) == 1)
                evs_state_set(&evs_state, EVSTATE_ERROR_WAIT);
            else
                {
                if (snp3_error_save != 0)
                    send_to_energy(ENERGY_INTERNAL_EM_GOOD);

                gsy_quick_polling_update(BUSY_OUTLET, 0);
                evs_state_set(&evs_state, EVSTATE_MODE_SEL);
                }
            }
        }
        break;

    case EVSTATE_PAUT_LID:
        {
        if (pMsg->EvsMngEvent == EVS_LID_UPDATE)
            {
            evs_gost_lid = 0;

            if (evs_error_get(error_array, 1, 1, 6) == 1)
                evs_state_set(&evs_state, EVSTATE_ERROR_WAIT);
            else
                evs_state_set(&evs_state, EVSTATE_MODE_SEL);
            }
        else if (pMsg->EvsMngEvent == EVS_AUTH_NEG)                                 // è stata negata l'autorizzazione alla ricarica
            {
            evs_gost_lid = 0;
            evs_state_set(&evs_state, EVSTATE_AUTH_NEG);
            }
        else if (pMsg->EvsMngEvent == EVS_AUTH_START)                               // è stata autorizzata la ricarica
            {
            evs_gost_lid = 0;
            evs_state_set(&evs_state, EVSTATE_PLUG_WAIT);
            }
        else if ((pMsg->EvsMngEvent == EVS_PLUG_DETECTED) || (pMsg->EvsMngEvent == EVS_PLUG_INSERTED) || (plug_state_get() != PLUG_NOT_PRESENT))
            {
            evs_gost_lid = 0;
            evs_gost_plug = 1;
            evsModbusState_force = 1;
            evs_state_set(&evs_state, EVSTATE_AUTH_PENDING);
            send_to_lcd(LCD_CARD_PENDING);
            }
        else if (pMsg->EvsMngEvent == EVS_SEC_TIM_EXPIRED)
            {
            evs_gost_lid = 0;

            if ((evs_mode > EVS_FREE_MODE) && (evs_lid_error(NO_PLUG_CONSISTENCY, 1) == 1))
                evs_state_set(&evs_state, EVSTATE_LID_ERROR);
            else if (suspending_set & PEN_SUSPENDING)
                {
                evs_state_set(&evs_state, EVSTATE_PEN_STATE);

                if (evs_mode == EVS_FREE_MODE)
                    evs_state_resume = EVSTATE_SOCKET_AVAILABLE;
                else
                    evs_state_resume = EVSTATE_AUTH_WAIT;
                }
            else if (evs_error_get(error_array, 1, 1, 6) == 1)
                evs_state_set(&evs_state, EVSTATE_ERROR_WAIT);
            else
                {
                if (snp3_error_save != 0)
                    send_to_energy(ENERGY_INTERNAL_EM_GOOD);

                gsy_quick_polling_update(BUSY_OUTLET, 0);
                evs_state_set(&evs_state, EVSTATE_MODE_SEL);
                }
            }
        }
        break;

    case EVSTATE_AUTH_NEG:      // authorization denied
    case EVSTATE_ERROR_WAIT:    // wait for resettable error clear
        {
        if (wifi_antenna_error == 1)
            break;

        if (pMsg->EvsMngEvent == EVS_AUTH_START)                                // è stata autorizzata la ricarica
            {
            if (evs_mode != EVS_FREE_MODE)
                {
                evs_error_update();
                evs_error_get(error_array, 0, 1, 6);
            
                if (error_array[1] == MIFARE_ANOM1)                             // mifare_error_hide = 1
                    evs_state_set(&evs_state, EVSTATE_PLUG_WAIT);
                }
            }
        else if (pMsg->EvsMngEvent == EVS_LID_UPDATE)
            {
            if ((evs_mode > EVS_FREE_MODE) && (evs_lid_error(NO_PLUG_CONSISTENCY, 1) == 1))
                evs_state_set(&evs_state, EVSTATE_LID_ERROR);
            }
        else if ((pMsg->EvsMngEvent == ERROR_WAIT_TIM_EXPIRED) || (pMsg->EvsMngEvent == EVS_AUTORIZATION_MODE))
            {
            if (pMsg->EvsMngEvent == EVS_AUTORIZATION_MODE)
                evs_mode = gsy_evs_mode_get();

            if (plug_state_get() != PLUG_NOT_PRESENT)
                evs_state_set(&evs_state, EVSTATE_PLUG_OUT);
            else if (evs_lid_error(NO_PLUG_CONSISTENCY, 1) == 1)
                evs_state_set(&evs_state, EVSTATE_LID_ERROR);
            else
                {
                evs_error_update();
    
                if (evs_mode == EVS_FREE_MODE)
                    control_enable = 1;             // mifare_error_hide = 1
                else
                    control_enable = 0;             // mifare_error_hide = 0
                
                if (evs_error_get(error_array, control_enable, 1, 0) == 0)
                    {
                    if (snp3_error_save != 0)
                        send_to_energy(ENERGY_INTERNAL_EM_GOOD);
    
                    evs_state_set(&evs_state, EVSTATE_MODE_SEL);
                    }
                else if (evs_error_array[0] & RCBO_ANOM0)
                    sendDiffRiarmMsg(DIFF_RIARM_EV_SET_ON);
                }
            }
        }
        break;

    case EVSTATE_PEN_STATE:
        {
        if (pMsg->EvsMngEvent == EVS_AUTH_REQUIRED)                         		    // è stata passata una carta
            gsy_quick_polling_update((BUSY_OUTLET | RFID_PENDING), 1);
        else if (pMsg->EvsMngEvent == EVS_AUTH_STOP)
            evs_state_set(&evs_state, EVSTATE_PLUG_OUT);
        else if (pMsg->EvsMngEvent == EVS_PULS_STOP)
            {
            if (evs_mode == EVS_FREE_MODE)
                evs_state_set(&evs_state, EVSTATE_PLUG_OUT);
            }
        else if (pMsg->EvsMngEvent == EVS_PEN_ALM_OFF)
            {
            suspending_set &=~ PEN_SUSPENDING;

            if (evs_state_resume != EVSTATE_NULL)
              {
              if (plug_presence_get() == 0)
                  {
                  evs_rtc_backup_set(BACKUP_CHARGE_STATUS, AUTH_RESET_SAVE);
                  evs_state_set(&evs_state, EVSTATE_MODE_SEL);
                  }
              else if (evs_state_resume == EVSTATE_S2_WAITING)
                evs_state_resume = EVSTATE_WAKEUP;
              else if (evs_state_resume == EVSTATE_INIT)
                send_to_evs(EVS_BLOCK_STEADY);

              evs_state_set(&evs_state, evs_state_resume);
              }

            evs_state_resume = EVSTATE_NULL;
            send_to_rfid(RFID_CONTROL_START);
            evs_error_set(CONTROL_BYTE_1, VBUS_ANOM1, 0);
            }
        else if (pMsg->EvsMngEvent == EVS_LID_UPDATE)
            {
            if ((evs_mode > EVS_FREE_MODE) && (evs_lid_error(NO_PLUG_CONSISTENCY, 1) == 1))
                evs_state_set(&evs_state, EVSTATE_LID_ERROR);
            }
        }
        break;

    case EVSTATE_V230_SUSPEND:
    case EVSTATE_POWER_OFF:
        {
        if (pMsg->EvsMngEvent == EVS_V230_ON)
            {
            if (evs_state_resume != EVSTATE_NULL)
              {
              if ((evs_state_resume == EVSTATE_CHARGING) || (evs_state_resume == EVSTATE_S2_WAITING) || (evs_state_resume == EVSTATE_SUSPENDING) || (evs_state_resume == EVSTATE_PLUG_OUT))
                {
                xp_short_enable_set(1);
                xp_presence_enable_set(1);
                xp_presence_freeze_set(0);

                if ((evs_state_resume == EVSTATE_CHARGING) || (evs_state_resume == EVSTATE_S2_WAITING))
                    {
                    send_to_pwm(PWM_OUTPUT_DC_START);
                    charging_mode = CHARGING_MODE_NULL;
                    mdb_ev_connectedd_hide = 1;
                    
                    if (evs_state_resume != EVSTATE_S2_WAITING)
                        ev_suspending = 0;

                    evs_state_resume = EVSTATE_M3S_M3T_DETECT;
                    }
              }

              if ((evs_state_resume == EVSTATE_SOCKET_AVAILABLE) || (evs_state_resume == EVSTATE_AUTH_WAIT) || (evs_state_resume == EVSTATE_DISABLED))
                evs_state_resume = EVSTATE_MODE_SEL;
              
              evs_state_set(&evs_state, evs_state_resume);
              }

            send_to_evs(EVS_MIRROR_READ);
            send_to_rfid(RFID_CONTROL_START);
            sendDiffRiarmMsg(DIFF_RIARM_EVENT_POLLING);
            evs_error_set(CONTROL_BYTE_1, VBUS_ANOM1, 0);
            }
        }
        break;

    default:
        break;
    }

if (evs_state_old != evs_state)
{
  if (getScuTypeMode() != SCU_SEM_STAND_ALONE)
  {
    setTransactionParam((sck_measures_t *)&measureSck, evs_state, 0, suspending_set);
    EVLOG_Message (EV_INFO, "EVS state changed to %s", evState_str[evs_state]);
  }
}

evs_state_old = evs_state; 
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  EvsMngTask
//
//  DESCRIPTION:    gestione macchina a stati della stazione
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void EvsMngTask(void *pvParameters)
{
uint8_t i;

/* init task */

/*-------- Creates an empty mailbox for EvsMngTask messages --------------------------*/
EvsMngQueue = xQueueCreate(16, sizeof(EvsMngMsg_st));
configASSERT(EvsMngQueue != NULL);

/*-------- Creates all timer for EvsMngTask ------------------------------------------*/
/* in this case we use one shot features */
for (i = 0; i < EVS_NUM_TIMER; i++)
    {
    xEvsMngTimers[i] = xTimerCreate("TimEvsMng", portMAX_DELAY, pdFALSE, (void*)(i), EvsMngTimCallBack);
    configASSERT(xEvsMngTimers[i] != NULL);
    }

EvsManager_init();

/* start task */

for (;;)
    {
    /* Wait for some event from SW */
    if (xQueueReceive(EvsMngQueue, (void *)&EvsMngMsg, portMAX_DELAY) == pdPASS)
        {
        EvsManager(&EvsMngMsg);
        }
    else
        {
        /* Wait for possible handled timeout */
        }
    }
}


/**
*
* @brief        put led string in color for uploading phase    
*
* @param [in]   none
*  
* @note          
*
* @retval       none
*
***********************************************************************************************************************/
void ledColorInUplaod(void)
{
  setLed(LED_B_GREEN, LED_EVENT_ON, (uint16_t)(0), LED_GREEN_DEFAULT);
  setLed(LED_A_BLU, LED_EVENT_ON, (uint16_t)(0), LED_BLUE_DEFAULT);
  setLed(LED_C_RED, LED_EVENT_ON, (uint16_t)(0), LED_RED_DEFAULT);
}

/**
*
* @brief        put led string in warning color for packet error    
*
* @param [in]   none
*  
* @note          
*
* @retval       none
*
***********************************************************************************************************************/
void ledColorPacketErrorUplaod(void)
{
  setLed(LED_B_GREEN, LED_EVENT_ON, (uint16_t)(0), LED_GREEN_DEFAULT);
  setLed(LED_A_BLU, LED_EVENT_OFF, (uint16_t)(0), LED_BLUE_DEFAULT);
  setLed(LED_C_RED, LED_EVENT_ON, (uint16_t)(0), LED_RED_DEFAULT);
}

/**
*
* @brief        put led string in alarm color for download fail    
*
* @param [in]   none
*  
* @note          
*
* @retval       none
*
***********************************************************************************************************************/
void ledColorUplaodFail(void)
{
  setLed(LED_B_GREEN, LED_EVENT_OFF, (uint16_t)(0), LED_GREEN_DEFAULT);
  setLed(LED_A_BLU, LED_EVENT_OFF, (uint16_t)(0), LED_BLUE_DEFAULT);
  setLed(LED_C_RED, LED_EVENT_ON, (uint16_t)(0), LED_RED_DEFAULT);
}

/**
*
* @brief        put led string in alarm color for download fail    
*
* @param [in]   none
*  
* @note          
*
* @retval       none
*
***********************************************************************************************************************/
void ledColorUplaodOk(void)
{
  setLed(LED_B_GREEN, LED_EVENT_OFF, (uint16_t)(0), LED_GREEN_DEFAULT);
  setLed(LED_A_BLU, LED_EVENT_ON, (uint16_t)(0), LED_BLUE_DEFAULT);
  setLed(LED_C_RED, LED_EVENT_OFF, (uint16_t)(0), LED_RED_DEFAULT);
}


/**
*
* @brief        Function to return the last / current state stable        
*
* @param [in ]  none 
*  
* @param [out]  none
*  
* @retval       evs_state_en: last state stable
*  
***********************************************************************************************************************/
evs_modbus_state_en getLastEvsStableStatus(void)
{
  return(evsModbusState);
}


/**
*
* @brief        Set led status for reserved condition     
*
* @param [in]   none 
*
* @retval       none
*
***********************************************************************************************************************/
void setLedForReservedMode (void)
{
//  setLed(LED_C_RED, LED_EVENT_ON, (uint16_t)(0), LED_RED_DEFAULT);
  setLed(LED_C_RED, LED_EVENT_BLINK, (uint16_t)(1000), LED_BLUE_DEFAULT);

  setLed(LED_B_GREEN, LED_EVENT_OFF, (uint16_t)(0), LED_GREEN_DEFAULT);

//  setLed(LED_A_BLU, LED_EVENT_ON, (uint16_t)(0), LED_BLUE_DEFAULT);
  setLed(LED_A_BLU, LED_EVENT_BLINK, (uint16_t)(1000), LED_BLUE_DEFAULT);
}


/**
  * @brief  set and start to use a new value for "wait for connector" 
  *         
  * @param  uint32_t: new timeout value 
  * 
  * @retval none
  */
void  setWaitTimeValue(uint16_t timeValue)
{
  sem_wait_time = (uint32_t)timeValue * (uint32_t)1000;
  sem_time_valid = TRUE;
}

#ifndef HW_MP28947
/**
  * @brief  set ISO15118 StateCP accordingly to evs state
  *         
  * @param  none
  * 
  * @retval stateCP used in the info struct of ISO15118
  */

//ccs2_cp_state_t ISO15118_get_StateCP (void)
//{
//  
//  /* Check CP state */
//  switch (s2_state_get())
//  {
//    case S2_STATE_A:
//      return ccs2StateA;   /* EV not connected to the charging station */
//        
//    case S2_STATE_B:
//    case S2_STATE_Bx:
//      if (is_pwm_CP_Active() == FALSE)
//        return ccs2StateB1;  /* EV is connected to the charging station without PWM active */
//      else
//        return ccs2StateB2;  /* EV is connected to the charging station with PWM active */
//      
//    case S2_STATE_C:
//      return ccs2StateC2;  /* EV is connected to the station and charging (no ventilation required) */    
//      
//    default:
//      break;
//  } 
//
//  /* CP not in A/B/C state, check now EVS state to see if it's in fault or unavailable */
//  switch (evsModbusState)
//  {
//    case FAULTED_STATE:
//    case UNVAILABLE_STATE:
//      return ccs2StateF;  /* Charging station is not available */
//  }
//  
//  return ccs2StateA;
//  
//}

///**
//  * @brief  set ISO15118 stopEVSE accordingly 
//  *         
//  * @param  none
//  * 
//  * @retval TRUE if stop EVSE is required
//  */
//
//uint8_t ISO15118_get_StopEVSE (void)
//{
//  
//  switch (evsModbusState)
//  {
//    case MDBSTATE_SUSPENDING_EVSE:
//    case MDBSTATE_END_CHARGE:
//      return TRUE;
//      
//    default:
//      return FALSE;
//    
//  }
//  
//  
//}

#else

/**    Check if EVcc IS is NULL 
*
* @brief       
*
* @param [in]  none
*  
* @retval      true/false 
*  
****************************************************************/

uint8_t is_EvccId_NULL (void)
{
    
  return true;   /* is NULL */
  
}

/**
*
* @brief       
*
* @param [in]  None
*  
* @retval      None
*  
****************************************************************/

void ISO15118_homeplugdev_info_host_set(uint8_t info_sel, uint16_t val)
{
  
  /* INTENTIONALLY EMPTY JUST TO AVOID COMPILER ERRORS */
  
}

#endif


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

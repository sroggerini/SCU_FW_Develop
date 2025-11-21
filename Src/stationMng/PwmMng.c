/**
* @file        PwmMng.c
*
* @brief       Power Management  - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: PwmMng.c 676 2025-01-30 15:54:37Z vania $
*
*     $Revision: 676 $
*
*     $Author: vania $
*
*     $Date: 2025-01-30 16:54:37 +0100 (gio, 30 gen 2025) $
*
*
* @copyright
*       Copyright (C) 2016 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

/************************************************************
 * Include
 ************************************************************/
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_rcc.h"
#else
#include "stm32h5xx_hal.h"
#include "stm32h5xx_ll_rcc.h"
#endif
#include "cmsis_os.h"

#include "sbcGsy.h"
#include "eeprom.h"
#include "InputsMng.h"
#include "rtcApi.h"

#include "EnergyMng.h"
#include "EvsMng.h"
#include "hts.h"
#include "LcdMng.h"
#include "PilotMng.h"
#include "PwmMng.h"
#include "sinapsi.h"
#include "scuMdb.h"
#include "scheduleMng.h"

#include "ExtInpMng.h"
#ifndef HW_MP28947  
#include "homeplugdev.h"
#endif
   
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local defines -------------------------------------------------------------------------------------------------------------------------- //
#define PWM_GARD_TIME                pdMS_TO_TICKS((uint32_t)(100))
#define TIME_RANGE_TIME              pdMS_TO_TICKS((uint32_t)(10000))
#define PMNG_RESTART_TIME            pdMS_TO_TICKS((uint32_t)(30000))
#define PWM_MANAGER_TIME             pdMS_TO_TICKS((uint32_t)(5000))
#define PWM_MANAGER_TIME_ISO15118    pdMS_TO_TICKS((uint32_t)(50000))

#define MIN_CURRENT                  (uint16_t)(60)        // corrente minima da normativa [A * 10]
#define NORM_CURRENT                 (uint16_t)(520)       // corrente di soglia della prima curva pwm_dc/corrente della normativa [A * 10]
   
#define PMNG_KINT                    (int16_t)(1000)
#define PMNG_KPRO                    (int16_t)(100)
   
#define PWM_ISO15118_COMM_VAL        950 /* ISO15118 PWM value used to check communication: dc = 5% */
   
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local typedef -------------------------------------------------------------------------------------------------------------------------- //
typedef enum
{
TIME_RANGE_IDLE = 0,
TIME_RANGE_RUNNING,
TIME_RANGE_DECREASE
}time_range_state_en;                   

typedef enum
{
TIME_RANGE_TIM = 0,
PMNG_RESTART_TIM,
PWM_MANAGER_TIM,
PWM_ISO15118_TIM,
PWM_NUM_TIMER
}PwmTim_en;                   

typedef enum
{
PMNG_EM_OFF = 0,
PMNG_EM_ON
}pwm_em_state_en;                   
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local constats ------------------------------------------------------------------------------------------------------------------------- //
static const uint8_t    pwm_days_of_month[]                     = {0 , 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
static const uint16_t   rotary_switch_current[NUM_ROTARY_POS]   = {60, 100, 130, 160, 200, 250, 320, 400, 500, 630};    // corrente massima da rotary switch [A * 10]
static const uint16_t   pmng_minimum_power_array[]              = {23, 46, 69, 92, 115, 138, 161, 184, 207, 230};       // 1%, 2%, 3%, 4%, 5%, 6%, 7%, 8%, 9%, 10%
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local references ----------------------------------------------------------------------------------------------------------------------- //
xQueueHandle PwmMngQueue =  NULL;

static PwmMngMsg_st         PwmMngMsg;

static TimerHandle_t        xPwmMngTimers[PWM_NUM_TIMER];

static uint16_t             pwm_dc;
static uint8_t              pwm_output_enable;

static uint16_t             pwm_max_current;

static uint8_t              max_current_byte;
static uint8_t              charging_current_byte;

static uint16_t             lcd_max_current;
static uint16_t             lcd_charging_current;

static uint16_t             charging_current;
static uint16_t             charging_current_old;
static uint16_t             hts_charging_current_old;

static uint8_t              pwm_rotary_switch;
static uint16_t             pwm_pp_current;

static uint16_t             pwm_m3t_current;
static uint16_t             pwm_m3s_current;
static uint16_t             pwm_gsy_current;
static uint16_t             pwm_sem_current;
static uint16_t             pwm_sched_current;

static int16_t              pid_error;
static int32_t              pid_prop;
static int32_t              pid_integ;

static uint16_t             pmng_current;
static uint32_t             pmng_current_long;

static int32_t              int_active_power;
static int32_t              ext_active_power;
static int32_t              ext_active_power_max;
static int32_t              ext_active_power_bif;

static uint8_t              eeprom_pmng_enable;
static uint8_t              eeprom_pmng_mode;
static uint8_t              eeprom_min_current;
static uint8_t              eeprom_dmax_power;
static uint8_t              eeprom_emeter_type;
static uint8_t              eeprom_unbalance_enable;
static uint16_t             eeprom_available_power;

static uint16_t             pmng_dmax_power;
static uint16_t             pmng_minimum_power;
static uint16_t             pmng_available_power;
static uint16_t             pmng_smart_plus_power;
static uint16_t             pmng_smart_plus_current;
 
static uint16_t             real_minimum_power;

static uint8_t              sem_available_power_updated;
static uint16_t             sem_available_power;
static uint16_t             sem_available_power_old;

static uint8_t              pmng_over_power_set;
static uint16_t             pmng_max_current;
static uint16_t             pmng_sat_current;

static uint8_t              pmng_pid_enable;
static uint8_t              pmng_restart_enable;
static uint8_t              pmng_suspending_enable;

static uint8_t              pmng_dmax_power_request;
static uint8_t              pmng_over_power_request;

static uint8_t              ext_active_power_update;

static uint8_t              active_phases_detected;
static uint8_t              active_phases_num;

static pwm_em_state_en      pmng_internal_em;
static pwm_em_state_en      pmng_external_em;

static uint8_t              pmng_app_schedule_mode;
static uint16_t             sched_available_power;

static uint8_t              eeprom_time_range_enable;


static uint8_t              time_range_power_decrease;

static time_range_state_en  time_range_state;
static uint8_t              time_range_power;

static uint8_t              pwm_iso15118;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local function prototypes -------------------------------------------------------------------------------------------------------------- //
xQueueHandle getPwmMngQueueHandle(void);

static void PwmMngTimCallBack(TimerHandle_t pxTimer);
static void pwm_rtc_backup_set(uint32_t reg, uint32_t val);
uint32_t pwm_rtc_backup_get(uint32_t reg);
static void pwm_set_timer(PwmTim_en timer, uint32_t set_time);
static uint8_t daylight_savings_time(struct DataAndTime_t *locDateTime);
static void pwm_day_week_update(struct DataAndTime_t *locDateTime);
static void pwm_dc_update(void);
static void time_range_update(uint16_t *power);
static void pmng_manager(uint16_t *pwm_current, uint16_t *max_current);
static void PwmManager_init(void);
static uint32_t PwmManager(PwmMngMsg_st *pMsg);

extern uint8_t is_EvccId_NULL (void);

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  getPwmMngQueueHandle
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
xQueueHandle getPwmMngQueueHandle(void)
{
return(PwmMngQueue);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  PwmMngTimCallBack
//
//  DESCRIPTION:    callback to manager timers   
//
//  INPUT:          TimerHandle_t: the elapsed timer 
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void PwmMngTimCallBack(TimerHandle_t pxTimer)
{
  uint32_t    timer_id;
  
timer_id = (uint32_t)pvTimerGetTimerID(pxTimer);        // find the led  which the timer is referred
  
if (timer_id == (uint32_t)(PMNG_RESTART_TIM))           // check if timer exist
    pmng_restart_enable = 1;
  
if (timer_id == (uint32_t)(TIME_RANGE_TIM))             // check if timer exist
    time_range_power_decrease = 1;
  
if (timer_id == (uint32_t)(PWM_MANAGER_TIM))            // check if timer exist
    send_to_pwm(PWM_CONTROL_TIMEOUT);
  
if (timer_id == (uint32_t)(PWM_ISO15118_TIM))            // check if timer exist
    send_to_pwm(PWM_ISO15118_TIMEOUT);
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pwm_rtc_backup_set
//
//  DESCRIPTION:    salva lo stato della ricarica per la gestione degli stati di low power mode
//
//  INPUT:          status
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void pwm_rtc_backup_set(uint32_t reg, uint32_t val)
{
HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), reg, val);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pwm_rtc_backup_get
//
//  DESCRIPTION:    salva lo stato della ricarica per la gestione degli stati di low power mode
//
//  INPUT:          status
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint32_t pwm_rtc_backup_get(uint32_t reg)
{
return HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)getHandleRtc(), reg);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pwm_set_timer
//
//  DESCRIPTION:    imposta il timer selezionato
//
//  INPUT:          timer da aggiornare: timer; tempo da impostare: set_time
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void pwm_set_timer(PwmTim_en timer, uint32_t set_time)
{
while ((xTimerChangePeriod (xPwmMngTimers[timer], set_time, PWM_GARD_TIME) != pdPASS));  // set timer
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  daylight_savings_time
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t daylight_savings_time(struct DataAndTime_t *locDateTime)
{
uint8_t i, ret = 0;

if ((locDateTime->Month > 3) && (locDateTime->Month < 10))
    ret = 1;
else if (locDateTime->Month == 3)
    {
    if ((locDateTime->DayWeek == 7) && (locDateTime->Day > 24) && (locDateTime->Hour >= 2))
        ret = 1;
    
    for (i=1; i<7; i++)
        {
        if ((locDateTime->DayWeek == i) && (locDateTime->Day > (24 + i)))
            ret = 1;
        }
    }
else if (locDateTime->Month == 10)
    {
    if (locDateTime->Day < 25)
        ret = 1;
    else if ((locDateTime->DayWeek == 7) && (locDateTime->Day > 24) && (locDateTime->Hour < 2))
        ret = 1;

    for (i=1; i<7; i++)
        {
        if ((locDateTime->DayWeek == i) && (locDateTime->Day <= (24 + i)))
            ret = 1;
        }
    }

return ret;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pwm_day_week_update
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void pwm_day_week_update(struct DataAndTime_t *locDateTime)
{
uint16_t i, calc, abs_day;

abs_day = locDateTime->Day;

for (i=1; i<locDateTime->Month; i++)
    abs_day += pwm_days_of_month[i];

if ((locDateTime->Month > 2) && ((locDateTime->Year % 4) == 0))    // anno bisestile
    abs_day += 1;

calc = (locDateTime->Year + ((locDateTime->Year - 1) / 4) - ((locDateTime->Year - 1) / 100) + ((locDateTime->Year - 1) / 400) + abs_day);

locDateTime->DayWeek = (calc % 7) + 6;

if (locDateTime->DayWeek > 7)
    locDateTime->DayWeek -= 7;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  send_to_pwm
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void send_to_pwm(uint8_t pwm_event)
{
PwmMngMsg_st    msgPwmSend;

msgPwmSend.PwmMngEvent = (PwmMngEvent_en)(pwm_event);
configASSERT(xQueueSendToBack(getPwmMngQueueHandle(), (void *)&msgPwmSend, portMAX_DELAY) == pdPASS);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pwm_currents_byte_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void pwm_currents_byte_get(uint8_t *dst_ptr)
{
*dst_ptr = charging_current_byte;
*(dst_ptr + 1) = max_current_byte;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pwm_currents_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void pwm_currents_get(uint8_t *dst_ptr)
{
*dst_ptr = ((lcd_charging_current + 5) / 10);
*(dst_ptr + 1) = ((lcd_max_current + 5) / 10);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pmng_sinapsi_mode_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t pmng_sinapsi_mode_get(void)
{
if ((eeprom_pmng_enable == HIDDEN_MENU_PMNG_ENB) && (getSinapsiEepromEn() == ENABLED))
    return 1;
else
    return 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pmng_app_schedule_mode_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         uint8_t: pwm mode
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t pmng_app_schedule_mode_get(void)
{
return pmng_app_schedule_mode;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pmng_app_schedule_mode_set
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void pmng_app_schedule_mode_set(uint8_t mode)
{
pmng_app_schedule_mode = mode;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pwm_dc_update
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void pwm_dc_update(void)
{
if (pwm_iso15118 == 1)
    pwm_dc = PWM_ISO15118_COMM_VAL;
else if (charging_current < NORM_CURRENT)    // corrente di soglia della prima curva pwm_dc/corrente della normativa
    pwm_dc = 1000 - (((charging_current * 10) + 3) / 6);
else    // if ((crg_curr >= CTHS_CURR) && (crg_curr < CMAX_CURR))
    pwm_dc = 1000 - ((((charging_current * 10) + 12) / 25) + 640);

startPwmOnCP(pwm_dc);
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  time_range_update
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void time_range_update(uint16_t *power)
{
uint8_t                 savings_time;
struct DataAndTime_t    locDateTime;

DateTimeGet(&locDateTime);

savings_time = daylight_savings_time(&locDateTime);

pwm_day_week_update(&locDateTime);

if ((*power >= 30) && (*power <= 45))
    {
    if ((locDateTime.DayWeek == 7)
    || ((locDateTime.Hour == (22 - savings_time)) && (locDateTime.Minute >= 15))
    ||  (locDateTime.Hour > (22 - savings_time))
    ||  (locDateTime.Hour < (5 - savings_time))
    || ((locDateTime.Hour == (5 - savings_time)) && (locDateTime.Minute < 45)))
        {
        time_range_state = TIME_RANGE_RUNNING;
        time_range_power = 55;
        }
    else if (time_range_state == TIME_RANGE_RUNNING)
        {
        time_range_state = TIME_RANGE_DECREASE;
        pwm_set_timer(TIME_RANGE_TIM, TIME_RANGE_TIME);
        }
    }
else if (time_range_state == TIME_RANGE_RUNNING)
    {
    time_range_state = TIME_RANGE_DECREASE;
    pwm_set_timer(TIME_RANGE_TIM, TIME_RANGE_TIME);
    }

if (time_range_state == TIME_RANGE_DECREASE)
    {
    if (time_range_power_decrease == 1)
        {
        if (time_range_power <= *power)
            {
            time_range_power = *power;
            pwm_set_timer(TIME_RANGE_TIM, portMAX_DELAY);
            time_range_state = TIME_RANGE_IDLE;
            }
        else
            {
            time_range_power -= 5;
    
            if (time_range_power <= *power)
                time_range_power = *power;

            pwm_set_timer(TIME_RANGE_TIM, TIME_RANGE_TIME);
            }
        
        time_range_power_decrease = 0;
        }
    }
else if (time_range_state == TIME_RANGE_IDLE)
    {
    time_range_power = *power;
    pwm_set_timer(TIME_RANGE_TIM, portMAX_DELAY);
    }

*power = time_range_power;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pmng_manager
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void pmng_manager(uint16_t *pwm_current, uint16_t *max_current)
{
sinapsiSetReg_st    *pmng_sinapsi_ptr;
uint8_t             pass[3], unbalance_enable, eeprom_min_power_index, eeprom_delta_error;
uint16_t            pmng_max_current_entry;

eeprom_param_get(PMNG_PWRLSB_EADD, pass, 2);
eeprom_available_power = ((uint16_t)(pass[1]) << 8) + pass[0];      // potenza totale disponibile da eeprom data [potenza contrattuale]
eeprom_param_get(PMNG_UNBAL_EADD, &eeprom_unbalance_enable, 1);     // legge consenso al carico sbilanciato nei sistemi trifase

if ((isSemMode() == TRUE) && (eeprom_unbalance_enable == 0))
    {
    eeprom_unbalance_enable = 1;
    SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Unbal, &eeprom_unbalance_enable, 1);      /* ex PMNG_UNBAL_EADD */
    }

evs_error_get(pass, 0, 0, 0);                                       // legge la presenza di errori [EMETER_EXT_ANOM2]

if (((eeprom_pmng_enable & HIDDEN_MENU_PMNG_ENB) == 0)/* || (pmng_internal_em == PMNG_EM_OFF) || ((pass[1] & EMETER_INT_ANOM1) == EMETER_INT_ANOM1)*/)
    {                                                  // power management non abilitato; errore energy meter interno
    pmng_pid_enable = 0;
    pmng_suspending_enable = 0;

    if (isSemMode() == FALSE)
        active_phases_num = 3;

    pmng_dmax_power_request = 0;
    pmng_over_power_request = 0;
    ext_active_power_update = 0;
    pmng_current = eeprom_min_current;                              // si inizializzano i parametri di controllo PID del power management
    pid_integ = ((int32_t)(eeprom_min_current) << 12);
    pmng_current_long = pid_integ;
    return;
    }

eeprom_param_get(PMNG_MULTIP_EADD, &eeprom_min_power_index, 1);     // fattore moltiplicativo della potenza minima per uscire da NO POWER
eeprom_param_get(PMNG_ERROR_EADD, &eeprom_delta_error, 1);          // errore ammesso nella regolazione di potenza [KW * 10] [da sottrarre al setpoint potenza disponibile]
eeprom_param_get(PMNG_DMAX_EADD, &eeprom_dmax_power, 1);            // fattore moltiplicativo della potenza per la sospensione immediata della ricarica
eeprom_param_get(PMNG_TRANGE_EADD, &eeprom_time_range_enable, 1);   // abilitazione modalità a fasce orarie

pmng_max_current_entry = *max_current;

energy_param_get(EM_ACTIVE_POWER, &int_active_power, 1);            // lettura della potenza erogata verso il veicolo

if (eeprom_emeter_type & EMETER_THREE_PH)   // **** SISTEMA TRIFASE
    {
    unbalance_enable = eeprom_unbalance_enable;

    if (active_phases_detected == 0)
        active_phases_num = 3;
    }
else                                        // **** SISTEMA MONOFASE
    {
    unbalance_enable = PMNG_UNBAL_ON;                               // assegnazione fittizia che serve per unificare il codice mono/trifase
    active_phases_detected = 1;
    active_phases_num = 1;
    }

if (getSinapsiEepromEn() == ENABLED)        // **** I PARAMETRI EMETER SONO FORNITI DAL MODULO SINAPSI/ALFA
    {
    unbalance_enable = PMNG_UNBAL_ON;

    pmng_sinapsi_ptr = getIom2Ginfo();
    
    pmng_available_power = ((pmng_sinapsi_ptr->m1Pd + 50) / 100);       // potenza totale disponibile [potenza "istantanea"]
    
    if (pmng_sinapsi_ptr->m1Papi >= 0)
        ext_active_power = ((pmng_sinapsi_ptr->m1Papi + 50) / 100);     // potenza totale assorbita dall'impianto = domestica + potenza assorbita dal veicolo [>= 0]
    else
        ext_active_power = ((pmng_sinapsi_ptr->m1Papi - 50) / 100);     // potenza totale assorbita dall'impianto = domestica + potenza assorbita dal veicolo [< 0]
    }
else                                        // **** I PARAMETRI EMETER SONO FORNITI DA EMETER INTERNO ED ESTERNO TRADIZIONALI
    {
    pmng_available_power = eeprom_available_power;

    if (isSemMode() == TRUE)
        {
        if (pmng_available_power > sem_available_power)
        	pmng_available_power = sem_available_power;

        unbalance_enable = PMNG_UNBAL_ON;
        }
/*    else if (pmng_app_schedule_mode == 1)        // **** POWER MANAGEMENT DA APP
        {
        if (pmng_available_power > Scheduler_getScheduledPower())
            pmng_available_power = Scheduler_getScheduledPower();
        }*/
    else if (eeprom_time_range_enable == 1)
        {
        if ((eeprom_pmng_mode != PMNG_ECO_SMART) && (eeprom_pmng_mode != PMNG_ECO_PLUS))
            time_range_update(&pmng_available_power);
        }

    if ((pmng_external_em == PMNG_EM_OFF) || ((pass[2] & EMETER_EXT_ANOM2) == EMETER_EXT_ANOM2)
        || ((pass[2] & SINAPSI_CHN2_ANOM2) == SINAPSI_CHN2_ANOM2))
        energy_param_get(EM_ACTIVE_POWER, &ext_active_power, 1);        // errore emeter esterno -> copia potenza assorbita dal veicolo in potenza totale assorbita
    else
        energy_param_get(EM_EXT_ACTIVE_POWER, &ext_active_power, 1);
    
    if (unbalance_enable == PMNG_UNBAL_OFF)
        {
        energy_param_get(EM_EXT_L1_ACTIVE_POWER, &ext_active_power_max, 1); // ricerca della fase più carica
        energy_param_get(EM_EXT_L2_ACTIVE_POWER, &ext_active_power, 1);
        
        if (ext_active_power_max < ext_active_power)
            ext_active_power_max = ext_active_power;

        energy_param_get(EM_EXT_L3_ACTIVE_POWER, &ext_active_power, 1);

        if (ext_active_power_max < ext_active_power)
            ext_active_power_max = ext_active_power;

        ext_active_power = (ext_active_power_max * 3);
        ext_active_power_bif = (ext_active_power_max * 2);
        }
    }

if (eeprom_dmax_power != 0)                 // **** DMAX
    pmng_dmax_power = (pmng_available_power + (((pmng_available_power * eeprom_dmax_power) + 50) / 100));  // calcolo della sovra potenza per sospensione immediata
else
    pmng_dmax_power = 0xFFFF;                                           // assegnazione per non far intervenire il limite

switch (active_phases_num)
    {
    case 1:
        {
        if (unbalance_enable == PMNG_UNBAL_OFF)
            {
            pmng_minimum_power = (uint16_t)(((((uint32_t)(eeprom_min_current)) * (6900 + pmng_minimum_power_array[eeprom_min_power_index])) + 5000) / 10000);
            pmng_max_current = ((pmng_available_power * 100) / 69);                                             // calcolo corrente massima dalla potenza disponibile
            }
        else
            {
            pmng_minimum_power = (uint16_t)(((((uint32_t)(eeprom_min_current)) * (2300 + pmng_minimum_power_array[eeprom_min_power_index])) + 5000) / 10000);
                                                                                                                // potenza minima per uscire dalla sospensione NO POWER
            pmng_max_current = ((pmng_available_power * 100) / 23);                                             // calcolo corrente massima dalla potenza disponibile
            }
        
        real_minimum_power = (uint16_t)(((((uint32_t)(eeprom_min_current)) * (2300 + pmng_minimum_power_array[eeprom_min_power_index])) + 5000) / 10000);

        pmng_smart_plus_power = (uint16_t)(((((uint32_t)(eeprom_min_current) + 10) * 2300) + 5000) / 10000);    // calcolo della corrente massima in ECO PLUS e ECO SMART

        pmng_smart_plus_current = ((pmng_smart_plus_power * 100) / 23);
        }
        break;

    case 2:
        {
        if (unbalance_enable == PMNG_UNBAL_OFF)
            {
            pmng_minimum_power = (uint16_t)(((((uint32_t)(eeprom_min_current)) * (6900 + pmng_minimum_power_array[eeprom_min_power_index])) + 5000) / 10000);
            pmng_max_current = ((pmng_available_power * 100) / 69);                                             // calcolo corrente massima dalla potenza disponibile
            }
        else
            {
            pmng_minimum_power = (uint16_t)(((((uint32_t)(eeprom_min_current)) * (4600 + pmng_minimum_power_array[eeprom_min_power_index])) + 5000) / 10000);
                                                                                                                // potenza minima per uscire dalla sospensione NO POWER
            pmng_max_current = ((pmng_available_power * 100) / 46);                                             // calcolo corrente massima dalla potenza disponibile
            }

        real_minimum_power = pmng_minimum_power = (uint16_t)(((((uint32_t)(eeprom_min_current)) * (4600 + pmng_minimum_power_array[eeprom_min_power_index])) + 5000) / 10000);

        pmng_smart_plus_power = (uint16_t)(((((uint32_t)(eeprom_min_current) + 10) * 4600) + 5000) / 10000);    // calcolo della corrente massima in ECO PLUS e ECO SMART
        pmng_smart_plus_current = ((pmng_smart_plus_power * 100) / 46);
        }
        break;

//    case 3:
    default:
        {
        pmng_minimum_power = (uint16_t)(((((uint32_t)(eeprom_min_current)) * (6900 + pmng_minimum_power_array[eeprom_min_power_index])) + 5000) / 10000);
                                                                        // potenza minima per uscire dalla sospensione NO POWER
        pmng_max_current = ((pmng_available_power * 100) / 69);         // calcolo della corrente massima dalla potenza disponibile

        real_minimum_power = pmng_minimum_power;

        pmng_smart_plus_power = (uint16_t)(((((uint32_t)(eeprom_min_current) + 10) * 6900) + 5000) / 10000);    // calcolo della corrente massima in ECO PLUS e ECO SMART
        pmng_smart_plus_current = ((pmng_smart_plus_power * 100) / 69);
        }
        break;
    }

if (pmng_max_current > *max_current)                                            // controllo della limitazione corrente massima possibile
    pmng_max_current = *max_current;
else if (pmng_max_current < eeprom_min_current)                                 // controllo per sicurezza
    pmng_max_current = eeprom_min_current;

if ((eeprom_pmng_mode == PMNG_ECO_SMART) || (eeprom_pmng_mode == PMNG_ECO_PLUS))                    // correzione di pmng_available_power per ECO PLUS e ECO SMART
    {
    if (evs_state_get() == EVSTATE_CHARGING)
        {
        if ((pmng_available_power + int_active_power - ext_active_power) >= pmng_smart_plus_power)  // potenza disponibile [epurata dalla Pist] >= pmng_smart_plus_power
            {
            pmng_available_power = pmng_smart_plus_power;

            if (unbalance_enable == PMNG_UNBAL_OFF)                             // sistema sicuramente trifase
                {
                if ((active_phases_num == 1) && (ext_active_power_max < pmng_available_power))
                    ext_active_power = ext_active_power_max;
                else if ((active_phases_num == 2) && (ext_active_power_bif < pmng_available_power))
                    ext_active_power = ext_active_power_bif;
                }
        
            if ((evs_state_get() == EVSTATE_CHARGING) && (ext_active_power > pmng_smart_plus_power))
                ext_active_power = int_active_power;
            }
        else
            {
            pmng_available_power = (pmng_available_power  + int_active_power - ext_active_power);   // potenza disponibile < pmng_smart_plus_power si lavora sul residuo
            
            if (ext_active_power < pmng_dmax_power)
                ext_active_power = int_active_power;
            }
        }

    if (pmng_over_power_set == 0)
        pmng_sat_current = pmng_smart_plus_current;
    }
else
    {
    if (pmng_over_power_set == 0)
        pmng_sat_current = pmng_max_current;
    else
        pmng_sat_current = pmng_max_current_entry;
    }

if (eeprom_pmng_mode == PMNG_ECO_PLUS)                                          // correzione di pmng_available_power ai fini PID di ECO PLUS
    pmng_available_power = 0;

pid_error = (int16_t)(pmng_available_power) - (int16_t)(ext_active_power);      // **** CALCOLO ERRORE PID

if ((evs_state_get() != EVSTATE_CHARGING) || ((pmng_external_em == PMNG_EM_OFF) && (pmng_internal_em == PMNG_EM_OFF)))
    {
    pid_integ = ((int32_t)(eeprom_min_current) << 12);                          // si inizializzano i parametri di controllo PID del power management

    pmng_current_long = pid_integ;
    pmng_current = (uint16_t)((pmng_current_long + 2048) >> 12);                // fattore di scala pid <-> corrente

    *pwm_current = pmng_current;
    
    if ((eeprom_pmng_mode == PMNG_ECO_SMART) || (eeprom_pmng_mode == PMNG_ECO_PLUS))
        pmng_sat_current = pmng_smart_plus_current;
    else
        pmng_sat_current = pmng_max_current;

    *max_current = pmng_sat_current;
    
    if ((pmng_restart_enable == 1) && (evs_state_get()== EVSTATE_SUSPENDING) && ((pid_error >= pmng_minimum_power)))
        send_to_evs(EVS_PMNG_RELEASE);                                          // condizioni per uscire da NO POWER

	pmng_over_power_set = 0;
	pmng_over_power_request = 0;
	pmng_dmax_power_request = 0;
    ext_active_power_update = 0;

    return;
    }

if ((ext_active_power > pmng_dmax_power) && (pmng_over_power_set == 0))         // sospensione immediata da sovra potenza [DMAX]
    {
    if ((pmng_dmax_power_request == 1) && (ext_active_power_update == 1))
        {
        pmng_restart_enable = 0;
        pmng_dmax_power_request = 0;
        send_to_evs(EVS_PMNG_SUSPENDING);
        pwm_set_timer(PMNG_RESTART_TIM, PMNG_RESTART_TIME);
        }
    else if (pmng_suspending_enable == 1)
        pmng_dmax_power_request = 1;
    }
else if ((pmng_pid_enable == 1) && ((pid_error < 0) ||  (pid_error > eeprom_delta_error)))  // **** RETROAZIONE PID
    {
    pmng_dmax_power_request = 0;

    pid_prop = (pid_error * PMNG_KPRO);                                         // contributo proporzionale
    
    if (pid_error > 0)
        {
        if (pmng_current < pmng_sat_current)
            {
            if (pid_integ < (0x80000000L - (pid_error * PMNG_KINT)))
                pid_integ += (pid_error * PMNG_KINT);                           // contributo integrale [pid_error > 0]
            }
        else if ((active_phases_detected == 1) && (pmng_external_em == PMNG_EM_ON) && (time_range_state != TIME_RANGE_DECREASE))
            {   // si supera la corrente massima derivata da potenza disponibile impostata/calcolata ma c'è ancora potenza disponibile [pid_error > eeprom_delta_error]
            if ((eeprom_pmng_mode == PMNG_ECO_SMART) || (eeprom_pmng_mode == PMNG_ECO_PLUS))
                {
                if (pmng_over_power_set == 0)
                    pmng_sat_current = pmng_max_current;
                else
                    pmng_sat_current = pmng_max_current_entry;

                pmng_over_power_set ++;
                }
            else
                {
                pmng_sat_current = pmng_max_current_entry;
                pmng_over_power_set = 1;
                }
            }
        
        pmng_current_long = pid_prop + pid_integ;
        
        if (pmng_current_long > ((int32_t)(pmng_sat_current) << 12))
            pmng_current_long = ((int32_t)(pmng_sat_current) << 12);
        }
    else    // if (pid_error <= 0)
        {
        if (pmng_current > eeprom_min_current)
            {
            if (pid_integ > (pid_error * PMNG_KINT))
                pid_integ += (pid_error * PMNG_KINT);                           // contributo integrale [pid_error <= 0]
            else
                pid_integ = ((int32_t)(eeprom_min_current) << 12) - 1;
            }
        else if ((pmng_suspending_enable == 1) && (pmng_over_power_set == 0))   // al di sotto della corrente minima, si va in sospensione
            {
            pmng_suspending_enable = 0;
            pmng_restart_enable = 0;
            pwm_set_timer(PMNG_RESTART_TIM, PMNG_RESTART_TIME);                 //  prima che sia trascorso PMNG_RESTART_TIME, non si può uscire dalla sospensione
            send_to_evs(EVS_PMNG_SUSPENDING);
            }

        pmng_current_long = pid_prop + pid_integ;
        }
    }

pmng_current = (uint16_t)((pmng_current_long + 2048) >> 12);    // **** ESTRAZIONE DEL VALORE DI CORRENTE DA PID

if (pmng_current < eeprom_min_current)                                          // controllo di sicurezza
    pmng_current = eeprom_min_current;

if ((pmng_over_power_set > 0) && (pid_error < 0))
    {
    if (ext_active_power > pmng_dmax_power)
        {
        if ((eeprom_pmng_mode == PMNG_ECO_SMART) || (eeprom_pmng_mode == PMNG_ECO_PLUS))
            pmng_sat_current = pmng_smart_plus_current;
        else
            pmng_sat_current = pmng_max_current;

//        pmng_current = eeprom_min_current;
        pmng_current = pmng_max_current;
        pid_integ = ((int32_t)(pmng_current) << 12);
        pmng_current_long = pid_integ;
        pmng_over_power_set = 0;
        pmng_over_power_request = 0;
        }
    else if ((pmng_over_power_request == 1) && (ext_active_power_update == 1))
        {
        if ((eeprom_pmng_mode == PMNG_ECO_SMART) || (eeprom_pmng_mode == PMNG_ECO_PLUS))
            {
            if (pmng_over_power_set == 2)
                {
                pmng_sat_current = pmng_max_current;
                pmng_current = pmng_max_current;
                }
            else
                {
                pmng_sat_current = pmng_smart_plus_current;
                pmng_current = eeprom_min_current;
                }

            pmng_over_power_set --;
            }
        else
            {
            pmng_sat_current = pmng_max_current;
            pmng_current = eeprom_min_current;
            pmng_over_power_set = 0;
            }
    
        pid_integ = ((int32_t)(pmng_current) << 12);
        pmng_current_long = pid_integ;
        pmng_over_power_request = 0;
        }
    else
        pmng_over_power_request = 1;
    }

ext_active_power_update = 0;

if (pmng_current < *pwm_current)                                                // limitazione pwm da pmng
    *pwm_current = pmng_current;

*max_current = pmng_sat_current;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  PwmManager_init
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void PwmManager_init(void)
{
eeprom_pmng_enable = 0;
eeprom_pmng_mode = PMNG_FULL;

pmng_pid_enable = 0;
pmng_suspending_enable = 0;

pmng_over_power_set = 0;
pmng_restart_enable = 0;
pmng_dmax_power_request = 0;
pmng_over_power_request = 0;

pmng_internal_em = PMNG_EM_OFF;
pmng_external_em = PMNG_EM_OFF;

active_phases_detected = 0;
active_phases_num = 3;

time_range_power = 0;
time_range_power_decrease = 0;
time_range_state = TIME_RANGE_IDLE;

pwm_output_enable = 0;
charging_current_old = 0xFFFF;

sem_available_power_updated = 0;
sem_available_power_old = 0;

pmng_app_schedule_mode = 0;

hts_charging_current_old = 0;	/* RIVEDERE: togliere variabile */

pwm_iso15118 = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  PwmManager
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint32_t PwmManager(PwmMngMsg_st *pMsg)
{
uint32_t    newTimeTick = pdMS_TO_TICKS(6000);                      // init di sicurezza
uint16_t    evs_current_max;
uint8_t     m3t_current, m3s_current;

  if (getScuTypeMode() == SCU_SEM_STAND_ALONE)
  {
    return((uint32_t)portMAX_DELAY);
  }


eeprom_param_get(PMNG_MODE_EADD, &eeprom_pmng_mode, 1);             // legge tipo di power management

eeprom_param_get(M3T_CURRENT_EADD, &m3t_current, 1);
eeprom_param_get(M3S_CURRENT_EADD, &m3s_current, 1);

pwm_m3t_current = ((uint16_t)(m3t_current) * 10);
pwm_m3s_current = ((uint16_t)(m3s_current) * 10);

charging_current = pwm_m3t_current;                                 // imposta pwm pari alla corrente massima in Modo3 standard [da eeprom]

pwm_rotary_switch = (uint8_t)getRotarySwitchPos();                  // lettura della posizione del rotary switch

setModbusRotarySwitchCurr((rotaryPos_e)pwm_rotary_switch);          // Set modbus register value with rotary switch current

if ((pwm_rotary_switch < NUM_ROTARY_POS) && (charging_current > rotary_switch_current[pwm_rotary_switch]))  // limitazione pwm da rotary switch
    charging_current = rotary_switch_current[pwm_rotary_switch];

pwm_pp_current = ppcurrent_get();                                   // lettura codice PP [corrente massima del cavo]

if ((pwm_pp_current > 0) && (charging_current > pwm_pp_current))    // limitazione pwm da codice PP
    charging_current = pwm_pp_current;

pwm_gsy_current = gsy_current_get();                                // lettura della corrente impostata via gsy [arbitraria]

if (charging_current > pwm_gsy_current)                             // limitazione pwm da corrente impostata via gsy
    charging_current = pwm_gsy_current;
        
eeprom_param_get(PMNG_CURRENT_EADD, &eeprom_min_current, 1);        // minima corrente pwm applicabile prima di andare in sospensione [A]
eeprom_param_get(EMETER_INT_EADD, &eeprom_emeter_type, 1);          // tipo di energy meter interno
eeprom_param_get(HIDDEN_MENU_ENB_EADD, &eeprom_pmng_enable, 1);     // legge power management enable
eeprom_pmng_enable &= (HIDDEN_MENU_SEM_ENB | HIDDEN_MENU_PMNG_ENB);

if (pMsg->PwmMngEvent == PWM_INTERNAL_EM_GOOD)                      // aggiornamento stato energy meter
    pmng_internal_em = PMNG_EM_ON;
else if (pMsg->PwmMngEvent == PWM_INTERNAL_EM_FAIL)
    pmng_internal_em = PMNG_EM_OFF;
else if (pMsg->PwmMngEvent == PWM_EXTERNAL_EM_GOOD)
    pmng_external_em = PMNG_EM_ON;
else if (pMsg->PwmMngEvent == PWM_EXTERNAL_EM_FAIL)
    pmng_external_em = PMNG_EM_OFF;

if ((isSemMode() == TRUE) && ((eeprom_pmng_enable & HIDDEN_MENU_PMNG_ENB) == 0))
    {                                                               // si converte il limite di potenza da SEM in limite di corrente perché pmng disabilitato
    if (sem_available_power_updated == 0)
        pmng_sem_power_set(sem_available_power, 0xFF);

    if (sem_available_power == 0)                                   // potenza SEM non disponibile neppure nel backup
        pwm_sem_current = eeprom_min_current;
    else if (eeprom_emeter_type & EMETER_THREE_PH)      // **** SISTEMA TRIFASE
        {
        pwm_sem_current = ((sem_available_power * 100) / 69);
        
        if ((eeprom_pmng_enable & HIDDEN_MENU_PMNG_ENB) == 0)
            {
            if (active_phases_num == 2)
                pwm_sem_current = ((sem_available_power * 100) / 46);
            else if (active_phases_num == 1)
                pwm_sem_current = ((sem_available_power * 100) / 23);
            }
        }
    else                                                // **** SISTEMA MONOFASE
        pwm_sem_current = ((sem_available_power * 100) / 23);

    if (charging_current > pwm_sem_current)
        charging_current = pwm_sem_current;
    }
else if ((pmng_app_schedule_mode == 1) && ((eeprom_pmng_enable & HIDDEN_MENU_PMNG_ENB) == 0))
    {
    sched_available_power = Scheduler_getScheduledPower();

    if (eeprom_emeter_type & EMETER_THREE_PH)      // **** SISTEMA TRIFASE
        {
        pwm_sched_current = ((sched_available_power * 100) / 69);
        
        if ((eeprom_pmng_enable & HIDDEN_MENU_PMNG_ENB) == 0)
            {
            if (active_phases_num == 2)
                pwm_sched_current = ((sched_available_power * 100) / 46);
            else if (active_phases_num == 1)
                pwm_sched_current = ((sched_available_power * 100) / 23);
            }
        }
    else                                                // **** SISTEMA MONOFASE
        pwm_sched_current = ((sched_available_power * 100) / 23);

    if (charging_current > pwm_sched_current)
        charging_current = pwm_sched_current;
    }

if (charging_current < eeprom_min_current)                          // controllo finale rispetto alla corrente minima da eeprom
    charging_current = eeprom_min_current;

evs_current_max = charging_current;                                 // imposta la corrente massima da HW dai confronti precedenti

pwm_max_current = charging_current;                                 // imposta pwm max pari alla corrente massima estratta dai confronti precedenti

pmng_manager(&charging_current, &pwm_max_current);                  // il power management può solo diminuire i pwm da cui charging_current e pwm_max_current

if (evs_charging_mode_get() == M3S_CHARGING_MODE)
    {
    if (pwm_max_current > pwm_m3s_current)                          // limitazione pwm max da modalità di carica [Modo3 semplificato]
        pwm_max_current = pwm_m3s_current;

    if (charging_current > pwm_m3s_current)                         // limitazione pwm da modalità di carica [Modo3 semplificato]
        charging_current = pwm_m3s_current;
    }

hts_cold_current_set(pwm_max_current);

if (charging_current > hts_max_current_get())                       // limitazione pwm da sensore di temperatura
    charging_current = hts_max_current_get();

if (charging_current < eeprom_min_current)
    charging_current = eeprom_min_current;

if (charging_current > evs_current_max)
    charging_current = evs_current_max;

if (charging_current < EVS_CURRENT_MIN)                             // controllo finale rispetto alla corrente minima da normativa e alla corrente massima da HW
    charging_current = EVS_CURRENT_MIN;

if (evs_state_get() == EVSTATE_CHARGING)
	{
    if (hts_charging_current_old != charging_current)
        {
        if (charging_current >= 300)
            htsPrintFr_set('4');
        else if (charging_current >= 220)
            htsPrintFr_set('3');
        else if (charging_current >= 140)
            htsPrintFr_set('2');
        else if (charging_current >= 61)
            htsPrintFr_set('1');
        else if (charging_current == EVS_CURRENT_MIN)
            htsPrintFr_set('0');
        }
    
    hts_charging_current_old = charging_current;
    }

setModbusMaxCurrentResult(pwm_max_current);
if (eeprom_emeter_type & EMETER_THREE_PH)   // **** SISTEMA TRIFASE
    setModbusMaxPowerResult((((uint32_t)(pwm_max_current)) * 69));
else                                        // **** SISTEMA MONOFASE
    setModbusMaxPowerResult((((uint32_t)(pwm_max_current)) * 23));

max_current_byte = (uint8_t)((pwm_max_current + 5) / 10);
charging_current_byte = (uint8_t)((charging_current + 5) / 10);

if ((isSemMode() == TRUE) && (sem_available_power_updated != 1))      // in attesa di aggiornamento da SEM si visualizza max_current_byte = 0
    lcd_max_current = 0;
else
    lcd_max_current = pwm_max_current;                                  // corrente di fondo scala per display

lcd_charging_current = charging_current;

if (pMsg->PwmMngEvent == PWM_OUTPUT_DC_START)
    {
    if (charging_current > EVS_START_CURRENT)                       // massimo pwm all'avvio [espresso in corrente] per evitare errate letture sulla presenza del diodo
        charging_current = EVS_START_CURRENT;

    charging_current_old = 0xFFFF;                                  // all'avvio, si forza l'aggiornamento immediato del pwm
    pwm_output_enable = 1;
    pMsg->PwmMngEvent = PWM_CONTROL_TIMEOUT;                        // all'avvio, si forza l'attuazione immediata del pwm
    }
else if ((pMsg->PwmMngEvent == PWM_OUTPUT_HIGH) || (pMsg->PwmMngEvent == PWM_OUTPUT_LOW))   // forzatura dei livelli di uscite del segnlale CP [pwm costante]
    {
    pwm_output_enable = 0;
    
    if (pMsg->PwmMngEvent == PWM_OUTPUT_HIGH)
        {
        stopPwmOnLevel(GPIO_PIN_SET);
        }
    else    // if (pMsg->PwmMngEvent == PWM_OUTPUT_LOW)
        stopPwmOnLevel(GPIO_PIN_RESET);
    }

if (pMsg->PwmMngEvent == PWM_CONTROL_TIMEOUT)
    {
    ISO15118_homeplugdev_info_host_set(ISO15118_MAXCURRENT, ((charging_current + 5) / 10));

    if ((pwm_output_enable == 1) && (charging_current != charging_current_old))   // aggiornamento hw pwm attuato
        {
        pwm_dc_update();
        energy_current_update((uint32_t)(charging_current));
        
        if (evs_state_get() != EVSTATE_CHARGING)                        // in EVS_CHARGING il refresh del display avviene ogni 1s per cui un aggiornamento da qui
            send_to_lcd(LCD_CURRENT_UPDATE);                            // rovina la "estetica" della visualizzazione          
        
        charging_current_old = charging_current;
        pwm_set_timer(PWM_MANAGER_TIM, PWM_MANAGER_TIME);
        }
    else
        pwm_set_timer(PWM_MANAGER_TIM, pdMS_TO_TICKS((uint32_t)(500)));
    }

return newTimeTick;

}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pmng_sem_power_set
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void pmng_sem_power_set(uint16_t available_power, uint8_t sem_update)
{
uint32_t    rtc_backup;

rtc_backup =  pwm_rtc_backup_get(BACKUP_CHARGE_TIME);

if (sem_update != 1)
    {
    if (evs_charging_resume_sem_get() == 1)
        {
        sem_available_power = ((rtc_backup & 0xFF000000) >> 24);
        evs_charging_resume_sem_reset();
        }
    else
        sem_available_power = 0;
    }
else
    {
    sem_available_power = (available_power + 50) / 100;
    evs_charging_resume_reset();
    }

if (sem_available_power != sem_available_power_old)
    {
    rtc_backup &= 0x00FFFFFF;
    rtc_backup += (((uint32_t)(sem_available_power)) << 24);
    pwm_rtc_backup_set(BACKUP_CHARGE_TIME, rtc_backup);
    sem_available_power_old = sem_available_power;
    }

sem_available_power_updated = sem_update;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pmng_suspending_enable_set
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void pmng_suspending_enable_set(void)
{
pmng_suspending_enable = 1;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pmng_suspending_enable_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t pmng_suspending_enable_get(void)
{
return pmng_suspending_enable;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pmng_pid_enable_set
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void pmng_pid_enable_set(uint8_t val)
{
pmng_pid_enable = val;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pwm_iso15118_set
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void pwm_iso15118_set(void)
{
pwm_iso15118 = 1;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pwm_iso15118_clr
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void pwm_iso15118_clr(void)
{
pwm_iso15118 = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pmng_pid_enable_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t pmng_pid_enable_get(void)
{
return pmng_pid_enable;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pmng_ext_active_power_update
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void pmng_ext_active_power_update(void)
{
ext_active_power_update = 1;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  active_phases_num_set
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void active_phases_num_set(uint8_t val)
{
if (val != 0)
    {
    active_phases_num = val;
    active_phases_detected = 1;
    }
else
    active_phases_detected = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  PwmMngTask
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void PwmMngTask(void *pvParameters)
{
uint8_t  i;
uint32_t timeTick;

/* init task */

/*-------- Creates an empty mailbox for PwmMngTask messages --------------------------*/
PwmMngQueue = xQueueCreate(1, sizeof(PwmMngMsg_st));
configASSERT(PwmMngQueue != NULL);

/*-------- Creates all timer for PwmMngTask ------------------------------------------*/
/* in this case we use one shot features */
for (i=0;i<PWM_NUM_TIMER; i++)
    {
    xPwmMngTimers[i] = xTimerCreate("TimPwmMng", portMAX_DELAY, pdFALSE, (void*)(i), PwmMngTimCallBack);
    configASSERT(xPwmMngTimers[i] != NULL);
    }

PwmManager_init();
timeTick = portMAX_DELAY;

/* start task */

for (;;)
    {
    /* Wait for some event from SW */
    if (xQueueReceive(PwmMngQueue, (void *)&PwmMngMsg, timeTick) == pdPASS)
        {
        timeTick = PwmManager(&PwmMngMsg);
        }
    else
        {
        /* Wait for possible handled timeout */
        PwmMngMsg.PwmMngEvent = PWM_CONTROL_TIMEOUT;
        timeTick = PwmManager(&PwmMngMsg);
        }
    }
}

/**
  * @brief  get the current power available in power management enviroment   
  *         
  * @param  uint8_t*: pointer where store info flag 
  * 
  * @retval power available 
  */
unsigned int getAvailablePower_Flag(uint8_t* pFlagPM, int16_t* pidError, uint16_t *pPotMinRestart)
{
    *pFlagPM = (eeprom_pmng_mode | (eeprom_unbalance_enable << 2));
    *pidError = pid_error;
    *pPotMinRestart = real_minimum_power * 100; 

    return (eeprom_available_power * 100);
}

/**
  * @brief  Translate Power manager settings from modbus to eeprom
  *         
  * @param  Address of register to manage
  * 
  * @retval None 
  */
void PM_Mdb_to_EEprom_Translate (uint16_t rAddr)
{
  
  uint16_t tmp16;
  uint8_t  tmp8;
  
  switch (rAddr)
  {
    case ADDR_PM_IMIN_RW:
        // Get I MIN
        tmp16 = getPmImin(); 
        // Set in eeprom        
        /*** SAVE ON EEPROM ***/
        SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Current, (uint8_t *)&tmp16, 1);         /* ex PMNG_CURRENT_EADD */
      break;
      
    case ADDR_PM_PMAX_RW:
        // Get PMAX value 
        tmp16 = getPmPmax();
        // Set in eeprom       
        /*** SAVE ON EEPROM ***/        
        SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Power, (uint8_t *)&tmp16, 2);          /* ex PMNG_PWRLSB_EADD - PMNG_PWRMSB_EADD */
      break;
      
    case ADDR_PM_FLAGS_RW:
        // Get PM flags 
        tmp16 = getPmFlags();
        // Get PM enable bit
        tmp8 = tmp16 & HIDDEN_MENU_PMNG_ENB;
        // Set in eeprom        
        /*** SAVE ON EEPROM ***/
        SCU_InfoStation_Set ((uint8_t *)&infoStation.Hidden_Menu.Enabled, &tmp8, 1);     /* ex HIDDEN_MENU_ENB_EADD */
        // Get Unbalance flag
        tmp8 = tmp16 >> 1;
        // Set in eeprom        
        /*** SAVE ON EEPROM ***/
        SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Unbal, &tmp8, 1);     /* ex PMNG_UNBAL_EADD */
        // Check if TIME RANGE functionality is enabled
        if (tmp16 & PM_TIME_RANGE_FUNC_MASK)
          tmp8 = TRUE;
        else
          tmp8 = FALSE;
        // Set in eeprom        
        /*** SAVE ON EEPROM ***/
        SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Trange, &tmp8, 1);     /* ex PMNG_TRANGE_EADD */
      break;
      
    case ADDR_PM_HPOWER_RW:
        // Get HPOWER value
        tmp16 = getPmHpower() - 1;
        // Set in eeprom        
        /*** SAVE ON EEPROM ***/
        SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Multip, (uint8_t *)&tmp16, 1);         /* ex PMNG_MULTIP_EADD */
      break;
      
    case ADDR_PM_DSET_RW:
        // Get DSET value
        tmp16 = getPmDset();
        // Set in eeprom        
        /*** SAVE ON EEPROM ***/
        SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Error, (uint8_t *)&tmp16, 1);         /* ex PMNG_ERROR_EADD */
      break;
      
    case ADDR_PM_DMAX_RW:
        // Get DMAX value
        tmp16 = getPmDmax();
        // Set in eeprom        
        /*** SAVE ON EEPROM ***/
        SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Dmax, (uint8_t *)&tmp16, 1);         /* ex PMNG_DMAX_EADD */
      break;
      
    case ADDR_PM_MODE_RW:
        // Get PM Mode
        tmp16 = getPmMode();
        // Set in eeprom
        /*** SAVE ON EEPROM ***/
        SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Mode, (uint8_t *)&tmp16, 1);         /* ex PMNG_MODE_EADD */
      break;
      
    default:
      break;
  }
   
}

/**
  * @brief  get the status of PWM on CP, if ACTIVE or NOT
  *         
  * @param  Address of register to manage
  * 
  * @retval None 
  */

uint8_t is_pwm_CP_Active (void)
{
  if (pwm_output_enable)
    return TRUE;
  
  return FALSE;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

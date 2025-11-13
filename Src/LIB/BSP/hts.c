/******************* (C) COPYRIGHT 2009 STMicroelectronics *********/
/**
* @file        ledMng.c
*
* @brief       Manager led RGB  - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: ledMng.c 177 2022-11-24 14:07:34Z stefano $
*
*     $Revision: 177 $
*
*     $Author: stefano $
*
*     $Date: 2022-11-24 15:07:34 +0100 (gio, 24 nov 2022) $
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
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#else
#include "stm32h5xx_hal.h"
#endif
#include "wrapper.h"

#include "rtcApi.h"

#include "EnergyMng.h"
#include "eeprom.h"
#include "hts.h"
#include "i2c.h"
#include "PwmMng.h"
#include "wrapper.h"
#include <string.h>
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local defines -------------------------------------------------------------------------------------------------------------------------- //
#define HTS_GARD_TIME               pdMS_TO_TICKS((uint32_t)(100))
#define HTS_ACTION_ENABLE_TIME      pdMS_TO_TICKS((uint32_t)(900000))   // 15 minuti
//#define HTS_ACTION_ENABLE_TIME      pdMS_TO_TICKS((uint32_t)(90000))   // 1,5 minuti

#ifdef TEST_EM_CLIMA
/*  Temperature Lg  queue  declaration */
#define   PERIODIC_TASK_MESSAGE_NUM ((uint8_t)2)
#define   NUM_PERIODIC_TASK         ((uint8_t)1)
/* define here the id for every periodic task */
#define   TEMPERATURE_LOG_TIMER_ID  ((uint8_t)0)

#define   PERIOD_TASK_STATE_IDLE      ((uint8_t)0)
#define   PERIOD_TASK_STATE_OPERATIVE ((uint8_t)1)

/* define here the period for every task */
#define   PERIOD_TASK_TEMP_LOG      pdMS_TO_TICKS((uint32_t)(300000))   // 5 minuti
//#define   PERIOD_TASK_TEMP_LOG      pdMS_TO_TICKS((uint32_t)(5000))   // 5 minuti
#define   PERIOD_TASK_GARD_TIME     pdMS_TO_TICKS((uint32_t)(500))    // 500ms


#define   NUM_NTC_POINTS              ((uint8_t)75)
#endif
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local typedef -------------------------------------------------------------------------------------------------------------------------- //
typedef enum
{
HTS_ACTION_ENABLE_TIM = 0,
HTS_NUM_TIMER
} HTSMngTim_en;

#ifdef TEST_EM_CLIMA
/*  Temperature Log structure  declaration */
typedef __packed struct
{
  uint8_t         periodicTaskLogId;
  uint16_t        periodTaskEvents;
} periodicTaskMngMsg_st;

typedef __packed struct
{
  uint8_t         periodicTaskState;
  uint16_t        timerId;
  uint16_t        timerTick;
} periodicTaskStateMng_st;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local constats ------------------------------------------------------------------------------------------------------------------------- //
static const uint32_t periodicTickTask[NUM_PERIODIC_TASK] = {
  PERIOD_TASK_TEMP_LOG
};

/* Tabella presa da: C:\Project\Scame\docVari\Cartel1.xlsx */
static const uint16_t ntcVal[NUM_NTC_POINTS] = {
3249,3067,2896,2735,2584,2442,2309,2183,2066,1955,
1851,1752,1660,1573,1491,1414,1341,1272,1207,1146,
1088,1034,982,933,887,844,803,764,727,692,
659,628,599,571,544,519,495,472,451,430,
411,393,375,359,343,328,314,300,287,275,
263,252,242,231,222,213,204,196,188,180,
173,166,159,153,147,141,136,131,126,121,
116,112,108,104,100
};
#endif
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local references ----------------------------------------------------------------------------------------------------------------------- //
xQueueHandle HTSMngQueue = NULL;

static HTSMngMsg_st     HTSMngMsg;

static TimerHandle_t    xHTSMngTimers[HTS_NUM_TIMER];

#ifndef TEST_EM_CLIMA
static uint8_t          nrg2104_temp_enable;
static uint16_t         nrg2104_temp_degrees;
static uint32_t         hts_sample_acc;
#endif

static uint16_t         hts_temp_array[32];
static uint8_t          hts_temp_idx;

static uint16_t         new_hts_temp;
static uint16_t         old_hts_temp;

static uint16_t         hts_cold_current;
static uint16_t         hts_max_current;

static uint8_t          hts_preventive_enable;
static uint8_t          hts_suspending;
static uint8_t          htsPrintEn;
static char             htsPrintFr;

static uint8_t          hts_camera_sim[5];
static uint16_t         hts_camera_delta;
static uint16_t         hts_camera_idx;
static int8_t           hts_camera_step;

#ifdef TEST_EM_CLIMA
/*  Temperature Log  local   declaration */
static    periodicTaskStateMng_st     periodicTaskStateMng[NUM_PERIODIC_TASK];
static    periodicTaskMngMsg_st       periodicTaskMngMsg;
static    TimerHandle_t               xPeriodicTaskTimers[NUM_PERIODIC_TASK];
#endif
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local function prototypes -------------------------------------------------------------------------------------------------------------- //
static void HTSManager_init(void);
static uint32_t HTSManager(HTSMngMsg_st *pMsg);
static void HTSMngTimCallBack(TimerHandle_t pxTimer);
static void hts_set_timer(HTSMngTim_en timer, uint32_t set_time);
static uint16_t NRG2104_manager(void);

#ifdef TEST_EM_CLIMA
/*  periodic task function  prototypes */
static void     periodTaskTimCallBack (TimerHandle_t pxTimer);
static void     periodicTaskManager   (periodicTaskMngMsg_st* pMsg);
static int16_t  NRG2104_mng           (void);

#endif
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 
/*  Temperature Lg  queue  declaration */
xQueueHandle periodicTaskMngQueue = NULL;

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  getHTSMngQueueHandle
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
xQueueHandle getHTSMngQueueHandle(void)
{
return(HTSMngQueue);
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
static void HTSMngTimCallBack(TimerHandle_t pxTimer)
{
uint32_t    timer_id;

timer_id = (uint32_t)pvTimerGetTimerID(pxTimer);                                    // find the led  which the timer is referred

switch (timer_id)
    {
    case HTS_ACTION_ENABLE_TIM:
        {
		hts_preventive_enable = 1;
        send_to_hts(HTS_TIMEOUT);
        }
        break;
    
    default:
        break;
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  hts_set_timer
//
//  DESCRIPTION:    imposta il timer selezionato
//
//  INPUT:          timer da aggiornare: timer; tempo da impostare: set_time
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void hts_set_timer(HTSMngTim_en timer, uint32_t set_time)
{
while ((xTimerChangePeriod (xHTSMngTimers[timer], set_time, HTS_GARD_TIME) != pdPASS));    // set timer
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  send_to_hts
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void send_to_hts(uint8_t hts_event)
{
HTSMngMsg_st    msgHTSSend;

msgHTSSend.HTSMngEvent = (HTSMngEvent_en)(hts_event);
configASSERT(xQueueSendToBack(getHTSMngQueueHandle(), (void *)&msgHTSSend, portMAX_DELAY) == pdPASS);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  hts_temp_degrees_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint16_t hts_temp_degrees_get(void)
{
return new_hts_temp;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  hts_max_current_get
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint16_t hts_max_current_get(void)
{
return hts_max_current;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  htsPrintFr_set
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void htsPrintFr_set(char limit)
{
htsPrintFr = limit;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  hts_cold_current_set
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void hts_cold_current_set(uint16_t hts_charging_current)
{
hts_cold_current = hts_charging_current;

if (hts_max_current == 0xFFFF)
    hts_max_current = hts_cold_current;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  NRG2104_manager
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint16_t NRG2104_manager(void)
{
#ifndef TEST_EM_CLIMA
uint32_t    NRG2104_temp_sample;

hts_sample_acc -= hts_temp_array[hts_temp_idx];
hts_temp_array[hts_temp_idx] = (uint16_t)getADCmV(TEMP_ADC_IN);
hts_sample_acc += hts_temp_array[hts_temp_idx];

NRG2104_temp_sample = ((hts_sample_acc + 8) >> 4);

hts_temp_idx ++;
hts_temp_idx &= 0x0F;

#ifdef NTC_GAIN_1P5
if (NRG2104_temp_sample >= 1964)
    nrg2104_temp_degrees = (uint16_t)(((34667000 - 8233 * NRG2104_temp_sample) + 50000) / 100000);
else if (NRG2104_temp_sample >= 1157)
    nrg2104_temp_degrees = (uint16_t)(((48209000 - 15277 * NRG2104_temp_sample) + 50000) / 100000);
else if (NRG2104_temp_sample >= 707)
    nrg2104_temp_degrees = (uint16_t)(((61832000 - 27278 * NRG2104_temp_sample) + 50000) / 100000);
else if (NRG2104_temp_sample >= 446)
    nrg2104_temp_degrees = (uint16_t)(((75580000 - 47066 * NRG2104_temp_sample) + 50000) / 100000);
else if (NRG2104_temp_sample >= 281)
    nrg2104_temp_degrees = (uint16_t)(((90130000 - 80355 * NRG2104_temp_sample) + 50000) / 100000);
else if (NRG2104_temp_sample >= 183)
    nrg2104_temp_degrees = (uint16_t)(((105290000 - 135330 * NRG2104_temp_sample) + 50000) / 100000);
else if (NRG2104_temp_sample >= 119)
    nrg2104_temp_degrees = (uint16_t)(((120890000 - 222470 * NRG2104_temp_sample) + 50000) / 100000);
else
    nrg2104_temp_degrees = (uint16_t)(((135330000 - 343330 * NRG2104_temp_sample) + 50000) / 100000);
#endif

#ifdef NTC_GAIN_6
if (NRG2104_temp_sample >= 2826)
    nrg2104_temp_degrees = (uint16_t)(((61832000 - 6820 * NRG2104_temp_sample) + 50000) / 100000);
else if (NRG2104_temp_sample >= 1782)
    nrg2104_temp_degrees = (uint16_t)(((75580000 - 11776 * NRG2104_temp_sample) + 50000) / 100000);
else if (NRG2104_temp_sample >= 1122)
    nrg2104_temp_degrees = (uint16_t)(((90130000 - 20089 * NRG2104_temp_sample) + 50000) / 100000);
else if (NRG2104_temp_sample >= 732)
    nrg2104_temp_degrees = (uint16_t)(((105290000 - 33832 * NRG2104_temp_sample) + 50000) / 100000);
else if (NRG2104_temp_sample >= 474)
    nrg2104_temp_degrees = (uint16_t)(((120890000 - 55617 * NRG2104_temp_sample) + 50000) / 100000);
else
    nrg2104_temp_degrees = (uint16_t)(((135330000 - 85832 * NRG2104_temp_sample) + 50000) / 100000);
#endif

if (hts_temp_idx == 0)
    nrg2104_temp_enable = 1;

if (nrg2104_temp_enable == 1)
	return nrg2104_temp_degrees;
else
	return 0;
#else
	return (uint16_t)250;  // always return 25,0°
#endif
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  HTSManager_init
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void HTSManager_init(void)
{
memset(hts_temp_array, 0, 4);

hts_temp_idx = 0;
#ifndef TEST_EM_CLIMA
hts_sample_acc = 0L;
nrg2104_temp_enable = 0;
#endif
new_hts_temp = 0;
old_hts_temp = 0;

hts_cold_current = 0xFFFF;
hts_max_current = 0xFFFF;

hts_preventive_enable = 1;
hts_suspending = 0;

htsPrintEn = 0;
htsPrintFr = 0;

hts_camera_sim[0] = 75;
hts_camera_sim[1] = 0;
hts_camera_sim[2] = 1;
hts_camera_sim[3] = 1;
hts_camera_sim[4] = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  hts_camera_sim_get
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void hts_camera_sim_get(uint8_t *dst_ptr)
{
uint8_t i;

for (i=0; i<5; i++)
    *(dst_ptr + i) = hts_camera_sim[i];
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  hts_camera_sim_set
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void hts_camera_sim_set(uint8_t idx, uint8_t val)
{
hts_camera_sim[idx] = val;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  HTSManager
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint32_t HTSManager(HTSMngMsg_st *pMsg)
{
uint32_t    temp, newTimeTick = pdMS_TO_TICKS(125);
uint16_t    delta_temp, temp_hysteresis, temp_threshold, hts_min_current;
uint8_t     dData, dDec, pmng_enable;
int32_t     hts_charging_current;
int8_t      delta_sign, hts_correction = 0;

eeprom_param_get(EMETER_INT_EADD, &dData, 1);               // tipo di energy meter interno

if (dData & EMETER_THREE_PH)            // **** SISTEMA TRIFASE
    energy_param_get(EM_CURRENT_L1, &hts_charging_current, 1); 
else                                    // **** SISTEMA MONOFASE
    energy_param_get(EM_CURRENT_L, &hts_charging_current, 1);

if (dData == EMETER_TYPE_NULL)
    {
	uint8_t data_array[2];
    pwm_currents_byte_get(data_array);
    hts_charging_current = (((int32_t)(data_array[0])) * 10);
    }

eeprom_param_get(TEMP_DELTA_EADD, &dData, 1);

if ((dData & 0x80) == 0)
    delta_sign = 1;

dData &= 0x7F;
delta_temp = ((uint16_t)(dData) * 10);

new_hts_temp = NRG2104_manager();

if (hts_camera_sim[4] == 0)                                // hts_camera_sim -> simulazione camera termica
    {
    hts_camera_delta = 0;
    hts_camera_idx = 0;
    }
else
    {
    if (hts_camera_sim[4] == 1)
        {
        hts_camera_idx += (uint16_t)(hts_camera_sim[2]) * 4;
        hts_camera_step = 1;
        }
    else if (hts_camera_sim[4] == 2)
        {
        hts_camera_idx += (uint16_t)(hts_camera_sim[3]) * 4;
        hts_camera_step = -1;
        }
    else if (hts_camera_sim[4] == 3)
        hts_camera_step = 0;

    if (hts_camera_idx >= 144)
        {
        hts_camera_delta += ((uint16_t)(hts_camera_sim[1]) * 10);
        hts_camera_sim[1] = 0;

        if ((hts_camera_sim[4] == 1) && ((new_hts_temp + hts_camera_delta) < (((uint16_t)(hts_camera_sim[0]) * 10) + 5)))
            hts_camera_delta += hts_camera_step;
        else if ((hts_camera_sim[4] == 2) && (hts_camera_delta > 0))
            hts_camera_delta += hts_camera_step;

        hts_camera_idx = 0;
        }

    new_hts_temp += hts_camera_delta;
    }

if (delta_sign == 1)
    new_hts_temp += delta_temp;
else if (new_hts_temp > delta_temp)
    new_hts_temp -= delta_temp;
else
    new_hts_temp = 0;

if ((htsPrintFr) || ((htsPrintEn) && (hts_temp_idx == 0)))  // con newTimeTick = pdMS_TO_TICKS(125) e 16 campioni di media, stampa ogni 4s
    {
    dData = (new_hts_temp / 10);
    dDec = (new_hts_temp % 10);

    getCurrentLocalTime(&temp);

    if (htsPrintFr)
        tPrintf("@R%c\n\r", htsPrintFr);
    else
        tPrintf("@T%d,%d - %s\n\r", dData, dDec, (char*)temp);
    
    htsPrintFr = 0;
    }

eeprom_param_get(TEMP_CTRL_ENB_EADD, &dData, 1);

if (dData == 0)     // hts disabilitato
    {
    if (hts_suspending == 1)
        send_to_evs(EVS_HTS_RELEASE);
    
    hts_max_current = hts_cold_current;
	hts_preventive_enable = 1;
    hts_suspending = 0;
    }
else
    {
    eeprom_param_get(HIDDEN_MENU_ENB_EADD, &pmng_enable, 1);

    eeprom_param_get(TEMP_CTRL_VAL_EADD, &dData, 1);
    temp_threshold = ((uint16_t)(dData) * 10);

    eeprom_param_get(TEMP_HYSTERESIS_EADD, &dData, 1);
    temp_hysteresis = ((uint16_t)(dData) * 10);

	if (pmng_enable == 0)                                   // power management disabilitato
        hts_min_current = EVS_CURRENT_MIN;
    else
        {
        eeprom_param_get(PMNG_CURRENT_EADD, &dData, 1);     // corrente minima di ricarica in power management
        hts_min_current = (((uint16_t)(dData)) * 10);
        }
    
    if (new_hts_temp >= (old_hts_temp + 10))                                 // TEMPERATURA IN AUMENTO [differenza > 0,5°C]
        {
        if (new_hts_temp >= temp_threshold)                                 // intervento immediato
        	{
            hts_correction = -1;
	        old_hts_temp = new_hts_temp;
	    	}
//        else if ((hts_preventive_enable == 1) && (new_hts_temp >= (temp_threshold - (temp_hysteresis / 2))))
        else if ((hts_preventive_enable == 1) && (new_hts_temp >= (temp_threshold - temp_hysteresis)))
        	{
            hts_correction = -1;
//    	    hts_preventive_enable = 0;
	        hts_set_timer(HTS_ACTION_ENABLE_TIM, HTS_ACTION_ENABLE_TIME);
	        old_hts_temp = new_hts_temp;
	    	}
        }
    else if (new_hts_temp <= (old_hts_temp - 10))                            // TEMPERATURA IN CALO [differenza > 1°C]
        {
/*        if (new_hts_temp <= (temp_threshold - temp_hysteresis - 5))
        	{
        	hts_correction = 1;
            hts_max_current = hts_cold_current;
	        old_hts_temp = new_hts_temp;
	    	}
        else if ((new_hts_temp <= (temp_threshold - 5)) && (hts_suspending == 0))
        	{
            hts_correction = 1;
	        old_hts_temp = new_hts_temp;
	    	}
        else*/ if (new_hts_temp <= temp_threshold)
        	{
            hts_correction = 1;
	        old_hts_temp = new_hts_temp;
	    	}

        hts_preventive_enable = 1;
        }

     if (new_hts_temp <= (temp_threshold - temp_hysteresis - 10))            // safety control
        {
        hts_correction = 1;
        hts_preventive_enable = 1;
        old_hts_temp = new_hts_temp;
        }
    
    if (hts_correction == -1)
        {
        if (hts_charging_current > (hts_min_current + 80))
            hts_max_current = (uint16_t)(hts_charging_current - 80);        // si tenta di abbassare la temperatura abbassando la corrente
        #ifdef HTS_IMQ
        else if (hts_charging_current > 80)
            hts_max_current = 80;                                           // si tenta di abbassare la temperatura impostando la corrente minima
        #else
        else if (hts_charging_current > hts_min_current)
            hts_max_current = hts_min_current;                              // si tenta di abbassare la temperatura impostando la corrente minima
        #endif
        else
            {
            send_to_evs(EVS_HTS_SUSPENDING);
            hts_preventive_enable = 1;
            hts_suspending = 1;
            }
        }
    else if (hts_correction == 1)
        {
        if (hts_max_current < (hts_cold_current - 80))
            hts_max_current += 80;
        else if (hts_max_current < hts_cold_current)
            hts_max_current = hts_cold_current;
        
        if (hts_suspending == 1)
            {
    		#ifdef HTS_IMQ
            hts_max_current = 80;
            #else
            hts_max_current = hts_min_current;
    		#endif

            send_to_evs(EVS_HTS_RELEASE);
            hts_suspending = 0;
            }
        }
    }
    
return newTimeTick;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  HTSMngTask
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void HTSMngTask(void *pvParameters)
{
uint32_t timeTick;
uint8_t  i;

/* init task */

/*-------- Creates an empty mailbox for HTSMngTask messages --------------------------*/
HTSMngQueue = xQueueCreate(4, sizeof(HTSMngMsg_st));
configASSERT(HTSMngQueue != NULL);

for (i=0; i<HTS_NUM_TIMER; i++)
    {
    xHTSMngTimers[i] = xTimerCreate("TimHTSMng", portMAX_DELAY, pdFALSE, (void*)(i), HTSMngTimCallBack);
    configASSERT(xHTSMngTimers[i] != NULL);
    }

HTSManager_init();

timeTick = pdMS_TO_TICKS(5000);

/* start task */

for (;;)
    {
    /* Wait for some event from SW */
    if (xQueueReceive(HTSMngQueue, (void *)&HTSMngMsg, timeTick) == pdPASS)
        {
        timeTick = HTSManager(&HTSMngMsg);
        }
    else
        {
        /* Wait for possible handled timeout */
        HTSMngMsg.HTSMngEvent = HTS_TIMEOUT;
        timeTick = HTSManager(&HTSMngMsg);
        }
    }
}

/**
*
* @brief        Delete all HTS Timer  
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void deleteHtsTimer (void)
{
  uint8_t   i;

  for (i=0; i<HTS_NUM_TIMER; i++)
  {
    if(xTimerIsTimerActive(xHTSMngTimers[(uint8_t)i])) 
    {
      (void)osTimerDelete (xHTSMngTimers[(uint8_t)i]);
    }
  }
}

/**
*
* @brief        Delete all HTS Timer  
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void printHtsEnable (uint8_t printEn)
{
  htsPrintEn = printEn;
}

#ifdef TEST_EM_CLIMA
/*
*********************************** SCAME ************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/


/**
*
* @brief       Task periodic timestamp with temperature
*
* @param [in]  (void const *: not used  
*  
* @retval      none 
*  
****************************************************************/
void periodicMngTask (void * pvParameters)
{
  uint8_t         ix;

  /*-------- Creates an empty mailbox for uart SITOS messages --------------------------*/
  periodicTaskMngQueue = xQueueCreate(PERIODIC_TASK_MESSAGE_NUM, sizeof(periodicTaskMngMsg_st));
  configASSERT(periodicTaskMngQueue != NULL);

  for (ix = 0; ix < NUM_PERIODIC_TASK; ix++)
  {
    periodicTaskStateMng[ix].periodicTaskState = PERIOD_TASK_STATE_OPERATIVE;
    periodicTaskStateMng[ix].timerId = (uint16_t)ix;
    periodicTaskStateMng[ix].timerTick = (uint16_t)0;
  }

  /*-------- Creates all timer for the led task  --------------------------*/
  for (ix = (uint8_t)0; ix < (uint8_t)NUM_PERIODIC_TASK; ix++)
  {
    /* in this case we use the auto-reload features */
    xPeriodicTaskTimers[ix] = xTimerCreate("PeriodTask", periodicTickTask[ix], pdTRUE, (void*)(ix), periodTaskTimCallBack);
    configASSERT(xPeriodicTaskTimers[ix] != NULL);
    /* start  timer */
    while ((xTimerChangePeriod (xPeriodicTaskTimers[ix], periodicTickTask[ix], PERIOD_TASK_GARD_TIME) != pdPASS)); 
  }

  for (;;)
  {
    /* Wait for some event from SW to change led status */
    if (xQueueReceive(periodicTaskMngQueue, (void *)&periodicTaskMngMsg, portMAX_DELAY) == pdPASS)
    {
      periodicTaskManager((periodicTaskMngMsg_st *)&periodicTaskMngMsg);
    }
    else
    {
      /* timeout: not used  */
    }
  }
}

/**
*
* @brief        esegue gestione comandi task eventi periodici
*
* @param [in]   periodicTaskMngMsg_st*: puntatore al messaggio da eseguire
*
* @retval       none
*
***********************************************************************************************************************/
static void periodicTaskManager (periodicTaskMngMsg_st* pMsg)
{

  switch (pMsg->periodTaskEvents)
  {
    default:
      break;
  }
}


/**
*
* @brief        callback to manager timers   
*
* @param [in]   TimerHandle_t: the elapsed timer 
*
* @retval       none
*
***********************************************************************************************************************/
static void periodTaskTimCallBack (TimerHandle_t pxTimer)
{
  uint32_t          timer_id, tempStr;
  int16_t           new_temp;

  /* find the led  which the timer is referred */
  timer_id = (uint32_t)pvTimerGetTimerID(pxTimer);

  /* check if timer linked to led colour exist  */
  switch(timer_id)
  {
    case TEMPERATURE_LOG_TIMER_ID:
      new_temp = NRG2104_mng();


      getCurrentLocalTime(&tempStr);
      periodicTaskStateMng[TEMPERATURE_LOG_TIMER_ID].timerTick++;

      tPrintf("%d;%d;%s\n\r",  periodicTaskStateMng[TEMPERATURE_LOG_TIMER_ID].timerTick, new_temp, (char*)tempStr);
      break;

    default:
      break;
  }

}

/**
*
* @brief        calcolo temperatura tra -22 e + 52    
*
* @param [in]   none 
*
* @retval       none
*
***********************************************************************************************************************/
static int16_t NRG2104_mng(void)
{
  uint16_t    adcTempValue, ix;

  adcTempValue = (uint16_t)getADCmV(TEMP_ADC_IN);
  for (ix = 0; ix < NUM_NTC_POINTS - 1; ix++)
  {
    if ((adcTempValue > ntcVal[ix + 1]) && (adcTempValue <= ntcVal[ix]))
    {
      break;
    }
  }
  return((int16_t)((int16_t)ix - (int16_t)22));
}
#endif

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

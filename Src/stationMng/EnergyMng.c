/**
* @file        EnergyMng.c
*
* @brief       Energy Management  - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: EnergyMng.c 762 2025-05-30 15:11:39Z stefano $
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

/************************************************************
 * Include 
 *  
 ************************************************************/
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_rcc.h"
#else
#include "stm32h5xx_hal.h"
#include "stm32h5xx_ll_rcc.h"
#endif
#include "cmsis_os.h"

#include "eeprom.h"
#include "wrapper.h"

#include "ContactMng.h"
#include "EnergyMng.h"
#include "EvsMng.h"
#include "EvsTimeMng.h"
#include "PersMng.h"
#include "PwmMng.h"

#include "Em_Task.h"

#include "ExtInpMng.h"

#include "scuMdb.h"
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local defines -------------------------------------------------------------------------------------------------------------------------- //


#define ENERGY_GARD_TIME        pdMS_TO_TICKS((uint32_t)(100))
#define ACTIVE_PHASES_TIME_R    pdMS_TO_TICKS((uint32_t)(30000))
#define ACTIVE_PHASES_TIME_C    pdMS_TO_TICKS((uint32_t)(10000))
#define SELF_CHN2_TIME          pdMS_TO_TICKS((uint32_t)(90000))
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local typedef -------------------------------------------------------------------------------------------------------------------------- //
typedef enum
{
ACTIVE_PHASES_TIM = 0,
SELF_CHN2_TIM,
ENERGY_NUM_TIMER
} EnergyMngTim_en;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local constats ------------------------------------------------------------------------------------------------------------------------- //
static const uint8_t rest_mono_parameter_array[] =      {EM_SYS_VOLTAGE,       EM_CURRENT_L,         EM_COS_PHI,          EM_ACTIVE_POWER,
                                                         EM_REACTIVE_POWER,    EM_TOT_ACTIVE_ENERGY, EM_TOT_REACT_ENERGY, EM_SES_ACTIVE_ENERGY};

static const uint8_t charging_mono_parameter_array[] =  {EM_SYS_VOLTAGE,       EM_COS_PHI,           EM_CURRENT_L,        EM_ACTIVE_POWER,
/* di alcuni parametri si fanno letture ripetute   */    EM_TOT_ACTIVE_ENERGY, EM_SES_ACTIVE_ENERGY, EM_CURRENT_L,        EM_TOT_REACT_ENERGY,
/* per tenere costante l'intervallo tra le letture */    EM_REACTIVE_POWER,    EM_ACTIVE_POWER,      EM_CURRENT_L,        EM_SES_ACTIVE_ENERGY};
/* di EM_CURRENT_L                                 */

static const uint8_t pmng_mono_parameter_array[]      = {EM_EXT_ACTIVE_POWER,  EM_ACTIVE_POWER,      EM_CURRENT_L,        EM_SYS_VOLTAGE,
                                                         EM_EXT_ACTIVE_POWER,  EM_ACTIVE_POWER,      EM_CURRENT_L,        EM_COS_PHI,
                                                         EM_EXT_ACTIVE_POWER,  EM_ACTIVE_POWER,      EM_CURRENT_L,        EM_REACTIVE_POWER,
                                                         EM_EXT_ACTIVE_POWER,  EM_ACTIVE_POWER,      EM_CURRENT_L,        EM_TOT_ACTIVE_ENERGY,
                                                         EM_EXT_ACTIVE_POWER,  EM_ACTIVE_POWER,      EM_CURRENT_L,        EM_TOT_REACT_ENERGY,
                                                         EM_EXT_ACTIVE_POWER,  EM_ACTIVE_POWER,      EM_CURRENT_L,        EM_SES_ACTIVE_ENERGY};

static const uint8_t rest_three_parameter_array[]     = {EM_SYS_PH1_VOLTAGE,   EM_CURRENT_L1,        EM_CURRENT_L2,       EM_CURRENT_L3,
                                                         EM_COS_PH1_PHI,       EM_ACTIVE_POWER,      EM_REACTIVE_POWER,   EM_TOT_ACTIVE_ENERGY,
                                                         EM_TOT_REACT_ENERGY,  EM_SES_ACTIVE_ENERGY};

static const uint8_t charging_three_parameter_array[] = {EM_SYS_PH1_VOLTAGE,   EM_COS_PH1_PHI,       EM_CURRENT_L1,       EM_SES_ACTIVE_ENERGY,
/* di alcuni parametri si fanno letture ripetute   */    EM_TOT_ACTIVE_ENERGY, EM_ACTIVE_POWER,      EM_CURRENT_L2,       EM_TOT_REACT_ENERGY,
/* per tenere costante l'intervallo tra le letture */    EM_REACTIVE_POWER,    EM_SES_ACTIVE_ENERGY, EM_CURRENT_L3,       EM_ACTIVE_POWER};
/* di EM_CURRENT_L1, EM_CURRENT_L2 e EM_CURRENT_L3 */  

static const uint8_t pmng_three_parameter_array[]     = {EM_EXT_ACTIVE_POWER,    EM_ACTIVE_POWER, EM_CURRENT_L1, EM_SYS_PH1_VOLTAGE,
                                                         EM_EXT_L1_ACTIVE_POWER, EM_ACTIVE_POWER, EM_CURRENT_L2, EM_COS_PH1_PHI,
                                                         EM_EXT_ACTIVE_POWER,    EM_ACTIVE_POWER, EM_CURRENT_L3, EM_REACTIVE_POWER,
                                                         EM_EXT_L2_ACTIVE_POWER, EM_ACTIVE_POWER, EM_CURRENT_L1, EM_TOT_ACTIVE_ENERGY,
                                                         EM_EXT_ACTIVE_POWER,    EM_ACTIVE_POWER, EM_CURRENT_L2, EM_TOT_REACT_ENERGY,
                                                         EM_EXT_L3_ACTIVE_POWER, EM_ACTIVE_POWER, EM_CURRENT_L3, EM_SES_ACTIVE_ENERGY};
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local references ----------------------------------------------------------------------------------------------------------------------- //
xQueueHandle EnergyMngQueue = NULL;

static EnergyMngMsg_st        EnergyMngMsg;

static TimerHandle_t          xEnergyMngTimers[ENERGY_NUM_TIMER];

static int32_t                energy_to_gsy_array[EM_READ_REG_NUM];
static uint8_t                energy_parameter_scroll;

static uint8_t                em_tot_active_energy_read;
static int32_t                em_tot_act_energy_old;

static uint8_t                ovc_error;
static uint16_t               ovc_counter;
static uint16_t               pwm_current_nom;
static uint16_t               max_current_010s;
static uint16_t               max_current_1K0s;

static uint8_t                em_active_phases_discovery;
static uint8_t                em_current_phase_detected;

static uint8_t                ble_send;

static uint8_t                energymng_running;

static uint8_t	              self_chn2_timeout;

static uint8_t		      isSemMode_phase_start;

static sinapsiSetReg_st       *em_sinapsi_ptr;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local function prototypes -------------------------------------------------------------------------------------------------------------- //
xQueueHandle getEnergyMngQueueHandle(void);
static void EnergyMngTimCallBack(TimerHandle_t pxTimer);
static void energy_set_timer(EnergyMngTim_en timer, uint32_t set_time);
static void overcurrent_manager(void);
static void energy_ble_update(emReadReg_e energy_parameter, uint32_t Ext_Act_Power_Val);
static void energy_parameters_reset(void);
static void energy_to_gsy_set(emReadReg_e energy_parameter, uint8_t emeter_type);
static void energy_parameters_scroll(void);
static void EnergyManager_init(void);
static uint32_t EnergyManager(EnergyMngMsg_st *pMsg);
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
extern infoStation_t  infoStation;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  getEnergyMngQueueHandle
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
xQueueHandle getEnergyMngQueueHandle(void)
{
return(EnergyMngQueue);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  EnergyMngTimCallBack
//
//  DESCRIPTION:    callback to manager timers
//
//  INPUT:          TimerHandle_t: the elapsed timer
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void EnergyMngTimCallBack(TimerHandle_t pxTimer)
{
uint32_t        timer_id;

timer_id = (uint32_t)pvTimerGetTimerID(pxTimer);                                   // find the led  which the timer is referred

if (timer_id == (uint32_t)(ACTIVE_PHASES_TIM))                                     // check if timer exist
    {
    if (evs_state_get() == EVSTATE_CHARGING)
        {
        em_active_phases_discovery = 1;
        energy_set_timer(ACTIVE_PHASES_TIM, portMAX_DELAY);
        }
    else
        energy_set_timer(ACTIVE_PHASES_TIM, ACTIVE_PHASES_TIME_R);
    }
else if (timer_id == (uint32_t)(SELF_CHN2_TIM))
    {
    if (evs_state_get() == EVSTATE_CHARGING)
        {
        self_chn2_timeout = 1;
        energy_set_timer(SELF_CHN2_TIM, portMAX_DELAY);
        }
    else
        energy_set_timer(SELF_CHN2_TIM, SELF_CHN2_TIME);
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  energy_set_timer
//
//  DESCRIPTION:    imposta il timer selezionato
//
//  INPUT:          timer da aggiornare: timer; tempo da impostare: set_time
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void energy_set_timer(EnergyMngTim_en timer, uint32_t set_time)
{
while ((xTimerChangePeriod (xEnergyMngTimers[timer], set_time, ENERGY_GARD_TIME) != pdPASS));  // set timer
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  send_to_energy
//
//  DESCRIPTION:    impacchetta l'evento da inviare a EnergyMngTask
//  
//  INPUT:          valore di EnergyMngEvent
//  
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void send_to_energy(EnergyMngEvent_en energy_event)
{
EnergyMngMsg_st    msgEnergySend;

msgEnergySend.EnergyMngEvent = (EnergyMngEvent_en)(energy_event);
configASSERT(xQueueSendToBack(getEnergyMngQueueHandle(), (void *)&msgEnergySend, portMAX_DELAY) == pdPASS);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  energy_param_get
//
//  DESCRIPTION:    lettura dei parametri energy_array richiesti
//
//  INPUT:          id del parametro di aprtenza: energy_parameter; puntatore destinatario: dst_ptr; numero parametri da leggere: num
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void energy_param_get(emReadReg_e energy_parameter, int32_t *dst_ptr, uint8_t num)
{
uint8_t     i, energy_limit_set, pass[4];
uint16_t    energy_limit;

eeprom_param_get(TCHARGE_MODE_EADD, &energy_limit_set, 1);

if (energy_limit_set == 1)
    {
    eeprom_param_get(ENRG_LIMIT_EADD, &pass[0], 1);
    energy_limit = ((uint16_t)(pass[0]) * 10);
    }
else
    energy_limit = 0;

if (user_card_auth_get() & ENERGY_CHARGE_AUTH)
    {
    if ((energy_limit_set == 0) || (user_card_energy_get() < energy_limit))   // energy_limit > 0 se energy_limit_set = 1
        energy_limit = user_card_energy_get();

    energy_limit_set = 1;
    }

for (i=0; i<num; i++)
    {
    if ((energy_limit_set == 1) && (((uint8_t)(energy_parameter) + i) == EM_SES_ACTIVE_ENERGY))
        {
        *(dst_ptr + i) = 0;

        if (energy_limit >= SES_ACTIVE_ENERGY)
            *(dst_ptr + i) = energy_limit - SES_ACTIVE_ENERGY;
        }
    else
        {
        *(dst_ptr + i) = energy_array[((uint8_t)(energy_parameter) + i)];
    
        if ((((uint8_t)(energy_parameter) + i) == EM_TOT_ACTIVE_ENERGY) && (em_tot_active_energy_read == 0))
            {
            eeprom_param_get(TOT_ENERGY0_EADD, pass, 4);
            *(dst_ptr + i) = ((int32_t)(pass[0]) << 24) + ((int32_t)(pass[1]) << 16) + ((int32_t)(pass[2]) << 8) + pass[3];
            }
        }
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  energy_to_gsy_get
//
//  DESCRIPTION:    
//
//  INPUT:          
//
//  OUTPUT:         
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void energy_to_gsy_get(emReadReg_e energy_parameter, int32_t *dst_ptr, uint8_t num)
{
uint8_t i, pass[4];

for (i=0; i<num; i++)
    {
    *(dst_ptr + i) = energy_to_gsy_array[((uint8_t)(energy_parameter) + i)];
    
    if ((((uint8_t)(energy_parameter) + i) == EM_TOT_ACTIVE_ENERGY) && (em_tot_active_energy_read == 0))
        {
        eeprom_param_get(TOT_ENERGY0_EADD, pass, 4);
        *(dst_ptr + i) = ((int32_t)(pass[0]) << 24) + ((int32_t)(pass[1]) << 16) + ((int32_t)(pass[2]) << 8) + pass[3];
        }
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  energy_current_update
//
//  DESCRIPTION:    aggiorna le correnti di riferimento per l'algoritmo di overcurrent [OVERCURRENT_CRL1]
//
//  INPUT:          corrente impostata dal pwm: current
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void energy_current_update(uint32_t current)
{
pwm_current_nom = (uint16_t)(current);
max_current_010s = (uint16_t)(((current * 125) + 50) / 100);   // massima corrente permessa al veicolo per 10s [+25%]
max_current_1K0s = (uint16_t)(((current * 110) + 50) / 100);   // massima corrente permessa al veicolo per 1000s [+10%]
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  overcurrent_manager
//
//  DESCRIPTION:    controllo della corrente erogata
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void overcurrent_manager(void)
{
uint8_t         emeter_type, control_enable, error_array[EVS_ERROR_ARRAY_SIZE];

eeprom_param_get(CONTROL_BYTE1_EADD, &control_enable, 1);
control_enable &= OVERCURRENT_CRL1;

eeprom_param_get(EMETER_INT_EADD, &emeter_type, 1);

if (evs_state_get() == EVSTATE_CHARGING)
    {
    evs_error_get(error_array, 1, 1, 6);
    
    if (CURRENT_L >= max_current_010s)  // attenzione: in monofase ho un nuovo campione valido ogni 300ms; in trifase ogni 900ms
        ovc_counter += 100;
    else if (CURRENT_L >= max_current_1K0s)
        ovc_counter ++;
    else if ((ovc_counter > 0) && (CURRENT_L > pwm_current_nom))
        ovc_counter --;
    else
        ovc_counter >>= 1;
    
    if ((control_enable == 0) || (error_array[1] & EMETER_INT_ANOM1))
        {
        ovc_counter = 0;
        ovc_error = 0;
        }
    else if (ovc_counter > 20000)       // step a 100ms -> 20s se CURRENT_L > max_current_010s; 2000s se CURRENT_L > max_current_1K0s
        {
        if (ovc_error == 0)
            send_to_evs(EVS_OVERCURRENT_ERROR);

        ovc_error = 1;
        }
    }
else
    {
    ovc_counter = 0;
    ovc_error = 0;
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  energy_ble_update
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void energy_ble_update(emReadReg_e energy_parameter, uint32_t Ext_Act_Power_Val)
{
    
  measureSck.duration = evstime_busy_get();

  measureSck.Etot = SES_ACTIVE_ENERGY;
  measureSck.Pist = ACTIVE_POWER;
  measureSck.currentL1 = CURRENT_L1;
  measureSck.currentL2 = CURRENT_L2;
  measureSck.currentL3 = CURRENT_L3;
  measureSck.Pest1 = 0;
  measureSck.Pest2 = 0;
  measureSck.Pest3 = 0;

  ble_send ++;

  /* JAPPT-233: The external power must be calculated reading from EM or Sinapsi, outside of this function                   */
  if (energy_parameter == EM_EXT_ACTIVE_POWER)
    measureSck.Pest  = Ext_Act_Power_Val; 
        
  if (ble_send == 10) 
  {
    evs_state_en currState = evs_state_get();
    if ((currState != EVSTATE_SOCKET_AVAILABLE) && (currState != EVSTATE_PLUG_OUT) && (currState != EVSTATE_AUTH_WAIT))
    {
      setTransactionParam(&measureSck, EVSTATE_NULL, 1, SUSPENDING_NULL);
    }
    ble_send = 0;
  }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  energy_parameters_reset
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void energy_parameters_reset(void)
{
uint8_t i;

for (i=0; i<EM_READ_REG_NUM; i++)
    energy_array[i] = 0L;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  energy_parameters_reset
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void i_parameters_reset(void)
{
uint8_t i;

for (i=EM_CURRENT_L1; i<=EM_CURRENT_L; i++)
    energy_array[i] = 0L;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  energy_to_gsy_set
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void energy_to_gsy_set(emReadReg_e energy_parameter, uint8_t emeter_type)
{
int32_t val;
uint8_t trueEmType;

if ((pmng_sinapsi_mode_get() == 1) && (energy_parameter == EM_EXT_ACTIVE_POWER))
    {
    em_sinapsi_ptr = getIom2Ginfo();
	val = em_sinapsi_ptr->m1Papi;
    }
else
    {
      trueEmType = get_emeter_detected_type(INTERNAL_EM);
      if (((emReadReg_e)(energy_parameter) == EM_TOT_ACTIVE_ENERGY) && ((trueEmType == EMETER_MONO_PH_SCAME) || (trueEmType == EMETER_THREE_PH_SCAME)))
        val = getTotalActiveEnergyFromModbusReg();
      else
        val = getEmRegisterValue((emReadReg_e)(energy_parameter), emeter_type);
    }

switch (energy_parameter)
    {
    case EM_SYS_VOLTAGE:
    case EM_SYS_PH1_VOLTAGE:
    case EM_ACTIVE_POWER:
    case EM_REACTIVE_POWER:
        {
		if (val < 0)
		    val = 0;

        val = (val * 10);
        }
        break;

    case EM_CURRENT_L1:
    case EM_CURRENT_L2:
    case EM_CURRENT_L3:
    case EM_CURRENT_L:
    case EM_TOT_ACTIVE_ENERGY:
    case EM_TOT_REACT_ENERGY:
    case EM_SES_ACTIVE_ENERGY:
    case EM_EXT_ACTIVE_POWER:
        {
		if (val < 0)
		    val = 0;

        val = ((val + 50) / 100);
        }
        break;

//  case EM_COS_PHI:
//  case EM_COS_PH1_PHI:
    default:
        {
        }
        break;
    }

energy_to_gsy_array[energy_parameter] = val;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  energy_parameters_scroll
//
//  DESCRIPTION:    lettura dei parametri elettrici di potenza, corrente, ecc.
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void energy_parameters_scroll(void)
{
uint8_t pass[4], pmng_enable, pmng_mode, unbalance_enable, energy_limit_set, emeter_type, energy_parameter, energy_array_num, *energy_array_ptr;
static uint8_t evs_state_Current = EVSTATE_IDLE; /* Fixed ticket SCU-85 */
static uint8_t evs_state_Saved = EVSTATE_IDLE;   /* Fixed ticket SCU-85 */
uint16_t energy_limit;
int32_t	val;
int32_t Ext_Act_Power_Val = 0;
 
eeprom_param_get(EMETER_INT_EADD, &emeter_type, 1);
  
if (emeter_type == EMETER_TYPE_NULL)
    {
    energy_parameters_reset();
    return;
    }
  
eeprom_param_get(HIDDEN_MENU_ENB_EADD, &pmng_enable, 1);
pmng_enable &= HIDDEN_MENU_PMNG_ENB;

eeprom_param_get(PMNG_MODE_EADD, &pmng_mode, 1);
eeprom_param_get(PMNG_UNBAL_EADD, &unbalance_enable, 1);

evs_state_Current = evs_state_get();  /* Fixed ticket SCU-85 */

if ((emeter_type == EMETER_TAMP) || (emeter_type == EMETER_TAMP_3))                 // energy meter TA
    {
    energy_parameters_reset();
    
    if (contact_state_get() == CONTACT_STATE_CLOSE)
        val = getEmRegisterValue((emReadReg_e)(EM_CURRENT_L), emeter_type); 
    else
        val = 0;
    
    if (val < 0)
        CURRENT_L = 0;
    else
        CURRENT_L = (val + 50) / 100;
    
    energy_to_gsy_set(EM_CURRENT_L, emeter_type);
    return;
    }
else if ((evs_state_Current == EVSTATE_CHARGING) || (evs_state_Current == EVSTATE_SUSPENDING) || (evs_state_Current == EVSTATE_S2_WAITING))   // scroll parametri "in carica"
    {
    if ((emeter_type == EMETER_MONO_PH_GAVAZZI) || (emeter_type == EMETER_MONO_PH_ALGO2))
        {
        if (pmng_enable == 0)
            {
            energy_array_ptr = (uint8_t*)(charging_mono_parameter_array);
            energy_array_num = sizeof(charging_mono_parameter_array);
            }
        else
            {
            energy_array_ptr = (uint8_t*)(pmng_mono_parameter_array);
            energy_array_num = sizeof(pmng_mono_parameter_array);
            }
        }
    else    // if ((emeter_type == EMETER_THREE_PH_GAVAZZI) || (emeter_type == EMETER_THREE_PH_ALGO2))
        {
        if (pmng_enable == 0)
            {
            energy_array_ptr = (uint8_t*)(charging_three_parameter_array);
            energy_array_num = sizeof(charging_three_parameter_array);
            }
        else
            {
            energy_array_ptr = (uint8_t*)(pmng_three_parameter_array);
            energy_array_num = sizeof(pmng_three_parameter_array);
            }
        }
    }
else                       // scroll parametri a riposo
    {
        
    if ((emeter_type == EMETER_MONO_PH_GAVAZZI) || (emeter_type == EMETER_MONO_PH_ALGO2))
        {
        energy_array_ptr = (uint8_t*)(rest_mono_parameter_array);
        energy_array_num = sizeof(rest_mono_parameter_array);
        }
    else    // if ((emeter_type == EMETER_THREE_PH_GAVAZZI) || (emeter_type == EMETER_THREE_PH_ALGO2))
        {
        energy_array_ptr = (uint8_t*)(rest_three_parameter_array);
        energy_array_num = sizeof(rest_three_parameter_array);
        }
    }
  
if (evs_state_Current != evs_state_Saved)    /* Different scenario? (Fixed ticket SCU-85) */
{
   energy_parameter_scroll = 0;             /* Restart scrolling (Fixed ticket SCU-85) */
   evs_state_Saved = evs_state_Current;     /* Update status (Fixed ticket SCU-85) */
}

energy_parameter = *(energy_array_ptr + energy_parameter_scroll);               // lettura parametro energy meter
if ((energy_parameter == EM_TOT_ACTIVE_ENERGY) && (isEmScamePresent() == TRUE))
{
val = getEepromTotalActiveEnergy();
}
else
{
val = getEmRegisterValue((emReadReg_e)(energy_parameter), emeter_type);
}
  
evs_error_get(pass, 1, 1, 6);
  
if (energy_parameter <= EM_SES_ACTIVE_ENERGY)
    {
    if ((pass[1] & EMETER_INT_ANOM1) == 0)
        {
        if (val < 0)
            energy_array[energy_parameter] = 0;
        else
            energy_array[energy_parameter] = (val + 50) / 100;
      
        if (evs_state_get() == EVSTATE_CHARGING)
            {
            if (emeter_type & EMETER_THREE_PH)
                {
                if ((energy_parameter == EM_CURRENT_L1) && (em_active_phases_discovery == 1))
                    {
                    if (((em_current_phase_detected & 0x10) == 0x00) && (CURRENT_L1 > 10))
                        em_current_phase_detected ++;
            
                    em_current_phase_detected |= 0x10;
                    }
                else if ((energy_parameter == EM_CURRENT_L2) && (em_active_phases_discovery == 1))
                    {
                    if (((em_current_phase_detected & 0x20) == 0x00) && (CURRENT_L2 > 10))
                        em_current_phase_detected ++;
            
                    em_current_phase_detected |= 0x20;
                    }
                else if ((energy_parameter == EM_CURRENT_L3)  && (em_active_phases_discovery == 1))
                    {
                    if (((em_current_phase_detected & 0x40) == 0x00) && (CURRENT_L3 > 10))
                        em_current_phase_detected ++;
            
                    em_current_phase_detected |= 0x40;
                    }
                else if ((energy_parameter == EM_ACTIVE_POWER) && (ACTIVE_POWER > 3) && (pmng_mode != PMNG_ECO_SMART))
                    {
                    if ((getSinapsiEepromEn() == ENABLED) && (unbalance_enable == PMNG_UNBAL_OFF))
                        {
                        }
                    else if ((isSemMode() == TRUE) && (isSemMode_phase_start == 1))
                        {
                        em_current_phase_detected = 0x73;
                        em_active_phases_discovery = 1;
                        isSemMode_phase_start = 0;
                        }
                    else if ((pmng_pid_enable_get() == 0) && (isSemMode() == FALSE))
                        {
                        em_current_phase_detected = 0x73;
                        em_active_phases_discovery = 1;
                        }
                    }
                }
            else if ((energy_parameter == EM_ACTIVE_POWER) && (ACTIVE_POWER > 3))
                {
                em_current_phase_detected = 0x71;
                em_active_phases_discovery = 1;
                }
        
            if (((em_current_phase_detected & 0x70) == 0x70) && (energy_parameter == EM_ACTIVE_POWER))
                {
                if ((pmng_enable == 0) && (getSinapsiEepromEn() == DISABLED) && ((pmng_app_schedule_mode_get() == 1) || (isSemMode() == TRUE)))
                    {
                    if (em_active_phases_discovery == 1)
                        {
                        if ((em_current_phase_detected & 0x0F) != 0x00)
                            {
                            active_phases_num_set((em_current_phase_detected & 0x03));
                            pmng_pid_enable_set(1);                             // abilitazione del pmng in caso di pmng disabilitato ma in presenza di app scheduling
                            }
              
                        em_current_phase_detected = 0x00;
                        em_active_phases_discovery = 0;
                        energy_set_timer(ACTIVE_PHASES_TIM, ACTIVE_PHASES_TIME_C);
                        }
                    }
                }
        
            if (energy_parameter == EM_TOT_ACTIVE_ENERGY)
                {
                em_tot_active_energy_read = 1;
          
                if ((TOT_ACTIVE_ENERGY != em_tot_act_energy_old) && (isEmScamePresent() == FALSE))
                    {
                      /* per EM Scame total energy è aggiornato in initEmParameter(...) case EVT_STOP_ENRG_ACT_SESS_MEAS */
                      pass[0] = (uint8_t)((TOT_ACTIVE_ENERGY & 0xFF000000) >> 24);
                      pass[1] = (uint8_t)((TOT_ACTIVE_ENERGY & 0x00FF0000) >> 16);
                      pass[2] = (uint8_t)((TOT_ACTIVE_ENERGY & 0x0000FF00) >> 8);
                      pass[3] = (uint8_t)((TOT_ACTIVE_ENERGY & 0x000000FF));
                      EEPROM_Save_Config (TOT_ENERGY0_EADD, pass, 4);
                      InitModbusRegisters_Stop = TRUE;   /* Stop the InitModbusRegister call during this operation */
                    }
          
                em_tot_act_energy_old = TOT_ACTIVE_ENERGY;
          
                eeprom_param_get(TCHARGE_MODE_EADD, &energy_limit_set, 1);
          
                if (energy_limit_set == 1)
                    {
                    eeprom_param_get(ENRG_LIMIT_EADD, &pass[0], 1);
                    energy_limit = ((uint16_t)(pass[0]) * 10);
                    }
                else
                    energy_limit = 0;
          
                if (user_card_auth_get() & ENERGY_CHARGE_AUTH)
                    {
                    if ((energy_limit_set == 0) || (user_card_energy_get() < energy_limit))   // energy_limit > 0 se energy_limit_set = 1
                        energy_limit = user_card_energy_get();
            
                    energy_limit_set = 1;
                    }
          
                if ((energy_limit_set == 1) && (SES_ACTIVE_ENERGY >= energy_limit))
                    {
                    if (evs_mode_get() == EVS_FREE_MODE)
                        send_to_evs(EVS_PULS_STOP);
                    else if (evs_mode_get() == EVS_PERS_MODE)
                        send_to_pers(PERS_CARD_CHARGE_EXPIRED);
                    else    // if (evs_mode_get() > EVS_PERS_MODE)
                        send_to_evs(EVS_AUTH_STOP);
                    }
                }
            }
        else
            {
            isSemMode_phase_start = 1;
            em_active_phases_discovery = 0;
            em_current_phase_detected = 0x00;
            energy_set_timer(ACTIVE_PHASES_TIM, ACTIVE_PHASES_TIME_R);
            }
        }
    }
  
if (energy_parameter >= EM_EXT_ACTIVE_POWER)
    {
    if (energy_parameter == EM_EXT_ACTIVE_POWER)
        pmng_ext_active_power_update();

    if ((pass[2] & (EMETER_EXT_ANOM2 | SINAPSI_CHN2_ANOM2)) == 0)
        {
        if (pmng_sinapsi_mode_get() == 1)
            {
            em_sinapsi_ptr = getIom2Ginfo();
      
            if (em_sinapsi_ptr->m1Papi >= 0)
                {
                /* JAPPT-233: Ext_Act_Power_Val is copyed inside the registers used by the app */
                Ext_Act_Power_Val = em_sinapsi_ptr->m1Papi + 50;
                energy_array[EM_EXT_ACTIVE_POWER] = (Ext_Act_Power_Val / 100);
                }
            else
                {
                Ext_Act_Power_Val = em_sinapsi_ptr->m1Papi - 50;
                energy_array[EM_EXT_ACTIVE_POWER] = (Ext_Act_Power_Val / 100);
                }
            }
        else
            {
            if (val < 0)
                {
                Ext_Act_Power_Val = (val - 50);
                energy_array[energy_parameter] = Ext_Act_Power_Val / 100;
                }
            else
                {
                Ext_Act_Power_Val = (val + 50);
                energy_array[energy_parameter] = Ext_Act_Power_Val / 100;
                }
            }
    
        if (evs_state_get() == EVSTATE_CHARGING)
            {
            if (((em_current_phase_detected & 0x70) == 0x70) && (energy_parameter == EM_EXT_ACTIVE_POWER))
                {
                if ((pmng_enable == 0) && (getSinapsiEepromEn() == DISABLED) && (pmng_app_schedule_mode_get() == 1))
                    {
                    }                                                           // non si fa niente, gestito di sopra
                else if (em_active_phases_discovery == 1)
                    {
                    if ((em_current_phase_detected & 0x0F) != 0x00)
                        {
                        active_phases_num_set((em_current_phase_detected & 0x03));
                        pmng_pid_enable_set(1);                             // abilitazione del pmng standard
                        }
          
                    em_current_phase_detected = 0x00;
                    em_active_phases_discovery = 0;
                    energy_set_timer(ACTIVE_PHASES_TIM, ACTIVE_PHASES_TIME_C);
                    }
                }

            if (internal_chn2_error_enable_get() == 1)
                {
                if (getSinapsiEepromEn() == ENABLED)
                    {
                    if (self_chn2_timeout == 1)
                        {
                        chn2_error_enable_set();

                        if (getSinapsiStatusPlc() != 3)
                            send_to_evs(EVS_SINAPSI_CHN2_FAIL);

                        internal_chn2_error_enable_reset();
                        self_chn2_timeout = 0;
                        energy_set_timer(SELF_CHN2_TIM, portMAX_DELAY);
                        }
                    }
                else
                    {
                    self_chn2_timeout = 0;
                    energy_set_timer(SELF_CHN2_TIM, SELF_CHN2_TIME);
                    }
                }
            }
        }
    }

if (evs_state_get() != EVSTATE_CHARGING)
    {
    self_chn2_timeout = 0;

    if (internal_chn2_error_enable_get() == 1)
        energy_set_timer(SELF_CHN2_TIM, SELF_CHN2_TIME);
    else
        energy_set_timer(SELF_CHN2_TIM, portMAX_DELAY);
    }

if (emeter_type & EMETER_THREE_PH)
    {
    CURRENT_L = CURRENT_L1;             // ricerca di CURRENT_L nei sistemi trifase; CURRENT_L è usata in overcurrent_manager
  
    if (CURRENT_L < CURRENT_L2)
        CURRENT_L = CURRENT_L2;
  
    if (CURRENT_L < CURRENT_L3)
        CURRENT_L = CURRENT_L3;
    }
else
    {
    CURRENT_L1 = CURRENT_L;
    CURRENT_L2 = 0;
    CURRENT_L3 = 0;
    }

/* JAPPT-233: External Active Power measure is passed through this function that fill the registers used by the app */
energy_ble_update((emReadReg_e)(energy_parameter), Ext_Act_Power_Val);
energy_to_gsy_set((emReadReg_e)(energy_parameter), emeter_type);
	    
if (energy_parameter_scroll < (energy_array_num - 1))
    energy_parameter_scroll ++;
else
    energy_parameter_scroll = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  EnergyManager_init
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void EnergyManager_init(void)
{
energy_parameters_reset();

em_tot_active_energy_read = 0;
em_tot_act_energy_old = 0;

em_active_phases_discovery = 0;
em_current_phase_detected = 0x00;

ble_send = 0;
energy_parameter_scroll = 0;

energymng_running = 0;

self_chn2_timeout = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  EnergyManager
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint32_t EnergyManager(EnergyMngMsg_st *pMsg)
{
uint32_t    newTimeTick;

if (pMsg->EnergyMngEvent == ENERGY_CONTROL_STOP)
    newTimeTick = portMAX_DELAY;
else if (pMsg->EnergyMngEvent == ENERGY_INTERNAL_EM_FAIL)
    {
    EnergyManager_init();
    newTimeTick = portMAX_DELAY;
    }
else if (pMsg->EnergyMngEvent == ENERGY_INTERNAL_EM_GOOD)
    {
    EnergyManager_init();
    energymng_running = 1;
    newTimeTick = pdMS_TO_TICKS(500);
    }
else
    {
    energy_parameters_scroll();
    overcurrent_manager();
    newTimeTick = pdMS_TO_TICKS(100);
    }
    
return newTimeTick;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  energymng_running_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t energymng_running_get(void)
{
return energymng_running;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  EnergyMngTask
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void EnergyMngTask(void *pvParameters)
{
uint8_t     i;
uint32_t    timeTick;

/* init task */

/*-------- Creates an empty mailbox for EnergyMngTask messages --------------------------*/
EnergyMngQueue = xQueueCreate(1, sizeof(EnergyMngMsg_st));
configASSERT(EnergyMngQueue != NULL);

/*-------- Creates all timer for EnergyMngTask ------------------------------------------*/
/* in this case we use one shot features */
for (i = 0; i < ENERGY_NUM_TIMER; i++)
    {
    xEnergyMngTimers[i] = xTimerCreate("TimEnergyMng", portMAX_DELAY, pdFALSE, (void*)(i), EnergyMngTimCallBack);
    configASSERT(xEnergyMngTimers[i] != NULL);
    }

EnergyManager_init();
timeTick = portMAX_DELAY;

/* start task */

for (;;)
    {
    /* Wait for some event from SW */
    if (xQueueReceive(EnergyMngQueue, (void *)&EnergyMngMsg, timeTick) == pdPASS)
        {
        timeTick = EnergyManager(&EnergyMngMsg);
        }
    else
        {
        /* Wait for possible handled timeout */
        EnergyMngMsg.EnergyMngEvent = ENERGY_CONTROL_TIMEOUT;
        timeTick = EnergyManager(&EnergyMngMsg);
        }
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

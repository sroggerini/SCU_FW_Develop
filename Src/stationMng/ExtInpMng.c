// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//
//  File:           ExtInpMng.c
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

#include "eeprom.h"

#include "BlockMng.h"
#include "EnergyMng.h"
#include "EvsMng.h"
#include "ExtInpMng.h"
#include "InputsMng.h"
#include "LcdMng.h"
#include "PersMng.h"
#include "PwmMng.h"
#include "sbcGsy.h"
#include "AutotestMng.h"
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local defines -------------------------------------------------------------------------------------------------------------------------- //
#define EXTINP_GARD_TIME            pdMS_TO_TICKS((uint32_t)(100))
#define PULS_HELD0_TIME             pdMS_TO_TICKS((uint32_t)(1000))
//#define PULS_HELD1_TIME             pdMS_TO_TICKS((uint32_t)(4000))
#define PULS_HELD1_TIME             pdMS_TO_TICKS((uint32_t)(3000))
#define PULS_HELD2_TIME             pdMS_TO_TICKS((uint32_t)(500))
#define HIDDEN_MENU_TIME            pdMS_TO_TICKS((uint32_t)(20000))
#define EEPROM_DEFT_TIME            pdMS_TO_TICKS((uint32_t)(20000))
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local typedef -------------------------------------------------------------------------------------------------------------------------- //
typedef enum // possible value of sw_prog_state
{
SW_PROG_NULL = 0,    // not valid sw_prog_state
SW_PROG_OPEN,
SW_PROG_CLOSE
}sw_prog_state_en;

typedef enum // possible value of puls_state
{
PULS_NULL_STATE = 0,    // not valid puls_state
PULS_OPEN,
PULS_CLOSE,
PULS_HELD,
PULS_WAIT
}puls_state_en;

typedef enum
{
EXTINP_PULS_TIM = 0,
EXTINP_HIDDEN_TIM,
EXTINP_SW_PROG_TIM,
EXTINP_NUM_TIMER
}ExtInpTim_en;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local constats ------------------------------------------------------------------------------------------------------------------------- //
static const uint8_t hidden_lcd_array[] = {LCD_PMNG_ENABLE, LCD_PMNG_TYPE, LCD_DOMESTIC_POWER, LCD_MIN_CURRENT, LCD_POWER_MULTIP, LCD_POWER_ERROR, LCD_POWER_DMAX,
                                           LCD_UNBAL_ENB, LCD_EMETER_CRL2, LCD_TIME_RANGE, LCD_TIMED_TIME_ENB, LCD_TIMED_TIME, LCD_ENRG_LIMIT,
                                           LCD_CHANGE_PASSWORD};
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local references ----------------------------------------------------------------------------------------------------------------------- //
xQueueHandle ExtInpMngQueue = NULL;

static ExtInpMngMsg_st        ExtInpMngMsg;

static TimerHandle_t          xExtInpMngTimers[EXTINP_NUM_TIMER];

static sw_prog_state_en       sw_prog_state;

static puls_state_en          puls_state;

static uint8_t                remote_enable_old;
static uint8_t                remote_state;

static uint8_t                hidden_menu_vis;
static uint8_t                hidden_menu_enb;

static uint8_t                hidden_menu_enable;
static hidden_menu_sel_en     hidden_menu_sel;

static uint8_t                hidden_enter_index;
static uint8_t                hidden_enter_max;
static uint16_t               *hidden_enter_data;
static uint8_t                hidden_enter_array[3];

static uint8_t                hidden_password_array[3];
static uint16_t               hidden_pmng_enable;
static uint16_t               hidden_pmng_mode;
static uint16_t               hidden_domestic_power;
static uint16_t               hidden_min_current;
static uint16_t               hidden_power_multip;
static uint16_t               hidden_power_error;
static uint16_t               hidden_power_dmax;
static uint16_t               hidden_unbal_enb;
static uint16_t               hidden_emeter_crl2;
static uint16_t               hidden_time_range;
static uint16_t               hidden_timed_time_enable;
static uint16_t               hidden_timed_time;
static uint16_t               hidden_enrg_limit;
static uint16_t               hidden_change_password;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local function prototypes -------------------------------------------------------------------------------------------------------------- //
static xQueueHandle getExtInpMngQueueHandle(void);
static void ExtInpMngTimCallBack(TimerHandle_t pxTimer);
static void extinp_set_timer(ExtInpTim_en timer, uint32_t set_time);
static void hidden_data_to_array(uint8_t *dst_ptr, uint16_t val, uint8_t num);

static void hidden_menu_start(void);
static void hidden_menu_manager(ExtInpMngMsg_st *pMsg);
static void sw_prog_init(void);
static void sw_prog_manager(ExtInpMngMsg_st *pMsg);
static void puls_manager_init(void);
static void puls_manager(ExtInpMngMsg_st *pMsg);
static void remote_manager_init(void);
static void remote_manager(ExtInpMngMsg_st *pMsg);
static void ExtInpManager_init(void);
static uint32_t ExtInpManager(ExtInpMngMsg_st *pMsg);
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  getExtInpMngQueueHandle
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static xQueueHandle getExtInpMngQueueHandle(void)
{
return(ExtInpMngQueue);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  ExtInpMngTimCallBack
//
//  DESCRIPTION:    callback to manager timers   
//
//  INPUT:          TimerHandle_t: the elapsed timer 
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void ExtInpMngTimCallBack(TimerHandle_t pxTimer)
{
uint32_t    timer_id;

timer_id = (uint32_t)pvTimerGetTimerID(pxTimer);                                    // find the led  which the timer is referred

if (timer_id == (uint32_t)(EXTINP_PULS_TIM))                                        // check if timer exist
    send_to_extinp(PULS_TIME_EXPIRED);
else if (timer_id == (uint32_t)(EXTINP_HIDDEN_TIM))                                 // check if timer exist
    send_to_extinp(HIDDEN_TIME_EXPIRED);
else if (timer_id == (uint32_t)(EXTINP_SW_PROG_TIM))                                // check if timer exist
    send_to_extinp(SW_PROG_TIME_EXPIRED);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  extinp_set_timer
//
//  DESCRIPTION:    imposta il timer selezionato
//
//  INPUT:          timer da aggiornare: timer; tempo da impostare: set_time
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void extinp_set_timer(ExtInpTim_en timer, uint32_t set_time)
{
while ((xTimerChangePeriod (xExtInpMngTimers[timer], set_time, EXTINP_GARD_TIME) != pdPASS));  // set timer
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  send_to_extinp
//
//  DESCRIPTION:    impacchetta l'evento da inviare a ExtInpMngTask
//  
//  INPUT:          valore di ExtInpMngEvent
//  
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void send_to_extinp(uint8_t extinp_event)
{
ExtInpMngMsg_st    msgExtInpSend;

  if (!((getAutotestStatus() != (uint8_t)0) && (extinp_event == EXTINP_SW_PROG)))
  {
    msgExtInpSend.ExtInpMngEvent = (ExtInpMngEvent_en)(extinp_event);
    configASSERT(xQueueSendToBack(getExtInpMngQueueHandle(), (void *)&msgExtInpSend, portMAX_DELAY) == pdPASS);
  }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  hidden_data_to_array
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void hidden_data_to_array(uint8_t *dst_ptr, uint16_t val, uint8_t num)
{
uint8_t     i;
uint16_t    data_div, data16u;

if (num == 0)
    return;
else if (num == 1)
    *dst_ptr = (uint8_t)(val);
else
    {
    data16u = val;
    data_div = 1;

    for (i=0; i<(num - 1); i++)
        data_div = data_div * 10;

    for (i=0; i<num; i++)
        {
        *(dst_ptr + i) = (uint8_t)(data16u / data_div);
        data16u = (val % data_div);
        data_div = (data_div / 10);
        }
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  hidden_menu_init
//
//  DESCRIPTION:    init menù nascosto
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void hidden_menu_init(void)
{
hidden_menu_sel = HIDDEN_IDLE;
hidden_menu_vis = 0x00;
hidden_menu_enb = 0x00;
hidden_menu_enable = 0;
send_to_evs(EVS_EXTERNAL_EM_UPDATE);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  hidden_menu_start
//
//  DESCRIPTION:    start menù nascosto
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void hidden_menu_start(void)
{
uint8_t emeter_type, data_array[2];

// xx eeprom_param_get(EMETER_INT_EADD, &emeter_type, 1);

eeprom_param_get(HIDDEN_MENU_ENB_EADD, (uint8_t*)(&hidden_pmng_enable), 1);
hidden_pmng_enable &= HIDDEN_MENU_PMNG_ENB;

eeprom_param_get(PMNG_MODE_EADD, (uint8_t*)(&hidden_pmng_mode), 1);

eeprom_param_get(PMNG_PWRLSB_EADD, data_array, 2);
hidden_domestic_power = ((uint16_t)(data_array[1]) << 8) + data_array[0];

eeprom_param_get(PMNG_CURRENT_EADD, (uint8_t*)(&hidden_min_current), 1);
eeprom_param_get(PMNG_MULTIP_EADD, (uint8_t*)(&hidden_power_multip), 1);
eeprom_param_get(PMNG_ERROR_EADD, (uint8_t*)(&hidden_power_error), 1);
eeprom_param_get(PMNG_DMAX_EADD, (uint8_t*)(&hidden_power_dmax), 1);
eeprom_param_get(CONTROL_BYTE2_EADD, (uint8_t*)(&hidden_emeter_crl2), 1);
eeprom_param_get(PMNG_UNBAL_EADD, (uint8_t*)(&hidden_unbal_enb), 1);
eeprom_param_get(PMNG_TRANGE_EADD, (uint8_t*)(&hidden_time_range), 1);
eeprom_param_get(TCHARGE_MODE_EADD, (uint8_t*)(&hidden_timed_time_enable), 1);
eeprom_param_get(TCHARGE_TIME_EADD, (uint8_t*)(&hidden_timed_time), 1);
eeprom_param_get(ENRG_LIMIT_EADD, (uint8_t*)(&hidden_enrg_limit), 1);

hidden_emeter_crl2 &= EMETER_EXT_CRL2;

eeprom_param_get(PMNG_PWDB0_EADD, hidden_password_array, 3);
hidden_change_password = ((hidden_password_array[0] - '0') * 100) + ((hidden_password_array[1] - '0') * 10) + (hidden_password_array[2] - '0');


if (((uint8_t)infoStation.emTypeInt > EMETER_TAMP) && ((uint8_t)infoStation.emTypeInt != EMETER_TAMP_3))
	{
	hidden_menu_sel = HIDDEN_TOTAL_ENERGY;
	send_to_lcd(LCD_TOTAL_ENERGY);
	}
else	// if (hidden_menu_vis != HIDDEN_MENU_NULL)
	{
	for (hidden_enter_index=0; hidden_enter_index<3; hidden_enter_index++)
		hidden_enter_array[hidden_enter_index] = '-';

	hidden_enter_index = 0;
	lcd_blink_enb_set(1);
	hidden_menu_sel = HIDDEN_ENTER_PASSWORD;
	send_to_lcd(LCD_ENTER_PASSWORD);
	}

extinp_set_timer(EXTINP_HIDDEN_TIM, HIDDEN_MENU_TIME);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  hidden_menu_manager
//
//  DESCRIPTION:    gestione menù nascosto
//
//  INPUT:          puntatore ExtInpMngEvent: pMsg
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void hidden_menu_manager(ExtInpMngMsg_st *pMsg)
{
uint8_t auth, emeter_type, data_array[3];

eeprom_param_get(EMETER_INT_EADD, &emeter_type, 1);

if (pMsg->ExtInpMngEvent == HIDDEN_TIME_EXPIRED)
    {
    hidden_menu_init();
    
    if (evs_state_get() == EVSTATE_ERROR_WAIT)
        send_to_lcd(LCD_EVS_ERROR);
    else
        send_to_evs(EVS_AUTORIZATION_MODE);
    }
else
    {
    if ((pMsg->ExtInpMngEvent == EXTINP_PULS_HIDDEN) || (pMsg->ExtInpMngEvent == EXTINP_PULS_HELD))
        extinp_set_timer(EXTINP_HIDDEN_TIM, HIDDEN_MENU_TIME);

    switch (hidden_menu_sel)
        {
        case HIDDEN_TOTAL_ENERGY:
            {
            if (pMsg->ExtInpMngEvent == EXTINP_PULS_HELD)
                {
                if (hidden_menu_vis != HIDDEN_MENU_NULL)
                    {
                    for (hidden_enter_index=0; hidden_enter_index<3; hidden_enter_index++)
                        hidden_enter_array[hidden_enter_index] = '-';

                    hidden_enter_index = 0;

                    lcd_blink_enb_set(1);
                    hidden_menu_sel = HIDDEN_ENTER_PASSWORD;
                    send_to_lcd(LCD_ENTER_PASSWORD);
                    }
                }
            }
            break;

        case HIDDEN_ENTER_PASSWORD:
            {
            if (pMsg->ExtInpMngEvent == EXTINP_PULS_HIDDEN)
                {
                if ((hidden_enter_array[hidden_enter_index] == '-') || (hidden_enter_array[hidden_enter_index] == '9'))
                    hidden_enter_array[hidden_enter_index] = '0';
                else
                    hidden_enter_array[hidden_enter_index] ++;
                
                lcd_blink_enb_set(1);
                }
            else if (pMsg->ExtInpMngEvent == EXTINP_PULS_HELD)
                {
                if (hidden_enter_index < 2)
                    {
                    lcd_blink_enb_set(1);
                    hidden_enter_index ++;
                    }
                else
                    {
                    auth = 1;

                    if ((hidden_enter_array[0] == '9') && (hidden_enter_array[1] == '6') && (hidden_enter_array[2] == '3'))
                        {
                        for (hidden_enter_index=0; hidden_enter_index<3; hidden_enter_index++)
                            hidden_password_array[hidden_enter_index] = '0';

                        SCU_InfoStation_Set ((uint8_t *)&infoStation.Hidden_Menu.Pwd, hidden_password_array, 3);       /* ex PMNG_PWDB0_EADD */
                        hidden_change_password = ((hidden_password_array[0] - '0') * 100) + ((hidden_password_array[1] - '0') * 10) + (hidden_password_array[2] - '0');
                        auth = 0;
                        }
                    else
                        {
                        for (hidden_enter_index=0; hidden_enter_index<3; hidden_enter_index++)
                            {
                            if (hidden_enter_array[hidden_enter_index] != hidden_password_array[hidden_enter_index])
                                auth = 0;
                            }
                        }
                    
                    lcd_blink_enb_set(0);

                    if (auth == 1)
                        {
                        if (hidden_menu_vis & HIDDEN_MENU_PMNG_VIS)
                            {
                            hidden_enter_data = &hidden_pmng_enable;
                            hidden_data_to_array(hidden_enter_array, *hidden_enter_data, 3);
                            hidden_menu_sel = HIDDEN_SHOW_PMNGENB;
    	                    hidden_enter_index = 2;
                            send_to_lcd(hidden_lcd_array[(((uint8_t)(hidden_menu_sel) - 3) / 2)]);
                            }
                        else if (hidden_menu_vis & HIDDEN_MENU_TMEG_VIS)
                            {
                            hidden_enter_data = &hidden_timed_time_enable;
                            hidden_data_to_array(hidden_enter_array, *hidden_enter_data, 3);
                            hidden_menu_sel = HIDDEN_SHOW_TIMED_TIME_ENB;
    	                    hidden_enter_index = 2;
                            send_to_lcd(hidden_lcd_array[(((uint8_t)(hidden_menu_sel) - 3) / 2)]);
                            }
                        else
                           hidden_menu_start();	// inserito per debug: si entra qui se le configurazioni non sono coerenti -> ERRORE
                        }
                    else
                        {
                        hidden_menu_sel = HIDDEN_TOTAL_ENERGY;
                        send_to_lcd(LCD_TOTAL_ENERGY);
                        }                    
                    }
                }
            }
            break;

        case HIDDEN_SHOW_PMNGENB:
        case HIDDEN_SHOW_PMNGTYPE:
        case HIDDEN_SHOW_DOMPOWER:
        case HIDDEN_SHOW_MINCURRENT:
        case HIDDEN_SHOW_PWRMULTIP:
        case HIDDEN_SHOW_PWRERROR:
        case HIDDEN_SHOW_PWRDMAX:
        case HIDDEN_SHOW_UNBAL:
        case HIDDEN_SHOW_EMETER_CRL2:
        case HIDDEN_SHOW_TIME_RANGE:
        case HIDDEN_SHOW_TIMED_TIME_ENB:
        case HIDDEN_SHOW_TIMED_TIME:
        case HIDDEN_SHOW_ENRG_LIMIT:
        case HIDDEN_SHOW_CHANGE_PASSWORD:
            {
            if (pMsg->ExtInpMngEvent == EXTINP_PULS_HIDDEN)
                {
                if (hidden_menu_sel < (HIDDEN_SEL_NUM - 2))
                    {
                    hidden_menu_sel += 2;
                    
                    if (hidden_menu_sel == HIDDEN_SHOW_UNBAL)
                        {
                        if ((emeter_type & EMETER_THREE_PH) == 0)   // sistema monofase, si salta HIDDEN_SHOW_UNBAL
                            hidden_menu_sel += 2;
                        }

                    if (hidden_menu_sel == HIDDEN_SHOW_TIME_RANGE)
                        {
                        if (gsy_connected_get() == 0)               // se la sbc non è connessa, si salta HIDDEN_SHOW_TIME_RANGE
                            hidden_menu_sel += 2;
                        }

                    if (hidden_menu_sel == HIDDEN_SHOW_TIMED_TIME_ENB)
                        {
                        if ((hidden_menu_vis & HIDDEN_MENU_TMEG_VIS) == 0)  // se la ricarica a tempo non è abilitabile, si saltano
                            hidden_menu_sel += 6;                           // HIDDEN_SHOW_TIMED_TIME_ENB; HIDDEN_SHOW_TIMED_TIME; HIDDEN_SHOW_ENRG_LIMIT
                        }
                    }
                else
                    {
                    if (hidden_menu_vis & HIDDEN_MENU_PMNG_VIS)
                        hidden_menu_sel = HIDDEN_SHOW_PMNGENB;
                    else	// if (hidden_menu_vis & HIDDEN_MENU_TMEG_VIS)
                        hidden_menu_sel = HIDDEN_SHOW_TIMED_TIME_ENB;
                    }
                
                if (hidden_menu_sel == HIDDEN_SHOW_PMNGENB)
                    hidden_enter_data = &hidden_pmng_enable;
                else if (hidden_menu_sel == HIDDEN_SHOW_PMNGTYPE)
                    hidden_enter_data = &hidden_pmng_mode;
                else if (hidden_menu_sel == HIDDEN_SHOW_DOMPOWER)
                    hidden_enter_data = &hidden_domestic_power;
                else if (hidden_menu_sel == HIDDEN_SHOW_MINCURRENT)
                    hidden_enter_data = &hidden_min_current;
                else if (hidden_menu_sel == HIDDEN_SHOW_PWRMULTIP)
                    hidden_enter_data = &hidden_power_multip;
                else if (hidden_menu_sel == HIDDEN_SHOW_PWRERROR)
                    hidden_enter_data = &hidden_power_error;
                else if (hidden_menu_sel == HIDDEN_SHOW_PWRDMAX)
                    hidden_enter_data = &hidden_power_dmax;
                else if (hidden_menu_sel == HIDDEN_SHOW_UNBAL)
                    hidden_enter_data = &hidden_unbal_enb;
                else if (hidden_menu_sel == HIDDEN_SHOW_EMETER_CRL2)
                    hidden_enter_data = &hidden_emeter_crl2;
                else if (hidden_menu_sel == HIDDEN_SHOW_TIME_RANGE)
                    hidden_enter_data = &hidden_time_range;
                else if (hidden_menu_sel == HIDDEN_SHOW_TIMED_TIME_ENB)
                    hidden_enter_data = &hidden_timed_time_enable;
                else if (hidden_menu_sel == HIDDEN_SHOW_TIMED_TIME)
                    hidden_enter_data = &hidden_timed_time;
                else if (hidden_menu_sel == HIDDEN_SHOW_ENRG_LIMIT)
                    hidden_enter_data = &hidden_enrg_limit;
                else if (hidden_menu_sel == HIDDEN_SHOW_CHANGE_PASSWORD)
                    hidden_enter_data = &hidden_change_password;
                
                if ((hidden_menu_sel == HIDDEN_SHOW_PWRERROR) || (hidden_menu_sel == HIDDEN_SHOW_PWRDMAX)
                 || (hidden_menu_sel == HIDDEN_SHOW_TIMED_TIME) || (hidden_menu_sel == HIDDEN_SHOW_ENRG_LIMIT))
                    hidden_enter_array[2] = *hidden_enter_data;
                else
                    hidden_data_to_array(hidden_enter_array, *hidden_enter_data, 3);

                hidden_enter_index = 2;
                send_to_lcd(hidden_lcd_array[(((uint8_t)(hidden_menu_sel) - 3) / 2)]);
                }
            else if (pMsg->ExtInpMngEvent == EXTINP_PULS_HELD)
                {
                hidden_menu_sel ++;
                
                if ((hidden_menu_sel == HIDDEN_ENTER_PMNGENB) || (hidden_menu_sel == HIDDEN_ENTER_PMNGTYPE) || (hidden_menu_sel == HIDDEN_ENTER_PWRMULTIP)
                 || (hidden_menu_sel == HIDDEN_ENTER_PWRERROR) || (hidden_menu_sel == HIDDEN_ENTER_PWRDMAX) || (hidden_menu_sel == HIDDEN_ENTER_UNBAL)
                 || (hidden_menu_sel == HIDDEN_ENTER_EMETER_CRL2) || (hidden_menu_sel == HIDDEN_ENTER_TIME_RANGE) || (hidden_menu_sel == HIDDEN_ENTER_TIMED_TIME)
                 || (hidden_menu_sel == HIDDEN_ENTER_ENRG_LIMIT) || (hidden_menu_sel == HIDDEN_ENTER_TIMED_TIME_ENB))
                    hidden_enter_index = 2;
                else
                    hidden_enter_index = 0;

                if (hidden_menu_sel == HIDDEN_ENTER_PMNGTYPE)
//                    hidden_enter_max = (PMNG_NUM - 1);
                    hidden_enter_max = PMNG_NUM;                          /* RIVEDERE: inveritre valori di PMNG_ECO_PLUS e PMNG_ECO_SMART */
                else if (hidden_menu_sel == HIDDEN_ENTER_PWRERROR)
                    hidden_enter_max = 20;
                else if (hidden_menu_sel == HIDDEN_ENTER_PWRDMAX)
                    hidden_enter_max = 100;
                else if (hidden_menu_sel == HIDDEN_ENTER_TIME_RANGE)
                    hidden_enter_max = 1;
                else if (hidden_menu_sel == HIDDEN_ENTER_TIMED_TIME)
                    hidden_enter_max = 12;
                else if (hidden_menu_sel == HIDDEN_ENTER_ENRG_LIMIT)
                    hidden_enter_max = 100;
                else if ((hidden_menu_sel == HIDDEN_ENTER_PMNGENB) || (hidden_menu_sel == HIDDEN_ENTER_TIMED_TIME_ENB) || (hidden_menu_sel == HIDDEN_ENTER_EMETER_CRL2))
                    hidden_enter_max = 0xFF;
                else
                    hidden_enter_max = 9;
                
                lcd_blink_enb_set(1);

                if (hidden_menu_sel == HIDDEN_ENTER_CHANGE_PASSWORD)
                    send_to_lcd(LCD_NEW_PASSWORD);
                else
                    send_to_lcd(hidden_lcd_array[(((uint8_t)(hidden_menu_sel) - 3) / 2)]);
                }
            }
            break;

        case HIDDEN_ENTER_PMNGENB:
        case HIDDEN_ENTER_PMNGTYPE:
        case HIDDEN_ENTER_DOMPOWER:
        case HIDDEN_ENTER_MINCURRENT:
        case HIDDEN_ENTER_PWRMULTIP:
        case HIDDEN_ENTER_PWRERROR:
        case HIDDEN_ENTER_PWRDMAX:
        case HIDDEN_ENTER_UNBAL:
        case HIDDEN_ENTER_EMETER_CRL2:
        case HIDDEN_ENTER_TIME_RANGE:
        case HIDDEN_ENTER_TIMED_TIME_ENB:
        case HIDDEN_ENTER_TIMED_TIME:
        case HIDDEN_ENTER_ENRG_LIMIT:
        case HIDDEN_ENTER_CHANGE_PASSWORD:
            {
            if (pMsg->ExtInpMngEvent == EXTINP_PULS_HIDDEN)
                {
                if (hidden_enter_array[hidden_enter_index] < hidden_enter_max)
                    {
                    if (hidden_menu_sel == HIDDEN_ENTER_PMNGENB)
                        hidden_enter_array[hidden_enter_index] ^= HIDDEN_MENU_PMNG_ENB;
                    else if (hidden_menu_sel == HIDDEN_ENTER_TIMED_TIME_ENB)
                        hidden_enter_array[hidden_enter_index] ^= 1;
                    else if (hidden_menu_sel == HIDDEN_ENTER_EMETER_CRL2)
                        hidden_enter_array[hidden_enter_index] ^= (EMETER_EXT_CRL2 | SINAPSI_CHN2_CRL2);
                    else if (hidden_menu_sel == HIDDEN_ENTER_UNBAL)
                        hidden_enter_array[hidden_enter_index] ^= PMNG_UNBAL_ON;
                    else if ((hidden_menu_sel == HIDDEN_ENTER_PWRDMAX) || (hidden_menu_sel == HIDDEN_ENTER_ENRG_LIMIT))
                        hidden_enter_array[hidden_enter_index] += 5;
                    else
                        {
                        if (hidden_menu_sel == HIDDEN_ENTER_PMNGTYPE)
                            {
                            if (hidden_enter_array[hidden_enter_index] == PMNG_FULL)
                                hidden_enter_array[hidden_enter_index] = PMNG_ECO_SMART;
                            else if (hidden_enter_array[hidden_enter_index] == PMNG_ECO_SMART)
                                hidden_enter_array[hidden_enter_index] = PMNG_ECO_PLUS;
                            else if (hidden_enter_array[hidden_enter_index] == PMNG_ECO_PLUS)
                                hidden_enter_array[hidden_enter_index] = PMNG_FULL;
                            }
                        else
                            hidden_enter_array[hidden_enter_index] ++;
                        }
                    }
                else if (hidden_menu_sel == HIDDEN_ENTER_PMNGTYPE)
                    hidden_enter_array[hidden_enter_index] = PMNG_FULL;
                else if (hidden_menu_sel == HIDDEN_ENTER_TIMED_TIME)
                    hidden_enter_array[hidden_enter_index] = 0;
                else if (hidden_menu_sel == HIDDEN_ENTER_ENRG_LIMIT)
                    hidden_enter_array[hidden_enter_index] = 5;
                else
                    hidden_enter_array[hidden_enter_index] = 0;
                
                lcd_blink_enb_set(1);
                }
            else if (pMsg->ExtInpMngEvent == EXTINP_PULS_HELD)
                {
                if (hidden_enter_index < 2)
                    {
                    lcd_blink_enb_set(1);
                    hidden_enter_index ++;
                    }
                else
                    {
                    lcd_blink_enb_set(0);
                    *hidden_enter_data = ((uint16_t)(hidden_enter_array[0]) * 100) + (hidden_enter_array[1] * 10) + hidden_enter_array[2];
                    
                    if (hidden_menu_sel == HIDDEN_ENTER_DOMPOWER)
                        {
                        if (*hidden_enter_data < 30)
                            *hidden_enter_data = 30;
                        else if (*hidden_enter_data > 550)
                            *hidden_enter_data = 550;
    
                        data_array[0] = (uint8_t)(*hidden_enter_data & 0x00FF);
                        data_array[1] = (uint8_t)(*hidden_enter_data >> 8);
                        SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Power, data_array, 2);  /* ex PMNG_PWRLSB_EADD */
                        hidden_data_to_array(hidden_enter_array, *hidden_enter_data, 3);
                        }
                    else if (hidden_menu_sel == HIDDEN_ENTER_MINCURRENT)
                        {
                        if (*hidden_enter_data < EVS_CURRENT_MIN)
                            *hidden_enter_data = EVS_CURRENT_MIN;
                        else if (*hidden_enter_data > 255)
                            *hidden_enter_data = 255;
    
                        SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Current, (uint8_t*)(hidden_enter_data), 1);   /* ex PMNG_CURRENT_EADD */
                        hidden_data_to_array(hidden_enter_array, *hidden_enter_data, 3);
                        }
                    else if (hidden_menu_sel == HIDDEN_ENTER_PMNGENB)
                        {
                        hidden_menu_enb &=~ HIDDEN_MENU_PMNG_ENB;
                        hidden_menu_enb |= (uint8_t)(*hidden_enter_data);
                        SCU_InfoStation_Set ((uint8_t *)&infoStation.Hidden_Menu.Enabled, &hidden_menu_enb, 1);    /* ex HIDDEN_MENU_ENB_EADD */
                        }
                    else if (hidden_menu_sel == HIDDEN_ENTER_TIMED_TIME_ENB)
                        SCU_InfoStation_Set ((uint8_t *)&infoStation.TCharge.Mode, (uint8_t*)(hidden_enter_data), 1);    /* ex TCHARGE_MODE_EADD */
                    else if (hidden_menu_sel == HIDDEN_ENTER_PWRMULTIP)
                        SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Multip, (uint8_t*)(hidden_enter_data), 1);     /* ex PMNG_MULTIP_EADD */
                    else if (hidden_menu_sel == HIDDEN_ENTER_PMNGTYPE)
                        SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Mode, (uint8_t*)(hidden_enter_data), 1);       /* ex PMNG_MODE_EADD */
                    else if (hidden_menu_sel == HIDDEN_ENTER_PWRERROR)
                        SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Error, (uint8_t*)(hidden_enter_data), 1);   /* ex PMNG_ERROR_EADD */
                    else if (hidden_menu_sel == HIDDEN_ENTER_PWRDMAX)
                        SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Dmax, (uint8_t*)(hidden_enter_data), 1);    /* ex PMNG_DMAX_EADD */
                    else if (hidden_menu_sel == HIDDEN_ENTER_UNBAL)
                        SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Unbal, (uint8_t*)(hidden_enter_data), 1);     /* ex PMNG_UNBAL_EADD */
                    else if (hidden_menu_sel == HIDDEN_ENTER_EMETER_CRL2)
                        SCU_InfoStation_Set ((uint8_t *)&infoStation.controlByte.Byte.Byte2, (uint8_t*)(hidden_enter_data), 1);    /* ex CONTROL_BYTE2_EADD */
                    else if (hidden_menu_sel == HIDDEN_ENTER_TIME_RANGE)
                        SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Trange, (uint8_t*)(hidden_enter_data), 1);     /* ex PMNG_TRANGE_EADD */
                    else if (hidden_menu_sel == HIDDEN_ENTER_TIMED_TIME)
                        SCU_InfoStation_Set ((uint8_t *)&infoStation.TCharge.Time, (uint8_t*)(hidden_enter_data), 1);     /* ex TCHARGE_TIME_EADD */
                    else if (hidden_menu_sel == HIDDEN_ENTER_ENRG_LIMIT)
                        SCU_InfoStation_Set ((uint8_t *)&infoStation.Energy_limit, (uint8_t*)(hidden_enter_data), 1);     /* ex ENRG_LIMIT_EADD */
                    else if (hidden_menu_sel == HIDDEN_ENTER_CHANGE_PASSWORD)
                        {
                        *hidden_enter_data = (hidden_enter_array[0] * 100) + (hidden_enter_array[1] * 10) + hidden_enter_array[2];
                        
                        for (hidden_enter_index=0; hidden_enter_index<3; hidden_enter_index++)
                            {
                            hidden_enter_array[hidden_enter_index] += '0';
                            hidden_password_array[hidden_enter_index] = hidden_enter_array[hidden_enter_index];
                            }

                        SCU_InfoStation_Set ((uint8_t *)&infoStation.Hidden_Menu.Pwd, hidden_enter_array, 3);        /* ex PMNG_PWDB0_EADD */                       
                        hidden_change_password = ((hidden_password_array[0] - '0') * 100) + ((hidden_password_array[1] - '0') * 10) + (hidden_password_array[2] - '0');

                        for (hidden_enter_index=0; hidden_enter_index<3; hidden_enter_index++)
                            hidden_enter_array[hidden_enter_index] -= '0';
                        }
                    
                    hidden_enter_index = 2;
                    hidden_menu_sel --;
                    send_to_lcd(hidden_lcd_array[(((uint8_t)(hidden_menu_sel) - 3) / 2)]);
                    }
                }
            }
            break;

        default:
            {
            }
            break;
        }
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  hidden_array_index_get
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t hidden_array_index_get(void)
{
return hidden_enter_index;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  hidden_menu_sel_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
hidden_menu_sel_en hidden_menu_sel_get(void)
{
return hidden_menu_sel;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  hidden_enter_array_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void hidden_enter_array_get(uint8_t *dst_ptr)
{
uint8_t i;

for (i=0; i<3; i++)
    *(dst_ptr + i) = hidden_enter_array[i];
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  hidden_menu_enable_set
//
//  DESCRIPTION:    -
//  
//  INPUT:          -
//  
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void hidden_menu_enable_set(uint8_t enable)
{
hidden_menu_enable = enable;

eeprom_param_get(HIDDEN_MENU_VIS_EADD, &hidden_menu_vis, 1);
eeprom_param_get(HIDDEN_MENU_ENB_EADD, &hidden_menu_enb, 1);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  sw_prog_init
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void sw_prog_init(void)
{
sw_prog_state = SW_PROG_NULL;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  sw_prog_manager
//
//  DESCRIPTION:    -
//
//  INPUT:          puntatore ExtInpMngEvent: pMsg
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void sw_prog_manager(ExtInpMngMsg_st *pMsg)
{

switch (sw_prog_state)
    {
    case SW_PROG_NULL:
        {
        if (pMsg->ExtInpMngEvent == EXTINP_SW_PROG)
            {
            if (getInputState(SWPROG_PIN_UP) == GPIO_PIN_RESET)
                {
                sw_prog_state = SW_PROG_CLOSE;
                extinp_set_timer(EXTINP_SW_PROG_TIM, EEPROM_DEFT_TIME);
                }
            else
                sw_prog_state = SW_PROG_OPEN;
            }
        }
        break;

    case SW_PROG_CLOSE:
        {
        if (pMsg->ExtInpMngEvent == SW_PROG_TIME_EXPIRED)
            {
            restoreFactoryDefault();  // after that a reset occur
            }
        else if (pMsg->ExtInpMngEvent == EXTINP_SW_PROG)
            {
            HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), BACKUP_CHARGE_STATUS, 0L);
            while(HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)getHandleRtc(), BACKUP_CHARGE_STATUS) != 0x00000000);
            NVIC_SystemReset(); 
            }
        }
        break;

//  case SW_PROG_OPEN:
    default:
        {
        if (pMsg->ExtInpMngEvent == EXTINP_SW_PROG)
            {
            sw_prog_state = SW_PROG_CLOSE;
            extinp_set_timer(EXTINP_SW_PROG_TIM, EEPROM_DEFT_TIME);
            }
        }
        break;
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  puls_manager_init
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void puls_manager_init(void)
{
puls_state = PULS_NULL_STATE;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  puls_manager
//
//  DESCRIPTION:    gestione pulsante esterno: antirimbalzo ulteriore; lettura al rilascio; tasto mantenuto premuto
//
//  INPUT:          puntatore ExtInpMngEvent: pMsg
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void puls_manager(ExtInpMngMsg_st *pMsg)
{
uint8_t         puls_enable;
evs_state_en    state = evs_state_get();

eeprom_param_get(CONTROL_BYTE0_EADD, &puls_enable, 1);
puls_enable &= PULS_CRL0;

if (puls_enable == 0)
    puls_manager_init();
else
    {
    switch (puls_state)
        {
        case PULS_NULL_STATE:
            {
            if (getInputState(IN2_PULSANTE_UP_PIN_UP) == GPIO_PIN_RESET)
                puls_state = PULS_CLOSE;
            else
                puls_state = PULS_OPEN;
            }
            break;
    
        case PULS_OPEN:
            {
            if (getInputState(IN2_PULSANTE_UP_PIN_UP) == GPIO_PIN_RESET)
                {
                send_to_pers(PERS_PULS_SET);
                puls_state = PULS_CLOSE;
                extinp_set_timer(EXTINP_PULS_TIM, PULS_HELD0_TIME);
                }
            }
            break;
    
        case PULS_CLOSE:
            {
            if (getInputState(IN2_PULSANTE_UP_PIN_UP) == GPIO_PIN_SET)
                {
                if (state == EVSTATE_PLUG_OUT)
                    send_to_block(BLOCK_MANUAL_REQ);
                else if (hidden_menu_sel > HIDDEN_IDLE)
                    send_to_extinp(EXTINP_PULS_HIDDEN);
                else
                    send_to_evs(EVS_PULS_STOP);

                send_to_pers(PERS_PULS_RESET);
                /* If autotest is active, notify the result of the test  */                
                NOTIFY_AUTOTEST_RESULT (AUTOTEST_EVENT_MOTOR_1, TEST_PASSED);          
                puls_state = PULS_OPEN;
                }
            else if (pMsg->ExtInpMngEvent == PULS_TIME_EXPIRED)
                {
                if (lcd_language_def_enable_get() == 1)
                    {
                    lcd_language_def_update();
                    send_to_pers(PERS_PULS_RESET);
                    puls_state = PULS_WAIT;
                    }
                else
                    {
                    if (hidden_menu_sel > HIDDEN_TOTAL_ENERGY)
                        extinp_set_timer(EXTINP_PULS_TIM, PULS_HELD2_TIME);
                    else
                        extinp_set_timer(EXTINP_PULS_TIM, PULS_HELD1_TIME);

                    puls_state = PULS_HELD;
                    }
                }
            else if (pMsg->ExtInpMngEvent == EXTINP_MASTER_CARD)
                puls_state = PULS_WAIT;
            }
            break;

        case PULS_HELD:
            {
            if (getInputState(IN2_PULSANTE_UP_PIN_UP) == GPIO_PIN_SET)
                {
                if (state == EVSTATE_PLUG_OUT)
                    send_to_block(BLOCK_MANUAL_REQ);
                else if (hidden_menu_sel > HIDDEN_IDLE)
                    send_to_extinp(EXTINP_PULS_HIDDEN);
                else
                    send_to_evs(EVS_PULS_STOP);

                puls_state = PULS_OPEN;
                }
            else if (pMsg->ExtInpMngEvent == PULS_TIME_EXPIRED)
                {
                send_to_pers(PERS_PULS_RESET);
                send_to_extinp(EXTINP_PULS_HELD);
                puls_state = PULS_WAIT;
                }
            else if (pMsg->ExtInpMngEvent == EXTINP_MASTER_CARD)
                puls_state = PULS_WAIT;
            }
            break;

        case PULS_WAIT:
            {
            if (getInputState(IN2_PULSANTE_UP_PIN_UP) == GPIO_PIN_SET)
                puls_state = PULS_OPEN;
            }
            break;

        default:
            {
            }
            break;
        }
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  remote_manager_init
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void remote_manager_init(void)
{
remote_enable_old = 0;
remote_state = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  remote_manager
//
//  DESCRIPTION:    lettura contatto di ingresso di abilitazione alla ricarica con antirimbalzo ulteriore
//
//  INPUT:          puntatore ExtInpMngEvent: pMsg
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void remote_manager(ExtInpMngMsg_st *pMsg)
{
uint8_t control_enable;

eeprom_param_get(CONTROL_BYTE0_EADD, &control_enable, 1);

if (control_enable & REMOTE_CRL0)
    {
      
    if  (inpMng[IN1_REMOTE_EXP0].stato == INP_STATE_DISABLED)    /* Fixed ticket SP-4 */
      return;
    
    if (remote_state == 0)
        {
        if (getInputState(IN1_REMOTE_EXP0) == GPIO_PIN_RESET)
            {
            send_to_evs(EVS_REMOTE_SUSPENDING);
            remote_state = 1;
            }
        }
    else if (getInputState(IN1_REMOTE_EXP0) == GPIO_PIN_SET)
        {
        send_to_evs(EVS_REMOTE_RELEASE);
        remote_state = 0;
        }
    }
else if (remote_enable_old & REMOTE_CRL0)
    {
    send_to_evs(EVS_REMOTE_RELEASE);
    remote_state = 0;
    }

remote_enable_old = control_enable;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  ExtInpManager_init
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void ExtInpManager_init(void)
{
sw_prog_init();
puls_manager_init();
remote_manager_init();
hidden_menu_init();
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  ExtInpManager
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint32_t ExtInpManager(ExtInpMngMsg_st *pMsg)
{
uint8_t emeter_type;

eeprom_param_get(EMETER_INT_EADD, &emeter_type, 1);

if (hidden_menu_sel > HIDDEN_IDLE)
    hidden_menu_manager(&ExtInpMngMsg);
else if ((pMsg->ExtInpMngEvent == EXTINP_PULS_HELD) && (hidden_menu_enable == 1))
    {
    if ((hidden_menu_vis != HIDDEN_MENU_NULL) || ((emeter_type > EMETER_TAMP) && (emeter_type != EMETER_TAMP_3)))
        hidden_menu_start();
    }

if ((pMsg->ExtInpMngEvent == EXTINP_SW_PROG) || (pMsg->ExtInpMngEvent == SW_PROG_TIME_EXPIRED))
    sw_prog_manager(&ExtInpMngMsg);

if ((pMsg->ExtInpMngEvent == PULS_SWITCH_UPDATE) || (pMsg->ExtInpMngEvent == EXTINP_MASTER_CARD) || (pMsg->ExtInpMngEvent == PULS_TIME_EXPIRED))
    puls_manager(&ExtInpMngMsg);

if ((pMsg->ExtInpMngEvent == REMOTE_SWITCH_UPDATE) || (pMsg->ExtInpMngEvent == EXTINP_TIMEOUT))
    remote_manager(&ExtInpMngMsg);

return pdMS_TO_TICKS(250);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  ExtInpMngTask
//
//  DESCRIPTION:    Handle power extinp
//
//  INPUT:          puntatore ExtInpMngEvent: pMsg
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void ExtInpMngTask(void *pvParameters)
{
uint8_t  i;
uint32_t timeTick;

/* init task */

/*-------- Creates an empty mailbox for ExtInpMngTask messages --------------------------*/
ExtInpMngQueue = xQueueCreate(4, sizeof(ExtInpMngMsg_st));
configASSERT(ExtInpMngQueue != NULL);

/*-------- Creates all timer for ExtInpMngTask ------------------------------------------*/
/* in this case we use one shot features */
for (i=0;i<EXTINP_NUM_TIMER; i++)
    {
    xExtInpMngTimers[i] = xTimerCreate("TimExtInpMng", portMAX_DELAY, pdFALSE, (void*)(i), ExtInpMngTimCallBack);
    configASSERT(xExtInpMngTimers[i] != NULL);
    }

ExtInpManager_init();
timeTick = portMAX_DELAY;

/* start task */

for (;;)
    {
    /* Wait for some event from SW */
    if (xQueueReceive(ExtInpMngQueue, (void *)&ExtInpMngMsg, timeTick) == pdPASS)
        {
        timeTick = ExtInpManager(&ExtInpMngMsg);
        }
    else
        {
        /* Wait for possible handled timeout */
        ExtInpMngMsg.ExtInpMngEvent = EXTINP_TIMEOUT;
        timeTick = ExtInpManager(&ExtInpMngMsg);
        }
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

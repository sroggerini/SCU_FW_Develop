// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  
//  File:           PersMng.c
//  Author:         Vania
//  Date:           06/11/2020
//
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local include -------------------------------------------------------------------------------------------------------------------------- //
#include "displayPin.h"
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_rcc.h"
#else
#include "stm32h5xx_hal.h"
#include "stm32h5xx_ll_rcc.h"
#endif
#include "cmsis_os.h"

#include "eeprom.h"
#include "rtcApi.h"
#include "sbcGsy.h"

#include "EvsMng.h"
#include "ExtInpMng.h"
#include "EvsTimeMng.h"
#include "LcdMng.h"
#include "PersMng.h"
#include "RfidMng.h"
#include "scuMdb.h"
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local defines -------------------------------------------------------------------------------------------------------------------------- //
#define MASTER_FORMAT_DIM   (uint16_t)(6)

#define PERS_GARD_TIME      pdMS_TO_TICKS((uint32_t)(100))
#define PERS_SEC_TIME       pdMS_TO_TICKS((uint32_t)(1000))
#define MODE_BLIND_TIME     pdMS_TO_TICKS((uint32_t)(3000))
#define PERS_DATA_TIME      pdMS_TO_TICKS((uint32_t)(3000))
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local typedef -------------------------------------------------------------------------------------------------------------------------- //
typedef enum // possible value for whitelist_update input
{
UID_DELETE = 0,
UID_ADD
}white_list_operation_en;

typedef enum
{
PERS_STATE_TIM = 0,
UID_CONFIRM_TIM,
UPDATE_WLIST_TIM,
UID_DELETE_TIM,
WLIST_DELETE_TIM,
PERS_DATA_TIM,
PERS_BLIND_TIM,
PERS_NUM_TIMER
}PersMngTim_en;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local constats ------------------------------------------------------------------------------------------------------------------------- //
static const uint8_t pers_days_of_month[] = {0 , 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
static const uint8_t master_format_array[MASTER_FORMAT_DIM] = {'M', 'A', 'S', 'T', 'E', 'R'};
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local references ----------------------------------------------------------------------------------------------------------------------- //
xQueueHandle PersMngQueue = NULL;

static PersMngMsg_st    PersMngMsg;

static TimerHandle_t    xPersMngTimers[PERS_NUM_TIMER];

static pers_state_en    pers_state;
static pers_state_en    pers_state_next;

static uint8_t          pers_puls_held;

static uint8_t          master_card_reg;
static uint8_t          master_uid_array[CARD_UID_DIM];

static uint8_t          new_uid_array[CARD_UID_DIM];
static uint8_t          new_block_array[INFO_BLOCK_DIM];

static uint8_t          current_uid_array[CARD_UID_DIM];
static uint8_t          current_block_array[CARD_UID_DIM];

static uint8_t          eeprom_uid_array[CARD_UID_DIM];

static uint8_t          uid_map_num;
static uint8_t          uid_map_array[USER_MAP_EEDIM];
static uint8_t          uid_map_byte;
static uint8_t          uid_map_bit;

static card_auth_en     user_card_mode;
static uint16_t         user_card_credit;
static uint32_t         user_card_date[4];
static uint32_t         user_card_time;
static int32_t          user_card_energy;

static uint8_t          pers_sec;

static LcdMngEvent_en   pers_LcdMngEvent;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local function prototypes -------------------------------------------------------------------------------------------------------------- //
static xQueueHandle getPersMngQueueHandle(void);

static void pers_day_week_update(struct DataAndTime_t *locDateTime);
static void PersMngTimCallBack(TimerHandle_t pxTimer);
static void pers_set_timer(PersMngTim_en timer, uint32_t set_time);
static uint8_t user_card_credit_decrease(uint8_t *block_ptr);
static uint8_t master_settings_reset(uint8_t *block_ptr);
static void pers_bcd08_to_dec(uint8_t *src_ptr, uint8_t *dst_ptr);
static void pers_wlist_update_set(pers_state_en *state_set);
static void pers_wlist_delete_set(pers_state_en *state_set);
static void pers_uid_delete_set(pers_state_en *state_set);
static void pers_uid_confirm_set(pers_state_en *state_set);
static void pers_blind_set(pers_state_en *state_set);
static uint8_t master_format_check(uint8_t *block_ptr);
static uint8_t master_uid_check(uint8_t *uid_ptr);
static void master_uid_update(uint8_t *uid_ptr);
static void master_operation_execute(uint8_t *block_ptr);
static void user_card_auth_read(card_auth_en *card_mode, uint8_t *block_ptr);
static void master_uid_reset(void);
static uint8_t whitelist_check(uint8_t *uid_ptr);
static void whitelist_update(uint8_t *uid_ptr, white_list_operation_en white_list_operation);
static void whitelist_reset(void);
static void current_uid_update(uint8_t *uid_ptr, uint8_t *block_ptr);
static uint8_t current_uid_match(uint8_t *uid_ptr, uint8_t *block_ptr);
static void current_uid_reset(void);
static void pers_mode_set(evs_mode_en mode);
static void pers_event_save(PersMngMsg_st *pMsg);
static void pers_rtc_backup_set(uint32_t reg, uint32_t val);
static void PersManager_init(void);
static void PersManager(PersMngMsg_st *pMsg);
static uint32_t pers_rtc_backup_get(uint32_t reg);
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  getPersMngQueueHandle
//  
//  DESCRIPTION:    -
//  
//  INPUT:          none
//  
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static xQueueHandle getPersMngQueueHandle(void)
{
return(PersMngQueue);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pers_rtc_backup_get
//
//  DESCRIPTION:    legge parametri della ricarica per la gestione degli stati di low power mode
//
//  INPUT:          status
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint32_t pers_rtc_backup_get(uint32_t reg)
{
return HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)getHandleRtc(), reg);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pers_day_week_update
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void pers_day_week_update(struct DataAndTime_t *locDateTime)
{
uint16_t i, calc, abs_day;

abs_day = locDateTime->Day;

for (i=1; i<locDateTime->Month; i++)
	abs_day += pers_days_of_month[i];

if ((locDateTime->Month > 2) && ((locDateTime->Year % 4) == 0))	// anno bisestile
	abs_day += 1;

calc = (locDateTime->Year + ((locDateTime->Year - 1) / 4) - ((locDateTime->Year - 1) / 100) + ((locDateTime->Year - 1) / 400) + abs_day);

locDateTime->DayWeek = (calc % 7) + 6;

if (locDateTime->DayWeek > 7)
	locDateTime->DayWeek -= 7;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  PersMngTimCallBack
//
//  DESCRIPTION:    callback to manager timers   
//
//  INPUT:          TimerHandle_t: the elapsed timer 
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void PersMngTimCallBack(TimerHandle_t pxTimer)
{
uint32_t    timer_id;

timer_id = (uint32_t)pvTimerGetTimerID(pxTimer);                                    // find the led  which the timer is referred

if (timer_id == (uint32_t)(UPDATE_WLIST_TIM))                                       // check if timer exist
    {
    if (pers_sec > 0)
        {
        pers_set_timer(UPDATE_WLIST_TIM, PERS_SEC_TIME);                            // reload timer

        if (pers_state == PERS_UPDATE_WLIST)
            {
            pers_sec --;
            send_to_lcd(PERS_TIME_EXPIRED);
            }
        }
    else
        send_to_pers(UPDATE_WLIST_TIM_EXPIRED);
    }

if (timer_id == (uint32_t)(UID_CONFIRM_TIM))                                         // check if timer exist
    {
    if (pers_sec > 0)
        {
        pers_set_timer(UID_CONFIRM_TIM, PERS_SEC_TIME);                              // reload timer

        if (pers_state == PERS_UID_CONFIRM)
            pers_sec --;
        }
    else
        send_to_pers(UID_CONFIRM_TIM_EXPIRED);
    }

if (timer_id == (uint32_t)(UID_DELETE_TIM))                                         // check if timer exist
    {
    if (pers_sec > 0)
        {
        pers_set_timer(UID_DELETE_TIM, PERS_SEC_TIME);                              // reload timer

        if (pers_state == PERS_UID_DELETE)
            {
            pers_sec --;
            send_to_lcd(PERS_TIME_EXPIRED);
            }
        }
    else
        send_to_pers(UID_DELETE_TIM_EXPIRED);
    }

if (timer_id == (uint32_t)(WLIST_DELETE_TIM))                                       // check if timer exist
    {
    pers_set_timer(UPDATE_WLIST_TIM, PERS_SEC_TIME);

    if (pers_sec > 0)
        {
        pers_set_timer(WLIST_DELETE_TIM, PERS_SEC_TIME);                            // reload timer

        if (pers_state == PERS_WLIST_DELETE)
            {
            pers_sec --;
            send_to_lcd(PERS_TIME_EXPIRED);
            }
        }
    else
        send_to_pers(WLIST_DELETE_TIM_EXPIRED);
    }

if (timer_id == (uint32_t)(PERS_BLIND_TIM))                                         // check if timer exist
    send_to_pers(PERS_BLIND_TIM_EXPIRED);

if (timer_id == (uint32_t)(PERS_DATA_TIM))                                         // check if timer exist
    send_to_pers(PERS_DATA_TIM_EXPIRED);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pers_set_timer
//
//  DESCRIPTION:    imposta il timer selezionato
//
//  INPUT:          timer da aggiornare: timer; tempo da impostare: set_time
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void pers_set_timer(PersMngTim_en timer, uint32_t set_time)
{
while ((xTimerChangePeriod (xPersMngTimers[timer], set_time, PERS_GARD_TIME) != pdPASS));  // set timer
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  send_to_pers
//  
//  DESCRIPTION:    impacchetta l'evento da inviare a PersMngTask
//  
//  INPUT:          valore di PersMngEvent
//  
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void send_to_pers(uint8_t pers_event)
{
PersMngMsg_st   msgPersSend;

msgPersSend.PersMngEvent = (PersMngEvent_en)(pers_event);
configASSERT(xQueueSendToBack(getPersMngQueueHandle(), (void *)&msgPersSend, portMAX_DELAY) == pdPASS);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pers_sec_get
//
//  DESCRIPTION:    ritorna il valore dei secondi corrente per il timeout di stato di pers_state
//
//  INPUT:          puntatore destinatario: dst_ptr
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void pers_sec_get(uint8_t *dst_ptr)
{
*dst_ptr = pers_sec;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  user_card_date_get
//
//  DESCRIPTION:    ritorna la data di scadenza presente sulla carta utente a scadenza
//
//  INPUT:          puntatore destinatario: dst_ptr
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void user_card_date_get(uint8_t *dst_ptr)
{
uint8_t i;

for (i=0; i<4; i++)
    *(dst_ptr + i) = user_card_date[i];
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  user_card_credit_get
//
//  DESCRIPTION:    ritorna il valore dei crediti presenti sulla carta utente a crediti
//
//  INPUT:          puntatore destinatario: dst_ptr
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void user_card_credit_get(uint16_t *dst_ptr)
{
*dst_ptr = user_card_credit;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  user_card_auth_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
card_auth_en user_card_auth_get(void)
{
return user_card_mode;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  user_card_time_decrease
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint32_t user_card_time_decrease(void)
{
if (user_card_time > 0)
    user_card_time --;

pers_rtc_backup_set(BACKUP_USER_CARD_TIME, user_card_time);

return user_card_time;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  user_card_time_decrease
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
int32_t user_card_energy_get(void)
{
return user_card_energy;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  user_card_credit_decrease
//
//  DESCRIPTION:    aggiorna il valore dei crediti presenti sulla carta utente a crediti
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t user_card_credit_decrease(uint8_t *block_ptr)
{
uint8_t ret = 0;

user_card_credit --;
            
*(block_ptr + 14) = (uint8_t)(user_card_credit >> 8);
*(block_ptr + 15) = (uint8_t)(user_card_credit & 0x00FF);
            
if (rfid_sl030_data_push(CMD_BLOCK_WRITE, block_ptr, 8) == SL030_OPERATION_SUCCEED) // write block 8
    ret = 1;

return ret;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  master_settings_reset
//
//  DESCRIPTION:    cancella i byte di aggiornamento dati sulla carta master [N0; N1]
//
//  INPUT:          none
//
//  OUTPUT:         1 = ok
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t master_settings_reset(uint8_t *block_ptr)
{
uint8_t ret = 0;

user_card_credit --;
            
*(block_ptr + 1) = 0xFF;
*(block_ptr + 2) = 0xFF;
            
if (rfid_sl030_data_push(CMD_BLOCK_WRITE, block_ptr, 8) == SL030_OPERATION_SUCCEED) // write block 8
    ret = 1;

return ret;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pers_bcd08_to_dec
//
//  DESCRIPTION:    converte in decimale il byte bcd di ingresso [max 2 cifre]
//
//  INPUT:          puntatore sorgente: src_ptr; puntatore destinatario: dst_ptr
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void pers_bcd08_to_dec(uint8_t *src_ptr, uint8_t *dst_ptr)
{
uint8_t data08u = *src_ptr;

*dst_ptr = (((data08u & 0xF0) >> 4) * 10);
*dst_ptr += (data08u & 0x0F);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pers_wlist_update_set
//
//  DESCRIPTION:    imposta parametri per pers_state = PERS_UPDATE_WLIST
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void pers_wlist_update_set(pers_state_en *state_set)
{
pers_sec = UPDATE_WLIST_SEC;
*state_set = PERS_UPDATE_WLIST;
pers_set_timer(UPDATE_WLIST_TIM, PERS_SEC_TIME);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pers_wlist_delete_set
//
//  DESCRIPTION:    imposta parametri per pers_state = PERS_WLIST_DELETE
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void pers_wlist_delete_set(pers_state_en *state_set)
{
pers_sec = UID_DELETE_SEC;
*state_set = PERS_WLIST_DELETE;
pers_set_timer(WLIST_DELETE_TIM, PERS_SEC_TIME);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pers_uid_delete_set
//
//  DESCRIPTION:    imposta parametri per pers_state = PERS_UID_DELETE
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void pers_uid_delete_set(pers_state_en *state_set)
{
pers_sec = UID_DELETE_SEC;
*state_set = PERS_UID_DELETE;
pers_set_timer(UID_DELETE_TIM, PERS_SEC_TIME);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pers_uid_confirm_set
//
//  DESCRIPTION:    imposta parametri per pers_state = PERS_UID_CONFIRM
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void pers_uid_confirm_set(pers_state_en *state_set)
{
pers_sec = UID_CONFIRM_SEC;
*state_set = PERS_UID_CONFIRM;
pers_set_timer(UID_CONFIRM_TIM, PERS_SEC_TIME);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pers_blind_set
//
//  DESCRIPTION:    imposta parametri per pers_state = PERS_MODE_BLIND
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void pers_blind_set(pers_state_en *state_set)
{
*state_set = PERS_MODE_BLIND;
pers_set_timer(PERS_BLIND_TIM, MODE_BLIND_TIME);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  master_format_check
//
//  DESCRIPTION:    controlla il formato dell'informazione contenuta nella carta letta [new_block_array]
//
//  INPUT:          puntatore blocco info [new_block_array]: block_ptr
//
//  OUTPUT:         1 = carta master; 0 = carta utente
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t master_format_check(uint8_t *block_ptr)
{
uint8_t i;

for (i=0; i<MASTER_FORMAT_DIM; i++)
    {
    if (*(block_ptr + 10 + i) != master_format_array[i])                            // confronto
        return 0;
    }

return 1;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  master_uid_check
//
//  DESCRIPTION:    confronta uid carta master letta con quello salvato in eeprom / ram registrato
//
//  INPUT:          puntatore uid carta letta: uid_ptr
//
//  OUTPUT:         1 = carta master riconosciuta; 0 = carta master non riconosciuta
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t master_uid_check(uint8_t *uid_ptr)
{
uint8_t i;

for (i=0; i<CARD_UID_DIM; i++)
    {
    if (*(uid_ptr + i) != master_uid_array[i])                                      // confronto
        return 0;
    }

return 1;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  master_uid_update
//
//  DESCRIPTION:    aggiorna in eeprom / ram uid carta master
//
//  INPUT:          puntatore uid carta letta: uid_ptr
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void master_uid_update(uint8_t *uid_ptr)
{
uint8_t i;
uint8_t *master_uid = master_uid_array;

for (i=0; i<CARD_UID_DIM; i++)
    *(master_uid + i) = *(uid_ptr + i);                         // aggiorna master_uid_array in ram

eeprom_param_set(MASTER_UID00_EADD, master_uid, CARD_UID_DIM); // aggiornamento in eeprom di master_uid_array

master_card_reg |= 0x01;
eeprom_param_set(PERS_MASTER_EADD, &master_card_reg, 1);       // aggiornamento in eeprom di flag master_uid_array registrato

WriteOnEeprom((EDATA_DEFAULT_EADD + PERS_MASTER_EADD), &master_card_reg, 1);        // parametro da salvare come default factory
WriteOnEeprom((EDATA_DEFAULT_EADD + MASTER_UID00_EADD), master_uid, CARD_UID_DIM); // parametro da salvare come default factory
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  master_format_check
//
//  DESCRIPTION:    -
//
//  INPUT:          puntatore blocco info [new_block_array]: block_ptr
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void master_operation_execute(uint8_t *block_ptr)
{
uint8_t                 data08u[4];
uint16_t                data16u;
struct DataAndTime_t    locDateTime;
uint16_t                card_operation = ((uint16_t)(*(block_ptr + 1)) << 8) + *(block_ptr + 2);

if (card_operation == 0xFFFF)                                   // aggiorna white list
    {
    eeprom_param_get(PERS_UIDNUM_EADD, &uid_map_num, 1);
    pers_wlist_update_set(&pers_state);
    send_to_lcd(LCD_UPDATE_WLIST);
    }
else if ((card_operation == 0x7007)                             // aggiorna data e ora
      || (card_operation == 0x7008))                            // aggiorna data, ora e parametri delle funzioni speciali [lingua, PMNG, ecc.]
    {
    if (master_settings_reset(block_ptr) == 0)
        send_to_lcd(LCD_NEW_DATE_FAILED);
    else
        {
        if (gsy_connected_get() == 0)                           // se la sbc non è connessa, aggiorna data e ora da carta master
            {
            pers_bcd08_to_dec((block_ptr + 3), &locDateTime.Second);
            pers_bcd08_to_dec((block_ptr + 4), &locDateTime.Minute);
            pers_bcd08_to_dec((block_ptr + 5), &locDateTime.Hour);
            pers_day_week_update(&locDateTime);
            pers_bcd08_to_dec((block_ptr + 6), &locDateTime.DayWeek);
            pers_bcd08_to_dec((block_ptr + 7), &locDateTime.Day);
            pers_bcd08_to_dec((block_ptr + 8), &locDateTime.Month);
            pers_bcd08_to_dec((block_ptr + 9), (block_ptr + 9));

            locDateTime.Year = (*(block_ptr + 9) + 2000);

            /* set new date and time */
            setDateTimeWithTimeZone((struct DataAndTime_t*)&locDateTime);

            data08u[0] = 1;
            EEPROM_Save_Config (RTC_VALID_EADD, data08u, 1);
            }

        if (card_operation == 0x7008)
            {
            rfid_new_block_get(block_ptr, 1);

            lcd_language_config(block_ptr);

            eeprom_param_get(HIDDEN_MENU_VIS_EADD, &data08u[1], 1);
            eeprom_param_get(HIDDEN_MENU_ENB_EADD, &data08u[0], 1);

            if (*(block_ptr + 5) & 0x01)
                {
                data08u[0] |= HIDDEN_MENU_PMNG_ENB;
                data08u[1] |= HIDDEN_MENU_PMNG_VIS;
                data08u[2] = PMNG_FULL;
                EEPROM_Save_Config (PMNG_MODE_EADD, &data08u[2], 1);
                }
            else
                data08u[0] &=~ HIDDEN_MENU_PMNG_ENB;

            EEPROM_Save_Config (HIDDEN_MENU_ENB_EADD, &data08u[0], 1);
            EEPROM_Save_Config (HIDDEN_MENU_VIS_EADD, &data08u[1], 1);

            if (*(block_ptr + 5) & 0x02)
                data08u[0] = PMNG_UNBAL_ON;
            else
                data08u[0] = PMNG_UNBAL_OFF;

            EEPROM_Save_Config (PMNG_UNBAL_EADD, data08u, 1);
            EEPROM_Save_Config (CONTROL_BYTE2_EADD, data08u, 1);

            if (*(block_ptr + 5) & 0x10)
                data08u[0] |= (EMETER_EXT_CRL2 | SINAPSI_CHN2_CRL2);
            else
                data08u[0] &=~ (EMETER_EXT_CRL2 | SINAPSI_CHN2_CRL2);

            //eeprom_param_get(PMNG_MODE_EADD, &data08u[1], 1);
            EEPROM_Save_Config (HIDDEN_MENU_VIS_EADD, &data08u[2], 1);
            EEPROM_Save_Config (HIDDEN_MENU_ENB_EADD, &data08u[1], 1);
            
            if (data08u[1] == 1)
                {
                data08u[1] |= HIDDEN_MENU_PMNG_ENB;
                data08u[2] |= HIDDEN_MENU_PMNG_VIS;
                data08u[3] = PMNG_FULL;
                EEPROM_Save_Config (PMNG_MODE_EADD, &data08u[3], 1);
                }
            else
                data08u[1] &=~ HIDDEN_MENU_PMNG_ENB;
            
            EEPROM_Save_Config (HIDDEN_MENU_ENB_EADD, &data08u[1], 1);
            EEPROM_Save_Config (HIDDEN_MENU_VIS_EADD, &data08u[2], 1);
            EEPROM_Save_Config (CONTROL_BYTE2_EADD, data08u, 1);
            setPmEmexInModbus(data08u[0]);

            if (*(block_ptr + 5) & 0x20)
            {
              eeprom_param_get(HIDDEN_MENU_VIS_EADD, &data08u[0], 1);
              data08u[0] |= HIDDEN_MENU_TMEG_VIS;
              eeprom_param_set(HIDDEN_MENU_VIS_EADD, &data08u[0], 1);
              data08u[0] = 1;
              WriteOnEeprom (HIDDEN_MENU_VIS_EADD, &data08u[0], 1);
            }
            else
            {
              eeprom_param_get(HIDDEN_MENU_VIS_EADD, &data08u[0], 1);
              data08u[0] &= (~HIDDEN_MENU_TMEG_VIS);
              eeprom_param_set(HIDDEN_MENU_VIS_EADD, &data08u[0], 1);
              data08u[0] = 0;
              WriteOnEeprom (HIDDEN_MENU_VIS_EADD, &data08u[0], 1);
            }

            EEPROM_Save_Config (TCHARGE_MODE_EADD, data08u, 1);
            
            if (*(block_ptr + 5) & 0x40)
            {
                data08u[0] = 1;
            }
            else
            {
                data08u[0] = 0;
            }

            EEPROM_Save_Config (PMNG_TRANGE_EADD, data08u, 1);
            
            data16u = ((((uint16_t)(*(block_ptr + 8)) << 8) + *(block_ptr + 7) + 50) / 100);
            data08u[0] = (uint8_t)(data16u & 0x00FF);
            data08u[1] = (uint8_t)(data16u >> 8);
            EEPROM_Save_Config (PMNG_PWRLSB_EADD, data08u, 2);
              
//            data08u[0] = ((*(block_ptr + 9) + 5) / 10);
            data08u[0] = *(block_ptr + 9);
            EEPROM_Save_Config (PMNG_CURRENT_EADD, data08u, 1);
            
            data08u[0] = *(block_ptr + 10) - 1;
            EEPROM_Save_Config (PMNG_MULTIP_EADD, data08u, 1);

            if (*(block_ptr + 11) <= 20)
                data08u[0] = *(block_ptr + 11);
            else
                data08u[0] = 20;

            EEPROM_Save_Config (PMNG_ERROR_EADD, data08u, 1);
            EEPROM_Save_Config (PMNG_DMAX_EADD, (block_ptr + 12), 1);

            if ((*(block_ptr + 13) > 0) && (*(block_ptr + 13) < 36))
                data08u[0] = *(block_ptr + 13);
            else
                data08u[0] = 2;

            EEPROM_Save_Config (TCHARGE_TIME_EADD, data08u, 1);
#ifdef CH_TIME_AUTO
            /* excluded by Nick 19/03/2023 */
            if (data08u[0] != 0)
                {
                data08u[0] = 1;
                eeprom_param_set(TCHARGE_MODE_EADD, data08u, 1);
                }

            eeprom_param_get(HIDDEN_MENU_VIS_EADD, &data08u[0], 1);
            data08u[0] |= HIDDEN_MENU_TMEG_VIS;
            eeprom_param_set(HIDDEN_MENU_VIS_EADD, &data08u[0], 1);
#endif
            
#ifdef LCD_TYPE_SET
            data08u[0] = *(block_ptr + 14);
            eeprom_param_set(LCD_TYPE_EADD, data08u, 1);
#endif
            send_to_evs(EVS_EXTERNAL_EM_UPDATE);
            }

        send_to_lcd(LCD_NEW_DATE_DONE);
        }

    pers_state_next = PERS_MODE_ACTIVE;
    pers_blind_set(&pers_state);
    }
else if (card_operation == 0xA55A)                              // cancella white list
    {
    pers_wlist_delete_set(&pers_state);
    send_to_lcd(LCD_DELETE_WLIST_CONFIRM);
    }
else    // errore
    {
    pers_state_next = PERS_MODE_ACTIVE;
    pers_blind_set(&pers_state);
    send_to_lcd(LCD_UID_ERROR);
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  user_card_auth_read
//
//  DESCRIPTION:    controlla il contenuto informativo del blocco 8 per autorizzare la riarica
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void user_card_auth_read(card_auth_en *card_mode, uint8_t *block_ptr)
{
uint8_t                 i, data08u, data_array[4];
uint16_t                mode;
struct DataAndTime_t    locDateTime;

*card_mode = NULL_AUTH;

mode = ((uint16_t)(*(block_ptr + 8)) << 8) + *(block_ptr + 13);

if (mode == 0xFFFF)
    *card_mode |= NO_LIMIT_AUTH;

if ((mode & 0xFF00) == 0xA500)      // check data di scadenza
    {
    for (i=0; i<4; i++)
        user_card_date[i] = *(block_ptr + 9 + i);

    DateTimeGet(&locDateTime);
    user_card_date_get(data_array);

    data08u = (uint8_t)(locDateTime.Year % 2000);
    pers_bcd08_to_dec(&data_array[3], &data_array[3]);
    pers_bcd08_to_dec(&data_array[2], &data_array[2]);
    pers_bcd08_to_dec(&data_array[1], &data_array[1]);
    pers_bcd08_to_dec(&data_array[0], &data_array[0]);
    
    if ((data_array[3] < data08u)
    || ((data_array[3] == data08u) && (data_array[2] < (uint8_t)(locDateTime.Month)))
    || ((data_array[3] == data08u) && (data_array[2] == (uint8_t)(locDateTime.Month)) && (data_array[1] < (uint8_t)(locDateTime.Day)))
    || ((data_array[3] == data08u) && (data_array[2] == (uint8_t)(locDateTime.Month)) && (data_array[1] == (uint8_t)(locDateTime.Day)) && (data_array[0] < (uint8_t)(locDateTime.Hour))))
        {
        *card_mode = EXPIRED_CARD;
        return;
        }
    else
        *card_mode |= DATE_CHARGE_AUTH;
    }

if ((mode & 0x00FF) == 0x00A5)      // check numero crediti
    {
    user_card_credit = ((uint16_t)(*(block_ptr + 14)) << 8) + *(block_ptr + 15);

    if (user_card_credit == 0)
        {
        *card_mode = CREDIT_EXHAUSTED;
        return;
        }
    else
        *card_mode |= CREDIT_CHARGE_AUTH;
    }

if ((*(block_ptr + 4) == 0xB3) && (*(block_ptr + 5) != 0xFF))
    {
    user_card_time = ((uint32_t)(*(block_ptr + 5)) * 900) + 1;
    *card_mode |= TIME_CHARGE_AUTH;
    send_to_evstime(EVSTIME_CARD_TIME_UPDATE);
    }

if ((*(block_ptr + 4) == 0xB3) && (*(block_ptr + 6) != 0xFF) && (*(block_ptr + 7) != 0xFF))
    {
    user_card_energy = (int32_t)(((uint16_t)(*(block_ptr + 6)) << 8) + *(block_ptr + 7));
    pers_rtc_backup_set(BACKUP_USER_CARD_ENERGY, (uint32_t)(user_card_energy));
    *card_mode |= ENERGY_CHARGE_AUTH;
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  master_uid_reset
//
//  DESCRIPTION:    reset master_uid_array
//                  
//  INPUT:          none
//                  
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void master_uid_reset(void)
{
uint8_t i;

for (i=0; i<CARD_UID_DIM; i++)
    master_uid_array[i] = 0xFF;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  whitelist_check
//
//  DESCRIPTION:    confronta l'uid entrante con quelli salvati in white list.
//                  Se esiste, aggiorna uid corrente [current_uid_array] e salva in ram la posizione nella mappa a bit degli uid [uid_map_array]
//
//  INPUT:          puntatore uid carta letta: uid_ptr
//
//  OUTPUT:         1 = uid entrante presente in white list e uid corrente aggiornato [current_uid_array]; 0 = uid entrante non presente
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t whitelist_check(uint8_t *uid_ptr)
{
uint8_t byte, bit, k, read_bit, uid_match;

eeprom_uid_map_get(uid_map_array);

for (byte=0; byte<USER_MAP_EEDIM; byte++)
    {
    read_bit = 0x01;                                                // imposta il controllo del primo bit del byte-esimo byte di uid_map_array

    for (bit=0; bit<8; bit++)
        {
        if ((uid_map_array[byte] & read_bit) == read_bit)           // all'indirizzo corrispondente al bit del byte-esimo byte di uid_map_array esiste un uid registrato
            {
            eeprom_uid_reg_get((((uint16_t)(byte) * 8 * CARD_UID_DIM) + (bit * CARD_UID_DIM)), eeprom_uid_array);   // carica uid registrato all'indirizzo byte, bit
            
            uid_match = 1;                                          // preimposta esito positivo del confronto

//            for (k=0; k<CARD_UID_DIM; k++)
            for (k=0; k<4; k++)
                {
                if (*(uid_ptr + k) != eeprom_uid_array[k])    // confronta uid registrato con quello entrante
                    uid_match = 0;
                }
            
            for (k=0; k<CARD_UID_DIM; k++)
                eeprom_uid_array[k] = 0xFF;
            
            if (uid_match == 1)                                     // confronto positivo
                {
                uid_map_byte = byte;                                // salva le coordinate di uid_map_array dell'uid letto
                uid_map_bit = bit;
                return 1;
                }
            }
        
        read_bit <<= 1;
        }
    }

return 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  whitelist_update
//
//  DESCRIPTION:    Salva o cancella in eeprom l'uid corrente [current_uid_array] salvato nella precedente whitelist_check
//                  Aggiorna in ram [uid_map_array] e in eeprom le coordiante [uid_map_byte, uid_map_bit] salvate nella precedente whitelist_check
//
//  INPUT:          operazione da eseguire: white_list_operation
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void whitelist_update(uint8_t *uid_ptr, white_list_operation_en white_list_operation)
{
uint8_t     esc, bit, write_bit, byte_val;
uint16_t    eadd;

if (white_list_operation == UID_ADD)
    {
    esc = 0;
    uid_map_byte = 0;

    while ((esc == 0) && (uid_map_byte < USER_MAP_EEDIM))
        {
        write_bit = 0x01;                                               // imposta il controllo del primo bit del byte-esimo byte di uid_map_array
        
        for (uid_map_bit=0; uid_map_bit<8; uid_map_bit++)
            {
            if ((uid_map_array[uid_map_byte] & write_bit) == 0)         // all'indirizzo corrispondente al bit del byte-esimo byte di uid_map_array lo spazio è disponibile
                {
                uid_map_array[uid_map_byte] |= write_bit;               // set bit
                esc = 1;
                break;
                }

            write_bit <<= 1;
            }
        
        if (esc == 0)
            uid_map_byte ++;
        }

    uid_map_num ++;
    }
else    // if (white_list_operation == UID_DELETE)
    {
    for (byte_val=0; byte_val<CARD_UID_DIM; byte_val++)
        *(uid_ptr + byte_val) = 0xFF;

    byte_val = uid_map_array[uid_map_byte];
    write_bit = 0x01;
    
    for (bit=0; bit<uid_map_bit; bit++)
        write_bit <<= 1;

    uid_map_array[uid_map_byte] &=~ write_bit; // reset bit
    uid_map_num --;
    }

// l'indirizzo eeprom si ricava dalla posizione del uid_map_bit-esimo bit nel uid_map_byte-esimo byte
eadd = USER_UID_EEOFFSET + ((uint16_t)(uid_map_byte) * 8 * CARD_UID_DIM) + (uid_map_bit * CARD_UID_DIM);

EEPROM_Save_Config (PERS_UIDNUM_EADD, &uid_map_num, 1);
EEPROM_Save_Config (USER_MAP00_EADD, uid_map_array, USER_MAP_EEDIM);
EEPROM_Save_Config (eadd, uid_ptr, CARD_UID_DIM);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  whitelist_reset
//
//  DESCRIPTION:    Cancella tutti gli uid registrati
//
//  INPUT:          operazione da eseguire: white_list_operation
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void whitelist_reset(void)
{
uint8_t i;

uid_map_num = 0;;

for (i=0; i<USER_MAP_EEDIM; i++)
    uid_map_array[i] = 0x00;

EEPROM_Save_Config (PERS_UIDNUM_EADD, &uid_map_num, 1);
EEPROM_Save_Config (USER_MAP00_EADD, uid_map_array, USER_MAP_EEDIM);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  current_uid_update
//
//  DESCRIPTION:    aggiorna in ram uid letto [current_uid_array]
//
//  INPUT:          puntatore uid carta letta: uid_ptr; replica uid su blocco 8: block_ptr
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void current_uid_update(uint8_t *uid_ptr, uint8_t *block_ptr)
{
uint8_t i;

for (i=0; i<CARD_UID_DIM; i++)
    current_uid_array[i] = *(uid_ptr + i);

for (i=0; i<CARD_UID_DIM; i++)
    current_block_array[i] = *(block_ptr + i);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  current_uid_match
//
//  DESCRIPTION:    confronta uid letto con quello corrente [current_uid_array]
//
//  INPUT:          puntatore uid carta letta: uid_ptr; replica uid su blocco 8: block_ptr
//
//  OUTPUT:         1 = confronto positivo; 0 = confronto negativo
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t current_uid_match(uint8_t *uid_ptr, uint8_t *block_ptr)
{
uint8_t i, ret;

ret = 1;

for (i=0; i<CARD_UID_DIM; i++)
    {
    if (*(uid_ptr + i) != current_uid_array[i])
        ret = 0;
    }

for (i=0; i<CARD_UID_DIM; i++)
    {
    if (*(block_ptr + i) != current_block_array[i])
        ret = 0;
    }

return ret;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  current_uid_reset
//
//  DESCRIPTION:    reset current_uid_array
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void current_uid_reset(void)
{
uint8_t i;

user_card_mode = NULL_AUTH;

for (i=0; i<CARD_UID_DIM; i++)
    current_uid_array[i] = 0xFF;

for (i=0; i<CARD_UID_DIM; i++)
    current_block_array[i] = 0xFF;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pers_mode_set
//
//  DESCRIPTION:    aggiornamento in eeprom di mode
//
//  INPUT:          nuovo mode: mode [EVS_PERS_MODE o EVS_FREE_MODE]
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void pers_mode_set(evs_mode_en mode)
{
evs_mode_en pers_mode = mode;

EEPROM_Save_Config (EVS_MODE_EADD, (uint8_t*)(&pers_mode), 1);
send_to_evs(EVS_AUTORIZATION_MODE);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pers_event_save
//
//  DESCRIPTION:    si aggiornano variabili dipendenti da eventi eventualmente non gestiti nello stato corrente
//
//  INPUT:          puntatore PersMngEvent: pMsg
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void pers_event_save(PersMngMsg_st *pMsg)
{
if (pMsg->PersMngEvent == PERS_PULS_SET)
    pers_puls_held = 1;
else if (pMsg->PersMngEvent == PERS_PULS_RESET)
    pers_puls_held = 0;

if (pers_state != PERS_MODE_ACTIVE)
    pers_puls_held = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pers_rtc_backup_set
//
//  DESCRIPTION:    salva parametri della ricarica per la gestione degli stati di low power mode
//
//  INPUT:          status
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void pers_rtc_backup_set(uint32_t reg, uint32_t val)
{
HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), reg, val);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  modbus_uid_write
//
//  DESCRIPTION:    -
//
//  INPUT:          uid array pointer;  
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void modbus_uid_write(uid_type_en uid_type, uint8_t *uid_ptr, uid_code_en uid_code)
{
uint8_t     i, uid_array[CARD_UID_DIM];
uint8_t*    pBuff;

if (uid_code == ASCII_UID)
    {
    for (i=0; i<CARD_UID_DIM; i++)
        {
        if (*(uid_ptr + i) >= 'A')
            *(uid_ptr + i) -= 0x37;
        else
            *(uid_ptr + i) -= 0x30;
    
       	if (i & 0x01)
       	    uid_array[(i >> 1)] = *(uid_ptr + i - 1) + *(uid_ptr + i);
       	else
       	    *(uid_ptr + i) <<= 4;
        }
    }

for (i=4; i<CARD_UID_DIM; i++)
   	uid_array[i] = 0x00;

if (uid_type == UID_MASTER)
	{
	for (i=0; i<CARD_UID_DIM; i++)
    	master_uid_array[i] = uid_array[i];

	eeprom_param_set(MASTER_UID00_EADD, master_uid_array, CARD_UID_DIM);

        master_card_reg |= 0x01;
        eeprom_param_set(PERS_MASTER_EADD, &master_card_reg, 1);       // aggiornamento in eeprom di flag master_uid_array registrato

        WriteOnEeprom((EDATA_DEFAULT_EADD + PERS_MASTER_EADD), &master_card_reg, 1);        // parametro da salvare come default factory
        WriteOnEeprom((EDATA_DEFAULT_EADD + MASTER_UID00_EADD), master_uid_array, CARD_UID_DIM); // parametro da salvare come default factory
	}
else    // if (uid_type == UID_USER)
    {
    eeprom_param_get(PERS_UIDNUM_EADD, &uid_map_num, 1);
    eeprom_uid_map_get(uid_map_array);

    if (uid_map_num < USER_UID_EENUM)
        whitelist_update(uid_array, UID_ADD);

    master_card_reg |= 0x10;
    eeprom_param_set(PERS_MASTER_EADD, &master_card_reg, 1);                                // aggiornamento in eeprom di flag user_uid registrato

    WriteOnEeprom((EDATA_DEFAULT_EADD + PERS_MASTER_EADD), &master_card_reg, 1);            // parametro da salvare come default factory

    WriteOnEeprom((EDATA_DEFAULT_EADD + PERS_UIDNUM_EADD), &uid_map_num, 1);                // parametro da salvare come default factory
    WriteOnEeprom((EDATA_DEFAULT_EADD + USER_MAP00_EADD), uid_map_array, USER_MAP_EEDIM);   // parametro da salvare come default factory

    pBuff = (uint8_t*)malloc(USER_UID_BYTE_NUM);

    ReadFromEeprom(USER_UID_EEOFFSET, pBuff, USER_UID_BYTE_NUM);
    WriteOnEeprom((EDATA_DEFAULT_EADD + USER_UID_EEOFFSET), pBuff, USER_UID_BYTE_NUM);      // aggiorno user uid factory default
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  manual_uid_factory
//
//  DESCRIPTION:    -
//
//  INPUT:          none;  
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void manual_uid_factory(void)
{
uint8_t*    pBuff;

eeprom_param_get(PERS_MASTER_EADD, &master_card_reg, 1);
eeprom_master_uid_get(master_uid_array);
eeprom_param_get(PERS_UIDNUM_EADD, &uid_map_num, 1);
eeprom_uid_map_get(uid_map_array);

if (master_card_reg & 0x01)
    {
    master_card_reg &=~ 0x10;
    
    if (uid_map_num > 0)
        master_card_reg |= 0x10;

    WriteOnEeprom((EDATA_DEFAULT_EADD + PERS_MASTER_EADD), &master_card_reg, 1);                // parametro da salvare come default factory
    WriteOnEeprom((EDATA_DEFAULT_EADD + MASTER_UID00_EADD), master_uid_array, CARD_UID_DIM);    // parametro da salvare come default factory
    WriteOnEeprom((EDATA_DEFAULT_EADD + PERS_UIDNUM_EADD), &uid_map_num, 1);                    // parametro da salvare come default factory
    WriteOnEeprom((EDATA_DEFAULT_EADD + USER_MAP00_EADD), uid_map_array, USER_MAP_EEDIM);       // parametro da salvare come default factory

    pBuff = (uint8_t*)malloc(USER_UID_BYTE_NUM);

    ReadFromEeprom(USER_UID_EEOFFSET, pBuff, USER_UID_BYTE_NUM);
    WriteOnEeprom((EDATA_DEFAULT_EADD + USER_UID_EEOFFSET), pBuff, USER_UID_BYTE_NUM);          // aggiorno user uid factory default
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  PersManager_init
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void PersManager_init(void)
{
pers_puls_held = 0;
current_uid_reset();
pers_state = PERS_IDLE_STATE;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  PersManager
//
//  DESCRIPTION:    -
//
//  INPUT:          puntatore PersMngEvent: pMsg
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void PersManager(PersMngMsg_st *pMsg)
{
uint32_t        data32;
uint16_t        card_operation;
uint8_t         *new_uid, *new_block;
sl030_type_en   new_type;
evs_mode_en     pers_mode;

new_uid = new_uid_array;
new_block = new_block_array;

eeprom_param_get(EVS_MODE_EADD, (uint8_t*)(&pers_mode), 1);

pers_event_save(pMsg);

if (pMsg->PersMngEvent == PERS_CARD_READ)
    {
    rfid_uid_get(new_uid, &new_type);
    rfid_new_block_get(new_block, 0);
    }

if (pMsg->PersMngEvent == PERS_RTC_BACKUP_SAVE)
    {
    data32 = ((uint32_t)current_uid_array[3] << 24) + ((uint32_t)current_uid_array[2] << 16) + ((uint32_t)current_uid_array[1] << 8) + ((uint32_t)current_uid_array[0] << 0);
    pers_rtc_backup_set(BACKUP_USER_UID_0003, data32);
    
    data32 = ((uint32_t)current_uid_array[7] << 24) + ((uint32_t)current_uid_array[6] << 16) + ((uint32_t)current_uid_array[5] << 8) + ((uint32_t)current_uid_array[4] << 0);
    pers_rtc_backup_set(BACKUP_USER_UID_0407, data32);
    
    data32 = ((uint32_t)current_block_array[3] << 24) + ((uint32_t)current_block_array[2] << 16) + ((uint32_t)current_block_array[1] << 8) + ((uint32_t)current_block_array[0] << 0);
    pers_rtc_backup_set(BACKUP_INFO_BLOCK_0003, data32);
    
    data32 = ((uint32_t)current_block_array[7] << 24) + ((uint32_t)current_block_array[6] << 16) + ((uint32_t)current_block_array[5] << 8) + ((uint32_t)current_block_array[4] << 0);
    pers_rtc_backup_set(BACKUP_INFO_BLOCK_0407, data32);

    pers_rtc_backup_set(BACKUP_USER_CARD_MODE, (uint32_t)(user_card_mode));
    }

switch (pers_state)
    {
    case PERS_IDLE_STATE:
        {
        if (pMsg->PersMngEvent == PERS_START)
            {
            eeprom_param_get(PERS_MASTER_EADD, &master_card_reg, 1);
            eeprom_param_get(PERS_UIDNUM_EADD, &uid_map_num, 1);

            if (master_card_reg & 0x01)
                eeprom_master_uid_get(master_uid_array);
            else
                master_uid_reset();

            if (pers_mode == EVS_PERS_MODE)
                pers_state = PERS_MODE_ACTIVE;
            else
                pers_state = PERS_MODE_INACTIVE;
            }
        }
        break;

    case PERS_MODE_INACTIVE:
        {
        if ((isSemMode() == TRUE) && (getCollaudoRunning() == FALSE))
            return;

        if (pMsg->PersMngEvent == PERS_START)
            {
            if (pers_mode == EVS_PERS_MODE)
                pers_state = PERS_MODE_ACTIVE;
            }
        else if (pMsg->PersMngEvent == PERS_CARD_READ)
            {
            if ((evs_state_get() > EVSTATE_SOCKET_AVAILABLE) && ((pers_rtc_backup_get(BACKUP_CHARGE_STATUS) & FREE_START_SAVE) == FREE_START_SAVE))
                return;
            else if ((pers_mode == EVS_FREE_MODE) && (master_format_check(new_block) == 1))              // tipo carta: carta master
                {
                if ((master_uid_check(new_uid) == 1) || ((master_card_reg & 0x01) == 0))                     // carta master: riconosciuta o assente in eeprom
                    {
                    if ((*new_block == 'P') || ((pers_puls_held == 1) && (master_card_reg & 0x01)))   // info aggiuntiva (o pulsante pemuto): imposta EVS_PERS_MODE
                        {
                        if ((master_card_reg & 0x01) == 0)
                            master_uid_update(new_uid);
                                                        
                        pers_mode_set(EVS_PERS_MODE);
                        send_to_extinp(EXTINP_MASTER_CARD);
                        pers_state = PERS_MODE_ACTIVE;
                        
                        if (getCollaudoRunning() == FALSE)
                             eeprom_uc_reset_set();
                        }
                    }
                else
                    {
                    pers_state_next = PERS_MODE_INACTIVE;
                    pers_blind_set(&pers_state);
                    send_to_lcd(LCD_MASTER_ERROR);
                    }
                }
            }
        }
        break;

    case PERS_MODE_ACTIVE:
        {
        if (pMsg->PersMngEvent == PERS_RTC_BACKUP_START)
            {
            data32 = pers_rtc_backup_get(BACKUP_USER_UID_0003);
            current_uid_array[3] = (uint8_t)(data32 >> 24);
            current_uid_array[2] = (uint8_t)(data32 >> 16);
            current_uid_array[1] = (uint8_t)(data32 >> 8);
            current_uid_array[0] = (uint8_t)(data32 >> 0);

            data32 = pers_rtc_backup_get(BACKUP_USER_UID_0407);
            current_uid_array[7] = (uint8_t)(data32 >> 24);
            current_uid_array[6] = (uint8_t)(data32 >> 16);
            current_uid_array[5] = (uint8_t)(data32 >> 8);
            current_uid_array[4] = (uint8_t)(data32 >> 0);

            data32 = pers_rtc_backup_get(BACKUP_INFO_BLOCK_0003);
            current_block_array[3] = (uint8_t)(data32 >> 24);
            current_block_array[2] = (uint8_t)(data32 >> 16);
            current_block_array[1] = (uint8_t)(data32 >> 8);
            current_block_array[0] = (uint8_t)(data32 >> 0);

            data32 = pers_rtc_backup_get(BACKUP_INFO_BLOCK_0407);
            current_block_array[7] = (uint8_t)(data32 >> 24);
            current_block_array[6] = (uint8_t)(data32 >> 16);
            current_block_array[5] = (uint8_t)(data32 >> 8);
            current_block_array[4] = (uint8_t)(data32 >> 0);

            user_card_mode = (card_auth_en)(pers_rtc_backup_get(BACKUP_USER_CARD_MODE));

            if (user_card_mode & TIME_CHARGE_AUTH)
                user_card_time = pers_rtc_backup_get(BACKUP_USER_CARD_TIME);

            if (user_card_mode & ENERGY_CHARGE_AUTH)
                user_card_energy = (int32_t)(pers_rtc_backup_get(BACKUP_USER_CARD_ENERGY));

            pers_state = PERS_MODE_UID_BUSY;
            }
        else if (pMsg->PersMngEvent == PERS_AUTORIZATION_MODE)
            {
            pers_mode_set(EVS_FREE_MODE);
            send_to_extinp(EXTINP_MASTER_CARD);
            pers_state = PERS_MODE_INACTIVE;
            }
        else if (pMsg->PersMngEvent == PERS_CARD_READ)
            {
            if (evs_state_get() == EVSTATE_PLUG_OUT)
                {
                }
            else if ((master_format_check(new_block) == 1) && (master_uid_check(new_uid) == 1) && (evs_gost_param_get() == 0))  // PRESA LIBERA e carta master riconosciuta
                {
                if ((*new_block == 'F') || (pers_puls_held == 1))                   // imposta EVS_FREE_MODE
                    {
                    pers_mode_set(EVS_FREE_MODE);
                    send_to_extinp(EXTINP_MASTER_CARD);
                    pers_state = PERS_MODE_INACTIVE;
                    eeprom_uc_reset_set();
                    }
                else if (*new_block == 'P')                                         // vai al dettaglio
                    master_operation_execute(new_block);
                else    // errore
                    {
                    pers_state_next = PERS_MODE_ACTIVE;
                    pers_blind_set(&pers_state);
                    send_to_lcd(LCD_UID_ERROR);
                    }
                }
            else if (whitelist_check(new_block) == 1)                               // carta utente presente in white list
                {
                user_card_auth_read(&user_card_mode, new_block);
            
                if ((user_card_mode == NULL_AUTH)                                   // fallita gestione new_block_array
                 || (user_card_mode == EXPIRED_CARD)                                // carta scaduta
                 || (user_card_mode == CREDIT_EXHAUSTED))                           // crediti esauriti
                    {
                    pers_state_next = PERS_MODE_ACTIVE;
                    pers_blind_set(&pers_state);
                    
                    if (user_card_mode == NULL_AUTH)
                        send_to_lcd(LCD_AUTHORIZATION_FAILED);
                    else if (user_card_mode == EXPIRED_CARD)
                        send_to_lcd(LCD_EXPIRED_CARD);
                    else    // if (user_card_mode == CREDIT_EXHAUSTED)
                        send_to_lcd(LCD_CREDIT_EXHAUSTED);
                    }
                else
                    {
                    current_uid_update(new_uid, new_block);
                    
                    /* Update modbus register for app */
                    setUidAuthorizationByCard(new_uid); 

/*                    if (user_card_mode == DATE_CHARGE_AUTH)
                        {
                        pers_state = PERS_SHOW_DATA;
                        pers_set_timer(PERS_DATA_TIM, PERS_DATA_TIME);
                        send_to_lcd(LCD_ONLY_DATE);
                        }
                    else if ((user_card_mode & DATE_CHARGE_AUTH) || (user_card_mode & CREDIT_CHARGE_AUTH))
                        {
                        pers_uid_confirm_set(&pers_state);

                        if ((user_card_mode & DATE_CHARGE_AUTH) && (user_card_mode & CREDIT_CHARGE_AUTH))   // carta valida [a scadenza e crediti]
                            send_to_lcd(LCD_CREDIT_DATE);
                        else if (user_card_mode & DATE_CHARGE_AUTH)                                         // carta valida [solo a scadenza]
                            send_to_lcd(LCD_ONLY_DATE);
                        else    // if (user_card_mode & CREDIT_CHARGE_AUTH)                                 // carta valida [solo a crediti]
                            send_to_lcd(LCD_ONLY_CREDIT);
                        }*/

                    if (user_card_mode & CREDIT_CHARGE_AUTH)
                        {
                        pers_uid_confirm_set(&pers_state);

                        if ((user_card_mode & DATE_CHARGE_AUTH) && (user_card_mode & CREDIT_CHARGE_AUTH))   // carta valida [a scadenza e crediti]
                            send_to_lcd(LCD_CREDIT_DATE);
                        else    // if (user_card_mode & CREDIT_CHARGE_AUTH)                                 // carta valida [solo a crediti]
                            send_to_lcd(LCD_ONLY_CREDIT);
                        }
                    else if (user_card_mode & DATE_CHARGE_AUTH)
                        {
                        pers_state = PERS_SHOW_DATA;
                        pers_set_timer(PERS_DATA_TIM, PERS_DATA_TIME);
                        send_to_lcd(LCD_ONLY_DATE);
                        }
                    else    // if ((user_card_mode & NO_LIMIT_AUTH) || (user_card_mode & TIME_CHARGE_AUTH) || (user_card_mode & ENERGY_CHARGE_AUTH))
                        {
                        pers_state = PERS_MODE_UID_BUSY;
                        send_to_evs(EVS_AUTH_START);
                        }
                    }
                }
            else                                                                    // carta utente non presente in white list 
                {
                pers_state_next = PERS_MODE_ACTIVE;
                pers_blind_set(&pers_state);

//                if (evs_gost_param_get() == 1)
                    send_to_evs(EVS_AUTH_NEG);
//                else
//                    send_to_lcd(LCD_UID_ERROR);
                }
            
            pers_puls_held = 0;
            }
        }
        break;

    case PERS_SHOW_DATA:
        {
        if (pMsg->PersMngEvent == PERS_DATA_TIM_EXPIRED)
            {
            pers_state = PERS_MODE_UID_BUSY;
            send_to_evs(EVS_AUTH_START);
            }
        }
        break;

    case PERS_UID_CONFIRM:
        {
        if (pMsg->PersMngEvent == PERS_CARD_READ)
            {
            if (current_uid_match(new_uid, new_block) == 1)                         // uid carta coincidente con quello di entrata -> si cancella /* RIVEDERE: controllare anche new_block */
                {
                    if (user_card_credit_decrease(new_block) == 1)
                        {
                        pers_state_next = PERS_MODE_UID_START;
                        pers_blind_set(&pers_state);
                        
                        if (user_card_mode == CREDIT_CHARGE_AUTH)
                            send_to_lcd(LCD_NEW_ONLY_CREDIT);
                        else    // if ((user_card_mode & DATE_CHARGE_AUTH) && (user_card_mode & CREDIT_CHARGE_AUTH))
                            send_to_lcd(LCD_NEW_CREDIT_DATE);
                        }
                    else
                        {
                        pers_state_next = PERS_MODE_ACTIVE;
                        pers_blind_set(&pers_state);
                        send_to_lcd(LCD_AUTHORIZATION_FAILED);
                        }
                }
            else // errore
                {
                pers_state_next = PERS_MODE_ACTIVE;
                pers_blind_set(&pers_state);
                send_to_lcd(LCD_UID_ERROR);
                }
            }
        else if (pMsg->PersMngEvent == UID_CONFIRM_TIM_EXPIRED)
            {
            pers_state = PERS_MODE_ACTIVE;
            send_to_lcd(LCD_CARD_WAIT);
            }
        }
        break;

    case PERS_MODE_UID_START:
    case PERS_MODE_UID_BUSY:
        {
        if (pMsg->PersMngEvent == PERS_CARD_READ)
            {
            if (((master_format_check(new_block) == 1) && (master_uid_check(new_uid) == 1)) // carta master: riconosciuta
              || (current_uid_match(new_uid, new_block) == 1))                              // uid carta coincidente con quello di entrata /* RIVEDERE: controllare anche new_block */
                {
                current_uid_reset();
                pers_state = PERS_MODE_ACTIVE;
                send_to_evs(EVS_AUTH_STOP);
                }
            else // errore
                {
                pers_state_next = PERS_MODE_UID_BUSY;
                pers_blind_set(&pers_state);
                pers_LcdMngEvent = LcdMngMsg_Old_get();
                send_to_lcd(LCD_UID_ERROR);
                }
            }
        else if (pMsg->PersMngEvent == PERS_UID_RELEASE)
            {
            current_uid_reset();
            pers_state = PERS_MODE_ACTIVE;
            }
        else if (pMsg->PersMngEvent == PERS_CARD_CHARGE_EXPIRED)
            {
            current_uid_reset();
            pers_state = PERS_MODE_ACTIVE;
            send_to_evs(EVS_AUTH_STOP);
            }
        }
        break;

    case PERS_UPDATE_WLIST:
        {
        if (pMsg->PersMngEvent == PERS_CARD_READ)
            {
            if (master_format_check(new_block) == 1)
                {
                if (master_uid_check(new_uid) == 1)
                    {
                    pers_state = PERS_MODE_ACTIVE;
                    send_to_lcd(LCD_CARD_WAIT);
                    }
                else
                    {
                    pers_state_next = PERS_MODE_ACTIVE;
                    pers_blind_set(&pers_state);
                    send_to_lcd(LCD_UID_ERROR);
                    }
                }
            else if (whitelist_check(new_block) == 1)                               // se carta utente presente in white list -> si avvia cancellazione
                {
                current_uid_update(new_uid, new_block);
                pers_uid_delete_set(&pers_state);
                send_to_lcd(LCD_UID_DELETE_CONFIRM);
                }
            else                                                                    // carta utente non presente in white list -> si registra
                {
                if (uid_map_num < USER_UID_EENUM)
                    {
                    whitelist_update(new_block, UID_ADD);
                    send_to_lcd(LCD_UID_ADDED);
                    }
                else
                    send_to_lcd(LCD_WLIST_FULL);

                pers_state_next = PERS_UPDATE_WLIST;
                pers_blind_set(&pers_state);
                }
            }
        else if (pMsg->PersMngEvent == UPDATE_WLIST_TIM_EXPIRED)
            {
            pers_state = PERS_MODE_ACTIVE;
            send_to_lcd(LCD_CARD_WAIT);
            }
        }
        break;

    case PERS_UID_DELETE:
        {
        if (pMsg->PersMngEvent == PERS_CARD_READ)
            {
            if (current_uid_match(new_uid, new_block) == 1)                         // uid carta coincidente con quello di entrata -> si cancella /* RIVEDERE: controllare anche new_block */
                {
                whitelist_update(new_block, UID_DELETE);
                send_to_lcd(LCD_UID_DELETED);
                }
            else // errore
                send_to_lcd(LCD_DELETE_ERROR);

            current_uid_reset();
            pers_state_next = PERS_UPDATE_WLIST;
            pers_blind_set(&pers_state);
            }
        else if (pMsg->PersMngEvent == UID_DELETE_TIM_EXPIRED)
            {
            pers_wlist_update_set(&pers_state);
            send_to_lcd(LCD_UPDATE_WLIST);
            }
        }
        break;

    case PERS_WLIST_DELETE:
        {
        if (pMsg->PersMngEvent == PERS_CARD_READ)
            {
            if ((master_format_check(new_block) == 1) && (master_uid_check(new_uid) == 1))
                {
                card_operation = ((uint16_t)(*(new_block + 1)) << 8) + *(new_block + 2);
    
                if (card_operation == 0xA55A)                                       // cancella white list
                    {
                    *(new_block + 1) = 0xFF;
                    *(new_block + 2) = 0xFF;

                    if (rfid_sl030_data_push(CMD_BLOCK_WRITE, new_block, 8) == SL030_OPERATION_SUCCEED) // write block 8
                        {
                        whitelist_reset();
                        send_to_lcd(LCD_WLIST_DELETED);
                        }
                    else    // errore
                        send_to_lcd(LCD_RESET_FAILED);
                    }
                else    // errore
                    send_to_lcd(LCD_RESET_FAILED);
                }
            else    // errore
                send_to_lcd(LCD_RESET_FAILED);

            pers_state_next = PERS_MODE_ACTIVE;
            pers_blind_set(&pers_state);
            }
        else if (pMsg->PersMngEvent == WLIST_DELETE_TIM_EXPIRED)
            {
            pers_state = PERS_MODE_ACTIVE;
            send_to_lcd(LCD_CARD_WAIT);
            }
        }
        break;

    case PERS_MODE_BLIND:
        {
        if (pMsg->PersMngEvent == PERS_BLIND_TIM_EXPIRED)
            {
            if (pers_state_next == PERS_MODE_UID_BUSY)
                {
                pers_state = pers_state_next;
                send_to_lcd(pers_LcdMngEvent);
                }
            else if (pers_state_next == PERS_UPDATE_WLIST)
                {
                pers_wlist_update_set(&pers_state);
                send_to_lcd(LCD_PERS_BACK);
                }
            else if (pers_state_next == PERS_MODE_UID_START)
               {
               pers_state = PERS_MODE_UID_BUSY;
               send_to_evs(EVS_AUTH_START);
               }
            else
                {
                pers_state = pers_state_next;
                
                if (evs_gost_param_get() == 0)
                    send_to_lcd(LCD_PERS_BACK);
                }
            }
        }
        break;

    default:
        {
        PersManager_init();
        send_to_pers(PERS_START);
        }
        break;
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  PersMngTask
//
//  DESCRIPTION:    gestione modalià personal
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void PersMngTask(void *pvParameters)
{
uint8_t i;

/* init task */

/*-------- Creates an empty mailbox for PersMngTask messages --------------------------*/
PersMngQueue = xQueueCreate(2, sizeof(PersMngMsg_st));
configASSERT(PersMngQueue != NULL);

/* in this case we use one shot features */
for (i=0;i<PERS_NUM_TIMER; i++)
    {
    xPersMngTimers[i] = xTimerCreate("TimPersMng", portMAX_DELAY, pdFALSE, (void*)(i), PersMngTimCallBack);
    configASSERT(xPersMngTimers[i] != NULL);
    }

PersManager_init();

/* start task */

for (;;)
    {
    /* Wait for some event from SW */
    if (xQueueReceive(PersMngQueue, (void *)&PersMngMsg, portMAX_DELAY) == pdPASS)
        {
        PersManager(&PersMngMsg);
        }
    else
        {
        /* Wait for possible handled timeout */
        }
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//																																									   //
//	File:		EvsTimeMng.c																																		   //
//	Author:		Vania																																				   //
//	Date:		06/11/2020																																			   //
//                                                          										                                                         		   //
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

#include "main.h"

#include "EvsMng.h"
#include "EvsTimeMng.h"
#include "LcdMng.h"
#include "PersMng.h"

#include "eeprom.h"
#include "scuMdb.h"
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local defines -------------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local typedef -------------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local constats ------------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local references ----------------------------------------------------------------------------------------------------------------------- //
xQueueHandle EvsTimeMngQueue = NULL;

static EvsTimeMngMsg_st		EvsTimeMngMsg;

static uint8_t				busy_time_run;
static uint8_t				charging_time_run;
static uint8_t				sem_charging_time_run;

static uint32_t				busy_seconds;
static uint32_t				charging_seconds;
static uint32_t				sem_charging_seconds;

static uint8_t				busy_time_array[4];
static uint8_t				charging_time_array[4];

static uint8_t				charge_time_array[4];

static uint32_t             timed_charge_seconds;

static uint8_t              rtc_backup_get;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local defines -------------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local function prototypes -------------------------------------------------------------------------------------------------------------- //
xQueueHandle getEvsTimeMngQueueHandle(void);

static void evstime_rtc_backup_set(uint32_t reg, uint32_t val);
uint32_t evstime_rtc_backup_get(uint32_t reg);
static void evstime_time_update(uint32_t *src_ptr, uint8_t *dst_ptr);
static void EvsTimeManager_init(void);
static uint32_t EvsTimeManager(EvsTimeMngMsg_st *pMsg);
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	getEvsTimeMngQueueHandle
//
//	DESCRIPTION:	-
//
//	INPUT:			none
//
//	OUTPUT:			-
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
xQueueHandle getEvsTimeMngQueueHandle(void)
{
return(EvsTimeMngQueue);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	send_to_evstime
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			-
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void send_to_evstime(uint8_t evstime_event)
{
EvsTimeMngMsg_st	msgEvsTimeSend;

msgEvsTimeSend.EvsTimeMngEvent = (EvsTimeMngEvent_en)(evstime_event);
configASSERT(xQueueSendToBack(getEvsTimeMngQueueHandle(), (void *)&msgEvsTimeSend, portMAX_DELAY) == pdPASS);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	busy_time_run_get
//
//	DESCRIPTION:	-
//
//	INPUT:			none
//
//	OUTPUT:			-
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t busy_time_run_get(void)
{
return busy_time_run;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	evstime_busy_get
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint32_t evstime_busy_get(void)
{
return busy_seconds;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	evstime_time_get
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
char evstime_time_get(uint8_t *time_array)
{
char        ret;
uint8_t     *src_ptr, config;
uint32_t    rtc_backup;

eeprom_param_get(TCHARGE_MODE_EADD, &config, 1);

if (rtc_backup_get == 1)
    {
    busy_seconds = (evstime_rtc_backup_get(BACKUP_CHARGE_TIME) & 0x00FFFFFF);
    charging_seconds = busy_seconds;
//    sem_charging_seconds = evstime_rtc_backup_get(BACKUP_SEM_CHARGE_TIME);
    rtc_backup_get = 0;
    }

if (config == 1)
    {
    eeprom_param_get(TCHARGE_TIME_EADD, &config, 1);

    if (config > 0)
        config = 1;
    }

if ((user_card_auth_get() & TIME_CHARGE_AUTH) || (config == 1))
    {
    src_ptr = charge_time_array;
    eeprom_param_get(BATTERY_CONFIG_EADD, &config, 1);
        
    if (config == 1)
        {
        rtc_backup = (evstime_rtc_backup_get(BACKUP_CHARGE_TIME) & 0xFF000000);
        rtc_backup += charging_seconds;
        evstime_rtc_backup_set(BACKUP_CHARGE_TIME, rtc_backup);
        }
    }
else
    {
    src_ptr = busy_time_array;
    eeprom_param_get(BATTERY_CONFIG_EADD, &config, 1);
        
    if (config == 1)
        {
        rtc_backup = (evstime_rtc_backup_get(BACKUP_CHARGE_TIME) & 0xFF000000);
        rtc_backup += busy_seconds;
        evstime_rtc_backup_set(BACKUP_CHARGE_TIME, rtc_backup);
        }
    }

if (*(src_ptr + 3) > 0)                 // day > 0
	{
	*time_array = *(src_ptr + 3);		// day
	*(time_array + 1) = *(src_ptr + 2);	// hrs
	ret = 'd';
	}
else if (*(src_ptr + 2) > 0)			// hrs > 0
	{
	*time_array = *(src_ptr + 2);		// hrs
	*(time_array + 1) = *(src_ptr + 1);	// min
	ret = 'h';
	}
else									// min
	{
	*time_array = *(src_ptr + 1);		// min
	*(time_array + 1) = *(src_ptr + 0);	// sec
	ret = 's';
	}

return ret;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evstime_rtc_backup_set
//
//  DESCRIPTION:    salva lo stato della ricarica per la gestione degli stati di low power mode
//
//  INPUT:          status
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void evstime_rtc_backup_set(uint32_t reg, uint32_t val)
{
HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), reg, val);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  evstime_rtc_backup_get
//
//  DESCRIPTION:    salva lo stato della ricarica per la gestione degli stati di low power mode
//
//  INPUT:          status
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint32_t evstime_rtc_backup_get(uint32_t reg)
{
return HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)getHandleRtc(), reg);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	evstime_time_update
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void evstime_time_update(uint32_t *src_ptr, uint8_t *dst_ptr)
{
uint32_t data = (*src_ptr);
uint8_t day, hrs, min, sec;

day = (uint16_t)(data / 86400);
data -= ((uint32_t)(day) * 86400);
hrs = (uint16_t)(data / 3600);
data -= ((uint32_t)(hrs) * 3600);
min = (uint16_t)(data / 60);
data -= ((uint32_t)(min) * 60);
sec = (uint16_t)(data);

*dst_ptr = sec;
*(dst_ptr + 1) = min;
*(dst_ptr + 2) = hrs;
*(dst_ptr + 3) = day;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	busy_time_init
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void busy_time_init(void)
{
uint8_t     i;

if ((rtc_backup_get == 0) && (busy_time_run == 0))
  busy_seconds = 0L;

evstime_time_update(&busy_seconds, busy_time_array);

eeprom_param_get(TCHARGE_MODE_EADD, &i, 1);

if (i == 1)
    {
    eeprom_param_get(TCHARGE_TIME_EADD, &i, 1);
    timed_charge_seconds = (uint32_t)(i) * 1800;
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	charging_time_run_get
//
//	DESCRIPTION:	-
//
//	INPUT:			none
//
//	OUTPUT:			-
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t charging_time_run_get(void)
{
return charging_time_run;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	charging_time_init
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void charging_time_init(void)
{
if ((rtc_backup_get == 0) && (charging_time_run == 0))
  charging_seconds = 0L;

evstime_time_update(&charging_seconds, charging_time_array);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	rtc_backup_get_reset
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void rtc_backup_get_reset(void)
{
rtc_backup_get = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	EvsTimeManager_init
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void EvsTimeManager_init(void)
{
rtc_backup_get = 0;
busy_time_run = 0;
charging_time_run = 0;
sem_charging_time_run = 0;
busy_time_init();
charging_time_init();
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	EvsTimeManager
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint32_t EvsTimeManager(EvsTimeMngMsg_st *pMsg)
{
uint8_t     timed_recharge, charge_time;
uint32_t    charge_seconds, rtc_backup;
uint32_t    newTimeTick = pdMS_TO_TICKS(1000);

eeprom_param_get(TCHARGE_MODE_EADD, &timed_recharge, 1);
eeprom_param_get(TCHARGE_TIME_EADD, &charge_time, 1);

if (busy_time_run == 1)
	{
	if (busy_seconds < 8913599L)	// 99 giorni, 99 ore, 59 minuti, 59 secondi
		busy_seconds ++;

	evstime_time_update(&busy_seconds, busy_time_array);

    if (timed_recharge == 1)
        {
        timed_charge_seconds --;
        charge_seconds = timed_charge_seconds;
        }
        
    if (user_card_auth_get() & TIME_CHARGE_AUTH)
        {
        charge_seconds = user_card_time_decrease();
        
        if (timed_recharge == 1)
            {
            if (timed_charge_seconds < charge_seconds)
                charge_seconds = timed_charge_seconds;
            }
        
        timed_recharge = 1;
        }
	
	if (timed_recharge == 1)
	    {
	    evstime_time_update(&charge_seconds, charge_time_array);

        if (charge_seconds == 0)
            {
            if (user_card_auth_get() & TIME_CHARGE_AUTH)
                send_to_pers(PERS_CARD_CHARGE_EXPIRED);
            else
                send_to_evs(EVS_AUTH_STOP);
            }
        }
	}
	
if (charging_time_run == 1)
	{
	if (charging_seconds < 8913599L)	// 99 giorni, 99 ore, 59 minuti, 59 secondi
		charging_seconds ++;

	evstime_time_update(&charging_seconds, charging_time_array);
	}

if (sem_charging_time_run == 1)
	{
	if (sem_charging_seconds < 8913599L)	// 99 giorni, 99 ore, 59 minuti, 59 secondi
	    {
	    if (evs_state_get() == EVSTATE_CHARGING)
	        {
		    sem_charging_seconds ++;
		    setInChargeTime(sem_charging_seconds);
//  		evstime_rtc_backup_set(BACKUP_SEM_CHARGE_TIME, sem_charging_seconds);
            }
		}
	}

if (pMsg->EvsTimeMngEvent == EVSTIME_CARD_TIME_UPDATE)
    {
    charge_seconds = user_card_time_decrease();

    if (timed_recharge == 1)
        {
        if (timed_charge_seconds < charge_seconds)
            charge_seconds = timed_charge_seconds;
        }

    evstime_time_update(&charge_seconds, charge_time_array);
    }

if (pMsg->EvsTimeMngEvent == EVSTIME_RTC_BACKUP_START)
	rtc_backup_get = 1;

if (pMsg->EvsTimeMngEvent == EVSTIME_BUSY_START)
	{
	busy_time_init();
	busy_time_run = 1;
	}

if (pMsg->EvsTimeMngEvent == EVSTIME_BUSY_STOP)
	busy_time_run = 0;

if (pMsg->EvsTimeMngEvent == EVSTIME_CHARGING_START)
	{
	charging_time_init();
	charging_time_run = 1;
	}

if (pMsg->EvsTimeMngEvent == EVSTIME_CHARGING_STOP)
    {
    rtc_backup = (evstime_rtc_backup_get(BACKUP_CHARGE_TIME) & 0xFF000000);
	evstime_rtc_backup_set(BACKUP_CHARGE_TIME, rtc_backup);
	charging_time_run = 0;
	charging_time_init();
    busy_time_run = 0;
	busy_time_init();
    }

if (pMsg->EvsTimeMngEvent == EVSTIME_SEM_CHARGING_START)
	{
	if (sem_charging_time_run == 0)
	    sem_charging_seconds = 0;

	setInChargeTime(sem_charging_seconds);
	sem_charging_time_run = 1;
	}

if (pMsg->EvsTimeMngEvent == EVSTIME_SEM_CHARGING_RESET)
    {
//	evstime_rtc_backup_set(BACKUP_SEM_CHARGE_TIME, 0L);
	sem_charging_seconds = 0;
    }

if (pMsg->EvsTimeMngEvent == EVSTIME_SEM_CHARGING_STOP)
	sem_charging_time_run = 0;

if ((busy_time_run == 1) || (charging_time_run == 1) || (sem_charging_time_run == 1))
	{
	send_to_lcd(EVSTIME_TIME_EXPIRED);
	newTimeTick = pdMS_TO_TICKS(1000);
	}
else
	newTimeTick = portMAX_DELAY;

return newTimeTick;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	EvsTimeMngTask
//
//	DESCRIPTION:	Handle power evstime
//
//	INPUT:			none
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void EvsTimeMngTask(void *pvParameters)
{
uint32_t timeTick;

/* init task */

/*-------- Creates an empty mailbox for EvsTimeMngTask messages --------------------------*/
EvsTimeMngQueue = xQueueCreate(4, sizeof(EvsTimeMngMsg_st));
configASSERT(EvsTimeMngQueue != NULL);

EvsTimeManager_init();
timeTick = portMAX_DELAY;

/* start task */

for (;;)
	{
	/* Wait for some event from SW */
	if (xQueueReceive(EvsTimeMngQueue, (void *)&EvsTimeMngMsg, timeTick) == pdPASS)
		{
		timeTick = EvsTimeManager(&EvsTimeMngMsg);
		}
	else 
		{
		/* Wait for possible handled timeout */
		EvsTimeMngMsg.EvsTimeMngEvent = EVSTIME_TIMEOUT;
		timeTick = EvsTimeManager(&EvsTimeMngMsg);
		}
	}
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

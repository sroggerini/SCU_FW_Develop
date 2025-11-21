// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//
//  File:           BlockMng.c
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
#include "EvsMng.h"

#include "sbcGsy.h"
#include "wrapper.h"
#include "ethInitTask.h"
#include "AutotestMng.h"
#ifndef HW_MP28947  
#include "homeplugdev.h"
#endif

#include "EnergyMng.h"
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local defines -------------------------------------------------------------------------------------------------------------------------- //
#define BLOCK_GARD_TIME             pdMS_TO_TICKS((uint32_t)(100))
#define DRIVE_MOVING_TIME           pdMS_TO_TICKS((uint32_t)(350))
#define SAFE_DRIVE_TIME             pdMS_TO_TICKS((uint32_t)(50))
#define DRIVE_COOLING_TIME          pdMS_TO_TICKS((uint32_t)(1000))
#define BLOCK_ENABLE_TIME           pdMS_TO_TICKS((uint32_t)(5000))
#define BLOCK_REQ_SAVE_TIME         pdMS_TO_TICKS((uint32_t)(500))
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local typedef -------------------------------------------------------------------------------------------------------------------------- //
typedef enum
{
BLOCK_STATE_TIM = 0,
BLOCK_DOWN_TIM,
BLOCK_DRIVING_TIM,
SAFE_DRIVE_TIM,
DRIVE_COOLING_TIM,
BLOCK_REQ_SAVE_TIM,
BLOCK_NUM_TIMER
}BlockMngTim_en;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local constats ------------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local references ----------------------------------------------------------------------------------------------------------------------- //
xQueueHandle BlockMngQueue = NULL;

static BlockMngMsg_st        BlockMngMsg;

static TimerHandle_t         xBlockMngTimers[BLOCK_NUM_TIMER];

static block_state_en        block_state;
static block_state_en        block_state_next;

static uint8_t               block_error;

__no_init uint8_t            block_polarity __attribute__((section(".bkpsram")));

static uint8_t               block_attempts_num;
static uint8_t               block_correct_position;

static uint8_t               block_drive_moving_stop;

static uint8_t               block_manual_enable;

static uint8_t               block_initial_check;

static block_state_en        block_req_save;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local function prototypes -------------------------------------------------------------------------------------------------------------- //
xQueueHandle getBlockMngQueueHandle(void);
static void BlockMngTimCallBack(TimerHandle_t pxTimer);
static void block_set_timer(BlockMngTim_en timer, uint32_t set_time);
static void block_rtc_backup_set(uint8_t valid, uint8_t pol);
static uint32_t block_rtc_backup_get(uint32_t reg);
static void BlockManager_init(void);
static void BlockManager(BlockMngMsg_st *pMsg);
static void send_to_evs_Gpio1(uint8_t evs_event);
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  getBlockMngQueueHandle
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
xQueueHandle getBlockMngQueueHandle(void)
{
return(BlockMngQueue);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  BlockMngTimCallBack
//
//  DESCRIPTION:    callback to manager timers   
//
//  INPUT:          TimerHandle_t: the elapsed timer 
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void BlockMngTimCallBack(TimerHandle_t pxTimer)
{
uint32_t    timer_id;

timer_id = (uint32_t)pvTimerGetTimerID(pxTimer);            // find the led  which the timer is referred

if (timer_id == (uint32_t)(BLOCK_REQ_SAVE_TIM))             // check if timer exist
    {
    send_to_block(BLOCK_STATE_TIM_EXPIRED);
    }

if (timer_id == (uint32_t)(DRIVE_COOLING_TIM))              // check if timer exist
    send_to_block(DRIVE_COOLING_TIM_EXPIRED);

if (timer_id == (uint32_t)(BLOCK_DRIVING_TIM))              // check if timer exist
    send_to_block(BLOCK_STATE_TIM_EXPIRED);

if (timer_id == (uint32_t)(SAFE_DRIVE_TIM))                 // check if timer exist
    send_to_block(BLOCK_STATE_TIM_EXPIRED);

if (timer_id == (uint32_t)(BLOCK_STATE_TIM))                // check if timer exist
    send_to_block(BLOCK_STATE_TIM_EXPIRED);

if (timer_id == (uint32_t)(BLOCK_DOWN_TIM))                 // check if timer exist
    block_manual_enable = 1;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  block_set_timer
//
//  DESCRIPTION:    imposta il timer selezionato
//
//  INPUT:          timer da aggiornare: timer; tempo da impostare: set_time
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void block_set_timer(BlockMngTim_en timer, uint32_t set_time)
{
while ((xTimerChangePeriod (xBlockMngTimers[timer], set_time, BLOCK_GARD_TIME) != pdPASS));    // set timer
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  block_movement_set
//
//  DESCRIPTION:    aziona gli attuatori nella direzione coerentemente alla variabile di ingresso sulla base del valore di "block_polarity"
//
//  INPUT:          lock: BLOCK_LOCK - ingaggia blocchi; BLOCK_UNLOCK-> libera blocchi
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void block_movement_set(uint8_t lock)
{
uint8_t     pol;
uint32_t	rtc_reg;

rtc_reg = block_rtc_backup_get(BACKUP_HW_INFO);

if (rtc_reg & BLOCK_POLARITY_VALID)
    pol = (rtc_reg & BLOCK_POLARITY_SET) >> 9;
else
    pol = block_polarity;

if (((lock == BLOCK_LOCK) && (pol == 0)) || ((lock == BLOCK_UNLOCK) && (pol == 1)))
    setOutBL1_P();
else    // if (((lock == BLOCK_LOCK) && (pol == 1)) || ((lock == BLOCK_UNLOCK) && (pol == 0)))
    setOutBL1_M();
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  send_to_block
//
//  DESCRIPTION:    impacchetta l'evento da inviare a BlockMngTask
//  
//  INPUT:          valore di BlockMngEvent
//  
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void send_to_block(uint8_t block_event)
{
BlockMngMsg_st  msgBlockSend;

msgBlockSend.BlockMngEvent = (BlockMngEvent_en)(block_event);
configASSERT(xQueueSendToBack(getBlockMngQueueHandle(), (void *)&msgBlockSend, portMAX_DELAY) == pdPASS);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	block_state_get
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			-
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
block_state_en block_state_get(void)
{
return block_state;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  block_anom0_update
//
//  DESCRIPTION:    aggiorna gli errori presenti
//
//  INPUT:          none
//             
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void block_anom0_update(void)
{
if (block_error == 1)
    evs_error_set(CONTROL_BYTE_0, BLOCK_ANOM0, 1);
else
    evs_error_set(CONTROL_BYTE_0, BLOCK_ANOM0, 0);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  block_error_reset
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void block_error_reset(void)
{
block_error = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  block_rtc_backup_set
//
//  DESCRIPTION:    salva parametri della ricarica per la gestione degli stati di low power mode
//
//  INPUT:          status
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void block_rtc_backup_set(uint8_t valid, uint8_t pol)
{
uint32_t	rtc_reg;
    
rtc_reg = block_rtc_backup_get(BACKUP_HW_INFO);

rtc_reg &=~ (BLOCK_POLARITY_VALID | BLOCK_POLARITY_SET);

if (valid == 1)
    rtc_reg |= BLOCK_POLARITY_VALID;

if (pol == 1)
    rtc_reg |= BLOCK_POLARITY_SET;

HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), BACKUP_HW_INFO, rtc_reg);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  block_rtc_backup_get
//
//  DESCRIPTION:    legge parametri della ricarica per la gestione degli stati di low power mode
//
//  INPUT:          status
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint32_t block_rtc_backup_get(uint32_t reg)
{
return HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)getHandleRtc(), reg);
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  BlockManager_init
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void BlockManager_init(void)
{
block_error = 0;
block_polarity = 1;
block_manual_enable = 1;
block_drive_moving_stop = 0;
block_req_save = BLOCK_NULL_STATE;
block_state = BLOCK_IDLE_STATE;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  BlockManager
//
//  DESCRIPTION:    -
//
//  INPUT:          puntatore BlockMngEvent: pMsg
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void BlockManager(BlockMngMsg_st *pMsg)
{
uint8_t     control_enable, actuator_enable, block_polarity_assigned;
uint32_t	rtc_reg;

// xx eeprom_param_get(CONTROL_BYTE0_EADD, &control_enable, 1);
control_enable = infoStation.controlByte.Byte.Byte0;
control_enable &= BLOCK_CRL0;

// xx eeprom_param_get(ACTUATORS_EADD, &actuator_enable, 1);

actuator_enable &= BLOCK_ATT0;
actuator_enable = infoStation.actuators;
// xx eeprom_param_get(BLOCK_DIR_EADD, &block_polarity_assigned, 1);
block_polarity_assigned = infoStation.blockDir;
block_polarity_assigned >>= 7;

if (control_enable == 0)
    block_error = 0;

switch (block_state) 
    {
    case BLOCK_IDLE_STATE:      // idle block_state
        {
        if (block_polarity_assigned == 0)
            block_rtc_backup_set(0, 0);                                     // reset flag nel registro RTC

        if (actuator_enable == BLOCK_ATT0)  // attuatori abilitati
            {
            rtc_reg = getFlagV230();

            if (rtc_reg == BACKUP_WAIT_END_POWER)                           // controllo blocchi in assenza tensione 
                {
                if (getLogicalBlockPin() == TRUE)                           // blocco chiuso
                    {
                    block_movement_set(BLOCK_UNLOCK);                       // apre i blocchi
                    block_set_timer(BLOCK_DRIVING_TIM, DRIVE_MOVING_TIME);  // inserito per sicurezza, dovrebbe andare via tensione èper cui il driver si ferma da sé
                    }

                block_state_next = BLOCK_DOWN_STATE;
                block_state = BLOCK_DOWN_STATE;
                gsy_quick_polling_update(LOCKED_OUTLET, 0);
                send_to_evs_Gpio1(EVS_BLOCK_STEADY);
                }
            else   // controllo blocchi all'uscita del reset (normale accensione)
                {
                if (getInputState(IN5_BLK_UP_PIN_UP) == GPIO_PIN_RESET)     // lettura dello switch di posizione del blocco [blocco chiuso]
                    {
                    block_movement_set(BLOCK_UNLOCK);                       // apre i blocchi
                    block_state_next = BLOCK_DOWN_STATE;                    // inizia il controllo del movimento corretto dei blocchi
                    }
                else
                    {
                    block_movement_set(BLOCK_LOCK);         // chiude i blocchi
                    block_state_next = BLOCK_UP_STATE;      // inizia il controllo del movimento corretto dei blocchi
                    }

                block_initial_check = 1;
                block_state = BLOCK_DRIVE_MOVING;
            	block_set_timer(BLOCK_DRIVING_TIM, DRIVE_MOVING_TIME);
            	}
            }
        else  // attuatori non abilitati
            {
            block_state = BLOCK_DOWN_STATE;                 // assegnazione fittizia arbitraria
            gsy_quick_polling_update(LOCKED_OUTLET, 0);
            send_to_evs_Gpio1(EVS_BLOCK_STEADY);
            }

        block_attempts_num = 0;
        block_correct_position = 0;
        }
        break;

    case BLOCK_DRIVE_MOVING:    // move up/down socket block
        {
        if (pMsg->BlockMngEvent == BLOCK_DOWN_REQ)
            {
            if ((block_initial_check == 0) && (block_polarity_assigned == 1))
                {
                block_req_save = BLOCK_DOWN_STATE;
                block_set_timer(BLOCK_REQ_SAVE_TIM, BLOCK_REQ_SAVE_TIME);
                }
            }

        if ((pMsg->BlockMngEvent == BLOCK_STATE_TIM_EXPIRED))
            {
            brakePhase();   // arresta eccitazione driver hardware del motore dei blocchi

            if ((block_polarity_assigned == 0) || (block_initial_check == 1) || (control_enable == BLOCK_CRL0))
                {
                if (((getInputState(IN5_BLK_UP_PIN_UP) == GPIO_PIN_RESET) && (block_state_next == BLOCK_UP_STATE))      // controllo di coerenza tra posizione desiderata
                 || ((getInputState(IN6_BLK_DWN_PIN_UP) == GPIO_PIN_RESET) && (block_state_next == BLOCK_DOWN_STATE))   // e switch letto [posizione corretta]
                 || ((block_initial_check == 1) && (control_enable == 0) && (block_polarity_assigned == 1)))            // controllo sugli switch disabilitato
                    {
                    block_error = 0;
                    block_correct_position = 1;

                    if ((block_initial_check == 1) || (block_polarity_assigned == 0))   // si continuano i movimenti
                        {
                        block_state = BLOCK_DRIVE_COOLING;
                        block_set_timer(DRIVE_COOLING_TIM, DRIVE_COOLING_TIME);
                        }
                    else    // polarità assegnata e posizione corretta
                        {
                        if (block_state_next == BLOCK_UP_STATE)
                            gsy_quick_polling_update(LOCKED_OUTLET, 1);
                        else
                            gsy_quick_polling_update(LOCKED_OUTLET, 0);
        
                        block_state = block_state_next;
                        send_to_evs_Gpio1(EVS_BLOCK_STEADY);
                        }
                    }
                else if ((control_enable == BLOCK_CRL0) || (block_polarity_assigned == 0))    // lettura switch non coerente con la posizione desiderata
                    {
                    block_attempts_num ++;
                    block_state = BLOCK_DRIVE_COOLING;
                    block_set_timer(DRIVE_COOLING_TIM, DRIVE_COOLING_TIME);
                    }
                }
            else    // if ((block_initial_check == 0) && (control_enable == 0x00) && (block_polarity_assigned == 1))
                {
                if (block_state_next == BLOCK_UP_STATE)
                    gsy_quick_polling_update(LOCKED_OUTLET, 1);
                else
                    gsy_quick_polling_update(LOCKED_OUTLET, 0);
            
                block_drive_moving_stop = 1;
                block_state = BLOCK_DRIVE_COOLING;
                block_set_timer(DRIVE_COOLING_TIM, DRIVE_COOLING_TIME);
                }
            }
        }
        break;

    case BLOCK_DRIVE_COOLING:   // cooling block driver before any other operation/movement
        {
        if (pMsg->BlockMngEvent == BLOCK_DOWN_REQ)
            {
            if ((block_initial_check == 0) && (block_polarity_assigned == 1))
                {
                block_req_save = BLOCK_DOWN_STATE;
                block_set_timer(BLOCK_REQ_SAVE_TIM, BLOCK_REQ_SAVE_TIME);
                }
            }

        if (pMsg->BlockMngEvent == DRIVE_COOLING_TIM_EXPIRED)
            {
            if (block_drive_moving_stop == 1)
                {
                block_drive_moving_stop = 0;
                block_state = block_state_next;
                block_req_save = BLOCK_NULL_STATE;
                send_to_evs_Gpio1(EVS_BLOCK_STEADY);
                }
            else if (block_correct_position == 0)
                {
                if (block_attempts_num < 3)                 // possiamo ritentare il movimento
                    {
                    if (block_state_next == BLOCK_UP_STATE)
                        block_movement_set(BLOCK_LOCK);
                    else
                        block_movement_set(BLOCK_UNLOCK);

                    block_state = BLOCK_DRIVE_MOVING;
                    block_set_timer(BLOCK_STATE_TIM, DRIVE_MOVING_TIME);
                    }
                else    // if (block_attempts_num >= 3)     // esaurito il numero massimo di tentativi per il movimento del blocco
                    {
                    if (block_polarity_assigned == 0)       // la polarità dei blocchi tentata non ha funzionato
                        {
                        block_polarity ^= 1;                                                // si inverte la polarità del motore
                        block_polarity_assigned = (BLOCK_ASSIGNMENT_DONE | block_polarity); // si salva in eeprom la configurazione dei blocchi
                        SCU_InfoStation_Set ((uint8_t *)&infoStation.blockDir, &block_polarity_assigned, 1);   /* ex BLOCK_DIR_EADD */
                        block_rtc_backup_set(1, block_polarity);                            // si salva anche nei registri rtc la configurazione dei blocchi
                        
                        block_state = BLOCK_IDLE_STATE;                                     // si riparte dall'inizio per verificare comunque il movimento
                        block_set_timer(DRIVE_COOLING_TIM, DRIVE_COOLING_TIME);
                        }
                    else
                        {
                        block_error = 1;
                        block_initial_check = 0;
                        block_state = block_state_next;     // si va nello stato richiesto
                        
                        if (block_state == BLOCK_UP_STATE)
                            gsy_quick_polling_update(LOCKED_OUTLET, 1);
                        else
                            gsy_quick_polling_update(LOCKED_OUTLET, 0);
    
                        send_to_evs_Gpio1(EVS_BLOCK_STEADY);
                        }
                    }
                }
            else    // if (block_correct_position == 1)
                {
                if (block_polarity_assigned == 0)           // la polarità dei blocchi tentata ha funzionato correttamente
                    {
                    block_polarity_assigned = (BLOCK_ASSIGNMENT_DONE | block_polarity); // si salva in eeprom la configurazione dei blocchi
                    SCU_InfoStation_Set ((uint8_t *)&infoStation.blockDir, &block_polarity_assigned, 1);   /* ex BLOCK_DIR_EADD */
                    block_rtc_backup_set(1, block_polarity);                            // si salva anche nei registri rtc la configurazione dei blocchi
                    }

                if (block_initial_check == 1)               // si controlla nell'altra direzione
                    {
                    if (block_state_next == BLOCK_UP_STATE)
                        {
                        block_movement_set(BLOCK_UNLOCK);
                        block_state_next = BLOCK_DOWN_STATE;
                        }
                    else
                        {
                        block_movement_set(BLOCK_LOCK);
                        block_state_next = BLOCK_UP_STATE;
                        }
                    
                    block_attempts_num = 0;
                    block_initial_check = 0;
                    block_correct_position = 0;
                    block_state = BLOCK_DRIVE_MOVING;
                    block_set_timer(BLOCK_DRIVING_TIM, DRIVE_MOVING_TIME);
                    }
                else
                    {
                    block_state = block_state_next;         // si va nello stato richiesto

                    if (block_state == BLOCK_UP_STATE)
                        gsy_quick_polling_update(LOCKED_OUTLET, 1);
                    else
                        gsy_quick_polling_update(LOCKED_OUTLET, 0);

                    send_to_evs_Gpio1(EVS_BLOCK_STEADY);
                    }
                }
            }
        }
        break;

    case BLOCK_UP_STATE:        // socket block up
        {
        if ((pMsg->BlockMngEvent == BLOCK_DOWN_REQ) || (block_req_save == BLOCK_DOWN_STATE))
            {
            block_req_save = BLOCK_NULL_STATE;

            if (actuator_enable == 0)
                {
                block_state = BLOCK_DOWN_STATE;
                gsy_quick_polling_update(LOCKED_OUTLET, 0);
                send_to_evs_Gpio1(EVS_BLOCK_STEADY);
                }
            else
                {
                block_movement_set(BLOCK_UNLOCK);

                block_attempts_num = 0;
                block_correct_position = 0;
                block_state_next = BLOCK_DOWN_STATE;
                block_state = BLOCK_DRIVE_MOVING;
                block_set_timer(BLOCK_STATE_TIM, DRIVE_MOVING_TIME);
                }
            }
        else if (pMsg->BlockMngEvent == BLOCK_UP_REQ)
            {
            gsy_quick_polling_update(LOCKED_OUTLET, 1);
            send_to_evs_Gpio1(EVS_BLOCK_STEADY);
            }
        }
        break;

    case BLOCK_DOWN_STATE:      // socket block down
        {
        if ((pMsg->BlockMngEvent == BLOCK_STATE_TIM_EXPIRED))
            brakePhase();       // arresta eccitazione driver hardware del motore dei blocchi [si può arrivare qui se rtc_reg = BACKUP_WAIT_END_POWER]

        if ((block_manual_enable == 1) && (pMsg->BlockMngEvent == BLOCK_MANUAL_REQ))
            {
            if (actuator_enable == BLOCK_ATT0)
                {
                block_manual_enable = 0;
                block_set_timer(BLOCK_DOWN_TIM, BLOCK_ENABLE_TIME);
                block_movement_set(BLOCK_UNLOCK);
                block_set_timer(BLOCK_STATE_TIM, DRIVE_MOVING_TIME);
                }
            }
        else if (pMsg->BlockMngEvent == BLOCK_UP_REQ)
            {
            if (actuator_enable == 0)
                {
                block_state = BLOCK_UP_STATE;
                gsy_quick_polling_update(LOCKED_OUTLET, 1);
                send_to_evs_Gpio1(EVS_BLOCK_STEADY);
                }
            else
                {
                block_movement_set(BLOCK_LOCK);

                block_attempts_num = 0;
                block_correct_position = 0;
                block_state_next = BLOCK_UP_STATE;
                block_state = BLOCK_DRIVE_MOVING;
                block_set_timer(BLOCK_STATE_TIM, DRIVE_MOVING_TIME);
                }
            }
        else if (pMsg->BlockMngEvent == BLOCK_DOWN_REQ)
            {
            gsy_quick_polling_update(LOCKED_OUTLET, 0);
            send_to_evs_Gpio1(EVS_BLOCK_STEADY);
            }
        }
        break;

    default:
        {
        }
        break;
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//    FUNCTION NAME:    BlockMngTask
//
//    DESCRIPTION:    Handle socket block
//
//    INPUT:            none
//
//    OUTPUT:            none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void BlockMngTask(void *pvParameters)
{
#ifdef COME_ERA
    uint16_t    delay;
#endif    
    uint8_t     i;

/* init task */

/*-------- Creates an empty mailbox for BlockMngTask messages --------------------------*/
BlockMngQueue = xQueueCreate(4, sizeof(BlockMngMsg_st));
configASSERT(BlockMngQueue != NULL);

/*-------- Creates all timer for BlockMngTask ------------------------------------------*/
/* in this case we use one shot features */
for (i=0;i<BLOCK_NUM_TIMER; i++)
    {
    xBlockMngTimers[i] = xTimerCreate("TimBlockMng", portMAX_DELAY, pdFALSE, (void*)(i), BlockMngTimCallBack);
    configASSERT(xBlockMngTimers[i] != NULL);
    }

    BlockManager_init();
    /* start task */
    /* ritardo introdotto come path per evitare il reset configASSERT( uxTopPriority ); in task.c */
    /* non mi è chiaro come questo processo interagisce con gli altri ma questo ritardo sembra    */
    /* risolva il restart allo startup Nick 01/08/2021 */
    osDelay(2000);
    
#ifndef HW_MP28947        
    startSbcUart();
#endif
    
    /*  SINAPSI-RSE */
    if ((getSinapsiEepromEn() == ENABLED) || (getSinapsiNewEnable() == TRUE))   
    {
      //setPMreadyForIom2G();
    }

    startV230Mng();
    if (checkVbusFlag() == 2)
    {
      checkStartSemSbcUartTask();
    }

#ifndef HW_MP28947    
    startIoExpPenFiltering(); 
#endif
    
    /* get lcd type  from LCD_TYPE_EADD   */
    // xx eeprom_param_get(LCD_TYPE_EADD, (uint8_t *)&i, 1);
    i = infoStation.LcdType;
    
#ifdef GD32F4xx    
    /* set the operating mode between WiFi and SBC */
    if ((i & SBC_WIFI_MASK) == SBC_WIFI_ON) setWifiSbcEnv(SBC_WIFI_ON); else setWifiSbcEnv(SBC_WIFI_OFF);
    
    if (((i & WIFI_MASK) == WIFI_ON) || (getAutotestStatus() == TRUE) || (getWifiSbcEnv() == SBC_WIFI_ON))
    {
      AppEmobTask_start();
    }
    else
#endif
    {
      AppEmobTask_stop();
      /* If autotest is active, notify the result of the test: if we arrive here, Flash, EEPROM, ioExp are OK  */
      notifyAutotestResult(AUTOTEST_EVENT_DIGITAL_IC, TEST_PASSED);
    }
    
#ifdef COME_ERA
    delay = (uint16_t)cpuIdCheksum16() & (uint16_t)0x03FF;  /* random delay from 0 to 1047 msec */
    osDelay((uint32_t)delay);
#else
    if (getStationId() %2 !=  0)
    {
      /* if this SCU has odd address 1(master), 3, 5,... */
      osDelay((uint32_t)5000);
    }
#endif

#ifdef USO_SOLO_DELAY_CASUALE
/* il pin GPIO1 è utilizzato per algoritmo di assegnazione indirizzo nel SEM */
    while (gpio1IoExpRead() == GPIO_PIN_RESET)
    {
      osDelay(50);
    }
    //GPIOD->ODR ^= (uint32_t)0x00000002; /* only for debug */
    (void)gpio1IoExpWrite(GPIO_PIN_RESET);
#endif
  
    ethMotorPowerOn();

    /* start Ethernet Init */
    ethSendEventInit();
    
#ifndef HW_MP28947  
    /* Start ISO15118 task manager */
    ISO15118_Task_SendEv(ISO15118_EV_START);        
#endif
    
    while (getAutotestStatus() == TRUE)
    {
      /* when autotest running the motor management is suspended */
      osDelay(1000);
    }


for (;;)
    {
    /* Wait for some event from SW */
    if (xQueueReceive(BlockMngQueue, (void *)&BlockMngMsg, portMAX_DELAY) == pdPASS)
        {
        BlockManager(&BlockMngMsg);
        }
    else
        {
        /* Wait for possible handled timeout */
        }
    }
}

/**
*
* @brief        Send message and reset GPIO1 Pin 
*
* @param [in]   uint8_t: event Id
*
* @retval       none 
*
***********************************************************************************************************************/
static void send_to_evs_Gpio1(uint8_t evs_event)
{
  send_to_evs(evs_event);
#ifdef USO_SOLO_DELAY_CASUALE
  while (gpio1IoExpRead() != GPIO_PIN_SET)
  {
    osDelay(100);
  }
#endif
}


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//
//  File:           ContactMng.c
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

#include "ContactMng.h"
#include "InputsMng.h"
#include "EvsMng.h"

#include "sbcGsy.h"

#include "wrapper.h"
#include "diffRiarm.h"
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local defines -------------------------------------------------------------------------------------------------------------------------- //
#define CONTACT_GARD_TIME           pdMS_TO_TICKS((uint32_t)(100))
#define CONTACT_STATE_TIME          pdMS_TO_TICKS((uint32_t)(1000))
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local typedef -------------------------------------------------------------------------------------------------------------------------- //
typedef enum
{
CONTACT_STATE_TIM = 0,
RCBO_STATE_TIM,
CONTACT_NUM_TIMER
}ContactMngTim_en;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local constats ------------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local references ----------------------------------------------------------------------------------------------------------------------- //
xQueueHandle ContactMngQueue = NULL;

static ContactMngMsg_st      ContactMngMsg;

static TimerHandle_t         xContactMngTimers[CONTACT_NUM_TIMER];

static contact_state_en      contact_state;

static uint8_t               contact_error;
static uint8_t               contact_state_check;

static uint8_t               mirror_contact_check_counter;

static ContactMngEvent_en    ContactMngEvent_save;

#ifdef GD32F4xx 
#define TIM_PWM_CNTCT  htim9
#else
#define TIM_PWM_CNTCT  htim15
#endif
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local function prototypes -------------------------------------------------------------------------------------------------------------- //
xQueueHandle getContactMngQueueHandle(void);

static void ContactMngTimCallBack(TimerHandle_t pxTimer);
static void contact_set_timer(ContactMngTim_en timer, uint32_t set_time);
static void ContactManager_init(void);
static void ContactManager(ContactMngMsg_st *pMsg);
void PWM_On_CNTCT_Stop (void);
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //


// --------------------------- extern variables ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim15;

#ifdef HW_MP28947

/**
*
* @brief        start PWM signal on CNTCT_PWM GPIO   
*
* @param [in ]  uint16_t: duty cycle ([0...1000]/1000)
*  
* @retval       none
*  
***********************************************************************************************************************/
void PWM_On_CNTCT_Start (uint8_t DutyCycle_Perc)
{
  /* Timer Output Compare Configuration Structure declaration */
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Init GPIO as alternate function */
  GPIO_InitStruct.Pin = CNTCT_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
#ifdef GD32F4xx  
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM9; 
#else  
  GPIO_InitStruct.Alternate = GPIO_AF4_TIM15; 
#endif    
  HAL_GPIO_Init(CNTCT_PWM_GPIO_Port, &GPIO_InitStruct);  
  
  /* TIM9  enable for PWM on PE5 on channel 1 */
  if(HAL_TIM_PWM_Start(&TIM_PWM_CNTCT, TIM_CHANNEL_1) != HAL_OK)
  {  
    /* PWM error  */
    Error_Handler();
  }
    
}

/**
*
* @brief        stop PWM signal on CNTCT_PWM pin
*
* @param [in ]  none
*  
* @retval       none
*  
***********************************************************************************************************************/
void PWM_On_CNTCT_Stop (void)
{
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* put the GPIO as output in Idle state: @ LOW level */
  HAL_GPIO_WritePin(CNTCT_PWM_GPIO_Port, CNTCT_PWM_Pin, GPIO_PIN_RESET);
  
  /* Init GPIO as alternate function */
  GPIO_InitStruct.Pin = CNTCT_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CNTCT_PWM_GPIO_Port, &GPIO_InitStruct);  
  
  /* TIM9  disable for PWM on PE5: Channel 1 on CNTCT_PWM gpio */
  if(HAL_TIM_PWM_Stop(&TIM_PWM_CNTCT, TIM_CHANNEL_1) != HAL_OK)
  {  
    /* PWM error  */
    Error_Handler();
  }
}

/**
*
* @brief        modify PWM duty cycle on CNTCT_PWM GPIO   
*
* @param [in ]  uint16_t: duty cycle ([0...1000]/1000)
*  
* @retval       none
*  
***********************************************************************************************************************/
void PWM_On_CNTCT_Modify (uint8_t New_DutyCycle_Perc)
{
  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef  sConfig;

  /* TIM9  disable for PWM on PE5: Channel 1 on CNTCT_PWM gpio */
  if(HAL_TIM_PWM_Stop(&TIM_PWM_CNTCT, TIM_CHANNEL_1) != HAL_OK)
  {  
    /* PWM error  */
    Error_Handler();
  }
  
  /* Set configuration for channel 1 */
  sConfig.OCMode     = TIM_OCMODE_PWM1;;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode = TIM_OCFAST_DISABLE;

  /* Set the pulse value for channel 1 */
  sConfig.Pulse = (uint32_t)((TIM9_PWM_PERIOD * New_DutyCycle_Perc) / 100);  
   
  if(HAL_TIM_PWM_ConfigChannel(&TIM_PWM_CNTCT, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
    
  /* TIM9  enable for PWM on PE5 on channel 1 */
  if(HAL_TIM_PWM_Start(&TIM_PWM_CNTCT, TIM_CHANNEL_1) != HAL_OK)
  {  
    /* PWM error  */
    Error_Handler();
  }
  
}

#endif

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  getContactMngQueueHandle
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
xQueueHandle getContactMngQueueHandle(void)
{
return(ContactMngQueue);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  ContactMngTimCallBack
//
//  DESCRIPTION:    callback to manager timers   
//
//  INPUT:          TimerHandle_t: the elapsed timer 
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void ContactMngTimCallBack(TimerHandle_t pxTimer)
{
uint32_t    timer_id;

timer_id = (uint32_t)pvTimerGetTimerID(pxTimer);                                    // find the led  which the timer is referred

if (timer_id == (uint32_t)(CONTACT_STATE_TIM))                                      // check if timer exist
    send_to_contact(CONTACT_STATE_TIM_EXPIRED);
else if (timer_id == (uint32_t)(RCBO_STATE_TIM))                                    // check if timer exist
    send_to_contact(RCBO_STATE_TIM_EXPIRED);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  contact_set_timer
//
//  DESCRIPTION:    imposta il timer selezionato
//
//  INPUT:          timer da aggiornare: timer; tempo da impostare: set_time
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void contact_set_timer(ContactMngTim_en timer, uint32_t set_time)
{
while ((xTimerChangePeriod (xContactMngTimers[timer], set_time, CONTACT_GARD_TIME) != pdPASS));    // set timer
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  send_to_contact
//
//  DESCRIPTION:    impacchetta l'evento da inviare a ContactMngTask
//  
//  INPUT:          valore di ContactMngEvent
//  
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void send_to_contact(uint8_t contact_event)
{
ContactMngMsg_st    msgContactSend;

msgContactSend.ContactMngEvent = (ContactMngEvent_en)(contact_event);
configASSERT(xQueueSendToBack(getContactMngQueueHandle(), (void *)&msgContactSend, portMAX_DELAY) == pdPASS);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  contact_anom0_update
//
//  DESCRIPTION:    aggiorna gli errori presenti
//
//  INPUT:          none
//             
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void contact_anom0_update(void)
{
if (contact_error == 1)
    evs_error_set(CONTROL_BYTE_0, MIRROR_ANOM0, 1);
else
    evs_error_set(CONTROL_BYTE_0, MIRROR_ANOM0, 0);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  ContactManager_init
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void ContactManager_init(void)
{
contact_error = 0;
mirror_contact_check_counter = 0;
contact_state_check = 0;
ContactMngEvent_save = CONTACT_EVENT_NULL;
contact_state = CONTACT_STATE_NULL;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  contact_state_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
contact_state_en contact_state_get(void)
{
return contact_state;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  ContactManager
//
//  DESCRIPTION:    -
//
//  INPUT:          puntatore ContactMngEvent: pMsg
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void ContactManager(ContactMngMsg_st *pMsg)
{
uint8_t     control_enable, actuator_enable;

eeprom_param_get(CONTROL_BYTE0_EADD, &control_enable, 1);
control_enable &= MIRROR_CRL0;

eeprom_param_get(ACTUATORS_EADD, &actuator_enable, 1);

if (pMsg->ContactMngEvent == RCBO_STATE_TIM_EXPIRED)
    setOutputState(SGCBOB, GPIO_PIN_RESET);                         // diseccita la bobina del magnetotermico allo scadere del timer

if (pMsg->ContactMngEvent == CONTACT_CONTROL_STOP)                  // sospende le operazioni di gestione del contattore
  contact_state = CONTACT_STATE_NULL;

switch (contact_state)
    {
    case CONTACT_STATE_NULL:    // idle contact_state
        {
        if (pMsg->ContactMngEvent == CONTACT_CONTROL_START)
            {
            if ((actuator_enable & CONTACT_ATT0) == 0)
                {
                contact_state_check = 1;
                contact_state = CONTACT_STATE_OPEN;
                gsy_quick_polling_update(POWERED_OUTLET, 0);
                contact_set_timer(CONTACT_STATE_TIM, CONTACT_STATE_TIME);
                }
            else
                {
                if (getInput(IN3_MIRROR_EXP0) == GPIO_PIN_RESET)
                    {
                    contact_state_check = 1;
                    contact_state = CONTACT_STATE_OPEN;
                    gsy_quick_polling_update(POWERED_OUTLET, 0);
                    }
                else
                    {
                    contact_state = CONTACT_STATE_CLOSE;
                    gsy_quick_polling_update(POWERED_OUTLET, 1);
                    send_to_contact(CONTACT_OPEN_REQ);
                    }
                }

            contact_set_timer(CONTACT_STATE_TIM, CONTACT_STATE_TIME);
            }
        }
        break;

    case CONTACT_STATE_OPEN:
        {
        if ((pMsg->ContactMngEvent == CONTACT_OPEN_REQ) && (evs_state_get() >= EVSTATE_V230_SUSPEND))   // sospende le operazioni di gestione del contattore
            ContactManager_init();
        else if (pMsg->ContactMngEvent == CONTACT_CLOSE_REQ)
            {
            gsy_quick_polling_update(POWERED_OUTLET, 1);

            if ((actuator_enable & CONTACT_ATT0) == 0)
                contact_state = CONTACT_STATE_CLOSE;
            else
                {
                setOutputState(CNTCT, GPIO_PIN_SET);            // eccita bobina contattore
#ifdef HW_MP28947        
                setOutputState(CNTCT_PWM, GPIO_PIN_SET);       // eccito bobina contattore
#endif                       
                contact_state = CONTACT_DRIVE_CLOSE;
                contact_set_timer(CONTACT_STATE_TIM, CONTACT_STATE_TIME);
                }
            }
        else if ((contact_state_check == 1) && (pMsg->ContactMngEvent == CONTACT_STATE_TIM_EXPIRED))
            {
            if (mirror_contact_check_counter < 2)
                {
                mirror_contact_check_enable();                                      // si alimenta il pin di alimentazione del contatto per andare a leggerlo
                mirror_contact_check_counter ++;
                }
            else    // if (mirror_contact_check_counter == 2)
                {
                if ((control_enable == MIRROR_CRL0) && (getInputStato(IN3_MIRROR_EXP0) == INP_STATE_RESET) && (evs_state_get() != EVSTATE_V230_SUSPEND))
                    {
                    if (contact_error == 0)
                        {
                        if ((actuator_enable & RCBO_ATT0) == RCBO_ATT0)
                            {
                            setOutputState(SGCBOB, GPIO_PIN_SET);                   // eccito bobina magnetotermico
                            sendDiffRiarmMsg(DIFF_RIARM_EV_SET_OFF);
                            }
                        
                        contact_set_timer(RCBO_STATE_TIM, CONTACT_STATE_TIME);
                        send_to_evs(EVS_MIRROR_ERROR);
                        }
                
                    contact_error = 1;
                    }
                else
                    contact_error = 0;
                
                mirror_contact_check_disable();
                mirror_contact_check_counter = 0;
                }
            }

        contact_set_timer(CONTACT_STATE_TIM, CONTACT_STATE_TIME);
        }
        break;

    case CONTACT_DRIVE_CLOSE:
        {
        if (pMsg->ContactMngEvent == CONTACT_OPEN_REQ)
            ContactMngEvent_save = CONTACT_OPEN_REQ;
        else if (pMsg->ContactMngEvent == CONTACT_STATE_TIM_EXPIRED)
            {
            contact_state = CONTACT_STATE_CLOSE;
#ifdef HW_MP28947        
            /* Set pwm to 60% on CNTCT_PWM pin */
            PWM_On_CNTCT_Start (60);
#endif            
            if (ContactMngEvent_save == CONTACT_OPEN_REQ)
                send_to_contact(CONTACT_STATE_OPEN);

            mirror_contact_check_counter = 0;
            contact_set_timer(CONTACT_STATE_TIM, CONTACT_STATE_TIME);
            }
        }
        break;

    case CONTACT_STATE_CLOSE:
        {
        if ((pMsg->ContactMngEvent == CONTACT_OPEN_REQ) || (ContactMngEvent_save == CONTACT_OPEN_REQ))
            {
            gsy_quick_polling_update(POWERED_OUTLET, 0);
            ContactMngEvent_save = CONTACT_EVENT_NULL;

            if ((actuator_enable & CONTACT_ATT0) == 0)
                {
                contact_state_check = 1;
                contact_state = CONTACT_STATE_OPEN;
                }
            else
                {
                setOutputState(CNTCT, GPIO_PIN_RESET);           // diseccito bobina contattore
#ifdef HW_MP28947        
                setOutputState(CNTCT_PWM, GPIO_PIN_RESET);       // diseccito bobina contattore
                PWM_On_CNTCT_Stop();
#endif                        
                contact_state = CONTACT_DRIVE_OPEN;

                if ((pMsg->ContactMngEvent == CONTACT_OPEN_REQ) && (evs_state_get() >= EVSTATE_V230_SUSPEND))
                    ContactManager_init();
                }

            contact_set_timer(CONTACT_STATE_TIM, CONTACT_STATE_TIME);
            }
        else if ((contact_state_check == 1) && (pMsg->ContactMngEvent == CONTACT_STATE_TIM_EXPIRED))
            {
            if (mirror_contact_check_counter < 2)
                {
                mirror_contact_check_enable();                                      // si alimenta il pin di alimentazione del contatto per andare a leggerlo
                mirror_contact_check_counter ++;
                }
            else    // if ((mirror_contact_check_counter == 2)
                {
                if ((control_enable == MIRROR_CRL0) && (getInputStato(IN3_MIRROR_EXP0) == INP_STATE_SET))
                    {
                    if (contact_error == 0)
                        {
                        if ((actuator_enable & RCBO_ATT0) == RCBO_ATT0)
                            {
                            setOutputState(SGCBOB, GPIO_PIN_SET);                       // eccito bobina magnetotermico
                            sendDiffRiarmMsg(DIFF_RIARM_EV_SET_OFF);
                            }
                        
                        contact_set_timer(RCBO_STATE_TIM, CONTACT_STATE_TIME);
                        send_to_evs(EVS_MIRROR_ERROR);
                        }
    
                    contact_error = 1;
                    }
                else
                    contact_error = 0;
    
                mirror_contact_check_disable();
                mirror_contact_check_counter = 0;
                }
            }

        contact_set_timer(CONTACT_STATE_TIM, CONTACT_STATE_TIME);
        }
        break;

    case CONTACT_DRIVE_OPEN:
        {
        if (pMsg->ContactMngEvent == CONTACT_CLOSE_REQ)
            ContactMngEvent_save = CONTACT_CLOSE_REQ;
        else if (pMsg->ContactMngEvent == CONTACT_STATE_TIM_EXPIRED)
            {
            contact_state_check = 1;
            contact_state = CONTACT_STATE_OPEN;
            
            if (ContactMngEvent_save == CONTACT_CLOSE_REQ)
                send_to_contact(CONTACT_CLOSE_REQ);

            mirror_contact_check_counter = 0;
            ContactMngEvent_save = CONTACT_EVENT_NULL;
            contact_set_timer(CONTACT_STATE_TIM, CONTACT_STATE_TIME);
            }
        }
        break;

    default:
        break;
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  ContactMngTask
//
//  DESCRIPTION:    gestione contattore
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void ContactMngTask(void *pvParameters)
{
uint8_t i;

/* init task */

/*-------- Creates an empty mailbox for ContactMngTask messages --------------------------*/
ContactMngQueue = xQueueCreate(4, sizeof(ContactMngMsg_st));
configASSERT(ContactMngQueue != NULL);

/*-------- Creates all timer for ContactMngTask ------------------------------------------*/
/* in this case we use one shot features */
for (i=0;i<CONTACT_NUM_TIMER; i++)
    {
    xContactMngTimers[i] = xTimerCreate("TimContactMng", portMAX_DELAY, pdFALSE, (void*)(i), ContactMngTimCallBack);
    configASSERT(xContactMngTimers[i] != NULL);
    }

ContactManager_init();

/* start task */

for (;;)
    {
    /* Wait for some event from SW */
    if (xQueueReceive(ContactMngQueue, (void *)&ContactMngMsg, portMAX_DELAY) == pdPASS)
        {
        ContactManager(&ContactMngMsg);
        }
    else
        {
        /* Wait for possible handled timeout */
        }
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

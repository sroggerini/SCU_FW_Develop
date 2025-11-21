/**
* @file        sbcGsy.c
*
* @brief       Uart for sbc communication  - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: sbcGsy.c 749 2025-05-07 12:32:38Z stefano $
*
*     $Revision: 749 $
*
*     $Author: stefano $
*
*     $Date: 2025-05-07 14:32:38 +0200 (mer, 07 mag 2025) $
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
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_rcc.h"
#else
#include "stm32h5xx_hal.h"
#include "stm32h5xx_ll_rcc.h"
#endif
#include "cmsis_os.h"
#include "prot_OnUsart.h"
#include "wrapper.h"
#include "sbcUart.h"
#include "sbcGsy.h"
#include "eeprom.h"
#include "rtcApi.h"
#include "Em_Task.h"
#include "scuMdb.h"
#include "adcTask.h"

#include "EvsMng.h"
#include "EnergyMng.h"
#include "LcdMng.h"
#include "PwmMng.h"
#include "PersMng.h"
#include "RfidMng.h"

#include "eeprom.h"
#include "main.h"
#include "telnet.h"
#include "wrapper.h"   

#include "ExtInpMng.h"

#include "httpserver-socket.h"

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

/*
***********************************SCAME**************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/

#define   DELAY_TO_UART_START     ((uint16_t)3000) 
#define   MONITOR_TO_UART         ((uint16_t)60000) 

typedef struct
	{
	uint8_t in;      // queue input/write pointer 
	uint8_t out;     // queue output/read pointer 
	uint8_t num;     // queue elements number
	uint8_t q[64];   // queue array 
	}uqueue;

#define CMD_REPLY_OFFSET    (uint8_t)(0x20)     // offset per preparare byte CMD da protocollo

#define RS485_ADD_WRITE     (uint8_t)(0x40)     // write RS485 device address command
#define RTC_INFO_WRITE      (uint8_t)(0x41)     // write calendar data and time command
#define RTC_INFO_READ       (uint8_t)(0xC1)     // read calendar data and time command
#define SOCKET_ENABLE_WRITE (uint8_t)(0x42)     // write socket enable command
#define SOCKET_ENABLE_READ  (uint8_t)(0xC2)     // read socket enable command
#define EVS_MODE_WRITE      (uint8_t)(0x43)     // write evs_mode command
#define EVS_MODE_READ       (uint8_t)(0xC3)     // read evs_mode command
#define SERNUM_WRITE        (uint8_t)(0x47)     // write serial number command
#define SERNUM_READ         (uint8_t)(0xC7)     // read serial number command
#define MAX0_FW_READ        (uint8_t)(0xC9)     // read firmware version command
#define MIFARE_FW_READ      (uint8_t)(0xCA)     // rfd_ver - read RFID reader firmware version command
#define HW_CONFIG_WRITE     (uint8_t)(0x48)     // write hw configuration command
#define HW_CONFIG_READ      (uint8_t)(0xC8)     // read hw configuration command
#define CONTROL_BYTEx_WRITE (uint8_t)(0x4B)     // write hardware control enable command
#define CONTROL_BYTEx_READ  (uint8_t)(0xCB)     // read hardware control enable command
#define ACTUATORS_WRITE     (uint8_t)(0x4C)     // ACTUATORS_EVAL - cntt_att; block_control_enable; sgcb_att - write actuators configuration command
#define ACTUATORS_READ      (uint8_t)(0xCC)     // read actuators configuration command
#define QUICK_POLLING_READ  (uint8_t)(0xD0)     // read quick polling command
#define SYNOP_POLLING_READ  (uint8_t)(0xD1)     // read synoptic polling command
#define EVS_AUTH_WRITE      (uint8_t)(0x52)     // write charge authorization command
#define GSY_CURRENT_WRITE   (uint8_t)(0x53)     // write gsy current command
#define GSY_CURRENT_READ    (uint8_t)(0xD3)     // read gsy current command
#define ENERGY_ARRAY_READ   (uint8_t)(0xD4)     // read energy and temperature command
#define RFUID_READ          (uint8_t)(0xD5)     // read UID command
#define CURRENT_CONFIG_READ (uint8_t)(0xD6)     // read curr_manager command
#define SUSP_CHARGE_WRITE   (uint8_t)(0x57)     // write suspend charge command
#define SUSP_CHARGE_READ    (uint8_t)(0xD7)     // read suspend charge command
#define HIDDEN_PAR_WRITE    (uint8_t)(0x4F)     // write hidden parameters command
#define HIDDEN_PAR_READ     (uint8_t)(0xCF)     // read hidden parameters command
#define RESET_REQUEST_WRITE (uint8_t)(0x5E)     // write reset request command
#define FW_UPDATE_WRITE     (uint8_t)(0x4E)     // write fw update command


#define GSY_RECEIVED_LEN    gsy_rx_array[2]
#define GSY_RECEIVED_CMD    gsy_rx_array[3]

#define GSY_REPLY_STX       gsy_tx_array[0]
#define GSY_REPLY_ADD       gsy_tx_array[1]
#define GSY_REPLY_LEN       gsy_tx_array[2]
#define GSY_REPLY_CMD       gsy_tx_array[3]

#define SCU_WITH_ST         ((uint8_t)0x01)
#define SCU_WITH_GD         ((uint8_t)0x02)

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

/*
***********************************SCAME**************************************
**                                                                          **
**                            Local Const                                   **
**                                                                          **
****************************************************************************** 
*/ 

static const uint8_t    gsy_days_of_month[] = {0 , 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
static const char       gsy_fw_version[] = FW_VERSION;

/*
***********************************SCAME**************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 

static uint8_t          gsy_rx_array[GSY_ARRAY_SIZE];  // buffer di ricezione ordinato da protocollo
static uint8_t          gsy_tx_array[GSY_ARRAY_SIZE];  // buffer di risposta ordinato da protocollo

static uint8_t          gsy_current;
static uint8_t          gsy_error_array[EVS_ERROR_ARRAY_SIZE];

static uint8_t          gsy_quick_polling;
static uint8_t          gsy_connected;

static uint8_t          gsy_evs_mode;

static card_auth_en     gsy_auth_neg_type;

static uint8_t          gsy_enable;
/*
***********************************SCAME**************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
static void     gsy_bcd_to_dec              (uint8_t *src_ptr, unsigned char *dst_ptr);
static void     gsy_dec_to_bcd              (unsigned char *src_ptr, uint8_t *dst_ptr);
static void     gsy_day_week_update         (struct DataAndTime_t *locDateTime);
static uint8_t  gsy_checksum                (uint8_t *src_ptr);
static uint8_t  gsy_protocol_compliance     (uint8_t *src_ptr);
static void     gsy_init                    (void);
static void     scu_to_gsy_error            (uint8_t *error_array);
static uint8_t  gsy_error_status            (void);
static uint8_t  gsy_manager                 (scuOpModes_e opMode);
static uint8_t  isValidRxFrame              (frameSbcRx_st* pFrameSbcRx);
static void     gsy_charge_status_reset     (void);

/*
***********************************SCAME**************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 
/* Energy meter   queue  declaration */
xQueueHandle  sbcQueue = NULL;
xQueueHandle  sbcUartRxTimeoutQueue = NULL;
osSemaphoreId uartInitSemaphoreHandle;

frameSbcRx_st    frameSbcRx;
uint32_t         timeTick;

xQueueHandle          scuTxSbcQueue = NULL;
scuToSbcTx_st         scuToSbcTx;
frameScuToSbcTx_st    frameScuToSbcTx;

uint8_t*              pDataBroad;

/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/

/*
***********************************SCAME**************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/

void sbcGestTask (void * pvParameters)
{
  scuOpModes_e        opMode;
  frameScuToSbcTx_st  tmpFrameScuToSbcTx;
  uint8_t*            pData;
  uint16_t            length;
  uint8_t*            pDataRS485;
  uint16_t            lengthRS485;
  uint8_t             esito;

  /*-------- Creates an empty mailbox for uart  messages --------------------------*/
  sbcQueue = xQueueCreate(NUM_BUFF_SBC_RX, sizeof(frameSbcRx_st));
  configASSERT(sbcQueue != NULL);
  
  /* We want this queue to be viewable in a RTOS kernel aware debugger, so register it. */
  vQueueAddToRegistry(sbcQueue, "SBC_GSY_Rx" );

  /* create a binary semaphore used for uart init access  */
  uartInitSemaphoreHandle = osSemaphoreNew(1, 1, NULL);

  pData = pDataRS485 = pDataBroad = NULL;

  /*----- Initialization FOR gsy      -------------------------------------------*/

  gsy_init();

  /* end initialization */

  //timeTick = pdMS_TO_TICKS(TIMER_TICK_500);
  timeTick = portMAX_DELAY;
  

  for (;;)
  {
    /* Wait for some event from Rx/Tx uart SBC (typically UART5)  */
    if (xQueueReceive(sbcQueue, (void *)&frameSbcRx, timeTick) == pdPASS)
    {
      if (frameSbcRx.messageEv == UART_RX_KO)
      {
        tPrintf("SBC UART5 Re-Init\n\r");
        /* an irrecuperable error occurred. Reinit uart is necessary */
        reInitSbcUart();
        // Rilascio il semaforo
        osSemaphoreRelease(uartInitSemaphoreHandle);
      }
      else
      {
        if (frameSbcRx.messageEv == UART_RX_INIT) 
        {
          timeTick = pdMS_TO_TICKS(DELAY_TO_UART_START);
        }
        else
        {
          opMode = getScuOpMode();
          if (opMode < SCU_M_P)
          {
            /* we are working as MAX0 emulation */
            if (isValidRxFrame(&frameSbcRx) == TRUE)
            {
              /*             destination            source                  length */
              memcpy((void*)&gsy_rx_array[0], (void*)&frameSbcRx.messageRx, (size_t)GSY_ARRAY_SIZE);
              esito = gsy_manager(opMode);
              if (esito == (uint8_t)TRUE)
              {
                if (getScuOpMode() == SCU_EMUMAX0_S)
                {
                  /* we are working in MAX0 slave emulation: so the answer must be sent on RS485 (USART1) */
                  txOnRs485Bus((uint8_t*)&GSY_REPLY_STX, (uint16_t)(GSY_REPLY_LEN + 3));
                }
                else
                {
                  /* we are working in MAX0 master emulation: so the answer must be sent on SBC UART (UART5-DMA1-Stream7) */
                  /* but remember that messages on SBC Rx must have 5msec distance, so we use a dedicated task  */
#ifdef FAST_DMA
                  UART_SBC_DMA_Tx((uint8_t*)&GSY_REPLY_STX, (uint16_t)(GSY_REPLY_LEN + 3));
#else                  
                                    
                  if (pDataRS485 != NULL) free(pDataRS485);
                  
                  lengthRS485 = (uint16_t)(GSY_REPLY_LEN + 3);
                  pDataRS485 = malloc(lengthRS485);
                  /*      destination                      source                  length */
                  memcpy((void*)pDataRS485, (void*)(uint8_t*)&GSY_REPLY_STX, (size_t)lengthRS485);

                  tmpFrameScuToSbcTx.msgEv = SCU_NEW_TX_MSG;
                  tmpFrameScuToSbcTx.pDataToSend = pDataRS485;
                  tmpFrameScuToSbcTx.totalLen = lengthRS485;
                  configASSERT(xQueueSendToBack(scuTxSbcQueue, (void *)&tmpFrameScuToSbcTx, portMAX_DELAY) == pdPASS);  // scuTxToSbcTask

#endif
                }
              }
              else
              {
                if (esito != (uint8_t)2) 
                {
                  if (getSinapsiStatusActivation() == (uint16_t)FALSE)
                  {
                    /* in Sinapsi - RSE the message bridge isn't applicable */
                    if (opMode == SCU_EMUMAX0_M) 
                    {
                      if (pData != NULL) free(pData);

                      length = frameSbcRx.totalLen;
                      pData = malloc(length);
                      /*      destination                          source                  length */
                      memcpy((void*)pData, (void*)frameSbcRx.messageRx.msgComplete, (size_t)length);

                      /* we aren't in FW download and the message isn't for us: bridge on RS485 must be done when SCU is master */
                      txOnRs485Bus(pData, length);
                      /* wait for end transmission: It is not properly correct, is better use a task manager but to be fast...*/
                      osDelay(10);
                    }
                  }
                }
                else
                {
                  if (esito == (uint8_t)2) 
                  {
                    /* suspend monitor on this UART: fw download is made on other task  */
                    timeTick = pdMS_TO_TICKS(portMAX_DELAY);
                  }
                }
              }
            }
          }
          else
          {
            /* we are working as SCU principal or secondary in SEM enviroment   */
            getFromModbusMap((frameMdbRx_st*)&frameSbcRx.messageRx, frameSbcRx.totalLen, opMode);
          }
          if ((opMode == SCU_EMUMAX0_M) || (isSemMasterFz() == TRUE))
          {
            resetSbcGardTime();
          }

        }
      }
    }
    else
    {
      /*----- Initialization UART FOR SBC      -------------------------------------------*/

      if (sbcPresence())
      {
        tPrintf("SBC UART5 Reprogramming\n\r");
        /** SBC and router ON **/
        HAL_GPIO_WritePin(SBC_PWR_GPIO_Port, SBC_PWR_Pin, GPIO_PIN_SET);
        /* an irrecuperable error occurred. Reinit uart is necessary */
        reInitSbcUart();
        // Rilascio il semaforo
        osSemaphoreRelease(uartInitSemaphoreHandle);
        /* monitor over uart5 sbc link */
        timeTick = pdMS_TO_TICKS(MONITOR_TO_UART);  /* a 60sec timeout is set */
      }
      else
      {
        tPrintf("SBC UART5 Re-Init\n\r");

        MX_PROT_UART_DeInit(PROT_UART_SBC);

        MX_PROT_UART_Init(PROT_UART_SBC);

        timeTick = pdMS_TO_TICKS(portMAX_DELAY);
      }

      /* end initialization */
    }
  }
}

#ifdef GD32F4xx

/**
  * @brief Manage the UART TIMeout functionality not present in GD32F4xx device.
  * @param pvParameters
  * @retval None
  */

void sbcUartRxTimeoutTask (void * pvParameters)
{
     
  /*-------- Creates an empty mailbox for uart  messages --------------------------*/
  sbcUartRxTimeoutQueue = xQueueCreate(NUM_BUFF_SBC_RX_TIMEOUT, sizeof(frameSbcRx_st));
  configASSERT(sbcUartRxTimeoutQueue != NULL);
  
     
  //timeTick = pdMS_TO_TICKS(TIMER_TICK_500);
  timeTick = portMAX_DELAY;
  

  for (;;)
  {
    /* Wait for some event from Rx uart SBC (UART5)  */
    if (xQueueReceive(sbcUartRxTimeoutQueue, (void *)&frameSbcRx, timeTick) == pdPASS)
    {
        /* Refresh the timer everytime the IDLE condition is detected */
        if (frameSbcRx.messageEv == UART_RX_IDLE)
          timeTick = pdMS_TO_TICKS(UART_RX_TIMEOUT);   
    }
    else
    {
        /* --------------------------- Timeout condition detected ----------------------------------------*/      
        timeTick = portMAX_DELAY;                     /* stop the timer */
        UART_SBC_RX_DMA_STREAM->CR &= ~DMA_SxCR_EN;   /* Disabling DMA will force transfer complete interrupt if enabled */
    }
  }
}

/**
  * @brief Get the queue handle for Uart Rx timeout management
  * @param  None
  * @retval None
  */

xQueueHandle getSbcUartRxTimeoutQueueHandle (void)
{
    return(sbcUartRxTimeoutQueue); 
}

#endif

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  gsy_charge_status_reset
//
//  DESCRIPTION:    salva lo stato della ricarica per la gestione degli stati di low power mode
//
//  INPUT:          status
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void gsy_charge_status_reset(void)
{
HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), BACKUP_CHARGE_STATUS, 0L);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  gsy_auth_neg_type_get
//
//  DESCRIPTION:    salva lo stato della ricarica per la gestione degli stati di low power mode
//
//  INPUT:          status
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
card_auth_en gsy_auth_neg_type_get(void)
{
return gsy_auth_neg_type;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  gsy_charge_status_get
//
//  DESCRIPTION:    salva lo stato della ricarica per la gestione degli stati di low power mode
//
//  INPUT:          status
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint32_t gsy_charge_status_get(void)
{
return HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)getHandleRtc(), BACKUP_CHARGE_STATUS);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  gsy_bcd_to_dec
//
//  DESCRIPTION:    converte in decimale il byte BCD di ingresso
//
//  INPUT:          puntatore sorgente: src_ptr; puntatore destinatario: dst_ptr
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void gsy_bcd_to_dec(uint8_t *src_ptr, unsigned char *dst_ptr)
{
*dst_ptr = ((*src_ptr >> 4) * 10) + (*src_ptr & 0x0F);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  gsy_dec_to_bcd
//
//  DESCRIPTION:    converte in BCD il byte decimale di ingresso
//
//  INPUT:          puntatore sorgente: src_ptr; puntatore destinatario: dst_ptr
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void gsy_dec_to_bcd(unsigned char *src_ptr, uint8_t *dst_ptr)
{
*dst_ptr = ((*src_ptr / 10) << 4) + (*src_ptr % 10);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  gsy_day_week_update
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void gsy_day_week_update(struct DataAndTime_t *locDateTime)
{
uint16_t i, calc, abs_day;

abs_day = locDateTime->Day;

for (i=1; i<locDateTime->Month; i++)
	abs_day += gsy_days_of_month[i];

if ((locDateTime->Month > 2) && ((locDateTime->Year % 4) == 0))	// anno bisestile
	abs_day += 1;

calc = (locDateTime->Year + ((locDateTime->Year - 1) / 4) - ((locDateTime->Year - 1) / 100) + ((locDateTime->Year - 1) / 400) + abs_day);

locDateTime->DayWeek = (calc % 7) + 6;

if (locDateTime->DayWeek > 7)
	locDateTime->DayWeek -= 7;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  gsy_checksum
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t gsy_checksum(uint8_t *src_ptr)
{
uint8_t i, len, cks;

cks = 0;
len = *(src_ptr + 2);

for (i = 0; i < (len + 1); i++)
    cks += *(src_ptr + i);

cks =~ cks;
cks += 1;

return cks;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  gsy_protocol_compliance
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t  gsy_protocol_compliance (uint8_t *src_ptr)
{
uint8_t /* add ,*/ len;

// xx eeprom_param_get(RS485_ADD_EADD, &add, 1);

if (*src_ptr != GSY_STX)
    return 0;

if ((*(src_ptr + 1) != infoStation.rs485Address) && (*(src_ptr + 1) != GSY_BROADCAST_ADDR))
    return 0;

len = *(src_ptr + 2);                               // get received message length

if (*(src_ptr + len + 2) != GSY_ETX)
    return 0;

if (*(src_ptr + len + 1) != gsy_checksum(src_ptr))  // check gsy_rx_array checksum
    return 0;

return 1;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  gsy_enable_set
//
//  DESCRIPTION:    
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void gsy_enable_set (void)
{
gsy_enable = 1;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  gsy_init
//
//  DESCRIPTION:    
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void gsy_init(void)
{
uint8_t i;

// xx eeprom_param_get(EVS_MODE_EADD, &gsy_evs_mode, 1);
gsy_evs_mode = infoStation.evs_mode;
gsy_connected = 0;
gsy_current = 63;
//gsy_quick_polling = (BUSY_OUTLET | PLUGGED_OUTLET);
gsy_quick_polling = (BUSY_OUTLET);

gsy_enable = 0;

for (i=0; i<EVS_ERROR_ARRAY_SIZE; i++)
    gsy_error_array[i] = 0x00;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  scu_to_gsy_error
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void scu_to_gsy_error(uint8_t *src_ptr)
{
uint8_t i, error_array[EVS_ERROR_ARRAY_SIZE];



for (i=0; i<EVS_ERROR_ARRAY_SIZE; i++)
    {
    error_array[i] = *(src_ptr + i);
    *(src_ptr + i) = 0x00;
    }

if (error_array[1] & CPLOST_ANOM1)
    *(src_ptr + 0) |= 0x01;
if (error_array[1] & CPSHORT_ANOM1)
    *(src_ptr + 0) |= 0x02;
if (error_array[1] & PPLOST_ANOM1)
    *(src_ptr + 0) |= 0x04;
if (error_array[1] & PPSHORT_ANOM1)
    *(src_ptr + 0) |= 0x08;
if (error_array[0] & RCBO_ANOM0)
    {
    *(src_ptr + 0) |= 0x10;
    *(src_ptr + 2) |= 0x10;
    }
if (error_array[0] & RCDM_ANOM0)
    *(src_ptr + 0) |= 0x20;
if (error_array[0] & MIRROR_ANOM0)
    {
    *(src_ptr + 0) |= 0x40;
    *(src_ptr + 2) |= 0x40;
    }
if (error_array[0] & BLOCK_ANOM0)
    *(src_ptr + 0) |= 0x80;

if (error_array[0] & VENT_ANOM0)
    *(src_ptr + 1) |= 0x01;
if (error_array[1] & OVERCURRENT_ANOM1)
    *(src_ptr + 1) |= 0x02;
if (error_array[0] & LID_ANOM0)
    *(src_ptr + 1) |= 0x04;
if (error_array[1] & VBUS_ANOM1)
    *(src_ptr + 1) |= 0x08;
if (error_array[1] & MIFARE_ANOM1)
    *(src_ptr + 1) |= 0x10;
if (error_array[2] & RECTIFIER_ANOM2)
    *(src_ptr + 1) |= 0x20;
if (error_array[1] & EMETER_INT_ANOM1)
    *(src_ptr + 1) |= 0x40;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  gsy_error_status
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t gsy_error_status(void)
{
uint8_t i, ret, error_array[EVS_ERROR_ARRAY_SIZE];

evs_error_get(error_array, 0, 0, 0);
scu_to_gsy_error(error_array);

/* Update modbsus register EVSE_ERROR1_RO and EVSE_ERROR2_RO */
//updateModbusErrorRegisters(error_array);

ret = 0x00;

//for (i=0; i<EVS_ERROR_ARRAY_SIZE; i++)
for (i=0; i<2; i++)
    {
    if (error_array[i] != gsy_error_array[i])
        {
        ret = ERROR_UPDATE;
        gsy_error_array[i] = error_array[i];
        }
    }

//if (gsy_error_array[0] & MIRROR_ANOM0)
//    ret |= REBOOT_REQUIRED;

return ret;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  gsy_connected_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t gsy_connected_get(void)
{
return gsy_connected;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  gsy_connected_set
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void gsy_connected_set(uint8_t status)
{
  gsy_connected = status;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  gsy_quick_polling_update
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void gsy_quick_polling_update(uint8_t bit, uint8_t set)
{
  gsy_quick_polling &=~ bit;

  if (set == 1)
  {
      gsy_quick_polling |= bit;
      if (bit == (BUSY_OUTLET | RFID_PENDING))
      {
        if (getScuOpMode() >= SCU_M_P) 
        {
          /* SEM enviroment: set the UID_AUTHORIZATION_RO area with the card info  */
          setUIDinfoROmap();
          /* SEM enviroment: send the info to notify manager */
          sendEventToSemMng(NOTIFY_TO_MASTER_TX, ADDR_UID_AUTHORIZATION_RO); 
        }
      }
  }

}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  gsy_quick_polling_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t gsy_quick_polling_get(void)
{
return gsy_quick_polling;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  gsy_current_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint16_t gsy_current_get(void)
{
return (uint16_t)(gsy_current * 10);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  gsy_evs_mode_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t gsy_evs_mode_get(void)
{
return gsy_evs_mode;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

/**
*
* @brief        Decoder message coming from SBC on UART5, prepares and sends the answer (gsy_tx_array)
*
* @param [in]   none
*
* @retval       uint8_t: TRUE, if an answer must be sent
*
***********************************************************************************************************************/
static uint8_t gsy_manager(scuOpModes_e opMode)
{
struct DataAndTime_t    locDateTime;
uint8_t                 i, reply, data8u, data8u_array[4], emeter_type, temp[8];
int32_t                 data32i;
headerFrameSbcRx_st     headerFrameSbcRx;
evs_state_en            evState;
uint16_t *              ptr16;

if (gsy_enable == 0)
  return (uint8_t)FALSE;

if (gsy_protocol_compliance(gsy_rx_array) == 0)
    return (uint8_t)FALSE;

evState = evs_state_get();

if ((evState < EVSTATE_DISABLED) && (GSY_RECEIVED_CMD != FW_UPDATE_WRITE)&& (evState != EVSTATE_INIT))
    return (uint8_t)FALSE;

//if (evs_state_get() < EVSTATE_DISABLED)
//    return (uint8_t)FALSE;

// xx eeprom_param_get(RS485_ADD_EADD, &data8u, 1);
data8u = infoStation.rs485Address;
// xx eeprom_param_get(EMETER_INT_EADD, &emeter_type, 1);
emeter_type = infoStation.emTypeInt;

GSY_REPLY_STX = GSY_STX;                                // set start
GSY_REPLY_ADD = data8u;                                 // set RS-485 SCU address
GSY_REPLY_LEN = 3;                                      // init reply len
GSY_REPLY_CMD = GSY_RECEIVED_CMD + CMD_REPLY_OFFSET;    // set reply cmd

reply = (uint8_t)TRUE;

toggleHeartLed();

switch (GSY_RECEIVED_CMD)
    {
    case RS485_ADD_WRITE:   // write RS485 device address command
        {
        if (GSY_RECEIVED_LEN == 4)
            {
            data8u = gsy_rx_array[4];
            SCU_InfoStation_Set ((uint8_t *)&infoStation.rs485Address, &data8u, 1);   /* ex RS485_ADD_EADD */
            gsy_tx_array[4] = data8u; 
            send_to_lcd(LCD_CURRENT_UPDATE);
            GSY_REPLY_LEN += 1;
            }
        }
        break;

    case RTC_INFO_WRITE:    // write calendar data and time command
        {
        if (GSY_RECEIVED_LEN == 10)
            {
            gsy_bcd_to_dec(&gsy_rx_array[4], &locDateTime.Second);
            gsy_bcd_to_dec(&gsy_rx_array[5], &locDateTime.Minute);
            gsy_bcd_to_dec(&gsy_rx_array[6], &locDateTime.Hour);

            gsy_bcd_to_dec(&gsy_rx_array[8], &locDateTime.Day);
            gsy_bcd_to_dec(&gsy_rx_array[9], &locDateTime.Month);
            gsy_bcd_to_dec(&gsy_rx_array[10], &data8u);
            
            locDateTime.Year = (data8u + 2000);

//            locDateTime.DayWeek = getDayOfWeek();
            gsy_day_week_update(&locDateTime);

            DateTimeSet(&locDateTime);
            
            data8u = 1;
            SCU_InfoStation_Set ((uint8_t *)&infoStation.rtcValid, &data8u, 1);    /* ex RTC_VALID_EADD */
            gsy_dec_to_bcd(&locDateTime.Second, &gsy_tx_array[4]);
            gsy_dec_to_bcd(&locDateTime.Minute, &gsy_tx_array[5]);
            gsy_dec_to_bcd(&locDateTime.Hour, &gsy_tx_array[6]);
            gsy_dec_to_bcd(&locDateTime.DayWeek, &gsy_tx_array[7]);
            gsy_dec_to_bcd(&locDateTime.Day, &gsy_tx_array[8]);
            gsy_dec_to_bcd(&locDateTime.Month, &gsy_tx_array[9]);
            
            data8u = (locDateTime.Year - 2000);
            gsy_dec_to_bcd(&data8u, &gsy_tx_array[10]);

            GSY_REPLY_LEN += 7;
            }
        }
        break;

    case RTC_INFO_READ:     // read calendar data and time command
        {
        if (GSY_RECEIVED_LEN == 3)
            {
            DateTimeGet(&locDateTime);

            gsy_dec_to_bcd(&locDateTime.Second, &gsy_tx_array[4]);
            gsy_dec_to_bcd(&locDateTime.Minute, &gsy_tx_array[5]);
            gsy_dec_to_bcd(&locDateTime.Hour, &gsy_tx_array[6]);
            gsy_day_week_update(&locDateTime);
            gsy_dec_to_bcd(&locDateTime.DayWeek, &gsy_tx_array[7]);
            gsy_dec_to_bcd(&locDateTime.Day, &gsy_tx_array[8]);
            gsy_dec_to_bcd(&locDateTime.Month, &gsy_tx_array[9]);
            
            data8u = (locDateTime.Year - 2000);
            gsy_dec_to_bcd(&data8u, &gsy_tx_array[10]);

            GSY_REPLY_LEN += 7;
            }
        }
        break;

    case SOCKET_ENABLE_WRITE:   // write socket enable command
        {
        if (GSY_RECEIVED_LEN == 4)
            {
            data8u = gsy_rx_array[4];
            SCU_InfoStation_Set ((uint8_t *)&infoStation.socketEnable, &data8u, 1);   /* ex SOCKET_ENABLE_EADD */
            send_to_evs(EVS_AUTORIZATION_MODE);
            gsy_tx_array[4] = data8u;
            GSY_REPLY_LEN += 1;
            }
        }
        break;

    case SOCKET_ENABLE_READ:    // read socket enable command
        {
        if (GSY_RECEIVED_LEN == 3)
            {
            // xx eeprom_param_get(SOCKET_ENABLE_EADD, &data8u, 1);
            gsy_tx_array[4] = infoStation.socketEnable;
            GSY_REPLY_LEN += 1;
            }
        }
        break;

    case EVS_MODE_WRITE:    // write evs_mode command
        {
        if (GSY_RECEIVED_LEN == 4)
            {
                // xx eeprom_param_get(EVS_MODE_EADD, (uint8_t*)(&i), 1);
                
                data8u = gsy_rx_array[4];
                /* The modality to set is different? (Fixed ticket SCU-80) */
                if (data8u != infoStation.evs_mode)
                {
                  SCU_InfoStation_Set ((uint8_t *)&infoStation.evs_mode, &data8u, 1);     /* ex EVS_MODE_EADD */
                  send_to_evs(EVS_AUTORIZATION_MODE);
                  send_to_pers(PERS_AUTORIZATION_MODE);
                  /* update station mode for APP */
                  setGeneralStationParameters(EDATA_VALID_PRG);
                  /* update station mode in modbus area (to use after in 0x500 regs) */
                  setStationOperationMode();
              }
              
              gsy_evs_mode = data8u;
              gsy_tx_array[4] = gsy_evs_mode;
              GSY_REPLY_LEN += 1;
            
/*            if ((i == EVS_PERS_MODE) || (i == EVS_NET_MODE) || (i == EVS_FREE_MODE))
              eeprom_uc_reset_set();*/
            }
        }
        break;

    case EVS_MODE_READ:	// read evs_mode command
        {
        if (GSY_RECEIVED_LEN == 3)
            {
            // xx eeprom_param_get(EVS_MODE_EADD, &data8u, 1);
            gsy_tx_array[4] = infoStation.evs_mode;
            GSY_REPLY_LEN += 1;
            }
        }
        break;

    case SERNUM_WRITE:  // write serial number command
        {
            if (GSY_RECEIVED_LEN == 7)
            {
              // xx eeprom_param_get(SERNUM_BYTE0_EADD, data8u_array, 4);            
              // Convert from BCD to Packed BCD (2 cipher in a byte)
              BCD_to_PackedBCD (&data8u_array[0], (uint8_t *)&infoStation.serial[0], MAX_SERIAL_LENGTH);
                  
              if ((data8u_array[0] == 0xFF) && (data8u_array[2] == 0xFF) && (data8u_array[2] == 0xFF) && (data8u_array[3] == 0xFF))
              {
                for (i=0; i <4; i++)
                    data8u_array[i] = gsy_rx_array[(i + 4)];                

                // Converto from PAcked BCD (2cipher in a byte) to BCD
                PackedBCD_to_BCD (&temp[0], &data8u_array[0], MAX_SERIAL_LENGTH);
                                
                SCU_InfoStation_Set ((uint8_t *)&infoStation.serial, (uint8_t *)&temp, MAX_SERIAL_LENGTH);   /* ex SERNUM_BYTE0_EADD */
              }
            
              for (i=0; i <4; i++)
                gsy_tx_array[(i + 4)] = data8u_array[i];

              GSY_REPLY_LEN += 4;
            }
        }
        break;
    
    case SERNUM_READ: // read serial number command
        {
        if (GSY_RECEIVED_LEN == 3)
            {
            // xx eeprom_param_get(SERNUM_BYTE0_EADD, data8u_array, 4);            
            BCD_to_PackedBCD ((uint8_t *)&data8u_array[0], (uint8_t *)&infoStation.serial, MAX_SERIAL_LENGTH);

            for (i=0; i <4; i++)
                gsy_tx_array[(i + 4)] = data8u_array[i];
#ifdef RICCARDO_PRONTO
#ifdef GD32F4xx  
            gsy_tx_array[8] = SCU_WITH_GD;  // bit 01 =01 bit equivale SCU con ST  bit 01 =10 bit equivale SCU con GD 
#else
            gsy_tx_array[8] = SCU_WITH_ST;  // bit 01 =01 bit equivale SCU con ST  bit 01 =10 bit equivale SCU con GD 
#endif
#else
            gsy_tx_array[8] = SCU_WITH_ST;  // bit 01 =01 bit equivale SCU con ST  bit 01 =10 bit equivale SCU con GD  
#endif
            GSY_REPLY_LEN += 5;
            }
        }
        break;

    case HW_CONFIG_WRITE: // write hw configuration command
        {
        if (GSY_RECEIVED_LEN == 8)
            {
            data8u = gsy_rx_array[4];
            SCU_InfoStation_Set ((uint8_t *)&infoStation.socketType, &data8u, 1);    /* ex SOCKET_TYPE_EADD */
            gsy_tx_array[4] = data8u;
            
            if ((data8u & (LID_OPEN_IN_CHARGE | LID_CLOSE_IN_CHARGE)) == 0)  // presa senza coperchio
                {
                // xx eeprom_param_get(CONTROL_BYTE0_EADD, &data8u, 1);
                data8u = infoStation.controlByte.Byte.Byte0;
                data8u &=~ LID_CRL0;
                SCU_InfoStation_Set ((uint8_t *)&infoStation.controlByte.Byte.Byte0, &data8u, 1);           /* ex CONTROL_BYTE0_EADD */
                evs_error_control(&data8u, CONTROL_BYTE0_EADD, 1);
                }

            data8u = gsy_rx_array[5];
            SCU_InfoStation_Set ((uint8_t *)&infoStation.emTypeInt, &data8u, 1);      /* ex EMETER_INT_EADD */
            if (data8u <= EMETER_TAMP)
                {
                evs_error_set(CONTROL_BYTE_1, EMETER_INT_ANOM1, 0);
                sendMsgStartTa();
                send_to_energy(ENERGY_INTERNAL_EM_GOOD);
                }

            gsy_tx_array[5] = data8u;
            
            data8u = gsy_rx_array[6];
            
            if (data8u > (uint8_t)(EVS_CURRENT_MAX / 10))
                data8u = (uint8_t)(EVS_CURRENT_MAX / 10);
            else if (data8u < (uint8_t)(EVS_CURRENT_MIN / 10))
                data8u = (uint8_t)(EVS_CURRENT_MIN / 10);

            SCU_InfoStation_Set ((uint8_t *)&infoStation.max_current, &data8u, 1);   /* ex M3T_CURRENT_EADD */
            gsy_tx_array[6] = data8u;
            
            data8u = gsy_rx_array[7];

//            if (data8u > (uint8_t)(M3S_CURRENT_MAX / 10))
//                data8u = (uint8_t)(M3S_CURRENT_MAX / 10);
            if (data8u > (uint8_t)(EVS_CURRENT_MAX / 10))
                data8u = (uint8_t)(EVS_CURRENT_MAX / 10);
            else if (data8u < (uint8_t)(EVS_CURRENT_MIN / 10))
                data8u = (uint8_t)(EVS_CURRENT_MIN / 10);

            SCU_InfoStation_Set ((uint8_t *)&infoStation.max_currentSemp, &data8u, 1);   /* ex M3S_CURRENT_EADD */
            gsy_tx_array[7] = data8u;
            
            data8u = gsy_rx_array[8];
            // xx eeprom_param_get(BATTERY_CONFIG_EADD, &i, 1); // get current battery backup status
            i = infoStation.batteryConfig;
            SCU_InfoStation_Set ((uint8_t *)&infoStation.batteryConfig, &data8u, 1);    /* ex BATTERY_CONFIG_EADD */
            gsy_tx_array[8] = data8u;
            if (data8u == 1)  // funzione di backup temporaneo attivato
                {
#ifdef SI_AUTOMATISMO_VBUS
               eeprom_param_get(CONTROL_BYTE1_EADD, &data8u, 1);
                data8u |= VBUS_CRL1;
                eeprom_param_set(CONTROL_BYTE1_EADD, &data8u, 1);
                evs_error_control(&data8u, CONTROL_BYTE1_EADD, 1);
#endif
              if (i != data8u)
              {
                /* workaround: to solve problem when set backup battery ON          */
                /* assenza tensione fatta subito dopo non funzionava correttamente  */
                activeImmediateReset();
              }
            }        
            
            GSY_REPLY_LEN += 5;
            
            send_to_evs(EVS_INTERNAL_EM_UPDATE);
            }
        }
        break;

    case HW_CONFIG_READ: // read hw configuration command
        {
        if (GSY_RECEIVED_LEN == 3)
            {
            // xx eeprom_param_get(SOCKET_TYPE_EADD, &gsy_tx_array[4], 1);
            gsy_tx_array[4] = infoStation.socketType;  
            // xx eeprom_param_get(EMETER_INT_EADD, &gsy_tx_array[5], 1);
            gsy_tx_array[5] = infoStation.emTypeInt;
            // xx eeprom_param_get(M3T_CURRENT_EADD, &gsy_tx_array[6], 1);
            gsy_tx_array[6] = infoStation.max_current;
            // xx eeprom_param_get(M3S_CURRENT_EADD, &gsy_tx_array[7], 1);
            gsy_tx_array[7] = infoStation.max_currentSemp;
            // xx eeprom_param_get(BATTERY_CONFIG_EADD, &gsy_tx_array[8], 1);
            gsy_tx_array[8] = infoStation.batteryConfig;
            GSY_REPLY_LEN += 5;
            }
        }
        break;

    case MAX0_FW_READ:    // mfw_ver - read Max firmware version command
        {
#ifdef HW_MP28947  
          HAL_GPIO_TogglePin(PRG_TX_GPIO_Port, PRG_TX_Pin);  // trigger for debug PRG_TX --> ONLY FOR DEBUG
#endif
        if (GSY_RECEIVED_LEN == 3)
            {
            gsy_tx_array[4] = gsy_fw_version[1];
            gsy_tx_array[5] = '.';
            gsy_tx_array[6] = gsy_fw_version[3];
            gsy_tx_array[7] = '.';
            gsy_tx_array[8] = gsy_fw_version[5];
            if (gsy_fw_version[6] == '\0')
            {
              gsy_tx_array[9] = '_';
              gsy_tx_array[10] = *((uint8_t*)(BOOT_ADDR_VER + 3));
              gsy_tx_array[11] = ' ';
            }
            else
            {
              gsy_tx_array[9] = gsy_fw_version[6];
              gsy_tx_array[10] = '_';
              gsy_tx_array[11] = *((uint8_t*)(BOOT_ADDR_VER + 3));
            }
#ifndef HW_MP28947  
            gsy_tx_array[12] = (uint8_t)getCodeHwVersion();
#else
            gsy_tx_array[12] = (uint8_t)'R';
#endif
            gsy_tx_array[13] = 'e';

            GSY_REPLY_LEN += 10;
            }
        }
        break;

    case MIFARE_FW_READ: // rfd_ver - read RFID reader firmware version command
        {
        if (GSY_RECEIVED_LEN == 3)
            {
            for (i=0; i<16; i++)
                gsy_tx_array[(4 + i)] = *(rfid_sl030_fw_get() + i);

            GSY_REPLY_LEN += 16;
            }
        }
        break;

    case CONTROL_BYTEx_WRITE:   // write hardware control enable command
        {
        if (GSY_RECEIVED_LEN == 6)
            {
            evs_control_save();
            // xx eeprom_param_get(CONTROL_BYTE2_EADD, &data8u, 1);

            for (i=0; i<3; i++)
                data8u_array[i] = gsy_rx_array[(4 + i)];

            data8u_array[2] = (data8u_array[2] & (~EMETER_EXT_CRL2)) | (infoStation.controlByte.Byte.Byte2 & EMETER_EXT_CRL2);      // GSY non gestisce l'abilitazione (Fixed ticket SCU-86)
            data8u_array[2] = (data8u_array[2] & (~SINAPSI_CHN2_CRL2)) | (infoStation.controlByte.Byte.Byte2 & SINAPSI_CHN2_CRL2);  // GSY non gestisce l'abilitazione SINAPSI (Fixed ticket SCU-86)
            
            SCU_InfoStation_Set ((uint8_t *)&infoStation.controlByte.Word, data8u_array, CONTROL_BYTE_NUM);  /* ex CONTROL_BYTE0_EADD */
            
            if ((rfid_state_get() == RFID_ERROR) && (data8u_array[1] & MIFARE_CRL1))
                send_to_rfid(RFID_CONTROL_UPDATE);

            /* Set in modbus map */
            setHwChecks((data8u_array[1] << 8) | data8u_array[0], (data8u_array[2]));
            
            evs_error_control(data8u_array, CONTROL_BYTE0_EADD, CONTROL_BYTE_NUM);
            
            for (i=0; i<3; i++)
                gsy_tx_array[(4 + i)] = data8u_array[i];

            GSY_REPLY_LEN += 3;
            }
        }
        break;

    case CONTROL_BYTEx_READ:    // read hardware control enable command
        {
        if (GSY_RECEIVED_LEN == 3)
            {
            // xx eeprom_param_get(CONTROL_BYTE0_EADD, data8u_array, CONTROL_BYTE_NUM);
            data8u_array[0] = infoStation.controlByte.Byte.Byte0;  
            data8u_array[1] = infoStation.controlByte.Byte.Byte1;  
            data8u_array[2] = infoStation.controlByte.Byte.Byte2;
            data8u_array[2] &=~ EMETER_EXT_CRL2;  // GSY non gestisce l'abilitazione

            for (i=0; i<3; i++)
                gsy_tx_array[(4 + i)] = data8u_array[i];

            GSY_REPLY_LEN += 3;
            }
        }
        break;

    case ACTUATORS_WRITE:   // write actuators configuration command
        {
        if (GSY_RECEIVED_LEN == 4)
            {
            data8u = gsy_rx_array[4];
            SCU_InfoStation_Set ((uint8_t *)&infoStation.actuators, &data8u, 1);   /* ex ACTUATORS_EADD */
            gsy_tx_array[4] = data8u;
            GSY_REPLY_LEN += 1;
            }
        }
        break;

    case ACTUATORS_READ:    // read actuators configuration command
        {
        if (GSY_RECEIVED_LEN == 3)
            {
            // xx eeprom_param_get(ACTUATORS_EADD, &gsy_tx_array[4], 1); 
            gsy_tx_array[4] = infoStation.actuators;  
            GSY_REPLY_LEN += 1;
            }     
        }
        break;

    case QUICK_POLLING_READ:    // read quick polling command
        {
        if (GSY_RECEIVED_LEN == 3)
            {
            gsy_tx_array[4] = gsy_quick_polling;
            gsy_quick_polling |= gsy_error_status();
            GSY_REPLY_LEN += 1;
            }
        }
        break;

    case SYNOP_POLLING_READ:    // read synoptic polling command
        {
        if (GSY_RECEIVED_LEN == 3)
            {
            gsy_tx_array[4] = gsy_quick_polling;

            for (i=0; i<EVS_ERROR_ARRAY_SIZE; i++)
                gsy_tx_array[(5 + i)] = gsy_error_array[i];

            gsy_quick_polling_update(ERROR_UPDATE, 0);
            GSY_REPLY_LEN += (EVS_ERROR_ARRAY_SIZE + 1);
            }
        }
        break;

    case EVS_AUTH_WRITE:    // write charge authorization command
        {
        if (GSY_RECEIVED_LEN == 4)
            {
            gsy_auth_neg_type = NULL_AUTH;

            if (gsy_rx_array[4] == 0x12)
                gsy_auth_neg_type = EXPIRED_CARD;
            else if (gsy_rx_array[4] == 0x22)
                gsy_auth_neg_type = CREDIT_EXHAUSTED;

            gsy_tx_array[4] = gsy_rx_array[4];
            GSY_REPLY_LEN += 1;

            if (gsy_rx_array[4] == 0x00)
                send_to_evs(EVS_AUTH_STOP);
            else if (gsy_rx_array[4] == 0x01)
                send_to_evs(EVS_AUTH_START);
            else if (gsy_rx_array[4] & 0x02)
                send_to_evs(EVS_AUTH_NEG);
            }
        }
        break;

    case GSY_CURRENT_WRITE: // write gsy current command
        {
        if (GSY_RECEIVED_LEN == 4)
            {
            gsy_current = gsy_rx_array[4];
            if (gsy_current == GSY_BROADCAST_PWM)
            {
              if (opMode == SCU_EMUMAX0_M)
              {
                setBroadcastDownload(TRUE);
              }
            }

            if (gsy_current > (uint8_t)(EVS_CURRENT_MAX / 10))
                gsy_current = (uint8_t)(EVS_CURRENT_MAX / 10);
            else if (gsy_current < (uint8_t)(EVS_CURRENT_MIN / 10))
                gsy_current = (uint8_t)(EVS_CURRENT_MIN / 10);

            gsy_tx_array[4] = gsy_current;
            GSY_REPLY_LEN += 1;
            }
        }
        break;

    case GSY_CURRENT_READ:  // read gsy current command
        {
        if (GSY_RECEIVED_LEN == 3)
            {
            gsy_tx_array[4] = gsy_current;
            GSY_REPLY_LEN += 1;
            }
        }
        break;

    case ENERGY_ARRAY_READ: // read energy and temperature command
        {
        if (GSY_RECEIVED_LEN == 3)
            {
            energy_to_gsy_get(EM_TOT_ACTIVE_ENERGY, &data32i, 1);

            for (i=4; i<8; i++)
                gsy_tx_array[i] = *(((uint8_t*)(&data32i)) + (7 - i));

            energy_to_gsy_get(EM_TOT_REACT_ENERGY, &data32i, 1);

            for (i=8; i<12; i++)
                gsy_tx_array[i] = *(((uint8_t*)(&data32i)) + (11 - i));

            energy_to_gsy_get(EM_ACTIVE_POWER, &data32i, 1);

            for (i=12; i<16; i++)
                gsy_tx_array[i] = *(((uint8_t*)(&data32i)) + (15 - i));

            energy_to_gsy_get(EM_REACTIVE_POWER, &data32i, 1);

            for (i=16; i<20; i++)
                gsy_tx_array[i] = *(((uint8_t*)(&data32i)) + (19 - i));

            energy_to_gsy_get(EM_SES_ACTIVE_ENERGY, &data32i, 1);

            for (i=20; i<22; i++)
                gsy_tx_array[i] = *(((uint8_t*)(&data32i)) + (21 - i));

            if (emeter_type & EMETER_THREE_PH)
                energy_to_gsy_get(EM_SYS_PH1_VOLTAGE, &data32i, 1);
            else
                energy_to_gsy_get(EM_SYS_VOLTAGE, &data32i, 1);
            
            for (i=22; i<24; i++)
                gsy_tx_array[i] = *(((uint8_t*)(&data32i)) + (23 - i));

            energy_to_gsy_get(EM_CURRENT_L, &data32i, 1);

            for (i=24; i<26; i++)
                gsy_tx_array[i] = *(((uint8_t*)(&data32i)) + (25 - i));

            energy_to_gsy_get(EM_CURRENT_L1, &data32i, 1);

            for (i=26; i<28; i++)
                gsy_tx_array[i] = *(((uint8_t*)(&data32i)) + (27 - i));

            energy_to_gsy_get(EM_CURRENT_L2, &data32i, 1);

            for (i=28; i<30; i++)
                gsy_tx_array[i] = *(((uint8_t*)(&data32i)) + (29 - i));

            energy_to_gsy_get(EM_CURRENT_L3, &data32i, 1);

            for (i=30; i<32; i++)
                gsy_tx_array[i] = *(((uint8_t*)(&data32i)) + (31 - i));

            data32i = (uint32_t)(getUpTemp());

            for (i=32; i<34; i++)
                gsy_tx_array[i] = *(((uint8_t*)(&data32i)) + (33 - i));

            GSY_REPLY_LEN += 30;
            }
        }
        break;

    case RFUID_READ:    // read UID command
        {
        if (GSY_RECEIVED_LEN == 3)
            {
            rfid_uid_get(&gsy_tx_array[5], (sl030_type_en*)(&gsy_tx_array[4]));
            gsy_quick_polling_update(RFID_PENDING, 0);
            GSY_REPLY_LEN += CARD_UID_DIM;
            }
        }
        break;

    case CURRENT_CONFIG_READ:   // read curr_manager command
        {
        if (GSY_RECEIVED_LEN == 3)
            {
//            uint8_t posRot = (uint8_t)getRotarySwitchPos();
            gsy_tx_array[4] = 0x00; /* RIVEDERE */
            gsy_tx_array[5] = 0x00;
            gsy_tx_array[6] = 0x00;
            gsy_tx_array[7] = 0x00;
            gsy_tx_array[8] = 0x00;
            gsy_tx_array[9] = 0x00;
            GSY_REPLY_LEN += 6;
            }
        }
        break;

    case SUSP_CHARGE_WRITE: // write suspend charge command
        {
        if (GSY_RECEIVED_LEN == 4)
            {
            data8u = gsy_rx_array[4];
            gsy_tx_array[4] = data8u;
            GSY_REPLY_LEN += 1;
            
            if (data8u == 0x01)
                send_to_evs(EVS_REMOTE_SUSPENDING);
            else
                send_to_evs(EVS_REMOTE_RELEASE);
            }
        }
        break;

    case SUSP_CHARGE_READ:  // read suspend charge command
        {
        if (GSY_RECEIVED_LEN == 3)
            {
            if (evs_suspending_set_get() & (UART_SUSPENDING | REMOTE_SUSPENDING))
                gsy_tx_array[4] = 0x01;
            else
                gsy_tx_array[4] = 0x00;

            GSY_REPLY_LEN += 1;
            }
        }
        break;

    case HIDDEN_PAR_WRITE:  // write hidden parameters command
        {
        if (GSY_RECEIVED_LEN == 8)
            {
            // xx eeprom_param_get(HIDDEN_MENU_ENB_EADD, &data8u, 1);
            data8u = infoStation.Hidden_Menu.Enabled;
            if (gsy_rx_array[4] == 1)
                {
                data8u |= HIDDEN_MENU_PMNG_ENB;
                SCU_InfoStation_Set ((uint8_t *)&infoStation.Hidden_Menu.Enabled, &data8u, 1);   /* ex HIDDEN_MENU_ENB_EADD */
                // xx eeprom_param_get(HIDDEN_MENU_VIS_EADD, &data8u, 1);
                data8u = infoStation.Hidden_Menu.Visible;
                data8u |= HIDDEN_MENU_PMNG_VIS;
                SCU_InfoStation_Set ((uint8_t *)&infoStation.Hidden_Menu.Visible, &data8u, 1);   /* ex HIDDEN_MENU_VIS_EADD */
                }
            else
                {
                data8u &=~ HIDDEN_MENU_PMNG_ENB;
                SCU_InfoStation_Set ((uint8_t *)&infoStation.Hidden_Menu.Enabled, &data8u, 1);   /* ex HIDDEN_MENU_ENB_EADD */
                }

            SCU_InfoStation_Set ((uint8_t *)&infoStation.TCharge.Mode, &gsy_rx_array[5], 1);   /* ex TCHARGE_MODE_EADD */
            gsy_tx_array[4] = gsy_rx_array[4];
            gsy_tx_array[5] = gsy_rx_array[5];
            gsy_tx_array[6] = 0x00;
            gsy_tx_array[7] = 0x00;
            gsy_tx_array[8] = 0x00;
            GSY_REPLY_LEN += 5;
            send_to_evs(EVS_EXTERNAL_EM_UPDATE);
            }
        }
        break;

    case HIDDEN_PAR_READ:   // read hidden parameters command
        {
        if (GSY_RECEIVED_LEN == 3)
            {
            //eeprom_param_get(PMNG_MODE_EADD, &gsy_tx_array[4], 1);
            // xx eeprom_param_get(HIDDEN_MENU_ENB_EADD, &data8u, 1);
            data8u = infoStation.Hidden_Menu.Enabled;  
            data8u &= HIDDEN_MENU_PMNG_ENB;
            gsy_tx_array[4] = data8u;
            // xx eeprom_param_get(TCHARGE_MODE_EADD, &gsy_tx_array[5], 1);
            gsy_tx_array[5] = infoStation.TCharge.Mode;
            gsy_tx_array[6] = 0x00;
            gsy_tx_array[7] = 0x00;
            gsy_tx_array[8] = 0x00;
            GSY_REPLY_LEN += 5;
            }
        }
        break;


    case RESET_REQUEST_WRITE:   // write reset request command
        {
        if (GSY_RECEIVED_LEN == 4)
            {
//            data8u = gsy_rx_array[4];
//            gsy_tx_array[4] = data8u;
//            GSY_REPLY_LEN += 1;

            gsy_charge_status_reset();
            while(gsy_charge_status_get() != 0x00000000);
            /* bug fix: if this is a broadcast reset the master must take alive the connection for 500ms atleast */
            activeImmediateReset();
            //setFlagForNvic();
            //NVIC_SystemReset();
            }
        }
        break;

    case FW_UPDATE_WRITE:   // write fwup_set command
        {          
            if (GSY_RECEIVED_LEN == 7)  /* in EMUMAX len must be 7 not 3 */
            {                            
              reply = (uint8_t)2;  /* EMUMAX0: la gestione del download è fatta nel task scuGsyDwldTask() */
              if ((gsy_rx_array[7] == (uint8_t)0) && (gsy_rx_array[4] == (uint8_t)0))
              {                               
                if ((opMode == SCU_EMUMAX0_M) && (getBroadcastDownload() == TRUE))
                {
                  /* we are in broadcast FW download and the bridge on RS485 must be done when SCU is master */
                  if (pDataBroad != NULL) free(pDataBroad);

                  pDataBroad = malloc(frameSbcRx.totalLen);
                  /*      destination                          source                  length */
                  memcpy((void*)pDataBroad, (void*)&gsy_rx_array[0], (size_t)frameSbcRx.totalLen);
                  /* we are in broadcast FW download and bridge on RS485 must be done when SCU is master for the message */
                  /* for that it is necessary to change the address to 0XFF and to calculate new checksum */
                  pDataBroad[1] = (uint8_t)GSY_BROADCAST_ADDR; //0xFF; 
                  pDataBroad[(gsy_rx_array[2] + 1)] = gsy_checksum(pDataBroad);
                  gsy_tx_array[(gsy_rx_array[2] + 2)] = GSY_ETX;
                  txOnRs485Bus(pDataBroad, frameSbcRx.totalLen);
                  /* wait for end transmission: It is not properly correct, is better use a task manager but to be fast...*/
                  osDelay(10);
                }
                if ((opMode == SCU_EMUMAX0_S) && (gsy_rx_array[1] == GSY_BROADCAST_ADDR))
                {
                  setBroadcastDownload(TRUE);
                }
                /* il byte HH della lunghezza deve essere a 0 così come il byte LL (il FW è multiplo di 1024) */
                data32i = (int32_t)((uint32_t)gsy_rx_array[5]*0x100) + (int32_t)((uint32_t)gsy_rx_array[6]*0x10000);
                setCodeLen((uint32_t)data32i);
                headerFrameSbcRx.messageEv = UART_RX_START_DWLD;
                configASSERT(xQueueSendToBack(getScuGsyDwldQueueHandle(), (void *)&headerFrameSbcRx, portMAX_DELAY) == pdPASS);  // scuGsyDwldTask()
              }

            }
        }
        break;

    default:
        reply = (uint8_t)FALSE;
        break;
    }

gsy_tx_array[(GSY_REPLY_LEN + 1)] = gsy_checksum(gsy_tx_array);
gsy_tx_array[(GSY_REPLY_LEN + 2)] = GSY_ETX;

if (reply == (uint8_t)TRUE)
    gsy_connected = 1;

return reply;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

/**
*
* @brief        Check if received frame is a valid frame 
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
static uint8_t isValidRxFrame (frameSbcRx_st* pFrameSbcRx)
{

  if (pFrameSbcRx->messageRx.reqSbcUser.startFrame == BOOT_FRAME_START)
  {
    return((uint8_t)TRUE);
  }

  if (((pFrameSbcRx->messageRx.reqSbcUser.startFrame == GSY_STX) && (pFrameSbcRx->messageRx.msgComplete[pFrameSbcRx->totalLen - 1] == GSY_ETX) &&
      (pFrameSbcRx->messageRx.msgComplete[pFrameSbcRx->totalLen - 2] == gsy_checksum((uint8_t*)&pFrameSbcRx->messageRx))) || 
      (pFrameSbcRx->messageRx.reqSbcUser.startFrame == BOOT_FRAME_START))
  {
    return((uint8_t)TRUE);
  }
  return((uint8_t)FALSE);
}

/**
*
* @brief        Get the pointer to task queue
*
* @param [in]   none
*
* @retval       xQueueHandle: pointer to defined  queue
*
***********************************************************************************************************************/
xQueueHandle getSbcAnswerQueueHandle(void)
{
   return(sbcQueue); 
}

/**
*
* @brief        Get the semaphore handle for uart re-init 
*
* @param [in]   none
*
* @retval       osSemaphoreId: handle
*
***********************************************************************************************************************/
osSemaphoreId getSbcUartInitSemaphoreHandle(void)
{
   return(uartInitSemaphoreHandle); 
}

/**
*
* @brief        Disable UART5 used in SBC comunication 
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void deInitSBCUsart(void)
{
  /*----- De-Initialization UART FOR SBC      ---------------------------*/

  MX_PROT_UART_DeInit(PROT_UART_SBC);

}

/**
*
* @brief        Disable UART1 used in RS485 comunication 
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void deInitRS485Usart(void)
{
  /*----- De-Initialization UART FOR RS485      ---------------------------*/

  MX_PROT_UART_DeInit(PROT_UART_SCU);

}


/**
*
* @brief        get the pointer to structure to manage the Rx message 
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
frameSbcRx_st* pMsgFrameSbcRx(void)
{
  return((frameSbcRx_st*)&frameSbcRx);
}

/**
*
* @brief        send a message to SBC on UART5  
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void sendToSbcOnUart(uint16_t data)
{
  uint8_t data8u;

  // xx eeprom_param_get(RS485_ADD_EADD, &data8u, 1);
  data8u = infoStation.rs485Address;
  GSY_REPLY_STX = GSY_STX;                                        // set start
  GSY_REPLY_ADD = data8u;                                         // set RS-485 SCU address
  GSY_REPLY_LEN = 4;                                              // init reply len
  GSY_REPLY_CMD = (uint8_t)(data & 0x00FF);                       // current frame LSB
  gsy_tx_array[4] = (uint8_t)((data >> 8) & 0x00FF);              // current frame MSB
  gsy_tx_array[(GSY_REPLY_LEN + 1)] = gsy_checksum(gsy_tx_array); 
  gsy_tx_array[(GSY_REPLY_LEN + 2)] = GSY_ETX;
  UART_SBC_DMA_Tx((uint8_t*)&GSY_REPLY_STX, (uint16_t)(GSY_REPLY_LEN + 3));
}
/**
*
* @brief        send a message to SBC on RS485  
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void sendToSbcOnRS485(uint16_t data)
{
  uint8_t data8u;

  // xx eeprom_param_get(RS485_ADD_EADD, &data8u, 1);
  data8u = infoStation.rs485Address;
  GSY_REPLY_STX = GSY_STX;                                        // set start
  GSY_REPLY_ADD = data8u;                                         // set RS-485 SCU address
  GSY_REPLY_LEN = 4;                                              // init reply len
  GSY_REPLY_CMD = (uint8_t)(data & 0x00FF);                       // current frame LSB
  gsy_tx_array[4] = (uint8_t)((data >> 8) & 0x00FF);              // current frame MSB
  gsy_tx_array[(GSY_REPLY_LEN + 1)] = gsy_checksum(gsy_tx_array); 
  gsy_tx_array[(GSY_REPLY_LEN + 2)] = GSY_ETX;
  /* we are working in MAX0 slave emulation: so the answer must be sent on RS485 (USART1) */
  txOnRs485Bus((uint8_t*)&GSY_REPLY_STX, (uint16_t)(GSY_REPLY_LEN + 3));

}

/**
*
* @brief        send a message to SBC to start communication on UART5  
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void startSbcUart (void)
{
  headerFrameSbcRx_st headerFrameSbcRx;
  uint32_t      locFlag230;

  locFlag230 = getFlagV230();
  if (locFlag230 != BACKUP_WAIT_END_POWER)
  {
    headerFrameSbcRx.totalLen = (uint16_t)0;
    headerFrameSbcRx.messageEv = UART_RX_INIT;

    /* start UART5 */
    xQueueSendToBack(getSbcAnswerQueueHandle(), (void *)&headerFrameSbcRx, portMAX_DELAY);
  }
}

/**
*
* @brief        Task to manage the packet transmission to SBC  
*
* @param [in]   void *: task parameters 
*
* @retval       none
*
***********************************************************************************************************************/
void scuTxToSbcTask (void * pvParameters)
{
  uint32_t        timeTick;

  /*-------- Creates an empty mailbox for uart5 Tx  messages --------------------------*/
  scuTxSbcQueue = xQueueCreate(NUM_BUFF_SCU_TO_SBC_TX, sizeof(frameScuToSbcTx_st));
  configASSERT(scuTxSbcQueue != NULL);
  

  /*----- Initialization FOR gsy      -------------------------------------------*/

  scuToSbcTx.state = SCU_TO_SBC_TX_STATE_IDLE;
  scuToSbcTx.pNextData = NULL;
  scuToSbcTx.nextLen = 0;
  timeTick = portMAX_DELAY;

  /* end initialization */

  //timeTick = pdMS_TO_TICKS(TIMER_TICK_500);
  timeTick = portMAX_DELAY;
  

  for (;;)
  {
    /* Wait for some event rerquiring Tx to SBC (tipically UART5 Tx)  */
    if (xQueueReceive(scuTxSbcQueue, (void *)&frameScuToSbcTx, timeTick) == pdPASS)
    {
      switch (scuToSbcTx.state)
      {
        case SCU_TO_SBC_TX_STATE_IDLE:
          if (frameScuToSbcTx.msgEv == SCU_NEW_TX_MSG)
          {
            scuToSbcTx.pNextData = NULL;
            scuToSbcTx.nextLen = 0;
            scuToSbcTx.state = SCU_TO_SBC_TX_STATE_TX_ON;
            /* path per MAX0 che genera un glitch in Tx che viene interpretato come 0xFF a fine pacchetto in download */
            if(frameScuToSbcTx.pDataToSend[0] == 0xFF) frameScuToSbcTx.pDataToSend[0] = (uint8_t)0x02;
            if((frameScuToSbcTx.totalLen == (uint16_t)8) && 
               (frameScuToSbcTx.pDataToSend[7] == (uint8_t)0xFF) && 
               (frameScuToSbcTx.pDataToSend[6] == (uint8_t)0x03)) frameScuToSbcTx.totalLen = (uint16_t)7;

            UART_SBC_DMA_Tx(frameScuToSbcTx.pDataToSend, frameScuToSbcTx.totalLen);
          }
          break;

        case SCU_TO_SBC_TX_STATE_TX_ON:
          if (frameScuToSbcTx.msgEv == SCU_END_TX_MSG)
          {
            osDelay(10);
            scuToSbcTx.state = SCU_TO_SBC_TX_STATE_IDLE;
          }
          else
          {
            if (frameScuToSbcTx.msgEv == SCU_NEW_TX_MSG)
            {
              scuToSbcTx.pNextData = frameScuToSbcTx.pDataToSend;
              scuToSbcTx.nextLen = frameScuToSbcTx.totalLen;
              scuToSbcTx.state = SCU_TO_SBC_TX_STATE_TX_PENDING;
            }
          }
          break;

        case SCU_TO_SBC_TX_STATE_TX_PENDING:
          if (frameScuToSbcTx.msgEv == SCU_END_TX_MSG)
          {
            /* 5 msec is the minimun time between two tx packet */
            osDelay(5);
            scuToSbcTx.state = SCU_TO_SBC_TX_STATE_TX_ON;
            UART_SBC_DMA_Tx(scuToSbcTx.pNextData, scuToSbcTx.nextLen);
          }
          break;

        default:
          break;

      }
    }
  }
}

/**
*
* @brief        Get the pointer to task queue
*
* @param [in]   none
*
* @retval       xQueueHandle: pointer to defined  queue
*
***********************************************************************************************************************/
xQueueHandle getScuToSbcTxQueueHandle(void)
{
   return(scuTxSbcQueue); 
}

/*************** END OF FILE ******************************************************************************************/



/******************* (C) COPYRIGHT 2009 STMicroelectronics *********/
/**
* @file        InputsMng.c
*
* @brief       polling and filtering on digital inputs - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: inputsMng.c 750 2025-05-08 10:20:36Z stefano $
*
*     $Revision: 750 $
*
*     $Author: stefano $
*
*     $Date: 2025-05-08 12:20:36 +0200 (gio, 08 mag 2025) $
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
#include <main.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <ctype.h>
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_rcc.h"
#else
#include "stm32h5xx_hal.h"
#include "stm32h5xx_ll_rcc.h"
#endif
#include "cmsis_os.h"
#include "main.h"
#include "InputsMng.h"

#include "BlockMng.h"
#include "EvsMng.h"
#include "ExtInpMng.h"
#include "RfidMng.h"
#include "ioExp.h"
#include "scuMdb.h"
#include "rtcApi.h"
#include "AutotestMng.h"
#include "ContactMng.h"

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#define     TIMER_SETUP_500                ((uint16_t)500)

#define     TIME_CNTT_ON                   ((uint16_t)150)
#define     TIME_CNTT_OFF                  ((uint16_t)200)
//#define     TEST_CON_CNTT                  1

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 
         inpMng_t         		inpMng[NUM_INPUT_SCU];
static   inpMngMsg_st     		impMngMsg;

static   rotaryPos_e          debugRotSwitch;    
      
#ifdef TEST_CON_CNTT
static   uint32_t             tickCntt;
static   uint16_t             statoCntt;
#endif
static   char                 strTime[] = "     ";

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local const                                   **
**                                                                          **
****************************************************************************** 
*/ 

#ifndef HW_MP28947

/* ---------------  definizione I/O per gestione ingressi digitali            -------------- */
static const uint16_t UIN_PIN[NUM_INPUT_SCU_PIN_UP]= { 
                                                DIODO_ADC_Pin,     // PD9  = DIODO_ADC
                                                RCDM_Pin,          // PE7  = RCDM
                                                IN2_Pin,           // PB10 = PULS/STOP
                                                IN5_Pin,           // PD0  = BLK_UP
                                                IN6_Pin,           // PC10 = BLK_DWN
                                                SW_PROG_Pin,       // PD14 = SW_PROG
                                                RFID_DTC_Pin       // PB15 = RFID_DTC                                                 
                                            };

static       GPIO_TypeDef* UIN_PORT[NUM_INPUT_SCU_PIN_UP] = {
                                                      DIODO_ADC_GPIO_Port,  // PD9  = DIODO_ADC
                                                      RCDM_GPIO_Port,       // PE7  = RCDM
                                                      IN2_GPIO_Port,        // PB10 = PULS/STOP
                                                      IN5_GPIO_Port,        // PD0  = BLK_UP
                                                      IN6_GPIO_Port,        // PC10 = BLK_DWN
                                                      SW_PROG_GPIO_Port,    // PD14 = SW_PROG
                                                      RFID_DTC_GPIO_Port
                                                    };

#else

/* ---------------  definizione I/O per gestione ingressi digitali            -------------- */
static const uint16_t UIN_PIN[NUM_INPUT_SCU_PIN_UP]= { 
                                                DIODO_ADC_Pin,     // PD9  = DIODO_ADC
                                                RCDM_Pin,          // PE7  = RCDM
                                                SW_PROG_Pin,       // PD14 = SW_PROG
                                                IN1_Pin,           // PB13 = REMOTE
                                                IN3_Pin,           // PB9 = MIRROR
                                                RFID_DTC_Pin       // PB15 = RFID_DTC                                                 
                                            };

static       GPIO_TypeDef* UIN_PORT[NUM_INPUT_SCU_PIN_UP] = {
                                                      DIODO_ADC_GPIO_Port,  // PD9  = DIODO_ADC
                                                      RCDM_GPIO_Port,       // PE7  = RCDM
                                                      SW_PROG_GPIO_Port,    // PD14 = SW_PROG
                                                      IN1_GPIO_Port,        // PB13 = REMOT
                                                      IN3_GPIO_Port,        // PB9 = MIRROR
                                                      RFID_DTC_GPIO_Port
                                                    };

#endif

static const char* inpName[] = {"IN0: DIODO_ADC", "RCDM", "IN2: PULSANTE", "IN5: BLOCCO UP", "IN6: BLOCCO DOWN", "SW_PROG", "IN9: RFID_DCT",
                                "PEN IN", "DIFF. RIARM.", "IN1: REMOTE", "IN3: MIRROR", "IN4: RCBO", "IN7: VENTILAZIONE", "IN8: COPERCHIO"};

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 
/*  Input manager queue  declaration */
xQueueHandle inpMngQueue = NULL;

/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/

/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Function                             **
**                                                                          **
******************************************************************************
*/

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
static uint32_t inpManager          (inpMngMsg_st* pMsg, uint32_t timeTick);
static void     initDigitalInput    (void);
static void     setRcdmPinAsIntrpt  (void);

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/


/**
*
* @brief       Gestione input digitali
*
* @param [in]  (void const *: not used  
*  
* @retval      none 
*  
****************************************************************/
void inputMngTask (void * pvParameters)
{
  uint32_t       timeTick, valInputs, maskIn;
  uint8_t        i;

  /*-------- Creates an empty mailbox for uart SITOS messages --------------------------*/
  inpMngQueue = xQueueCreate(INP_MAX_MESSAGE_NUM, sizeof(inpMngMsg_st));
  configASSERT(inpMngQueue != NULL);

  /* init task state: default all inputs are enabled with default threshold filtering  */
  valInputs = readDigitalInput();
  valInputs |= getDigitalIoExpInput();
  for (i = 0, maskIn = 1; i < NUM_INPUT_SCU; i++, maskIn = maskIn << 1)
  {
#ifdef HW_MP28947
    if (i > NUM_OF_GPIO_INPUT)      /* disable inputs not managed in HW_MP28947 */
      inpMng[i].stato = INP_STATE_DISABLED;
    else
#else    
      inpMng[i].stato = INP_STATE_IDLE;
#endif    
    inpMng[i].currValInp = valInputs & maskIn;  // store the input value after reset
    inpMng[i].currFilteringInp = 0;
    inpMng[i].filteringThreshold = DEFAULT_THRESHOLD;
  }

  strTime[5] = '\0';

  /* tick di campionamento tipico 10ms */
  timeTick = portMAX_DELAY;
#ifdef TEST_CON_CNTT
  tickCntt = 0;
  statoCntt = 0;
#endif

  /* task could be call printf function so it starts only when console task is initializated see: startConsole()*/

  for (;;)
  {
    /* Wait for some event from SW to be transmitted on telnet */
    if (xQueueReceive(inpMngQueue, (void *)&impMngMsg, timeTick) == pdPASS)
    {
      if (impMngMsg.taskEv == INP_EVENT_STOP_FILTERING)
      {
        timeTick = portMAX_DELAY;  // suspend polling
      }
      else
      {
        timeTick = inpManager((inpMngMsg_st *)&impMngMsg, timeTick);
      }
    }
    else
    {
      /* timeout:new polling over all inputs  */
      impMngMsg.taskEv = INP_EVENT_POLLING_ON_TO;
      timeTick = inpManager((inpMngMsg_st *)&impMngMsg, timeTick);
      if ((i & (uint8_t)0x3F) == 0)
      {
        /* every 64 samples change earth led status */
        toggleHeartLed();
        if (debugRotSwitch != getRotarySwitchPos())
        {
           debugRotSwitch = getRotarySwitchPos();
#ifndef HW_MP28947
           tPrintf("Rotary Switch Pos = %d\n\r", (uint16_t)debugRotSwitch);
#else
           if (debugRotSwitch == POS6_32A)
              tPrintf("Maximum current set at 32A \n\r");    
           else
              tPrintf("Maximum current set at 16A \n\r");               
#endif           
        }
        i = 0;
      }
      i++;
#ifdef TEST_CON_CNTT 
      switch (statoCntt)
      {
        case 0:
          /* contattore OFF */
          if (tickCntt >= TIME_CNTT_OFF)
          {
            tickCntt = 0;
            statoCntt = 1;
            setOutputState(CNTCT, GPIO_PIN_SET);                                // eccito bobina contattore
          }
          else
          {
            tickCntt++;
          }
          break;

        case 1:
          /* contattore OON */
          if (tickCntt >= TIME_CNTT_ON)
          {
            tickCntt = 0;
            statoCntt = 0;
            setOutputState(CNTCT, GPIO_PIN_RESET);                                // diseccito bobina contattore
          }
          else
          {
            tickCntt++;
          }
          break;
      }
#endif
    }
  }
}



/**
*
* @brief        esegue gestione comandi sulla FAT
*
* @param [in]   fatMngMsg_st*: puntatore al messaggio da eseguire
*
* @retval       none
*
***********************************************************************************************************************/
static uint32_t inpManager (inpMngMsg_st* pMsg, uint32_t timeTick)
{
  uint32_t    valInputs, maskIn, newTimeTick, temp;
  uint8_t     numIn;

  /* set new tick polling as current tick */
  newTimeTick = timeTick;

  /* start processo */
  if (pMsg->taskEv == INP_EVENT_POLLING_ON_TO)
  {
    /* polling on all enabled inputs; read all inputs */
    valInputs = readDigitalInput();
    temp = (uint32_t)getDigitalIoExpInput();
    valInputs |= temp;
    for (numIn = 0, maskIn = (uint32_t)1; numIn < NUM_INPUT_SCU; numIn++, maskIn = maskIn << 1)
    {
      switch (inpMng[numIn].stato)
      {
        case INP_STATE_DISABLED:
          break;

        case INP_STATE_IDLE:
          inpMng[numIn].currValInp = ((valInputs & maskIn) == 0) ? (uint16_t)0 : (uint16_t)1;
          if (inpMng[numIn].currValInp == (uint32_t)0)
          {
            if (numIn == (IN1_REMOTE_EXP0))
            {
              tPrintf("remote chiuso at power on!\n\r");
              EVLOG_Message (EV_INFO, "Remote chiuso at power on!");                    
              send_to_extinp(REMOTE_SWITCH_UPDATE);
            }
            else if (numIn == IN2_PULSANTE_UP_PIN_UP)
            {
              tPrintf("Pulsante premuto at power on!\n\r");
              EVLOG_Message (EV_INFO, "Pulsante premuto at power on!");                                  
              send_to_extinp(PULS_SWITCH_UPDATE);
            }
            else if (numIn == (IN4_RCBO_EXP0))
            {
              tPrintf("rcbo close at power on!\n\r");
              EVLOG_Message (EV_INFO, "RCBO close at power on!");                                  
              send_to_evs(EVS_RCBO_CLOSE);
            }
            else if (numIn == (IN3_MIRROR_EXP0))
            {
//              tPrintf("mirror close at power on!\n\r");
//              send_to_evs(EVS_MIRROR_READ);
            }
            else if (numIn == IN5_BLK_UP_PIN_UP)
            {
              tPrintf("Block position switch up close at power on!\n\r");
              EVLOG_Message (EV_INFO, "Block position switch up close at power on!");                                                
              send_to_block(BLOCK_SWITCH_UP);
            }
            else if (numIn == IN6_BLK_DWN_PIN_UP)
            {
              tPrintf("Block position switch down close at power on!\n\r");
              EVLOG_Message (EV_INFO, "Block position switch down close at power on!");                                                
              send_to_block(BLOCK_SWITCH_DOWN);
            }
            else if (numIn == (IN8_LID_EXP0))
            {
              tPrintf("Lid position switch close at power on!\n\r");
              EVLOG_Message (EV_INFO, "Lid position switch close at power on!");                                                
              send_to_evs(EVS_LID_UPDATE);
            }
            else if (numIn == (IN7_VENT_EXP0))
            {
              tPrintf("Ventilazione ON\n\r");
              EVLOG_Message (EV_INFO, "Ventilazione ON");                                                
              send_to_evs(EVS_VENT_UPDATE);
            }
            else if (numIn == RCDM_IN_UP_PIN_UP)
            {
              tPrintf("rcdm close at power on!\n\r");
              EVLOG_Message (EV_INFO, "RCDM close at power on!");                                                
            }
            
            else if (numIn == (PEN_ALARM))
            {
              send_to_evs(EVS_PEN_ALM_OFF);
            }
            else if (numIn == SWPROG_PIN_UP)
            {
              tPrintf("SW_PROG premuto at power on!\n\r");
//            send_to_evs(EVS_SW_PROG);
              EVLOG_Message (EV_INFO, "SW_PROG premuto at power on!");                                                
              send_to_extinp(EXTINP_SW_PROG);
            }

            /* for this input the filtering autome start in reset state */
            inpMng[numIn].stato = INP_STATE_RESET;
          }
          else
          {
            if (numIn == (IN1_REMOTE_EXP0))
            {
              tPrintf("remote aperto at power on!\n\r");
              EVLOG_Message (EV_INFO, "Remote aperto at power on!");                                                
              send_to_extinp(REMOTE_SWITCH_UPDATE);
            }
            else if (numIn == IN2_PULSANTE_UP_PIN_UP)
            {
              tPrintf("Pulsante non premuto at power on!\n\r");
              EVLOG_Message (EV_INFO, "Pulsante non premuto at power on!");                                                
              send_to_extinp(PULS_SWITCH_UPDATE);
            }
            else if (numIn == (IN4_RCBO_EXP0))
            {
              tPrintf("rcbo open at power on!\n\r");
              EVLOG_Message (EV_INFO, "RCBO open at power on!");                                                              
            }
            else if (numIn == (IN3_MIRROR_EXP0))
            {
//              tPrintf("mirror open at power on!\n\r");
//              send_to_evs(EVS_MIRROR_READ);
            }
            else if (numIn == (IN8_LID_EXP0))
            {
              tPrintf("Lid position switch open at power on!\n\r");
              EVLOG_Message (EV_INFO, "Lid position switch open at power on!");                                                                            
              send_to_evs(EVS_LID_UPDATE);
            }
            else if (numIn == (IN7_VENT_EXP0))
            {
              tPrintf("Ventilazione OFF\n\r");
              send_to_evs(EVS_VENT_UPDATE);
            }
            else if (numIn == RCDM_IN_UP_PIN_UP)
            {
              tPrintf("rcdm open at power on!\n\r");
              EVLOG_Message (EV_INFO, "RCDM open at power on!");                                                                            
              send_to_evs(EVS_RCDM_OPEN);
            }
            else if (numIn == (PEN_ALARM))
            {
              send_to_evs(EVS_PEN_ALM_ON);
            }
            else if (numIn == SWPROG_PIN_UP)
            {
              tPrintf("SW_PROG aperto at power on!\n\r");
              EVLOG_Message (EV_INFO, "SW_PROG aperto at power on!");                                                                            
              send_to_extinp(EXTINP_SW_PROG);
            }

            /* for this input the filtering autome start in set state */
            inpMng[numIn].stato = INP_STATE_SET;

            if (numIn == IN6_BLK_DWN_PIN_UP)
            {
              if ((inpMng[IN5_BLK_UP_PIN_UP].stato == INP_STATE_SET) && (inpMng[IN6_BLK_DWN_PIN_UP].stato == INP_STATE_SET))	// blocco rimasto a metà strada
              {
                temp = getFlagV230();
                if ((temp != BACKUP_WAIT_END_POWER))
                {
                  tPrintf("blocco presa rimasto a meta' strada!\n\r");
                  EVLOG_Message (EV_ERROR, "Blocco presa is stuck!");                                                                            
                  send_to_block(BLOCK_SWITCH_UP);
                }
                else
                {
                  tPrintf("Sensori errati in uscita da sleep mode!\n\r");
                  EVLOG_Message (EV_ERROR, "Sensor error exiting from Sleep mode!");                                                                            
                  send_to_block(BLOCK_SWITCH_DOWN);
                }
              }
            }
          }
          break;
 
        case INP_STATE_RESET:
          if (inpMng[numIn].currValInp != (valInputs & maskIn) && ((valInputs & maskIn) != (uint32_t)0))
          {
            /* possible change pin status from low --> high */
            if ((numIn == (uint8_t)IN3_MIRROR_EXP0) && (getEmergencyStatus() != EMRG_ST_VBUS_DIS) && (isV230PresentValid() == FALSE)) 
            {
              inpMng[numIn].currValInp = GPIO_PIN_RESET;
              break;
            }
            inpMng[numIn].stato = INP_STATE_GOING_HIGH;
            inpMng[numIn].currFilteringInp = (uint16_t)0;
          }
          break;

        case INP_STATE_GOING_HIGH:
          if (inpMng[numIn].currValInp != (valInputs & maskIn) && ((valInputs & maskIn) != (uint32_t)0))
          {
            /* possible change pin status from low --> high */
            inpMng[numIn].currFilteringInp++;
            if (inpMng[numIn].currFilteringInp >= inpMng[numIn].filteringThreshold)
            {
              /* confirm the change */
              inpMng[numIn].stato = INP_STATE_SET;             /* 31 30  ........ 1 0*/
              inpMng[numIn].currValInp = (valInputs & maskIn); /* 0x0000.....1... 0 0 */

              if (numIn != IN3_MIRROR_EXP0)
	              tPrintf("%s to HIGH\n\r", inpName[numIn]);

              send_to_contact((uint8_t)CONTACT_IO_UPD); 

            if (numIn == (IN1_REMOTE_EXP0))
              {
                tPrintf("remote aperto !\n\r");
                EVLOG_Message (EV_INFO, "Remote input is OPEN");                                                                            
                send_to_extinp(REMOTE_SWITCH_UPDATE);
              }
              else if (numIn == IN9_RFID_DTC_PIN_UP)
              {
                tPrintf("RFID card released!\n\r");
                EVLOG_Message (EV_INFO, "RFID card is released");                                                                            
                send_to_rfid(RFID_CARD_RELEASED);
              }
              else if (numIn == IN2_PULSANTE_UP_PIN_UP)
              {
                tPrintf("Pulsante rilasciato !\n\r");
                EVLOG_Message (EV_INFO, "Button is released");                                                                            
                send_to_extinp(PULS_SWITCH_UPDATE);
                /* Macchina di collaudo: pulsante trattato come errore   */                
                setModbusErrorTesting(IN2_PULSANTE_UP_PIN_UP, GPIO_PIN_RESET);
              }
              else if (numIn == (IN4_RCBO_EXP0))
              {
                tPrintf("rcbo open!\n\r");
                EVLOG_Message (EV_INFO, "RCBO is open");                                                                                            
              }
              else if (numIn == (IN3_MIRROR_EXP0))
              {
//                tPrintf("mirror open!\n\r");
                /* If autotest is active, notify the result of the test  */
                NOTIFY_AUTOTEST_RESULT (AUTOTEST_EVENT_MIRROR, TEST_PASSED);                          
              }
              else if (numIn == (IN8_LID_EXP0))
              {
                tPrintf("Lid position switch open!\n\r");
                EVLOG_Message (EV_INFO, "LID position switch is open");                                                                                                            
                send_to_evs(EVS_LID_UPDATE);
              }
              else if (numIn == (IN7_VENT_EXP0))
              {
                tPrintf("Ventilazione OFF\n\r");
                EVLOG_Message (EV_INFO, "Ventilation is OFF");                                                                                                            
                send_to_evs(EVS_VENT_UPDATE);
              }
              else if (numIn == RCDM_IN_UP_PIN_UP)
              {
                tPrintf("rcdm open!\n\r");
                EVLOG_Message (EV_INFO, "RCDM is open");                                                                                                            
                if (getCollaudoRunning() == TRUE)
                {
                  setModbusErrorTesting(RCDM_IN_UP_PIN_UP, GPIO_PIN_SET);
                }
                else
                {
                  send_to_evs(EVS_RCDM_OPEN);
                }
              }
              else if (numIn == (PEN_ALARM))
              {
                send_to_evs(EVS_PEN_ALM_ON);
                tPrintf("PEN alarm ON!\n\r");
                EVLOG_Message (EV_INFO, "PEN alarm is ON");                                                                                                            
              }
              else if (numIn == SWPROG_PIN_UP)
              {
                tPrintf("SW_PROG aperto!\n\r");
                EVLOG_Message (EV_INFO, "SW PROG is open");                                                                                                            
                send_to_extinp(EXTINP_SW_PROG);
              }
            }
          }
          else
          {
            /* just a spike, so come back */
            inpMng[numIn].stato = INP_STATE_RESET;
          }
          break;

        case INP_STATE_SET:
          if (inpMng[numIn].currValInp != (valInputs & maskIn) && ((valInputs & maskIn) == (uint32_t)0))
          {
            /* possible change pin status from high --> low */
            if ((numIn == (uint8_t)IN3_MIRROR_EXP0) && (getEmergencyStatus() != EMRG_ST_VBUS_DIS) && (isV230PresentValid() == FALSE))
            {
              inpMng[numIn].currValInp = GPIO_PIN_SET;
              break;
            }
            inpMng[numIn].stato = INP_STATE_GOING_LOW;
            inpMng[numIn].currFilteringInp = (uint16_t)0;
          }
          break;

        case INP_STATE_GOING_LOW:
          if (inpMng[numIn].currValInp != (valInputs & maskIn) && ((valInputs & maskIn) == (uint32_t)0))
          {
            /* confirm possible change pin status from high --> low */
            inpMng[numIn].currFilteringInp++;
            if (inpMng[numIn].currFilteringInp >= inpMng[numIn].filteringThreshold)
            {
              /* confirm the change */
              inpMng[numIn].stato = INP_STATE_RESET;             /* 31 30  ........ 1 0*/
              inpMng[numIn].currValInp = (uint16_t)0;            /* 0x0000.....0... 0 0 */

              send_to_contact((uint8_t)CONTACT_IO_UPD); 

              if (numIn == (IN1_REMOTE_EXP0))
              {
                tPrintf("remote chiuso !\n\r");
                EVLOG_Message (EV_INFO, "Remote input is closed");                                                                                                            
                send_to_extinp(REMOTE_SWITCH_UPDATE);
              }
              else if (numIn == IN2_PULSANTE_UP_PIN_UP)
              {
                tPrintf("Pulsante premuto!\n\r");
                EVLOG_Message (EV_INFO, "IN2 input has been pressed");                                                                                                            
                send_to_extinp(PULS_SWITCH_UPDATE);
                /* If autotest is active, notify the result of the test  */                
                NOTIFY_AUTOTEST_RESULT (AUTOTEST_EVENT_MOTOR_1, TEST_PASSED);          
                /* Macchina di collaudo: pulsante trattato come errore   */                
                setModbusErrorTesting(IN2_PULSANTE_UP_PIN_UP, GPIO_PIN_SET);
              }
              else if (numIn == (IN4_RCBO_EXP0))
              {
                tPrintf("rcbo close!\n\r");
                EVLOG_Message (EV_INFO, "RCBO is closed");                                                                                                            
                send_to_evs(EVS_RCBO_CLOSE);
              }
              else if (numIn == (IN3_MIRROR_EXP0))
              {
//                tPrintf("Mirror close!\n\r");
             }
              else if (numIn == IN5_BLK_UP_PIN_UP)
              {
                tPrintf("Block position switch up close!\n\r");
                EVLOG_Message (EV_INFO, "Block position switch up close!");                                                                                                            
                send_to_block(BLOCK_SWITCH_UP);
                /* If autotest is active, notify the result of the test  */                
                NOTIFY_AUTOTEST_RESULT (AUTOTEST_EVENT_MOTOR_2, TEST_PASSED);          
              }
              else if (numIn == IN6_BLK_DWN_PIN_UP)
              {
                tPrintf("Block position switch down close!\n\r");
                EVLOG_Message (EV_INFO, "Block position switch down close!");                                                                                                            
                send_to_block(BLOCK_SWITCH_DOWN);
                /* If autotest is active, notify the result of the test  */                
                NOTIFY_AUTOTEST_RESULT (AUTOTEST_EVENT_MOTOR_3, TEST_PASSED);          
              }
              else if (numIn == (IN8_LID_EXP0))
              {
                tPrintf("Lid position switch close!\n\r");
                EVLOG_Message (EV_INFO, "Lid position switch close!");                                                                                                            
                send_to_evs(EVS_LID_UPDATE);
              }
              else if (numIn == (IN7_VENT_EXP0))
              {
                tPrintf("Ventilazione ON\n\r");
                EVLOG_Message (EV_INFO, "Ventilation is ON");                                                                                                            
                send_to_evs(EVS_VENT_UPDATE);
              }
              else if (numIn == IN9_RFID_DTC_PIN_UP)
              {
                tPrintf("RFID card detected!\n\r");
                EVLOG_Message (EV_INFO, "RFID card detected!");                                                                                                            
                send_to_rfid(RFID_CARD_DETECT);
              }
              else if (numIn == RCDM_IN_UP_PIN_UP)
              {
                tPrintf("rcdm close!\n\r");
                EVLOG_Message (EV_INFO, "RCDM is closed!");                                                                                                            
                if (getCollaudoRunning() == TRUE)
                {
                  setModbusErrorTesting(RCDM_IN_UP_PIN_UP, GPIO_PIN_RESET);
                }
              }
              else if (numIn == (PEN_ALARM))
              {
                send_to_evs(EVS_PEN_ALM_OFF);
                tPrintf("PEN alarm OFF!\n\r");
                EVLOG_Message (EV_INFO, "PEN alarm OFF!");                                                                                                            
              }
              else if (numIn == SWPROG_PIN_UP)
              {
                tPrintf("SW_PROG premuto!\n\r");
                EVLOG_Message (EV_INFO, "SW_PROG premuto!");                                                                                                            
//              send_to_evs(EVS_SW_PROG);
                send_to_extinp(EXTINP_SW_PROG);
              }
              else
              {
                tPrintf("%s to LOW\n\r", inpName[numIn]);
              }
            }
          }
          else
          {
            /* just a spike, so come back */
            inpMng[numIn].stato = INP_STATE_SET;
          }
          break;

        default:
          break;
      }
    } // end for
    /* check emergency message presence */
    temp = getFlagV230Msg();
    switch (temp)
    {
      case MSG_230VAC_OFF_OPEN_SOCKET:
        send_to_evs((uint8_t)EVS_V230_OFF);
        setBootEvent(ANOMALY_REBOOT);
        /* reset the msg flag */
        setFlagV230Msg((uint32_t)0);
        tPrintf("Assenza tensione!\n\r");
        EVLOG_Message (EV_INFO, "Assenza tensione!");      
        break;
        
      case MSG_230VAC_ON_START_RECHARGE:
        send_to_evs((uint8_t)EVS_V230_ON); 
        /* reset the msg flag */
        setFlagV230Msg((uint32_t)0);
        tPrintf("Ritorno tensione!\n\r");
        EVLOG_Message (EV_INFO, "Ritorno tensione!");      
        break;

      case MSG_230VAC_SUSPEND_RECHARGE:
        send_to_evs((uint8_t)EVS_V230_SUSPEND);
        /* reset the msg flag */
        setFlagV230Msg((uint32_t)0);
        tPrintf("Sospensione!\n\r");
        EVLOG_Message (EV_INFO, "Sospensione!");              
        break;
        
        
      default:
        break;
      
    }
  } // end if on pMsg->taskEv == INP_EVENT_POLLING_ON_TO
  
  if (pMsg->taskEv == INP_EVENT_START_POLLING)
  {
    /* Init digital inputs */
    initDigitalInput();    
    newTimeTick = pdMS_TO_TICKS(SAMPLE_TICK);
  }
  return(newTimeTick);
}


/**
*
* @brief        Get the pointer to FAT memager queue
*
* @param [in]   none
*
* @retval       xQueueHandle: pointer to defined fat mng queue
*
***********************************************************************************************************************/
xQueueHandle getInptMngQueueHandle(void)
{
   return(inpMngQueue);
}

/**
*
* @brief       get an input pin status 
*
* @param [in]  uni_TypeDef:  input id 
*  
* @retval      GPIO_PinState:  GPIO_PIN_RESET / GPIO_PIN_SET
*  
****************************************************************/
GPIO_PinState getInput(dIn_TypeDef pinId)
{
  if (inpMng[pinId].currValInp == (uint16_t)0)
  {
    return(GPIO_PIN_RESET);
  }
  else
  {
    return(GPIO_PIN_SET);
  }
}


inpStates_e getInputStato(dIn_TypeDef pinId)
{
  if ((getCollaudoRunning() == TRUE) &&  (pinId == IN3_MIRROR_EXP0))
  {
     return(INP_STATE_SET);
  }
  else
  {
    return(inpMng[pinId].stato);
  }
}


/**
*
* @brief  		  Returns all the input pins status    
*
* @param [out]  uint8_t* - pointer where to store the status af the pins
*  
* @param [in]   none
*  
* @note         none
*  
* @retval       uint32_t: right aligned, the status for all inputs 
*  
***********************************************************************************************************************/
uint32_t readDigitalInput (void)
{
  uint32_t mask, statoInputs;
  uint8_t  i;

  for (i = 0, mask = (uint32_t)1, statoInputs = 0U; i < NUM_INPUT_SCU_PIN_UP; i++, mask = mask << 1)
  {
    if (HAL_GPIO_ReadPin(UIN_PORT[i], UIN_PIN[i]) == GPIO_PIN_SET)
    {
      statoInputs |= mask;
    }
    else
    {
      statoInputs &= (~mask);
    }
  }
  return(statoInputs);
}

/**
*
* @brief  		  set input digital pin with pull-resistor     
*
* @param [out]  none
*  
* @param [in]   none
*  
* @note         none
*  
* @retval       none 
*  
***********************************************************************************************************************/
static void initDigitalInput (void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint8_t pinCnt;

  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  for (pinCnt = 0; pinCnt < (uint8_t)IN9_RFID_DTC_PIN_UP; pinCnt++)
  {
    if (pinCnt != (uint8_t)RCDM_IN_UP_PIN_UP)
    {
      GPIO_InitStruct.Pin = UIN_PIN[pinCnt];
      HAL_GPIO_Init(UIN_PORT[pinCnt], &GPIO_InitStruct);
    }
    else
    {
      setRcdmPinAsIntrpt();
    }
  }
}

/**
*
* @brief        set RCDM input  as interrupt EXTI9_5 interrupt      
*
* @param [in]   none
*
* @retval       none  
*
***********************************************************************************************************************/
static void setRcdmPinAsIntrpt (void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Configure PE.7 pin as input pull-up */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_PULLUP; 
  GPIO_InitStructure.Pin = UIN_PIN[RCDM_IN_UP_PIN_UP];
  HAL_GPIO_Init(UIN_PORT[RCDM_IN_UP_PIN_UP], &GPIO_InitStructure);

  /* Enable and set EXTI lines 9 to 5 Interrupt to the lowest priority */
#ifdef GD32F4xx  
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
#else
  HAL_NVIC_SetPriority(EXTI7_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI7_IRQn);  
#endif  
}

/**
*
* @brief  		  Returns all the input pins status    
*
* @param [out]  uint8_t* - pointer where to store the status af the pins
*  
* @param [in]   none
*  
* @note         none
*  
* @retval       uint32_t: right aligned, the status for all inputs 
*  
***********************************************************************************************************************/

#ifdef HW_MP28947  
GPIO_PinState GPIO_Read_IN3_MIRROR (void)
{

   return HAL_GPIO_ReadPin(UIN_PORT[IN3_MIRROR_EXP0], UIN_PIN[IN3_MIRROR_EXP0]);
   
}
#endif

/**
*
* @brief       get a pointer to time string  
*
* @param [in]  none 
*  
* @retval      GPIO_PinState:  GPIO_PIN_RESET / GPIO_PIN_SET
*  
****************************************************************/
char*  getTimePtr(void)
{
  return(strTime);
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //



/*************** END OF FILE ******************************************************************************************/


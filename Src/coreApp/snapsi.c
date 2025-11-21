/******************* (C) COPYRIGHT 2009 STMicroelectronics *********/
/**
* @file        sinapsi.c
*
* @brief       Manager chain 2 protocol (SINAPSI - RSE)  - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: snapsi.c 682 2025-02-11 11:33:08Z npiergi $
*
*     $Revision: 682 $
*
*     $Author: npiergi $
*
*     $Date: 2025-02-11 12:33:08 +0100 (mar, 11 feb 2025) $
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
#include "wrapper.h"
#include "rtcApi.h"
#include "sinapsi.h"
#include "eeprom.h"
#include "PwmMng.h"

#define   DEFAULT_SINAPSI_TIME             pdMS_TO_TICKS((uint16_t)1000)

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#define     TIMER_SETUP_500                ((uint16_t)500)

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 
static    sinapsiMng_t      sinapsiMng;
static    sinapsiMngMsg_st  sinapsiMngMsg;
static    TimerHandle_t     xSinapsiTimers[NUM_SINAPSI_TIMER];
#ifdef SINAPSI_RSE
static    uint32_t          msgCounter;
#endif
static    uint16_t          sinapsiM1Pd, chain2Status;
/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local const                                   **
**                                                                          **
****************************************************************************** 
*/                               
                                  /* IOM_CTRL  */
const rseSetReg_st rseDefaultInfo = {(uint16_t)1, 
                                  /*  M2_PAII_TS      M2_PAII        M2_PC_TS           M2_PC           */
                                     (uint32_t)0,     (uint16_t)0,   (uint32_t)0,       (uint16_t)0,             
         
                                  /*  M1_PC_TS        M1_PC           M1_PD_TS          M1_PD           */
                                     (uint32_t)0,     (uint16_t)3000, (uint32_t)0,      (uint16_t)3000,

                                  /*  M1_PAPI_TS      M1_PAPI         M1_PAII_TS        M1_PAII           */
                                     (uint32_t)0,     (uint16_t)0,    (uint32_t)0,      (uint16_t)0,

                                  /*  M1_RES_DIST_TS  M1_TIME_RES_DIST  M1_PFO_TS        M1_PFO        */
                                     (uint32_t)0,     (uint16_t)0,    (uint32_t)0,      (uint16_t)0,

                                  /*  M1_PMRTLIM      M1_PMRTLIM_TS   M1_SOSP_RIC_TLIM M1_UT_SYNC      */
                                      (uint16_t)0,    (uint32_t)0,    (uint32_t)0,      (uint32_t)0
}; 


/*
*********************************** SCAME ************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 
/*  Input manager queue  declaration */
xQueueHandle sinapsiMngQueue = NULL;

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
static void     sinapsiManager        (sinapsiMngMsg_st* pMsg);
static void     sinapsiTimCallBack    (TimerHandle_t pxTimer);
#ifdef SINAPSI_RSE
static uint32_t rseCheckSum           (rseSetRegEEPROM_st* pInfo);
static uint32_t rseStoreInfo          (rseSetRegEEPROM_st* pInfo);
static void     rseUpdateInfoAndSend  (rseSetReg_st* pInfo);
#endif
/*
*********************************** SCAME ************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/


/**
*
* @brief       Gestione dati chain 2 from sinapsi
*
* @param [in]  (void const *: not used  
*  
* @retval      none 
*  
****************************************************************/
void sinapsiMngTask (void * pvParameters)
{
  uint8_t         ix;

  /*-------- Creates an empty mailbox for uart SITOS messages --------------------------*/
  sinapsiMngQueue = xQueueCreate(SINAPSI_MAX_MESSAGE_NUM, sizeof(sinapsiMngMsg_st));
  configASSERT(sinapsiMngQueue != NULL);

  sinapsiMng.stato        = SINAPSI_STATE_IDLE;
  sinapsiMng.activeModule = (uint16_t)0;
#ifdef SINAPSI_RSE
  msgCounter              = (uint32_t)0;
#endif

  /*-------- Creates all timer for the led task  --------------------------*/
  for (ix = (uint8_t)0; ix < (uint8_t)NUM_SINAPSI_TIMER; ix++)
  {
    /* in this case we use the auto-reload features */
    xSinapsiTimers[ix] = xTimerCreate("TimSinapsi", DEFAULT_SINAPSI_TIME, pdFALSE, (void*)(ix), sinapsiTimCallBack);
    configASSERT(xSinapsiTimers[ix] != NULL);
  }
  sinapsiM1Pd = chain2Status = (uint16_t)0;

  for (;;)
  {
    /* Wait for some event from SW to change led status */
    if (xQueueReceive(sinapsiMngQueue, (void *)&sinapsiMngMsg, portMAX_DELAY) == pdPASS)
    {
      sinapsiManager((sinapsiMngMsg_st *)&sinapsiMngMsg);
    }
    else
    {
      /* timeout: not used  */
    }
  }
}



/**
*
* @brief        esegue gestione info ricevute dal modulo IOM2G chain 2 sinapsi
*
* @param [in]   sinapsiMngMsg_st*: puntatore al messaggio da eseguire
*
* @retval       none
*
***********************************************************************************************************************/
static void sinapsiManager (sinapsiMngMsg_st* pMsg)
{
  rseSetRegEEPROM_st* pSrc;
#ifdef SINAPSI_RSE
  rseSetReg_st*       pTmp;
#else
  uint16_t            tmp;
#endif

  switch (sinapsiMng.stato)
  {
    case SINAPSI_STATE_IDLE:
      switch (pMsg->sinapsiEvents)
      {
#ifdef SINAPSI_RSE
        case SINAPSI_EVENT_START:
          sinapsiMng.activeModule = (uint16_t)0;

          /**** mettere lettura EEPROM (?) ****/
          pSrc = &sinapsiMng.rseSetRegEEPROM;
          ReadFromEeprom(RSE_CONFIG_EEPROM_ADDRESS, (uint8_t*)pSrc, sizeof(rseSetRegEEPROM_st));

          if (rseCheckSum(pSrc) == (uint32_t)0)
          {
            pTmp = getRWSinapsiInfo();
            /**** in the EEPROM there are correct info Copy it in modbus map ***/
            /*      destination                                          source                  length */
            memcpy((void*)pTmp, (void*)&sinapsiMng.rseSetRegEEPROM.rseSetRegRAM, sizeof(rseSetReg_st));
          }
          else
          {
            /**** no info in EEPROM: use default data  ***/
            /*      destination      source                  length */
            memcpy((void*)&pSrc->rseSetRegRAM, (void*)&rseDefaultInfo, sizeof(rseSetReg_st));
            (void)rseStoreInfo(pSrc);
          }
          resetSinapsiIOM2CtrlReg();
          sinapsiMng.rseSetRegEEPROM.rseSetRegRAM.iomCtrl = (uint16_t)0;
          
          rseUpdateInfoAndSend(&pSrc->rseSetRegRAM);
          sinapsiMng.stato = SINAPSI_STATE_WAIT_MODULES;
          break;
#else
        case SINAPSI_EVENT_PM_CONN:
          /* set HW switch */
          setSinapsiPinRS485();
          /**** init with  default data  ***/
          pSrc = &sinapsiMng.rseSetRegEEPROM;
          /*      destination      source                  length */
          memcpy((void*)&pSrc->rseSetRegRAM, (void*)&rseDefaultInfo, sizeof(rseSetReg_st));
          /* from EEPROM get the "available" power */
          // xx eeprom_param_get(PMNG_PWRLSB_EADD, (uint8_t *)&tmp, 2);          
          sinapsiMng.activeModule = ((uint16_t)IOM2G_ACTIVE_MASK | (uint16_t)PM_ACTIVE_MASK);
          sinapsiMng.sinapsiSetReg.m1Pd = (infoStation.Pmng.Power * (uint16_t)100); 
          sinapsiMng.stato = SINAPSI_STATE_FULL_CONN;          
          break;

#endif
        default:
          break;
      }
      break;

    case SINAPSI_STATE_FULL_CONN:
    case SINAPSI_STATE_WAIT_MODULES:
      switch (pMsg->sinapsiEvents)
      {
#ifdef SINAPSI_RSE
        case SINAPSI_EVENT_CHANGE:
          /* upgrade module state and reset control register */
          sinapsiMng.activeModule |= (uint16_t)IOM2G_ACTIVE_MASK;
          if ((sinapsiMng.activeModule & ((uint16_t)IOM2G_ACTIVE_MASK | (uint16_t)PM_ACTIVE_MASK)) == ((uint16_t)IOM2G_ACTIVE_MASK | (uint16_t)PM_ACTIVE_MASK))
          {
            sinapsiMng.stato = SINAPSI_STATE_FULL_CONN;          
          }
          resetSinapsiIOM2CtrlReg();

          pTmp = getRWSinapsiInfo(); 
          rseUpdateInfoAndSend(pTmp); 
          /******************* save the current SINAPSI modbus map **********************************/
          /*                         destination                     source                  length */
          memcpy((void*)&sinapsiMng.rseSetRegEEPROM.rseSetRegRAM, (void*)pTmp, sizeof(rseSetReg_st));
          /****  salvataggio parametri in  EEPROM  ****/
          pSrc = &sinapsiMng.rseSetRegEEPROM;
          (void)rseStoreInfo(pSrc);
          msgCounter++;
          tPrintf ("Message Num = %d\r\n", msgCounter);
          
          break;

        case SINAPSI_EVENT_PM_CONN:
          sinapsiMng.activeModule |= (uint16_t)PM_ACTIVE_MASK;
          if ((sinapsiMng.activeModule & ((uint16_t)IOM2G_ACTIVE_MASK | (uint16_t)PM_ACTIVE_MASK)) == ((uint16_t)IOM2G_ACTIVE_MASK | (uint16_t)PM_ACTIVE_MASK))
          {
            sinapsiMng.stato = SINAPSI_STATE_FULL_CONN;          
          }
          break;
#else

#endif
        default:
          break;

      }
      break;

    default:
      break;

  }

}

/**
*
* @brief        Get the pointer to Led manager queue
*
* @param [in]   none
*
* @retval       xQueueHandle: pointer to defined led mng queue
*
***********************************************************************************************************************/
xQueueHandle getSinapsiMngQueueHandle(void)
{
   return(sinapsiMngQueue);
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
static void sinapsiTimCallBack (TimerHandle_t pxTimer)
{
  uint32_t          timer_id;

  /* find the led  which the timer is referred */
  timer_id = (uint32_t)pvTimerGetTimerID(pxTimer);

  switch (timer_id)
  {
    case (uint32_t)TIM_DIST:
      sinapsiMng.sinapsiSetReg.m1TimeResDistBool = (uint16_t)1;
      /****** informare station manager per il Power Managent   */
      /* send event to station manager */
      send_to_pwm(PWM_SINAPSI_UPDATE);
      send_to_evs(EVS_SINAPSI_UPDATE);
      tPrintf(" Distacco contatore M1 imminente!\n\r");
      break;

    case (uint32_t)TIM_SUSP:
      /* trascorso il tempo di sospensione forzata    */
      sinapsiMng.sinapsiSetReg.m1SospRicTlimBool = (uint16_t)0;
      /****** informare station manager per riprendere la normale gestione    */
      /* send event to station manager */
      send_to_pwm(PWM_SINAPSI_UPDATE);
      send_to_evs(EVS_SINAPSI_UPDATE);
      tPrintf(" Fine sospensione forzata!\n\r");
      break;

    case (uint32_t)TIM_PMAX:
      /* trascorso il tempo di utilizzo extra power    */
      /* return back to m1Pd at 500W step every 10 sec */
      if ((sinapsiMng.sinapsiSetReg.m1Pd - sinapsiMng.rseSetRegEEPROM.rseSetRegRAM.m1Pd) >= STEP_DEC_POWER)
      {
        sinapsiMng.sinapsiSetReg.m1Pd -= STEP_DEC_POWER;
        while ((xTimerChangePeriod (xSinapsiTimers[TIM_PMAX], pdMS_TO_TICKS(STEP_DEC_TIME * 1000), SINAPSI_TIMER_GARD_TIME) != pdPASS)); 
        tPrintf(" Periodo extra Power at %d[W]!\n\r", sinapsiMng.sinapsiSetReg.m1Pd);
      }
      else
      {
        sinapsiMng.sinapsiSetReg.m1PmrTlim = (uint16_t)0;
        sinapsiMng.sinapsiSetReg.m1Pd = sinapsiMng.rseSetRegEEPROM.rseSetRegRAM.m1Pd;
        tPrintf(" Fine periodo extra Power!\n\r");
      }
      /****** informare station manager per riprendere la normale gestione    */
      /* send event to station manager */
      send_to_pwm(PWM_SINAPSI_UPDATE);
      send_to_evs(EVS_SINAPSI_UPDATE);
      break;

    default:
      break;
  }
}

#ifdef SINAPSI_RSE
/**
*
* @brief        Chek if the data info is OK   
*
* @param [in]   rseSetRegEEPROM_st: pointer to info structure  
*
* @retval       uint32_t: 0 if cheksum is OK 
*
***********************************************************************************************************************/
static uint32_t rseCheckSum (rseSetRegEEPROM_st* pInfo)
{
  uint16_t  ix;
  uint32_t  val;
  uint8_t*  pData;

  pData = (uint8_t*)pInfo;
  for (ix = 0, val = (uint32_t)0; ix < (sizeof(rseSetRegEEPROM_st) - sizeof(pInfo->checsumInfo)); ix++)
  {
    val += pData[ix];
  }
  if ((val != 0) && (val == pInfo->checsumInfo))
  {
    return(0);
  }
  return(1);
}

/**
*
* @brief        Find checsum for data info and store it in EEPROM   
*
* @param [in]   rseSetRegEEPROM_st: pointer to info structure  
*
* @retval       uint32_t: 0 if operation is OK  
*
***********************************************************************************************************************/
static uint32_t rseStoreInfo (rseSetRegEEPROM_st* pInfo)
{
  uint16_t  ix;
  uint32_t  val;
  uint8_t*  pData;

  pData = (uint8_t*)pInfo;
  for (ix = 0, val = (uint32_t)0; ix < (sizeof(rseSetRegEEPROM_st) - sizeof(pInfo->checsumInfo)); ix++)
  {
    val += pData[ix];
  }
  pInfo->checsumInfo = val;

  val = WriteOnEeprom(RSE_CONFIG_EEPROM_ADDRESS, (uint8_t*)pData, sizeof(rseSetRegEEPROM_st));

  return(val);
}

/**
*
* @brief        Find checsum for data info and store it in EEPROM   
*
* @param [in]   rseSetReg_st: pointer to info sinapsi structure  
*
* @retval       none  
*
***********************************************************************************************************************/
static void  rseUpdateInfoAndSend (rseSetReg_st* pInfo)
{
  uint32_t  val, currTime;

  sinapsiMng.sinapsiSetReg.m2Paii = pInfo->m2Paii;
  sinapsiMng.sinapsiSetReg.m2Pc = pInfo->m2Pc;
  sinapsiMng.sinapsiSetReg.m1Pc = pInfo->m1Pc;
  sinapsiMng.sinapsiSetReg.m1Pd = pInfo->m1Pd;
  sinapsiMng.sinapsiSetReg.m1Papi = pInfo->m1Papi;
  sinapsiMng.sinapsiSetReg.m1Paii = pInfo->m1Paii;
  sinapsiMng.sinapsiSetReg.m1Pfo = pInfo->m1Pfo;
  sinapsiMng.sinapsiSetReg.m1PmrTlim = pInfo->m1PmrTlim;
  if ((sinapsiMng.rseSetRegEEPROM.rseSetRegRAM.m1UtSync != pInfo->m1UtSync) && (pInfo->m1UtSync > (uint32_t)1635156981))
  {
    /* the unix time is changed ed è posteriore al 25-10-2021: so it is possibile to resyncronize the RTC */
    setDateTimeFromUnixT(pInfo->m1UtSync);
  }
  if ((pInfo->m1TimeResDist != (uint16_t)0) || (pInfo->m1SospRicTlimTs != (uint16_t)0) || (pInfo->m1PmrTlim != (uint16_t)0))
  {
    /* there one o more events on timeout to restore / set  */
    currTime = getCurrentUnixTime();
#ifdef YES_OVERWRITE
    if (pInfo->m1TimeResDist != (uint16_t)0xFFFF)  /* accept new set always  */
#else
    if ((pInfo->m1TimeResDist != (uint16_t)0xFFFF) && (checkSinapsiTimer(TIMER_DIST_T) == 0))  /* timer on Tdist isn'active */
#endif
    {
      val = currTime;
      sinapsiMng.sinapsiSetReg.m1TimeResDistBool = (uint16_t)0;
      if (val < (pInfo->m1ResDistTs + pInfo->m1TimeResDist))  /* current time < time when disconnection will be done */
      {
        val = (pInfo->m1ResDistTs + pInfo->m1TimeResDist) - val;   /* number of seconds to detach line for overload */
        tPrintf ("<<<< Tempo al distacco = %d [sec] >>>>>\r\n", val);
        /* reload timer for send the warning  */
        while ((xTimerChangePeriod (xSinapsiTimers[TIM_DIST], pdMS_TO_TICKS(val * 1000), SINAPSI_TIMER_GARD_TIME) != pdPASS)); 
      }
      else
      {
        pInfo->m1ResDistTs = (uint32_t)0;
        sinapsiMng.sinapsiSetReg.m1PmrTlim =  (uint16_t)0;
      }
    }
    else
    {
      if ((pInfo->m1TimeResDist == (uint16_t)0xFFFF) && (checkSinapsiTimer(TIMER_DIST_T) != 0))  /* timer on Tdist is active */
      {
          /* stop timer */
           xTimerStop(xSinapsiTimers[TIM_DIST], SINAPSI_TIMER_GARD_TIME);
      }
    }
#ifdef NO_OVERWRITE
    if ((pInfo->m1SospRicTlimTs != (uint16_t)0)  && (checkSinapsiTimer(TIMER_SUSP_T) == 0))  /* timer on Tsosp isn'active */
#else
      if (pInfo->m1SospRicTlimTs != (uint16_t)0) /* accept new set always  */
#endif
    {
      val = currTime;
      if (val < pInfo->m1SospRicTlimTs)
      {
        val = (pInfo->m1SospRicTlimTs - val);   /* number of seconds to be stay in suspend mode */
        sinapsiMng.sinapsiSetReg.m1SospRicTlimBool = (uint16_t)1;
        /* reload timer for send the warning  */
        while ((xTimerChangePeriod (xSinapsiTimers[TIM_SUSP], pdMS_TO_TICKS(val * 1000), SINAPSI_TIMER_GARD_TIME) != pdPASS)); 
      }
      else
      {
        pInfo->m1SospRicTlimTs = (uint32_t)0;
        sinapsiMng.sinapsiSetReg.m1SospRicTlimBool = (uint16_t)0;
      }
    }
#ifdef NO_OVERWRITE
    if ((pInfo->m1PmrTlim != (uint16_t)0) && (checkSinapsiTimer(TIMER_PMAX_T) == 0))  /* timer on TPmax isn'active */ 
#else
      if (pInfo->m1PmrTlim != (uint16_t)0) /* accept new set always  */
#endif
    {
     val = currTime;
     if (pInfo->m1PmrTlimTs > (uint32_t)STEP_DEC_TIME)
      {
        /* deltaP = PmaxT - m1Pd Incremento può essere immediato ma il ritorno alla m1Pd deve avvenire diminunedo 500W ogni 10 sec */
        /* Quindi a PmaxT si può andare per un tempo inferiore ovvero tLim - tCurr - 10 (deltaP / 500) */
        val = (pInfo->m1PmrTlimTs - STEP_DEC_TIME * ((pInfo->m1PmrTlim - sinapsiMng.rseSetRegEEPROM.rseSetRegRAM.m1Pd) / STEP_DEC_POWER));   /* number of seconds to be continue to use PmaxT  */
        /* reload timer for send the warning  */
        while ((xTimerChangePeriod (xSinapsiTimers[TIM_PMAX], pdMS_TO_TICKS(val * 1000), SINAPSI_TIMER_GARD_TIME) != pdPASS)); 
        sinapsiMng.sinapsiSetReg.m1Pd = pInfo->m1PmrTlim;
      }
      else
      {
        pInfo->m1PmrTlimTs = (uint32_t)0;
        sinapsiMng.sinapsiSetReg.m1PmrTlim = pInfo->m1PmrTlim = (uint16_t)0;
        sinapsiMng.sinapsiSetReg.m1Pd = sinapsiMng.rseSetRegEEPROM.rseSetRegRAM.m1Pd;
      }
    }
  }
  if (sinapsiMng.stato == SINAPSI_STATE_FULL_CONN)
  {
    /* send event to station manager */
    send_to_pwm(PWM_SINAPSI_UPDATE);
    send_to_evs(EVS_SINAPSI_UPDATE);
  }
}
#endif

/**
*
* @brief        Check if a timer is active for a function    
*
* @param [in]   sinapsiIdx_e: timer id   
*
* @retval       uint16_t: 0 timer off, != 0 time is active   
*
***********************************************************************************************************************/
uint32_t  checkSinapsiTimer (sinapsiIdx_e tId)
{
  TickType_t xRemainingTime;

  if(xTimerIsTimerActive(xSinapsiTimers[(uint8_t)tId])) 
  {
    xRemainingTime = xTimerGetExpiryTime( xSinapsiTimers[(uint8_t)tId] ) - xTaskGetTickCount();
    return((uint32_t)xRemainingTime);
  }
  return((uint32_t)0);
}

/**
*
* @brief        Get the pointer to IOM2G sinapsi meter info 
*
* @param [in]   none
*
* @retval       sinapsiSetReg_st*: pointer to IOM2G info structure
*
***********************************************************************************************************************/
sinapsiSetReg_st* getIom2Ginfo(void)
{
   return((sinapsiSetReg_st*)&sinapsiMng.sinapsiSetReg); 
}

/**
*
* @brief        set the power module activation  
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void setPMreadyForIom2G(void)
{
  sinapsiMngMsg_st  tmpSinapsiMsg;

  tmpSinapsiMsg.sinapsiEvents = SINAPSI_EVENT_PM_CONN;
  configASSERT(xQueueSendToBack(sinapsiMngQueue, (void *)&tmpSinapsiMsg, portMAX_DELAY) == pdPASS);
}

/**
*
* @brief        start the Sinapsi IOM2G management
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void startSinapsiIom2G(void)
{
  sinapsiMngMsg_st  tmpSinapsiMsg;

  tmpSinapsiMsg.sinapsiEvents = SINAPSI_EVENT_START;
  configASSERT(xQueueSendToBack(sinapsiMngQueue, (void *)&tmpSinapsiMsg, portMAX_DELAY) == pdPASS);
}

/**
*
* @brief        send an event when a new data from sinapsi is ready
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void newSinapsiIom2GData(void)
{
  sinapsiMngMsg_st  tmpSinapsiMsg;

  tmpSinapsiMsg.sinapsiEvents = SINAPSI_EVENT_CHANGE;
  configASSERT(xQueueSendToBack(sinapsiMngQueue, (void *)&tmpSinapsiMsg, portMAX_DELAY) == pdPASS);
}

/**
*
* @brief        Get the flag status for sinapsi activation
*
* @param [in]   none
*
* @retval       uint16_t: sinapsi statu activation
*
***********************************************************************************************************************/
uint16_t getSinapsiStatusActivation(void)
{
   return(sinapsiMng.activeModule);
}

/**
*
* @brief        Set sinapsi Pin switch for RS485
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void setSinapsiPinRS485(void)
{
#ifdef GD32F4xx  
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* SINAPSI is active LOW:  */
  HAL_GPIO_WritePin(SINAPSI_GPIO_Port, SINAPSI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SINAPSI_Pin PA5 */
  GPIO_InitStruct.Pin = SINAPSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(SINAPSI_GPIO_Port, &GPIO_InitStruct);
  
#endif
}

/**
*
* @brief        Set sinapsi default M1 power available 
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void setSinapsiM1PdDefault(uint16_t p1Val)
{
  sinapsiM1Pd = p1Val;
}

/**
*
* @brief        Get sinapsi default M1 power available 
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
uint16_t getSinapsiM1PdDefault(void)
{
  return(sinapsiM1Pd);
}

/**
*
* @brief        Set sinapsi chain2 status  
*
* @param [in]   uint16_t: chain 2 status 0 = fail 1 = good
*
* @retval       none
*
***********************************************************************************************************************/
void setSinapsiChain2Status(uint16_t p1Val)
{
  chain2Status = p1Val;
}

/**
*
* @brief        Get sinapsi chain2 status 
*
* @param [in]   none
*
* @retval       uint16_t: chain 2 status 0 = fail 1 = good
*
***********************************************************************************************************************/
uint16_t getSinapsiChain2Status(void)
{
  return(chain2Status);
}



/*************** END OF FILE ******************************************************************************************/


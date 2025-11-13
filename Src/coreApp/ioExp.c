/******************* (C) COPYRIGHT 2009 STMicroelectronics *********/
/**
* @file        ioExp.c
*
* @brief       Manager IO expander on I2C - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: ioExp.c 750 2025-05-08 10:20:36Z stefano $
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
#include "ioExp.h"
#include "InputsMng.h"
#include "ContactMng.h"
#include "RfidMng.h"
#include "scuMdb.h"


/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#define     TIMER_SETUP_500                ((uint16_t)500)

#define     MASK_GPIO1_IOEXP1              ((uint8_t)0x01)
/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 
static   ioMng_t          ioMng;
static   ioMngMsg_st      ioMngMsg;

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local const                                   **
**                                                                          **
****************************************************************************** 
*/ 
  /* default for U39.3=PEN_ALM is the only pin with interrupt capability  */
static const ioExpMap_st ioExpMap = {
  {DEVICEID_CTRL_IDX,           DEVICEID_CTRL_REG,              0U}, 
  {IO_DIR_IDX,                  IO_DIR_REG,                     0x34U}, 
  {OUTPUT_STATE_IDX,            OUTPUT_STATE_REG,               0x04U},
  {OUTPUT_STATE_HIZ_IDX,        OUTPUT_STATE_HIZ_REG,           0U}, 
  {INPUT_DEFAULT_STATE_IDX,     INPUT_DEFAULT_STATE_REG,        0U}, 
  {PULL_ENABLE_IDX,             PULL_ENABLE_REG,                0x09U},
  {PULL_UP_DOWN_IDX,            PULL_UP_DOWN_REG,               0xFFU}, 
  {INPUT_STATE_IDX,             INPUT_STATE_REG,                0U}, 
  {INPUT_INTPR_MASK_IDX,        INPUT_INTPR_MASK_REG,           0xF7U},
  {INTPR_STATUS_IDX,            INTPR_STATUS_REG,               0U}};

  /* U40.0=IN1=REMOTE; .1=MUTE in HW <=V3.2 -  NC in >=V3.3 - PEN_ALM for HW v4.0 (GD32F4xx); .2=IN3=MIRROR; .3=IN4=RCBO; .4=NC in HW <=V3.2, in HW >=V3.3 MIRR_EN=1(def); .5=nFAULT; .6=IN7=VENT   U40.7=LID   */
  /* NOTE FOR GD32F4xx: for PEN_ALM input, the interrupt must be activated. */
static const ioExpMap_st ioExp2Map = {
  {DEVICEID_CTRL_IDX,           DEVICEID_CTRL_REG,              0U},
#ifdef GD32F4xx  
  {IO_DIR_IDX,                  IO_DIR_REG,                     0x10U},                        
#else
  {IO_DIR_IDX,                  IO_DIR_REG,                     0x12U},                        
#endif  
  {OUTPUT_STATE_IDX,            OUTPUT_STATE_REG,               0x10U},
  {OUTPUT_STATE_HIZ_IDX,        OUTPUT_STATE_HIZ_REG,           0U}, 
  {INPUT_DEFAULT_STATE_IDX,     INPUT_DEFAULT_STATE_REG,        0U},
#ifdef GD32F4xx
  {PULL_ENABLE_IDX,             PULL_ENABLE_REG,                0x46U},
  {PULL_UP_DOWN_IDX,            PULL_UP_DOWN_REG,               0x46U},      
#else  
  {PULL_ENABLE_IDX,             PULL_ENABLE_REG,                0x40U},
  {PULL_UP_DOWN_IDX,            PULL_UP_DOWN_REG,               0x40U},      
#endif  
  {INPUT_STATE_IDX,             INPUT_STATE_REG,                0U},                 
#ifdef GD32F4xx  
  {INPUT_INTPR_MASK_IDX,        INPUT_INTPR_MASK_REG,           0xFDU},
#else  
  {INPUT_INTPR_MASK_IDX,        INPUT_INTPR_MASK_REG,           0xFFU},
#endif  
  {INTPR_STATUS_IDX,            INTPR_STATUS_REG,               0U}};


/*
*********************************** SCAME ************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 
/*  Input manager queue  declaration */
xQueueHandle ioMngQueue = NULL;

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
static uint32_t ioExpManager                (ioMngMsg_st* pMsg);
static void     ioExpUpdating               (void);
#ifndef HW_MP28947
static void     EXTI15_10_IRQHandler_Config (void);
static void     EXTI15_10_IRQHandler_DeInit (void);
static void     resetINTxIsr                (void);
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
* @brief       I2C_ClearBusyFlagErratum
*
*              Recovery function used as workaround to fix an issue of I2C
*              documented in the errata sheet as:
*
*              The I2C analog filters embedded in the I2C I/Os may be tied 
*              to low level, whereas SCL and SDA lines are kept at high level. 
*              This can occur after an MCU power-on reset, or during ESD stress. 
*              Consequently, the I2C BUSY flag is set, and the I2C cannot enter master mode 
*              (START condition cannot be sent). The I2C BUSY flag cannot be cleared by the SWRST control bit, 
*              nor by a peripheral or a system reset. BUSY bit is cleared under reset, but it is set high again 
*              as soon as the reset is released, because the analog filter output is still at low level. 
*              This issue occurs randomly.               
*              WORKAROUND:
*              The SCL and SDA analog filter output is updated after a transition occurs on the SCL and SDA line respectively. 
*              The SCL and SDA transition can be forced by software configuring the I2C I/Os in output mode. 
*              Then, once the analog filters are unlocked and output the SCL and SDA lines level, the BUSY flag can be reset with a software reset, 
*              and the I2C can enter master mode.
*
*
* @param [in]  hi2c: I2C to recover
*
* @retval      none 
*  
*************************************************************************************************/

void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef* hi2c)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint32_t SDA_pin, SCL_pin, I2C_AlternateFunc;
    GPIO_TypeDef *SDA_port, *SCL_port;
    uint32_t tickstart;
  
    if (hi2c == &hi2c3)  /* I2C used for IO exp and EEPROM (I2C3) */
    {
        SDA_pin = I2C3_SDA_Pin;
        SCL_pin = I2C3_SCL_Pin;
        SDA_port = I2C3_SDA_GPIO_Port;
        SCL_port = I2C3_SCL_GPIO_Port;
        I2C_AlternateFunc = GPIO_AF4_I2C3;
    }
    else if (hi2c == &HI2CSMB0)  /* I2C used for RFID (I2C1)*/
    {      
        SDA_pin = SMB0_SDA_Pin;
        SCL_pin = SMB0_SCL_Pin;
        SDA_port = SMB0_SDA_GPIO_Port;
        SCL_port = SMB0_SCL_GPIO_Port;
        I2C_AlternateFunc = GPIO_AF4_I2C1;
    }
    else
      return;       /* error passing the I2C */
      
    
    // 1. Clear PE bit.
    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_PE);

    //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_I2C_DeInit(hi2c);

    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;

    GPIO_InitStructure.Pin = SCL_pin;
    HAL_GPIO_Init(SCL_port, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = SDA_pin;
    HAL_GPIO_Init(SDA_port, &GPIO_InitStructure);

    /* Refresh IWDG timer */
    HAL_IWDG_Refresh(&hiwdg);
    /* Init tickstart for timeout managment*/
    tickstart = HAL_GetTick();      

    // 3. Check SCL and SDA High level in GPIOx_IDR.
    HAL_GPIO_WritePin(SDA_port, SDA_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SCL_port, SCL_pin, GPIO_PIN_SET);
    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(SCL_port, SCL_pin))
    {      
      if((HAL_GetTick() - tickstart ) > I2C_ERRATUM_TIMEOUT_VALUE)
        break;
      asm("nop");
    }

    /* Refresh IWDG timer */
    HAL_IWDG_Refresh(&hiwdg);
    /* Init tickstart for timeout managment*/
    tickstart = HAL_GetTick();      

    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(SDA_port, SDA_pin))
    {
      if((HAL_GetTick() - tickstart ) > I2C_ERRATUM_TIMEOUT_VALUE)
        break;
      asm("nop");
    }
    
    // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(SDA_port, SDA_pin, GPIO_PIN_RESET);

    // 5. Check SDA Low level in GPIOx_IDR.
    /* Refresh IWDG timer */
    HAL_IWDG_Refresh(&hiwdg);
    /* Init tickstart for timeout managment*/
    tickstart = HAL_GetTick();      

    while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(SDA_port, SDA_pin))
    {
      if((HAL_GetTick() - tickstart ) > I2C_ERRATUM_TIMEOUT_VALUE)
        break;
      asm("nop");
    }
    
    // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(SCL_port, SCL_pin, GPIO_PIN_RESET);

    // 7. Check SCL Low level in GPIOx_IDR.
    /* Refresh IWDG timer */
    HAL_IWDG_Refresh(&hiwdg);
    /* Init tickstart for timeout managment*/
    tickstart = HAL_GetTick();      

    while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(SCL_port, SCL_pin))
    {
      if((HAL_GetTick() - tickstart ) > I2C_ERRATUM_TIMEOUT_VALUE)
        break;
      asm("nop");
    }
    // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(SCL_port, SCL_pin, GPIO_PIN_SET);

    // 9. Check SCL High level in GPIOx_IDR.
    /* Refresh IWDG timer */
    HAL_IWDG_Refresh(&hiwdg);
    /* Init tickstart for timeout managment*/
    tickstart = HAL_GetTick();      

    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(SCL_port, SCL_pin))
    {
      if((HAL_GetTick() - tickstart ) > I2C_ERRATUM_TIMEOUT_VALUE)
        break;
      asm("nop");
    }
    // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(SDA_port, SDA_pin, GPIO_PIN_SET);

    // 11. Check SDA High level in GPIOx_IDR.
    /* Refresh IWDG timer */
    HAL_IWDG_Refresh(&hiwdg);
    /* Init tickstart for timeout managment*/
    tickstart = HAL_GetTick();      

    while (GPIO_PIN_SET != HAL_GPIO_ReadPin(SDA_port, SDA_pin))
    {
      if((HAL_GetTick() - tickstart ) > I2C_ERRATUM_TIMEOUT_VALUE)
        break;
      asm("nop");
    }
    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Alternate = I2C_AlternateFunc;

    GPIO_InitStructure.Pin = SCL_pin;
    HAL_GPIO_Init(SCL_port, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = SDA_pin;
    HAL_GPIO_Init(SDA_port, &GPIO_InitStructure);

#ifdef GD32F4xx    
    // 13. Set SWRST bit in I2Cx_CR1 register.
    SET_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    /* 14. Clear SWRST bit in I2Cx_CR1 register. */
    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");
#else
    /* The reset of the peripheral in STM32F7xx is done disabling the peripheral */
    
    /* 14. Disable the I2C peripheral by resetting the PE bit in I2Cx_CR1 register */ 
    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_PE);
    asm("nop");    
#endif    
    /* 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register */
    SET_BIT(hi2c->Instance->CR1, I2C_CR1_PE);
    asm("nop");

    // Call initialization function.
    HAL_I2C_Init(hi2c);
}

/**
*
* @brief       Gestione input digitali
*
* @param [in]  (void const *: not used  
*  
* @retval      none 
*  
****************************************************************/
void ioMngTask (void * pvParameters)
{
  uint32_t       timeTick;

  /*-------- Creates an empty mailbox for uart SITOS messages --------------------------*/
  ioMngQueue = xQueueCreate(IOEXP_MAX_MESSAGE_NUM, sizeof(ioMngMsg_st));
  configASSERT(ioMngQueue != NULL);

  /* init from default data structure */
  memcpy((void*)&ioMng.ioExpMap, (void*)&ioExpMap, sizeof(ioMng.ioExpMap));
  memcpy((void*)&ioMng.ioExp2Map, (void*)&ioExp2Map, sizeof(ioMng.ioExp2Map));


  /* tick di campionamento tipico 10ms */
  timeTick = portMAX_DELAY;

  setPenFilteringStatus(DISABLED);
  ioMng.penTickFirst = ioMng.penTickLast = (uint32_t)0;
  ioMng.penAlarmStatus = PEN_ALARM_ENABLE_MASK; /* enable PEN alarm detection and init it to false */
  ioMng.penAlarmInpVal =  ioMng.tickBoard = (uint16_t)0; 
  ioMng.pwrBoardDetected = PWR_BOARD_UNDEF;


  /* task could be call printf function so it starts only when console task is initializated see: startConsole()*/

  for (;;)
  {
    /* Wait for some event from SW to be transmitted on telnet */
    if (xQueueReceive(ioMngQueue, (void *)&ioMngMsg, timeTick) == pdPASS)
    {
      timeTick = ioExpManager((ioMngMsg_st *)&ioMngMsg);
    }
    else
    {
      ioMngMsg.taskEv = IO_EVENT_IO_POLLING;
      /* timeout:new polling over all inputs  */
      /* timeTick = portMAX_DELAY; */
      timeTick = ioExpManager((ioMngMsg_st *)&ioMngMsg);
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
static uint32_t ioExpManager (ioMngMsg_st* pMsg)
{
  
  uint32_t        localTimeTick;
  localTimeTick = SETUP_TIME_TICK;

#ifndef HW_MP28947  
  
  uint16_t        deviceAddr;
  uint32_t        deltaT;
  uint8_t         idError = 0, idErrorPark = 0, numRetry;
    
  energy_meter_e  value;

  numRetry = (uint8_t)0;

  /* start processo */
  if ((pMsg->taskEv == IO_EVENT_IOEXP_CHECK) || (pMsg->taskEv == IO_EVENT_START_IOEXP_POLLING))
  {

    do
    {
      /* read new IOEXP device id  */
      idError = ReadFromIoExp(DEVICE_IOEXP_0, ioMng.ioExp2Map.deviceIdCtrl.addr_reg, (uint8_t*)&ioMng.ioExp2Map.deviceIdCtrl.val_reg, (uint16_t)1);
      if (idError == 0U)
      {
        /* programming ioExp pin direction */
        idError = WriteToIoExp(DEVICE_IOEXP_0, ioExp2Map.direction.addr_reg, ioExp2Map.direction.val_reg);
        /* programming ioExp pull-up for input pin  */
        idError |= WriteToIoExp(DEVICE_IOEXP_0, ioExp2Map.pullEnable.addr_reg, ioExp2Map.pullEnable.val_reg);
        idError |= WriteToIoExp(DEVICE_IOEXP_0, ioExp2Map.pullUpPullDown.addr_reg, ioExp2Map.pullUpPullDown.val_reg);
        /* programming ioExp interrupt register   */
        idError |= WriteToIoExp(DEVICE_IOEXP_0, ioExp2Map.intrMask.addr_reg, ioExp2Map.intrMask.val_reg);
        /* reading input pin   */
        idError |= ReadFromIoExp(DEVICE_IOEXP_0, ioMng.ioExp2Map.inStatus.addr_reg, (uint8_t*)&ioMng.ioExp2Map.inStatus.val_reg, (uint16_t)1);
        /* set output  pin   */
        idError |= WriteToIoExp(DEVICE_IOEXP_0, ioExp2Map.outStateHiZ.addr_reg, ioExp2Map.outStateHiZ.val_reg);
        idError |= WriteToIoExp(DEVICE_IOEXP_0, ioExp2Map.outState.addr_reg, ioExp2Map.outState.val_reg);

        /* reading interrupt Status    */
        idError |= ReadFromIoExp(DEVICE_IOEXP_0, ioMng.ioExp2Map.inStatus.addr_reg, (uint8_t*)&ioMng.ioExp2Map.inStatus.val_reg, (uint16_t)1);
        /* programming interrupt mask register   */
        idError |= WriteToIoExp(DEVICE_IOEXP_0, ioExp2Map.intrMask.addr_reg, ioExp2Map.intrMask.val_reg);
        /* reading interrupt Status    */
        idError = ReadFromIoExp(DEVICE_IOEXP_0, ioMng.ioExp2Map.intrStatus.addr_reg, (uint8_t*)&ioMng.ioExp2Map.intrStatus.val_reg, (uint16_t)1);
        /* clear ioExp interrupt status register   */
        idError |= WriteToIoExp(DEVICE_IOEXP_0, ioExp2Map.intrStatus.addr_reg, ioExp2Map.intrStatus.val_reg);
      }
      if ((idError == 0U) && (idErrorPark == 0U))
      {
        tPrintf("IoExp Addr = %02X and %02X working!!\n\r", DEVICE_IOEXP_0, DEVICE_IOEXP_1);
        EVLOG_Message(EV_INFO, "IoExp Addr = %02X and %02X working!!\n\r", DEVICE_IOEXP_0, DEVICE_IOEXP_1);
        ioExpUpdating();
        if (pMsg->taskEv == IO_EVENT_START_IOEXP_POLLING)
        {
          startCompletePolling();
        }
      }
      else
      {
        if (idError != 0U)
        {
          tPrintf("Device Id = %02X Err = %d, Device Id = %02X Err = %d\n\r", DEVICE_IOEXP_1, idErrorPark, DEVICE_IOEXP_0, idError);
          EVLOG_Message(EV_ERROR, "Device Id = %02X Err = %d, Device Id = %02X Err = %d\n\r", DEVICE_IOEXP_1, idErrorPark, DEVICE_IOEXP_0, idError);
        }
        if (numRetry < 3)
        {
          numRetry++;
          /* Reset I2C peripheral in order to exit from a stuck condition */
          I2C_ClearBusyFlagErratum(&hi2c3); 
        }
        else
        {
          setFlagForNvic();
          NVIC_SystemReset();
        }
      }
    } 
    while ((idError != 0U) || (idErrorPark != 0U));
    
  } // end if on pMsg->taskEv == INP_EVENT_POLLING_ON_TO  

  if (pMsg->taskEv == IO_EVENT_IO_POLLING)
  {
    if ((ioMng.penAlarmStatus & PEN_ALARM_MSG_OFF_MASK) != (uint16_t)0)
    {
      ioMng.penAlarmStatus &= (~PEN_ALARM_MSG_OFF_MASK);
      ioMng.penTickFirst = ioMng.penTickLast = (uint32_t)0;
      ioMng.penAlarmInpVal = (uint16_t)0; 
    }
    else
    {
      if ((ioMng.penAlarmStatus & PEN_ALARM_ACTIVE_MASK) != (uint16_t)0)
      {
        ioMng.penAlarmStatus |= PEN_ALARM_MSG_OFF_MASK; /* next polling will set OFF the PEN alarm if it is not present */
      }
    }
    ioExpUpdating();
    //send_to_contact((uint8_t)CONTACT_IO_UPD); 

  }

  if (pMsg->taskEv == IO_EVENT_IO_WRITING)
  {
    /* update out state */
    switch (pMsg->outRegId)
    {
      case REM_ACT_ON:
        if (pMsg->val == (uint8_t)0)
        {
          ioMng.ioExpMap.outState.val_reg &= (~R_ACT_ON);
        }
        else
        {
          ioMng.ioExpMap.outState.val_reg |= R_ACT_ON;
        }
        deviceAddr = DEVICE_IOEXP_1;
        break;

      case REM_ACT_OFF:
        if (pMsg->val == (uint8_t)0)
        {
          ioMng.ioExpMap.outState.val_reg &= (~R_ACT_OFF);
        }
        else
        {
          ioMng.ioExpMap.outState.val_reg |= R_ACT_OFF;
        }
        deviceAddr = DEVICE_IOEXP_1;
        break;

      case V_MUTE:
        if (pMsg->val != (uint8_t)0)
        {
          ioMng.ioExp2Map.outState.val_reg &= (~VMUTE_OFF);
        }
        else
        {
          ioMng.ioExp2Map.outState.val_reg |= VMUTE_OFF;
        }
        deviceAddr = DEVICE_IOEXP_0;
        break;

      case MIRROR_ENA:
        if (pMsg->val == (uint8_t)FALSE)
        {
          ioMng.ioExp2Map.outState.val_reg &= (~MIRROR_EN_MASK);
        }
        else
        {
          ioMng.ioExp2Map.outState.val_reg |= MIRROR_EN_MASK;
        }
        deviceAddr = DEVICE_IOEXP_0;
        break;
        
      default:
        break;

    }
    
    if (deviceAddr == DEVICE_IOEXP_0)
    {
      /* set output  pin new IOEXP U40  */
      idError = WriteToIoExp(DEVICE_IOEXP_0, ioMng.ioExp2Map.outState.addr_reg, ioMng.ioExp2Map.outState.val_reg);
    }
    
  }
  if (pMsg->taskEv == IO_EVENT_PEN_FILTERING)
  {
    setPenFilteringStatus(ENABLED);
    if ((ioMng.penAlarmStatus & PEN_ALARM_ENABLE_MASK) != 0)
    {
      /* Configure EXTI15_10 (U39.2 = PEN_ALM connected to PE.12 pin) in interrupt mode */
      EXTI15_10_IRQHandler_Config();
    }
    
    /* reading interrupt Status    */
    //idError = ReadFromIoExp(DEVICE_IOEXP_0, ioMng.ioExp2Map.intrStatus.addr_reg, (uint8_t*)&ioMng.ioExp2Map.intrStatus.val_reg, (uint16_t)1);
    /* clear ioExp interrupt status register   */
    idError |= WriteToIoExp(DEVICE_IOEXP_0, ioExp2Map.intrStatus.addr_reg, ioExp2Map.intrStatus.val_reg);
    /* read input pin status register   */
    //idError |= ReadFromIoExp(DEVICE_IOEXP_0, ioMng.ioExp2Map.inStatus.addr_reg, (uint8_t*)&ioMng.ioExp2Map.inStatus.val_reg, (uint16_t)1);

  }

  if (pMsg->taskEv == IO_EVENT_PEN_PIN_EDGE)
  {
    /* reading interrupt Status    */
    idError = ReadFromIoExp(DEVICE_IOEXP_0, ioMng.ioExp2Map.intrStatus.addr_reg, (uint8_t*)&ioMng.ioExp2Map.intrStatus.val_reg, (uint16_t)1);
    /* clear ioExp interrupt status register   */
    idError |= WriteToIoExp(DEVICE_IOEXP_0, ioExp2Map.intrStatus.addr_reg, ioExp2Map.intrStatus.val_reg);
    /* read input pin status register   */
    idError |= ReadFromIoExp(DEVICE_IOEXP_0, ioMng.ioExp2Map.inStatus.addr_reg, (uint8_t*)&ioMng.ioExp2Map.inStatus.val_reg, (uint16_t)1);
    
    if (idError == 0U)
    {
        /* in HW 4.xx, the PENALM input is on IOEXP0 (U40) and not on IOEXP1 as in HW 3.xx */
      if ((ioMng.ioExp2Map.intrStatus.val_reg & GPIO_PEN_MASK) == GPIO_PEN_MASK)        
      {
        /* a rising edge on PEN_ALM has been detected In the IoExp the default state is 0. So INTx pin goes low when  IoExp PEN input goes High */
        /* on micro the interrupt must be active on INTx (PE12) falling edge                                                                    */
        if (ioMng.penTickFirst == (uint32_t)0)
        {
          ioMng.penTickFirst = HAL_GetTick();
        }  
        else
        {
          ioMng.penTickLast = HAL_GetTick();
          deltaT = ioMng.penTickLast - ioMng.penTickFirst;
          if ((deltaT >= PEN_PERIOD_DIS_MIN) && (deltaT <= PEN_PERIOD_DIS_MAX)) 
          {
            /* in this case PEN = VAC230_TRG. This meas that PEN hardware isn't present, so this firmware can be disabled */
            EXTI15_10_IRQHandler_DeInit();
            ioMng.penTickFirst = ioMng.penTickLast = (uint32_t)0;
            ioMng.penAlarmInpVal = (uint16_t)0; 
            ioMng.pwrBoardDetected = PWR_BOARD_MON;
            tPrintf("Scheda MON detected\n\r");
            EVLOG_Message(EV_INFO, "Scheda MON detected");
            setPowerOutage(POWER_MON);
          }
          else
          {
            if ((deltaT > PEN_PERIOD_MIN) && (deltaT < PEN_PERIOD_MAX)) 
            {
              /* PEN alarm detected Typically, period is 130msec */
              ioMng.penAlarmStatus |= PEN_ALARM_ACTIVE_MASK;
              ioMng.penAlarmStatus &= (~PEN_ALARM_MSG_OFF_MASK);        /** PEN alarm active, so no msg OFF must be sent */
              ioMng.penAlarmInpVal |= ((uint16_t)0x0001 << PEN_ALARM);  /** set PEN alarm input bit                       */
            }
            ioMng.penTickFirst = ioMng.penTickLast;
          }
        }
      }
    }
    ioExpUpdating();
  }
  else
  {
        
    if (ioMng.pwrBoardDetected == PWR_BOARD_UNDEF)
    {
      ioMng.tickBoard++;
      if (ioMng.tickBoard >= TIMEOUT_CHECK_BOARD)
      {
        /* read input pin status register   */
        idError |= ReadFromIoExp(DEVICE_IOEXP_0, ioMng.ioExp2Map.inStatus.addr_reg, (uint8_t*)&ioMng.ioExp2Map.inStatus.val_reg, (uint16_t)1);        
        if ((ioMng.ioExp2Map.inStatus.val_reg & GPIO_PEN_MASK) == (uint8_t)GPIO_PEN_MASK)
        {
          /*  if this pin is fixet to 1 the power board is BACKUP */
          ioMng.pwrBoardDetected = PWR_BOARD_BACKUP;
          value = getStationEmTypeInt();
          if ((value == MONO_SCAME) || (value == TRI_SCAME))
          {
            ioMng.pwrBoardDetected = PWR_BOARD_UNKNOW;
            tPrintf("Monitor by EM Scame\n\r");
            EVLOG_Message(EV_INFO, "Monitor by EM Scame");
            setPowerOutage(POWER_NONE);
          }
          else
          {
            if (getCounterZC() > 2 )
            {
              tPrintf("Scheda BACKUP detected\n\r");
              EVLOG_Message(EV_INFO, "Scheda BACKUP detected");
              setPowerOutage(POWER_BACK);
            }
            else
            {
              tPrintf("No V230 monitor board detected\n\r");
              EVLOG_Message(EV_INFO, "No V230 monitor board detected");
              setPowerOutage(POWER_NONE);
            }
          }
        }
        else
        {
          /*  if this pin is fixet to 0 the power board is PEN */
          ioMng.pwrBoardDetected = PWR_BOARD_PEN;
          tPrintf("Scheda PEN  / SINAPSI detected\n\r");
          EVLOG_Message(EV_INFO, "Scheda PEN  / SINAPSI detected");
          setPowerOutage(POWER_PEN);
        }
        /*  SINAPSI-RSE */
        if ((getSinapsiEepromEn() == ENABLED) || (getSinapsiNewEnable() == TRUE))   
        {
          setPMreadyForIom2G();
        }
      }
      resetINTxIsr();
    }
    else
    {
      if (ioMng.pwrBoardDetected == PWR_BOARD_PEN)
      {
        resetINTxIsr();
      }
    }
        
  }

#endif
  
  return (localTimeTick);
}

/**
*
* @brief        start the complete polling message  
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void startCompletePolling(void)
{
  inpMngMsg_st	msgInpSend;

  msgInpSend.taskEv = (inpEvents_e)INP_EVENT_START_POLLING;
  configASSERT(xQueueSendToBack(getInptMngQueueHandle(), (void *)&msgInpSend, portMAX_DELAY) == pdPASS);
}


/**
*
* @brief        stop the complete polling message  
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void stopCompletePolling(void)
{
  inpMngMsg_st	msgInpSend;

  msgInpSend.taskEv = (inpEvents_e)INP_EVENT_STOP_FILTERING;
  configASSERT(xQueueSendToBack(getInptMngQueueHandle(), (void *)&msgInpSend, portMAX_DELAY) == pdPASS);
}


/**
*
* @brief        ioexp updating 
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
static void ioExpUpdating (void)
{
  
  uint16_t      ioExpStatus;
  uint8_t       idError_0 = 0, idError_1 = 0, numRetry;

  numRetry = (uint8_t)0;
  
  do
  {
    ioExpStatus  = (uint16_t)0;
   /* reading input pin   */
#ifndef GD32F4xx    
    idError_0 = ReadFromIoExp(DEVICE_IOEXP_1, ioMng.ioExpMap.inStatus.addr_reg, (uint8_t*)&ioMng.ioExpMap.inStatus.val_reg, (uint16_t)1);
#endif    
    if (idError_0 != 0)
    {
      tPrintf("Device Id = %02X Error reading", DEVICE_IOEXP_1);
      EVLOG_Message (EV_ERROR, "Device Id = %02X Error reading", DEVICE_IOEXP_1);
    }
    else
    {
      if ((ioMng.ioExpMap.inStatus.val_reg & R_ACT_STATUS_MASK) != 0) ioExpStatus |= ((uint16_t)0x0001 << R_ACT_STATUS_EXP1);
      ioExpStatus |= ioMng.penAlarmInpVal; 
    }          
    idError_1 = ReadFromIoExp(DEVICE_IOEXP_0, ioMng.ioExp2Map.inStatus.addr_reg, (uint8_t*)&ioMng.ioExp2Map.inStatus.val_reg, (uint16_t)1);
    if (idError_1 != 0)
    {
      tPrintf("New Device Id = %02X Error reading\n\r", DEVICE_IOEXP_0);
      EVLOG_Message (EV_ERROR, "New Device Id = %02X Error reading", DEVICE_IOEXP_0);
      /* Reset I2C peripheral in order to exit from a stuck condition */
      I2C_ClearBusyFlagErratum(&hi2c3);
    }
    else
    {
       
      if ((ioMng.ioExp2Map.inStatus.val_reg & IN1_MASK) != 0) ioExpStatus |= ((uint16_t)0x0001 << IN1_REMOTE_EXP0);
      if ((ioMng.ioExp2Map.inStatus.val_reg & IN3_MASK) != 0) ioExpStatus |= ((uint16_t)0x0001 << IN3_MIRROR_EXP0);
      if ((ioMng.ioExp2Map.inStatus.val_reg & IN4_MASK) != 0) ioExpStatus |= ((uint16_t)0x0001 << IN4_RCBO_EXP0);
      if ((ioMng.ioExp2Map.inStatus.val_reg & IN7_MASK) != 0) ioExpStatus |= ((uint16_t)0x0001 << IN7_VENT_EXP0);
      if ((ioMng.ioExp2Map.inStatus.val_reg & IN8_MASK) != 0) ioExpStatus |= ((uint16_t)0x0001 << IN8_LID_EXP0);
      ioMng.ioExpInpStatus = ioExpStatus;
    }
    if ((idError_0 != (uint8_t)0) || (idError_1 != (uint8_t)0))
    {
      /* an error reading ioExp on I2C3 occured --> we try to re-init I2C3 STM32 Module */
      if (numRetry < 3)
      {
        numRetry++;
        /* try to re-init I2C STM32 module */
        reinitI2CforEprom();  
      }
      else
      {
        setFlagForNvic();
        NVIC_SystemReset();
      }
    }
  } while ((idError_0 != (uint8_t)0) || (idError_1 != (uint8_t)0));
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
xQueueHandle getIoMngQueueHandle(void)
{
   return(ioMngQueue);
}

/**
*
* @brief       get an input pin status (no filtering, curent 
*              value)
*
* @param [in]  uni_TypeDef:  input id 
*  
* @retval      GPIO_PinState:  GPIO_PIN_RESET / GPIO_PIN_SET
*  
****************************************************************/
GPIO_PinState getIoExpInput(ioIn_TypeDef_e pinId)
{
  uint16_t mask, longIoData;



  if (pinId < NUM_IN_IOEXP)
  {
    longIoData = (uint16_t)ioMng.ioExpMap.inStatus.val_reg | ((uint16_t)ioMng.ioExp2Map.inStatus.val_reg << (uint16_t)8);
    mask = ((uint16_t)1 << (uint16_t)pinId);

    if ((longIoData & mask)  == (uint16_t)0)
    {
      return(GPIO_PIN_RESET);
    }
    else
    {
      return(GPIO_PIN_SET);
    }
  }
  return(GPIO_PIN_RESET);
}

/**
*
* @brief        get the current value for all digital input on IOEXPs      
*
* @param [in]   none
*
* @retval       uint32_t: the Input status 
*
***********************************************************************************************************************/
uint32_t getDigitalIoExpInput(void)
{
  
  return ((uint32_t)ioMng.ioExpInpStatus);
  
}




// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	send_to_inp
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			- IO_EVENT_PEN_PIN_EDGE
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void send_to_inp(uint8_t inp_event)
{
  ioMngMsg_st	msgInpSend;

  msgInpSend.taskEv = (ioEvents_e)(inp_event);
  configASSERT(xQueueSendToBack(getIoMngQueueHandle(), (void *)&msgInpSend, portMAX_DELAY) == pdPASS);
}


/**
*
* @brief        Configures EXTI lines 15 to 10 (U39.2 = PEN_ALM connected to PE.12 pin) in interrupt mode      
*
* @param [in]   none
*
* @retval       none 
*
***********************************************************************************************************************/

#ifndef HW_MP28947

static void EXTI15_10_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOE clock */
  GPIO_INTX_CLK_ENABLE();

  /* Configure PE.12 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = INT_INx_Pin;
  HAL_GPIO_Init(INT_INx_GPIO_Port, &GPIO_InitStructure);

  /* Enable and set EXTI lines 15 to 10 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 15);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
*
* @brief        Configures EXTI lines 15 to 10 (U39.2 = PEN_ALM connected to PE.12 pin) in interrupt mode      
*
* @param [in]   none
*
* @retval       none 
*
***********************************************************************************************************************/
static void EXTI15_10_IRQHandler_DeInit(void)
{
  /* only the interrupt on this pin must be disable because EXTI15_10_IRQHandler must be active for bluethoot interrupt */
  HAL_GPIO_DeInit(INT_INx_GPIO_Port, INT_INx_Pin);
}

/**
*
* @brief        Remove interrupt condition from INTx pin       
*
* @param [in]   none
*
* @retval       none 
*
***********************************************************************************************************************/
static void resetINTxIsr(void)
{
#ifndef GD32F4xx  
    if (HAL_GPIO_ReadPin(INT_INx_GPIO_Port, INT_INx_Pin) == GPIO_PIN_RESET)
    {
      /* reading interrupt Status    */
      (void)ReadFromIoExp(DEVICE_IOEXP_1, ioMng.ioExpMap.intrStatus.addr_reg, (uint8_t*)&ioMng.ioExpMap.intrStatus.val_reg, (uint16_t)1);
      /* clear ioExp interrupt status register   */
      (void)WriteToIoExp(DEVICE_IOEXP_1, ioExpMap.intrStatus.addr_reg, ioExpMap.intrStatus.val_reg);
      /* read input pin status register   */
      (void)ReadFromIoExp(DEVICE_IOEXP_1, ioMng.ioExpMap.inStatus.addr_reg, (uint8_t*)&ioMng.ioExpMap.inStatus.val_reg, (uint16_t)1);
    }
#endif    
}
#endif

/**
*
* @brief        Configures GPIO_0 on U39.12 (IO0) in input pull-up and read it 
*
* @param [in]   none
*
* @retval       GPIO_PinState: GPIO_PIN_RESET / GPIO_PIN_SET 
*
***********************************************************************************************************************/
GPIO_PinState gpio1IoExpRead(void)
{
  GPIO_PinState pinState;
#ifndef GD32F4xx
  uint8_t       regVal, error, numRetry = 0;
#endif  
  pinState  = GPIO_PIN_SET;


#ifndef GD32F4xx
  
  do
  {
    /* programming ioExp pin direction */
    error = WriteToIoExp(DEVICE_IOEXP_1, ioExpMap.direction.addr_reg, ioExpMap.direction.val_reg);
    if (error == 0)
    {
     /* reading input pin   */
      error = ReadFromIoExp(DEVICE_IOEXP_1, ioMng.ioExpMap.inStatus.addr_reg, (uint8_t*)&regVal, (uint16_t)1);
      if ((error == 0) && ((regVal & MASK_GPIO1_IOEXP1) == 0))
      {
        pinState  = GPIO_PIN_RESET;
      }
    }
    if (error != (uint8_t)0)
    {
      /* an error reading ioExp on I2C3 occured --> we try to re-init I2C3 STM32 Module */
      if (numRetry < 3)
      {
        numRetry++;
        /* try to re-init I2C STM32 module */
        reinitI2CforEprom();  
      }
      else
      {
        setFlagForNvic();
        NVIC_SystemReset();
      }
    }
  } while (error != (uint8_t)0);
#else

#ifndef DEBUG_TRACE_PIN   
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pins : GPIO1_uP_Pin PD1  */
  GPIO_InitStruct.Pin = GPIO1_uP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO1_uP_GPIO_Port, &GPIO_InitStruct);
  osDelay(2);

  pinState = HAL_GPIO_ReadPin(GPIO1_uP_GPIO_Port, GPIO1_uP_Pin);
#endif
#endif
  
  return(pinState);
}

/**
*
* @brief        Configures GPIO_1 on U9.82 (IO1 = PD1) as outpu and write on it 
*
* @param [in]   none
*
* @retval       uint8_t: 0 no error else code error 
*
***********************************************************************************************************************/
uint8_t  gpio1IoExpWrite(GPIO_PinState pinVal)
{
  uint8_t       error = 0;
#ifndef GD32F4xx  
  uint8_t       numRetry = 0;
#endif

#ifndef GD32F4xx    
      
  do
  {
    
    /* programming ioExp pin direction */
    error = WriteToIoExp(DEVICE_IOEXP_1, ioExpMap.direction.addr_reg, (ioExpMap.direction.val_reg | MASK_GPIO1_IOEXP1));
    if (error == 0)
    {
      ioMng.ioExpMap.outState.val_reg = (pinVal  == GPIO_PIN_SET) ? (ioMng.ioExpMap.outState.val_reg | MASK_GPIO1_IOEXP1) : 
                                                                    (ioMng.ioExpMap.outState.val_reg & (~MASK_GPIO1_IOEXP1));
      /* set output  pin on GPIO1 U39.0 */
      error = WriteToIoExp(DEVICE_IOEXP_1, ioMng.ioExpMap.outState.addr_reg, ioMng.ioExpMap.outState.val_reg);
    }
    if (error != (uint8_t)0)
    {
      /* an error reading ioExp on I2C3 occured --> we try to re-init I2C3 STM32 Module */
      if (numRetry < 3)
      {
        numRetry++;
        /* try to re-init I2C STM32 module */
        reinitI2CforEprom();  
      }
      else
      {
        setFlagForNvic();   /* try to restart the system....*/
        NVIC_SystemReset();
      }
    }
  } while (error != (uint8_t)0);
#else
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO1_uP  now in output mode    */
  HAL_GPIO_WritePin(GPIO1_uP_GPIO_Port, GPIO1_uP_Pin, pinVal);

  /*Configure GPIO pins : GPIO1_uP_Pin PD1  */
  GPIO_InitStruct.Pin = GPIO1_uP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO1_uP_GPIO_Port, &GPIO_InitStruct);
#endif  
  
  return(error);
}

/**
*
* @brief        Configures GPIO_0 on U9 (IO0 = PA11) in input pull-up and read it 
*
* @param [in]   none
*
* @retval       GPIO_PinState: GPIO_PIN_RESET / GPIO_PIN_SET 
*
***********************************************************************************************************************/
GPIO_PinState gpio0IoExpRead(void)
{
    
  GPIO_PinState pinState;

#ifndef DEBUG_TRACE_PIN  
  
  pinState  = GPIO_PIN_SET;
    
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /*CAN_RX  now in input mode    */

  /*Configure GPIO pins : GPIO0_uP_Pin PA11  */
  GPIO_InitStruct.Pin = GPIO0_uP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO0_uP_GPIO_Port, &GPIO_InitStruct);
  osDelay(2);

  pinState = HAL_GPIO_ReadPin(GPIO0_uP_GPIO_Port, GPIO0_uP_Pin);

#else
  
  osDelay(50);
  
  pinState  = GPIO_PIN_RESET;
  
#endif
  
  return(pinState);
}

/**
*
* @brief        Configures GPIO_0 on U9 (IO0 = PA11) as output and write on it 
*
* @param [in]   none
*
* @retval       uint8_t: 0 no error else code error 
*
***********************************************************************************************************************/
void  gpio1IoWrite(GPIO_PinState pinVal)
{
  /* GPIO1_up now in output mode    */
  HAL_GPIO_WritePin(GPIO1_uP_GPIO_Port, GPIO1_uP_Pin, pinVal);

}

/**
*
* @brief        Configures GPIO_0 on U9 (IO0 = PA11) as output and write on it 
*
* @param [in]   none
*
* @retval       uint8_t: 0 no error else code error 
*
***********************************************************************************************************************/
uint8_t  gpio0IoExpWrite(GPIO_PinState pinVal)
{
  uint8_t       error = 0;

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pins : GPIO0_uP_Pin PA11  */
  HAL_GPIO_WritePin(GPIO0_uP_GPIO_Port, GPIO0_uP_Pin, pinVal);

  /*Configure GPIO pins : GPIO0_uP_Pin PD1  */
  GPIO_InitStruct.Pin = GPIO0_uP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO0_uP_GPIO_Port, &GPIO_InitStruct);
  
  return(error);
}


/**
*
* @brief        Get an input on ioExp 0 (U40) 
*
* @param [in]   none
*
* @retval       GPIO_PinState: GPIO_PIN_RESET / GPIO_PIN_SET 
*
***********************************************************************************************************************/
GPIO_PinState ioExpRead(uint8_t pinMask)
{
  GPIO_PinState pinState;

  pinState  = GPIO_PIN_SET;


  ioExpUpdating();

  if ((ioMng.ioExp2Map.inStatus.val_reg & pinMask) == (uint8_t)0) pinState = GPIO_PIN_RESET;

  return(pinState);
}


/**
*
* @brief        Update the discovered power outage  
*
* @param [in]   none
*
* @retval       none 
*
***********************************************************************************************************************/
void powerOutageUodate(void)
{
  setPowerOutage((uint8_t)ioMng.pwrBoardDetected);
}

/*************** END OF FILE ******************************************************************************************/


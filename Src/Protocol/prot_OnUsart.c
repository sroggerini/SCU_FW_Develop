/**
* @file        prot_OnUsart.c
*
* @brief       protocol managet on USART - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: prot_OnUsart.c 762 2025-05-30 15:11:39Z stefano $
*
*     $Revision: 762 $
*
*     $Author: stefano $
*
*     $Date: 2025-05-30 17:11:39 +0200 (ven, 30 mag 2025) $
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
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#else
#include "stm32h5xx_hal.h"
#endif
#include "cmsis_os.h"
#include "DataLink_dbg.h"
#include "prot_OnUsart.h"
#include "dbg_Task.h"
#include "Em_Task.h"
#include "sbcGsy.h"
#include "scuMdb.h"
#include "wrapper.h"
#include "telnet.h"
#include "httpserver-socket.h"
#include "sbcSem.h"

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Global Const                                   **
**                                                                          **
****************************************************************************** 
*/ 
/* task for debug console */
const osThreadAttr_t dbgGestTask_attributes = {
  .name = "DBG_TASK",
  .priority = (osPriority_t) osPriorityLow, 
  .stack_size = configMINIMAL_STACK_SIZE * 7         
};
const osThreadAttr_t dbgTxTask_attributes = {
  .name = "DBG_DL_TX_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 10        
};
const osThreadAttr_t dbgRxTask_attributes = {
  .name = "DBG_DL_RX_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 10         
  //.stack_size = configMINIMAL_STACK_SIZE * 6
};

const osThreadAttr_t emTxTask_attributes = {
  .name = "EM_DL_TX_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 5
};
const osThreadAttr_t emRxTask_attributes = {
  .name = "EM_DL_RX_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 5
};

/* Definitions for energy meter Task   */
const osThreadAttr_t emMngTask_attributes = {
  .name = "EM_MNG",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 6  
};

/* Definitions for SBC Task   */
const osThreadAttr_t sbcMngTask_attributes = {
  .name = "SBC_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 6   // Fixed ticket SCU-80
};

#ifdef GD32F4xx
/* Definitions for SBC Task used to manage the UART timeout functionality */
const osThreadAttr_t sbcUartRxTask_attributes = {
  .name = "SBC_UART_RX_TASK",
  .priority = (osPriority_t) osPriorityHigh7,
  .stack_size = configMINIMAL_STACK_SIZE * 2    // Original value --> 4 - Maximum call chain --> 2
};
#endif

const osThreadAttr_t scuTxToSbcTask_attributes = {
  .name = "SCU_TX_SBC_TASK",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 2    
};

/* Definitions for SCU Task   */
const osThreadAttr_t scuMngTask_attributes = {
  .name = "SCU_TASK",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 4   
};

#ifdef GD32F4xx
/* Definitions for SCU Task used to manage the UART timeout functionality */
const osThreadAttr_t scuUartRxTimeoutTask_attributes = {
  .name = "SCU_UART_RX_TASK",
  .priority = (osPriority_t) osPriorityHigh7,
  .stack_size = configMINIMAL_STACK_SIZE * 3
};
#endif

/* Definitions for SCU Task   */
const osThreadAttr_t scuDataLinkTask_attributes = {
  .name = "SCU_DL_TASK",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 5     
};

/* Definitions for SCB (SEM) USART Task scuSemGestTask()  */
const osThreadAttr_t scuSemMngTask_attributes = {
  .name = "SCU_SEM_TASK",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 6     
};

/* Definitions for SCB (SEM) modbus upgrade register Task   */
const osThreadAttr_t sbcSemGestTask_attributes = {
  .name = "SBC_SEM_TASK",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 5     
};

/* Definitions for RS485 Modbus protocol (SEM) transmission Task   */
const osThreadAttr_t rs485SemGestTask_attributes = {
  .name = "RS485_SEM_TASK",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 4     // 
};

/* task for debug console */
const osThreadAttr_t consoleTask_attributes = {
  .name = "CONSOLE_TASK",
/* Reduced the task priority since the application was stucking in vTaskSwitchContext at startup (HW_MP28947_GD32F4xx) */  
#ifdef HW_MP28947  
  .priority = (osPriority_t) osPriorityLow ,         
#else
  .priority = (osPriority_t) osPriorityBelowNormal ,         
#endif  
  .stack_size = configMINIMAL_STACK_SIZE * 8         
};


/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local Const                                   **
**                                                                          **
****************************************************************************** 
*/ 

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 
static usartStruct_st usartStruct[NUM_PROT_USART];


/*
*********************************** SCAME ************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 
/* handle for RS232 UART7 debug */
osThreadId_t dbgGestTaskHandle;
osThreadId_t dbgTxTaskHandle;
osThreadId_t dbgRxTaskHandle;
osThreadId_t emMngTaskHandle;
osThreadId_t scuMngTaskHandle;
osThreadId_t scuUartRxTaskHandle;
osThreadId_t scuDataLinkTaskHandle;
osThreadId_t scuGestMngTaskHandle;
osThreadId_t scuSemMngTaskHandle;
osThreadId_t sbcGestTaskHandle;
osThreadId_t sbcUartRxTaskHandle;
osThreadId_t scuTxToSbcHandle;
osThreadId_t sbcMngTaskHandle;
osThreadId_t rs485MngTaskHandle;
osThreadId_t consoleTaskHandle;



RxStruct_t      locBuffRx;

/* handle for RS485 UART2 Energy Meter  */
osThreadId_t emGestTaskHandle;
osThreadId_t emTxTaskHandle;
osThreadId_t emRxTaskHandle;

// SERIALE/RS485 SCU
uint32_t      SerialBaudRate;
unsigned char SerialProtocol;
RxStruct_t    BuffRxSerial;
TxStruct_t    BuffTxSerial;
TxStruct_t    BuffTxTempSerial;

// SERIALE/RS485 Energy Meter
uint32_t      Serial2BaudRate;
unsigned char Serial2Protocol;
RxStruct_t    BuffRxSerial2;
TxStruct_t    BuffTxSerial2;
TxStruct_t    BuffTxTempSerial2;
		
	// SERIALE SBC
uint32_t      Serial3BaudRate;
unsigned char Serial3Protocol;
RxStruct_t    BuffRxSerial3;
TxStruct_t    BuffTxSerial3;
TxStruct_t    BuffTxTempSerial3;

	// SERIALE Debug
uint32_t      Serial4BaudRate;
unsigned char Serial4Protocol;
RxStruct_t    BuffRxSerial4;
TxStruct_t    BuffTxSerial4;
TxStruct_t    BuffTxTempSerial4;

uint16_t      packetNum;

/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/
extern UART_HandleTypeDef huart7;

extern uint8_t            initUartDBG_End, initUart2End, initUart1End;


/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Function                             **
**                                                                          **
******************************************************************************
*/
#ifdef UART_FOR_DEBUG
extern void UART_DBG_DMA_IRQHandler    (void);
#endif
extern HAL_StatusTypeDef  HAL_UART_PROT_Rx_Ena_IT               (UART_HandleTypeDef* huart, uint8_t* pDestDMA, uint16_t maxSize);
extern void               MX_PROT_UART_DBG_Init                 (UART_HandleTypeDef* huart, uint32_t br, uint32_t wl, uint32_t bs, uint32_t par);
extern void               MX_PROT_UART_SBC_Init                 (UART_HandleTypeDef* huart, uint32_t br, uint32_t wl, uint32_t bs, uint32_t par);
extern void               UART_SITOS_IRQHandler                 (void);
extern void               setSysTickStatus                      (uint16_t status);

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
static void         uartProtError_Handler     (char * file, int line);
static void         usartStructInitBuff       (uint8_t intf);

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/

void protOnUartTask (void * pvParameters)
{
  uint32_t        ix, jx;
//  uint32_t        regVal;

  for (jx = 0, ix = PROT_UART_SCU; ix < NUM_PROT_USART; ix++, jx++)
  {
    /* Init structures used to initialize serials */
    usartStruct[jx].intf = (uint16_t)ix;
    usartStructInitBuff((uint8_t)ix);
    /* De-init the current usart interface */
    MX_PROT_UART_DeInit((uint8_t)jx);

    if (ix == PROT_UART_DBG)  /* RS232 debug on USART7 needs a special management */
    {
      /* definition and creation of debugGestTask */
      dbgRxTaskHandle = osThreadNew(dl_DbgRxProcess, NULL, &dbgRxTask_attributes);

      /* definition and creation of sitos Data Link Tx Task */
      dbgTxTaskHandle = osThreadNew(dl_DbgTxProcess, NULL, &dbgTxTask_attributes);

      /* definition and creation of debugGestTask */
      dbgGestTaskHandle = osThreadNew(dbgGestTask, NULL, &dbgGestTask_attributes);

      /* definition and creation of consoleTask */
      consoleTaskHandle = osThreadNew(consoleTask, NULL, &consoleTask_attributes);

    }
    else
    {
      if (ix == PROT_UART_SBC)  /* UART5 for SBC communication  */
      {
               
        /* definition and creation of SBC manager Task */
        sbcGestTaskHandle = osThreadNew(sbcGestTask, NULL, &sbcMngTask_attributes); 
        scuTxToSbcHandle  = osThreadNew(scuTxToSbcTask, NULL, &scuTxToSbcTask_attributes); 

#ifdef GD32F4xx
        /* Create the Task used to manage the Timeout functionality of UART peripheral, not present in GD32F4xx microcontroller */
        sbcUartRxTaskHandle = osThreadNew(sbcUartRxTimeoutTask, NULL, &sbcUartRxTask_attributes);         
#endif
         
        /*-------- Creates an empty mailbox for this usart messages --------------------------*/
        //usartQueues[jx] = xQueueCreate(NUM_BUFF_USART_RX, sizeof(usartStruct_st));
        //configASSERT(usartQueues[jx] != NULL);
        
      }
      else
      {
        if (ix == PROT_UART_EM)  /* UART2 for Energy meter communication  */
        {
          /* definition and creation of Energy meter manager Task */
//#ifdef COMMENTATO_PER_VOICE_TEST
          emMngTaskHandle = osThreadNew(emGestTask, NULL, &emMngTask_attributes); 
//#endif
        }
        else
        {
          if (ix == PROT_UART_SCU)  /* UART1 for SCU bus communication   */
          {
#ifdef SOSPESA
              //regVal = getFlagV230();
              //if ((regVal != BACKUP_WAIT_END_POWER))
              {
                scuDataLinkTaskHandle = osThreadNew(scuDataLinkTask, NULL, &scuDataLinkTask_attributes); 
                scuMngTaskHandle = osThreadNew(scuGestTask, NULL, &scuMngTask_attributes); 
                scuMngTaskHandle = osThreadNew(scuSemGestTask, NULL, &scuSemMngTask_attributes); 
#ifdef GD32F4xx 
                /* Create the Task used to manage the Timeout functionality of UART peripheral, not present in GD32F4xx microcontroller */                
                //  REMOVED SINCE it doesn't work with more traffic --> scuUartRxTaskHandle = osThreadNew(scuUartRxTimeoutTask, NULL, &scuUartRxTimeoutTask_attributes);                 
#endif                
              }
#endif
          }
        }
      }
    }
  } 

  /* no other function needs, so this task can be killed */
  vTaskDelete(NULL);
}


/**
*
* @brief        Set the pointer to Rx-Tx Uart buffer info
*
* @param [in]   uint8_t: interface Id  
*
* @retval       none 
*
***********************************************************************************************************************/
static void  usartStructInitBuff(uint8_t intf)
{
  if (intf == PROT_UART_SCU)
  {
    usartStruct[0].pBuffRxSerial      = (RxStruct_t*)getBuffRxScu();
    usartStruct[0].pBuffTxTempSerial  = (TxStruct_t*)&BuffTxTempSerial;   // eliminare?
    usartStruct[0].pBuffTxSerial      = (TxStruct_t*)&BuffTxSerial;       // eliminare?
    usartStruct[0].uartIstance        = USART1;
    if ((getScuOpMode() < SCU_M_P) && (getStatusDwnl() == FALSE))
    {
      /* GSY or EMUMAX0 enviroment */
      SerialBaudRate                    = UART_SCU_DEFAULT_BR;              // messa a 19200 perchè Algo2 non supporta 57600
    }
    else
    {
      SerialBaudRate                    = UART_SBC_DWLD_BR;                 // messa a 230400 in SEM
    }
    usartStruct[0].baudeRate          = (uint32_t)SerialBaudRate;
    usartStruct[0].stopBit            = UART_STOPBITS_1;
    usartStruct[0].parity             = UART_PARITY_NONE;
    usartStruct[0].wordLen            = UART_WORDLENGTH_8B;
    usartStruct[0].pHuart             = (UART_HandleTypeDef*)&huart1;
  }
  else
  {
    if (intf == PROT_UART_EM)
    {
      usartStruct[1].pBuffRxSerial      = (RxStruct_t*)getBuffRxEm();
      usartStruct[1].pBuffTxTempSerial  = (TxStruct_t*)&BuffTxTempSerial2;  // eliminare?
      usartStruct[1].pBuffTxSerial      = (TxStruct_t*)&BuffTxSerial2;      // eliminare?
      usartStruct[1].uartIstance        = UART_EM;  // USART2 typically
      Serial2BaudRate                   = UART_EM_DEFAULT_BR;
      usartStruct[1].baudeRate          = (uint32_t)Serial2BaudRate;
      usartStruct[1].stopBit            = UART_STOPBITS_1;
      usartStruct[1].parity             = UART_PARITY_NONE;
      usartStruct[1].wordLen            = UART_WORDLENGTH_8B;
      usartStruct[1].pHuart             = (UART_HandleTypeDef*)&UART_EM_HANDLE;
    }
    else
    {
      if (intf == PROT_UART_SBC)
      {
        usartStruct[2].pBuffRxSerial      = (RxStruct_t*)getBuffRxSbc();
        usartStruct[2].pBuffTxTempSerial  = (TxStruct_t*)&BuffTxTempSerial3;  // eliminare?
        usartStruct[2].pBuffTxSerial      = (TxStruct_t*)&BuffTxSerial3;      // eliminare?
        usartStruct[2].uartIstance        = UART5; //UART_SBC;
        Serial3BaudRate                   = (uint32_t)19200; //UART_SBC_DEFAULT_BR;
        usartStruct[2].baudeRate          = (uint32_t)Serial3BaudRate;
        usartStruct[2].stopBit            = UART_STOPBITS_1;
        usartStruct[2].parity             = UART_PARITY_NONE;
        usartStruct[2].wordLen            = UART_WORDLENGTH_8B;
        usartStruct[2].pHuart             = (UART_HandleTypeDef*)&huart5; //UART_SBC_HANDLE;
      }
      else
      {
        if (intf == PROT_UART_DBG)
        {
          usartStruct[3].pBuffRxSerial      = (RxStruct_t*)&BuffRxSerial4;
          usartStruct[3].pBuffTxTempSerial  = (TxStruct_t*)&BuffTxTempSerial4;  // eliminare?
          usartStruct[3].pBuffTxSerial      = (TxStruct_t*)&BuffTxSerial4;      // eliminare?
          usartStruct[3].uartIstance        = UART_DBG;
          Serial4BaudRate                   = UART_DBG_DEFAULT_BR;
          usartStruct[3].baudeRate          = (uint32_t)Serial4BaudRate;
          usartStruct[3].stopBit            = UART_STOPBITS_1;
          usartStruct[3].parity             = UART_PARITY_NONE;
          usartStruct[3].wordLen            = UART_WORDLENGTH_8B;
          usartStruct[3].pHuart             = (UART_HandleTypeDef*)&UART_DBG_HANDLE;
        }
      }
    }
  }

}

/**
*
* @brief       Get the pointer for the Rx DMA transfer 
*
* @param [in]  uint8_t: interface Id  
*  
* @retval      uint8_t*: the Rx DMA pointer linked to the interface
*  
****************************************************************/
uint8_t* getDMAptr (uint8_t intf)
{
  if (intf == PROT_UART_EM)
  {
    return ((uint8_t*)usartStruct[intf].pBuffRxSerial);
  }
  else
  {
    if (intf == PROT_UART_SBC)
    {
      return ((uint8_t*)usartStruct[intf].pBuffRxSerial);
    }
    else
    {
      if (intf == PROT_UART_SCU)
      {
        return ((uint8_t*)usartStruct[intf].pBuffRxSerial);
      }
      else
      {
        return ((uint8_t*)usartStruct[intf].pBuffRxSerial->Buffer);
      }
    }
  }
}

/**
*
* @brief       Get the usart structure pointer for the interface
*
* @param [in]  uint8_t: interface Id  
*  
* @retval      usartStruct_st*: the pointer linked to the interface
*  
****************************************************************/
usartStruct_st* getIntfStructPtr (uint8_t intf)
{
  if (intf < NUM_PROT_USART)
  {
    return((usartStruct_st*)&usartStruct[intf]);
  }
  else
  {
    return((usartStruct_st*)NULL);
  }
}

/**
*
* @brief       Get the usart baude rate for the interface
*
* @param [in]  uint8_t: interface Id  
*  
* @retval      uint32_t: baude rate value
*  
****************************************************************/
uint32_t getIntfBaudeRate (uint8_t intf)
{
  if (intf < NUM_PROT_USART)
  {
    return(usartStruct[intf].baudeRate);
  }
  else
  {
    return((uint32_t)0);
  }
}

/**
*
* @brief       USART PROT  init function
*
* @param [in]  uint8_t: interface Id  
*  
* @retval      none 
*  
****************************************************************/
void MX_PROT_UART_Init(uint8_t intf)
{
  HAL_StatusTypeDef     result;
  uint8_t ix;
  scuOpModes_e          scuMode;

  ix = intf;
  scuMode = getScuOpMode();
  if (intf == PROT_UART_EM )
  {
    do
    {
      huart2.Instance = usartStruct[ix].uartIstance;    // UART_EM = USART2 typically
      huart2.Init.BaudRate = usartStruct[ix].baudeRate;
      huart2.Init.WordLength = usartStruct[ix].wordLen;
      huart2.Init.StopBits = usartStruct[ix].stopBit;
      huart2.Init.Parity = usartStruct[ix].parity;
      huart2.Init.Mode = UART_MODE_TX_RX;
      huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      huart2.Init.OverSampling = UART_OVERSAMPLING_16;
#ifndef GD32F4xx  
      huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_ENABLE; // UART_ONE_BIT_SAMPLE_DISABLE;
      huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
      if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, RS485_EM_ASSERTION_TIME, RS485_EM_DEASSERTION_TIME) != HAL_OK)
      {
        uartProtError_Handler(__FILE__, __LINE__);
      }
      
      result = HAL_UARTEx_ReceiveToIdle_DMA (usartStruct[ix].pHuart, (uint8_t*)usartStruct[ix].pBuffRxSerial, DMA_RX_EM_BUFFER_SIZE);
      
#else
      
      /* Initialize UART2 periph: note that this works in RS485 mode but since STM32F4 and GD32F4 devices doesn't support 
      this feature, we are forced to do it normally */
      if (HAL_UART_Init(&huart2) != HAL_OK)
      {
        uartProtError_Handler(__FILE__, __LINE__);
      }
            
      result = HAL_UART_PROT_Rx_Ena_IT (usartStruct[ix].pHuart, (uint8_t*)usartStruct[ix].pBuffRxSerial, DMA_RX_EM_BUFFER_SIZE);
      
#endif           
      
      if (result != HAL_OK) 
      {
        MX_PROT_UART_DeInit(PROT_UART_EM);
      }

    } while (result != HAL_OK);
  }
  else
  {
    if (intf == PROT_UART_DBG)
    {
      /*----- attivo sempre la ricezione: input from uart7   -------------*/
      MX_PROT_UART_DBG_Init(usartStruct[ix].pHuart, usartStruct[ix].baudeRate, usartStruct[ix].wordLen, usartStruct[ix].stopBit, usartStruct[ix].parity);
      (void)HAL_UART_PROT_Rx_Ena_IT(usartStruct[ix].pHuart, (uint8_t*)usartStruct[ix].pBuffRxSerial->Buffer, UART_DBG_BUFF_LEN);
    }
    else
    {
      if (intf == PROT_UART_SBC)
      {
        if (sbcPresence())
        {
          /*----- attivo sempre la ricezione: input from uart5   -------------*/
          if ((scuMode >= SCU_M_P) && (getStatusDwnl() == (uint8_t)0)) /* SEM enviromoment and no download running */
          //if (getScuOpMode() >= SCU_EMUMAX0_M)
          {
            Serial3BaudRate = UART_SBC_SEM_BR; // for SCU principal or secondary the BR=115200
            /* override the SBC to SCU  default baude rate */
            usartStruct[ix].baudeRate = Serial3BaudRate;  
          }
          MX_PROT_UART_SBC_Init(usartStruct[ix].pHuart, usartStruct[ix].baudeRate, usartStruct[ix].wordLen, usartStruct[ix].stopBit, usartStruct[ix].parity);
          if (getFastBridge() == ENABLED)
          {
            /* used in downlaod phase: the byte coming from SBC must be immediatly put on RS485 */
            do 
            {
              result = HAL_UART_Receive_IT(usartStruct[ix].pHuart, (uint8_t*)usartStruct[ix].pBuffRxSerial, (uint16_t)NUM_BUFF_SBC_MSG_RX);
              bridgeVarInit();
              setSysTickStatus(TRUE);
            }while(result != HAL_OK);
          }
          else
          {
            if ((((isSemMasterFz() == TRUE) || (scuMode == SCU_S_P)) && (isModbusManagerActive() == TRUE)) || (scuMode <= SCU_EMUMAX0_M))
            {
              (void)HAL_UART_PROT_Rx_Ena_IT(usartStruct[ix].pHuart, (uint8_t*)usartStruct[ix].pBuffRxSerial, (uint16_t)NUM_BUFF_SBC_MSG_RX);
            }
          }
        }
      }
      else
      {
        if (intf == PROT_UART_SCU)
        {
          usartStructInitBuff(PROT_UART_SCU);
          huart1.Instance = usartStruct[ix].uartIstance;    // USART1
          huart1.Init.BaudRate = usartStruct[ix].baudeRate;
          if ((scuMode < SCU_M_P) && (getFastBridge() == DISABLED))
          {
            if (getStatusDwnl() == (uint8_t)FALSE) /* if downloading is starting baude rate is set in the function lowLevelBrChange() */
            {
              /* override the SCU default baude rate */
              huart1.Init.BaudRate = usartStruct[ix].baudeRate = UART_SCU_DEFAULT_BR; // for SCU in emulation MAX= USART1 BR=19200
            }
          }
          huart1.Init.WordLength = usartStruct[ix].wordLen;
          huart1.Init.StopBits = usartStruct[ix].stopBit;
          huart1.Init.Parity = usartStruct[ix].parity;
          huart1.Init.Mode = UART_MODE_TX_RX;
          huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
          huart1.Init.OverSampling = UART_OVERSAMPLING_16;
#ifndef GD32F4xx  
          huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE; //UART_ONE_BIT_SAMPLE_ENABLE; //UART_ONE_BIT_SAMPLE_DISABLE;
          huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
          if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, RS485_EM_ASSERTION_TIME, RS485_EM_DEASSERTION_TIME) != HAL_OK)
          {
            uartProtError_Handler(__FILE__, __LINE__);
          }
#else
          /* Init UART used to communicate with SCU*/
          if (HAL_UART_Init(&huart1) != HAL_OK)
          {
            uartProtError_Handler(__FILE__, __LINE__);
          }
#endif         
          /* Enable the reception */
          /* Move the DE pin in order to manage the RS-485 communication: enable the RX part */
          HAL_GPIO_WritePin(UART1_DE_GPIO_Port, UART1_DE_Pin, GPIO_PIN_RESET);   
          (void)HAL_UART_PROT_Rx_Ena_IT(usartStruct[ix].pHuart, (uint8_t*)usartStruct[ix].pBuffRxSerial, NUM_BUFF_SBC_MSG_RX);
        }
        else
        {
          ;
        }
      }
    }
  }

}

/**
*
* @brief       USART PROT  de-init function
*
* @param [in]  uint8_t: interface Id  
*  
* @retval      none 
*  
****************************************************************/
void MX_PROT_UART_DeInit(uint8_t intf)
{
  uint8_t   ix;

  ix = intf;
  if (intf == PROT_UART_SCU)
  {
    huart1.Instance = usartStruct[ix].uartIstance;    //USART1
    huart1.Init.BaudRate = usartStruct[ix].baudeRate; //UART_SCU_DEFAULT_BR;
    huart1.Init.WordLength = usartStruct[ix].wordLen; //UART_WORDLENGTH_9B; // UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = usartStruct[ix].parity; //UART_PARITY_EVEN; //UART_PARITY_NONE; 
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
#ifndef GD32F4xx  
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_ENABLE; //UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
#endif    
    if (HAL_UART_DeInit(&huart1) != HAL_OK)
    {
      uartProtError_Handler(__FILE__, __LINE__);
    }
    initUart1End = 0xA1;
  }
  else 
  {
    if (intf == PROT_UART_EM )
    {
      HAL_UART_Abort(&huart2);

      huart2.Instance = usartStruct[ix].uartIstance;
      huart2.Init.BaudRate = usartStruct[ix].baudeRate; //RS485_BR_DEFAULT;
      huart2.Init.WordLength = usartStruct[ix].wordLen; //UART_WORDLENGTH_9B; // UART_WORDLENGTH_8B;
      huart2.Init.StopBits = UART_STOPBITS_1;
      huart2.Init.Parity = usartStruct[ix].parity; //UART_PARITY_EVEN; //UART_PARITY_NONE; 
      huart2.Init.Mode = UART_MODE_TX_RX;
      huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      huart2.Init.OverSampling = UART_OVERSAMPLING_16;
#ifndef GD32F4xx  
      huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_ENABLE; //UART_ONE_BIT_SAMPLE_DISABLE;
      huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
#endif      
      if (HAL_UART_DeInit(&huart2) != HAL_OK)
      {
        uartProtError_Handler(__FILE__, __LINE__);
      }

      initUart2End = 0xA1;
    }
    else
    {
      if (intf == PROT_UART_DBG)
      {
        UART_DBG_HANDLE.Instance = usartStruct[ix].uartIstance;
        UART_DBG_HANDLE.Init.BaudRate = usartStruct[ix].baudeRate; //RS485_BR_DEFAULT;
        UART_DBG_HANDLE.Init.WordLength = usartStruct[ix].wordLen; //UART_WORDLENGTH_9B; // UART_WORDLENGTH_8B;
        UART_DBG_HANDLE.Init.StopBits = UART_STOPBITS_1;
        UART_DBG_HANDLE.Init.Parity = usartStruct[ix].parity; //UART_PARITY_EVEN; //UART_PARITY_NONE; 
        UART_DBG_HANDLE.Init.Mode = UART_MODE_TX_RX;
        UART_DBG_HANDLE.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        UART_DBG_HANDLE.Init.OverSampling = UART_OVERSAMPLING_16;
#ifndef GD32F4xx  
        UART_DBG_HANDLE.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_ENABLE; //UART_ONE_BIT_SAMPLE_DISABLE;
        UART_DBG_HANDLE.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
#endif        
        if (HAL_UART_DeInit(&UART_DBG_HANDLE) != HAL_OK)
        {
          uartProtError_Handler(__FILE__, __LINE__);
        }
        initUartDBG_End = 0xA1;
      }
      else
      {
        if (intf == PROT_UART_SBC)
        {
          huart5.Instance = usartStruct[ix].uartIstance;    //USART5
          huart5.Init.BaudRate = usartStruct[ix].baudeRate; //UART_SCU_DEFAULT_BR;
          huart5.Init.WordLength = usartStruct[ix].wordLen; //UART_WORDLENGTH_9B; // UART_WORDLENGTH_8B;
          huart5.Init.StopBits = UART_STOPBITS_1;
          huart5.Init.Parity = usartStruct[ix].parity; //UART_PARITY_EVEN; //UART_PARITY_NONE; 
          huart5.Init.Mode = UART_MODE_TX_RX;
          huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
          huart5.Init.OverSampling = UART_OVERSAMPLING_16;
#ifndef GD32F4xx  
          huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_ENABLE; //UART_ONE_BIT_SAMPLE_DISABLE;
          huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
#endif          
          if (HAL_UART_DeInit(&huart5) != HAL_OK)
          {
            uartProtError_Handler(__FILE__, __LINE__);
          }
        }
      }
    }
  }
}

/**
*
* @brief       USART change baude rate 
*
* @param [in]  uint8_t: interface Id  
* @param [in]  uint32_t: new baude rate   
*  
* @retval      none 
*  
****************************************************************/
HAL_StatusTypeDef MX_PROT_UART_ChangeBaudeRate (uint8_t intf, uint32_t br)
{
  if (intf == PROT_UART_SCU)
  {
    SerialBaudRate = huart1.Init.BaudRate = usartStruct[intf].baudeRate = br;
    reInitScuUart();
  }
  else 
  {
    if (intf == PROT_UART_EM )
    {
      huart2.Init.BaudRate = usartStruct[intf].baudeRate = br;
      /* Set the UART Communication parameters */
#ifndef GD32F4xx      
      if (UART_SetConfig(&huart2) == HAL_ERROR)
#else
      if (HAL_UART_Init(&huart2) == HAL_ERROR)  
#endif        
      {
        return HAL_ERROR;
      }
    }
    else
    {
      if (intf == PROT_UART_SBC)
      {
        Serial3BaudRate = huart5.Init.BaudRate = usartStruct[intf].baudeRate = br;
        reInitSbcUart();
      }
    }
  }
  return(HAL_OK);
}

/**
*
* @brief        Reinit UART SBC from errors  
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void reInitSbcUart (void)
{
  
  if ((huart5.Instance->CR1 & UART_STATE_ENABLE) == RESET)
  {
    __HAL_RCC_UART5_FORCE_RESET();
    HAL_Delay (100);
    __HAL_RCC_UART5_RELEASE_RESET();
  }
    
  if (huart5.hdmarx != NULL)
  {
    HAL_DMA_Abort(huart5.hdmarx); 
    HAL_DMA_DeInit(huart5.hdmarx);
  }
  MX_PROT_UART_DeInit(PROT_UART_SBC);

  MX_PROT_UART_Init(PROT_UART_SBC);
}

/**
*
* @brief        Reinit UART SCU (NModbus) from errors  
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void reInitScuUart (void)
{
  if (huart1.hdmarx != NULL)
  {
    HAL_DMA_Abort(huart1.hdmarx); 
    HAL_DMA_DeInit(huart1.hdmarx);
  }
  MX_PROT_UART_DeInit(PROT_UART_SCU);

  MX_PROT_UART_Init(PROT_UART_SCU);
  /* Clear TC flag */
  __HAL_UART_CLEAR_FLAG(&UART_SCU_HANDLE, UART_FLAG_TC);        

}

/**
*
* @brief        Reactivate SCU DMA rx after a reception
*
* @param [in]   None
*
* @retval       None
*
***********************************************************************************************************************/

void UART_SCU_Reactivate_Rx(void)
{
    HAL_UART_PROT_Rx_Ena_IT(usartStruct[PROT_UART_SCU].pHuart, (uint8_t*)usartStruct[PROT_UART_SCU].pBuffRxSerial, NUM_BUFF_SBC_MSG_RX);
}

/**
*
* @brief        disable UART1   
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void disableRS485 (void)
{
  __HAL_UART_DISABLE(&huart1);
}

/**
*
* @brief        Reinit UART SCU (Modbus) from errors  
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void fastInitScuUart (void)
{
  if (huart1.hdmarx != NULL)
  {
    HAL_DMA_Abort(huart1.hdmarx); 
    HAL_DMA_DeInit(huart1.hdmarx);
  }
  MX_PROT_UART_Init(PROT_UART_SCU);
}

/**
*
* @brief        Starts task for uarts Sem Task  
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void startSemSbcUartTask (void)
{
  if (scuDataLinkTaskHandle == NULL) 
  {
    scuDataLinkTaskHandle = osThreadNew(scuDataLinkTask, NULL, &scuDataLinkTask_attributes); 
    scuGestMngTaskHandle = osThreadNew(scuGestTask, NULL, &scuMngTask_attributes); 
    scuSemMngTaskHandle = osThreadNew(scuSemGestTask, NULL, &scuSemMngTask_attributes); 
  }
}

/**
*
* @brief        Delete EM Task  
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void deleteEmTask (void)
{
  if( emMngTaskHandle != NULL )
  {
    vTaskDelete(emMngTaskHandle);
  }
}

/**
*
* @brief        Sem task activation   
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void activeSemTask (void)
{
  if (isSemMode() == TRUE)
  {
    sbcMngTaskHandle = osThreadNew(sbcSemGestTask, NULL, &sbcSemGestTask_attributes);  
    rs485MngTaskHandle = osThreadNew(rs485SemGestTask, NULL, &rs485SemGestTask_attributes);  
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void uartProtError_Handler(char * file, int line)
{
  while(1)
  {
    ;
  }
}



/*************** END OF FILE ******************************************************************************************/


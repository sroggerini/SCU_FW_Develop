/**
* @file        uartDbg.c
*
* @brief       Uart for debug - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: uartDbg.c 694 2025-02-18 14:06:30Z stefano $
*
*     $Revision: 694 $
*
*     $Author: stefano $
*
*     $Date: 2025-02-18 15:06:30 +0100 (mar, 18 feb 2025) $
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
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#else
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_uart_legacy.h"
#endif
#include "cmsis_os.h"
#include "dbg_Task.h"
#include "DataLink_dbg.h"
#include "prot_OnUsart.h"
#include "sbcUart.h"
#include "scuMdb.h"
#include "httpserver-socket.h"
#include "uart_Legacy.h"
#include "uartDbg.h"

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/


#define _GET_SBC_RX_DMA_CNT         UART_SBC_RX_DMA_STREAM->NDTR
#define _IS_SBC_RX_DMA_TC_FLAG      DMA1->LISR & DMA_LISR_TCIF0
#define _CLEAR_SBC_RX_DMA_TC_FLAG   DMA1->LIFCR = DMA_LISR_TCIF0

#define _REINIT_SBC_RX_DMA_CHANNEL(pSource)   UART_SBC_DMA->LIFCR = DMA_LISR_DMEIF0 | DMA_LISR_FEIF0 |   \
                                              DMA_LISR_HTIF0 | DMA_LISR_TCIF0 | DMA_LISR_TEIF0;          \
                                              UART_SBC_RX_DMA_STREAM->M0AR = (uint32_t)pData;            /* Set memory address for DMA again */  \
                                              UART_SBC_RX_DMA_STREAM->NDTR = NUM_BUFF_SBC_MSG_RX;        /* Set number of bytes to receive */    \
                                              UART_SBC_RX_DMA_STREAM->CR |= DMA_SxCR_EN;                 /* Start DMA transfer */               
#define _DISABLE_SBC_RX_DMA         UART_SBC_RX_DMA_STREAM->CR &= ~DMA_SxCR_EN
                                              



/* Macros for GD32F4xx microcontroller */

#ifdef GD32F4xx
/* Get DMA counter */                                              
#define _GET_DBG_RX_DMA_CNT         UART_DBG_RX_DMA_STREAM->NDTR
/* Check if data are available */
#define _IS_DBG_RX_DMA_AVAILABLE    UART_DBG_DMA->LISR & UART_DBG_RX_DMA_TC_FLAG
/* Clear Transfer complete flag */
#define _CLEAR_DBG_RX_AVAILABLE_FLAG   UART_DBG_DMA->LIFCR |= UART_DBG_RX_DMA_TC_FLAG
/* Disable DMA Rx flag */                                              
#define _DISABLE_DBG_RX_DMA         UART_DBG_RX_DMA_STREAM->CR &= ~DMA_SxCR_EN 
/* Reinit DMA channel */                                              
#define _REINIT_DBG_RX_DMA_CHANNEL(pSource)   UART_DBG_DMA->LIFCR = DMA_LISR_DMEIF2 | DMA_LISR_FEIF2 |    \
                                              DMA_LISR_HTIF2 | DMA_LISR_TCIF2 | DMA_LISR_TEIF2;           \
                                              UART_DBG_RX_DMA_STREAM->M0AR = (uint32_t)pSource;                          /* Set memory address for DMA again */  \
                                              UART_DBG_RX_DMA_STREAM->NDTR = 0xFFFF;                                     /* Set number of bytes to receive */    \
                                              UART_DBG_RX_DMA_STREAM->CR |= DMA_SxCR_EN;                                 /* Start DMA transfer */                                            
#else
/* Get DMA counter */                                              
#define _GET_DBG_RX_DMA_CNT             __HAL_DMA_GET_COUNTER(&handle_GPDMA2_Channel2)
/* Check if data are available */
#define _IS_DBG_RX_DMA_AVAILABLE        __HAL_DMA_GET_FLAG(&handle_GPDMA2_Channel2, DMA_FLAG_SUSP)
/* Check IDLE flag */
#define _IS_DBG_RX_DMA_IDLE_FLAG        __HAL_DMA_GET_FLAG(&handle_GPDMA2_Channel2, DMA_FLAG_IDLE)
/* Clear Transfer complete flag */
#define _CLEAR_DBG_RX_AVAILABLE_FLAG    __HAL_DMA_CLEAR_FLAG(&handle_GPDMA2_Channel2, DMA_FLAG_SUSP)   
/* Disable DMA Rx flag */
#define _DISABLE_DBG_RX_DMA             __HAL_DMA_DISABLE(&handle_GPDMA2_Channel2) 
/* Reinit DMA channel */                                              
#define _REINIT_DBG_RX_DMA_CHANNEL(pSource)  __HAL_DMA_CLEAR_FLAG(&handle_GPDMA2_Channel2, (DMA_FLAG_IDLE | DMA_FLAG_TC | DMA_FLAG_HT | DMA_FLAG_DTE |   \
                                                                                          DMA_FLAG_ULE | DMA_FLAG_USE | DMA_FLAG_SUSP | DMA_FLAG_TO));   \
                                             UART_DBG_RX_DMA_STREAM->CDAR = (uint32_t)pSource;                         /* Set memory address for DMA again */ \
                                             __HAL_DMA_SET_COUNTER (&handle_GPDMA2_Channel2, 0xFFFF);                  /* Set number of bytes to receive */   \
                                             __HAL_DMA_ENABLE(&handle_GPDMA2_Channel2);                                /* Start DMA transfer */ \

#endif

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 
#ifdef USE_MY_SECTION
#pragma arm section rwdata   = "RW_SDRAM_FAT"

static uint8_t  rxDbgStatus         __attribute__((section ("RW_SDRAM_FAT")));
#else
static uint8_t  rxDbgStatus;
#endif

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 

#ifdef USE_MY_SECTION
/* UART  handler declaration */
DMA_HandleTypeDef*  DMA2_s7_hdmatx      __attribute__((section ("RW_SDRAM_FAT")));

/* Buffer used for reception and transmission */
WififrameDbg_st     dbgMsgRx_IT         __attribute__((section ("RW_SDRAM_FAT")));
frameDbg_st         dbgMsgTx_IT         __attribute__((section ("RW_SDRAM_FAT")));
frameDbg_st         dbgMsgRxBurst_IT    __attribute__((section ("RW_SDRAM_FAT")));
msgBurstDbgRx_st    msgBurstDbgRx       __attribute__((section ("RW_SDRAM_FAT")));
usartStruct_st      usart1Struct_IT     __attribute__((section ("RW_SDRAM_FAT")));
usartStruct_st      usart2Struct_IT     __attribute__((section ("RW_SDRAM_FAT")));
usartStruct_st      usart6Struct_IT     __attribute__((section ("RW_SDRAM_FAT")));
frameDbg_st         pi1MsgTx_IT         __attribute__((section ("RW_SDRAM_FAT")));
frameDbg_st         pi2MsgTx_IT         __attribute__((section ("RW_SDRAM_FAT")));
#else
/* UART  handler declaration */
DMA_HandleTypeDef*  DMA2_s7_hdmatx      __attribute__((section ("RW_SDRAM_FAT")));

/* Buffer used for reception and transmission */
WififrameDbg_st     dbgMsgRx_IT;
//frameDbg_st         dbgMsgRx_IT;
frameDbg_st         dbgMsgTx_IT;
frameDbg_st         dbgMsgRxBurst_IT;

frameDbg_st         dbgMsgRxConsole_IT;

msgBurstDbgRx_st    msgBurstDbgRx;
usartStruct_st      usart1Struct_IT;
usartStruct_st      usart2Struct_IT;
usartStruct_st      usart5Struct_IT;
frameDbg_st         pi1MsgTx_IT; 
#endif


/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/
extern uint8_t            initUartDBG_End, initUart2End, initUart1End;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef  handle_GPDMA2_Channel2;

/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Function                             **
**                                                                          **
******************************************************************************
*/
extern void startConsole        (uint8_t* pBuff);
extern void init_DMA2_s7_hdmatx (UART_HandleTypeDef* huart);

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
static void   uartDbgError_Handler    (char * file, int line);

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/

/**
*
* @brief       USART DBG  init function
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/
void MX_DBG_UART_Init(void)
{

  UART_DBB_CLK_ENABLE();

  UART_DBG_HANDLE.Instance = UART_DBG;
  UART_DBG_HANDLE.Init.BaudRate = getIntfBaudeRate((uint8_t)PROT_UART_DBG); // UART_DBG_DEFAULT_BR;
  UART_DBG_HANDLE.Init.WordLength = UART_WORDLENGTH_8B; 
  UART_DBG_HANDLE.Init.StopBits = UART_STOPBITS_1;
  UART_DBG_HANDLE.Init.Parity = UART_PARITY_NONE; //UART_PARITY_EVEN; //UART_PARITY_NONE; //UART_PARITY_EVEN;
  UART_DBG_HANDLE.Init.Mode = UART_MODE_TX_RX;
  UART_DBG_HANDLE.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UART_DBG_HANDLE.Init.OverSampling = UART_OVERSAMPLING_16;
#ifndef GD32F4xx  
  UART_DBG_HANDLE.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_ENABLE;
  UART_DBG_HANDLE.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
#endif  
  if (HAL_UART_Init(&UART_DBG_HANDLE) != HAL_OK)
  {
    uartDbgError_Handler(__FILE__, __LINE__);
  }

}

/**
*
* @brief       USART PROT  init function
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/
void MX_PROT_UART_DBG_Init (UART_HandleTypeDef *huart, uint32_t br, uint32_t wl, uint32_t bs, uint32_t par)
{

  huart->Instance = UART_DBG;
  huart->Init.BaudRate = br; //UART_DBG_DEFAULT_BR;
  huart->Init.WordLength = wl; 
  huart->Init.StopBits = bs;
  huart->Init.Parity = par; //UART_PARITY_EVEN; //UART_PARITY_NONE; //UART_PARITY_EVEN;
  huart->Init.Mode = UART_MODE_TX_RX;
  huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart->Init.OverSampling = UART_OVERSAMPLING_16;
#ifndef GD32F4xx    
  huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_ENABLE;
  huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
#endif  
  if (HAL_UART_Init(huart) != HAL_OK)
  {
    uartDbgError_Handler(__FILE__, __LINE__);
  }

}

/**
*
* @brief       USART DEBUG  DeInit function
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/
void MX_DBG_UART_DeInit(void)
{

  UART_DBG_HANDLE.Instance = UART_DBG;
  UART_DBG_HANDLE.Init.BaudRate = UART_DBG_DEFAULT_BR;
  UART_DBG_HANDLE.Init.WordLength = UART_WORDLENGTH_8B; //UART_WORDLENGTH_9B; // UART_WORDLENGTH_8B;
  UART_DBG_HANDLE.Init.StopBits = UART_STOPBITS_1;
  UART_DBG_HANDLE.Init.Parity = UART_PARITY_NONE; // UART_PARITY_EVEN; //UART_PARITY_NONE; //UART_PARITY_EVEN;
  UART_DBG_HANDLE.Init.Mode = UART_MODE_TX_RX;
  UART_DBG_HANDLE.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UART_DBG_HANDLE.Init.OverSampling = UART_OVERSAMPLING_16;
#ifndef GD32F4xx    
  UART_DBG_HANDLE.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_ENABLE;
  UART_DBG_HANDLE.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
#endif  
  if (HAL_UART_DeInit(&UART_DBG_HANDLE) != HAL_OK)
  {
    uartDbgError_Handler(__FILE__, __LINE__);
  }
  initUartDBG_End = 0xA1;
}


/**
*
* @brief       Rx enable interrupt sources
*
* @param [in]  huart: UART handle  
*  
* @retval      none 
*  
****************************************************************/
void HAL_UART_DBG_Rx_Ena_IT (void)
{
  UART_HandleTypeDef *huart;

  huart = &UART_DBG_HANDLE;

  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

  /* Enable the UART Parity Error and Idle line Interrupts */
  SET_BIT(huart->Instance->CR1, USART_CR1_PEIE |  USART_CR1_IDLEIE);

#ifndef GD32F4xx
  /* Enable DMA suspend interrupt: is used for the reception of the chars from USART6 (TeraTerm) */
  __HAL_DMA_ENABLE_IT(huart->hdmarx, DMA_IT_SUSP);  
#endif  

    /* Clear IDLE flag by setting  IDLECF in clear register  */
  __HAL_UART_CLEAR_IDLEFLAG(huart); 

  /* Put UART peripheral in reception process ###########################*/
  /* Any data received will be stored in "MsgRx_IT" buffer data received is MSG_MAX_LEN */
  if (HAL_UART_Receive_DMA(huart, (uint8_t *)dbgMsgRx_IT.infoRxDbg, 0xFFFF) != HAL_OK)
  {
    /* Transfer error in reception process */
    uartDbgError_Handler(__FILE__, __LINE__);
  }
}

/**
*
* @brief       Rx enable interrupt sources
*
* @param [in]  UART_HandleTypeDef*: handler for UART structure  
* @param [in]  uint8_t*: pointer where the DMA stores the incoming data  
* @param [in]  uint16_t: max consecutive the incoming data  
*  
* @retval      none 
*  
****************************************************************/
HAL_StatusTypeDef HAL_UART_PROT_Rx_Ena_IT (UART_HandleTypeDef* huart, uint8_t* pDestDMA, uint16_t maxSize)
{

  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

  /* Enable the UART Parity Error and Idle line Interrupts   */
#ifndef GD32F4xx                                                                             
  if ((huart->Instance == UART_SBC) || (huart->Instance == UART_SCU))
  {                                                                       /** anche SINAPSI su questa UART_SCU trasmette con dei "silenzi" */
                                                                          /** introduce 130us di idle contro un tempo di bit =
                                                                           *  8,7 circa Con 33 bit tollero 33x8,7 = 286us -->
                                                                           *  dovrebbe bastare  03/11/2021   */
    /* a 230400bps durante il download SBC, su RS485, ho misurato buchi da 300us Questo vuol dire 300us x 230400 = 53 bit circa Faccio 100bit */
    //SET_BIT(huart->Instance->CR1, USART_CR1_PEIE |  USART_CR1_IDLEIE);  
    SET_BIT(huart->Instance->CR1, USART_CR1_PEIE |  USART_CR1_RTOIE);  /* Nick modifica perchè SBC a 115200 introduce un idle >> di uno stop bit */
    /* Clear RTOF flag by setting  RTOCF in clear register  */
    __HAL_UART_CLEAR_RTOFLAG(huart);
    if (HAL_UART_EnableReceiverTimeout(huart) == HAL_OK)
    {
      if (getStatusDwnl() == FALSE)  
      {
        /* if downloading isn't running */
        HAL_UART_ReceiverTimeout_Config(huart, (uint32_t)33);             /* Nick dobbiamo tollerare almeno 22 bit di idle facciamo 3 byte= 33 bit  */
      }
      else
      {
        HAL_UART_ReceiverTimeout_Config(huart, (uint32_t)100);              /* Nick dobbiamo tollerare almeno 53 bit a 230400 facciamo 100 bit  */
                                                                            /* Nick dobbiamo tollerare almeno 100 bit a 460800 facciamo 200 bit  */
      }
    }
  }
  else
#endif    
  {
    /* Enable Parity error and IDLE state behaviour */
    SET_BIT(huart->Instance->CR1, USART_CR1_PEIE |  USART_CR1_IDLEIE);  
    /* Clear IDLE flag by setting  IDLECF in clear register  */
    __HAL_UART_CLEAR_IDLEFLAG(huart); 
  }
  


  /* Put UART peripheral in reception process ###########################*/
  /* Any data received will be stored in pDestDMA buffer (data received must be <  maxSize) */
  if (HAL_UART_Receive_DMA(huart, pDestDMA, maxSize) != HAL_OK)
  {
    /* Transfer error in reception process */
    //uartDbgError_Handler(__FILE__, __LINE__);
    return(HAL_ERROR);
  }
  return(HAL_OK);
} 

/**
*
* @brief       Global interrupt handler for USART7 used by 
*              Debug
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/
void debug_IRQHandler(void) 
{
  if (initUartDBG_End == 0xA1)
  {
    /* la gestione della ricezione sulle USART è fatta con il DMA e con l'interrupt di idle */
    /* In pratica ogni carattere ricevuto viene trasferito in memoria tramite DMA e solo    */
    /* dopo che la periferiche vede un "idle" fa scattare questo interrupt che decreta la   */
    /* fine del messaggio di ricezione. Il messaggio viene recuperato forzando interrupt    */
    /* del DMA. Nick 05/08/2019                                                             */
    HAL_UART_IRQHandler(&UART_DBG_HANDLE);

    /* Check for IDLE flag */
    if (UART_DBG_ISR & USART_FLAG_IDLE) 
    {   
      /* We want IDLE flag only */
      /* This part is important */
      /* Clear IDLE flag by reading status register first */
      __HAL_UART_CLEAR_IDLEFLAG(&UART_DBG_HANDLE); 
      /* And follow by reading data register */
      volatile uint32_t tmp;                        /* Must be volatile to prevent optimizations */
      tmp = UART_DBG_ISR;                          /* Read status register */
      tmp = UART_DBG_RDR;                          /* Read data register */
      (void)tmp;                                  /* Prevent compiler warnings */
      /* Disabling DMA will force transfer complete interrupt if enabled */
      _DISABLE_DBG_RX_DMA;
    }
  }
  else
  {
    MX_DBG_UART_DeInit();
  }
}
 
/**
*
* @brief       Global interrupt handler for DMA2 stream1 for Rx 
*              USART7 used by DEBUG
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/
void UART_DBG_DMA_IRQHandler(void) 
{
    xQueueHandle  uartQueue;
    portBASE_TYPE xHigherPriorityTaskWoken;
    
    xHigherPriorityTaskWoken = pdFALSE;
    /* Check if some data has been received from TeraTerm */
    if (_IS_DBG_RX_DMA_AVAILABLE) 
    { 
      _CLEAR_DBG_RX_AVAILABLE_FLAG;   /* Clear flag for available data: TCF for GD32F4xx or SUSPF for STM32H5xx */        
      /* Calculate number of bytes actually transferred by DMA */
      dbgMsgRx_IT.payloadLen = (uint16_t)(0xFFFF - _GET_DBG_RX_DMA_CNT);
      /* Compute data only if really received */
      if (dbgMsgRx_IT.payloadLen > 0)
      {
        uartQueue = getDbgDlRxQueueHandle();
        dbgMsgRxConsole_IT.payloadLen = dbgMsgRx_IT.payloadLen;
        dbgMsgRxConsole_IT.dbgMsgDir = RX_FROM_HOST;
        memcpy(dbgMsgRxConsole_IT.infoRxDbg, dbgMsgRx_IT.infoRxDbg, 256);
        configASSERT(xQueueSendToBackFromISR(uartQueue, (void *)&dbgMsgRxConsole_IT, &xHigherPriorityTaskWoken) == pdPASS);  // processDbgDl() or wifiManager()
        memset((void*)&dbgMsgRxConsole_IT, 0, (size_t)(sizeof(frameDbg_st))); // for debug put all 0
      }
      /* Prepare DMA for next transfer */
      /* Important! DMA stream won't start if all flags are not cleared first */
      _REINIT_DBG_RX_DMA_CHANNEL((uint32_t)dbgMsgRx_IT.infoRxDbg);        
    }
}

/**
*
* @brief       Starts transmission on debug Uart  using DMA 
*
* @param [in]  uint8_t*: pointer to string to be transmitted  
* @param [in]  uint8_t : message len   
*  
* @retval      none 
*  
****************************************************************/
void UART_DBG_DMA_Tx(uint8_t* pTxBuffer, uint16_t len) 
{
  
  /* Uart dbg must be enabled */
  if (UART_CheckIdleState(&UART_DBG_HANDLE) != HAL_OK)
  {
    /* Transfer error in transmission process */
    uartDbgError_Handler(__FILE__, __LINE__);
  }
//  if(len > 0)
//  {
  memcpy ((void*)&dbgMsgTx_IT, (void*)pTxBuffer, (size_t)len);
  
  /*## Start the transmission process #####################################*/
  /* User start transmission data through "pTxBuffer" buffer */
  if(HAL_UART_Transmit_DMA(&UART_DBG_HANDLE, (uint8_t*)&dbgMsgTx_IT, len)!= HAL_OK)
  {
    /* Transfer error in transmission process */
    uartDbgError_Handler(__FILE__, __LINE__);
  }
  }
//}

/**
*
* @brief       Check if UART is Idle 
*
* @param [in]  none  
*  
* @retval      HAL_StatusTypeDef: HAL_OK if uart Tx idle 
*  
****************************************************************/
HAL_StatusTypeDef UART_DBG_IDLE_Tx(void) 
{
  HAL_UART_StateTypeDef stateTx;

  stateTx = HAL_UART_GetState(&UART_DBG_HANDLE);

  if ((stateTx == HAL_UART_STATE_READY) || (stateTx == HAL_UART_STATE_BUSY_RX))
  {
    return(HAL_OK);
  }
  return(HAL_ERROR);
}

/**
*
* @brief       Rx Transfer completed callback
*
* @param [in]  huart: UART handle  
*  
* @note        In This project never we arrive here. Rx complete is manageded by "line idle" interrupt 
*  
* @retval      none 
*  
****************************************************************/
void HAL_UART_DBG_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t       rxChar;
  uint8_t*      pRxChar;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  pRxChar = (uint8_t* )(huart->pRxBuffPtr - 1);
  rxChar = *pRxChar;
  (void)HAL_UART_Receive_IT(huart, huart->pRxBuffPtr, (uint16_t)1);
  switch (rxDbgStatus)
  {
    case WAIT_END1:
      switch (rxChar)
      {
        case DBG_END1:
        case DBG_END2:
          rxDbgStatus = WAIT_END2;   
          break;
        default:
          break;
      }
      break;

    case WAIT_END2:
      switch (rxChar)
      {
        case DBG_END1:
        case DBG_END2:
          rxDbgStatus = WAIT_END1;   
          /* All bytes has been received, put it in the queue */
          configASSERT(xQueueSendToBackFromISR(getDbgQueueHandle(), (void *)&dbgMsgRx_IT, &xHigherPriorityTaskWoken) == pdPASS);
          break;
      }
        break;

    default:
      break;
  }
}

/**
*
* @brief       Tx Transfer completed callback
*
* @param [in]  huart: UART handle  
*  
* @retval      none 
*  
****************************************************************/
void HAL_UART_DBG_TxCpltCallback(UART_HandleTypeDef *huart)
{

}

/**
*
* @brief       Manage the Rx info when the message is sent a burst from SST
*
* @param [in]  uint8_t*:  pointer to received burst   
* @param [in]  uint16_t*: num bytes in the  received burst   
*  
* @note        The Siemens simulator send messagge in different burst with interval of 20mses or more 
*  
* @retval      none 
*  
****************************************************************/
void burstReceptionDbgMng (uint8_t* pRxChar, uint16_t numBytes)
{   
  uint8_t  ix;
  uint8_t* pBuff;    
  uint8_t  infoRxDbg[DBG_MSG_MAX_LEN];
    
  pBuff = (uint8_t*)infoRxDbg;

  /* process all bytes in the burst */
  for (ix = 0; ix < numBytes; ix++)
  {
    pBuff[msgBurstDbgRx.rxIndex] = pRxChar[ix];
    msgBurstDbgRx.rxIndex++;                    
    switch (msgBurstDbgRx.cRxStatus)
    {
      case WAIT_END1:
        switch (pRxChar[ix])
        {
          case DBG_END1:
          case DBG_END2:
              msgBurstDbgRx.cRxStatus = WAIT_END2; 
              break;
          default:
            break;                    
        }
        break;
            
      case WAIT_END2:
        if ((pRxChar[ix] == DBG_END1) || (pRxChar[ix] == DBG_END2))
        {
          /* if one o more bytes has been received, put it in the queue */
          startConsole((uint8_t*)pBuff);          
        }

        msgBurstDbgRx.rxIndex = 0;
        msgBurstDbgRx.cRxStatus = WAIT_END1;               
        break;
        
      default:
        break;
    }
  }
  
}

/**
*
* @brief        Rx activation for DBBG UART for burst reception 
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void rxDbgBurstActivation (void)
{
 msgBurstDbgRx.cRxStatus = WAIT_END1;
 msgBurstDbgRx.rxIndex   = 0;
 msgBurstDbgRx.nLengthRx = 0;

}

/********* -----------  USART1 -- SBC  -------- ***************************/
/**
*
* @brief       USART PROT  init function
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/
void MX_PROT_UART_SBC_Init (UART_HandleTypeDef *huart, uint32_t br, uint32_t wl, uint32_t bs, uint32_t par)
{

  huart->Instance = UART5; //UART_SBC;
  huart->Init.BaudRate = br; //UART_DBG_DEFAULT_BR;
  huart->Init.WordLength = wl; 
  huart->Init.StopBits = bs;
  huart->Init.Parity = par; //UART_PARITY_EVEN; //UART_PARITY_NONE; //UART_PARITY_EVEN;
  huart->Init.Mode = UART_MODE_TX_RX;
  huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart->Init.OverSampling = UART_OVERSAMPLING_16; //UART_OVERSAMPLING_8; //UART_OVERSAMPLING_16;
#ifndef GD32F4xx  
  huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_ENABLE; //UART_ONE_BIT_SAMPLE_DISABLE; //UART_ONE_BIT_SAMPLE_ENABLE;
  huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
#endif  
  if (HAL_UART_Init(huart) != HAL_OK)
  {
    uartDbgError_Handler(__FILE__, __LINE__);
  }

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void uartDbgError_Handler(char * file, int line)
{
  while(1)
  {
    ;
  }
}

/*************** END OF FILE ******************************************************************************************/


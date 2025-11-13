/**
* @file        sbcUart.c
*
* @brief       Uart for SBC communication - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: sbcUart.c 646 2024-12-11 10:06:21Z npiergi $
*
*     $Revision: 646 $
*
*     $Author: npiergi $
*
*     $Date: 2024-12-11 11:06:21 +0100 (mer, 11 dic 2024) $
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
#include "sbcGsy.h"
#include "prot_OnUsart.h"
#include "wrapper.h"
#include "scuMdb.h"
#include "uart_Legacy.h"
#include "httpserver-socket.h"

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/

/* Macros for GD32F4xx microcontroller */

#ifdef GD32F4xx

#define _GET_SBC_RX_DMA_CNT         UART_SBC_RX_DMA_STREAM->NDTR
#define _IS_SBC_RX_DMA_TC_FLAG      DMA1->LISR & DMA_LISR_TCIF0
#define _CLEAR_SBC_RX_DMA_TC_FLAG   DMA1->LIFCR = DMA_LISR_TCIF0

#define _REINIT_SBC_RX_DMA_CHANNEL(pSource)   UART_SBC_DMA->LIFCR = DMA_LISR_DMEIF0 | DMA_LISR_FEIF0 |   \
                                              DMA_LISR_HTIF0 | DMA_LISR_TCIF0 | DMA_LISR_TEIF0;          \
                                              UART_SBC_RX_DMA_STREAM->M0AR = (uint32_t)pSource;          /* Set memory address for DMA again */  \
                                              UART_SBC_RX_DMA_STREAM->NDTR = NUM_BUFF_SBC_MSG_RX;        /* Set number of bytes to receive */    \
                                              UART_SBC_RX_DMA_STREAM->CR |= DMA_SxCR_EN;                 /* Start DMA transfer */               
#define _DISABLE_SBC_RX_DMA         UART_SBC_RX_DMA_STREAM->CR &= ~DMA_SxCR_EN
                                              
#else

/* Macros for STM32H5xx microcontroller */
                                              
#define _GET_SBC_RX_DMA_CNT         (UART_SBC_RX_DMA_STREAM->CBR1 & DMA_CBR1_BNDT_Msk)                                             
#define _IS_SBC_RX_DMA_TC_FLAG      (UART_SBC_RX_DMA_STREAM->CSR & DMA_CSR_TCF)                                             
#define _CLEAR_SBC_RX_DMA_TC_FLAG   (UART_SBC_RX_DMA_STREAM->CFCR |= DMA_CFCR_TCF)
                                              
#define _REINIT_SBC_RX_DMA_CHANNEL(pSource)   UART_SBC_RX_DMA_STREAM->CFCR = DMA_CFCR_TCF | DMA_CFCR_HTF | DMA_CFCR_DTEF | \
                                              DMA_CFCR_USEF | DMA_CFCR_ULEF | DMA_CFCR_SUSPF | DMA_CFCR_TOF;               \
                                              UART_SBC_RX_DMA_STREAM->CSAR = (uint32_t)pSource;          /* Set memory address for DMA again */  \
                                              UART_SBC_RX_DMA_STREAM->CBR1 = NUM_BUFF_SBC_MSG_RX;        /* Set number of bytes to receive */    \
                                              UART_SBC_RX_DMA_STREAM->CCR |= DMA_CCR_EN;                 /* Start DMA transfer */                
#define _DISABLE_SBC_RX_DMA         UART_SBC_RX_DMA_STREAM->CCR &= ~DMA_CCR_EN
                                              
#endif

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 


/* Buffer used for reception and transmission */
frameSbcRx_st          sbcMsgRx_IT;
frameSbcTx_st          sbcMsgTx_IT;
uint16_t               tempLen;


/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/

extern xQueueHandle  sbcQueue;
extern statusFlag_e  fastBridgeStatus;
extern uint32_t      counterSlaveDwnl;

/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Function                             **
**                                                                          **
******************************************************************************
*/

extern BaseType_t xQueueResetFromISR( QueueHandle_t xQueue );

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
static void   uartEmError_Handler    (char * file, int line);

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/

/**
*
* @brief       Global interrupt handler for USART5 used in SBC 
*              communication
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/
void sbc_IRQHandler(void) 
{
  /* la gestione della ricezione sulle USART è fatta con il DMA e con l'interrupt di idle */
  /* In pratica ogni carattere ricevuto viene trasferito in memoria tramite DMA e solo    */
  /* dopo che la periferiche vede un "idle" fa scattare questo interrupt che decreta la   */
  /* fine del messaggio di ricezione. Il messaggio viene recuperato forzando interrupt    */
  /* del DMA. Nick 05/08/2019                                                             */
  HAL_UART_IRQHandler(&UART_SBC_HANDLE);
  uint32_t  regISR;
  portBASE_TYPE xHigherPriorityTaskWoken;

  regISR = UART_SBC_ISR;
  /* Check for IDLE flag */
  if ((regISR & (USART_FLAG_ORE | USART_FLAG_FE | USART_FLAG_PE)) != (uint32_t)0) 
  { 
    
    /* The ORE bit is cleared by reading the SR register and then the DR register */
    volatile uint32_t tmp;                        /* Must be volatile to prevent optimizations */
    tmp = UART_SBC_ISR;
    tmp = UART_SBC_RDR;                           /* Read data register */
    (void)tmp;                                    /* Prevent compiler warnings */
    
    /* Clear error flags  */
    __HAL_UART_CLEAR_IDLEFLAG(&UART_SBC_HANDLE); 
    __HAL_UART_CLEAR_OREFLAG(&UART_SBC_HANDLE); 
    __HAL_UART_CLEAR_PEFLAG(&UART_SBC_HANDLE);
    __HAL_UART_CLEAR_FEFLAG(&UART_SBC_HANDLE);
#ifndef GD32F4xx      
    __HAL_UART_CLEAR_RTOFLAG(&UART_SBC_HANDLE); 
#endif
    sbcMsgRx_IT.totalLen = (uint16_t)0;
    sbcMsgRx_IT.messageEv = UART_RX_KO;
    if (xSemaphoreTakeFromISR(getSbcUartInitSemaphoreHandle(), &xHigherPriorityTaskWoken) == pdTRUE)
    {
      /* there are some error: send a message to restart UART5 */
      xQueueSendToBackFromISR(getSbcAnswerQueueHandle(), (void *)&sbcMsgRx_IT, &xHigherPriorityTaskWoken);
    }

  }
  else
  {
    /* Check for IDLE flag */             /* Nick modifica perchè SBC a 115200 introduce un idle >> di uno sto bit */
    if (UART_SBC_ISR & USART_FLAG_IDLE)   /* Stefano --> la gestione è stata reinserita per GD32F4xx siccome non c'è la funzionalità di timeout su SBC */                                            
    {
      /* We want IDLE flag only */
      /* This part is important */
      /* Clear IDLE and RTO flags by reading status register first */
      __HAL_UART_CLEAR_IDLEFLAG(&UART_SBC_HANDLE);     /* Nick modifica perchè SBC a 115200 introduce un idle >> di uno sto bit */
      /* And follow by reading data register */
      volatile uint32_t tmp;                        /* Must be volatile to prevent optimizations */
      tmp = UART_SBC_ISR;                           /* Read status register */
      tmp = UART_SBC_RDR;                           /* Read data register */
      (void)tmp;                                    /* Prevent compiler warnings */      

      /* In case of FW update of a slave device */
      if (fastBridgeStatus == ENABLED)
      {
        return;        
      }
      
      /* During the FW download, the end of the stream is managed checking the DMA counter */
      if (codeInfo.state == FW_STATE_RX)
      {
        if (_GET_SBC_RX_DMA_CNT == (NUM_BUFF_SBC_MSG_RX - BUFFER_FW_PAYLOAD_CKS))
        {
            _DISABLE_SBC_RX_DMA;   /* Disabling DMA will force transfer complete interrupt if enabled */
        }              
        return;
      }
      
      /* Send message to the task */
      if (xQueueIsQueueFullFromISR(getSbcUartRxTimeoutQueueHandle()) == pdFALSE)
      {                 
        /* An IDLE condition is detected */
        sbcMsgRx_IT.messageEv = UART_RX_IDLE;                
        /* Send a message to the task responsible to manage the timeout condition */
        xQueueSendToBackFromISR(getSbcUartRxTimeoutQueueHandle(), (void *)&sbcMsgRx_IT, &xHigherPriorityTaskWoken);   // sbcUartRxTimeoutTask()
      }

    }
  }
}

/**
*
* @brief       Global interrupt handler for DMA1 stream0 for 
*              USART5 used for Rx from SBC
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/
void UART_SBC_IRQHandler_DMA_IRQHandler(void) 
{
  portBASE_TYPE xHigherPriorityTaskWoken;
  void*         pData;
  uint8_t       ix, cnt, len;
  scuOpModes_e  opMode;
  
  xHigherPriorityTaskWoken = pdFALSE;
  
  /* Check transfer complete flag */
  if (_IS_SBC_RX_DMA_TC_FLAG)
  {
      /* Clear transfer complete flag */  
      _CLEAR_SBC_RX_DMA_TC_FLAG;
      /* Calculate number of bytes actually transfered by DMA so far */
      /**
       * Transfer could be completed by 2 events:
       *  - All data actually transfered (NDTR = 0)
       *  - Stream disabled inside USART IDLE line detected interrupt (NDTR != 0)
       *  For both events we send DMA RX buffer using debug queue 
       **/
      pData = (void*)getDMAptr(PROT_UART_SBC);
      //GPIOD->ODR ^= (uint32_t)0x00000002; /* only for debug */
      if (_GET_SBC_RX_DMA_CNT < NUM_BUFF_SBC_MSG_RX)  // 1024 + 16 = 1040bytes 
      {
        tempLen = (uint16_t)(NUM_BUFF_SBC_MSG_RX - _GET_SBC_RX_DMA_CNT);
        sbcMsgRx_IT.totalLen = (uint16_t)(NUM_BUFF_SBC_MSG_RX - _GET_SBC_RX_DMA_CNT);
        sbcMsgRx_IT.messageEv = UART_RX_OK;
        if (sbcMsgRx_IT.totalLen <= SBC_BUFF_SIZE_GSY)
        {
          opMode = getScuOpMode();
          if (opMode >= SCU_M_P)
          {
            /* SEM enviroment */
            if (xQueueIsQueueFullFromISR(getSbcAnswerQueueHandle()) == pdFALSE)
            {
              /* if one o more bytes has been received, put it in the queue */
              xQueueSendToBackFromISR(getSbcAnswerQueueHandle(), (void *)&sbcMsgRx_IT, &xHigherPriorityTaskWoken);   // sbcGestTask() --> getFromModbusMap()
            }
          }
          else
          {
            if (sbcMsgRx_IT.totalLen >= SBC_MIN_SIZE_GSY)
            {
              if ((((uint8_t*)pData)[2] + 3 == sbcMsgRx_IT.totalLen) || (((uint8_t*)pData)[0] == BOOT_FRAME_START))
              {
                if (xQueueIsQueueFullFromISR(getSbcAnswerQueueHandle()) == pdFALSE)
                {
                  /* if one o more bytes has been received, put it in the queue */
                  //configASSERT(xQueueSendToBackFromISR(getSbcAnswerQueueHandle(), (void *)&sbcMsgRx_IT, &xHigherPriorityTaskWoken) == pdPASS);
                  xQueueSendToBackFromISR(getSbcAnswerQueueHandle(), (void *)&sbcMsgRx_IT, &xHigherPriorityTaskWoken);   // sbcGestTask()
                }
              }
              else
              {
                len = (uint8_t)sbcMsgRx_IT.totalLen;
                /* a volte è possibile che SBC accorpi  due messaggi uno immediatamente in coda all'altro */
                sbcMsgRx_IT.totalLen = (uint16_t)((uint8_t*)pData)[2] + 3;

                if (len >= sbcMsgRx_IT.totalLen)
                {
                  ix = (uint8_t)sbcMsgRx_IT.totalLen;
                  if (xQueueIsQueueFullFromISR(getSbcAnswerQueueHandle()) == pdFALSE)
                  {
                    xQueueSendToBackFromISR(getSbcAnswerQueueHandle(), (void *)&sbcMsgRx_IT, &xHigherPriorityTaskWoken);  // sbcGestTask()
                    /* extract second messaggio */
                    if (len > (uint8_t)sbcMsgRx_IT.totalLen)
                    {
                      if ((len - (uint8_t)sbcMsgRx_IT.totalLen) >= SBC_MIN_SIZE_GSY)
                      {
                        sbcMsgRx_IT.totalLen = (uint16_t)(len - (uint8_t)sbcMsgRx_IT.totalLen);
                        for (cnt = 0; ix < len; ix++, cnt++)
                        { 
                          ((uint8_t*)pData)[cnt] = ((uint8_t*)pData)[ix];
                        }
                        /* if one o more bytes has been received, put it in the queue */
                        configASSERT(xQueueSendToBackFromISR(getSbcAnswerQueueHandle(), (void *)&sbcMsgRx_IT, &xHigherPriorityTaskWoken) == pdPASS);
                      }
                    }
                  }
                }
              }
            }
          }
        }
        else
        {
          sbcMsgRx_IT.messageEv = UART_RX_DWLD;
          /* long message has been received: only for download FW it is possible  */
          configASSERT(xQueueSendToBackFromISR(getScuGsyDwldQueueHandle(), (void *)&sbcMsgRx_IT, &xHigherPriorityTaskWoken) == pdPASS);
        }
      }
      /* Prepare DMA for next transfer */
      /* Important! DMA stream won't start if all flags are not cleared first */
      _REINIT_SBC_RX_DMA_CHANNEL ((uint32_t)pData);       
  }
} 

/**
*
* @brief       Starts transmission on protocol Uart5  using DMA
*
* @param [in]  uint8_t*: pointer to string to be transmitted  
* @param [in]  uint8_t : message len   
*  
* @retval      none 
*  
****************************************************************/
void UART_SBC_DMA_Tx(uint8_t* pTxBuffer, uint16_t len) 
{
  UART_HandleTypeDef *huart;

  if ((len != 0) && (pTxBuffer != NULL))
  {
    huart = &UART_SBC_HANDLE;
    /* Uart dbg must be enabled */
    if (UART_CheckIdleState(&UART_SBC_HANDLE) != HAL_OK)
    {
      /* Transfer error in transmission process */
      uartEmError_Handler(__FILE__, __LINE__);
    }
    while (HAL_DMA_GetState(huart->hdmatx) != HAL_DMA_STATE_READY)
    {
      osDelay(50);
    }

    memcpy ((void*)&sbcMsgTx_IT.messageTx, (void*)pTxBuffer, (size_t)len);

#ifdef GD32F4xx    
    /* Enable the Transmit complete interrupt in order to move the DE pin required to manage the RS-485 RX phase */
    SET_BIT(huart->Instance->CR1, USART_CR1_TCIE);
#endif    
    /*## Start the transmission process #####################################*/
    /* User start transmission data through "pTxBuffer" buffer */
    //if(HAL_UART_Transmit_DMA(&huart6, (uint8_t*)pTxBuffer, len)!= HAL_OK)
    if(HAL_UART_Transmit_DMA(&UART_SBC_HANDLE, (uint8_t*)&sbcMsgTx_IT.messageTx, len)!= HAL_OK)
    {
      /* Transfer error in transmission process */
      uartEmError_Handler(__FILE__, __LINE__);
    }
  }
}

/**
*
* @brief       Check if UART SBC is Idle 
*
* @param [in]  none  
*  
* @retval      HAL_StatusTypeDef: HAL_OK if uart Tx idle 
*  
****************************************************************/
HAL_StatusTypeDef UART_SBC_IDLE_Tx(void) 
{
  HAL_UART_StateTypeDef stateTx;

  stateTx = HAL_UART_GetState(&UART_SBC_HANDLE);

  if ((stateTx == HAL_UART_STATE_READY) || (stateTx == HAL_UART_STATE_BUSY_RX))
  {
    return(HAL_OK);
  }
  return(HAL_ERROR);
}



/**
*
* @brief       Get the pointer for the EM Rx DMA transfer 
*
* @param [in]  none  
*  
* @retval      uint8_t*: the Rx DMA pointer as first element in the MODBUS answer
*  
****************************************************************/
uint8_t* getBuffRxSbc (void)
{
  return ((uint8_t*)&sbcMsgRx_IT.messageRx);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void uartEmError_Handler(char * file, int line)
{
  while(1)
  {
    ;
  }
}

/*************** END OF FILE ******************************************************************************************/


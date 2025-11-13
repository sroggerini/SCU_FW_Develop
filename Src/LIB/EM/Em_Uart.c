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
*     $Id: Em_Uart.c 646 2024-12-11 10:06:21Z npiergi $
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
#endif
#include "cmsis_os.h"
#include "dbg_Task.h"
#include "DataLink_dbg.h"
#include "Em_Task.h"
#include "prot_OnUsart.h"
#include "wrapper.h"
#include "uart_Legacy.h"   
 
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
frameEm_st          emMsgRx_IT;
frameEm_st          emMsgTx_IT;


/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/

extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef handle_GPDMA1_Channel3;

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
static void   uartEmError_Handler    (char * file, int line);

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/

#ifdef GD32F4xx

/************ Function redefinition for STM32F4xx device ***************/

/**
  * @brief Check the UART Idle State.
  * @param huart UART handle.
  * @retval HAL status
  */
HAL_StatusTypeDef UART_CheckIdleState(UART_HandleTypeDef *huart)
{
  
  // REMOVED BECAUSE THIS CHECK DOESN'T WORK --> uint32_t tickstart;
  // REMOVED BECAUSE THIS CHECK DOESN'T WORK --> 
  // REMOVED BECAUSE THIS CHECK DOESN'T WORK --> /* Initialize the UART ErrorCode */
  // REMOVED BECAUSE THIS CHECK DOESN'T WORK --> huart->ErrorCode = HAL_UART_ERROR_NONE;
  // REMOVED BECAUSE THIS CHECK DOESN'T WORK --> 
  // REMOVED BECAUSE THIS CHECK DOESN'T WORK --> /* Init tickstart for timeout managment*/
  // REMOVED BECAUSE THIS CHECK DOESN'T WORK --> tickstart = HAL_GetTick();
  // REMOVED BECAUSE THIS CHECK DOESN'T WORK --> 
  // REMOVED BECAUSE THIS CHECK DOESN'T WORK --> /* Check if the Transmitter is enabled */
  // REMOVED BECAUSE THIS CHECK DOESN'T WORK --> if ((huart->Instance->CR1 & USART_CR1_TE) == USART_CR1_TE)
  // REMOVED BECAUSE THIS CHECK DOESN'T WORK --> {            
  // REMOVED BECAUSE THIS CHECK DOESN'T WORK -->    /* Wait until IDLE flag is reset */
  // REMOVED BECAUSE THIS CHECK DOESN'T WORK -->    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) == RESET)
  // REMOVED BECAUSE THIS CHECK DOESN'T WORK -->    {
  // REMOVED BECAUSE THIS CHECK DOESN'T WORK -->       if((HAL_GetTick() - tickstart ) > HAL_UART_TIMEOUT_VALUE)
  // REMOVED BECAUSE THIS CHECK DOESN'T WORK -->         return HAL_TIMEOUT;
  // REMOVED BECAUSE THIS CHECK DOESN'T WORK -->    }
  // REMOVED BECAUSE THIS CHECK DOESN'T WORK --> }

  /* This function was originally used by STM32F7xx to check the REACK and TEACK flags before 
     starting the TX process.Since in STM32F4xx such flags are not present, the only action 
     we have to take in place is to set the correct UART state                               */
  
  /* Initialize the UART State */
  huart->gState = HAL_UART_STATE_READY;
  huart->RxState = HAL_UART_STATE_READY;

  __HAL_UNLOCK(huart);

  return HAL_OK;
}
#endif

/**
*
* @brief       Global interrupt handler for USART used by Debug
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/
void em_IRQHandler(void) 
{
  /* la gestione della ricezione sulle USART è fatta con il DMA e con l'interrupt di idle */
  /* In pratica ogni carattere ricevuto viene trasferito in memoria tramite DMA e solo    */
  /* dopo che la periferiche vede un "idle" fa scattare questo interrupt che decreta la   */
  /* fine del messaggio di ricezione. Il messaggio viene recuperato forzando interrupt    */
  /* del DMA. Nick 05/08/2019                                                             */
  HAL_UART_IRQHandler(&UART_EM_HANDLE);

  /* Check for IDLE flag */
  if (UART_EM_ISR & UART_FLAG_IDLE) 
  {   
    /* We want IDLE flag only */
    /* This part is important */
    /* Clear IDLE flag by reading status register first */
    __HAL_UART_CLEAR_IDLEFLAG(&UART_EM_HANDLE); 
    /* And follow by reading data register */
    volatile uint32_t tmp;                       /* Must be volatile to prevent optimizations */
    tmp = UART_EM_ISR;                           /* Read status register */
    tmp = UART_EM_RDR;                           /* Read data register */
    (void)tmp;                                   /* Prevent compiler warnings */
#ifdef GD32F4xx    
    UART_EM_RX_DMA_STREAM->CR &= ~DMA_SxCR_EN;   /* Disabling DMA will force transfer complete interrupt if enabled */
#else
    __HAL_DMA_DISABLE(&handle_GPDMA1_Channel3);   /* Disabling DMA will force transfer complete interrupt if enabled (????) */
#endif    
        
  }
  
#ifdef GD32F4xx    
  /* Check for transmit complete flag */
  if (UART_EM_ISR & UART_FLAG_TC)
  {
    /* Clear Transmit complete flag */
    __HAL_UART_CLEAR_FLAG(&UART_EM_HANDLE, UART_FLAG_TC);     
    /* Move the DE pin in order to manage the RS-485 communication and enable the RX part */
    HAL_GPIO_WritePin(UART2_DE_GPIO_Port, UART2_DE_Pin, GPIO_PIN_RESET);   
  }
#endif
}
 
#ifdef GD32F4xx

/**
*
* @brief       Global interrupt handler for DMA1 stream5 for 
*              USART2 used for Rx from Energy meter
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/

void UART_EM_IRQHandler_DMA_IRQHandler(void) 
{
  portBASE_TYPE xHigherPriorityTaskWoken;
  void*         pData;
  uint8_t*      pChar;
  
  xHigherPriorityTaskWoken = pdFALSE;
  /* Check transfer complete flag */
  if (DMA1->HISR & DMA_HISR_TCIF5) 
  { 
      DMA1->HIFCR = DMA_HISR_TCIF5;           /* Clear transfer complete flag */
      
      /* Calculate number of bytes actually transfered by DMA so far */
      /**
       * Transfer could be completed by 2 events:
       *  - All data actually transfered (NDTR = 0)
       *  - Stream disabled inside USART IDLE line detected interrupt (NDTR != 0)
       *  For both events we send DMA RX buffer using debug queue 
       **/
      pData = (void*)getDMAptr(PROT_UART_EM);
      if (UART_EM_DMA_STREAM->NDTR < DMA_RX_EM_BUFFER_SIZE)
      {
        emMsgRx_IT.totalLen = (uint16_t)(DMA_RX_EM_BUFFER_SIZE - UART_EM_DMA_STREAM->NDTR);
        
        pChar = (uint8_t*)pData;
        if (((pChar[0] == 0x01) || (pChar[0] == 0x02)) && (emMsgRx_IT.totalLen > 4))
        {
          if (xQueueIsQueueFullFromISR(getEmAnswerQueueHandle()) == pdFALSE)
          {
            /* if one o more bytes has been received, put it in the queue */
            configASSERT(xQueueSendToBackFromISR(getEmAnswerQueueHandle(), (void *)&emMsgRx_IT, &xHigherPriorityTaskWoken) == pdPASS);
          }
        }
      }
      /* Prepare DMA for next transfer */
      /* Important! DMA stream won't start if all flags are not cleared first */
      UART_EM_DMA->HIFCR = DMA_HISR_DMEIF5 | DMA_HISR_FEIF5 | 
                              DMA_HISR_HTIF5 | DMA_HISR_TCIF5 | DMA_HISR_TEIF5;
      UART_EM_DMA_STREAM->M0AR = (uint32_t)pData;                           /* Set memory address for DMA again */
      UART_EM_DMA_STREAM->NDTR = DMA_RX_EM_BUFFER_SIZE;                     /* Set number of bytes to receive */
      UART_EM_DMA_STREAM->CR |= DMA_SxCR_EN;                                /* Start DMA transfer */
  }
}

/**
*
* @brief       Global interrupt handler for DMA1 stream6 for 
*              USART2 used for Tx to Energy meter
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/

void UART_EM_DMA_TX_IRQHandler(void)
{
      
  /* Manage associated IRQHandler */  
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  
}

#else

/**
*
* @brief       Global interrupt handler for DMA1 stream5 for 
*              USART2 used for Rx from Energy meter
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/

void UART_EM_IRQHandler_DMA_IRQHandler(void) 
{
  portBASE_TYPE xHigherPriorityTaskWoken;
  void*         pData;
  uint8_t*      pChar;
  
  xHigherPriorityTaskWoken = pdFALSE;
  /* Check transfer complete flag */
  if ((__HAL_DMA_GET_FLAG(&handle_GPDMA1_Channel3, DMA_FLAG_TC) != 0U)) 
  { 
      /* Clear transfer complete flag */
      __HAL_DMA_CLEAR_FLAG(&handle_GPDMA1_Channel3, DMA_FLAG_TC);      
      /* Calculate number of bytes actually transfered by DMA so far */
      /**
       * Transfer could be completed by 2 events:
       *  - All data actually transfered (NDTR = 0)
       *  - Stream disabled inside USART IDLE line detected interrupt (NDTR != 0)
       *  For both events we send DMA RX buffer using debug queue 
       **/
      pData = (void*)getDMAptr(PROT_UART_EM);
      if (__HAL_DMA_GET_COUNTER(&handle_GPDMA1_Channel3) < DMA_RX_EM_BUFFER_SIZE)
      {
        emMsgRx_IT.totalLen = (uint16_t)(DMA_RX_EM_BUFFER_SIZE - (UART_EM_RX_DMA_STREAM->CBR1 & DMA_CBR1_BNDT));
        
        pChar = (uint8_t*)pData;
        if (((pChar[0] == 0x01) || (pChar[0] == 0x02)) && (emMsgRx_IT.totalLen > 4))
        {
          if (xQueueIsQueueFullFromISR(getEmAnswerQueueHandle()) == pdFALSE)
          {
            /* if one o more bytes has been received, put it in the queue */
            configASSERT(xQueueSendToBackFromISR(getEmAnswerQueueHandle(), (void *)&emMsgRx_IT, &xHigherPriorityTaskWoken) == pdPASS);
          }
        }
      }
      /* Prepare DMA for next transfer */
      /* Important! DMA stream won't start if all flags are not cleared first */  
      __HAL_DMA_CLEAR_FLAG(&handle_GPDMA1_Channel3, (DMA_FLAG_IDLE | DMA_FLAG_TC | DMA_FLAG_HT | DMA_FLAG_DTE | 
                                                    DMA_FLAG_ULE | DMA_FLAG_USE | DMA_FLAG_SUSP | DMA_FLAG_TO));            
      UART_EM_RX_DMA_STREAM->CSAR = (uint32_t)pData;                           /* Set memory address for DMA again */
      __HAL_DMA_SET_COUNTER (&handle_GPDMA1_Channel3, DMA_RX_EM_BUFFER_SIZE);   /* Set number of bytes to receive */
      __HAL_DMA_ENABLE(&handle_GPDMA1_Channel3);                                /* Start DMA transfer */
  }
  
}


#endif 

/**
*
* @brief       Starts transmission on protocol Uart2  using DMA
*
* @param [in]  uint8_t*: pointer to string to be transmitted  
* @param [in]  uint8_t : message len   
*  
* @retval      none 
*  
****************************************************************/
void UART_EM_DMA_Tx(uint8_t* pTxBuffer, uint16_t len) 
{

  memcpy ((void*)&emMsgTx_IT.messageTx, (void*)pTxBuffer, (size_t)len);

  /* Uart dbg must be enabled */
  if (UART_CheckIdleState(&UART_EM_HANDLE) != HAL_OK)
  {
    /* Transfer error in transmission process */
    uartEmError_Handler(__FILE__, __LINE__);
  }
    
#ifdef GD32F4xx  
  /* Since it's managed like an RS485, put the DE pin to HIGH level in order to enable the TX */
  HAL_GPIO_WritePin(UART2_DE_GPIO_Port, UART2_DE_Pin, GPIO_PIN_SET);
#endif  
    
  /*## Start the transmission process #####################################*/
  /* User start transmission data through "pTxBuffer" buffer */
  //if(HAL_UART_Transmit_DMA(&huart6, (uint8_t*)pTxBuffer, len)!= HAL_OK)
  if(HAL_UART_Transmit_DMA(&UART_EM_HANDLE, (uint8_t*)&emMsgTx_IT.messageTx, len)!= HAL_OK)
  {
    /* Transfer error in transmission process */
    uartEmError_Handler(__FILE__, __LINE__);
  }
}

/**
*
* @brief       Check if UART EM is Idle 
*
* @param [in]  none  
*  
* @retval      HAL_StatusTypeDef: HAL_OK if uart Tx idle 
*  
****************************************************************/
HAL_StatusTypeDef UART_EM_IDLE_Tx(void) 
{
  HAL_UART_StateTypeDef stateTx;

  stateTx = HAL_UART_GetState(&UART_EM_HANDLE);

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
uint8_t* getBuffRxEm (void)
{
  return ((uint8_t*)&emMsgRx_IT.messageTx.nodeReadInputReg.unitId);
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


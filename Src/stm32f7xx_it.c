/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_it.h"
#include "FreeRTOS.h" 
#include "task.h"
#include "dbg_Task.h"
#include "adcTask.h"
#include "Em_Task.h"
#include "sbcGsy.h"
#include "scuMdb.h"
#include "ioExp.h"
#include "flashFat.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
  
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static  uint16_t lcdRunning = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External function prototypes -----------------------------------------------*/
/* USER CODE BEGIN EFP */

extern void xPortSysTickHandler (void);

/* USER CODE END EFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ETH_HandleTypeDef heth;

/* USER CODE BEGIN EV */
extern UART_HandleTypeDef   huart1;
extern UART_HandleTypeDef   huart2;
extern UART_HandleTypeDef   huart5;
extern UART_HandleTypeDef   huart7;
extern ADC_HandleTypeDef    AdcHandle, AdcTAHandle, AdcVINHandle, AdcCPHandle;
extern TIM_HandleTypeDef    htimAdc, htimV230;
extern RTC_HandleTypeDef    hrtc;
extern DMA_HandleTypeDef*   hDma1St7;                  /*!< set handle for DMA1 Stream 7      */

extern statusFlag_e         fastBridgeStatus;
extern uint16_t             packetNum;

/* USER CODE END EV */

/******************************************************************************/
/*           Local user  function                                             */ 
/******************************************************************************/

/**
  * @brief set the sysTick flag 
  */
void setSysTickStatus(uint16_t status)
{
  lcdRunning = status;
}

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void) 
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1) 
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    if (lcdRunning == FALSE)
    {
      xPortSysTickHandler();
    }
  }
  HAL_IncTick();
}

/**
  * @brief This function handles Ethernet global interrupt.
  */
void ETH_IRQHandler(void)
{
  /* USER CODE BEGIN ETH_IRQn 0 */

  /* USER CODE END ETH_IRQn 0 */
  HAL_ETH_IRQHandler(&heth);
  /* USER CODE BEGIN ETH_IRQn 1 */

  /* USER CODE END ETH_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
* @brief This function handles DMA1 stream1 channel 5 (UART7_TX => DEBUG) global interrupt.
*/
void UART_DBG_DMA_TX_IRQHandler (void)  //  UART_PI1_DMA_TX_IRQHandler
{
  /* USER CODE BEGIN DMA1_Stream1 ch5: USART7 Tx debug  */

  HAL_DMA_IRQHandler(huart7.hdmatx);  

  /* USER CODE END DMA2_Stream1  */ 
}


/**
* @brief This function handles DMA1 stream6 channel 4 (UART2_TX => Energy Meter) global interrupt.
*/
void UART_EM_DMA_TX_IRQHandler(void)
{
  /* USER CODE END DMA1_Stream6 channel 4: USART2 Tx Energy Meter */
  HAL_DMA_IRQHandler(hDma1St7);
  /* USER CODE END DMA1_Stream6 channel 4 */
}


/**
* @brief This function handles USART7 global interrupt.
*/
#ifndef UART_FOR_DEBUG
void UART_DBG_IRQHandler(void)
{
  /* USER CODE BEGIN USART7_IRQn 0 */

  debug_IRQHandler();

  /* USER CODE END USART7_IRQn 1 */
}
#endif

/**
* @brief This function handles DMA1 stream3 UART7 Rx global interrupt. 
*/
void UART_DBG_DMA_RX_IRQHandler(void)
{
  /* USER CODE END DMA1_Stream3 channel 5: USART7 Rx debug */

  UART_DBG_IRQHandler_DMA_IRQHandler();  

  /* USER CODE END DMA2_Stream1_IRQn 0 channel 5 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void UART_EM_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  em_IRQHandler();

  /* USER CODE END USART2_IRQn 1 */
}
 
/**
* @brief This function handles DMA1 stream5 UART2 Rx global interrupt. 
*/
void UART_EM_DMA_RX_IRQHandler(void)
{
  /* USER CODE END DMA1_Stream5 channel 4: USART2 Rx Energy meter  */

  UART_EM_IRQHandler_DMA_IRQHandler();  

  /* USER CODE END DMA1_Stream5 0 channel 4 */
}

/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler (void) 
{
  HAL_TIM_IRQHandler(&htimAdc); 
}

/**
* @brief  This function handles DMA interrupt request.
* @param  None
* @retval None
*/
void ADCx_DMA_IRQHandler(void)
//void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(AdcHandle.DMA_Handle);
}

/**
* @brief  This function handles DMA2 Stream1 CH2 interrupt request used on TA measure
* @param  None
* @retval None
*/
void ADCxTA_DMA_IRQHandler(void)
{
  HAL_DMA_IRQHandler(AdcTAHandle.DMA_Handle);
}

/**
* @brief  This function handles DMA2 Stream2 CH1 interrupt request used on CP_ADC measure
* @param  None
* @retval None
*/
void ADCxCP_DMA_IRQHandler(void)
{
  HAL_DMA_IRQHandler(AdcCPHandle.DMA_Handle);
}


/**
* @brief  This function handles ADC interrupt request 
* @param  None
* @retval None
*/
void ADC_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&AdcVINHandle);
}




/* ---------------   SBC UART5 interrupt handlers  -------------------- */
/**
* @brief This function handles USART5 global interrupt.
*/
void UART_SBC_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  uint32_t sbcRx, isrflags, errorflags;

//  GPIOD->ODR ^= (uint32_t)0x00000002; /* only for debug */
  /* USER CODE BEGIN USAR12_IRQn 0 */
  isrflags   = UART_SBC->ISR;
  if ((fastBridgeStatus == DISABLED) || ((isrflags & USART_ISR_RXNE) == 0))
  {
    sbc_IRQHandler();
  }
  else
  {
    errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE | USART_ISR_RTOF));
    UART_SBC->ICR = (USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NCF | USART_ICR_ORECF | USART_ICR_IDLECF);
    if ((isrflags & USART_ISR_RXNE) != 0U)
    {
      if (errorflags == (uint32_t)0)
      {
        sbcRx = UART_SBC->RDR;  /* received byte from SBC */
        UART_SCU->TDR = sbcRx;  /* bridge on RS485 UART */
        packetNum--;
        if (packetNum == 0)
        {
          lcdRunning = FALSE;  /* we use the same flag in LCD enviroment to stop/restart calls to FREERTOS */
          packetNum = BUFFER_FW_PAYLOAD_CKS;
        }
        else
        {
          lcdRunning = TRUE;
        }
      }
      else
      {
        /* error: the SBC come back to 19200...  */
        setFlagForNvic();
        NVIC_SystemReset();
      }
    }
    else
    {
      lcdRunning = FALSE; 
    }
  }
//  GPIOD->ODR ^= (uint32_t)0x00000002; /* only for debug */


  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream5 UART2 Rx global interrupt. 
*/
void UART_SBC_DMA_RX_IRQHandler(void)
{
  /* USER CODE END DMA1_Stream0 channel 4: USART5 Rx SBC  */

  UART_SBC_IRQHandler_DMA_IRQHandler();  

  /* USER CODE END DMA1_Stream0 0 channel 4 */
}

/**
* @brief This function handles DMA1 stream7 channel 4 (UART5_TX => SBC) global interrupt.
*/
void UART_SBC_DMA_TX_IRQHandler(void)
{
  frameScuToSbcTx_st  tmpFrameScuToSbcTx;
  portBASE_TYPE       xHigherPriorityTaskWoken;

  xHigherPriorityTaskWoken = pdFALSE;

  /* USER CODE END DMA1_Stream7 channel 4: USART5 Tx SBC */
  /* remember: to SBC the messages must have 5msec distance */
  HAL_DMA_IRQHandler(UART_SBC_HANDLE.hdmatx);  

  tmpFrameScuToSbcTx.msgEv = SCU_END_TX_MSG;
  tmpFrameScuToSbcTx.pDataToSend = NULL;
  tmpFrameScuToSbcTx.totalLen = (uint16_t)0;

  configASSERT(xQueueSendToBackFromISR(getScuToSbcTxQueueHandle(), (void *)&tmpFrameScuToSbcTx, &xHigherPriorityTaskWoken) == pdPASS); // scuTxToSbcTask

  /* USER CODE END DMA1_Stream7 channel 4 */
}

/**
  * @brief  This function handles external lines 9 to 5 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(V230_WD_PIN);
  HAL_GPIO_EXTI_IRQHandler(RCDM_Pin);
}

/**
  * @brief  This function handles external line 1 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(CAN1_TX_Pin);
}


/**
  * @brief  This function handles RTC Auto wake-up interrupt request.
  * @param  None
  * @retval None
  */
void RTC_WKUP_IRQHandler(void)
{
  HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
}

/* USART1 Rx for SCU RS485 modbus */
/**
* @brief This function handles USART1 global interrupt.
*/
void UART_SCU_IRQHandler(void)
{
  /* USER CODE BEGIN USAR12_IRQn 0 */

  scuModbus_IRQHandler();

  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream7 channel 4 (UART1_TX => SCU MODBUS) global interrupt.
*/
void UART_SCU_DMA_TX_IRQHandler(void)
{
  /* USER CODE END DMA2_Stream7 channel 4: USART2 Tx SCU Modbus */

  HAL_DMA_IRQHandler(UART_SCU_HANDLE.hdmatx);  

  /* USER CODE END DMA2_Stream7 channel 4 */
}

/**
  * @brief  This function handles external lines 15 to 10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
  /* the interrupt is due to PEN function (INT_INx = PE12 pin) */
  HAL_GPIO_EXTI_IRQHandler(INT_INx_Pin);
}

/**
  * @brief  This function handles TIM7 V230 OFF interrupt request.
  * @param  None
  * @retval None
  */
void TIMxV230_IRQHandler (void) 
{
  HAL_TIM_IRQHandler(&htimV230); 
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

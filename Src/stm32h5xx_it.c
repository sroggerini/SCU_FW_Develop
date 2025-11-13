/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h5xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h5xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
#include "dbg_Task.h"
#include "Em_Task.h"
#include "scuMdb.h"
#include "sbcGsy.h"
#include "adcTask.h"
#include "scuWifi.h"
#include "uart_Legacy.h"
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

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef handle_GPDMA1_Channel2;
extern DMA_HandleTypeDef handle_GPDMA1_Channel4;
extern ADC_HandleTypeDef AdcVINHandle, AdcCPHandle;
extern RTC_HandleTypeDef hrtc;
extern DMA_HandleTypeDef handle_GPDMA1_Channel1;
extern DMA_HandleTypeDef handle_GPDMA1_Channel0;
extern DMA_HandleTypeDef handle_GPDMA1_Channel5;
extern DMA_HandleTypeDef handle_GPDMA1_Channel3;
extern DMA_HandleTypeDef handle_GPDMA1_Channel7;
extern DMA_HandleTypeDef handle_GPDMA1_Channel6;
extern DMA_HandleTypeDef handle_GPDMA2_Channel5;
extern DMA_HandleTypeDef handle_GPDMA2_Channel2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* External function prototypes -----------------------------------------------*/
/* USER CODE BEGIN EFP */
extern void xPortSysTickHandler (void);
/* USER CODE END EFP */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/

/**
  * @brief set the sysTick flag 
  */
void setSysTickStatus(uint16_t status)
{
  
}

/**
  * @brief This function init var in bridge UART5 --> UART1-RS485 (SBC --> SCU).
  */
void bridgeVarInit (void)
{
  
}

/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
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
  * @brief This function handles Prefetch fault, memory access fault.
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

#ifndef HW_MP28947
/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}
#endif

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

#ifndef HW_MP28947
/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}
#endif

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
#ifdef DEBUG_TRACE_PIN   
#ifndef HW_MP28947  
    HAL_GPIO_TogglePin(GPIO1_uP_GPIO_Port, GPIO1_uP_Pin);  // trigger for debug GPIO1 --> ONLY FOR DEBUG
#else
    // ONLY FOR DEBUG --> HAL_GPIO_TogglePin(RFID_PWR_GPIO_Port, RFID_PWR_Pin);  // trigger on RFID PWR pin 
#endif    
#endif   
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
    
    if (lcdRunning == FALSE)
    {
      xPortSysTickHandler();
    }
    
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
  /* USER CODE END SysTick_IRQn 1 */   
  
}

/******************************************************************************/
/* STM32H5xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h5xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles Tamper global interrupt.
  */
void TAMP_IRQHandler(void)
{
  /* USER CODE BEGIN TAMP_IRQn 0 */

  /* USER CODE END TAMP_IRQn 0 */
  HAL_RTCEx_TamperIRQHandler(&hrtc);
  /* USER CODE BEGIN TAMP_IRQn 1 */

  /* USER CODE END TAMP_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 0 global interrupt.
  */
void GPDMA1_Channel0_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel0_IRQn 0 */

  /* USER CODE END GPDMA1_Channel0_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel0);
  /* USER CODE BEGIN GPDMA1_Channel0_IRQn 1 */

  /* USER CODE END GPDMA1_Channel0_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 1 global interrupt.
  */
void GPDMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel1_IRQn 0 */

  /* USER CODE END GPDMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel1);
  /* USER CODE BEGIN GPDMA1_Channel1_IRQn 1 */

  /* USER CODE END GPDMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 2 global interrupt.
  */
void GPDMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel2_IRQn 0 */

  /* USER CODE END GPDMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel2);
  /* USER CODE BEGIN GPDMA1_Channel2_IRQn 1 */

  /* USER CODE END GPDMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 3 global interrupt.
  */
void GPDMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel3_IRQn 0 */

  /* USER CODE END GPDMA1_Channel3_IRQn 0 */
  // HAL_DMA_IRQHandler(&handle_GPDMA1_Channel3);
  UART_EM_IRQHandler_DMA_IRQHandler();
  /* USER CODE BEGIN GPDMA1_Channel3_IRQn 1 */

  /* USER CODE END GPDMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 4 global interrupt.
  */
void GPDMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel4_IRQn 0 */

  /* USER CODE END GPDMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel4);
  /* USER CODE BEGIN GPDMA1_Channel4_IRQn 1 */

  /* USER CODE END GPDMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 5 global interrupt.
  */
void GPDMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel5_IRQn 0 */

  /* USER CODE END GPDMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel5);
  /* USER CODE BEGIN GPDMA1_Channel5_IRQn 1 */

  /* USER CODE END GPDMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 6 global interrupt.
  */
void GPDMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel6_IRQn 0 */

  /* USER CODE END GPDMA1_Channel6_IRQn 0 */
  // AUTO-GENERATED by CUBE-MX but INTENTIONALLY REMOVED --> HAL_DMA_IRQHandler(&handle_GPDMA1_Channel6);
    UART_WIFI_DMA_IRQHandler();
  /* USER CODE BEGIN GPDMA1_Channel6_IRQn 1 */

  /* USER CODE END GPDMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles GPDMA1 Channel 7 global interrupt.
  */
void GPDMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA1_Channel7_IRQn 0 */

  /* USER CODE END GPDMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel7);
  /* USER CODE BEGIN GPDMA1_Channel7_IRQn 1 */

  /* USER CODE END GPDMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles ADC1 global interrupt.
  */
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */

  /* USER CODE END ADC1_IRQn 0 */
  HAL_ADC_IRQHandler(&AdcVINHandle);
  /* USER CODE BEGIN ADC1_IRQn 1 */

  /* USER CODE END ADC1_IRQn 1 */
}


/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  // AUTO-GENERATED by CUBE-MX but INTENTIONALLY REMOVED --> HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  em_IRQHandler();
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  // AUTO-GENERATED by CUBE-MX but INTENTIONALLY REMOVED --> HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
  UART_WIFI_IRQHandler();
  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles ADC2 global interrupt.
  */
void ADC2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC2_IRQn 0 */

  /* USER CODE END ADC2_IRQn 0 */
  HAL_ADC_IRQHandler(&AdcCPHandle);
  /* USER CODE BEGIN ADC2_IRQn 1 */

  /* USER CODE END ADC2_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  // INTENTIONALLY REMOVED --> HAL_UART_IRQHandler(&huart6);
  debug_IRQHandler();  
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/**
  * @brief This function handles GPDMA2 Channel 2 global interrupt.
  */
void GPDMA2_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA2_Channel2_IRQn 0 */

  /* USER CODE END GPDMA2_Channel2_IRQn 0 */
  // INTENTIONALLY REMOVED --> HAL_DMA_IRQHandler(&handle_GPDMA2_Channel2);
  UART_DBG_DMA_IRQHandler();  
  /* USER CODE BEGIN GPDMA2_Channel2_IRQn 1 */

  /* USER CODE END GPDMA2_Channel2_IRQn 1 */
}

/**
  * @brief This function handles GPDMA2 Channel 5 global interrupt.
  */
void GPDMA2_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN GPDMA2_Channel5_IRQn 0 */

  /* USER CODE END GPDMA2_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&handle_GPDMA2_Channel5);
  /* USER CODE BEGIN GPDMA2_Channel5_IRQn 1 */

  /* USER CODE END GPDMA2_Channel5_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

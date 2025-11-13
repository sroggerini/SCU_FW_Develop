/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : stm32f7xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
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
/* USER CODE BEGIN Includes */
#include "uartDbg.h"
#include "prot_OnUsart.h"
#include "displayPin.h"
#include "adcTask.h"
#include "Em_Task.h"
#include "sbcGsy.h"
#include "RfidMng.h"
#include "scuMdb.h"
#include "stm32f7xx_periph_init.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */
 
/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static DMA_HandleTypeDef    hdma_tx, hdmaDgb_tx, hdmaScu_tx;
static DMA_HandleTypeDef    hdma_rx, hdmaDgb_rx, hdmaScu_rx;
static DMA_HandleTypeDef    hdma_adc, hdma_adcTA, hdma_adcCP;
static DMA_HandleTypeDef    hdma_em_tx, hdma_em_rx;
static DMA_HandleTypeDef    hdmaSbc_tx, hdmaSbc_rx;
static DMA_HandleTypeDef    hdma_dac1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
                        
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                    /**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}


/**
* @brief QSPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hqspi: QSPI handle pointer
* @retval None
*/
void HAL_QSPI_MspInit(QSPI_HandleTypeDef* hqspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hqspi->Instance==QUADSPI)
  {
  /* USER CODE BEGIN QUADSPI_MspInit 0 */

  /* USER CODE END QUADSPI_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_QSPI_CLK_ENABLE();
  
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**QUADSPI GPIO Configuration    
    PE2     ------> QUADSPI_BK1_IO2
    PB2     ------> QUADSPI_CLK
    PD11     ------> QUADSPI_BK1_IO0
    PD12     ------> QUADSPI_BK1_IO1
    PD13     ------> QUADSPI_BK1_IO3
    PB6     ------> QUADSPI_BK1_NCS 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN QUADSPI_MspInit 1 */

  /* USER CODE END QUADSPI_MspInit 1 */
  }

}

/**
* @brief QSPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hqspi: QSPI handle pointer
* @retval None
*/
void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef* hqspi)
{
  if(hqspi->Instance==QUADSPI)
  {
  /* USER CODE BEGIN QUADSPI_MspDeInit 0 */

  /* USER CODE END QUADSPI_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_QSPI_CLK_DISABLE();
  
    /**QUADSPI GPIO Configuration    
    PE2     ------> QUADSPI_BK1_IO2
    PB2     ------> QUADSPI_CLK
    PD11     ------> QUADSPI_BK1_IO0
    PD12     ------> QUADSPI_BK1_IO1
    PD13     ------> QUADSPI_BK1_IO3
    PB6     ------> QUADSPI_BK1_NCS 
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_2);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2|GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13);

  /* USER CODE BEGIN QUADSPI_MspDeInit 1 */

  /* USER CODE END QUADSPI_MspDeInit 1 */
  }

}

/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
  if(htim_base->Instance==TIM6)
  {
    /* TIM6 Periph clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();
  }

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */
  
    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM1 GPIO Configuration    
    PE9     ------> TIM1_CH1
    PE11     ------> TIM1_CH2
    PE13     ------> TIM1_CH3
    PE14     ------> TIM1_CH4 
    */
    GPIO_InitStruct.Pin = DLED_A_Pin|DLED_B_Pin|DLED_C_Pin|LCD_PWM_T1CH4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }

}
/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }

}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==UART_DBG)
  {
  /* USER CODE BEGIN UART7_MspInit 0 */

  /* USER CODE END UART7_MspInit 0 */
    /* Peripheral clock enable */
    UART_DBB_CLK_ENABLE();
  
    /* Enable DMA clock */
    DMAx_DBG_CLK_ENABLE();
  
    /* Port clock enable */
    UART_DBG_RX_GPIO_CLK_ENABLE();
    /**UART7 GPIO Configuration    
    PE7     ------> UART7_RX
    PE8     ------> UART7_TX 
    */
    GPIO_InitStruct.Pin = UART_DBG_TX_PIN|UART_DBG_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = UART_DBG_RX_AF;
    HAL_GPIO_Init(UART_DBG_TX_GPIO_PORT, &GPIO_InitStruct);

    /* USER CODE BEGIN UART7_MspInit 1 */
    /* USART7 interrupt Init */
    /*## Configure the DMA ##################################################*/
    /* Configure the DMA handler for Transmission process */
    hdmaDgb_tx.Instance                 = UART_DBG_TX_DMA_STREAM;
    hdmaDgb_tx.Init.Channel             = UART_DBG_TX_DMA_CHANNEL;
    hdmaDgb_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdmaDgb_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdmaDgb_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdmaDgb_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdmaDgb_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdmaDgb_tx.Init.Mode                = DMA_NORMAL;
    hdmaDgb_tx.Init.Priority            = DMA_PRIORITY_LOW;
    hdmaDgb_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdmaDgb_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdmaDgb_tx.Init.MemBurst            = DMA_MBURST_INC4;
    hdmaDgb_tx.Init.PeriphBurst         = DMA_PBURST_INC4;

    HAL_DMA_Init(&hdmaDgb_tx);

    /* Associate the initialized DMA handle to the UART handle */
    __HAL_LINKDMA(huart, hdmatx, hdmaDgb_tx);

    /* Configure the DMA handler for reception process */
    hdmaDgb_rx.Instance                 = UART_DBG_RX_DMA_STREAM;
    hdmaDgb_rx.Init.Channel             = UART_DBG_RX_DMA_CHANNEL;
    hdmaDgb_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdmaDgb_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdmaDgb_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdmaDgb_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdmaDgb_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdmaDgb_rx.Init.Mode                = DMA_NORMAL;
    hdmaDgb_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdmaDgb_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdmaDgb_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdmaDgb_rx.Init.MemBurst            = DMA_MBURST_INC4;
    hdmaDgb_rx.Init.PeriphBurst         = DMA_PBURST_INC4;

    HAL_DMA_Init(&hdmaDgb_rx);

    /* Associate the initialized DMA handle to the the UART handle */
    __HAL_LINKDMA(huart, hdmarx, hdmaDgb_rx);

    /*## Configure the NVIC for DMA #########################################*/
    /* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
    HAL_NVIC_SetPriority(UART_DBG_DMA_TX_IRQn, 10, 10);
    HAL_NVIC_EnableIRQ(UART_DBG_DMA_TX_IRQn);

    /* NVIC configuration for DMA transfer complete interrupt (USARTx_RX) */
    HAL_NVIC_SetPriority(UART_DBG_DMA_RX_IRQn, 10, 10);
    HAL_NVIC_EnableIRQ(UART_DBG_DMA_RX_IRQn);

    /* NVIC configuration for USART, to catch the TX complete and the RX line idle (Rx complete)*/
    HAL_NVIC_SetPriority(UART_DBG_IRQn, 10, 10);
    HAL_NVIC_EnableIRQ(UART_DBG_IRQn);

    /* USER CODE END UART7_MspInit 1 */
  }
  else if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    PA12     ------> USART1_DE 
    */
    GPIO_InitStruct.Pin = UART_SCU_TX_PIN|UART_SCU_RX_PIN|UART_SCU_DE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = UART_SCU_TX_AF;
    HAL_GPIO_Init(UART_SCU_TX_GPIO_PORT, &GPIO_InitStruct);
    

    /* USER CODE BEGIN USART1_MspInit 1 */
    /*## Configure the DMA ##################################################*/
    /* Configure the DMA handler for Transmission process */
    hdmaScu_tx.Instance                 = UART_SCU_TX_DMA_STREAM;
    hdmaScu_tx.Init.Channel             = UART_SCU_TX_DMA_CHANNEL;
    hdmaScu_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdmaScu_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdmaScu_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdmaScu_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdmaScu_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdmaScu_tx.Init.Mode                = DMA_NORMAL;
    hdmaScu_tx.Init.Priority            = DMA_PRIORITY_LOW;
    hdmaScu_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdmaScu_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdmaScu_tx.Init.MemBurst            = DMA_MBURST_INC4;
    hdmaScu_tx.Init.PeriphBurst         = DMA_PBURST_INC4;

    HAL_DMA_Init(&hdmaScu_tx);

    /* Associate the initialized DMA handle to the UART handle */
    __HAL_LINKDMA(huart, hdmatx, hdmaScu_tx);

    /* Configure the DMA handler for reception process */
    hdmaScu_rx.Instance                 = UART_SCU_RX_DMA_STREAM;
    hdmaScu_rx.Init.Channel             = UART_SCU_RX_DMA_CHANNEL;
    hdmaScu_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdmaScu_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdmaScu_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdmaScu_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdmaScu_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdmaScu_rx.Init.Mode                = DMA_NORMAL;
    hdmaScu_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdmaScu_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdmaScu_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdmaScu_rx.Init.MemBurst            = DMA_MBURST_INC4;
    hdmaScu_rx.Init.PeriphBurst         = DMA_PBURST_INC4;

    HAL_DMA_Init(&hdmaScu_rx);

    /* Associate the initialized DMA handle to the the UART handle */
    __HAL_LINKDMA(huart, hdmarx, hdmaScu_rx);

    /*## Configure the NVIC for DMA #########################################*/
    /* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
    HAL_NVIC_SetPriority(UART_SCU_DMA_TX_IRQn, 10, 10);
    HAL_NVIC_EnableIRQ(UART_SCU_DMA_TX_IRQn);

    /* NVIC configuration for DMA transfer complete interrupt (USARTx_RX) */
    HAL_NVIC_SetPriority(UART_SCU_DMA_RX_IRQn, 10, 10);
    HAL_NVIC_EnableIRQ(UART_SCU_DMA_RX_IRQn);

    /* NVIC configuration for USART, to catch the TX complete and the RX line idle (Rx complete)*/
    HAL_NVIC_SetPriority(UART_SCU_IRQn, 10, 10);
    HAL_NVIC_EnableIRQ(UART_SCU_IRQn);
    /* USER CODE END USART1_MspInit 1 */
  }
  else if(huart->Instance==UART_EM)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration    
    PD4     ------> USART2_DE
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct); 

  /* USER CODE BEGIN USART2_MspInit 1 */
    /* USART7 interrupt Init */
    /*## Configure the DMA ##################################################*/
    /* Configure the DMA handler for Transmission process */
    hdma_em_tx.Instance                 = UART_EM_TX_DMA_STREAM;
    hdma_em_tx.Init.Channel             = UART_EM_TX_DMA_CHANNEL;
    hdma_em_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_em_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_em_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_em_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_em_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_em_tx.Init.Mode                = DMA_NORMAL;
    hdma_em_tx.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_em_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_em_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_em_tx.Init.MemBurst            = DMA_MBURST_INC4;
    hdma_em_tx.Init.PeriphBurst         = DMA_PBURST_INC4;

    HAL_DMA_Init(&hdma_em_tx);

    /* Associate the initialized DMA handle to the UART handle */
    __HAL_LINKDMA(huart, hdmatx, hdma_em_tx);

    /* Configure the DMA handler for reception process */
    hdma_em_rx.Instance                 = UART_EM_RX_DMA_STREAM;
    hdma_em_rx.Init.Channel             = UART_EM_RX_DMA_CHANNEL;
    hdma_em_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_em_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_em_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_em_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_em_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_em_rx.Init.Mode                = DMA_NORMAL;
    hdma_em_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_em_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_em_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_em_rx.Init.MemBurst            = DMA_MBURST_INC4;
    hdma_em_rx.Init.PeriphBurst         = DMA_PBURST_INC4;

    HAL_DMA_Init(&hdma_em_rx);

    /* Associate the initialized DMA handle to the the UART handle */
    __HAL_LINKDMA(huart, hdmarx, hdma_em_rx);

    /*## Configure the NVIC for DMA #########################################*/
    /* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
    HAL_NVIC_SetPriority(UART_EM_DMA_TX_IRQn, 10, 10);
    HAL_NVIC_EnableIRQ(UART_EM_DMA_TX_IRQn);

    /* NVIC configuration for DMA transfer complete interrupt (USARTx_RX) */
    HAL_NVIC_SetPriority(UART_EM_DMA_RX_IRQn, 10, 10);
    HAL_NVIC_EnableIRQ(UART_EM_DMA_RX_IRQn);

    /* NVIC configuration for USART, to catch the TX complete and the RX line idle (Rx complete)*/
    HAL_NVIC_SetPriority(UART_EM_IRQn, 10, 10);
    HAL_NVIC_EnableIRQ(UART_EM_IRQn);

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(huart->Instance==UART_SBC)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */

  /* USER CODE END UART5_MspInit 0 */
    /* Peripheral clock enable */
    UART_SBC_CLK_ENABLE();
  
    /* Enable DMA clock */
    DMAx_SBC_CLK_ENABLE();
  
    /* Port clock enable */
    UART_SBC_RX_GPIO_CLK_ENABLE();
    /**UART7 GPIO Configuration    
    PD2      ------> UART5_RX
    PC12     ------> UART5_TX 
    */
    GPIO_InitStruct.Pin = UART_SBC_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = UART_SBC_RX_AF;
    HAL_GPIO_Init(UART_SBC_TX_GPIO_PORT, &GPIO_InitStruct);
 
    GPIO_InitStruct.Pin = UART_SBC_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = UART_SBC_RX_AF;
    HAL_GPIO_Init(UART_SBC_RX_GPIO_PORT, &GPIO_InitStruct);

    /* USER CODE BEGIN UART5_MspInit 1 */
    /* USART7 interrupt Init */
    /*## Configure the DMA ##################################################*/
    /* Configure the DMA handler for Transmission process */
    hdmaSbc_tx.Instance                 = UART_SBC_TX_DMA_STREAM;
    hdmaSbc_tx.Init.Channel             = UART_SBC_TX_DMA_CHANNEL;
    hdmaSbc_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdmaSbc_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdmaSbc_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdmaSbc_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdmaSbc_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdmaSbc_tx.Init.Mode                = DMA_NORMAL;
    hdmaSbc_tx.Init.Priority            = DMA_PRIORITY_LOW;
    hdmaSbc_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdmaSbc_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdmaSbc_tx.Init.MemBurst            = DMA_MBURST_INC4;
    hdmaSbc_tx.Init.PeriphBurst         = DMA_PBURST_INC4;

    HAL_DMA_Init(&hdmaSbc_tx);

    /* Associate the initialized DMA handle to the UART handle */
    __HAL_LINKDMA(huart, hdmatx, hdmaSbc_tx);

    /* Configure the DMA handler for reception process */
    hdmaSbc_rx.Instance                 = UART_SBC_RX_DMA_STREAM;
    hdmaSbc_rx.Init.Channel             = UART_SBC_RX_DMA_CHANNEL;
    hdmaSbc_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdmaSbc_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdmaSbc_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdmaSbc_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdmaSbc_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdmaSbc_rx.Init.Mode                = DMA_NORMAL;
    hdmaSbc_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdmaSbc_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdmaSbc_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdmaSbc_rx.Init.MemBurst            = DMA_MBURST_INC4;
    hdmaSbc_rx.Init.PeriphBurst         = DMA_PBURST_INC4;

    HAL_DMA_Init(&hdmaSbc_rx);

    /* Associate the initialized DMA handle to the the UART handle */
    __HAL_LINKDMA(huart, hdmarx, hdmaSbc_rx);

    /*## Configure the NVIC for DMA #########################################*/
    /* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
    HAL_NVIC_SetPriority(UART_SBC_DMA_TX_IRQn, 10, 10);
    HAL_NVIC_EnableIRQ(UART_SBC_DMA_TX_IRQn);

    /* NVIC configuration for DMA transfer complete interrupt (USARTx_RX) */
    HAL_NVIC_SetPriority(UART_SBC_DMA_RX_IRQn, 6, 10);
    HAL_NVIC_EnableIRQ(UART_SBC_DMA_RX_IRQn);

    /* NVIC configuration for USART, to catch the TX complete and the RX line idle (Rx complete)*/
    if (getFastBridge() == ENABLED)
    {
      HAL_NVIC_SetPriority(UART_SBC_IRQn, 2, 10);
    }
    else
    {
      HAL_NVIC_SetPriority(UART_SBC_IRQn, 10, 10);
    }
    HAL_NVIC_EnableIRQ(UART_SBC_IRQn);

    /* USER CODE END UART7_MspInit 1 */
  }


}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==UART_DBG)
  {
  /* USER CODE BEGIN UART7_MspDeInit 0 */

  /* USER CODE END UART7_MspDeInit 0 */
    /* Peripheral clock disable */
    UART_DBB_CLK_DISABLE();
  
    /**UART7 GPIO Configuration    
    PE7     ------> UART7_RX
    PE8     ------> UART7_TX 
    */
    HAL_GPIO_DeInit(UART_DBG_TX_GPIO_PORT, UART_DBG_RX_PIN|UART_DBG_TX_PIN);

  /* USER CODE BEGIN UART7_MspDeInit 1 */

  /* USER CODE END UART7_MspDeInit 1 */
  }
  else if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    PA12     ------> USART1_DE 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_12);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PD4     ------> USART2_DE
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(huart->Instance == UART5)
  {
  /* USER CODE BEGIN USART5_MspDeInit 0 */

  /* USER CODE END USART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();
  

    /**** USART5 GPIO        Configuration 
         PD2      ------> UART5_RX
         PC12     ------> UART5_TX 
    ***/  

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */
/**
* @brief HAL_RTC_MspInit MSP Initialization
* This function freeze the hardware resources used in this example
* @param RTC_HandleTypeDef: RTC_Base handle pointer
* @retval None
*/
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc, uint32_t clockSource)
{
  RCC_OscInitTypeDef        RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

  if(hrtc->Instance==RTC)
  {
    /*##-1- Enables the PWR Clock and Enables access to the backup domain ###################################*/
    /* To change the source clock of the RTC feature (LSE, LSI), You have to:
   - Enable the power clock using __HAL_RCC_PWR_CLK_ENABLE()
   - Enable write access using HAL_PWR_EnableBkUpAccess() function before to 
     configure the RTC clock source (to be done once after reset).
   - Reset the Back up Domain using __HAL_RCC_BACKUPRESET_FORCE() and 
     __HAL_RCC_BACKUPRESET_RELEASE().
   - Configure the needed RTc clock source */
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

     /*##-2- Configure LSE as RTC clock source ###################################*/
    if (clockSource == RCC_OSCILLATORTYPE_LSI)
    {
      RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI;
      RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
      RCC_OscInitStruct.LSIState = RCC_LSI_ON;
      RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
      PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    }
    else
    {
      RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSE; //RCC_OSCILLATORTYPE_LSI; //RCC_OSCILLATORTYPE_LSE;
      RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
      RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
      RCC_OscInitStruct.LSEState = RCC_LSE_ON; //RCC_LSE_OFF; // RCC_LSE_ON;
      PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    }
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    { 
      Error_Handler();
    }

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    { 
      Error_Handler();
    }
  
    /*##-3- Enable RTC peripheral Clocks #######################################*/
    /* Enable RTC Clock */
    __HAL_RCC_RTC_ENABLE();

    /*##-4- Configure the NVIC for RTC Alarm ###################################*/
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);

  }
}

/**
* @brief HAL_RTC_MspInit MSP DE-Initialization
* This function freeze the hardware resources used in this example
* @param RTC_HandleTypeDef: RTC_Base handle pointer
* @retval None
*/
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{

  if(hrtc->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }

}

#ifdef USE_VOICE
/**** ATTENZIONE: Il DMA del DAC della voce va in conflitto con UART2 usata nell'EM. Per adesso lasciamo perdere */

/**
  * @brief DAC MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param hdac: DAC handle pointer
  * @retval None
  */
void HAL_DAC_MspInit(DAC_HandleTypeDef *hdac)
{
  GPIO_InitTypeDef          GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock ****************************************/
  DACx_CHANNEL_GPIO_CLK_ENABLE();
  /* DAC Periph clock enable */
  DACx_CLK_ENABLE();
  /* DMA1 clock enable */
  DMAx_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* DAC Channel1 GPIO pin configuration */
  GPIO_InitStruct.Pin = DACx_CHANNEL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DACx_CHANNEL_GPIO_PORT, &GPIO_InitStruct);

  /**** DAC for voice generation **********************************************/
  /*##-1- Enable peripherals and GPIO Clocks #################################*/

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* DAC Channel1 GPIO pin configuration */
  GPIO_InitStruct.Pin = DACx_VOICE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DACx_VOICE_GPIO_PORT, &GPIO_InitStruct);

  /*##-3- Configure the DMA ##########################################*/
  /* Set the parameters to be configured for DACx_DMA_STREAM */
  hdma_dac1.Instance = DACx_DMA_INSTANCE;

  hdma_dac1.Init.Channel  = DACx_DMA_VOICE;

  hdma_dac1.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_dac1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_dac1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_dac1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_dac1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_dac1.Init.Mode = DMA_NORMAL; //DMA_CIRCULAR;
  hdma_dac1.Init.Priority = DMA_PRIORITY_HIGH;

  HAL_DMA_Init(&hdma_dac1);

  /* Associate the initialized DMA handle to the DAC handle */
  __HAL_LINKDMA(hdac, DMA_Handle2, hdma_dac1);

  /*##-4- Configure the NVIC for DMA #########################################*/
  /* Enable the DMA1_Stream6 IRQ Channel */
  HAL_NVIC_SetPriority(DACx_DMA_IRQn, 12, 12);
  HAL_NVIC_EnableIRQ(DACx_DMA_IRQn);
}

/**
  * @brief  DeInitializes the DAC MSP.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
void HAL_DAC_MspDeInit(DAC_HandleTypeDef *hdac)
{
  /*##-1- Reset peripherals ##################################################*/
  DACx_FORCE_RESET();
  DACx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* De-initialize the DAC Channel1 GPIO pin */
  HAL_GPIO_DeInit(DACx_CHANNEL_GPIO_PORT, DACx_CHANNEL_PIN);
}
#else
/**
* @brief DAC MSP Initialization
* This function configures the hardware resources used in this example
* @param hdac: DAC handle pointer
* @retval None
*/
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hdac->Instance==DAC)
  {
  /* USER CODE BEGIN DAC_MspInit 0 */

  /* USER CODE END DAC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_DAC_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**DAC GPIO Configuration    
    PA4     ------> DAC_OUT1
    PA5     ------> DAC_OUT2 
    */
    GPIO_InitStruct.Pin = LCD_V0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN DAC_MspInit 1 */

  /* USER CODE END DAC_MspInit 1 */
  }

}

/**
* @brief DAC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hdac: DAC handle pointer
* @retval None
*/
void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac)
{
  if(hdac->Instance==DAC)
  {
  /* USER CODE BEGIN DAC_MspDeInit 0 */

  /* USER CODE END DAC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DAC_CLK_DISABLE();
  
    /**DAC GPIO Configuration    
    PA4     ------> DAC_OUT1
    PA5     ------> DAC_OUT2 
    */
    HAL_GPIO_DeInit(GPIOA, LCD_V0_Pin);

  /* USER CODE BEGIN DAC_MspDeInit 1 */

  /* USER CODE END DAC_MspDeInit 1 */
  }

}
#endif

/**
* @brief I2C MSP Initialization
* This function configures the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspInit 0 */

  /* USER CODE END I2C3_MspInit 0 */
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**I2C3 GPIO Configuration    
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();
  /* USER CODE BEGIN I2C3_MspInit 1 */

  /* USER CODE END I2C3_MspInit 1 */
  }
  else
  {
    if(hi2c->Instance==I2CSMBx)
    {
    /* USER CODE BEGIN I2C1_MspInit 0 */

    /* USER CODE END I2C1_MspInit 0 */

      SMBx_RCC_GPIOx_CLK_ENABLE();
      /**I2C3 GPIO Configuration    
      PB7     ------> I2C1_SDA
      PB8     ------> I2C1_SCL 
      */
      GPIO_InitStruct.Pin = SMB0_SDA_Pin;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = SMBx_GPIO_AF;
      HAL_GPIO_Init(SMB0_SDA_GPIO_Port, &GPIO_InitStruct);

      GPIO_InitStruct.Pin = SMB0_SCL_Pin;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = SMBx_GPIO_AF;
      HAL_GPIO_Init(SMB0_SCL_GPIO_Port, &GPIO_InitStruct);

      /* Peripheral clock enable */
      __HAL_RCC_I2C1_CLK_ENABLE();
    /* USER CODE BEGIN I2C3_MspInit 1 */

    /* USER CODE END I2C3_MspInit 1 */
    }

  }
}

/**
* @brief I2C MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspDeInit 0 */

  /* USER CODE END I2C3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C3_CLK_DISABLE();
  
    /**I2C3 GPIO Configuration    
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

  /* USER CODE BEGIN I2C3_MspDeInit 1 */

  /* USER CODE END I2C3_MspDeInit 1 */
  }

}

/**
  * @brief ADC MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef          GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock ****************************************/
  ADCx_CHANNEL_GPIO_CLK_ENABLE();
  /* ADC1-ADC3 Periph clock enable */
  ADCx_CLK_ENABLE();
  ADCxTA_CLK_ENABLE();
  ADCxVIN_CLK_ENABLE();
  ADCxCP_CLK_ENABLE();

  /* Enable DMA2 clock */
  DMAx_ADC_CLK_ENABLE(); 
    
  /*##-2- Configure peripheral GPIO ##########################################*/ 
  /* ADC1 Channel 0 GPIO pin configuration CP */
  GPIO_InitStruct.Pin = CP_ADC_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CP_ADC_GPIO_PORT, &GPIO_InitStruct);

  /* ADC1 Channel 8 GPIO pin configuration PP_ADC */
  GPIO_InitStruct.Pin = PP_ADC_PIN;
  HAL_GPIO_Init(PP_ADC_GPIO_PORT, &GPIO_InitStruct);

  /* ADC1 Channel 12 GPIO pin configuration SW_ADC (Ampere current selection) */
  GPIO_InitStruct.Pin = SW_ADC_PIN;
  HAL_GPIO_Init(SW_ADC_GPIO_PORT, &GPIO_InitStruct);

  /* ADC1 Channel 10 GPIO pin configuration TA1_ADC  */
  GPIO_InitStruct.Pin = TA1_PIN;
  HAL_GPIO_Init(TA1_GPIO_PORT, &GPIO_InitStruct);

  /* ADC1 Channel 13 GPIO pin configuration TEMP_ADC */
  GPIO_InitStruct.Pin = TEMP_PIN;
  HAL_GPIO_Init(TEMP_GPIO_PORT, &GPIO_InitStruct);

  /* ADC1 Channel 3 GPIO pin configuration IM_ADC */
  GPIO_InitStruct.Pin = IM_ADC_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IM_ADC_GPIO_PORT, &GPIO_InitStruct);

  /* ADC1 Channel 9 GPIO pin configuration VIN_ADC */
  GPIO_InitStruct.Pin = VIN_PIN;
  HAL_GPIO_Init(VIN_GPIO_PORT, &GPIO_InitStruct);

  /*** initialization DMA for ADC 1  ******************************************/
  if (hadc->Instance == ADCx)
  {
    if (hadc->Init.ExternalTrigConvEdge != ADC_EXTERNALTRIGCONVEDGE_NONE)
    {
      /*##-3- Configure the DMA streams ##########################################*/
      /* Set the parameters to be configured */
      hdma_adc.Instance = ADCx_DMA_STREAM; 

      hdma_adc.Init.Channel  = ADCx_DMA_CHANNEL;
      hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
      hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
      hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
      hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; //DMA_PDATAALIGN_WORD;
      hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD; //DMA_MDATAALIGN_WORD;
      hdma_adc.Init.Mode = DMA_CIRCULAR;
      hdma_adc.Init.Priority = DMA_PRIORITY_HIGH;
      hdma_adc.Init.FIFOMode = DMA_FIFOMODE_DISABLE;         
      hdma_adc.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
      hdma_adc.Init.MemBurst = DMA_MBURST_SINGLE;
      hdma_adc.Init.PeriphBurst = DMA_PBURST_SINGLE; 

      HAL_DMA_Init(&hdma_adc);
        
      /* Associate the initialized DMA handle to the the ADC handle */
      __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc);

      /*##-4- Configure the NVIC for DMA #########################################*/
      /* NVIC configuration for DMA transfer complete interrupt */
      HAL_NVIC_SetPriority(ADCx_DMA_IRQn, 12, 12);   
      HAL_NVIC_EnableIRQ(ADCx_DMA_IRQn);
    }
  }
  else
  {
    if (hadc->Instance == ADCxTA)
    {
      /*** initialization DMA for ADC 3 (for TA only)  ************************************/

      /*##-3- Configure the DMA streams ##########################################*/
      /* Set the parameters to be configured */
      hdma_adcTA.Instance = ADCxTA_DMA_STREAM; 

      hdma_adcTA.Init.Channel  = ADCxTA_DMA_CHANNEL;
      hdma_adcTA.Init.Direction = DMA_PERIPH_TO_MEMORY;
      hdma_adcTA.Init.PeriphInc = DMA_PINC_DISABLE;
      hdma_adcTA.Init.MemInc = DMA_MINC_ENABLE;
      hdma_adcTA.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; //DMA_PDATAALIGN_WORD;
      hdma_adcTA.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD; //DMA_MDATAALIGN_WORD;
      hdma_adcTA.Init.Mode = DMA_CIRCULAR;
      hdma_adcTA.Init.Priority = DMA_PRIORITY_HIGH;
      hdma_adcTA.Init.FIFOMode = DMA_FIFOMODE_DISABLE;         
      hdma_adcTA.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
      hdma_adcTA.Init.MemBurst = DMA_MBURST_SINGLE;
      hdma_adcTA.Init.PeriphBurst = DMA_PBURST_SINGLE; 

      HAL_DMA_Init(&hdma_adcTA);
        
      /* Associate the initialized DMA handle to the the ADC handle */
      __HAL_LINKDMA(hadc, DMA_Handle, hdma_adcTA);

      /*##-4- Configure the NVIC for DMA #########################################*/
      /* NVIC configuration for DMA transfer complete interrupt */
      HAL_NVIC_SetPriority(ADCxTA_DMA_IRQn, 12, 12);   
      HAL_NVIC_EnableIRQ(ADCxTA_DMA_IRQn);
    }
    else
    {
      if (hadc->Instance == ADC2)
      {
        if (hadc->Init.ExternalTrigConv == ADC_EXTERNALTRIGCONV_TxVIN_TRGO)
        {
          /*##-4- Configure the NVIC for ADC2=ADCxVIN  VIN #########################################*/
          /* NVIC configuration for DMA transfer complete interrupt */
          HAL_NVIC_SetPriority(ADC_IRQn, 12, 12);   
          HAL_NVIC_EnableIRQ(ADC_IRQn);
        }
        else
        {
          if (hadc->Init.ExternalTrigConv != ADC_EXTERNALTRIGCONV_T1_CC1)
          {
            /*** initialization DMA for ADC 2 (for CP only)  ************************************/

            /*##-3- Configure the DMA streams ##########################################*/
            /* Set the parameters to be configured */
            hdma_adcCP.Instance = ADCxCP_DMA_STREAM; 

            hdma_adcCP.Init.Channel  = ADCxCP_DMA_CHANNEL;
            hdma_adcCP.Init.Direction = DMA_PERIPH_TO_MEMORY;
            hdma_adcCP.Init.PeriphInc = DMA_PINC_DISABLE;
            hdma_adcCP.Init.MemInc = DMA_MINC_ENABLE;
            hdma_adcCP.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; //DMA_PDATAALIGN_WORD;
            hdma_adcCP.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD; //DMA_MDATAALIGN_WORD;
            hdma_adcCP.Init.Mode = DMA_CIRCULAR;
            hdma_adcCP.Init.Priority = DMA_PRIORITY_HIGH;
            hdma_adcCP.Init.FIFOMode = DMA_FIFOMODE_DISABLE;         
            hdma_adcCP.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
            hdma_adcCP.Init.MemBurst = DMA_MBURST_SINGLE;
            hdma_adcCP.Init.PeriphBurst = DMA_PBURST_SINGLE; 

            HAL_DMA_Init(&hdma_adcCP);

            /* Associate the initialized DMA handle to the the ADC handle */
            __HAL_LINKDMA(hadc, DMA_Handle, hdma_adcCP);

            /*##-4- Configure the NVIC for DMA2 stream2 ch1  #########################################*/
            /* NVIC configuration for DMA transfer complete interrupt */
            HAL_NVIC_SetPriority(ADCxCP_DMA_IRQn, 12, 12);   
            HAL_NVIC_EnableIRQ(ADCxCP_DMA_IRQn);
          }
        }
      }
    }
  }
}
  
/**
  * @brief ADC MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO to their default state
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  
  /*##-1- Reset peripherals ##################################################*/
  ADCx_FORCE_RESET();
  ADCx_RELEASE_RESET();

  HAL_GPIO_DeInit(CP_ADC_GPIO_PORT, CP_ADC_PIN);
  HAL_GPIO_DeInit(PP_ADC_GPIO_PORT, PP_ADC_PIN);
  HAL_GPIO_DeInit(SW_ADC_GPIO_PORT, SW_ADC_PIN);
  HAL_GPIO_DeInit(TA1_GPIO_PORT,    TA1_PIN);
  HAL_GPIO_DeInit(TEMP_GPIO_PORT,   TEMP_PIN);
 
  /*##-3- Disable the DMA Streams ############################################*/
  /* De-Initialize the DMA Stream associate to transmission process */
  if (hdma_adc.Instance != NULL)
  {
    HAL_DMA_DeInit(&hdma_adc);  
  }
    
  /*##-4- Disable the NVIC for DMA ###########################################*/
  HAL_NVIC_DisableIRQ(ADCx_DMA_IRQn);
}



/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

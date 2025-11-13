/**
* @file        stm32f7xx_periph_init.c
*
* @brief       Peripheral initialization for stm32f7xx  - Implementation -
*
* @riskClass   C 
*
* @moduleID  
*
* @vcsInfo
*     $Id: stm32f7xx_periph_init.c
*
*     $Revision: 
*
*     $Date: 
*
*
* @copyright
*       Copyright (C) 2020 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

/************************************************************
 * Include
 ************************************************************/
   
#include "main.h" 
#include "stm32f7xx_periph_init.h"

/*
***********************************SCAME**************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/

/*
***********************************SCAME**************************************
**                                                                          **
**                            Local Const                                   **
**                                                                          **
****************************************************************************** 
*/ 

/*
***********************************SCAME**************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 


DAC_HandleTypeDef hdac;
QSPI_HandleTypeDef hqspi;
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart5;
IWDG_HandleTypeDef hiwdg;
I2C_HandleTypeDef  hi2c3;
I2C_HandleTypeDef  hSmb0;

static TIM_HandleTypeDef  htim6;

/*
***********************************SCAME**************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 


/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/

extern uint32_t calpValue, calmValue;

/*
***********************************SCAME**************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DAC_Init(void);
void MX_QUADSPI_Init(void);
void MX_RTC_Init(uint32_t clockSource);
void MX_TIM1_Init(void);
void MX_UART7_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART1_UART_Init(void);
void MX_I2C3_Init(void);
void StopMode_Measure(void);
void TIM6_Config(void);
 
/*
*********************************** SCAME ************************************
**                                                                          **
**                         External Function prototype                      **
**                                                                          **
******************************************************************************
*/

extern void Error_Handler(void);
extern void _Error_Handler(char * file, int line);
extern void HAL_TIM_MspPostInit   (TIM_HandleTypeDef *htim);
extern void setNewRtcCal (uint32_t calp, uint32_t calm);
extern void i2c1_Handler(char *file, int line);

/*
***********************************SCAME**************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/

 
/**
  * @brief  The application entry point.
  * @retval int
  */  
 


/*********************************************************************************************************************/
/************************  Peripheral initialization for STM32F7xx device ********************************************/
/*********************************************************************************************************************/

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */
  HAL_DAC_DeInit(&hdac);
  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config 
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief Internal watch dog Init
  * @param None
  * @retval None
  */
#ifdef ACTIVE_IWDG
void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  __HAL_DBGMCU_FREEZE_IWDG();
}
#endif

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  //hqspi.Init.ClockPrescaler = 4;   // QSPIck = AHBck / (ClockPrescaler + 1) If AHBCLKDivider = RCC_SYSCLK_DIV1 --> 216 /(4 + 1) = 43,2Mhz <<  max possible (104MHz)
  hqspi.Init.ClockPrescaler = 8;     // QSPIck = AHBck / (ClockPrescaler + 1) If AHBCLKDivider = RCC_SYSCLK_DIV1 --> 216 /(8 + 1) = 24Mhz <<  max possible (104MHz)
  hqspi.Init.FifoThreshold  = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 19;  /* we have W25Q80 = 1MB = 0x0.0000 - F.FFFF 19+1=20 address bit */
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
#ifndef COLLAUDO_PEN  
  HAL_GPIO_WritePin(GPIOE, LCD_D0_Pin|LCD_D1_Pin|LCD_D2_Pin|LCD_D3_Pin 
                          |LCD_RS_Pin|LCD_CS_Pin, GPIO_PIN_RESET);
#else
  HAL_GPIO_WritePin(GPIOE, LCD_D0_Pin|LCD_D1_Pin|LCD_D2_Pin|LCD_D3_Pin 
                          |LCD_RS_Pin|LCD_CS_Pin
                          |LEDA_T1CH1_Pin | LEDB_T1CH2_Pin | LEDC_T1CH3_Pin, GPIO_PIN_RESET);
#endif

  /*Configure GPIO pins : LCD_D0_Pin LCD_D1_Pin LCD_D2_Pin LCD_D3_Pin 
                           LCD_RS_Pin LCD_CS_Pin */
#ifndef COLLAUDO_PEN						   
  GPIO_InitStruct.Pin = LCD_D0_Pin|LCD_D1_Pin|LCD_D2_Pin|LCD_D3_Pin 
                          |LCD_RS_Pin|LCD_CS_Pin;
#else
  GPIO_InitStruct.Pin = LCD_D0_Pin|LCD_D1_Pin|LCD_D2_Pin|LCD_D3_Pin 
                          |LCD_RS_Pin|LCD_CS_Pin | LEDA_T1CH1_Pin | LEDB_T1CH2_Pin | LEDC_T1CH3_Pin;
#endif						  
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : DIODO_CHECK_Pin */
  GPIO_InitStruct.Pin = DIODO_ADC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIODO_ADC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RCDM_Pin */
  GPIO_InitStruct.Pin = RCDM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RCDM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN2_Pin  */
  GPIO_InitStruct.Pin = IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SBC_CONN_Pin */
  GPIO_InitStruct.Pin = SBC_CONN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN; //GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /*Configure GPIO pins : IN6_Pin IN3_Pin IN4_Pin VAC230_DET_Pin */
  GPIO_InitStruct.Pin = IN6_Pin|BT_IRQ_Pin|VAC230_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWRDWN1L_GPIO_Port, PWRDWN1L_Pin, GPIO_PIN_SET);
  /*Configure GPIO pins : PWRDWN1L_Pin */
  GPIO_InitStruct.Pin = PWRDWN1L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWRDWN1L_GPIO_Port, &GPIO_InitStruct);


  /*Configure GPIO pins : IN5_Pin  */
  GPIO_InitStruct.Pin = IN5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOD, OUTBL1_M_Pin|OUTBL1_P_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, OUTBL1_M_Pin|OUTBL1_P_Pin|CNTCT_Pin|SGCBOB_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pins : OUTBL1_M_Pin OUTBL1_P_Pin CNTCT_Pin*/
  GPIO_InitStruct.Pin = OUTBL1_M_Pin|OUTBL1_P_Pin|CNTCT_Pin|SGCBOB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RW_Pin INT_INx_Pin */
  GPIO_InitStruct.Pin = LCD_RW_Pin | INT_INx_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure Heart Led Port  */
  HAL_GPIO_WritePin(H_LED_GPIO_Port, H_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD2_Pin */
  GPIO_InitStruct.Pin = H_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(H_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWM_CP_GPIO_Port, PWM_CP_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pins : PWM_CP_Pin  */
  GPIO_InitStruct.Pin = PWM_CP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWM_CP_GPIO_Port, &GPIO_InitStruct);


  /*CANTX and CANRX used as debug pin   */
  HAL_GPIO_WritePin(CAN1_TX_GPIO_Port, CAN1_TX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CAN1_TX_Pin PD1 */
  GPIO_InitStruct.Pin = CAN1_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN1_TX_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(CAN1_RX_GPIO_Port, CAN1_RX_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pins : CAN1_RX_Pin PA11 */
  GPIO_InitStruct.Pin = CAN1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN1_RX_GPIO_Port, &GPIO_InitStruct);

  /* SINAPSI is active LOW: default is "1" --> NO sinapsi */
  GPIO_InitStruct.Pin = SINAPSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SINAPSI_GPIO_Port, &GPIO_InitStruct);
  
  /*********** WIRELESS AREA *****************************************/
  
//  if (chipWireless == CHIP_EXPRESSIF)
//  {
  /*Configure GPIO pin Output Level */
  // NOT PROVIDED for STM32 micro --> HAL_GPIO_WritePin(WIFI_EN_GPIO_Port, WIFI_EN_Pin, GPIO_PIN_SET);

  // NOT PROVIDED for STM32 micro --> /*Configure GPIO pin : WIFI_EN_Pin */
  // NOT PROVIDED for STM32 micro --> GPIO_InitStruct.Pin = WIFI_EN_Pin;
  // NOT PROVIDED for STM32 micro --> GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  // NOT PROVIDED for STM32 micro --> GPIO_InitStruct.Pull = GPIO_NOPULL;
  // NOT PROVIDED for STM32 micro --> HAL_GPIO_Init(WIFI_EN_GPIO_Port, &GPIO_InitStruct);

  /*********** WIFI /DBG  AREA *****************************************/

  /*Configure GPIO pin Output Level for DBG --> PC use */
  HAL_GPIO_WritePin(WIFI_NSS2_GPIO_Port, WIFI_NSS2_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pins : BT_RST_Pin */
  GPIO_InitStruct.Pin = WIFI_NSS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  //GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WIFI_NSS2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  //HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  /*** EXT Int on IRQ pin ****/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 10, 10);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief I2C1 init function
  * @param None
  * @retval None
  */
void MX_I2C1_Init(void)
{
hSmb0.Instance = I2C1;
hSmb0.Init.Timing = 0x20404768;
hSmb0.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
hSmb0.Init.OwnAddress1 = 0;
hSmb0.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
hSmb0.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
hSmb0.Init.OwnAddress2 = 0;
hSmb0.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
hSmb0.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

if (HAL_I2C_Init(&hSmb0) != HAL_OK)
    {
    i2c1_Handler(__FILE__, __LINE__);
    }

  /* Configure Analogue filter */
if (HAL_I2CEx_ConfigAnalogFilter(&hSmb0, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
    i2c1_Handler(__FILE__, __LINE__);
    }

  /*Configure Digital filter */
if (HAL_I2CEx_ConfigDigitalFilter(&hSmb0, 0) != HAL_OK)
    {
    i2c1_Handler(__FILE__, __LINE__);
    }
}

/**
  * @brief I2C3 init function
  * @param None
  * @retval None
  */
void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x20406868; // 0x20404848; //0x20406868;  --> 6868 genera un po' meno di 100KHz il che mi garantisce da sorprese... credo
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
void MX_RTC_Init(uint32_t clockSource)
{

    /**Initialize RTC Only
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
  hrtc.Init.SynchPrediv = RTC_SYNCH_PREDIV;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc, clockSource) != HAL_OK)
  {
    Error_Handler();
  }
  setNewRtcCal (calpValue, calmValue);
}

/***************************** END OF FILE ************************************/
/**
* @file        adcTask.c
*
* @brief       Software to manage ADC samples  - Implementation
*
* @author      Nicolino Piergiovanni, Clusone
*
* @riskClass   C
*
* @moduleID    SAD_DRV_130
*
* @vcsInfo
*     $Id: adcTask1.c 76 2022-06-20 09:46:05Z npiergi $
*
*     $Revision: 76 $
*
*     $Author: npiergi $
*
*     $Date: 2022-06-20 11:46:05 +0200 (lun, 20 giu 2022) $
*
*
* @copyright
*       Copyright (C) 2017 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

/****************** INCLUDES ******************************************************************************************/
#include <string.h>
#include <intrinsics.h>
#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <math.h>
#include "main.h"
#include "adcTask.h"
#include "cmsis_os.h"
#include "sbcGsy.h"

#define ADC_MAX_MESSAGE_NUM 2


/****************** LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM ************************************************************/
typedef  struct
{
  uint8_t     stato;
  uint8_t     codCom;
  uint8_t     iADC;
  uint8_t     idMao;
  uint8_t     valid;
  uint8_t     validTA;
  uint8_t     activeTAMeasure;
} adcTask_t;

/****************** LOCAL VARIABLES ***********************************************************************************/

static adcTask_t          adcTask = {ADC_IDLE, 0, 0, 0, 0};

const GPIO_TypeDef* ADC_GPIO_PORT[ADCn] = {CP_ADC_GPIO_PORT,
                                           PP_ADC_GPIO_PORT,
                                           SW_ADC_GPIO_PORT,
                                           TA1_GPIO_PORT,
                                           TEMP_GPIO_PORT};

const uint16_t ADC_GPIO_PIN[ADCn] = {CP_ADC_PIN,
                                     PP_ADC_PIN,
                                     SW_ADC_PIN,   
                                     TA1_PIN,  
                                     TEMP_PIN};


adcMsg_t         adcConvReady;
static adcQueueMsg_t    adcQueueReady;

/****************** LOCAL CONST  ***********************************************************************************/
#ifdef NUCLEO_F746
                                               /* position     0               1              2              3                4  */
static const uint16_t rotaryLimits[NUM_ROTARY_POS][2] = {{3100U, 3300}, {2300U, 2420U}, {1780U, 1920}, {1480U, 1650U}, {800U, 870U},
                                               /* position     5               6            7              8             9 */
                                                         {730U, 790U}, {680U, 725U}, {620U, 680U}, {1280U, 1400U}, {1100U, 1200U}};
#else
                                               /* position     0               1              2              3                4  */
static const uint16_t rotaryLimits[NUM_ROTARY_POS][2] = {{3100U, 3300}, {2480U, 2640U}, {2080U, 2180}, {1750U, 1850U}, {990U, 1090U},
                                               /* position     5               6            7              8             9 */
                                                         {920U, 989U}, {840U, 919U}, {710U, 839U}, {1480U, 1600U}, {1320U, 1410U}};
#endif


/****************** GLOBAL VARIABLES *******************************************************************************/
xQueueHandle adcQueue = NULL;
/* ADC handler declaration */
ADC_HandleTypeDef    AdcHandle, AdcTAHandle, AdcVINHandle, AdcCPHandle;
/* TIM handler declaration */
TIM_HandleTypeDef  htimAdc, htimTAAdc, htimVINadc, htimCPAdc;
/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef      sConfig;

  
/* Variable used to get converted value */
uint16_t    uhADCxConvertedValue[NUM_ADC_SCU] = {0, 0, 0, 0, 0, 0};
uint32_t    meanVal[NUM_ADC_SCU];
uint32_t    accVal[NUM_ADC_SCU];
int16_t     taSamplesValue[NUM_TA_SAMPLES];
uint16_t    rmsSample[NUM_TA_SAMPLES_MEAN];
uint32_t    rmsVal, gainN, gainD, rmsValueMean;
uint8_t     cnt, ix, dataValid;
uint16_t    cpSamplesValue[NUM_CP_SAMPLES];





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
static void     TIM_Config        (void);
static uint32_t adcGest           (adcQueueMsg_t* msgRcv);
static void     adcInitConfig     (void);
static void     adcInitVar		    (void);
static void     ADC_Config        (void);
static void     AdcError_Handler  (void); 

static void     adcInitTAVar      (void);
static void     adcInitTAConfig   (void);
static void     ADC_TA_Config     (void);
static void     TIM_TA_Config     (void);

static void     ADC_VIN_Config    (void);
static void     TIM_VIN_Config    (void);
static void     startVINconversion(void);

static uint8_t  taConfigComplete  (void);
static uint32_t rmsEstimation     (void);
static void     startTAconversion (void);
static void     stopTAconversion  (void);

static void     adcInitCPConfig   (void);
static void     stopCPconversion  (void);
static void     startCPconversion (void);
static void     startSamplingTA   (void);

// **************************************************

// **************************************************
void adcGestTask(void* pvParameters)
{
  uint32_t       timeTick;
  /*-----------------------------------------------------------------------------------*/
  //  Creates an empty mailbox for adc messages
  adcQueue = xQueueCreate(ADC_MAX_MESSAGE_NUM, sizeof(adcQueueMsg_t));
  configASSERT(adcQueue != NULL);

  // Start message.
  adcQueueReady.adcEv = EV_ADC_START;
  adcConvReady.meanData[0][0]  = (uint16_t)0;
  configASSERT(xQueueSendToBack(adcQueue, (void *)&adcQueueReady, portMAX_DELAY) == pdPASS);

  timeTick = portMAX_DELAY;
  for (;;)
  {
    // Wait for some event from Rx uart debug.
    if (xQueueReceive(adcQueue, (void *)&adcQueueReady, timeTick) == pdPASS)
    {
      timeTick = adcGest(&adcQueueReady);
    }
    else
    {
      adcQueueReady.adcEv = EV_ADC_TA_POLLING;
      timeTick = adcGest(&adcQueueReady);
    }
  }
}

static uint32_t  adcGest (adcQueueMsg_t* msgRcv)
{
  uint32_t nextTimeTick;
  uint64_t tempRMS;
  uint8_t ix;

  nextTimeTick = portMAX_DELAY;
  /* start processo */
  switch (adcTask.stato)
  {
    case ADC_IDLE:
      switch (msgRcv->adcEv)
      {
        case EV_ADC_START:
          adcInitConfig();
          adcInitVar();
          /* init CP conversion: 32 samples on 2msec every 10ms */
          adcInitCPConfig();
          /* prepare ADC for TA */
          adcInitTAVar();
          adcInitTAConfig();
          break;

        case EV_ADC_TA_START_CONV:
          adcInitTAVar();
          startSamplingTA();
          nextTimeTick = pdMS_TO_TICKS(POLLING_TA_250MS);
          break;

        case EV_ADC_VIN_START_CONV:
          triggerVINConfig();
          break;

        case EV_ADC_TA_POLLING:
          if (taConfigComplete() == TRUE)
          {
            /* it is necessary to find the rms value for this new cycle */
            rmsVal = rmsEstimation();
            rmsVal = (rmsVal * gainN) / gainD;
            /* questo valore va corretto in base al valore calibrato */
            if (cnt >= NUM_TA_SAMPLES_MEAN)
            {
              cnt = 0;
              dataValid = 1;
            }
            rmsSample[cnt] = rmsVal;
            cnt++;
            if (dataValid == 1)
            {
              for (ix = 0, tempRMS = 0; ix < NUM_TA_SAMPLES_MEAN; ix++)
              {
                tempRMS += rmsSample[ix];
              }
              
              if ((gsy_quick_polling_get() & POWERED_OUTLET) == POWERED_OUTLET)
                rmsValueMean = (uint32_t)(tempRMS / NUM_TA_SAMPLES_MEAN);
              else
                rmsValueMean = 0;
            }
            startTAconversion();
          }
          if (adcTask.activeTAMeasure == TRUE)
          {
            nextTimeTick = pdMS_TO_TICKS(POLLING_TA_250MS);
          }
          break;

        case EV_ADC_TA_END_CONV:
          if (dataValid == 1)
          {
            stopTAconversion();
          }
          break;

        default:
          break;
      }

      break;
 
    default:
      break;
  }
  return(nextTimeTick);
}

// *****************************************
static void adcInitConfig () 
{
  /*##-1- TIMx Peripheral Configuration ######################################*/
  TIM_Config();

  /*##-2- Configure the ADC1 peripheral ######################################*/
  ADC_Config();
 
  /*##-3- Start the conversion process and enable interrupt ##################*/ 
  if(HAL_ADC_Start_DMA(&AdcHandle, (uint32_t*)&uhADCxConvertedValue[0], NUM_ADC_SCU) != HAL_OK)
  {
    /* Start Conversation Error */
    AdcError_Handler(); 
  } 

  /*##-4- TIM2 counter enable ################################################*/ 
  if(HAL_TIM_OC_Start_IT(&htimAdc, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Counter Enable Error */
    AdcError_Handler();
  }
}

// *****************************************
static void adcInitVar ()
{
  uint8_t j, i;

  for (i = 0; i < NUM_ADC_SCU; i++)
  {
  	accVal[i] = 0;

    for (j = 0; j < NUM_SAMPLE_FOR_MEAN; j++)
    {
      adcConvReady.meanData[i][j] = 0;
    }
  }
}

/**
*
* @brief        TIMx configuration
*
* @param [out]  none
*
* @retval       none
*
***********************************************************************************************************************/
static void TIM_Config(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
  uint32_t                uwPrescalerValue = 0;

  /* TIM2 clock enable */
  TIM_ADC_CLK_ENABLE();

   
  /*##-1- Configure the TIM peripheral #######################################*/ 
  /* -----------------------------------------------------------------------
    In this example TIM2 input clock (TIM2CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1 (prescale is / 4 see main.c: RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;)  
      TIM2CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM2CLK = HCLK / 2 = SystemCoreClock / 2 
    To get TIM2 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM2CLK / TIM2 counter clock) - 1
    Prescaler = ((SystemCoreClock / 2) /10 KHz) - 1
  
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency  
  ----------------------------------------------------------------------- */  
  
  /* Compute the prescaler value to have TIM2 counter clock equal to 3,6MHz */
  uwPrescalerValue = ((SystemCoreClock / 2) / 3600000) - 1;
  
  /* Set TIMx instance */
  htimAdc.Instance = TIMx;
   
  /* Initialize TIM3 peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = ((SystemCoreClock/2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  //htimAdc.Init.Period = 10000 - 1;
  //htimAdc.Init.Prescaler = uwPrescalerValue;
  htimAdc.Init.Prescaler     = uwPrescalerValue;
//  htimAdc.Init.Period        = PERIOD_VALUE;  // now 72000/3,6MHz = 7,2 10^4 / 3,6 10^6 = 2 * 10^-2 = 20msec
  htimAdc.Init.Period        = PERIOD_VALUE;  // now 36000/3,6MHz = 3,6 10^4 / 3,6 10^6 = 1 * 10^-2 = 10msec
  htimAdc.Init.ClockDivision = 0;             // quindi prendiamo un campione ogni 20msec
  htimAdc.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&htimAdc) != HAL_OK)
  {
    /* Initialization Error */
    AdcError_Handler();
  }
  /* Common configuration for all channels */
  sConfig.OCMode     = TIM_OCMODE_PWM1;;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode = TIM_OCFAST_DISABLE;

  /* Set the pulse value for channel 1 */
  sConfig.Pulse = PULSE1_VALUE;  
  if(HAL_TIM_PWM_ConfigChannel(&htimAdc, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    AdcError_Handler();
  }
   
  /* TIM2 TRGO selection */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    
  if(HAL_TIMEx_MasterConfigSynchronization(&htimAdc, &sMasterConfig) != HAL_OK)
  {
    /* TIM8 TRGO selection Error */
    AdcError_Handler();
  }  
}

/**
*
* @brief        ADC configuration
*
* @param [out]  none
*
* @retval       none
*
*******************************************************************************/
static void ADC_Config(void)
{
  ADC_ChannelConfTypeDef  sConfig;
  
   /* ADC Initialization */
  AdcHandle.Instance                   = ADCx;
 
  AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4;  // PLCK2=108MHz --> ADCLK=27MHz

  AdcHandle.Init.Resolution            = ADC_RESOLUTION; //ADC_RESOLUTION_10B;  

  AdcHandle.Init.ScanConvMode          = ENABLE;
  AdcHandle.Init.ContinuousConvMode    = DISABLE; //ENABLE;
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle.Init.NbrOfDiscConversion   = 0;
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
  AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_Tx_TRGO; //ADC_EXTERNALTRIGCONV_T2_CC1;
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.NbrOfConversion       = NUM_ADC_SCU;
  AdcHandle.Init.DMAContinuousRequests = ENABLE;
  AdcHandle.Init.EOCSelection          = DISABLE;

  if(HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    /* ADC Initialization Error */
    AdcError_Handler();
  }
  
  /* Configure ADC1 regular channel */  
  /* la sequenza deve essere quella riportata nell'enum  adcIn_e */
  sConfig.Channel = VIN_ADC_CHANNEL;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfig.Offset = 0;
  
  if(HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    AdcError_Handler();
  }
 
  sConfig.Channel = PP_ADC_CHANNEL;
  sConfig.Rank = 2;
  if(HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    AdcError_Handler();
  }

  sConfig.Channel = SW_ADC_CHANNEL;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if(HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    AdcError_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 4;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES; // must be > 10us --> 10u * 27M = 270 cycles (480 > 270 OK!!)
  if(HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    AdcError_Handler();
  }

  sConfig.Channel = TEMP_CHANNEL;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfig.Rank = 5;
  if(HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    AdcError_Handler();
  }

  /** ADC for check socket motor driver current  */
  sConfig.Channel = IM_ADC_CHANNEL;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfig.Rank = 6;
  if(HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    AdcError_Handler();
  }
}


/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion, and 
  *         you can add your own implementation.    
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  uint8_t i, j;
  
  if (AdcHandle->Instance == ADCx)
  {
    /* Get the converted value of regular channel */
    //uhADCxConvertedValue[0] = HAL_ADC_GetValue(AdcHandle);
    //HAL_GPIO_TogglePin(CAN1_TX_GPIO_Port, CAN1_TX_Pin);  // trigger for debug 
#ifdef COME_ERA
    for (i = 0; i < NUM_ADC_SCU; i++)
    {
      /* aggiorno con i campioni attuali */
      accVal[i] -= adcConvReady.meanData[i][adcTask.iADC];
      adcConvReady.meanData[i][adcTask.iADC] = uhADCxConvertedValue[i];
      accVal[i] += adcConvReady.meanData[i][adcTask.iADC];
      meanVal[i] = accVal[i] / NUM_SAMPLE_FOR_MEAN;
    }
    adcTask.iADC++; /* posizione del prossimo campione */

    if (adcTask.iADC >= NUM_SAMPLE_FOR_MEAN)
    {
      adcTask.iADC = 0;
      adcTask.valid = 1;
    }
#else
    /* upgrade all the input signals */
    for (i = 0; i < NUM_ADC_SCU; i++)
    {
      /* aggiorno con i campioni attuali */
      adcConvReady.meanData[i][adcTask.iADC] = uhADCxConvertedValue[i];
    }
    adcTask.iADC++; /* posizione del prossimo campione */
    if (adcTask.iADC >= NUM_SAMPLE_FOR_MEAN)
    {
      adcTask.iADC = 0;
      adcTask.valid = 1;
    }
    if (adcTask.valid == 1)
    {
      for (i = 0; i < NUM_ADC_SCU; i++)
      {
        for (j = 0, accVal[i] = 0; j < NUM_SAMPLE_FOR_MEAN; j++)
        {
          accVal[i] += adcConvReady.meanData[i][j];
        }
        /* calcolo la media sugli ultimi  NUM_SAMPLE_FOR_MEAN */
        meanVal[i] = accVal[i] / NUM_SAMPLE_FOR_MEAN;
      }
    }
#endif
    /* every 10ms restart sampling at 62.5us on CP signal */
    startCPconversion();
    //HAL_GPIO_WritePin(CAN1_TX_GPIO_Port, CAN1_TX_Pin, GPIO_PIN_SET);  // Pin for debug High
  }
  else
  {
    if (AdcHandle->Instance == ADCxTA)
    {
      /* stop the ADC3: */
      (void)HAL_ADC_Stop_DMA(&AdcTAHandle);
      /* stop the timer: on TA signal no more ADC conversion is need */
      (void)HAL_TIM_Base_Stop(&htimTAAdc);
      /* we have the pool samples, so the RMS calcutation can start */
      adcTask.validTA = TRUE;
    }
    else
    {
//      HAL_GPIO_WritePin(CAN1_TX_GPIO_Port, CAN1_TX_Pin, GPIO_PIN_RESET);  // Pin for debug Low
      /* stop 2ms sampling at 62.5us on CP signal: wait for next tick (10ms) */
      stopCPconversion();
    }
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  ;
}

/**
*
* @brief        Send a message to start the conversion on TA 
*
* @param [out]  none
*
* @retval       none
*
*******************************************************************************/
void sendMsgStartTa(void)
{
  adcQueueMsg_t         adcMsgTask;

  // Start message.
  adcMsgTask.adcEv    = EV_ADC_TA_START_CONV;
  /**** genera un IM trigger legato a corruzione dati che non mi spiego  */
  configASSERT(xQueueSendToBack(adcQueue, (void *)&adcMsgTask, portMAX_DELAY) == pdPASS);
}

/**
*
* @brief        Send a message to stop the conversion on TA 
*
* @param [out]  none
*
* @retval       none
*
*******************************************************************************/
void sendMsgStopTa(void)
{
  adcQueueMsg_t         adcMsgTask;

  // Start message.
  adcMsgTask.adcEv    = EV_ADC_TA_END_CONV;
  /**** genera un IM trigger legato a corruzione dati che non mi spiego  */
  configASSERT(xQueueSendToBack(adcQueue, (void *)&adcMsgTask, portMAX_DELAY) == pdPASS);
}

/**
*
* @brief        ADC configuration for TA measurement 
*
* @param [out]  none
*
* @retval       none
*
*******************************************************************************/
static void ADC_TA_Config(void)
{
  ADC_ChannelConfTypeDef  sConfig;
  
   /* ADC Initialization */
  AdcTAHandle.Instance                   = ADCxTA;
 
  AdcTAHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;

  AdcTAHandle.Init.Resolution            = ADC_RESOLUTION; //ADC_RESOLUTION_12B; 

  AdcTAHandle.Init.ScanConvMode          = DISABLE;
  AdcTAHandle.Init.ContinuousConvMode    = DISABLE; //ENABLE;
  AdcTAHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcTAHandle.Init.NbrOfDiscConversion   = 0;
  AdcTAHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
  AdcTAHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_TxTA_TRGO; 
  AdcTAHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcTAHandle.Init.NbrOfConversion       = 1;
  AdcTAHandle.Init.DMAContinuousRequests = DISABLE;
  AdcTAHandle.Init.EOCSelection          = DISABLE;

  if(HAL_ADC_Init(&AdcTAHandle) != HAL_OK)
  {
    /* ADC Initialization Error */
    AdcError_Handler();
  }
  
  /* Configure ADC3 regular channel */  
  sConfig.Channel = TA1_CHANNEL;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if(HAL_ADC_ConfigChannel(&AdcTAHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    AdcError_Handler();
  }
}

/**
*
* @brief        TIMxTA configuration
*
* @param [out]  none
*
* @retval       none
*
***********************************************************************************************************************/
static void TIM_TA_Config(void)
{
  TIM_MasterConfigTypeDef sMasterConfigTA;
  TIM_OC_InitTypeDef      sConfigTA;
  uint32_t                uwPrescalerValue = 0;

  /* TIM5 clock enable */
  TIM_TA_ADC_CLK_ENABLE();

   
  /*##-1- Configure the TIM peripheral #######################################*/ 
  /* -----------------------------------------------------------------------
    In this example TIM5 input clock (TIM5CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1. ( / 4 see main.c where we have: RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;) 
      TIM5CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM5CLK = HCLK / 2 = SystemCoreClock / 2 
    To get TIM5 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM5CLK / TIM5 counter clock) - 1
    Prescaler = ((SystemCoreClock ) /10 KHz) - 1
  
  ----------------------------------------------------------------------- */  
  
  /* Compute the prescaler value to have TIM5 counter clock equal to 3,6MHz */
  uwPrescalerValue = ((SystemCoreClock / 2) / 3600000) - 1;
  
  /* Set TIMx instance */
  htimTAAdc.Instance = TIMxTA;
   
  htimTAAdc.Init.Prescaler     = uwPrescalerValue;
  htimTAAdc.Init.Period        = PERIOD_TA_VALUE;     // now 3600/3,6MHz = 3,6 10^3 / 3,6 10^6 = 1 * 10^-3 = 1msec
  htimTAAdc.Init.ClockDivision = 0;                   // quindi prendiamo un campione ogni 1msec
  htimTAAdc.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&htimTAAdc) != HAL_OK)
  {
    /* Initialization Error */
    AdcError_Handler();
  }
  /* Common configuration for all channels */
  sConfigTA.OCMode     = TIM_OCMODE_PWM1;;
  sConfigTA.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigTA.OCFastMode = TIM_OCFAST_DISABLE;

  /* Set the pulse value for channel 1 */
  sConfigTA.Pulse = PULSE1_TA_VALUE;  
  if(HAL_TIM_PWM_ConfigChannel(&htimTAAdc, &sConfigTA, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    AdcError_Handler();
  }
   
  /* TIM5 TRGO selection */
  sMasterConfigTA.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfigTA.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    
  if(HAL_TIMEx_MasterConfigSynchronization(&htimTAAdc, &sMasterConfigTA) != HAL_OK)
  {
    /* TIM5 TRGO selection Error */
    AdcError_Handler();
  }  
}

/**
*
* @brief        Restart TA Conversion 
*
* @param [out]  none
*
* @retval       none
*
***********************************************************************************************************************/
static void startTAconversion (void)
{

  adcTask.validTA = FALSE;

  /*##-1- Start the conversion process and enable interrupt ##################*/ 
  if(HAL_ADC_Start_DMA(&AdcTAHandle, (uint32_t*)&taSamplesValue[0], NUM_TA_SAMPLES) != HAL_OK)
  {
    /* Start Conversation Error */
    AdcError_Handler(); 
  } 

  /*##-2- TIM5 counter enable ################################################*/ 
  if(HAL_TIM_Base_Start(&htimTAAdc) != HAL_OK)
  {
    /* Start Timer Error */
    AdcError_Handler();
  }
}

/**
*
* @brief        Stop TA Conversion 
*
* @param [out]  none
*
* @retval       none
*
***********************************************************************************************************************/
void stopTAconversion (void)
{
  adcTask.activeTAMeasure = FALSE;
}

/**
*
* @brief        TIMxTA initialization
*
* @param [out]  none
*
* @retval       none
*
***********************************************************************************************************************/
static void adcInitTAConfig (void)
{

  /*##-1- TIMx Peripheral Configuration ######################################*/
  TIM_TA_Config();

  /*##-2- Configure the ADC3 peripheral ######################################*/
  ADC_TA_Config();

}

/**
*
* @brief        Start sampling on TA
*
* @param [out]  none
*
* @retval       none
*
***********************************************************************************************************************/
static void startSamplingTA (void)
{

  /*##-3- Start the conversion process and enable interrupt ##################*/ 
  startTAconversion();

}


// *****************************************
static void adcInitTAVar (void)
{
  uint8_t i;

  adcTask.validTA = FALSE;
  adcTask.activeTAMeasure = TRUE;
  cnt = (uint8_t)0;
  gainN = gainD = (uint32_t)1;

  for (i = 0; i < NUM_TA_SAMPLES; i++)
  {
  	taSamplesValue[i] = 0;
  }
  for (i = 0; i < NUM_TA_SAMPLES_MEAN; i++)
  {
  	rmsSample[i] = 0;
  }
}

// *****************************************
static uint8_t taConfigComplete (void)
{
  return(adcTask.validTA);
}

/**
  * @brief  RMS estimation  
  * @param  none
  * @retval uint32_t: the rms estimated value 
  */
static uint32_t rmsEstimation(void)
{
  uint16_t  ix;
  uint64_t  sum;

  for (ix = 0, sum = 0; ix < NUM_TA_SAMPLES; ix++)
  {
    taSamplesValue[ix] -= DEFAULT_ADC_MEAN_VAL;
    sum += (uint32_t)(taSamplesValue[ix] * taSamplesValue[ix]);
  }
  /* teniamo conto del campionamento (0,5ms) e del periodo (20ms)*/
  /* 0,5 / 20 = 5 / 200                                          */
  /* teniamo conto dello step dell'ADC pari a 3,3V / 1024 */
  /* 3300 / 1024 = 5 / 200                                          */
  /**** oppure ****/
  /* teniamo conto del campionamento (1ms) e del periodo (20ms)*/
  /* 1 / 20                                                   */
  /* teniamo conto dello step dell'ADC pari a 3,3V / 1024 */
  /* 3300 / 1024 * 3300 / 1024 * 1 / 20                       */
   sum = (519 * sum) / 1000;
   return ((uint32_t)sqrtf((float)sum));
}

/**
*
* @brief        TIMxVIN initialization
*
* @param [out]  none
*
* @retval       none
*
***********************************************************************************************************************/
void triggerVINConfig (void)
{

  /*##-2- Configure the ADC2 peripheral ######################################*/
  ADC_VIN_Config();
 
  /*##-1- TIMx Peripheral Configuration ######################################*/
  TIM_VIN_Config();

  /*##-3- Start the conversion process and enable interrupt ##################*/ 
  startVINconversion();
}

/**
*
* @brief        TIMxVIN configuration
*
* @param [out]  none
*
* @retval       none
*
***********************************************************************************************************************/
static void TIM_VIN_Config(void)
{
  TIM_MasterConfigTypeDef sMasterConfigVIN;
  uint32_t                uwPrescalerValue = 0;

  /* TIM4 clock enable */
  TIM_VIN_ADC_CLK_ENABLE();

   
  /*##-1- Configure the TIM peripheral #######################################*/ 
  /* -----------------------------------------------------------------------
    In this example TIM4 input clock (TIM4CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1. ( / 4 see main.c where we have: RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;) 
      TIM4CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM4CLK = HCLK / 2 = SystemCoreClock / 2 
    To get TIM4 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM4CLK / TIM4 counter clock) - 1
    Prescaler = ((SystemCoreClock / 2) /10 KHz) - 1
  
  ----------------------------------------------------------------------- */  
  
  /* Compute the prescaler value to have TIM4 counter clock equal to 10KHz */
  uwPrescalerValue = ((SystemCoreClock / 2) / 10000) - 1;
  
  /* Set TIMx instance */
  htimVINadc.Instance = TIMxVIN;
   
  htimVINadc.Init.Prescaler     = uwPrescalerValue;
  htimVINadc.Init.Period        = PERIOD_VIN_VALUE;     // now 10000/10KHz = 10 10^3 / 10 10^6  = 1sec
  htimVINadc.Init.ClockDivision = 0;                    // quindi prendiamo un campione ogni 1sec
  htimVINadc.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&htimVINadc) != HAL_OK)
  {
    /* Initialization Error */
    AdcError_Handler();
  }

   /* TIM4 TRGO selection */
  sMasterConfigVIN.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfigVIN.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    
  if(HAL_TIMEx_MasterConfigSynchronization(&htimVINadc, &sMasterConfigVIN) != HAL_OK)
  {
    /* TIM5 TRGO selection Error */
    AdcError_Handler();
  }  
}

/**
*
* @brief        ADC configuration for VIN measurement (use window watch dog 
*               feature)
*
* @param [out]  none
*
* @retval       none
*
*******************************************************************************/
static void ADC_VIN_Config(void)
{
  ADC_ChannelConfTypeDef  sConfig;
  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};

   /* ADC Initialization */
  AdcVINHandle.Instance                   = ADCxVIN;
 
  AdcVINHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;

  AdcVINHandle.Init.Resolution            = ADC_RESOLUTION; //ADC_RESOLUTION_12B; 

  AdcVINHandle.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  AdcVINHandle.Init.ContinuousConvMode    = DISABLE; //ENABLE; DISABLE;
  AdcVINHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcVINHandle.Init.NbrOfDiscConversion   = 0;
  AdcVINHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
  AdcVINHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_TxVIN_TRGO; 
  AdcVINHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcVINHandle.Init.NbrOfConversion       = 1;
  AdcVINHandle.Init.DMAContinuousRequests = DISABLE;
  AdcVINHandle.Init.EOCSelection          = ADC_EOC_SEQ_CONV; // to avoid stop for overrun set DMA=0 and EOCS=0 as in 15.8.3

  if(HAL_ADC_Init(&AdcVINHandle) != HAL_OK)
  {
    /* ADC Initialization Error */
    AdcError_Handler();
  }
  
  /* Configure ADC2 regular channel */  
  sConfig.Channel = VIN_CHANNEL;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if(HAL_ADC_ConfigChannel(&AdcVINHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    AdcError_Handler();
  }

  /** Configure the analog watchdog
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = VIN_HTRH;
  AnalogWDGConfig.LowThreshold = VIN_LTRH;
  AnalogWDGConfig.Channel = VIN_CHANNEL;
  AnalogWDGConfig.ITMode = ENABLE;
  if (HAL_ADC_AnalogWDGConfig(&AdcVINHandle, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** start ADC module for conversion  */
  HAL_ADC_Start(&AdcVINHandle);

  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(ADC_IRQn, 12, 12);   
  HAL_NVIC_EnableIRQ(ADC_IRQn);

}

/**
*
* @brief        Restart VIN Conversion 
*
* @param [out]  none
*
* @retval       none
*
***********************************************************************************************************************/
static void startVINconversion (void)
{

  /*##-1- TIM4 counter enable ################################################*/ 
  if(HAL_TIM_Base_Start(&htimVINadc) != HAL_OK)
  {
    /* Start Timer Error */
    AdcError_Handler();
  }
}

/**
*
* @brief        Stop Vin Conversion 
*
* @param [out]  none
*
* @retval       none
*
***********************************************************************************************************************/
void stopVINconversion (void)
{
  /* stop the ADC2: activated */
  (void)HAL_ADC_Stop_DMA(&AdcVINHandle);
  /* stop the timer */
  (void)HAL_TIM_Base_Stop(&htimVINadc);
}


/**
*
* @brief        ADC configuration for CP measurement 
*
* @param [out]  none
*
* @retval       none
*
*******************************************************************************/
static void ADC_CP_Config(void)
{
  ADC_ChannelConfTypeDef  sConfig;
  
   /* ADC Initialization */
  AdcCPHandle.Instance                   = ADCxCP;
 
  AdcCPHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;

  AdcCPHandle.Init.Resolution            = ADC_RESOLUTION; //ADC_RESOLUTION_12B; 

  AdcCPHandle.Init.ScanConvMode          = DISABLE;
  AdcCPHandle.Init.ContinuousConvMode    = DISABLE; //ENABLE;
  AdcCPHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcCPHandle.Init.NbrOfDiscConversion   = 0;
  AdcCPHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
  AdcCPHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_TxCP_TRGO; 
  AdcCPHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcCPHandle.Init.NbrOfConversion       = 1;
  AdcCPHandle.Init.DMAContinuousRequests = DISABLE;
  AdcCPHandle.Init.EOCSelection          = DISABLE;

  if(HAL_ADC_Init(&AdcCPHandle) != HAL_OK)
  {
    /* ADC Initialization Error */
    AdcError_Handler();
  }
  
  /* Configure ADC2 regular channel */  
  sConfig.Channel = CP_CHANNEL;  // channel 0
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if(HAL_ADC_ConfigChannel(&AdcCPHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    AdcError_Handler();
  }
}

/**
*
* @brief        TIMxCP configuration
*
* @param [out]  none
*
* @retval       none
*
***********************************************************************************************************************/
static void TIM_CP_Config(void)
{
  TIM_MasterConfigTypeDef sMasterConfigCP;
  TIM_OC_InitTypeDef      sConfigCP;
  uint32_t                uwPrescalerValue = 0;

  /* TIM5 clock enable */
  TIM_CP_ADC_CLK_ENABLE();

   
  /*##-1- Configure the TIM peripheral #######################################*/ 
  /* -----------------------------------------------------------------------
    In this example TIM6 input clock (TIM6CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1. ( / 4 see main.c where we have: RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;) 
      TIM6CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM6CLK = HCLK / 2 = SystemCoreClock / 2 
    To get TIM6 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM6CLK / TIM6 counter clock) - 1
    Prescaler = ((SystemCoreClock ) /10 KHz) - 1
  
  ----------------------------------------------------------------------- */  
  
  /* Compute the prescaler value to have TIM6 counter clock equal to 3,6MHz */
  uwPrescalerValue = ((SystemCoreClock / 2) / 3600000) - 1;
  
  /* Set TIMx instance */
  htimCPAdc.Instance = TIMxCP;
   
  htimCPAdc.Init.Prescaler     = uwPrescalerValue;
  htimCPAdc.Init.Period        = PERIOD_CP_VALUE;     // now 225/3,6MHz = 225 / 3,6 10^6 = 62,5 * 10^-6 = 62,5usec
  htimCPAdc.Init.ClockDivision = 0;                   // quindi prendiamo un campione ogni 62,5usec
  htimCPAdc.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&htimCPAdc) != HAL_OK)
  {
    /* Initialization Error */
    AdcError_Handler();
  }
  /* Common configuration for all channels */
  sConfigCP.OCMode     = TIM_OCMODE_PWM1;;
  sConfigCP.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigCP.OCFastMode = TIM_OCFAST_DISABLE;

  /* Set the pulse value for channel 1 */
  sConfigCP.Pulse = PULSE_CP_VALUE;  
  if(HAL_TIM_PWM_ConfigChannel(&htimCPAdc, &sConfigCP, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    AdcError_Handler();
  }
   
  /* TIM6 TRGO selection */
  sMasterConfigCP.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfigCP.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    
  if(HAL_TIMEx_MasterConfigSynchronization(&htimCPAdc, &sMasterConfigCP) != HAL_OK)
  {
    /* TIM6 TRGO selection Error */
    AdcError_Handler();
  }  
}

/**
*
* @brief        Restart CP Conversion 
*
* @param [out]  none
*
* @retval       none
*
***********************************************************************************************************************/
static void startCPconversion (void)
{


  /*##-1- Start the conversion process and enable interrupt ##################*/ 
  if(HAL_ADC_Start_DMA(&AdcCPHandle, (uint32_t*)&cpSamplesValue[0], NUM_CP_SAMPLES) != HAL_OK)
  {
    /* Start Conversation Error */
    AdcError_Handler(); 
  } 

  /*##-2- TIM6 counter enable ################################################*/ 
  if(HAL_TIM_Base_Start(&htimCPAdc) != HAL_OK)
  {
    /* Start Timer Error */
    AdcError_Handler();
  }
}

/**
*
* @brief        Stop CP Conversion 
*
* @param [out]  none
*
* @retval       none
*
***********************************************************************************************************************/
static void stopCPconversion (void)
{
  /* stop the ADC2: activated only by CP (VIN doesn't use it)*/
  (void)HAL_ADC_Stop_DMA(&AdcCPHandle);
  /* stop the timer: on CP signal no more ADC conversion is need */
  (void)HAL_TIM_Base_Stop(&htimCPAdc);
}

/**
*
* @brief        TIMxCP initialization
*
* @param [out]  none
*
* @retval       none
*
***********************************************************************************************************************/
static void adcInitCPConfig (void)
{

  /*##-1- TIMx Peripheral Configuration ######################################*/
  TIM_CP_Config();

  /*##-2- Configure the ADC2 peripheral ######################################*/
  ADC_CP_Config();
 
  /*##-3- Start the conversion starts at the end of conversion group channels ##################*/ 
}

/**
  * @brief  get the current CP mean valuew  
  * @param  none
  * @retval uint16_t: the mean CP estimated value [mV] 
  */
uint16_t getCPvalue(void)
{
  uint32_t valCp;
  uint8_t i;

  for (i = (uint8_t)0, valCp = (uint32_t)0; i < NUM_CP_SAMPLES; i++)
  {
  	valCp += (uint32_t)cpSamplesValue[i];
  }
  valCp /= (uint32_t)NUM_CP_SAMPLES;
  valCp *= (uint32_t)3300;
  valCp /= (uint32_t)ADC_MAX_VALUE;

  return ((uint16_t)valCp);
}


/**
  * @brief  get the current RMS TA estimation  
  * @param  none
  * @retval uint32_t: the rms estimated value [mA]
  */
uint32_t getTAcurrent(void)
{
#ifdef ZENER_D8
  if (rmsValueMean < 13)
    return 0;
  else if (rmsValueMean <= 385)
    return ((rmsValueMean*353 - 4355) / 10);    // - 4360 + 5
  else
    return ((rmsValueMean*596 - 97584) / 10);    // - 97589 + 5
#else
  if (rmsValueMean < 3)
    return 0;
  else
    return ((rmsValueMean*329 - 771) / 10);    // - 776 + 5
#endif
}

/**
  * @brief  get the current up Temperature  
  * @param  none
  * @retval int16_t: temperature in 1/10 °C 
  */
int16_t getUpTemp(void)
{
  int32_t temperature;

  /* from data sheet Avg_Slope = 2,5mV/°C V25 = 0,76V and T[°C] = {(Vsense -V25)/Avg_Slope} + 25 */
  /* Avg_Slope = 25/105mV/°C V25 = 760mVV  --> 10T[°C] = {(Vsense -V25)/Avg_Slope}*10 + 250 */
  temperature = (((int32_t)meanVal[UP_TEMP_IN] * 3300 - (int32_t)778240) / 256) + 250;
  return((int16_t)temperature);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void AdcError_Handler(void)
{
  while(1)
  {
  }
}

/**
*
* @brief       Restituisce il valore misurato su ADC richiesto (10 bit)
*
* @param [in]  adcIn_e - indice ADC secondo enum adcIn_e
*
* @retval      uint16_t; valore ADC in mV 
*
*******************************************************************************/
uint16_t getADCmV(adcIn_e ixADC)
{
  uint32_t val;
   
  if (CP_ADC_IN == ixADC)
  {
    /* CP_ADC has a special management: at tick=10ms 32 samples every 62.5usec = 2msec */
    val = (uint32_t)getCPvalue();
  }
  else
  {
    val = 0x0000FFFF;
    if (NUM_ADC_SCU > ixADC)
    {
      val = ((uint32_t)meanVal[ixADC]);
      val *= (uint32_t)3300;
      val /= (uint32_t)ADC_MAX_VALUE;
    }
  }
  return ((uint16_t)val);
}

/**
*
* @brief       Restituisce la flag di validità
*
* @param [in]  -
*
* @retval      uint8_t     valid;
*
*******************************************************************************/
uint8_t getADCvalid(void)
{
  return ((uint8_t)adcTask.valid);
}


/**
*
* @brief  	   Disable DMA ADC interrupt 
*
* @param [in]  none 
*
* @retval      none 
*
***********************************************************************************************************************/
void disableDMA_ADCinterrupt(void)
{
  TIM_HandleTypeDef   gtim;
  TIM_HandleTypeDef*  ptim;

  /* Disable the transfer complete interrupt */
  HAL_ADC_Stop_DMA(&AdcHandle);
  /* Disable the TIM  */
  ptim = (&gtim);
  ptim->Instance = TIMx;
  HAL_TIM_Base_Stop(ptim);
  ptim->Instance = TIM9;
  HAL_TIM_Base_Stop(ptim);
}

/**
*
* @brief       Stop timer for ADC conversion  
*
* @param [in]  none  
*
* @retval      none 
*
*******************************************************************************/
void  stopTimerAdcConv (void)
{
  /*##-4- TIM2 counter enable ################################################*/ 
  if(HAL_TIM_OC_Stop_IT(&htimAdc, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Counter Enable Error */
    AdcError_Handler();
  }
}

/**
*
* @brief       Restituisce la posizione del rotary switch
*
* @param [in]  none
*
* @retval      rotaryPos_e: rotary position from adc value Return NUM_ROTARY_POS on  error
*
*******************************************************************************/
rotaryPos_e getRotarySwitchPos(void)
{
  uint32_t val;
  uint8_t  ix;
  
  val = ((uint32_t)meanVal[SW_ADC_IN]);
  val *= (uint32_t)3300;
  val /= (uint32_t)ADC_MAX_VALUE;

  for (ix = (uint8_t)POS0_6A; ix < (uint8_t)NUM_ROTARY_POS; ix++)
  {
    if (((uint16_t)val >= rotaryLimits[ix][0]) && ((uint16_t)val < rotaryLimits[ix][1]))
    {
      return ((rotaryPos_e)ix);
    }
  }
  return ((rotaryPos_e)NUM_ROTARY_POS);
}

/**
*
* @brief        Vin initialization and readind
*
* @param [out]  none
*
* @retval       uint16_t: Vin value 
*
***********************************************************************************************************************/
uint16_t vinPwrValue (void)
{
  uint32_t                uhADCxConvertedValue = 0;
  ADC_ChannelConfTypeDef  sConfig;

  /*##-1- Configure the ADC peripheral #######################################*/
  AdcHandle.Instance          = ADCxVIN;
  
  if (HAL_ADC_DeInit(&AdcCPHandle) != HAL_OK)
  {
    /* ADC de-initialization Error */
    Error_Handler();
  }


  AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
  AdcHandle.Init.Resolution            = ADC_RESOLUTION;
  AdcHandle.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.NbrOfDiscConversion   = 0;
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Conversion start trigged at each external event */
  AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.NbrOfConversion       = 1;
  AdcHandle.Init.DMAContinuousRequests = DISABLE;
  AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV; //DISABLE; 

  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    /* ADC initialization Error */
    Error_Handler();
  }

  /*##-2- Configure ADC regular channel ######################################*/

  sConfig.Channel      = VIN_CHANNEL;                 /* Sampled channel number */
  sConfig.Rank         = 1;                           /* Rank of sampled channel number ADCx_CHANNEL */
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;     /* Sampling time (number of clock cycles unit) */
  sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */

  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }

  
  /*##-3- Start the conversion process #######################################*/
  if (HAL_ADC_Start(&AdcHandle) != HAL_OK)
  {
    /* Start Conversation Error */
    Error_Handler();
  }

  /*##-4- Wait for the end of conversion #####################################*/
  if (HAL_ADC_PollForConversion(&AdcHandle, 10) != HAL_OK)
  {
    /* End Of Conversion flag not set on time */
    Error_Handler();
  }
  else
  {
    /* ADC conversion completed */
    /*##-5- Get the converted value of regular channel  ########################*/
    uhADCxConvertedValue = HAL_ADC_GetValue(&AdcHandle);
    /* Vin = Vadc * (10 /160) * (3300 /1024) dove 10/160 è ilpartitore resistivo e 3300/1024 la risoluzione ADC */
    uhADCxConvertedValue = (uhADCxConvertedValue * (uint32_t)3300 * 16) / (1024);
  }
  return((uint16_t)uhADCxConvertedValue);
}


/*************** END OF FILE ******************************************************************************************/


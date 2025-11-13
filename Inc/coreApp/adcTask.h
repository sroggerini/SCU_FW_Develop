/**
* @file        adcTask.h
*
* @brief       Management input - output from Rs232 terminal  - Definition
*
* @author      Nicolino Piergiovanni, Clusone
*
* @riskClass   C
*
* @moduleID    SAD_APP_130
*
* @vcsInfo
*     $Id: adcTask.h 650 2024-12-20 16:07:37Z stefano $
*  
*     $Revision: 650 $
*  
*     $Author: stefano $
*  
*     $Date: 2024-12-20 17:07:37 +0100 (ven, 20 dic 2024) $
*
* @copyright
*       Copyright (C) 2017 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

/****************** INCLUDES ******************************************************************************************/
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_hal_adc.h"
#else
#include "stm32h5xx_hal.h"
#include "stm32h5xx_ll_rcc.h"
#include "stm32h5xx_hal_adc.h"
#include "stm32h5xx_hal_adc_legacy.h"
#endif
#include "wrapper.h"

/****************** LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM ************************************************************/

typedef enum
{
  EV_ADC_START = 0,
  EV_ADC_TA_START_CONV,
  EV_ADC_VIN_START_CONV,
  EV_ADC_TA_POLLING,
  EV_ADC_TA_END_CONV,
  EV_ADC_LAST = 0xFFFF
} adcMsg_e;

/* remember: num samples  must be power of 2 for more simple division */
#define NUM_SAMPLE_FOR_MEAN                4 

typedef struct
{
  uint16_t   meanData[NUM_ADC_SCU][NUM_SAMPLE_FOR_MEAN];
} adcMsg_t;

typedef struct
{
  adcMsg_e   adcEv;
} adcQueueMsg_t;

//$ANP
/******* vecchi dati ADC ******
         per il momento li inserisco..... */

/* ADC resolution  */ 
           
/* NOTE: the resolution of the ADC has been changed from 10bit to 12bit, so the following value has changed */           
#define ADC_MAX_VALUE  4096
/* NOTE: the resolution of the ADC has been changed from 10bit to 12bit since GD32F4xx ever gives the result of conversion on 12bit
         so to simplify the computations, also the resolution of ST's ADC has been aligned to that */
#define ADC_RESOLUTION ADC_RESOLUTION_12B 

// ***  State definition
#define  ADC_IDLE                 0
#define  ADC_RUNNING              1   /* stato gestione campioni ADC      */

/***************** definizione ingressi ADC   **************************/
#define ADCn                             ((uint16_t)NUM_ADC_SCU)

#define CP_ADC_PIN                       GPIO_PIN_0
#define CP_ADC_GPIO_PORT                 GPIOA
#define CP_ADC_GPIO_CLK_ENABLE()         __GPIOA_CLK_ENABLE()
#define CP_ADC_GPIO_CLK_DISABLE()        __GPIOA_CLK_DISABLE()

#define PP_ADC_PIN                       GPIO_PIN_0
#define PP_ADC_GPIO_PORT                 GPIOB
#define PP_ADC_GPIO_CLK_ENABLE()         __GPIOB_CLK_ENABLE()
#define PP_ADC_GPIO_CLK_DISABLE()        __GPIOB_CLK_DISABLE()

#define SW_ADC_PIN                       GPIO_PIN_2
#define SW_ADC_GPIO_PORT                 GPIOC
#define SW_ADC_GPIO_CLK_ENABLE()         __GPIOC_CLK_ENABLE()
#define SW_ADC_GPIO_CLK_DISABLE()        __GPIOC_CLK_DISABLE()

#define TA1_PIN                          GPIO_PIN_0
#define TA1_GPIO_PORT                    GPIOC
#define TA1_GPIO_CLK_ENABLE()            __GPIOC_CLK_ENABLE()
#define TA1_GPIO_CLK_DISABLE()           __GPIOC_CLK_DISABLE()

#define TEMP_PIN                         GPIO_PIN_3
#define TEMP_GPIO_PORT                   GPIOC
#define TEMP_GPIO_CLK_ENABLE()           __GPIOC_CLK_ENABLE()
#define TEMP_GPIO_CLK_DISABLE()          __GPIOC_CLK_DISABLE()

#define IM_ADC_PIN                       IM_ADC_Pin
#define IM_ADC_GPIO_PORT                 IM_ADC_GPIO_Port
#define IM_ADC_GPIO_CLK_ENABLE()         __GPIOA_CLK_ENABLE()
#define M_ADC_GPIO_CLK_DISABLE()        __GPIOA_CLK_DISABLE()

#define VIN_PIN                          GPIO_PIN_1
#define VIN_GPIO_PORT                    GPIOB
#define VIN_GPIO_CLK_ENABLE()           __GPIOB_CLK_ENABLE()
#define VIN_GPIO_CLK_DISABLE()          __GPIOB_CLK_DISABLE()


#define ADCx_GPIO_CLK_ENABLE(__INDEX__)  do{if((__INDEX__) == 0) CP_ADC_GPIO_CLK_ENABLE(); else \
                                            if((__INDEX__) == 1) PP_ADC_GPIO_CLK_ENABLE(); else \
                                            if((__INDEX__) == 2) SW_ADC_GPIO_CLK_ENABLE(); else \
                                            if((__INDEX__) == 3) TA1_GPIO_CLK_ENABLE(); else \
                                            if((__INDEX__) == 9) TEMP_GPIO_CLK_ENABLE(); else \
                                            if((__INDEX__) == 4) IM_ADC_GPIO_CLK_ENABLE(); \
                                            }while(0)

#define ADCx_GPIO_CLK_DISABLE(__INDEX__)  do{if((__INDEX__) == 0) CP_ADC_GPIO_CLK_DISABLE(); else \
                                             if((__INDEX__) == 1) PP_ADC_GPIO_CLK_DISABLE(); else \
                                             if((__INDEX__) == 2) SW_ADC_GPIO_CLK_DISABLE(); else \
                                             if((__INDEX__) == 3) TA1_GPIO_CLK_DISABLE(); else \
                                             if((__INDEX__) == 9) TEMP_GPIO_CLK_ENABLE(); else \
                                             if((__INDEX__) == 4) IM_ADC_GPIO_CLK_DISABLE(); \
                                             }while(0)



/***************** definizione timer trigger ADC    **************************/
#define TIMx                            TIM2
#define TIM_ADC_CLK_ENABLE()            __HAL_RCC_TIM2_CLK_ENABLE();

#define TIMx_FORCE_RESET()              __HAL_RCC_TIM2_FORCE_RESET()
#define TIMx_RELEASE_RESET()            __HAL_RCC_TIM2_RELEASE_RESET()

/* Definition for TIMx's NVIC */
#define TIMx_IRQn                      TIM2_IRQn
#define TIMx_IRQHandler                TIM2_IRQHandler
     
/* Definition for TIMx Pins */
#define TIMx_CHANNEL_GPIO_PORT()       __HAL_RCC_GPIOC_CLK_ENABLE()
#define GPIO_PIN_CHANNEL1              GPIO_PIN_6

/* --------------  constant definition for sampling period ---------- */                                               
//#define  PERIOD_VALUE       (72000 - 1)  /* Period Value  */
#define  PERIOD_VALUE       (36000 - 1)  /* Period Value  */
#define  PULSE1_VALUE       36000        /* Capture Compare 1 Value  Not Used */

/************* Definition for ADCx clock resources ******************/
#define ADCx                            ADC1
#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC1_CLK_ENABLE();
#define ADCx_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();__HAL_RCC_GPIOC_CLK_ENABLE()
#define DMAx_ADC_CLK_ENABLE()           __HAL_RCC_DMA2_CLK_ENABLE()     
     
#define ADCx_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()

#ifndef HW_MP28947                                               
                                               
#define CP_ADC_CHANNEL                  ADC_CHANNEL_0
#define PP_ADC_CHANNEL                  ADC_CHANNEL_8
#define SW_ADC_CHANNEL                  ADC_CHANNEL_12
#define UP_TEMP_CHANNEL                 ADC_CHANNEL_18
#define TEMP_CHANNEL                    ADC_CHANNEL_13
#define IM_ADC_CHANNEL                  ADC_CHANNEL_3
#define VIN_ADC_CHANNEL                 ADC_CHANNEL_9
                                               
#else
                                               
#define CP_ADC_CHANNEL                  ADC_CHANNEL_0
                                               
#ifdef GD32F4xx                                               
#define TA23TYPE_ADC_CHANNEL            ADC_CHANNEL_4
#define TA1TYPE_ADC_CHANNEL             ADC_CHANNEL_8
#define VIN_ADC_CHANNEL                 ADC_CHANNEL_9                                                                                             
#define TEMP_CHANNEL                    ADC_CHANNEL_13
#else
#define VIN_ADC_CHANNEL                 ADC_CHANNEL_5                                              
#define TEMP_CHANNEL                    ADC_CHANNEL_12
#endif
                                                                                              
#endif                                                                                             
     
/* Definition for ADCx's DMA */
#define ADCx_DMA_CHANNEL                DMA_CHANNEL_0
#define ADCx_DMA_STREAM                 DMA2_Stream0         

/* Definition for ADCx's NVIC */
#define ADCx_DMA_IRQn                   DMA2_Stream0_IRQn
#define ADCx_DMA_IRQHandler             DMA2_Stream0_IRQHandler     

/************* Definition for ADCxTA clock resources ******************/
#define ADCxTA                            ADC3
#define ADCxTA_CLK_ENABLE()               __HAL_RCC_ADC3_CLK_ENABLE();
#define ADCxTA_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();__HAL_RCC_GPIOC_CLK_ENABLE()
#define DMAxTA_ADC_CLK_ENABLE()           __HAL_RCC_DMA2_CLK_ENABLE()     
     
#define ADCxTA_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()
#define ADCxTA_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()


#define TA1_CHANNEL                       ADC_CHANNEL_10

/* Definition for ADCx's DMA */
#define ADCxTA_DMA_CHANNEL                DMA_CHANNEL_2
#define ADCxTA_DMA_STREAM                 DMA2_Stream1         

/* Definition for ADCx's NVIC */
#define ADCxTA_DMA_IRQn                   DMA2_Stream1_IRQn
#define ADCxTA_DMA_IRQHandler             DMA2_Stream1_IRQHandler     

/***************** definizione timer trigger ADC3    **************************/
#define TIMxTA                            TIM5
#define TIM_TA_ADC_CLK_ENABLE()           __HAL_RCC_TIM5_CLK_ENABLE();
 
#define TIMxTA_FORCE_RESET()              __HAL_RCC_TIM5_FORCE_RESET()
#define TIMxTA_RELEASE_RESET()            __HAL_RCC_TIM5_RELEASE_RESET()

/* Definition for TIMx's NVIC */
#define TIMxTA_IRQn                      TIM5_IRQn
#define TIMxTA_IRQHandler                TIM5_IRQHandler
                                               
/* Definition for TIMx Pins */
#define TIMxTA_CHANNEL_GPIO_PORT()       __HAL_RCC_GPIOC_CLK_ENABLE()
#define GPIO_PIN_CHANNEL1                GPIO_PIN_6

/* --------------  constant definition for sampling period ---------- */                                               
#define  PERIOD_TA_VALUE       (3600 - 1)   /* Period Value  */
#define  PULSE1_TA_VALUE       1800         /* Capture Compare 1 Value  Not Used */

/** Num samples on TA signal to calculate RMS value   **/
#define  NUM_TA_SAMPLES        ((uint8_t)20)
//#define  NUM_TA_SAMPLES_MEAN   ((uint8_t)4)
#define  NUM_TA_SAMPLES_MEAN   ((uint8_t)16)
/** TA signal on ADC has 1,65V mean value. If ADC has 10bit this mean the following value **/
#define  DEFAULT_ADC_MEAN_VAL  ((uint16_t)(ADC_MAX_VALUE / 2))
/** Measuring polling time in msec on TA signal                          **/
#define     POLLING_TA_250MS                ((uint16_t)250)

/***************** definizione timer trigger ADC2    **************************/
#define TIMxVIN                           TIM4
#define TIM_VIN_ADC_CLK_ENABLE()          __HAL_RCC_TIM4_CLK_ENABLE();
 
#define TIMxVIN_FORCE_RESET()             __HAL_RCC_TIM4_FORCE_RESET()
#define TIMxVIN_RELEASE_RESET()           __HAL_RCC_TIM4_RELEASE_RESET()

/* Definition for TIMx's NVIC */
#define TIMxVIN_IRQn                     TIM4_IRQn
#define TIMxVIN_IRQHandler               TIM4_IRQHandler
                                               
/* Definition for TIMx Pins */
#define TIMxVIN_CHANNEL_GPIO_PORT()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define GPIO_PIN_CHANNEL9                GPIO_PIN_1

/* --------------  constant definition for sampling period ---------- */                                               
#define  PERIOD_VIN_VALUE       (10000 - 1)  /* Period Value  */
#define  PULSE1_VIN_VALUE       (5000)         /* Capture Compare 1 Value  Not Used */

/************* Definition for ADCxTA clock resources ******************/
#define ADCxVIN                            ADC1
#define ADCxVIN_CLK_ENABLE()              __HAL_RCC_ADC2_CLK_ENABLE();
#define ADCxVIN_CHANNEL_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();__HAL_RCC_GPIOC_CLK_ENABLE()
#define DMAxVIN_ADC_CLK_ENABLE()          __HAL_RCC_DMA2_CLK_ENABLE()     
     
#define ADCxVIN_FORCE_RESET()             __HAL_RCC_ADC_FORCE_RESET()
#define ADCxVIN_RELEASE_RESET()           __HAL_RCC_ADC_RELEASE_RESET()

#ifdef GD32F4xx
#define VIN_CHANNEL                       ADC_CHANNEL_9
#else
#define VIN_CHANNEL                       ADC_CHANNEL_5
#endif
                                               
/* NOTE: 
VIN_HTRH, the threshold high for analog watchdog, is not important for that purpose, but is set @25V anyway.
With the threshold @24V, the analog watchdog interrupt happens as soon as the function is enabled.    */

#define VIN_HTRH                         0xFFF            // VMax = 25V: con R3=150K ed R5=10K N=462+10% --> VIN = 25/16 = 1,56 --> N = 4096 * 1,56 / 3,3 =  1939                                               
#define VIN_LTRH_15V                     ((uint32_t)1164) // Vmin=15V: con R3=150K ed R5=10K Vmin=15V --> VIN = 15/16 -->0,937V --> N= 4096 * 0,937 / 3,3 = 1164 */
#define VIN_LTRH_18V                     ((uint32_t)1396) // Vmin=18V: con R3=150K ed R5=10K Vmin=18V --> VIN = 18/16 -->1,125V --> N= 4096 * 1,125 / 3,3 = 1396 */
#define VIN_LTRH_20V                     ((uint32_t)1551) // Vmin=20V: con R3=150K ed R5=10K Vmin=20V --> VIN = 20/16 -->1,25V --> N= 4096 * 1,25 / 3,3 = 1551 */
                                               
/************* Definition for ADCxCP clock resources ******************/
#define ADCxCP                            ADC2
#define ADCxCP_CLK_ENABLE()               __HAL_RCC_ADC2_CLK_ENABLE();
#define ADCxCP_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();__HAL_RCC_GPIOC_CLK_ENABLE()
#define DMAxCP_ADC_CLK_ENABLE()           __HAL_RCC_DMA2_CLK_ENABLE()     
     
#define ADCxCP_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()
#define ADCxCP_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()


#define CP_CHANNEL                        ADC_CHANNEL_0

#ifdef GD32F4xx                                               
/* Definition for ADCx's DMA */
#define ADCxCP_DMA_CHANNEL                DMA_CHANNEL_1
#define ADCxCP_DMA_STREAM                 DMA2_Stream3         

/* Definition for ADCx's NVIC */
#define ADCxCP_DMA_IRQn                   DMA2_Stream3_IRQn
#define ADCxCP_DMA_IRQHandler             DMA2_Stream3_IRQHandler                                                    
#else                                               
/* Definition for ADCx's DMA */
#define ADCxCP_DMA_CHANNEL                DMA_CHANNEL_1
#define ADCxCP_DMA_STREAM                 DMA2_Stream2         

/* Definition for ADCx's NVIC */
#define ADCxCP_DMA_IRQn                   DMA2_Stream2_IRQn
#define ADCxCP_DMA_IRQHandler             DMA2_Stream2_IRQHandler     
#endif
                                               
/***************** definizione timer trigger ADC2    **************************/
                                               
#define TIMxCP                            TIM8
#define TIM_CP_ADC_CLK_ENABLE()           __HAL_RCC_TIM8_CLK_ENABLE();
 
#define TIMxCP_FORCE_RESET()              __HAL_RCC_TIM8_FORCE_RESET()
#define TIMxCP_RELEASE_RESET()            __HAL_RCC_TIM8_RELEASE_RESET()

/* Definition for TIMx's NVIC */
#define TIMxCP_IRQn                      TIM8_DAC_IRQn
#define TIMxCP_IRQHandler                TIM8_DAC_IRQHandler
                                               
/* Definition for TIMx Pins */
#define TIMxCP_CHANNEL_GPIO_PORT()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define GPIO_CP_PIN_CHANNEL0             GPIO_PIN_0

/* --------------  constant definition for sampling period ---------- */                                               
#define  PERIOD_CP_VALUE       (225 - 1)   /* Period Value -> 62.5usec  */
#define  PULSE_CP_VALUE        112         /* Capture Compare 1 Value  Not Used */

/** Num samples on CP signal to calculate mean CP value   **/
#define  NUM_CP_SAMPLES        ((uint8_t)128)  /* 128 samples at 62,5us = 8msec */
#define  NUM_CP_SAMPLES_MEAN   ((uint8_t)4)

/* Definitions for Legacy purpose */                                               
#ifdef GD32F4xx
#define ADC_REGULAR_RANK_1                      1
#define ADC_SCAN_DISABLE                        ((uint32_t)0x00000000)          /* Scan mode disabled */                                               
#define ADC_EXTERNALTRIGCONV_TxTA_TRGO           ADC_EXTERNALTRIGCONV_T5_CC1     /* This change require some other implementations I guess */
#define ADC_EXTERNALTRIGCONV_TxVIN_TRGO          ADC_EXTERNALTRIGCONV_T4_CC4     /* This change require some other implementations I guess  */
#define ADC_EXTERNALTRIGCONV_TxCP_TRGO           ADC_EXTERNALTRIGCONV_T8_TRGO    /* This change require some other implementations I guess  */
#endif
                                               
/****************** LOCAL VARIABLES ***********************************************************************************/

/****************** GLOBAL VARIABLES ***********************************************************************************/

/****************** EXTERNAL VARIABLES ********************************************************************************/
extern xQueueHandle               adcQueue;

/****************** EXTERNAL PROCEDURE PROTOTYPE **********************************************************************/

/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
void      adcGestTask               (void* pvParameters);
void      stopTimerAdcConv          (void);
void      startTimerAdcConv         (void);
uint8_t   getADCvalid               (void);
void      triggerVINConfig          (void);
uint16_t  getCPvalue                (void);

void      setTempOffsetValue        (uint16_t valOffset);
void      ADC_CP_Config             (void);

/*************** END OF FILE ******************************************************************************************/















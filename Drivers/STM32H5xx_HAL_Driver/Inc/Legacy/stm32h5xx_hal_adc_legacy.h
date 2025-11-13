/**
  ******************************************************************************
  * @file    stm32h5xx_hal_adc.h
  * @author  MCD Application Team
  * @brief   Header file of ADC HAL module.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32H5xx_HAL_ADC_LEGACY_H
#define STM32H5xx_HAL_ADC_LEGACY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal_def.h"
#include "stm32h5xx_hal_adc.h"

/* Include low level driver */
#include "stm32h5xx_ll_adc.h"

/** @addtogroup STM32H5xx_HAL_Driver
  * @{
  */

/** @addtogroup ADC
  * @{
  */

/* Exported types ------------------------------------------------------------*/

#define ADC_EXTERNALTRIGCONV_T2_TRGO             ADC_EXTERNALTRIG_T2_TRGO                                                             
#define ADC_SAMPLETIME_15CYCLES                  ADC_SAMPLETIME_12CYCLES_5
#define ADC_SAMPLETIME_56CYCLES                  ADC_SAMPLETIME_47CYCLES_5
#define ADC_SAMPLETIME_112CYCLES                 ADC_SAMPLETIME_92CYCLES_5
#define ADC_SAMPLETIME_480CYCLES                 ADC_SAMPLETIME_247CYCLES_5
#define ADC_EXTERNALTRIGCONV_TxTA_TRGO           ADC_EXTERNALTRIG_T3_TRGO  
#define ADC_EXTERNALTRIGCONV_TxVIN_TRGO          ADC_EXTERNALTRIG_T4_CC4 
#define ADC_EXTERNALTRIGCONV_TxCP_TRGO           ADC_EXTERNALTRIG_T8_TRGO  
#define ADC_EXTERNALTRIGCONV_T1_CC1              ADC_EXTERNALTRIG_T1_CC1

#define ADC_IRQn                                 ADC1_IRQn                               
  
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif


#endif /* STM32H5xx_HAL_ADC_LEGACY_H */

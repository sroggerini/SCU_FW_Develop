/**
  ******************************************************************************
  * @file    stm32h5xx_ll_rcc.h
  * @author  MCD Application Team
  * @brief   Header file of RCC LL module.
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
#ifndef __STM32H5xx_LL_RCC_LEGACY_H
#define __STM32H5xx_LL_RCC_LEGACY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx.h"

/** @addtogroup STM32H5xx_LL_Driver
  * @{
  */

  /**
	* @brief  Check if RCC flag BOR reset is set or not.
	* @rmtoll RSR          BORRSTF       LL_RCC_IsActiveFlag_BORRST
	* @retval State of bit (1 or 0).
	*/

#define LL_RCC_IsActiveFlag_PORRST() 	LL_RCC_IsActiveFlag_BORRST()

#define __HAL_RCC_PWR_CLK_ENABLE      __HAL_RCC_BKPRAM_CLK_ENABLE

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32H5xx_LL_RCC_LEGACY_H */


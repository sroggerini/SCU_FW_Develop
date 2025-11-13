/**
  ******************************************************************************
  * @file    stm32h5xx_hal_uart_legacy.h
  * @author  MCD Application Team
  * @brief   Header file of UART HAL module.
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
#ifndef STM32H5xx_HAL_UART_LEGACY_H
#define STM32H5xx_HAL_UART_LEGACY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal_def.h"
#include "stm32h5xx_hal_usart.h"


/** @brief  Clear the USART RTOF pending flag.
  * @param  __HANDLE__ specifies the USART Handle.
  * @retval None
  */
#define __HAL_UART_CLEAR_RTOFLAG(__HANDLE__)   __HAL_USART_CLEAR_FLAG((__HANDLE__), USART_CLEAR_RTOF)

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32H5xx_HAL_UART_LEGACY_H */


/**
  ******************************************************************************
  * @file    st_device.h
  * @author  AMG/IPC Application Team
  * @brief   This file contains all the functions prototypes for metrology
  @verbatim
  @endverbatim

  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ST_DEVICE_H
#define __ST_DEVICE_H

#ifdef __cplusplus
 extern "C" {
#endif
 
   
/* Includes ------------------------------------------------------------------*/
#ifdef GD32F4xx  
#include "stm32f4xx_hal.h"
#else
#include "stm32h5xx_hal.h"
#endif   
   
/* Communication */
#define USART_SPEED		    115200
#define USART_TIMEOUT		    10
#define SPI_STPM_SPEED      32
#define SPI_TIMEOUT		      10
     
/* I2C EEPROM */
#define I2C_EEPROM		    I2C1
#define I2C_TIMEOUT		    100
  
#define CDC_POLLING_INTERVAL             1 /* in ms. The max is 65ms and the min is 1ms */
/*----------------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Definition of the return platform: */
typedef enum {
  HW_STCOMET=0,
  HW_ST8500,
  HW_STM32,
  HW_STM32_ST8500,
  HW_STM32_ST8500_FACTORY
} hwType_t;

typedef enum {
  PLC_NONE=0,
  PLC_G3_COORD,
  PLC_G3_DEVICE,
  PLC_PRIME_SN,
  PLC_PRIME_BN,
} plcType_t;

typedef enum {
  METRO_NONE=0,
  METRO_ENABLED,
} metroType_t;

typedef enum {
  RF_NONE=0,
  RF_G3,
  RF_WMBUS,
} rfType_t;

typedef enum {
  TRACE_ASCII=0,
  TRACE_BINARY_G3,
} traceType_t;

   typedef enum {
  PLTF_UNDEF = 0,
  PLTF_STMET,                   // STMET
  PLTF_STCOMET_COORD = 10,    // COMET
  PLTF_STCOMET_DEVICE,
  PLTF_STCOMET_COORD_METER,
  PLTF_STCOMET_DEVICE_METER,
  PLTF_STM32_METER = 20,      // STM32 + STPM3x
  PLTF_STM32_COORD = 30,      // STM32 + STarCom
  PLTF_STM32_DEVICE,
  PLTF_STARCOM_COORD = 40,    // STarCom
  PLTF_STARCOM_DEVICE,
} platform_t;



/* Exported constants --------------------------------------------------------*/
/*                      FW VERSION definition */
#define MAJOR_VERSION       1
#define MINOR_VERSION       6
#define PATCH_VERSION       4

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /* __ST_DEVICE_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

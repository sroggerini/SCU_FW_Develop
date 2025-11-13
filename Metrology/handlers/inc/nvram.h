/**
*   @file      nvram.h
*   @author    IPC - Industrial BU
*   @date      17 May 2016
*   @brief     This file includes types and macros for NVRAM
*   @note      (C) COPYRIGHT 2013 STMicroelectronics
*
* @attention
*
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*/
#ifndef NVRAM_H
#define NVRAM_H

/*******************************************************************************
* INCLUDE FILES:
*******************************************************************************/

/** @addtogroup GENERIC
  * @{
  */

/*******************************************************************************
* CONSTANTS & MACROS:
*******************************************************************************/

// definitions for NVRAM
#define NVRAM_SIZE_GEN              128
#define NVRAM_SIZE_LEG              512

#define NVRAM_MARKER_GEN            0xAA0000AA
#define NVRAM_MARKER_LEG            0xAA3333AA
#define NVRAM_VERSION_GEN           1
#define NVRAM_VERSION_LEG           1

#define NVRAM_SAVE_NOW              0
#define NVRAM_SAVE_SOON             1
#define NVRAM_SAVE_LATE             2
#define NVRAM_ERASE                 0xFF

#define NUM_SENSOR_TYPE             4       /* managed current Sensor for TA23 are four: 1) ITACOIL SBT002 
                                                                                         2) ITACOIL SBT012
                                                                                         3) HONGFA HMCT406 
                                                                                         4) SCT 10-1-200TS-A02 delivered by SMEUCO srl*/

/*******************************************************************************
* TYPES:
*******************************************************************************/
typedef enum {
  NVM_SAVE_ALL = 0,
  NVM_SAVE_GEN,
  NVM_SAVE_LEG,
  NVM_SAVE_WITHOUT_ERASE_ALL = 16,
  NVM_SAVE_WITHOUT_ERASE_GEN,
  NVM_SAVE_WITHOUT_ERASE_LEG,
  NVM_SAVE_NONE = 0xFF,
} nvmSave_t;

/*******************************************************************************
* NVM definitions for GENERIC:
*******************************************************************************/
typedef struct {
  uint32_t       marker;
  uint16_t       version;
  uint16_t       platform;
} nvmGen_t;


/*******************************************************************************
* NVM definitions for LEGAL:
*******************************************************************************/
typedef struct {
  uint32_t       marker;
  uint16_t       version;
  uint16_t       crc;
  uint8_t        SerialNumber[8];
  uint32_t       config[2];
  uint32_t       data1[21];
  uint32_t       data2[21]; 
  uint32_t       data3[21]; 
  uint32_t       data4[21]; 
  uint32_t       powerFact[NUM_SENSOR_TYPE][4];
  uint32_t       voltageFact[4];
  uint32_t       currentFact[NUM_SENSOR_TYPE][4];
  int16_t        V_CalibFact[3];
  int16_t        I_CalibFact[3];
  uint16_t       CalibStatus;
  uint16_t       Config;
} nvmLeg_t;


/*******************************************************************************
* NVM definitions:
*******************************************************************************/
typedef struct {
  union {
    nvmGen_t  gen;
    uint8_t        tmpGen[NVRAM_SIZE_GEN];
  };
  union {
    nvmLeg_t  leg;
    uint8_t        tmpLeg[NVRAM_SIZE_LEG];
  };
} nvm_t;

/*******************************************************************************
* GLOBAL VARIABLES:
*******************************************************************************/

/*******************************************************************************
* FUNCTIONS:
*******************************************************************************/

#endif /* NVRAM_H */

/**
  * @}
  */

/* End Of File */

/**
* @file        ExtFlash.c
*
* @brief       manager for QSPI Flash - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: ExtFlash_stm32f7.c 150 2022-10-24 15:30:31Z stefano $
*
*     $Revision: 150 $
*
*     $Author: stefano $
*
*     $Date: 2022-10-24 17:30:31 +0200 (lun, 24 ott 2022) $
*
*
* @copyright
*       Copyright (C) 2017 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

/************************************************************
 * Include
 ************************************************************/
#include <string.h>
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_dma.h"
#include "ff.h"
#include "ffconf.h"
#include "main.h"
#include "ExtFlash.h"
#include "cmsis_os.h"
#include "flashFat.h"

/*
*********************************** AESYS ************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
******************************************************************************
*/
// Struttura FAT simulata su QSPI
struct StructFAT FAT;

// Tipo di QSPI
unsigned char QSPIModel;

// Info sulla QSPI in uso
QSPI_Info QSPIInformation;
unsigned char FlashID[10];
unsigned char WriteOptimization;

// Dettagli dimensioni flash QSPI
unsigned int FlashDimBlk, FlashDimPag;

// Puntatore al buffer di backup
unsigned int  PBackupQSPI;

extern osThreadId_t wifiMngTaskHandle;


/*
*********************************** AESYS ************************************
**                                                                          **
**                            Local  Variables                              **
**                                                                          **
******************************************************************************
*/

/*
*********************************** AESYS ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/
// Struttura per pilotare la QSPI
extern QSPI_HandleTypeDef hqspi;

// Semaforo
extern osSemaphoreId FlashSemaphoreHandle;


/*
*********************************** AESYS ************************************
**                                                                          **
**                            External Function                             **
**                                                                          **
******************************************************************************
*/


/*
*********************************** AESYS ************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
// Funzioni gestione flash
static uint8_t QSPI_WriteEnable           (void);
static uint8_t QSPI_AutoPollingMemReady   (uint32_t Timeout);
#ifdef USE_ERASE_CHIP
static uint8_t BSP_QSPI_Erase_Chip        (void);
#endif
static uint8_t BSP_QSPI_Read              (uint8_t* pData, uint32_t ReadAddr, uint32_t Size);
static uint8_t BSP_QSPI_Write             (uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
static uint8_t BSP_QSPI_GetStatusReg1     (uint8_t *reg);
static uint8_t BSP_QSPI_GetConfigReg1     (uint8_t *reg);
static uint8_t BSP_QSPI_SetQuadMode       (void);
static uint8_t BSP_QSPI_Erase_SubBlock    (uint32_t BlockAddress);

//-----------------------------------------------------------------
// Inizio routine gestione FLASH a basso livello
//-----------------------------------------------------------------


/**
  * @brief  This function send a Write Enable and wait it is effective.
  * @retval None
  */
static uint8_t QSPI_WriteEnable(void)
{
  QSPI_CommandTypeDef     s_command;
  QSPI_AutoPollingTypeDef s_config;

  /* Enable write operations */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = WRITE_ENABLE_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait for write enabling */
  s_config.Match           = FLASH_SR_WREN;
  s_config.Mask            = FLASH_SR_WREN;
  s_config.MatchMode       = QSPI_MATCH_MODE_AND;
  s_config.StatusBytesSize = 1;
  s_config.Interval        = 0x10;
  s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  s_command.Instruction    = READ_STATUS_REG_CMD;
  s_command.DataMode       = QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling(&hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  This function read the SR of the memory and wait the EOP.
  * @retval None
  */
static uint8_t QSPI_AutoPollingMemReady(uint32_t Timeout)
{
  QSPI_CommandTypeDef     s_command;
  QSPI_AutoPollingTypeDef s_config;

  /* Configure automatic polling mode to wait for memory ready */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = READ_STATUS_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  s_config.Match           = 0;
  s_config.Mask            = FLASH_SR_WIP | FLASH_SR_WREN;
  s_config.MatchMode       = QSPI_MATCH_MODE_AND;
  s_config.StatusBytesSize = 1;
  s_config.Interval        = 0x10;
  s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling(&hqspi, &s_command, &s_config, Timeout) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  Reads an amount of data from the QSPI memory.
  * @param  pData: Pointer to data to be read
  * @param  ReadAddr: Read start address
  * @param  Size: Size of data to read
  * @retval QSPI memory status
  */
static uint8_t BSP_QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)
{
  QSPI_CommandTypeDef s_command;

  if ((ReadAddr >= QSPIInformation.FlashSize) || ((ReadAddr + Size) > QSPIInformation.FlashSize))
  {
    /* address out of range */
    return(QSPI_ERROR);
  }

  /* Initialize the read command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = QUAD_OUT_FAST_READ_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.Address           = ReadAddr;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = QSPIInformation.numReadDummy;     //FLASH_DUMMY_CYCLES_READ_QUAD;
  s_command.NbData            = Size;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(&hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  Writes an amount of data to the QSPI memory.
  * @param  pData: Pointer to data to be written
  * @param  WriteAddr: Write start address
  * @param  Size: Size of data to write
  * @retval QSPI memory status
  */
static uint8_t BSP_QSPI_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
{
  QSPI_CommandTypeDef s_command;
  uint32_t end_addr, current_size, current_addr;

  if ((WriteAddr >= QSPIInformation.FlashSize) || ((WriteAddr + Size) > QSPIInformation.FlashSize))
  {
    /* address out of range */
    return(QSPI_ERROR);
  }

  /* Calculation of the size between the write address and the end of the page */
  current_addr = 0;

  while (current_addr <= WriteAddr)
  {
    current_addr += QSPIInformation.ProgPageSize;
  }

  current_size = current_addr - WriteAddr;

  /* Check if the size of the data is less than the remaining place in the page */
  if (current_size > Size)
  {
    current_size = Size;
  }

  /* Initialize the address variables */
  current_addr = WriteAddr;
  end_addr = WriteAddr + Size;

  /* Initialize the program command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = QUAD_IN_FAST_PROG_CMD;
  s_command.AddressMode       = QSPIInformation.numAddressLine;   //QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  do
  {
    s_command.Address = current_addr;
    s_command.NbData  = current_size;

    /* Enable write operations */
    if (QSPI_WriteEnable() != QSPI_OK)
    {
      return QSPI_ERROR;
    }

    /* Configure the command */
    if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return QSPI_ERROR;
    }

    /* Transmission of the data */
    if (HAL_QSPI_Transmit(&hqspi, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return QSPI_ERROR;
    }

    /* Configure automatic polling mode to wait for end of program */
    if (QSPI_AutoPollingMemReady(HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != QSPI_OK)
    {
      return QSPI_ERROR;
    }

    /* Update the address and size variables for next page programming */
    current_addr += current_size;
    pData += current_size;
    current_size = ((current_addr + QSPIInformation.ProgPageSize) > end_addr) ? (end_addr - current_addr) : QSPIInformation.ProgPageSize;

  }
  while (current_addr < end_addr);

  return QSPI_OK;
}

/**
  * @brief  Erases the specified 32K block of the QSPI memory.
  * @param  BlockAddress: Block address to erase
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_Erase_32KBlock(uint32_t BlockAddress)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the erase command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = SECTOR_32K_ERASE_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.Address           = BlockAddress;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Enable write operations */
  if (QSPI_WriteEnable() != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  /* Send the command */
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait for end of erase */
  if (QSPI_AutoPollingMemReady(FLASH_SUBSECTOR_ERASE_MAX_TIME) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}



/**
  * @brief  Erases the specified block of the QSPI memory.
  * @param  BlockAddress: Block address to erase
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_Erase_Block(uint32_t BlockAddress)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the erase command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = SECTOR_ERASE_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE; //QSPIInformation.numAddressLine;
  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.Address           = BlockAddress;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Enable write operations */
  if (QSPI_WriteEnable() != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  /* Send the command */
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait for end of erase */
  if (QSPI_AutoPollingMemReady(FLASH_SUBSECTOR_ERASE_MAX_TIME) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  Erases the specified sub-block of the QSPI memory.
  * @param  BlockAddress: Block address to erase
  * @retval QSPI memory status
  */
static uint8_t BSP_QSPI_Erase_SubBlock(uint32_t BlockAddress)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the erase command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;

  if (QSPIModel != sFLASH_UNKNOWN)
  {
    s_command.Instruction       = SECTOR_4K_ERASE_CMD;
    s_command.AddressMode       = QSPI_ADDRESS_1_LINE; //QSPIInformation.numAddressLine; //QSPI_ADDRESS_1_LINE;
    s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
    s_command.Address           = BlockAddress;
    s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DataMode          = QSPI_DATA_NONE;
    s_command.DummyCycles       = 0;
    s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
    s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    /* Enable write operations */
    if (QSPI_WriteEnable() != QSPI_OK)
    {
      return QSPI_ERROR;
    }

    /* Send the command */
    if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return QSPI_ERROR;
    }

    /* Configure automatic polling mode to wait for end of erase */
    if (QSPI_AutoPollingMemReady(FLASH_SUBSECTOR_ERASE_MAX_TIME) != QSPI_OK)
    {
      return QSPI_ERROR;
    }
  }

  return QSPI_OK;
}

#ifdef USE_ERASE_CHIP
/**
  * @brief  Erases the entire QSPI memory.
  * @retval QSPI memory status
  */
static uint8_t BSP_QSPI_Erase_Chip(void)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the erase command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;

  if ((QSPIModel == QSPI_N25Q512A))
    s_command.Instruction       = BULK_ERASE_CMD;
  else if (QSPIModel == QSPI_MT25QL01G)
    s_command.Instruction       = MT25QL01G_BULK_ERASE_CMD;
  else if (QSPIModel == QSPI_MT25Q512A)
    s_command.Instruction       = MT25QL01G_BULK_ERASE_QUAD_CMD;

  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Enable write operations */
  if (QSPI_WriteEnable() != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  /* Send the command */
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait for end of erase */
  if (QSPI_AutoPollingMemReady(N25Q512A_BULK_ERASE_MAX_TIME) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}
#endif

/**
  * @brief
  * @retval
  */
static uint8_t BSP_QSPI_SetQuadMode(void)
{
  uint8_t statusReg;
  uint8_t configReg;

  if ((QSPIModel == sFLASH_W25Q80DV) || (QSPIModel == sFLASH_W25Q016DV))
  {
    BSP_QSPI_GetStatusReg1(&statusReg);
    BSP_QSPI_GetConfigReg1(&configReg);

    /* Update non-volatile configuration register 1 (CR1) */
    if (!TestBit(configReg, 1) || !TestBit(configReg, 6) || !TestBit(configReg, 7))
    {
      QSPI_CommandTypeDef s_command;
      QSPI_AutoPollingTypeDef s_config;

      /* Initialize the read flag status register command */
      s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
      s_command.Instruction       = WRITE_CFG_REG_CMD;
      s_command.AddressMode       = QSPI_ADDRESS_NONE;
      s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
      s_command.DataMode          = QSPI_DATA_1_LINE;
      s_command.DummyCycles       = 0;
      s_command.NbData            = 2;
      s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
      s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
      s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

      Set_Bit(configReg, 1);
      //Set_Bit(configReg, 6);
      ResetBit(configReg, 6);
      uint8_t data[] = { statusReg, configReg };

      QSPI_WriteEnable();

      if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
      {
        return QSPI_ERROR;
      }

      /* Transmission of the data */
      if (HAL_QSPI_Transmit(&hqspi, data, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
      {
        return QSPI_ERROR;
      }

      s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
      s_command.Instruction       = READ_STATUS_REG_CMD;
      s_command.AddressMode       = QSPI_ADDRESS_NONE;
      s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
      s_command.DataMode          = QSPI_DATA_1_LINE;
      s_command.DummyCycles       = 0;
      s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
      s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
      s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

      /* Configure automatic polling mode to wait for write enabling */
      s_config.Match           = 0;
      s_config.Mask            = FLASH_BIT_SR_WREN | FLASH_BIT_SR_WIP;
      s_config.MatchMode       = QSPI_MATCH_MODE_AND;
      s_config.StatusBytesSize = 1;
      s_config.Interval        = 0x10;
      s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

      if (HAL_QSPI_AutoPolling(&hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
      {
        BSP_QSPI_GetStatusReg1(&statusReg);
        return QSPI_ERROR;
      }
    }

    BSP_QSPI_GetStatusReg1(&statusReg);
  }
  else if ((QSPIModel == sFLASH_IS25LP016) || (QSPIModel == sFLASH_IS25LP080))
  {
    QSPI_CommandTypeDef     s_command;
    QSPI_AutoPollingTypeDef s_config;

    BSP_QSPI_GetStatusReg1(&statusReg);

    /* Update non-volatile configuration register  (CR) */
    if (!TestBit(statusReg, 6))
    {
      /* Initialize the read flag status register command */
      s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
      s_command.Instruction       = WRITE_CFG_REG_CMD;
      s_command.AddressMode       = QSPI_ADDRESS_NONE;
      s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
      s_command.DataMode          = QSPI_DATA_1_LINE;
      s_command.DummyCycles       = 0;
      s_command.NbData            = 1;
      s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
      s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
      s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

      Set_Bit(statusReg, 6);  /* set bit QE as in data sheet Table 6.1 pag 15  */

      QSPI_WriteEnable();

      if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
      {
        return QSPI_ERROR;
      }

      /* Transmission of the data */
      if (HAL_QSPI_Transmit(&hqspi, (uint8_t*)&statusReg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
      {
        return QSPI_ERROR;
      }

      s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
      s_command.Instruction       = READ_STATUS_REG_CMD;
      s_command.AddressMode       = QSPI_ADDRESS_NONE;
      s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
      s_command.DataMode          = QSPI_DATA_1_LINE;
      s_command.DummyCycles       = 0;
      s_command.NbData            = 1;
      s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
      s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
      s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

      /* Configure automatic polling mode to wait for write enabling */
      s_config.Match           = 0;
      s_config.Mask            = FLASH_BIT_SR_WREN | FLASH_BIT_SR_WIP;
      s_config.MatchMode       = QSPI_MATCH_MODE_AND;
      s_config.StatusBytesSize = 1;
      s_config.Interval        = 0x10;
      s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

      if (HAL_QSPI_AutoPolling(&hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
      {
        return QSPI_ERROR;
      }
    }
  }
  else if (QSPIModel == sFLASH_SST26VF016B)
  {
    QSPI_CommandTypeDef     s_command;
    QSPI_AutoPollingTypeDef s_config;

    BSP_QSPI_GetStatusReg1(&statusReg);
    BSP_QSPI_GetConfigReg1(&configReg);

    /* Update non-volatile configuration register  (CR) */
    if (!TestBit(configReg, 1) || TestBit(configReg, 7))
    {
      /* Initialize the read flag status register command */
      s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
      s_command.Instruction       = WRITE_CFG_REG_CMD;
      s_command.AddressMode       = QSPI_ADDRESS_NONE;
      s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
      s_command.DataMode          = QSPI_DATA_1_LINE;
      s_command.DummyCycles       = 0;
      s_command.NbData            = 2;
      s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
      s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
      s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

      Set_Bit(configReg, 1);
      ResetBit(configReg, 7);
      uint8_t data[] = { statusReg, configReg };

      QSPI_WriteEnable();

      if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
      {
        return QSPI_ERROR;
      }

      /* Transmission of the data */
      if (HAL_QSPI_Transmit(&hqspi, data, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
      {
        return QSPI_ERROR;
      }

      s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
      s_command.Instruction       = READ_STATUS_REG_CMD;
      s_command.AddressMode       = QSPI_ADDRESS_NONE;
      s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
      s_command.DataMode          = QSPI_DATA_1_LINE;
      s_command.DummyCycles       = 0;
      s_command.NbData            = 1;
      s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
      s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
      s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

      /* Configure automatic polling mode to wait for write enabling */
      s_config.Match           = 0;
      s_config.Mask            = FLASH_BIT_SR_WREN | FLASH_BIT_SR_WIP;
      s_config.MatchMode       = QSPI_MATCH_MODE_AND;
      s_config.StatusBytesSize = 1;
      s_config.Interval        = 0x10;
      s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

      if (HAL_QSPI_AutoPolling(&hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
      {
        return QSPI_ERROR;
      }
    }

    /* unprotect the device */
    /* Enable write operations */
    s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction       = GLOBAL_BLOCK_PROTECTION_UNLOCK_CMD;
    s_command.AddressMode       = QSPI_ADDRESS_NONE;
    s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DataMode          = QSPI_DATA_NONE;
    s_command.DummyCycles       = 0;
    s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
    s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return QSPI_ERROR;
    }

    /* Configure automatic polling mode to wait for write enabling */
    s_config.Match           = 0;
    s_config.Mask            = FLASH_SR_WREN | FLASH_SR_WIP;
    s_config.MatchMode       = QSPI_MATCH_MODE_AND;
    s_config.StatusBytesSize = 1;
    s_config.Interval        = 0x10;
    s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

    s_command.Instruction    = READ_STATUS_REG_CMD;
    s_command.DataMode       = QSPI_DATA_1_LINE;

    if (HAL_QSPI_AutoPolling(&hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return QSPI_ERROR;
    }

    return QSPI_OK;
  }
  else
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}


/**
  * @brief  Reads current status register 1 of the QSPI memory.
  * @retval QSPI status register 1
  */
static uint8_t BSP_QSPI_GetStatusReg1(uint8_t *reg)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the read flag status register command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = READ_STATUS_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(&hqspi, reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  Reads current configuration register 1 of the QSPI memory.
  * @retval QSPI configuration register 1
  */
static uint8_t BSP_QSPI_GetConfigReg1(uint8_t *reg)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the read flag status register command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = READ_CFG_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(&hqspi, reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

/**
  * @brief  Reads current status of the QSPI memory.
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_GetStatus(void)
{
  QSPI_CommandTypeDef s_command;
  uint8_t reg;

  /* Initialize the read flag status register command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = READ_FLAG_STATUS_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(&hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Check the value of the register */
  if ((reg & (FLASH_FSR_PRERR | FLASH_FSR_VPPERR | FLASH_FSR_PGERR | FLASH_FSR_ERERR)) != 0)
  {
    return QSPI_ERROR;
  }
  else if ((reg & (FLASH_FSR_PGSUS | FLASH_FSR_ERSUS)) != 0)
  {
    return QSPI_SUSPENDED;
  }
  else if ((reg & FLASH_FSR_READY) != 0)
  {
    return QSPI_OK;
  }
  else
  {
    return QSPI_BUSY;
  }
}

/**
  * @brief  Return the configuration of the QSPI memory.
  * @param  pInfo: pointer on the configuration structure
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_GetInfo(QSPI_Info* pInfo)
{
  /* Configure the structure with the memory configuration */
  if (QSPIModel == sFLASH_W25Q80DV)
  {
    pInfo->FlashSize            = FLASH_080_SIZE;
    pInfo->EraseSectorSize      = FLASH_SUBSECTOR_SIZE;
    pInfo->EraseSectorsNumber   = (FLASH_080_SIZE / FLASH_SUBSECTOR_SIZE);
    pInfo->ProgPageSize         = FLASH_PAGE_SIZE;
    pInfo->ProgPagesNumber      = (FLASH_080_SIZE / FLASH_PAGE_SIZE);
    pInfo->numAddressLine       = QSPI_ADDRESS_1_LINE;
    pInfo->numReadDummy         = FLASH_DUMMY_CYCLES_READ_QUAD;
    pInfo->transactionAddress   = FLASH_TRANSACTION_080_ADD;
    pInfo->numTransactionSector = FLASH_TRANSACTION_NUM_SEC;
    return QSPI_OK;
  }
  else
  {
    if (QSPIModel == sFLASH_SST26VF016B)
    {
      pInfo->FlashSize            = FLASH_016_SIZE;
      pInfo->EraseSectorSize      = FLASH_SUBSECTOR_SIZE;
      pInfo->EraseSectorsNumber   = (FLASH_016_SIZE / FLASH_SUBSECTOR_SIZE);
      pInfo->ProgPageSize         = FLASH_PAGE_SIZE;
      pInfo->ProgPagesNumber      = (FLASH_016_SIZE / FLASH_PAGE_SIZE);
      pInfo->numAddressLine       = QSPI_ADDRESS_4_LINES;
      pInfo->numReadDummy         = FLASH_DUMMY_CYCLES_READ_QUAD;
      pInfo->transactionAddress   = FLASH_TRANSACTION_016_ADD;
      pInfo->numTransactionSector = FLASH_TRANSACTION_NUM_SEC;
      return QSPI_OK;
    }
    else
    {
      if (QSPIModel == sFLASH_IS25LP016)
      {
        pInfo->FlashSize            = FLASH_016_SIZE;
        pInfo->EraseSectorSize      = FLASH_SUBSECTOR_SIZE;
        pInfo->EraseSectorsNumber   = (FLASH_016_SIZE / FLASH_SUBSECTOR_SIZE);
        pInfo->ProgPageSize         = FLASH_PAGE_SIZE;
        pInfo->ProgPagesNumber      = (FLASH_016_SIZE / FLASH_PAGE_SIZE);
        pInfo->numAddressLine       = QSPI_ADDRESS_1_LINE;
        pInfo->numReadDummy         = FLASH_DUMMY_CYCLES_READ_QUAD;
        pInfo->transactionAddress   = FLASH_TRANSACTION_016_ADD;
        pInfo->numTransactionSector = FLASH_TRANSACTION_NUM_SEC;
        return QSPI_OK;
      }
      else
      {
        if (QSPIModel == sFLASH_IS25LP080)
        {
          pInfo->FlashSize            = FLASH_080_SIZE;
          pInfo->EraseSectorSize      = FLASH_SUBSECTOR_SIZE;
          pInfo->EraseSectorsNumber   = (FLASH_080_SIZE / FLASH_SUBSECTOR_SIZE);
          pInfo->ProgPageSize         = FLASH_PAGE_SIZE;
          pInfo->ProgPagesNumber      = (FLASH_080_SIZE / FLASH_PAGE_SIZE);
          pInfo->numAddressLine       = QSPI_ADDRESS_1_LINE;
          pInfo->numReadDummy         = FLASH_DUMMY_CYCLES_READ_QUAD;
          pInfo->transactionAddress   = FLASH_TRANSACTION_080_ADD;
          pInfo->numTransactionSector = FLASH_TRANSACTION_NUM_SEC;
          return QSPI_OK;
        }
      }
    }
  }

  return QSPI_ERROR;
}

static uint8_t BSP_QSPI_GetDeviceId(uint8_t* pIdreg, uint8_t len)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the read flag status register command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = READ_ID_CMD2;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = (uint32_t)len;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(&hqspi, pIdreg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

//-----------------------------------------------------------------
// Fine routine gestione FLASH a basso livello
//-----------------------------------------------------------------

//-----------------------------------------------------------------
/*
  FlashWrite (Scrive su Flash)

  Valori passati:
    u32_t   indirizzo;
    u8_t *    buffer (buffer da scrivere);
    u32_t   lbuffer (quanti byte scrivere);
    u32_t   lbufferbackup (per ridurre i tempi indicare quanto salvare di ciascun settore)

  Valore di ritorno:
    u8_t  0x00 tutto OK
        0xFF in caso di errore;
*/
//-----------------------------------------------------------------
unsigned char FlashWrite(unsigned int Address, unsigned char *BuffWr, unsigned int LBuffWr, unsigned int LBuffBackup)
{
  // Variabile di controllo operazione su flash
  unsigned char RetErase, RetWrite;
  // Contatore di quanti byte sono stati scritti
  unsigned int BytesWritten, BytesToProcess;

  // Calcolo settore di partenza e settore di fine
  unsigned int StartSector = (Address / FlashDimBlk);
  unsigned int EndSector = ((Address + LBuffWr - 1) / FlashDimBlk);  //<------- aggiunto  - 1

  RetErase = 0x00;
  RetWrite = 0x00;

  // Controllo se il semaforo è libero
  if(osSemaphoreAcquire(FlashSemaphoreHandle, 1000) == osOK)
  {
    // Ciclo di cancellazione settori
    while (StartSector <= EndSector)
    {
      // Cancello il settore
      RetErase = BSP_QSPI_Erase_SubBlock(StartSector * FlashDimBlk);

      if (RetErase != QSPI_OK)
      {
        // Rilascio il semaforo
        osSemaphoreRelease(FlashSemaphoreHandle);
        return (0xFF);
      }

      // Incremento
      StartSector++;
    }

    // Ciclo di scrittura
    // Contatore di quanti byte sono stati scritti
    BytesWritten = 0;
    // Calcolo quanti byte devo scrivere al primo giro
    BytesToProcess = (LBuffWr <= FlashDimBlk) ? LBuffWr : FlashDimBlk;

    while (BytesWritten < LBuffWr)
    {
      // Scrivo
      RetWrite = BSP_QSPI_Write((BuffWr + BytesWritten), Address + BytesWritten, BytesToProcess);

      if (RetWrite != QSPI_OK)
      {
        // Rilascio il semaforo
        osSemaphoreRelease(FlashSemaphoreHandle);
        return(0xFF);
      }

      // Aggiorno le variabili per il prossimo giro
      BytesWritten += BytesToProcess;
      BytesToProcess = ((LBuffWr - BytesWritten) <= FlashDimBlk) ? (LBuffWr - BytesWritten) : FlashDimBlk;
    }

    // Rilascio il semaforo
    osSemaphoreRelease(FlashSemaphoreHandle);
  }

  return(RetWrite);
}
//-----------------------------------------------------------------

/*
  FlashFatWrite (Scrive FAT su Flash cancellando sub-sector da 4K)

  Valori passati:
    u32_t   indirizzo;
    u8_t *    buffer (buffer da scrivere);
    u32_t   lbuffer (quanti byte scrivere);

  Valore di ritorno:
    u8_t  0x00 tutto OK
        0xFF in caso di errore;
*/
//-----------------------------------------------------------------
unsigned char FlashFatWrite(unsigned int Address, unsigned char *BuffWr, unsigned int LBuffWr, unsigned int LBuffBackup)
{
  // Variabile di controllo operazione su flash
  unsigned char RetErase = 0x00, RetWrite = 0x00;
  unsigned char testErase[16];

  // Ciclo di scrittura
  // Contatore di quanti byte sono stati scritti
  unsigned int BytesWritten = 0;
  // Calcolo quanti byte devo scrivere al primo giro
  unsigned int BytesToProcess = (LBuffWr <= FlashDimBlk) ? LBuffWr : FlashDimBlk;

  // Calcolo settore di partenza e settore di fine
  unsigned int StartSector = (Address / FlashDimBlk);
  unsigned int EndSector = ((Address + LBuffWr - 1) / FlashDimBlk);


  // Controllo se il semaforo è libero
  if(osSemaphoreAcquire(FlashSemaphoreHandle, 1000) == osOK)
  {
    // Ciclo di cancellazione settori
    while (StartSector <= EndSector)
    {
      // Cancello il sub-settore
      RetErase = BSP_QSPI_Erase_SubBlock(StartSector * FlashDimBlk);

      if (RetErase != QSPI_OK)
      {
        // Rilascio il semaforo
        osSemaphoreRelease(FlashSemaphoreHandle);
        return (0xFF);
      }

      // Incremento
      RetErase = BSP_QSPI_Read(testErase, (StartSector * FlashDimBlk), sizeof(testErase));

      if ((RetErase != QSPI_OK) || (testErase[0] != 0xFF) || (testErase[1] != 0xFF))
      {
        // Rilascio il semaforo
        osSemaphoreRelease(FlashSemaphoreHandle);
        return(0xFF);
      }

      StartSector++;
    }

    while (BytesWritten < LBuffWr)
    {
      // Scrivo
      RetWrite = BSP_QSPI_Write((BuffWr + BytesWritten), Address + BytesWritten, BytesToProcess);

      if (RetWrite != QSPI_OK)
      {
        // Rilascio il semaforo
        osSemaphoreRelease(FlashSemaphoreHandle);
        return (0xFF);
      }

      // Aggiorno le variabili per il prossimo giro
      BytesWritten += BytesToProcess;
      BytesToProcess = ((LBuffWr - BytesWritten) <= FlashDimBlk) ? (LBuffWr - BytesWritten) : FlashDimBlk;
    }

    // Rilascio il semaforo
    osSemaphoreRelease(FlashSemaphoreHandle);
  }

  return(RetWrite);
}
//-----------------------------------------------------------------


//-----------------------------------------------------------------
/*
  FlashRead (Legge da Flash)

  Valori passati:
    u8_t    indirizzo pagina;
    u8_t *    buffer (buffer in cui inserire i dati letti);
    u32_t   lbuffer (quanti byte leggere);
  Valore di ritorno:
    u8_t  0xFF in caso di errore;
            0x00 se tutto bene;
*/
//-----------------------------------------------------------------
unsigned char FlashRead(unsigned int Address, unsigned char *BuffRd, unsigned int LBuffRd)
{
  // Variabile di controllo operazione su flash
  unsigned char RetRead = 0x00;

  // La lettura fa fatta a "sottosettori"
  // Contatore di quanti byte sono stati letti
  unsigned int BytesRead = 0;
  // Calcolo quanti byte devo leggere al primo giro
  unsigned int BytesToProcess = (LBuffRd <= FlashDimBlk) ? LBuffRd : FlashDimBlk;
  
  // Controllo se il semaforo è libero
  if(osSemaphoreAcquire(FlashSemaphoreHandle, 1000) == osOK)
  {
    while (BytesRead < LBuffRd)
    {
      RetRead = BSP_QSPI_Read((BuffRd + BytesRead), Address + BytesRead, BytesToProcess);
      
      if (RetRead != QSPI_OK)
      {
        // Rilascio il semaforo
        osSemaphoreRelease(FlashSemaphoreHandle);
        return(0xFF);
      }

      // Aggiorno le variabili per il prossimo giro
      BytesRead += BytesToProcess;
      BytesToProcess = ((LBuffRd - BytesRead) <= FlashDimBlk) ? (LBuffRd - BytesRead) : FlashDimBlk;
    }

    // Rilascio il semaforo
    osSemaphoreRelease(FlashSemaphoreHandle);
  }

  return(RetRead);
}
//-----------------------------------------------------------------

//-----------------------------------------------------------------
//  FlashSaveConfiguration
//-----------------------------------------------------------------
unsigned char FlashSaveConfiguration(unsigned char *BuffWr, unsigned int LBuffWr)
{
  unsigned char ErrFlash = 0U;

#ifdef DA_FARE
  // Termino il file
  *(BuffWr + LBuffWr) = 0x00;
  /* quindi la lunghezza si incrementa di uno */
  LBuffWr++;
  char          fullFileName[16] = {'\0'};

  ErrFlash = (unsigned char)TRUE;
  /* il nome del file sarà del tipo:  xC0000000.dat */
  sprintf((char *)fullFileName, "x%08X.", AddressConfiguration);
  /*         destination       source */
  strcat((char *)fullFileName, (char *)FATCONF_EXT);
  unsigned int wLen = FlashFatWriteFile(fullFileName, (char *)BuffWr, LBuffWr, (uint8_t*)FATCONF_EXT, FATFS_DEVICE);

  if (wLen == LBuffWr)
  {
    // writing successfully
    ErrFlash = (unsigned char)FALSE;
  }

#endif
  return(ErrFlash);
}
//-----------------------------------------------------------------



//-----------------------------------------------------------------
/*
  FlashFormat (formatta la flash)

  Valori passati:
    nessuno
  Valore di ritorno:
    u8_t  0xFF in caso di errore;
            0x00 se tutto bene;
*/
//-----------------------------------------------------------------
unsigned char FlashFormat(void)
{
  unsigned char RetVal;
  unsigned int  sector;
  uint32_t address;

  RetVal = address = 0;

#ifdef USE_ERASE_CHIP
  RetVal = BSP_QSPI_Erase_Chip();
#else
#ifndef ERASE_CHIP_WITH_SUBSECTORS

  /*we have 1024 64K sector (512 64K sector from 0x0200.0000=OFFSET_FLASH_ADDRESS to 0x03FF.FFFF) */
  for (sector = 0; sector < FLASH_FAT_NUM_64K_SECTOR; sector++)
  {
    address = OFFSET_FLASH_ADDRESS + (sector * (uint32_t)FLASH_SECTOR_64K_SIZE);
    RetVal = BSP_QSPI_Erase_Block(address);

    if (RetVal != QSPI_OK)
    {
      break;
    }
    else
    {
      refreshWD();
    }
  }

#else

  /*we have 16384 4K subsector. For the FAT we use  8192 4Ksubsector from 0x0200.0000=OFFSET_FLASH_ADDRESS to 0x03FF.FFFF */
  for (sector = 0; sector < (unsigned int)FLASH_FAT_NUM_SECTOR; sector++)
  {
    address = OFFSET_FLASH_ADDRESS + (sector * (uint32_t)0x1000);
    RetVal = BSP_QSPI_Erase_SubBlock(address);

    if (RetVal != QSPI_OK)
    {
      break;
    }
    else
    {
      refreshWD();
    }
  }

#endif
#endif
  return(RetVal);
}

/*
  FlashFormat (formatta la parte di flash dedicata alla configurazione 0x0000.0000-1FFF.FFFF)

  Valori passati:
    nessuno
  Valore di ritorno:
    u8_t  0xFF in caso di errore;
            0x00 se tutto bene;
*/
//-----------------------------------------------------------------
unsigned char FlashConfigFormat(void)
{
  unsigned char RetVal;
  unsigned int  sector;
  uint32_t address;

  RetVal = address = 0;

#ifndef ERASE_CHIP_WITH_SUBSECTORS

  /*we have 1024 64K sector (512 64K sector from 0x0200.0000=OFFSET_FLASH_ADDRESS to 0x03FF.FFFF) */
  for (sector = 0; sector < FLASH_FAT_NUM_64K_SECTOR; sector++)
  {
    address = (uint32_t)0x00000000 + (sector * (uint32_t)FLASH_SECTOR_64K_SIZE);
    RetVal = BSP_QSPI_Erase_Block(address);

    if (RetVal != QSPI_OK)
    {
      break;
    }
    else
    {
      refreshWD();
    }
  }

#endif
  return(RetVal);
}


//-----------------------------------------------------------------
//  FATInit
//-----------------------------------------------------------------
void FATInit(void)
{
#ifdef DA_FARE
  volatile unsigned char ErrFlash, AzzFAT;
  unsigned short LookupValue;
  char          fullFileName[16] = {'\0'};
  unsigned int  readByte, byteToRead;

  LookupValue = 0;
  ErrFlash = 0;
  /* il nome del file sarà del tipo:  x00000000.dat */
  sprintf((char *)fullFileName, "x%08X.", AddressFAT);
  /*         destination       source */
  strcat((char *)fullFileName, (char *)FATCONF_EXT);

  if (FlashFatReadFile(fullFileName, NULL, &byteToRead, (uint8_t*)NULL, FAT_DEVICE_FLASH))
  {
    readByte = (unsigned int)FlashFatReadFile(fullFileName, (char *)&FAT, (unsigned int*)&byteToRead, NULL,  FAT_DEVICE_FLASH);

    if ((readByte == (unsigned int)0) || ( readByte != byteToRead))
    {
      ErrFlash = (unsigned char)0xFF;
    }
  }
  else
  {
    /* file doesn't exist */
    ErrFlash = (unsigned char)0xFF;
  }

#endif
}
//-----------------------------------------------------------------

//-----------------------------------------------------------------
//  FlashInit
//-----------------------------------------------------------------
void FlashInit(void)
{

  // Termino l'inizializzazione cominciata in MX_QUADSPI_Init()
  // La funzione MX_QUADSPI_Init() viene generata in automatico, quindi aggiungo qui le altre funzioni

  // Leggo l'ID della flash
  BSP_QSPI_GetDeviceId((uint8_t*)FlashID, sizeof(FlashID));

  // Controllo se è una flash gestita
  if (memcmp(FlashID, "\xEF\x40\x14", 3) == 0)     /* "\xEF\x40\x14"  WINBOND - W25 - 8Mb */
  {
    QSPIModel = sFLASH_W25Q80DV;
    /* QUADSPI parameter configuration: number of address bit */
    hqspi.Init.FlashSize = FLASH_080_ADDRESS_SIZE;
  }
  else
  {
    if (memcmp(FlashID, "\xEF\x40\x15", 3) == 0)    /* "\xEF\x40\x15"  W25Q16JV WINBOND - W25 - 16Mb */
    {
      QSPIModel = sFLASH_W25Q016DV;
      /* QUADSPI parameter configuration: number of address bit */
      hqspi.Init.FlashSize = FLASH_016_ADDRESS_SIZE;
    }
    else
    {
      if (memcmp(FlashID, "\x9D\x60\x14", 3) == 0)    /* "\x9D\x60\x14"  S25LP080D ISSI - LP - 8Mb */
      {
        QSPIModel = sFLASH_IS25LP080;
        /* QUADSPI parameter configuration: number of address bit  */
        hqspi.Init.FlashSize = FLASH_080_ADDRESS_SIZE;
      }
      else
      {
        if (memcmp(FlashID, "\x9D\x60\x15", 3) == 0)    /* "\x9D\x60\x15"  S25LP016D ISSI - LP - 16Mb */
        {
          QSPIModel = sFLASH_IS25LP016;
          /* QUADSPI parameter configuration: number of address bit  */
          hqspi.Init.FlashSize = FLASH_016_ADDRESS_SIZE;
        }
        else
        {
          if (memcmp(FlashID, "\xBF\x26\x41", 3) == 0)    /* "\xBF\x26\x41"  sFLASH_SST26VF016B Microchip - 26V - 16Mb */
          {
            QSPIModel = sFLASH_SST26VF016B;
            /* QUADSPI parameter configuration: number of address bit  */
            hqspi.Init.FlashSize = FLASH_016_ADDRESS_SIZE;
          }
          else
          {
            QSPIModel = sFLASH_UNKNOWN;
          }
        }
      }
    }
  }

  if (QSPIModel != sFLASH_UNKNOWN)
  {
    if (HAL_QSPI_Init(&hqspi) != HAL_OK) //  reinizializzazione modulo QSPI di STM32F765
    {
      // Flash in errore, blocco tutto
      while(1);
    }

    // Assegno alla struttura Info i valori relativi alla QSPI in uso
    BSP_QSPI_GetInfo(&QSPIInformation);

    FlashDimBlk = QSPIInformation.EraseSectorSize; // Dimensione di un sotto-settore
    FlashDimPag = QSPIInformation.ProgPageSize; // Dimensione di una pagina

    /* uso la flash in QSPI mode */
    BSP_QSPI_SetQuadMode();

  }
  else
  {
    // Flash non gestita, blocco tutto
    //while(1);
  }
}

/**
  * @brief  Return if  QSPI flash has been detected.
  * @param  none
  * @retval QSPI detected status
  */
uint8_t sFLASH_Detected(void)
{
  /* Configure the structure with the memory configuration */
  if ((QSPIModel == sFLASH_W25Q80DV) || (QSPIModel == sFLASH_W25Q016DV) ||
      (QSPIModel == sFLASH_IS25LP016) || (QSPIModel == sFLASH_SST26VF016B) || (QSPIModel == sFLASH_IS25LP080))
  {
    return QSPI_OK;
  }
  else
  {
    return QSPI_ERROR;
  }

}

/**
  * @brief  Erases the specified sub-block of the QSPI memory.
  * @param  BlockAddress: Block address to erase
  * @retval QSPI memory status
  */
uint8_t raw_sFLASH_Erase_SubBlock(uint32_t BlockAddress)
{
  return(BSP_QSPI_Erase_SubBlock(BlockAddress));
}

/**
  * @brief  Writes an amount of data to the QSPI memory.
  * @param  pData: Pointer to data to be written
  * @param  WriteAddr: Write start address
  * @param  Size: Size of data to write
  * @retval QSPI memory status
  */
uint8_t raw_sFLASH_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
{
  refreshWD();
  return(BSP_QSPI_Write(pData, WriteAddr, Size));
}

/**
  * @brief  Reads an amount of data to the QSPI memory.
  * @param  pData: Pointer where data read is stored
  * @param  ReadAddr: Read start address
  * @param  Size: Size of data to read
  * @retval QSPI memory status
  */
uint8_t raw_sFLASH_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)
{
  return(BSP_QSPI_Read(pData, ReadAddr, Size));
}

/**
  * @brief  Erase data area in QSPI-Flash.
  * @retval QSPI memory status
  */
uint8_t raw_sFLASH_EraseDataArea(void)
{
  uint32_t      qspiAdd;
  uint8_t       result;
  osSemaphoreId semaphoreFat;

  semaphoreFat = getFatSemaphore();

  if(osSemaphoreAcquire(semaphoreFat, portMAX_DELAY) == osOK)
  {
    for (qspiAdd = FLASH_FAT_SIZE; qspiAdd < QSPIInformation.FlashSize; qspiAdd += FLASH_SUBSECTOR_SIZE)
    {
      result = raw_sFLASH_Erase_SubBlock(qspiAdd);
      refreshWD();

      if (result != QSPI_OK)
      {
        break;
      }
    }
  }

  osSemaphoreRelease(semaphoreFat);
  return(result);
}

/**
  * @brief  Erase data area in QSPI-Flash.
  * @param  none
  * @retval char*: pointer to start address data area in QSPI flash
  */
char* pQspiDataArea(void)
{
  return((char*)FLASH_FAT_SIZE);
}


uint8_t getStatusReg(void)
{
  uint8_t statusReg;

  BSP_QSPI_GetStatusReg1(&statusReg);
  return (statusReg);
}

uint8_t getConfigReg(void)
{
  uint8_t configReg;

  BSP_QSPI_GetConfigReg1(&configReg);
  return (configReg);
}


/**
  * @brief  Get address transaction start area
  * @param  none
  * @retval uint32_t: pointer to start address transaction data area in QSPI flash
  */
uint32_t getTransactionAddress(void)
{
  return (QSPIInformation.transactionAddress);
}

//-----------------------------------------------------------------


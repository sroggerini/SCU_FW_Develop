/**
* @file        ExtFlash.c
*
* @brief       manager for SPI Flash - Implementation for stm32f4xx device
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: ExtFlash_stm32f4.c 599 2024-09-26 07:03:24Z stefano $
*
*     $Revision: 599 $
*
*     $Author: 
*
*     $Date: 2024-09-26 09:03:24 +0200 (gio, 26 set 2024) $
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_dma.h"
#include "ff.h"
#include "ffconf.h"
#include "main.h"
#include "ExtFlash.h"
#include "cmsis_os.h"
#include "wrapper.h"   

/*
*********************************** AESYS ************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
******************************************************************************
*/
// Struttura FAT simulata su QSPI
struct StructFAT FAT;

// Tipo di Serial Flash
unsigned char sFLASH_Model;

// Info sulla Serial FLASH in uso
sFLASH_Info sFLASH_Information;

unsigned char FlashID[10];
unsigned char WriteOptimization;

// Dettagli dimensioni flash QSPI
unsigned int FlashDimBlk, FlashDimPag;

// Puntatore al buffer di backup
unsigned int  PBackupQSPI;


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
// Struttura per pilotare la sFLASH
extern SPI_HandleTypeDef hspi1;

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
static uint8_t sFLASH_WriteEnable               (void);
static uint8_t sFLASH_WaitForWriteInProgress   (uint32_t Timeout);
static uint8_t sFLASH_Read                      (uint8_t* pData, uint32_t ReadAddr, uint32_t Size);
static uint8_t sFLASH_Write                     (uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
static uint8_t sFLASH_GetStatusReg1             (uint8_t *reg);
static uint8_t sFLASH_GetConfigReg1             (uint8_t *reg);
static uint8_t sFLASH_Erase_SubBlock            (uint32_t BlockAddress);

//-----------------------------------------------------------------
// Inizio routine gestione FLASH a basso livello
//-----------------------------------------------------------------

/**
  * @brief  This function send a byte using the SPI, checking the result
  * @retval Result as OK or ERROR
  */
static uint8_t sFLASH_SendByte(uint8_t Data)
{
  /* Transmit byte via SPI */
  if (HAL_SPI_Transmit(&hspi1, &Data, sizeof(uint8_t), SPI_TIMEOUT_VALUE) != HAL_OK) 
  {
    return HAL_ERROR;
  }
  
  return HAL_OK;
}

/**
  * @brief  This function send and receive a byte using the SPI, checking the result
  * @retval Result as OK or ERROR
  */
static uint8_t sFLASH_SendReceiveByte(uint8_t Data, uint8_t *pRetData)
{
  /* Transmit and receive byte via SPI */
  if (HAL_SPI_TransmitReceive(&hspi1, &Data, pRetData, sizeof(uint8_t), SPI_TIMEOUT_VALUE) != HAL_OK) 
  {
    return HAL_ERROR;
  }
  
  return HAL_OK;
}

/**
  * @brief  This function receive a block of data from the SPI
  * @retval Result as OK or ERROR
  */
static uint8_t sFLASH_ReceiveData(uint8_t *pData, uint16_t Size)
{
  /* Receive a block of data via SPI */
  if (HAL_SPI_Receive(&hspi1, pData, Size, SPI_TIMEOUT_VALUE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  return HAL_OK;
}

/**
  * @brief  This function transmit a block of data to the SPI
  * @retval Result as OK or ERROR
  */
static uint8_t sFLASH_TransmitData(uint8_t *pData, uint16_t Size)
{
  /* Transmit a block of data via SPI */
  if (HAL_SPI_Transmit(&hspi1, pData, Size, SPI_TIMEOUT_VALUE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  return HAL_OK;
}


/**
  * @brief  This function send a Write Enable and wait it is effective.
  * @retval Result as OK or ERROR
  */
static uint8_t sFLASH_WriteEnable(void)
{
  
  /* Put NSS to LOW level */
  NSS_TO_LOW_LEVEL 
    
  /* Transmit command used to unlock the device */
  if (sFLASH_SendByte(WRITE_ENABLE_CMD) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  /* Put NSS to HIGH level */
  NSS_TO_HIGH_LEVEL

  return HAL_OK;
}

/**
  * @brief  This function read the SR of the memory and wait the EOP.
  * @retval Result as OK or ERROR
  */
static uint8_t sFLASH_WaitForWriteInProgress(uint32_t Timeout)
{
  uint8_t StatusReg;
    
  uint32_t tickstart = HAL_GetTick();
  
  
  do {

    /* Put NSS to LOW level */
    NSS_TO_LOW_LEVEL  
    
    /* Transmit command used to unlock the device */
    if (sFLASH_SendByte(READ_STATUS_REG_CMD) != HAL_OK)
    {
      return HAL_ERROR;
    }
        
    /* Transmit a DUMMY byte and read the status register */
    if (sFLASH_SendReceiveByte(sFLASH_DUMMY_BYTE, (uint8_t *)&StatusReg) != HAL_OK)
    {
      return HAL_ERROR;
    } 
    
    /* Put NSS to HIGH level */
    NSS_TO_HIGH_LEVEL
      
    /* Check if timeout is expired */
    if((HAL_GetTick() - tickstart ) > Timeout)
    {        
      return HAL_TIMEOUT;
    }
          
    /* Check Status register if WIP flag is set --> an operation is in progress */
  } while((StatusReg & sFLASH_STATUS_REG_WIP_MASK) != 0);
    
  return HAL_OK;
}

/**
  * @brief  Reads an amount of data from the QSPI memory.
  * @param  pData: Pointer to data to be read
  * @param  ReadAddr: Read start address
  * @param  Size: Size of data to read
  * @retval QSPI memory status
  */
static uint8_t sFLASH_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)
{
    
  if ((ReadAddr >= sFLASH_Information.FlashSize) || ((ReadAddr + Size) > sFLASH_Information.FlashSize))
  {
    /* address out of range */
    return HAL_ERROR;
  }
  
  /* Put NSS to LOW level */
  NSS_TO_LOW_LEVEL  

    /* Transmit command used to read the device content */
  if (sFLASH_SendByte(READ_CMD) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Transmit the address of the data to be read (1st byte) */  
  if (sFLASH_SendByte((ReadAddr & 0xFF0000) >> 16) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Transmit the address of the data to be read (2nd byte) */  
  if (sFLASH_SendByte((ReadAddr & 0xFF00) >> 8) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Transmit the address of the data to be read (3rd byte) */  
  if (sFLASH_SendByte(ReadAddr & 0xFF) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Receive data */
  if (sFLASH_ReceiveData(pData, Size) != HAL_OK)
  {
    return HAL_ERROR;
  }
 
  /* Put NSS to HIGH level */
  NSS_TO_HIGH_LEVEL  
    
  return HAL_OK;
}

/**
  * @brief  Writes an amount of data to the QSPI memory.
  * @param  pData: Pointer to data to be written
  * @param  WriteAddr: Write start address
  * @param  Size: Size of data to write
  * @retval QSPI memory status
  */
static uint8_t sFLASH_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
{
    
  uint32_t end_addr, current_size, current_addr;

  if ((WriteAddr >= sFLASH_Information.FlashSize) || ((WriteAddr + Size) > sFLASH_Information.FlashSize))
  {
    /* address out of range */
    return HAL_ERROR;
  }

  /* Calculation of the size between the write address and the end of the page */
  current_addr = 0;

  while (current_addr <= WriteAddr)
  {
    current_addr += sFLASH_Information.ProgPageSize;
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
  
  do
  {
       
    /* Before any page program operation, a Write enable command must be performed */
    if (sFLASH_WriteEnable() != HAL_OK)
    {     
      return HAL_ERROR;    
    }

    /* Put NSS to LOW level */
    NSS_TO_LOW_LEVEL  
      
    /* Send Page Program command */
    if (sFLASH_SendByte(PAGE_PROG_CMD) != HAL_OK)   
    {
      return HAL_ERROR;
    }
    /* Send the address of the location to write (1st byte) */
    if (sFLASH_SendByte((current_addr & 0xFF0000) >> 16) != HAL_OK)
    {
      return HAL_ERROR;
    }
    /* Send the address of the location to write (2nd byte) */
    if (sFLASH_SendByte((current_addr & 0xFF00) >> 8) != HAL_OK)
    {
      return HAL_ERROR;
    }
    /* Send the address of the location to write (3rd byte) */
    if (sFLASH_SendByte(current_addr & 0xFF) != HAL_OK)
    {
      return HAL_ERROR;
    }
    /* Send the data to write */  
    if (sFLASH_TransmitData(pData, current_size) != HAL_OK)
    {
      return HAL_ERROR;
    } 
    
    /* Put NSS to HIGH level */
    NSS_TO_HIGH_LEVEL
    
    /* Configure automatic polling mode to wait for end of program */
    if (sFLASH_WaitForWriteInProgress(SPI_TIMEOUT_VALUE) != HAL_OK)
    {
      return HAL_ERROR;
    }
    
    /* Update the address and size variables for next page programming */
    current_addr += current_size;
    pData += current_size;
    current_size = ((current_addr + sFLASH_Information.ProgPageSize) > end_addr) ? (end_addr - current_addr) : sFLASH_Information.ProgPageSize;

  }
  while (current_addr < end_addr);
  
  return HAL_OK;
}


/**
  * @brief  Erases the specified block of the QSPI memory.
  * @param  BlockAddress: Block address to erase
  * @retval QSPI memory status
  */
uint8_t sFLASH_Erase_Block(uint32_t BlockAddress)
{
  
  /* Send Write enable command */
  if (sFLASH_WriteEnable() != HAL_OK)
  {
    return HAL_ERROR;    
  }
  
  /* Put NSS to LOW level */
  NSS_TO_LOW_LEVEL  
    
  /* Transmit command used to erase a sector */
  if (sFLASH_SendByte(SECTOR_ERASE_CMD) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Send the address of the location that need to be erased (1st byte)*/
  if (sFLASH_SendByte((BlockAddress & 0xFF0000) >> 16) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Send the address of the location that need to be erased (2nd byte)*/
  if (sFLASH_SendByte((BlockAddress & 0xFF00) >> 8) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Send the address of the location that need to be erased (3rd byte)*/
  if (sFLASH_SendByte(BlockAddress & 0xFF0) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  /* Put NSS to HIGH level */
  NSS_TO_HIGH_LEVEL
  
  /* Read the status register to poll the WIP flag */
  if (sFLASH_WaitForWriteInProgress(FLASH_SUBSECTOR_ERASE_MAX_TIME) != HAL_OK)
  {
    return HAL_ERROR;
  }
    
  return HAL_OK;
  
}

/**
  * @brief  Erases the specified sub-block of the QSPI memory.
  * @param  BlockAddress: Block address to erase
  * @retval QSPI memory status
  */
static uint8_t sFLASH_Erase_SubBlock(uint32_t BlockAddress)
{
  
  /* Send Write enable command */
  if (sFLASH_WriteEnable() != HAL_OK)
  {
    return HAL_ERROR;    
  }
  
  /* Put NSS to LOW level */
  NSS_TO_LOW_LEVEL  
    
  /* Transmit command used to erase a sector */
  if (sFLASH_SendByte(SECTOR_4K_ERASE_CMD) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Send the address of the location that need to be erased (1st byte)*/
  if (sFLASH_SendByte((BlockAddress & 0xFF0000) >> 16) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Send the address of the location that need to be erased (2nd byte)*/
  if (sFLASH_SendByte((BlockAddress & 0xFF00) >> 8) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Send the address of the location that need to be erased (3rd byte)*/
  if (sFLASH_SendByte(BlockAddress & 0xFF0) != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  /* Put NSS to HIGH level */
  NSS_TO_HIGH_LEVEL
   
  /* Read the status register to poll the WIP flag */
  if (sFLASH_WaitForWriteInProgress(FLASH_SUBSECTOR_ERASE_MAX_TIME) != HAL_OK)
  {
    return HAL_ERROR;
  }
    
  return HAL_OK;

}

/**
  * @brief  Reads current status register 1 of the QSPI memory.
  * @retval QSPI status register 1
  */
static uint8_t sFLASH_GetStatusReg1(uint8_t *reg)
{
   
  /* Put NSS to LOW level */
  NSS_TO_LOW_LEVEL  
    
  /* Transmit command  */
  if (sFLASH_SendByte(READ_STATUS_REG_CMD) != HAL_OK)
  {
    return HAL_ERROR;
  }  
    
  /* Transmit a DUMMY byte so I can read the status register */
  if (sFLASH_SendReceiveByte(sFLASH_DUMMY_BYTE, reg) != HAL_OK)
  {
    return HAL_ERROR;
  } 

  /* Put NSS to HIGH level */
  NSS_TO_HIGH_LEVEL
    
  return HAL_OK;

}

/**
  * @brief  Reads current configuration register 1 of the QSPI memory.
  * @retval QSPI configuration register 1
  */
static uint8_t sFLASH_GetConfigReg1(uint8_t *reg)
{
  
  /* Put NSS to LOW level */
  NSS_TO_LOW_LEVEL  
    
  /* Transmit command  */
  if (sFLASH_SendByte(READ_FUNCTION_REG_CMD) != HAL_OK)
  {
    return HAL_ERROR;
  } 
    
  /* Transmit a DUMMY byte so I can read the status register */
  if (sFLASH_SendReceiveByte(sFLASH_DUMMY_BYTE, reg) != HAL_OK)
  {
    return HAL_ERROR;
  } 

  /* Put NSS to HIGH level */
  NSS_TO_HIGH_LEVEL
    
  return HAL_OK;

}


/**
  * @brief  Return the configuration of the QSPI memory.
  * @param  pInfo: pointer on the configuration structure
  * @retval QSPI memory status
  */
uint8_t sFLASH_GetInfo(sFLASH_Info* pInfo)
{
  /* Configure the structure with the memory configuration */
  if (sFLASH_Model == sFLASH_W25Q80DV)
  {
    pInfo->FlashSize            = FLASH_080_SIZE;
    pInfo->EraseSectorSize      = EXT_FLASH_SUBSECTOR_SIZE;
    pInfo->EraseSectorsNumber   = (FLASH_080_SIZE / EXT_FLASH_SUBSECTOR_SIZE);
    pInfo->ProgPageSize         = EXT_FLASH_PAGE_SIZE;
    pInfo->ProgPagesNumber      = (FLASH_080_SIZE / EXT_FLASH_PAGE_SIZE);
    pInfo->numReadDummy         = FLASH_DUMMY_CYCLES_READ_QUAD;
    pInfo->transactionAddress   = FLASH_TRANSACTION_080_ADD;
    pInfo->numTransactionSector = FLASH_TRANSACTION_NUM_SEC;
    return HAL_OK;
  }
  else
  {
    if (sFLASH_Model == sFLASH_SST26VF016B)
    {
      pInfo->FlashSize            = FLASH_016_SIZE;
      pInfo->EraseSectorSize      = EXT_FLASH_SUBSECTOR_SIZE;
      pInfo->EraseSectorsNumber   = (FLASH_016_SIZE / EXT_FLASH_SUBSECTOR_SIZE);
      pInfo->ProgPageSize         = EXT_FLASH_PAGE_SIZE;
      pInfo->ProgPagesNumber      = (FLASH_016_SIZE / EXT_FLASH_PAGE_SIZE);
      pInfo->numReadDummy         = FLASH_DUMMY_CYCLES_READ_QUAD;
      pInfo->transactionAddress   = FLASH_TRANSACTION_016_ADD;
      pInfo->numTransactionSector = FLASH_TRANSACTION_NUM_SEC;
      return HAL_OK;
    }
    else
    {
      if (sFLASH_Model == sFLASH_IS25LP016)
      {
        pInfo->FlashSize            = FLASH_016_SIZE;
        pInfo->EraseSectorSize      = EXT_FLASH_SUBSECTOR_SIZE;
        pInfo->EraseSectorsNumber   = (FLASH_016_SIZE / EXT_FLASH_SUBSECTOR_SIZE);
        pInfo->ProgPageSize         = EXT_FLASH_PAGE_SIZE;
        pInfo->ProgPagesNumber      = (FLASH_016_SIZE / EXT_FLASH_PAGE_SIZE);
        pInfo->numReadDummy         = FLASH_DUMMY_CYCLES_READ_QUAD;
        pInfo->transactionAddress   = FLASH_TRANSACTION_016_ADD;
        pInfo->numTransactionSector = FLASH_TRANSACTION_NUM_SEC;
        return HAL_OK;
      }
      else
      {
        if (sFLASH_Model == sFLASH_IS25LP080)
        {
          pInfo->FlashSize            = FLASH_080_SIZE;
          pInfo->EraseSectorSize      = EXT_FLASH_SUBSECTOR_SIZE;
          pInfo->EraseSectorsNumber   = (FLASH_080_SIZE / EXT_FLASH_SUBSECTOR_SIZE);
          pInfo->ProgPageSize         = EXT_FLASH_PAGE_SIZE;
          pInfo->ProgPagesNumber      = (FLASH_080_SIZE / EXT_FLASH_PAGE_SIZE);
          pInfo->numReadDummy         = FLASH_DUMMY_CYCLES_READ_QUAD;
          pInfo->transactionAddress   = FLASH_TRANSACTION_080_ADD;
          pInfo->numTransactionSector = FLASH_TRANSACTION_NUM_SEC;
          return HAL_OK;
        }
        else
        {
          if (sFLASH_Model == sFLASH_GD25Q16C)
          {
            pInfo->FlashSize            = FLASH_016_SIZE;
            pInfo->EraseSectorSize      = EXT_FLASH_SUBSECTOR_SIZE;
            pInfo->EraseSectorsNumber   = (FLASH_016_SIZE / EXT_FLASH_SUBSECTOR_SIZE);
            pInfo->ProgPageSize         = EXT_FLASH_PAGE_SIZE;
            pInfo->ProgPagesNumber      = (FLASH_016_SIZE / EXT_FLASH_PAGE_SIZE);
            pInfo->numReadDummy         = FLASH_DUMMY_CYCLES_READ_QUAD;
            pInfo->transactionAddress   = FLASH_TRANSACTION_016_ADD;
            pInfo->numTransactionSector = FLASH_TRANSACTION_NUM_SEC;
            return HAL_OK;
          }
        }
      }
    }
  }

  return HAL_ERROR;
}

static uint8_t sFLASH_GetDeviceId(uint8_t* pIdreg)
{
     
  /* Put NSS to LOW level */
  NSS_TO_LOW_LEVEL  
    
  /* Transmit command  */
  if (sFLASH_SendByte(READ_MANUFACTURER_AND_PROD_ID) != HAL_OK)
  {
    return HAL_ERROR;
  } 
      
  /* Transmit the 1st  DUMMY byte, receive Manufacturer ID*/
  if (sFLASH_SendReceiveByte(sFLASH_DUMMY_BYTE, pIdreg) != HAL_OK)
  {
    return HAL_ERROR;
  } 
    
  /* Transmit the 2nd DUMMY byte, receive the Memory Type */
  if (sFLASH_SendReceiveByte(sFLASH_DUMMY_BYTE, (pIdreg + 1)) != HAL_OK)
  {
    return HAL_ERROR;
  } 
  
  /* Transmit the 3rd DUMMY byte, receive the Capacity */
  if (sFLASH_SendReceiveByte(sFLASH_DUMMY_BYTE, (pIdreg + 2)) != HAL_OK)
  {
    return HAL_ERROR;
  } 
     
  /* Put NSS to HIGH level */
  NSS_TO_HIGH_LEVEL

  return HAL_OK;
  
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
      RetErase = sFLASH_Erase_SubBlock(StartSector * FlashDimBlk);

      if (RetErase != HAL_OK)
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
      RetWrite = sFLASH_Write((BuffWr + BytesWritten), Address + BytesWritten, BytesToProcess);

      if (RetWrite != HAL_OK)
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
      RetErase = sFLASH_Erase_SubBlock(StartSector * FlashDimBlk);

      if (RetErase != HAL_OK)
      {
        // Rilascio il semaforo
        osSemaphoreRelease(FlashSemaphoreHandle);
        return (0xFF);
      }

      // Incremento
      RetErase = sFLASH_Read(testErase, (StartSector * FlashDimBlk), sizeof(testErase));

      if ((RetErase != HAL_OK) || (testErase[0] != 0xFF) || (testErase[1] != 0xFF))
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
      RetWrite = sFLASH_Write((BuffWr + BytesWritten), Address + BytesWritten, BytesToProcess);

      if (RetWrite != HAL_OK)
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
      RetRead = sFLASH_Read((BuffRd + BytesRead), Address + BytesRead, BytesToProcess);

      if (RetRead != HAL_OK)
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
  RetVal = sFLASH_Erase_Chip();
#else
#ifndef ERASE_CHIP_WITH_SUBSECTORS

  /*we have 1024 64K sector (512 64K sector from 0x0200.0000=OFFSET_FLASH_ADDRESS to 0x03FF.FFFF) */
  for (sector = 0; sector < FLASH_FWFILE_NUM_64K_SECTOR; sector++)
  {
    address = OFFSET_FLASH_ADDRESS + (sector * (uint32_t)FLASH_SECTOR_64K_SIZE);
    RetVal = sFLASH_Erase_Block(address);

    if (RetVal != HAL_OK)
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
  for (sector = 0; sector < (unsigned int)FLASH_FWFILE_NUM_SECTOR; sector++)
  {
    address = OFFSET_FLASH_ADDRESS + (sector * (uint32_t)0x1000);
    RetVal = sFLASH_Erase_SubBlock(address);

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
  for (sector = 0; sector < FLASH_FWFILE_NUM_64K_SECTOR; sector++)
  {
    address = (uint32_t)0x00000000 + (sector * (uint32_t)FLASH_SECTOR_64K_SIZE);
    RetVal = sFLASH_Erase_Block(address);

    if (RetVal != HAL_OK)
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

  /* Create semaphore */
  SPI1_semaphore = osSemaphoreNew(1, 1, NULL);

  // Termino l'inizializzazione cominciata in MX_QUADSPI_Init()
  // La funzione MX_QUADSPI_Init() viene generata in automatico, quindi aggiungo qui le altre funzioni
   
  // Leggo l'ID della flash
  sFLASH_GetDeviceId((uint8_t*)FlashID); 
  
  // Controllo se è una flash gestita
  if (memcmp(FlashID, "\xEF\x40\x14", 3) == 0)     /* "\xEF\x40\x14"  WINBOND - W25 - 8Mb */
  {
    sFLASH_Model = sFLASH_W25Q80DV;
  }
  else
  {
    if (memcmp(FlashID, "\xEF\x40\x15", 3) == 0)    /* "\xEF\x40\x15"  W25Q16JV WINBOND - W25 - 16Mb */
    {
      sFLASH_Model = sFLASH_W25Q016DV;
    }
    else
    {
      if (memcmp(FlashID, "\x9D\x60\x14", 3) == 0)    /* "\x9D\x60\x14"  S25LP080D ISSI - LP - 8Mb */
      {
        sFLASH_Model = sFLASH_IS25LP080;
      }
      else
      {
        if (memcmp(FlashID, "\x9D\x60\x15", 3) == 0)    /* "\x9D\x60\x15"  S25LP016D ISSI - LP - 16Mb */
        {
          sFLASH_Model = sFLASH_IS25LP016;
        }
        else
        {
          if (memcmp(FlashID, "\xBF\x26\x41", 3) == 0)    /* "\xBF\x26\x41"  sFLASH_SST26VF016B Microchip - 26V - 16Mb */
          {
            sFLASH_Model = sFLASH_SST26VF016B;
          }
          else
          {
            if (memcmp(FlashID, "\xC8\x40\x15", 3) == 0)    /* "\xC8\x40\x15"  GD25Q16C GIGA DEVICE - GD25Q - 16Mb */
            {
              sFLASH_Model = sFLASH_GD25Q16C;
            }
            else
            {
              sFLASH_Model = sFLASH_UNKNOWN;
            }
          }
        }
      }
    }
  }

  if (sFLASH_Model != sFLASH_UNKNOWN)
  {
       
    if (HAL_SPI_Init(&hspi1) != HAL_OK) //  reinizializzazione modulo SPI di STM32F765
    {
      // Flash in errore, blocco tutto
      while(1);
    }
    
    // Assegno alla struttura Info i valori relativi alla QSPI in uso
    sFLASH_GetInfo(&sFLASH_Information);

    FlashDimBlk = sFLASH_Information.EraseSectorSize; // Dimensione di un sotto-settore
    FlashDimPag = sFLASH_Information.ProgPageSize; // Dimensione di una pagina
    
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
  if ((sFLASH_Model == sFLASH_W25Q80DV) || (sFLASH_Model == sFLASH_W25Q016DV) ||
      (sFLASH_Model == sFLASH_IS25LP016) || (sFLASH_Model == sFLASH_SST26VF016B) || (sFLASH_Model == sFLASH_IS25LP080) ||
      (sFLASH_Model == sFLASH_GD25Q16C))
  {
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }

}

/**
  * @brief  Erases the specified sub-block of the QSPI memory.
  * @param  BlockAddress: Block address to erase
  * @retval QSPI memory status
  */
uint8_t raw_sFLASH_Erase_SubBlock(uint32_t BlockAddress)
{
  return(sFLASH_Erase_SubBlock(BlockAddress));
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
  return(sFLASH_Write(pData, WriteAddr, Size));
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
  return(sFLASH_Read(pData, ReadAddr, Size));
}

/**
  * @brief  Erase data area in QSPI-Flash.
  * @retval QSPI memory status
  */
uint8_t raw_sFLASH_EraseDataArea(void)
{
  uint32_t      qspiAdd;
  uint8_t       result;
  
  for (qspiAdd = FLASH_FWFILE_SIZE; qspiAdd < sFLASH_Information.FlashSize; qspiAdd += EXT_FLASH_SUBSECTOR_SIZE)
  {
    result = raw_sFLASH_Erase_SubBlock(qspiAdd);
    refreshWD();

    if (result != HAL_OK)
    {
      break;
    }
  }

  return(result);
}

/**
  * @brief  Erase data area in QSPI-Flash.
  * @param  none
  * @retval char*: pointer to start address data area in QSPI flash
  */
char* pQspiDataArea(void)
{
  return((char*)FLASH_FWFILE_SIZE);
}


uint8_t getStatusReg(void)
{
  uint8_t statusReg;

  sFLASH_GetStatusReg1(&statusReg);
  return (statusReg);
}

uint8_t getConfigReg(void)
{
  uint8_t configReg;

  sFLASH_GetConfigReg1(&configReg);
  return (configReg);
}


/**
  * @brief  Get address transaction start area
  * @param  none
  * @retval uint32_t: pointer to start address transaction data area in QSPI flash
  */
uint32_t getTransactionAddress(void)
{
  return (sFLASH_Information.transactionAddress);
}

//-----------------------------------------------------------------


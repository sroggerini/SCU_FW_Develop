/**
* @file        i2c.c
*
* @brief       API for I2C - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: i2c.c 749 2025-05-07 12:32:38Z stefano $
*
*     $Revision: 749 $
*
*     $Author: stefano $
*
*     $Date: 2025-05-07 14:32:38 +0200 (mer, 07 mag 2025) $
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
#include "main.h"
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#else
#include "stm32h5xx_hal.h"
#endif
#include "cmsis_os.h" /* _FS_REENTRANT set to 1 and CMSIS API chosen */
#include "i2c.h"
#include "eeprom.h"
#include "wrapper.h"

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 
static   osSemaphoreId      i2c3Semaphore;

/*
*********************************** SCAME ************************************
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
extern I2C_HandleTypeDef HI2Cx;
extern infoStation_t  infoStation;

/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Function                             **
**                                                                          **
******************************************************************************
*/

extern void I2C_ClearBusyFlagErratum (I2C_HandleTypeDef* hi2c);

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
static void i2c3_Handler(char * file, int line);

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/

/**
*
* @brief       ReadFromEeprom (24C64 = 64Kbits = 4KB -> 
*              0000...1FFF --> A0...A12)
*
* @param [in]  unsigned short: start read address 
*              unsigned char *: pointer where store read data
*              unsigned short: number of data to be read 
*  
* @retval      unsigned char: 0 when successfull read, error code otherwise 
*  
****************************************************************/
unsigned char ReadFromEeprom(unsigned short Address, unsigned char *Buffer, unsigned short Length)
{
  unsigned char TxBuffer[2];
  
  // Preparo il messaggio
  TxBuffer[0] = (Address >> 8) & 0xFF;
  TxBuffer[1] = Address & 0xFF;

  if(osSemaphoreAcquire(i2c3Semaphore, portMAX_DELAY) == osOK)
  {
    
    // Verifico che il dispositivo sia pronto
    while (HAL_I2C_IsDeviceReady(&HI2Cx, DEVICE_EEPROM, 3, 100) != HAL_OK)
    {
      /* Reset I2C peripheral in order to exit from a stuck condition */
      I2C_ClearBusyFlagErratum(&HI2Cx);       
      osSemaphoreRelease(i2c3Semaphore); 
      return (4);
    }

    while (HAL_I2C_Master_Transmit(&HI2Cx, DEVICE_EEPROM,(uint8_t *)TxBuffer, 2, 100) != HAL_OK)
    {
      if (HAL_I2C_GetError(&HI2Cx) != HAL_I2C_ERROR_AF)
      {
        osSemaphoreRelease(i2c3Semaphore); 
        return(1);
      }
    }
    
    /* A little delay before reading data */
    // ONLY FOR DEBUG --> HAL_Delay (2); 
    
    while (HAL_I2C_Master_Receive(&HI2Cx, DEVICE_EEPROM, (uint8_t *)Buffer, Length, 1000) != HAL_OK)
    {
      if (HAL_I2C_GetError (&HI2Cx) != HAL_I2C_ERROR_AF)
      {
        osSemaphoreRelease(i2c3Semaphore); 
        return(2);
      }
    } 
    osSemaphoreRelease(i2c3Semaphore); 
    return(0);
  }
  osSemaphoreRelease(i2c3Semaphore); 
  return(1);
}

/**
*
* @brief       ReadFromEeprom_no_Semaph (24C64 = 64Kbits = 4KB -> 
*              0000...1FFF --> A0...A12) without the check of the semaphore
*
* @param [in]  unsigned short: start read address 
*              unsigned char *: pointer where store read data
*              unsigned short: number of data to be read 
*  
* @retval      unsigned char: 0 when successfull read, error code otherwise 
*  
****************************************************************/
unsigned char ReadFromEeprom_no_Semaph (unsigned short Address, unsigned char *Buffer, unsigned short Length)
{
  unsigned char TxBuffer[2];
  
  // Preparo il messaggio
  TxBuffer[0] = (Address >> 8) & 0xFF;
  TxBuffer[1] = Address & 0xFF;

  // Verifico che il dispositivo sia pronto
  while (HAL_I2C_IsDeviceReady(&HI2Cx, DEVICE_EEPROM, 3, 100) != HAL_OK)
  {
    /* Reset I2C peripheral in order to exit from a stuck condition */
    I2C_ClearBusyFlagErratum(&HI2Cx);       
    return (4);
  }
  
  while (HAL_I2C_Master_Transmit(&HI2Cx, DEVICE_EEPROM,(uint8_t *)TxBuffer, 2, 100) != HAL_OK)
  {
    if (HAL_I2C_GetError(&HI2Cx) != HAL_I2C_ERROR_AF)
    {
      return(1);
    }
  }
  
  //while (HAL_I2C_IsDeviceReady(&HI2Cx, DEVICE_EEPROM, 3, 100) != HAL_OK) 
  {
  }
  while (HAL_I2C_Master_Receive(&HI2Cx, DEVICE_EEPROM, (uint8_t *)Buffer, Length, 1000) != HAL_OK)
  {
    if (HAL_I2C_GetError (&HI2Cx) != HAL_I2C_ERROR_AF)
    {
      return(2);
    }
  } 
  return(0);
}

/**
*
* @brief       checkI2c
*
* @param [in]  none 
*  
* @retval      unsigned char: HAL_OK when successfull CRC on EEPROM data 
*  
****************************************************************/
unsigned char checkI2c(void)
{
  unsigned char           result;
  uint8_t                 ContByte, TempXORChecksum;
  uint8_t*                pSrc;
  NetworkConfiguration_t  netConf;

  if(osSemaphoreAcquire(i2c3Semaphore, portMAX_DELAY) == osOK)
  {
    do
    {
      pSrc = (uint8_t*)&netConf;
      // Il MAC Address a differenza del resto è sulla EEPROM a bordo
      ReadFromEeprom(NET_CONFIG_EEPROM_ADDRESS, pSrc, sizeof(NetworkConfiguration_t));
      // Calcolo il checksum
      for(ContByte = 0, TempXORChecksum = 0; ContByte < sizeof(NetworkConfiguration_t); ContByte++)
      {
        TempXORChecksum += pSrc[ContByte];
      }
      if ((uint8_t)0 != TempXORChecksum)
      {
        // Se il checksum non corrisponde o lettura con errore reinizializzo il modulo I2C  
        reinitI2CforEprom();  
        result = (unsigned char)4;
      }
      else
      {
        result = HAL_OK;
      }
    } while (result != HAL_OK);
  }
  osSemaphoreRelease(i2c3Semaphore); 
  return(result);
}

/**
*
* @brief       WriteOnEeprom (24C64 = 64Kbits = 4KB -> 
*              0000...1FFF --> A0...A12)
*
* @param [in]  unsigned short: start write address 
*              unsigned char *: pointer where read data to write
*              unsigned short: number of data to be written 
*  
* @retval      unsigned char: 0 when successfull write, error code otherwise 
*  
****************************************************************/
unsigned char WriteOnEeprom(unsigned short Address, unsigned char *Buffer, unsigned short Length)
{
  
  unsigned char TxBuffer[2 + EEPROM_PAGE_SIZE];
  
  unsigned short PBuffer = 0;
  
  uint16_t page_offset;
  uint16_t space_in_page;
  uint16_t chunk_size;
  
  if(osSemaphoreAcquire(i2c3Semaphore, portMAX_DELAY) == osOK)
  {
    
    EEPROM_Check_Data_Before_Write (Address);
        
    while (Length > 0)
    {
      /* Check if page is not aligned to 32byte (PAGE SIZE)*/
      page_offset = Address % EEPROM_PAGE_SIZE;
      /* Calculate space availability in the page according to the offset */
      space_in_page = EEPROM_PAGE_SIZE - page_offset;
      /* Calculate number of bytes to write according to the lenght */
      chunk_size = (Length < space_in_page) ? Length : space_in_page;
      
      // Preparo il messaggio
      TxBuffer[0] = (Address >> 8) & 0xFF;
      TxBuffer[1] = Address & 0xFF;

      /* Prepare buffer to write */
      for (unsigned char ContByte = 0; ContByte < chunk_size; ContByte++)
        TxBuffer[2 + ContByte] = *(Buffer + PBuffer + ContByte);
      
      // Verifico che il dispositivo sia pronto
      while (HAL_I2C_IsDeviceReady(&HI2Cx, DEVICE_EEPROM, 3, 100) != HAL_OK)
      {
        /* Reset I2C peripheral in order to exit from a stuck condition */
        I2C_ClearBusyFlagErratum(&HI2Cx);       
        osSemaphoreRelease(i2c3Semaphore); 
        return (4);
      }
      
      /* Transmit buffer to EEPROM device */
      if (HAL_I2C_Master_Transmit(&HI2Cx, DEVICE_EEPROM, (uint8_t *)TxBuffer, chunk_size + 2, 1000) != HAL_OK)
      {
        osSemaphoreRelease(i2c3Semaphore); 
        return (4);
      }
                  
      // wait for writing: Verifico che il dispositivo sia pronto
      while (HAL_I2C_IsDeviceReady(&HI2Cx, DEVICE_EEPROM, 3, 100) != HAL_OK); 
      
      /* Move forward */
      Address += chunk_size;
      PBuffer += chunk_size;
      Length -= chunk_size;

    }
    osSemaphoreRelease(i2c3Semaphore); 
  }
  return (0);
   
}

/**
*
* @brief       ReadFromIoExp
*
* @param [in]  unsigned short:  slave address 
*              unsigned char: start read address
*              unsigned char*:  pointer where store read data
*              unsigned short: number of data to be read 
*  
* @retval      unsigned char: 0 when successfull read, error code otherwise 
*  
****************************************************************/
unsigned char ReadFromIoExp(uint16_t slaveAddr, unsigned char Address, unsigned char *Buffer, unsigned short Length)
{
  unsigned char TxBuffer[2];
  
  // Preparo il messaggio
  TxBuffer[0] = Address;

  if(osSemaphoreAcquire(i2c3Semaphore, portMAX_DELAY) == osOK)
  {
      // Verifico che il dispositivo sia pronto
    while (HAL_I2C_IsDeviceReady(&HI2Cx, slaveAddr, 3, 100) != HAL_OK)
    {
      /* Reset I2C peripheral in order to exit from a stuck condition */
      I2C_ClearBusyFlagErratum(&HI2Cx);       
      osSemaphoreRelease(i2c3Semaphore); 
      return (4);
    }
      
    while (HAL_I2C_Master_Transmit(&HI2Cx, slaveAddr,(uint8_t *)TxBuffer, 1, 100) != HAL_OK)
    {
      if (HAL_I2C_GetError(&HI2Cx) != HAL_I2C_ERROR_AF)
      {
        osSemaphoreRelease(i2c3Semaphore); 
        return(1);
      }
    }
    
    while (HAL_I2C_Master_Receive(&HI2Cx, slaveAddr, (uint8_t *)Buffer, Length, 1000) != HAL_OK)
    {
      if (HAL_I2C_GetError (&HI2Cx) != HAL_I2C_ERROR_AF)
      {
        osSemaphoreRelease(i2c3Semaphore); 
        return(2);
      }
    } 
    osSemaphoreRelease(i2c3Semaphore); 
    return(0);
  }
  osSemaphoreRelease(i2c3Semaphore); 
  return(0);
}

/**
*
* @brief       Write on IOExpander 
*
* @param [in]  unsigned short:  slave address  
*              unsigned short: start write address
*              unsigned char *: pointer where read data to write
*  
* @retval      unsigned char: 0 when successfull write, error code otherwise 
*  
****************************************************************/
unsigned char WriteToIoExp(uint16_t slaveAddr, unsigned short Address, unsigned char data)
{
  unsigned char TxBuffer[2];
  
  if(osSemaphoreAcquire(i2c3Semaphore, portMAX_DELAY) == osOK)
  {
    // Preparo il messaggio
    TxBuffer[0] = Address;
    
    TxBuffer[1] = data;

    // Verifico che il dispositivo sia pronto
    while (HAL_I2C_IsDeviceReady(&HI2Cx, slaveAddr, 3, 100) != HAL_OK) 
    {
      osSemaphoreRelease(i2c3Semaphore); 
      return (4);
    }
    if (HAL_I2C_Master_Transmit(&HI2Cx, slaveAddr, (uint8_t *)TxBuffer, (uint16_t)sizeof(TxBuffer), 1000) != HAL_OK)
    {
      osSemaphoreRelease(i2c3Semaphore); 
      return (4);
    }
  }
  osSemaphoreRelease(i2c3Semaphore); 
  return (0);
}


/**
*
* @brief        Create the semaphore to I2C3 access  
*
* @param [in]   none
*
* @retval       none  
*
***********************************************************************************************************************/
void  i2c3SemaphoreCreate (void)
{
  /* create a binary semaphore used for I2C3 access */
  i2c3Semaphore = osSemaphoreNew(1, 1, NULL);

  if( i2c3Semaphore != NULL )
  {
      // The semaphore was created successfully.
      // The semaphore can now be used.
    //vQueueAddToRegistry ((QueueHandle_t)i2c3Semaphore, "I2C3_SEM");
  }
  else
  {
    i2c3_Handler(__FILE__, __LINE__);
  }
}


/**
*
* @brief       WriteOnAudio 
*
* @param [in]  unsigned short: start write address 
*              unsigned char : volume level for audio device
*              00..64
*  
* @retval      unsigned char: 0 when successfull write, error code otherwise 
*  
****************************************************************/
unsigned char WriteOnAudio(unsigned char volume)
{
  unsigned char TxBuffer[DEVICE_DATA_SIZE];
  
 
  if(osSemaphoreAcquire(i2c3Semaphore, portMAX_DELAY) == osOK)
  {
    // Preparo il messaggio
    TxBuffer[0] = volume;
    
    // Aspetto che il canale I2C sia libero
    while (HAL_I2C_GetState(&HI2Cx) != HAL_I2C_STATE_READY)
    {
    }
    // Verifico che il dispositivo sia pronto
    //while (HAL_I2C_IsDeviceReady(&HI2Cx, DEVICE_EEPROM, 3, 100) != HAL_OK) 
    {
      
    }
    if (HAL_I2C_Master_Transmit(&HI2Cx, DEVICE_AUDIO, (uint8_t *)TxBuffer, DEVICE_DATA_SIZE, 1000) != HAL_OK)
    {
      osSemaphoreRelease(i2c3Semaphore); 
      return (4);
    }
    // wait for writing: Verifico che il dispositivo sia pronto
    while (HAL_I2C_IsDeviceReady(&HI2Cx, DEVICE_AUDIO, 3, 100) != HAL_OK); 
    osSemaphoreRelease(i2c3Semaphore); 
  }
  return (0);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void i2c3_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}


/*************** END OF FILE ******************************************************************************************/


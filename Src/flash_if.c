/**
* @file        main.c
*
* @brief       Main Boot - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: flash_if.c 599 2024-09-26 07:03:24Z stefano $
*
*     $Revision: 599 $
*
*     $Author: stefano $
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
#include "main.h"
#include "flash_if.h"

/**
  * @brief  Unlocks Flash for write access
  * @param  None
  * @retval None
  */
void FLASH_If_Init(void)
{
	HAL_FLASH_Unlock(); 

	/* Clear pending flags (if any) */  
#ifdef GD32F4xx
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#else
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_STRBERR | FLASH_FLAG_INCERR);
#endif	
}

/**
  * @brief  Gets the sector of a given address
  * @param  Address: Flash address
  * @retval The sector of a given address
  */
unsigned int GetSector(unsigned int Address)
{
	unsigned int sector = 0;

	if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
		sector = FLASH_SECTOR_0;  
	else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
		sector = FLASH_SECTOR_1;  
	else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
		sector = FLASH_SECTOR_2;  
	else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
		sector = FLASH_SECTOR_3;  
	else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
		sector = FLASH_SECTOR_4;  
#ifdef GD32F4xx
	else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
		sector = FLASH_SECTOR_5;  
	else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
		sector = FLASH_SECTOR_6;  
	else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
		sector = FLASH_SECTOR_7;  
	else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
		sector = FLASH_SECTOR_8;  
	else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
		sector = FLASH_SECTOR_9;  
	else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
		sector = FLASH_SECTOR_10;  
	else 
		sector = FLASH_SECTOR_11;  
#else
	else 
		sector = FLASH_SECTOR_5;  
#endif		

	return sector;
}

/**
  * @brief  This function does an erase of all user flash area
  * @param  StartSector: start of user flash area
  * @retval 0: user flash area successfully erased
  *         1: error occurred
  */
unsigned int FLASH_If_Erase(unsigned int StartAddrSector)
{
	unsigned int UserStartSector;
	unsigned int SectorError;
	FLASH_EraseInitTypeDef pEraseInit;

	/* Unlock the Flash to enable the flash control register access *************/ 
	FLASH_If_Init();

#ifdef GD32F4xx  
  /* workaround to avoid erase error on PGSERR  */
  FLASH->CR |= FLASH_CR_PG;
  FLASH->CR &= CR_PSIZE_MASK;
  FLASH->CR |= FLASH_PSIZE_DOUBLE_WORD;
#endif

	/* Get the sector where start the user flash area Prima mettere PG = 1 e PSZ = 3 */
	UserStartSector = GetSector(StartAddrSector);

	pEraseInit.TypeErase = TYPEERASE_SECTORS;
	pEraseInit.Sector = UserStartSector;
        pEraseInit.NbSectors = (uint32_t)SECTOR_FOR_BOOT - (uint32_t)UserStartSector;

#ifdef GD32F4xx
	pEraseInit.VoltageRange = VOLTAGE_RANGE_3;
#endif
  
	if (HAL_FLASHEx_Erase(&pEraseInit, &SectorError) != HAL_OK)
	{
		/* Error occurred while page erase */
		return (1);
	}

	return (0);
}

/**
  * @brief  This function writes a data buffer in flash (data are 32-bit aligned).
  * @note   After writing data buffer, the flash content is checked.
  * @param  FlashAddress: start address for writing data buffer
  * @param  Data: pointer on data buffer
  * @param  DataLength: length of data buffer (unit is 32-bit word)   
  * @retval 0: Data successfully written to Flash memory
  *         1: Error occurred while writing data in Flash memory
  *         2: Written Data in flash memory is different from expected one
  */

#ifdef GD32F4xx

unsigned int FLASH_If_Write(unsigned int FlashAddress, unsigned int MaxAddress, unsigned int* Data, unsigned int DataLength)
{
	unsigned int i = 0;

  for (i = 0; (i < DataLength) && (FlashAddress <= (MaxAddress)); i++)  
	{
		/* Device voltage range supposed to be [2.7V to 3.6V], the operation will be done by word */ 
		if (HAL_FLASH_Program(TYPEPROGRAM_WORD, FlashAddress, *(unsigned int*)(Data+i)) == HAL_OK)      
		{
			/* Check the written value */
			if (*(unsigned int*)FlashAddress != *(unsigned int*)(Data+i))
			{
				/* Flash content doesn't match SRAM content */
				return(FLASHIF_WRITINGCTRL_ERROR);
			}
			/* Increment FLASH destination address */
			FlashAddress += 4;
		}
		else
		{
			/* Error occurred while writing data in Flash memory */
			return (FLASHIF_WRITING_ERROR);
		}
	}
	return (FLASHIF_OK);
}

#else

unsigned int FLASH_If_Write(unsigned int FlashAddress, unsigned int MaxAddress, unsigned int* Data, unsigned int DataLength)
{
	unsigned int i = 0;

  for (i = 0; (i < DataLength) && (FlashAddress <= (MaxAddress)); i += 16)   
	{
		/* Device voltage range supposed to be [2.7V to 3.6V], the operation will be done by word */ 
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, FlashAddress, *(uint8_t*)(Data + i)) == HAL_OK)      
		{
			/* Check the written value */
			if (*(unsigned int*)FlashAddress != *(unsigned int*)(Data + i))
			{
				/* Flash content doesn't match SRAM content */
				return(FLASHIF_WRITINGCTRL_ERROR);
			}
			/* Increment FLASH destination address */
			FlashAddress += 16;
		}
		else
		{
			/* Error occurred while writing data in Flash memory */
			return (FLASHIF_WRITING_ERROR);
		}
	}
	return (FLASHIF_OK);
}

#endif

/**
  * @brief  Returns the write protection status of user flash area.
  * @param  None
  * @retval 0: No write protected sectors inside the user flash area
  *         1: Some sectors inside the user flash area are write protected
  */
unsigned short FLASH_If_GetWriteProtectionStatus(void)
{
	unsigned int ProtectedSECTOR = 0xFFF;
	FLASH_OBProgramInitTypeDef OptionsBytesStruct;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Check if there are write protected sectors inside the user flash area ****/
	HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

	/* Get pages already write protected ****************************************/
	ProtectedSECTOR = ~(OptionsBytesStruct.WRPSector) & FLASH_SECTOR_TO_BE_PROTECTED;

	/* Check if desired pages are already write protected ***********************/
	if(ProtectedSECTOR != 0)
	{
		/* Some sectors inside the user flash area are write protected */
		return FLASHIF_PROTECTION_WRPENABLED;
	}
	else
	{ 
		/* No write protected sectors inside the user flash area */
		return FLASHIF_PROTECTION_NONE;
	}
}

/**
  * @brief  Configure the write protection status of user flash area.
  * @param  modifier DISABLE or ENABLE the protection
  * @retval HAL_StatusTypeDef HAL_OK if change is applied.
  */
HAL_StatusTypeDef FLASH_If_WriteProtectionConfig(unsigned int modifier)
{
	unsigned int ProtectedSECTOR = 0xFFF;
	FLASH_OBProgramInitTypeDef config_new, config_old;
	HAL_StatusTypeDef result = HAL_OK;

	/* Get pages write protection status ****************************************/
	HAL_FLASHEx_OBGetConfig(&config_old);

	/* The parameter says whether we turn the protection on or off */
	config_new.WRPState = modifier;

	/* We want to modify only the Write protection */
	config_new.OptionType = OPTIONBYTE_WRP;

	/* No read protection, keep BOR and reset settings */
#ifdef GD32F4xx  
	config_new.RDPLevel = OB_RDP_LEVEL_0;
#endif  
	config_new.USERConfig = config_old.USERConfig;  
	/* Get pages already write protected ****************************************/
	ProtectedSECTOR = config_old.WRPSector | FLASH_SECTOR_TO_BE_PROTECTED;

	/* Unlock the Flash to enable the flash control register access *************/ 
	HAL_FLASH_Unlock();

	/* Unlock the Options Bytes *************************************************/
	HAL_FLASH_OB_Unlock();

	config_new.WRPSector    = ProtectedSECTOR;
	result = HAL_FLASHEx_OBProgram(&config_new);

	return result;
}





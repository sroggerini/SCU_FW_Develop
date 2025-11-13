// Define per evitare inclusioni ricorsive
#ifndef __FLASH_IF_H
	#define __FLASH_IF_H

	/* Includes ------------------------------------------------------------------*/
#ifdef GD32F4xx   
	#include "stm32f4xx_hal.h"


	// Indirizzi dei settori della flash interna
	#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbyte */
	#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbyte */
	#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbyte */
	#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbyte */
	#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbyte */
	#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbyte */
	#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbyte */
	#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbyte */
	#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbyte */
	#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbyte */
	#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbyte */
	#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbyte */

  // Bit che rappresentano l'area della flash interna
  #define FLASH_SECTOR_TO_BE_PROTECTED (OB_WRP_SECTOR_0 | OB_WRP_SECTOR_1 | OB_WRP_SECTOR_2 | OB_WRP_SECTOR_3 |\
                      OB_WRP_SECTOR_4 | OB_WRP_SECTOR_5 | OB_WRP_SECTOR_6 | OB_WRP_SECTOR_7 | OB_WRP_SECTOR_8 |\
                      OB_WRP_SECTOR_9 | OB_WRP_SECTOR_10 | OB_WRP_SECTOR_11)
#else
	#include "stm32h5xx_hal.h"
	
	// Indirizzi dei settori della flash interna
	#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 32 Kbyte */
	#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08008000) /* Base @ of Sector 1, 32 Kbyte */
	#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08010000) /* Base @ of Sector 2, 32 Kbyte */
	#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x08018000) /* Base @ of Sector 3, 32 Kbyte */
	#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08020000) /* Base @ of Sector 4, 128 Kbyte */
	#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08040000) /* Base @ of Sector 5, 256 Kbyte */

  // Bit che rappresentano l'area della flash interna
  #define FLASH_SECTOR_TO_BE_PROTECTED        OB_WRP_SECTOR_ALL
	
#endif
	// Codici di errore
	enum 
	{
		FLASHIF_OK = 0,
		FLASHIF_ERASEKO,
		FLASHIF_WRITINGCTRL_ERROR,
		FLASHIF_WRITING_ERROR
	};
	  
	enum
	{
		FLASHIF_PROTECTION_NONE         = 0,
		FLASHIF_PROTECTION_PCROPENABLED = 0x1,
		FLASHIF_PROTECTION_WRPENABLED   = 0x2,
		FLASHIF_PROTECTION_RDPENABLED   = 0x4,
	};


  void				        FLASH_If_Init                     (void);
  unsigned int		    FLASH_If_Erase                    (unsigned int StartSector);
  unsigned int		    FLASH_If_Write                    (unsigned int FlashAddress, unsigned int MaxAddress, unsigned int* Data, unsigned int DataLength);
  unsigned short		  FLASH_If_GetWriteProtectionStatus (void);
  HAL_StatusTypeDef	  FLASH_If_WriteProtectionConfig    (unsigned int modifier);
  unsigned char       StoreMainFW                       (void);

#endif  /* __FLASH_IF_H */

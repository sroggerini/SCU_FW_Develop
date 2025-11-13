/**
* @file        flash.h
*
* @brief       FAT flash inormation  - Definition -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: ExtFlash.h 747 2025-04-29 15:10:36Z stefano $
*
*     $Revision: 747 $
*
*     $Author: stefano $
*
*     $Date: 2025-04-29 17:10:36 +0200 (mar, 29 apr 2025) $
*
*
* @copyright
*       Copyright (C) 2016 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

/************************************************************
 * Include
 ************************************************************/
#include "cmsis_os.h"


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EXT_FLASH_H
#define __EXT_FLASH_H

/* the flash FAT psarameter are tailored on the minimum flash, i.e MT25QL512 -> 64MB */
/* half upper of the flash is reserved for FAT                                       */
#define BLOCK_SIZE                   ((uint32_t)_MAX_SS)
#define OFFSET_FLASH_ADDRESS         ((uint32_t)0x00000000)
#define FLASH_FWFILE_80_SIZE         ((uint32_t)0x000F0000)  /* 0x000C0000 = 768K or 0x000A0000=640K per FW FILE e 256K/384K per dati raw (720K per FAT e 304K per dati raw fail su f_mkfs riga 5369) */
                                                             /* 0x00D00000 = 832K per FW FILE */
                                                             /* 0x00F00000 = 960K per FW FILE */
//#define FLASH_FWFILE_016_SIZE        ((uint32_t)0x00100000)  /* 1M per FAT e 1M per dati raw */
#define FLASH_FWFILE_016_SIZE        ((uint32_t)0x00100000)  /* 1M per FW file */
#define FLASH_FWFILE_SIZE            FLASH_FWFILE_016_SIZE   /* FLASH_FWFILE_80_SIZE  */
#define FLASH_FWFILE_NUM_SECTOR      ((uint32_t)(FLASH_FWFILE_SIZE / BLOCK_SIZE))
#define FLASH_SECTOR_32K_SIZE        ((uint32_t)0x8000)
#define FLASH_FWFILE_NUM_32K_SECTOR  ((uint32_t)(FLASH_FWFILE_SIZE / (uint32_t)0x8000))
#define FLASH_SECTOR_64K_SIZE        ((uint32_t)0x10000)
#define FLASH_FWFILE_NUM_64K_SECTOR  ((uint32_t)(FLASH_FWFILE_SIZE / (uint32_t)0x10000))

#define FLASH_FWFILE_016_MIN_SIZE    ((uint32_t)0x00100000) // the min space to be reserved for downloaded file 

#define FLASH_TRANSACTION_080_ADD    ((uint32_t)0x000FC000)
//#define FLASH_TRANSACTION_080_ADD ((uint32_t)0x000FF000)
//#define FLASH_TRANSACTION_016_ADD ((uint32_t)0x001FF000)
//#define FLASH_TRANSACTION_016_ADD ((uint32_t)0x001FC000)  // changed by Martin for 15kB of transaction data
#define FLASH_TRANSACTION_016_ADD    ((uint32_t)0x001FC000) 
#define FLASH_TRANSACTION_NUM_SEC    ((uint32_t)4)
#define FLASH_TRANSACTION_SIZE       (FLASH_TRANSACTION_NUM_SEC * BLOCK_SIZE)
#define FLASH_CHECKSUM_VALUE         ((uint16_t)0)

/* define address in ext flash where new fw is loaded */
#define  EXT_SFLASH_NEW_FW_ADDR_START   0
/* 1Mbyte of ext flash for new firmware downloaded during fw update procedure */
#define  EXT_SFLASH_NEW_FW_ADDR_END     0xFFFFF     

/* we try to allocate 480K of contiguos area for FW download */
#define CONTIGUOS_FW_AREA         ((uint32_t)0x00078000)

/* Size of the flash */
#define FLASH_080_SIZE                       ((uint32_t)0x00100000)     /* 8Mbits =>  1MBytes */
#define FLASH_080_ADDRESS_SIZE               ((uint32_t)19)             /* A0...A19 <--> FSIZE in QUADSPI_DCR uP reg  8Mb = 1Mx8  */

#define FLASH_016_SIZE                       ((uint32_t)0x00200000)     /* 16Mbits => 2MBytes */
#define FLASH_016_ADDRESS_SIZE               ((uint32_t)20)             /* A0...A20 <--> FSIZE in QUADSPI_DCR uP reg  16Mb = 2Mx8 */

#define EXT_FLASH_SECTOR_SIZE                ((uint32_t)0x10000)       /* 32/64    sectors of 64KBytes 1MB/2MB */
#define EXT_FLASH_SUBSECTOR_SIZE             ((uint32_t)0x1000)        /* 256/512  sectors of 4KBytes  1MB/2MB */
#define EXT_FLASH_PAGE_SIZE                  ((uint32_t)0x100)         /* 4096/8192 pages of 256 bytes 1MB/2NB */

#define SPI_TIMEOUT_VALUE                    5000U /* 5s */
#define SIZEOF_FLASH_ADDRESS_SPI_MODE           3
#define sFLASH_DUMMY_BYTE                    0xA5
#define sFLASH_STATUS_REG_WIP_MASK           0x01

extern  osSemaphoreId                        SPI1_semaphore;

// Deselect sFLASH and release semaphore
#define NSS_TO_HIGH_LEVEL                    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET); \
                                             osSemaphoreRelease (SPI1_semaphore); 
// Wait if semaphore is busy, acquire it and select sFLASH
#define NSS_TO_LOW_LEVEL                     while (osSemaphoreAcquire (SPI1_semaphore, osWaitForever) != osOK); \
                                             HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
// Deselect ISO15118 module and release semaphore
#define NSS2_TO_HIGH_LEVEL                   HAL_GPIO_WritePin(SPI1_NSS2_GPIO_Port, SPI1_NSS2_Pin, GPIO_PIN_SET); \
                                             osSemaphoreRelease (SPI1_semaphore); 
// Wait if semaphore is busy, acquire it and select ISO15118 module                                             
#define NSS2_TO_LOW_LEVEL                    while (osSemaphoreAcquire (SPI1_semaphore, osWaitForever) != osOK); \
                                             HAL_GPIO_WritePin(SPI1_NSS2_GPIO_Port, SPI1_NSS2_Pin, GPIO_PIN_RESET); 


/* flash erase timeout in msec */
#define FLASH_SUBSECTOR_ERASE_MAX_TIME       800

#define FLASH_DUMMY_CYCLES_READ_QUAD         ((uint32_t)8)

/* N25Q512A Commands  */

/* Reset Operations */
#define RESET_ENABLE_CMD                     0x66
#define RESET_MEMORY_CMD                     0x99

/* Identification Operations */
#define READ_ID_CMD                          0x9E
#define READ_ID_CMD2                         0x9F
#define MULTIPLE_IO_READ_ID_CMD              0xAF
#define READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5A

#define READ_SPI_DEVICE_ID                   0x90
#define READ_MANUFACTURER_AND_PROD_ID        0x9F

/* Read Operations */
#define READ_CMD                             0x03
#define READ_4_BYTE_ADDR_CMD                 0x13

#define FAST_READ_CMD                        0x0B
#define FAST_READ_DTR_CMD                    0x0D
#define FAST_READ_4_BYTE_ADDR_CMD            0x0C

#define DUAL_OUT_FAST_READ_CMD               0x3B
#define DUAL_OUT_FAST_READ_DTR_CMD           0x3D
#define DUAL_OUT_FAST_READ_4_BYTE_ADDR_CMD   0x3C

#define DUAL_INOUT_FAST_READ_CMD             0xBB
#define DUAL_INOUT_FAST_READ_DTR_CMD         0xBD
#define DUAL_INOUT_FAST_READ_4_BYTE_ADDR_CMD 0xBC

#define QUAD_OUT_FAST_READ_CMD               0x6B
#define QUAD_OUT_FAST_READ_DTR_CMD           0x6D
#define QUAD_OUT_FAST_READ_4_BYTE_ADDR_CMD   0x6C

#define QUAD_INOUT_FAST_READ_CMD             0xEB
#define QUAD_INOUT_FAST_READ_DTR_CMD         0xED
#define QUAD_INOUT_FAST_READ_4_BYTE_ADDR_CMD 0xEC

/* Write Operations */
#define WRITE_ENABLE_CMD                     0x06
#define WRITE_DISABLE_CMD                    0x04

/* Register Operations */
#define READ_STATUS_REG_CMD                  0x05
#define WRITE_STATUS_REG_CMD                 0x01

#define READ_CFG_REG_CMD                     0x35
#define WRITE_CFG_REG_CMD                    0x01

#define READ_FUNCTION_REG_CMD                0x48

#define READ_LOCK_REG_CMD                    0xE8
#define WRITE_LOCK_REG_CMD                   0xE5

#define READ_FLAG_STATUS_REG_CMD             0x70
#define CLEAR_FLAG_STATUS_REG_CMD            0x50

#define READ_NONVOL_CFG_REG_CMD              0xB5
#define WRITE_NONVOL_CFG_REG_CMD             0xB1

#define READ_VOL_CFG_REG_CMD                 0x85
#define WRITE_VOL_CFG_REG_CMD                0x81

#define READ_ENHANCED_VOL_CFG_REG_CMD        0x65
#define WRITE_ENHANCED_VOL_CFG_REG_CMD       0x61

#define READ_EXT_ADDR_REG_CMD                0xC8
#define WRITE_EXT_ADDR_REG_CMD               0xC5

/* Program Operations */
#define PAGE_PROG_CMD                        0x02
#define PAGE_PROG_4_BYTE_ADDR_CMD            0x12

#define DUAL_IN_FAST_PROG_CMD                0xA2
#define EXT_DUAL_IN_FAST_PROG_CMD            0xD2

#define QUAD_IN_FAST_PROG_CMD                0x32
#define EXT_QUAD_IN_FAST_PROG_CMD            0x12 /*0x38*/
#define QUAD_IN_FAST_PROG_4_BYTE_ADDR_CMD    0x34

//#define MT25QL01G_ENTER_QUAD_MODE_CMD        0x35
#define FLASH_ENTER_QUAD_MODE_CMD            0x35
#define FLASH_SST26_ENTER_QUAD_MODE_CMD      0x38

/* Erase Operations */
#define SUBSECTOR_ERASE_CMD                  0x52
#define SUBSECTOR_4K_ERASE_CMD_MT25QL01G     0x20
#define SUBSECTOR_ERASE_4_BYTE_ADDR_CMD      0x21

#define SECTOR_32K_ERASE_CMD                 0x52
#define SECTOR_4K_ERASE_CMD                  0x20
#define SECTOR_ERASE_CMD                     0xD8
#define SECTOR_ERASE_4_BYTE_ADDR_CMD         0xDC
#define SECTOR32K_ERASE_4_BYTE_ADDR_CMD      0x5C

#define BULK_ERASE_CMD                       0xC7

#define MT25QL01G_BULK_ERASE_CMD             0xC4
#define MT25QL01G_BULK_ERASE_QUAD_CMD        0x60


#define PROG_ERASE_RESUME_CMD                0x7A
#define PROG_ERASE_SUSPEND_CMD               0x75

/* Protection Operation */
#define GLOBAL_BLOCK_PROTECTION_UNLOCK_CMD   0x98

/* One-Time Programmable Operations */
#define READ_OTP_ARRAY_CMD                   0x4B
#define PROG_OTP_ARRAY_CMD                   0x42

/* 4-byte Address Mode Operations */
#define ENTER_4_BYTE_ADDR_MODE_CMD           0xB7
#define EXIT_4_BYTE_ADDR_MODE_CMD            0xE9

/* MT25QL01G Registers  */
/* device in 4-byte addressing mode: bit 0 = 0 */
#define MT25QL01G_NVR_LOW                    0xFF
#define MT25QL01G_NVR_HIGH                   0xFE 

/* N25Q512A Registers  */ 
/* Status Register */
#define FLASH_SR_WIP                         ((uint8_t)0x01)    /*!< Write in progress */
#define FLASH_SR_WREN                        ((uint8_t)0x02)    /*!< Write enable latch */
#define FLASH_SR_BLOCKPR                     ((uint8_t)0x5C)    /*!< Block protected against program and erase operations */
#define FLASH_SR_PRBOTTOM                    ((uint8_t)0x20)    /*!< Protected memory area defined by BLOCKPR starts from top or bottom */
#define FLASH_SR_SRWREN                      ((uint8_t)0x80)    /*!< Status register write enable/disable */

#define FLASH_BIT_SR_WIP                     ((uint8_t)0x01)    /*!< Write in progress */
#define FLASH_BIT_SR_WREN                    ((uint8_t)0x02)    /*!< Write enable latch */

/* Flag Status Register */
#define FLASH_FSR_NBADDR                     ((uint8_t)0x01)    /*!< 3-bytes or 4-bytes addressing */
#define FLASH_FSR_PRERR                      ((uint8_t)0x02)    /*!< Protection error */
#define FLASH_FSR_PGSUS                      ((uint8_t)0x04)    /*!< Program operation suspended */
#define FLASH_FSR_VPPERR                     ((uint8_t)0x08)    /*!< Invalid voltage during program or erase */
#define FLASH_FSR_PGERR                      ((uint8_t)0x10)    /*!< Program error */
#define FLASH_FSR_ERERR                      ((uint8_t)0x20)    /*!< Erase error */
#define FLASH_FSR_ERSUS                      ((uint8_t)0x40)    /*!< Erase operation suspended */
#define FLASH_FSR_READY                      ((uint8_t)0x80)    /*!< Ready or command in progress */


/* Serial FLASH Info */
typedef struct {
  unsigned int FlashSize;             /*!< Size of the flash */
  unsigned int EraseSectorSize;       /*!< Size of sectors for the erase operation */
  unsigned int EraseSectorsNumber;    /*!< Number of sectors for the erase operation */
  unsigned int ProgPageSize;          /*!< Size of pages for the program operation */
  unsigned int ProgPagesNumber;       /*!< Number of pages for the program operation */
  unsigned int numAddressLine;        /*!< Number of line read/write operation */
  unsigned int numReadDummy;          /*!< Number of dummy cycles in read operation */
  unsigned int transactionAddress;    /*!< start address where store transaction data  */
  unsigned int numTransactionSector;  /*!< num sector for transaction data  */
} sFLASH_Info;

/* QSPI Error codes */
#define QSPI_OK            ((uint8_t)0x00)
#define QSPI_ERROR         ((uint8_t)0x01)
#define QSPI_BUSY          ((uint8_t)0x02)
#define QSPI_NOT_SUPPORTED ((uint8_t)0x04)
#define QSPI_SUSPENDED     ((uint8_t)0x08)

enum
{
  sFLASH_UNKNOWN = 0,
  sFLASH_W25Q80DV,          // W25Q80DV     1MB
  sFLASH_W25Q016DV,         // W25Q016DV    2MB
  sFLASH_IS25LP016,         // IS25LP016    2MB
  sFLASH_SST26VF016B,       // SST26VF016B  2MB
  sFLASH_IS25LP080,         // IS25LP080    1MB
  sFLASH_GD25Q16C           // GD25Q16C     2MB
};

/* FAT */
#define FAT_BLK          0
#define FAT_MARK        "\xBB\x66\xBB\x66\xBB\x66\xBB\x66\xBB\x66"
#define FAT_MARK_LENGTH      10
#define FAT_EMPTY_PAGE_LENGTH  0xFFFFFFFF


/* FILE DI CONFIGURAZIONE */
#define CONFIG_BLK    1

/* Define per archiviazione su flash (QSPI o FATFS) */
#define DATA_BLK      3
#define UPL_N_ARCHIVE_MAX   5
#define UPL_N_RECORD_MAX    256
#define N_MAX_RECORD_PER_SECTOR 8

/* Define per visualizzazione di default */
#define DEFAULT_TYPE_NONE               0
#define DEFAULT_TYPE_VIS_EXTENSIBLE_MEP 1
#define DEFAULT_TYPE_DGT_AUTOARRANQUE   2

/* STRUTTURA FAT (BLOCCO 1 DELLA FLASH) */
struct StructFAT
{
  unsigned char Mark[FAT_MARK_LENGTH];
  unsigned char NRecordsPerSector[UPL_N_ARCHIVE_MAX];
  unsigned int StartAddress[UPL_N_ARCHIVE_MAX];
  unsigned short NRecords[UPL_N_ARCHIVE_MAX];
  unsigned int RecordLength[UPL_N_ARCHIVE_MAX][UPL_N_RECORD_MAX];
  unsigned int RecordMaxDimension[UPL_N_ARCHIVE_MAX];
  unsigned int UploadID[UPL_N_ARCHIVE_MAX];
};

/* STRUTTURA PER UPLOAD */
typedef struct
{
  unsigned short Code;
  unsigned char Type;
  unsigned short PacketNumber;
  unsigned char Interface;
  unsigned int PData;
  unsigned int FlashAdd;
  unsigned int PFlashAdd;
  unsigned int SizeMax;
} StructUPL;

extern unsigned char  FlashWrite                    (unsigned int Address, unsigned char *BuffWr, unsigned int LBuffWr, unsigned int LBuffBackup);
extern unsigned char  FlashFatWrite                 (unsigned int Address, unsigned char *BuffWr, unsigned int LBuffWr, unsigned int LBuffBackup);
extern unsigned char  FlashRead                     (unsigned int Address, unsigned char *BuffRd, unsigned int LBuffRd);
extern unsigned char  FlashInsertFATRecord          (unsigned short IDArchive, unsigned short Code, unsigned char *BuffWr, unsigned short LBuffWr);
extern unsigned int   FlashReadFATRecord            (unsigned short IDArchive, unsigned short Code, unsigned char *BuffRd);
extern unsigned char  FlashRemoveFATRecord          (unsigned short IDArchive, unsigned short Code);
extern unsigned char  FlashFormatFATRecord          (unsigned short IDArchive);
extern unsigned char  FlashSaveIdFAT                (unsigned short IDArchive, unsigned int UploadID);
extern unsigned int   FlashGetIdFAT                 (unsigned short IDArchive);
extern unsigned char  FlashSaveConfiguration        (unsigned char *BuffWr, unsigned int LBuffWr);
extern unsigned char  FlashGetConfiguration         (unsigned char *BuffRd, unsigned int MaxLength);
extern unsigned char  FlashSaveDefaultVisualization (unsigned char Type, unsigned char *BuffWr, unsigned int LBuffWr);
extern unsigned char  FlashGetDefaultVisualization  (unsigned char *BuffRd, unsigned int MaxLength);
extern unsigned char  FlashUpdateNumberOfRecord     (unsigned short IDArchive, unsigned short NumberOfRecord);
extern unsigned short FlashGetNumberOfRecord        (unsigned short IDArchive);
extern unsigned int   FlashGetFileLength            (unsigned short IDArchive, unsigned short Code);
extern unsigned char  FlashSaveFAT                  (void);
extern void           FATInit                       (void);
extern void           FlashInit                     (void);
extern unsigned char  sFLASH_Detected               (void);
extern unsigned char  flashSectorErase              (unsigned int BlockAddress);
extern unsigned char  BSP_QSPI_Erase_Block          (unsigned int BlockAddress);
extern unsigned char  raw_sFLASH_Erase_SubBlock     (unsigned int BlockAddress);
extern unsigned char  raw_sFLASH_Write              (unsigned char* pData, unsigned int WriteAddr, unsigned int Size);
extern uint8_t        raw_sFLASH_Read               (uint8_t* pData, uint32_t ReadAddr, uint32_t Size);
extern uint8_t        raw_sFLASH_EraseDataArea      (void);
extern char*          pQspiDataArea                 (void);
extern void           initializeFatInFlashArea      (void);
extern uint8_t        BSP_QSPI_Erase_64KBlock       (uint32_t BlockAddress);
extern unsigned char  FlashFormat                   (void);
extern unsigned char  FlashConfigFormat             (void);
extern uint8_t        sFLASH_Erase_Block            (uint32_t BlockAddress);

unsigned char         FlashRenameConfiguration      (void);
unsigned char         FlashRestoreConfiguration     (void);
unsigned char         FlashCreateBackupConfiguration(void);
uint8_t               getConfigReg                  (void);
uint8_t               getStatusReg                  (void);
uint32_t              getTransactionAddress         (void);

#endif // __EXT_FLASH_H

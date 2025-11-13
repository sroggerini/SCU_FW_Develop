#ifndef _FLASH_FAT_H
#define _FLASH_FAT_H
#include <stdint.h>


#include "ff.h"

#ifndef FALSE
#define FALSE   0
#define TRUE    1
#define false   0
#define true    1
#endif

#define FLASH_BLK_SIZE		65536
#define BLK_START_FW		0							    // blocco inizio fw da eseguire
#define MAX_BLK_FW			(1310720 / FLASH_BLK_SIZE)	    // 1MB+256KB riservato al fw
#define MAX_BLK_PLD			(1048576 / FLASH_BLK_SIZE)	    // 1MB riservato alla pld

#define	BLK_START_NEW_FW	(BLK_START_FW + MAX_BLK_FW)	
#define	BLK_START_NEW_PLD	(BLK_START_NEW_FW + MAX_BLK_FW)	



// in the EPCS, we reserved for the PLD half of the space used to store the .PLD file in the CFI flash
#define FLASH_FAT_FILES_START_BLOCK		(MAX_BLK_PLD / 2 + 2)
#define FLASH_FAT_FILES_AREA_DIM      	(unsigned int)((1024 * 1024 * 2) - (FLASH_FAT_FILES_START_BLOCK * FLASH_BLK_SIZE))
#define FLASH_FAT_MAX_FILE_DIM     		(1024 * 4)
#define FLASH_FAT_FILES_PER_BLK 		(FLASH_BLK_SIZE / FLASH_FAT_MAX_FILE_DIM)
#define FLASH_FAT_MAX_ENTRIES   		(FLASH_FAT_FILES_AREA_DIM / FLASH_FAT_MAX_FILE_DIM)
#define FLASH_FAT_MAX_BLOCKS    		(FLASH_FAT_FILES_AREA_DIM / FLASH_BLK_SIZE)

#define FLASH_FAT_BLOCK				    (BLK_START_NEW_PLD + MAX_BLK_PLD)
#define FLASH_FAT_ADDRESS		        (FLASH_FAT_BLOCK * FLASH_BLK_SIZE)

#define FLASH_FAT_MAX_FILENAME_LEN      48

/* used as offset in pit file management. See FIELD_BUFFER_TYPE6_LEVEL in MEP.c */
#define FIELD_BUFFER_TYPE_OFFSET 999


#define FLASH_FAT_MAX_EXT_IN_DIR        4
#define CONFIG_DIR_SUFF                 "CONFIG"
#define FW_DIR_SUFF                     "FW"
#define FONT_DIR_SUFF                   "FONT"
#define GRAPHICS_DIR_SUFF               "GRAPHICS"
#define WEB_DIR_SUFF                    "WEB"
#define VER1_DIR_SUFF                   "VER2"

#define CONFIG_DIRF_PRE                 "CONFIG/"
#define FW_DIR_PRE                      "FW/"
#define FONT_DIR_PRE                    "FONT/"
#define GRAPHICS_DIR_PRE                "GRAPHICS/"
#define WEB_DIR_PRE                     "WEB/"
#define VER1_DIR_PRE                    "VER2/"

#define CONFIG_EXT                      "txt"
#define CONFIG2_EXT                     "cfg"
#define FW_EXT                          "bin"
#define MSG_EXT                         "dms"
#define FONT_EXT                        "fnm"
#define FONT_EXT2                       "idn"
#define GRAPHICS_EXT                    "bmp"
#define GRAPHICS_EXT2                   "jpg"
#define GRAPHICS_EXT3                   "png"
#define FATCONF_EXT                     "dat"
#define PIT_EXT                         "pit"
#define WEB_EXT                         "htm"
#define ARC_EXT                         "tar"
#define FPGA_EXT                        "rpd"

#define FW_SCU_PREFIX                   "SCAMESCU"

typedef enum _OP_ERRORS
{
    OP_ERR_NO_ERRORS, OP_ERR_INTERNAL_ERROR, OP_ERR_BAD_PARAMETERS, OP_ERR_STRING_TOO_LONG, OP_ERR_TOO_MANY_PAGES, OP_ERR_WRONG_XML, OP_ERR_COMMUNICATION, OP_ERR_NOT_IMPLEMENTED
} OP_ERRORS;

typedef struct
{
  uint8_t   dirName[FLASH_FAT_MAX_FILENAME_LEN];
  uint8_t   dirFilePref[FLASH_FAT_MAX_FILENAME_LEN];
  uint8_t   pExtInside[FLASH_FAT_MAX_EXT_IN_DIR][4];
}flashDirInfo_st;

typedef enum
{
  CONFIG_DIR = 0,
  FONT_DIR,
  WEB_DIR,
  VER_DIR,
  NUM_DIR
}dirId_e;

typedef struct _FLASH_FILE_ENTRY
{
    char name[FLASH_FAT_MAX_FILENAME_LEN + 1];
    unsigned int address;
    unsigned int length;
    unsigned int used;
    unsigned int free1;
    unsigned int free2;
    unsigned int free3;
    unsigned int free4;
} stFlashEntry;

// the first byte is the version of the FAT structure
#define FLASH_FAT_SIGNATURE	        0x000AE515

typedef struct _FLASH_FAT
{
    stFlashEntry entry[FLASH_FAT_MAX_ENTRIES];
    unsigned int signature;
    unsigned int numEntries;
    unsigned int nextBlkToUse;
    unsigned int free1;
    unsigned int free2;
    unsigned int free3;
    unsigned int free4;
} stFlashFat;


void          FlashFatEraseAll        (char *extension, fatDevice_e fatDevice);
int           FlashFatEraseFile       (char *fileName, fatDevice_e fatDevice);
int           FlashFatRenameFile      (char *fileNameExt, char *fileNameDst, fatDevice_e fatDevice);
int           FlashFatWriteFile       (char *fileName, char *buffer, unsigned int length, uint8_t* ext, fatDevice_e fatDevice);
int           FlashFatReadFile        (char *fileName, char *buffer, unsigned int *length, uint8_t* ext, fatDevice_e fatDevice);
FRESULT       deviceFatFormat         (fatDevice_e fatDevice);
uint8_t*      findExtDir              (uint8_t* ext);
void          moveFileFromRootToFolder(fatDevice_e fatDevice);
FRESULT       sdFatFormat             (void);
FRESULT       sdFatSetDir             (void);
FRESULT       newWelcomeFile          (uint8_t* pNewInfo);
uint32_t      fileExist               (uint8_t* fileName, fatDevice_e fatDevice);
void          fatSemaphoreCreate      (void);
unsigned int* getNewFWsdramAddress    (void);
void          setNewFWavailable       (unsigned int FWLength); 
osSemaphoreId getFatSemaphore         (void);
void          storeInfoFileFtp        (char* name, uint32_t len, uint32_t addr);
uint8_t       moveFileFtpInFat        (void);

FRESULT       dirExist                (uint8_t* dirName, fatDevice_e fatDevice);
FRESULT       dirRemove               (uint8_t* dirName, uint8_t* ext);

uint8_t       checkFlagForNvic        (void);
void          resetFlagForNvic        (void);
void          setFlagForNvic          (void);
uint32_t      getFatFreeSpace         (void);

#endif

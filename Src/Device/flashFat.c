/**
* @file        flashFat.c
*
* @brief       Task for Files operation - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: flashFat.c 763 2025-06-05 15:05:27Z npiergi $
*
*     $Revision: 763 $
*
*     $Author: npiergi $
*
*     $Date: 2025-06-05 17:05:27 +0200 (gio, 05 giu 2025) $
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
#define _FLASH_FAT_C

#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>

#define FLASH_QSPI_FATFS

#include "fatfs.h"
#include "telnet.h"

#include "flashFat.h"

/*
***********************************SCAME**************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
typedef struct _FILE_DOWNLOAD
{
  char*       pFwFileNameExt;
  uint16_t*   pFwBufferW;
  uint8_t*    pFwBuffer;
  uint32_t    ix;
  uint32_t    fwLenght;
  uint16_t    cks;
  uint16_t    cks0;
  uint8_t     newFw;
} fileFwInfo_t;


/*
***********************************SCAME**************************************
**                                                                          **
**                            Local Const                                   **
**                                                                          **
****************************************************************************** 
*/ 
const flashDirInfo_st flashDirInfo[NUM_DIR] =  
  {
    {CONFIG_DIR_SUFF,   CONFIG_DIRF_PRE,   {{CONFIG_EXT},    {CONFIG2_EXT},     {""},              {""}}},
    {FONT_DIR_SUFF,     FONT_DIR_PRE,      {{FONT_EXT},      {FONT_EXT2},       {""},              {""}}},
    {WEB_DIR_SUFF,      WEB_DIR_PRE,       {{WEB_EXT},       {""},              {""},              {""}}},
    {VER1_DIR_SUFF,     VER1_DIR_PRE,      {{""},            {""},              {""},              {""}}}
  };

/*
***********************************SCAME**************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/
extern char    USERPath[4];
extern uint8_t buffWork[4096];

/*
***********************************SCAME**************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 

static   FIL                File;				    /* File objects */  
static   DIR                Dir;			      /* Directory object */  
static   FILINFO            Finfo;          /* File info object */
#ifdef AESYS
static   fileFwInfo_t       fileFwInfo;
static   fileFwInfo_t       fileBmpInfo;
static   fileFwInfo_t       fileHtmInfo;
#endif
static   stFlashEntry       uplFileInfo;
static   osSemaphoreId      fatSemaphore;

/*
***********************************SCAME**************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 

/*
***********************************SCAME**************************************
**                                                                          **
**                            External Function                             **
**                                                                          **
******************************************************************************
*/

/*
***********************************SCAME**************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
static void _ErrorFat_Handler(char * file, int line);


/*
***********************************SCAME**************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/
int strcmpi (char *comparand, const char *comparator)
{
  char *txt1;
  char *txt2;
  int  i, result;
  char c;   

  i = 0;
  txt1 = malloc(strlen(comparand));
  while (comparand[i])
  {
    c=comparand[i];
    txt1[i] = toupper(c);
    i++;
  }
  txt1[i] = '\0';
  txt2 = malloc(strlen(comparator));
  i = 0;
  while (comparator[i])
  {
    c=comparator[i];
    txt2[i] = toupper(c);
    i++;
  }
  txt2[i] = '\0';
	// case insensitive comparison
  result = strcmp (txt1, txt2);
  free(txt1);
  free(txt2);
  return (result);
}


// PUBLIC PART ///////////////////////////////////////////////////


int FlashFatEraseFile(char *fileNameExt, fatDevice_e fatDevice)
{
  uint8_t*  fullFileName;
  FRESULT   res;
  char      *pPath, *fileName, *subDir, *ext;
  uint8_t   fileNameLen;
  
  if(osSemaphoreAcquire(fatSemaphore, portMAX_DELAY) == osOK)
  {
    fileName = (char*)malloc(strlen(fileNameExt) + 1);
    strcpy((char*)fileName, (char*)fileNameExt);
      
    ext = strchr(fileName, '.');
    if (ext)
    {
      *ext = '\0';  // introduco terminatore per identificare il solo nome
      ext++;        // punto all'estensione

      subDir = (char *)findExtDir((uint8_t*)ext);
      pPath = getFatDevicePath (fatDevice);
      if (subDir != NULL)
      {
        /* conservative memory allocation: use always fileNameExt */
        /* remember to add +1 for '\0' string terminator and +1 for '/' dir separator */
        fileNameLen = (size_t)strlen(fileNameExt) + (size_t)strlen(subDir) + sizeof(USERPath) + 2;
        //subDir = (char *)dirFileName;
        fullFileName = (uint8_t*)calloc((size_t)fileNameLen, (size_t)1);
        /*         destination       source */
        strcpy((char *)fullFileName, (char *)pPath);
        /*         destination       source */
        strcat((char *)fullFileName, (char *)subDir);
        /*         destination       source */
        strcat((char *)fullFileName, (char *)"/");
        /*         destination       source */
        strcat((char *)fullFileName, (char *)fileNameExt);
      }
      else
      {
        fullFileName = (uint8_t*)calloc((size_t)strlen(fileNameExt) + sizeof(USERPath) + 2, (size_t)1);
        /*         destination       source */
        strcpy((char *)fullFileName, (char *)pPath);
        /*         destination       source */
        strcat((char *)fullFileName, (char *)fileNameExt);
      }
      res = f_unlink((TCHAR*)fullFileName);  /* Delete an existing file or directory */
      free(fullFileName);
    }
    free(fileName);
    osSemaphoreRelease(fatSemaphore); 
  }
  return ((int)res);
}

void FlashFatEraseAll(char *extension, fatDevice_e fatDevice)
{
  uint8_t*  fileToDelete;
  uint8_t*  fullFileName;
  uint8_t*  fullExt;
  FRESULT   res;
  uint8_t   dirFileName[] = "";
  char*     subDir;
  char      extCharId [] = ".";
  char*     pPath;
  
  pPath = getFatDevicePath (fatDevice);
 
  if (extension)
  {
    subDir = (char *)findExtDir((uint8_t*)extension);
    if (subDir == NULL)
    {
      subDir = (char *)dirFileName;
    }
    fullFileName = (uint8_t*)calloc((size_t)strlen(subDir) + sizeof(USERPath), (size_t)1);
    /*         destination       source */
    strcpy((char *)fullFileName, (char *)pPath);
    /*         destination       source */
    strcat((char *)fullFileName, (char *)subDir);
    
    fullExt      = (uint8_t*)calloc((size_t)strlen(extension) + sizeof(extCharId), (size_t)1);
    /*         destination       source */
    strcpy((char *)fullExt, (char *)extCharId);
    /*         destination       source */
    strcat((char *)fullExt, (char *)extension);
    
    res = f_opendir(&Dir, (TCHAR*)fullFileName);
    if (res) 
    { 
      tPrintf("Error opening dir: code=%d\n\r", (uint16_t)res); 
    }
    else
    {
      for(;;) 
      {
        res = f_readdir(&Dir, &Finfo);
        if ((res != FR_OK) || !Finfo.fname[0]) break;
        if ((Finfo.fattrib & AM_DIR) == 0) 
        {
          /* isn't a directory, it is a file!! */
          fileToDelete = (uint8_t*)strstr(Finfo.fname, (char*)fullExt);
          if (fileToDelete != NULL)
          {
            /* this file must be deleted */
            if (FlashFatEraseFile(Finfo.fname, fatDevice) == 0)
            {
              tPrintf("Error deleting file: %s\n\r", Finfo.fname);
            }
          }
        }
      }
    }
    free(fullFileName);
    free(fullExt);
  }
  else
  {
    (void)deviceFatFormat(fatDevice);
  }
}

int FlashFatRenameFile(char *fileNameExt, char *fileNameDst, fatDevice_e fatDevice)
{
  uint8_t*  fullFileName;
  uint8_t*  fullFileNameDst;
  FRESULT   res;
  char      *pPath, *ext, *subDir, *fileName;
  
  res = FR_DENIED;
  if(osSemaphoreAcquire(fatSemaphore, portMAX_DELAY) == osOK)
  {
    fileName = (char*)malloc(strlen(fileNameExt) + 1);
    strcpy((char*)fileName, (char*)fileNameExt);

    pPath = getFatDevicePath (fatDevice);
   
    ext = strchr(fileName, '.');

    if (ext)
    {
      *ext = '\0';  // introduco terminatore per identificare il solo nome
      ext++;        // punto all'estensione

      subDir = (char *)findExtDir((uint8_t*)ext);
      pPath = getFatDevicePath (fatDevice);
      /* conservative memory allocation: use always fileNameExt */
      /* remember to add +1 for '\0' string terminator and +1 for '/' dir separator */
      fullFileName = (uint8_t*)calloc((size_t)strlen(fileName) + (size_t)strlen(subDir) + (size_t)strlen(USERPath) + 2, (size_t)1);
      fullFileNameDst = (uint8_t*)calloc((size_t)strlen(fileNameDst) + (size_t)strlen(subDir) + (size_t)strlen(USERPath) + 2, (size_t)1);
      /*         destination       source */
      strcpy((char *)fullFileName, pPath);
      /*         destination       source */
      strcpy((char *)fullFileNameDst, pPath);
      /*         destination       source */
      if (subDir != NULL)
      {
        strcat((char *)fullFileName, (char *)subDir);
        /*         destination       source */
        strcat((char *)fullFileName, "/");
        /*         destination       source */
        strcat((char *)fullFileNameDst, (char *)subDir);
        /*         destination       source */
        strcat((char *)fullFileNameDst, "/");
      }
    }
    else
    {
      pPath = getFatDevicePath (fatDevice);
      /* conservative memory allocation: use always fileNameExt */
      /* remember to add +1 for '\0' string terminator and +1 for '/' dir separator */
      fullFileName = (uint8_t*)calloc((size_t)strlen(fileName) + (size_t)strlen(USERPath) + 2, (size_t)1);
      fullFileNameDst = (uint8_t*)calloc((size_t)strlen(fileNameDst) + (size_t)strlen(USERPath) + 2, (size_t)1);
      /*         destination       source */
      strcpy((char *)fullFileName, pPath);
      /*         destination       source */
      strcpy((char *)fullFileNameDst, pPath);
    }
    /*         destination       source */
    strcat((char *)fullFileName, (char *)fileNameExt);
    /*         destination       source */
    strcat((char *)fullFileNameDst, (char *)fileNameDst);

    res = FR_OK;
    if (strcmp((char*)fullFileName, (char*)fullFileNameDst) != (int)0)
    {
      /* rename only if the source and destination have different name and/or path */
      res = f_unlink((TCHAR*)fullFileNameDst);  /* Delete, if exist, a file with new name  */
      res = f_rename ((TCHAR*)fullFileName, (TCHAR*)fullFileNameDst); 
    }
    free(fileName);
    free(fullFileName);
    free(fullFileNameDst);
    osSemaphoreRelease(fatSemaphore); 
  }
  return ((int)res);
}


/**
*
* @brief        Write a file on device 
*
* @param [in]   char *: file name with extension (3 char)
* @param [in]   char *: pointer to buffer containt the file data
* @param [in]   unsigned int: data file length
* @param [in]   uint8_t*: pointer to ext string (NULL as default)
* @param [in]   fatDevice_e: default device where the file will be stored
*
* @retval       int: the written bytes (equal data length typically)
*
***********************************************************************************************************************/
int FlashFatWriteFile(char *fileNameExt, char *buffer, unsigned int length, uint8_t* extF, fatDevice_e fatDevice)
{
  uint8_t   mode, fileNameLen;
  uint8_t*  fullFileName;
  uint32_t  byteWritten, len, deltaBuffLen, addr;
  FRESULT   res;
  char*     subDir;
  char*     pPath;
  char*     fileName;
  char*     ext;

  if(osSemaphoreAcquire(fatSemaphore, portMAX_DELAY) == osOK)
  {
    fileName = (char*)malloc(strlen(fileNameExt) + 1);
    strcpy((char*)fileName, (char*)fileNameExt);
      
    if (extF == NULL)
    {
      ext = strchr(fileName, '.');
      if (ext)
      {
        *ext = '\0';  // introduco terminatore per identificare il solo nome
        ext++;        // punto all'estensione
      }
    }
    else
    {
      ext = (char*)extF;
    }
    subDir = (char *)findExtDir((uint8_t*)ext);
    pPath = getFatDevicePath (fatDevice);
    if (subDir != NULL)
    {
      /* conservative memory allocation: use always fileNameExt */
      fileNameLen = (size_t)strlen(fileNameExt) + (size_t)strlen(subDir) + sizeof(USERPath) + 2;
      //subDir = (char *)dirFileName;
      fullFileName = (uint8_t*)calloc((size_t)fileNameLen, (size_t)1);
      /*         destination       source */
      strcpy((char *)fullFileName, (char *)pPath);
      /*         destination       source */
      strcat((char *)fullFileName, (char *)subDir);
      /*         destination       source */
      strcat((char *)fullFileName, (char *)"/");
      /*         destination       source */
      strcat((char *)fullFileName, (char *)fileNameExt);
    }
    else
    {
      fullFileName = (uint8_t*)calloc((size_t)strlen(fileNameExt) + sizeof(USERPath) + 2, (size_t)1);
      /*         destination       source */
      strcpy((char *)fullFileName, (char *)pPath);
      /*         destination       source */
      strcat((char *)fullFileName, (char *)fileNameExt);
    }
    mode = FA_READ;
    res = f_open(&File, (TCHAR*)fullFileName, mode);
    if ((res == FR_EXIST) || (res == FR_OK))
    {
      f_close(&File); 
      res = f_unlink((TCHAR*)fullFileName);  /* Delete an existing file or directory */
    }
    f_close(&File);
    mode =  FA_WRITE | FA_CREATE_ALWAYS | FA_CREATE_NEW;
    res = f_open(&File, (TCHAR*)fullFileName, mode);
    if (res != FR_OK) { free(fileName); free(fullFileName); osSemaphoreRelease(fatSemaphore); return 0; } 

    /* copy using 4K buffer starting from 0x000B.4000 QSPI area */
    deltaBuffLen = sizeof(buffWork); 
    if (length < deltaBuffLen)
    {
      /* current length file is under 4K buffer */
      deltaBuffLen = length; 
    }
    for (addr = (uint32_t)buffer, len = 0U; len < length; )
    {
      if (raw_sFLASH_Read((uint8_t*)buffWork, addr, deltaBuffLen) == HAL_OK)
      {
        res = f_write(&File, (void*)buffWork, deltaBuffLen, (UINT*)&byteWritten);
        if ((res != FR_OK) || (byteWritten != deltaBuffLen)) break;
      }
      addr += deltaBuffLen; // next address for data 
      len += deltaBuffLen;  // update num bytes written
      if ((length - len) < deltaBuffLen)
      {
        /* last chunk of file < 4K */
        deltaBuffLen = length - len;
      }
      refreshWD();
    } // end for 
    //res = f_write(&File, buffer, length, (UINT*)&byteWritten);
    if ((res != FR_OK) || (len != length)) { free(fileName); free(fullFileName); osSemaphoreRelease(fatSemaphore); return 0; }
    f_close(&File);
    free(fileName);
    free(fullFileName);
    osSemaphoreRelease(fatSemaphore); 
  }
  return (len);
}

/**
*
* @brief        Read a file from device 
*
* @param [in]   char *: file name with extension (3 char)
* @param [in]   char *: pointer to buffer where read data are put (if NULL only the file len is returned if file exist)
* @param [in]   unsigned int*: pointer where number of byte read is written 
* @param [in]   uint8_t*: pointer to ext string (NULL as default)
* @param [in]   fatDevice_e: default device where the file will be stored
*
* @retval       int: 1 when all OK, 0 on any error (basically when the file doen't exist)
*
***********************************************************************************************************************/
int FlashFatReadFile(char *fileNameExt, char *buffer, unsigned int *length, uint8_t* extF, fatDevice_e fatDevice)
{
  uint8_t*  fullFileName;
  uint32_t  byteRead = 0;
  int       ok = 0;
  FRESULT   res;
  char*     subDir;
  char*     pPath;
  char*     fileName;
  char*     ext;

  if(osSemaphoreAcquire(fatSemaphore, portMAX_DELAY) == osOK)
  {
    fileName = (char*)malloc(strlen(fileNameExt));
    strcpy((char*)fileName, (char*)fileNameExt);
      
    if (extF == NULL)
    {
      ext = strchr(fileName, '.');
      if (ext)
      {
        *ext = '\0';  // introduco terminatore per identificare il solo nome
        ext++;        // punto all'estensione
      }
    }
    else
    {
      ext = (char*)extF;
    }
    subDir = (char *)findExtDir((uint8_t*)ext);
    pPath = getFatDevicePath (fatDevice);
    if (subDir != NULL)
    {
      /* conservative memory allocation: use always fileNameExt */
      fullFileName = (uint8_t*)calloc((size_t)strlen(fileNameExt) + (size_t)strlen(subDir) + sizeof(USERPath) + 1, (size_t)1);
      /*         destination       source */
      strcpy((char *)fullFileName,      (char *)pPath);
      /*         destination       source */
      strcat((char *)fullFileName,      (char *)subDir);
      /*         destination       source */
      strcat((char *)fullFileName, (char *)"/");
      /*         destination       source */
      strcat((char *)fullFileName, (char *)fileNameExt);
    }
    else
    {
      fullFileName = (uint8_t*)calloc((size_t)strlen(fileNameExt) + sizeof(USERPath), (size_t)1);
      /*         destination       source */
      strcpy((char *)fullFileName, (char *)pPath);
      /*         destination       source */
      strcat((char *)fullFileName, (char *)fileNameExt);
    }

    res = f_stat((TCHAR*)fullFileName, &Finfo);
    
    if (res == FR_OK)
    { 
      /* the file EXIT */ 
      // get only the file length
      if (buffer == NULL)
      {
        *length = (unsigned int)Finfo.fsize;
        ok = 1;
      }
      else
      {
        res = f_open(&File, (TCHAR*)fullFileName, FA_OPEN_EXISTING | FA_READ);
        if ((res == FR_OK) || (res == FR_EXIST))
        {
          res = f_read(&File, buffer, (UINT)*length, (UINT*)&byteRead);
          if ((res == FR_OK) && (byteRead == (uint32_t)*length)) 
          { 
            ok = byteRead;
          }
        }
        f_close(&File);
      }
    }
    else
    {
      *length = (unsigned int)0;
    }
    free(fileName);
    free(fullFileName);
    osSemaphoreRelease(fatSemaphore); 
  }
  return ok;
}

/**
*
* @brief        Prepare FAT Fs on QSPI Flash with a full format
*
* @param [in]   none
*
* @retval       FRESULT: FR_OK if format OK and all sub dir created 
*
***********************************************************************************************************************/
FRESULT deviceFatFormat (fatDevice_e fatDevice)
{
  FRESULT   res;
  uint8_t   ix;
  uint8_t*  fullFileName;
  char      *pPath;
  
  pPath     = getFatDevicePath (fatDevice);

  // FatFs Initialization: format the flash device 
  res = f_mkfs((TCHAR const*)pPath, FM_FAT, 0, buffWork, sizeof(buffWork));
  if(res == FR_OK)
  {
    /* try to re-mount the volume */
    if(f_mount(&USERFatFS, (TCHAR const*)pPath, 1) == FR_OK)
    {
      tPrintf("Format&mount eseguito!\n\r");
      /* now we create the directory structures */
      for (ix = 0; ix < NUM_DIR; ix++)
      {
        fullFileName = (uint8_t*)calloc((size_t)strlen((char*)flashDirInfo[ix].dirName) + sizeof(USERPath), (size_t)1);
        /*         destination       source */
        strcpy((char *)fullFileName, (char *)pPath);
        /*         destination       source */
        strcat((char *)fullFileName, (char *)flashDirInfo[ix].dirName);
        res = f_mkdir ((TCHAR*)fullFileName); /* Create a sub directory */ 
        if(res != FR_OK)
        {
          tPrintf("Error creating %s directory. Code=%d\n\r", fullFileName, (uint16_t)res);
          free(fullFileName); 
          break; 
        }
        free(fullFileName); 
      }

    }
    else
    {
      tPrintf("Error mounting the device. Code=%d\n\r", (uint16_t)res);
    }
  }
  else
  {
    tPrintf("Error Formatting the device. Code=%d\n\r", (uint16_t)res);
  }
  return(res);
}

/**
*
* @brief        Change default message when new string info arrive
*
* @param [in]   uint8_t*: pointer to new string
*
* @retval       FRESULT: FR_OK if new file is created successfully 
*
***********************************************************************************************************************/
FRESULT newWelcomeFile (uint8_t* pNewInfo)
{
  FRESULT   res = FR_OK;
  return(res);
}

/**
*
* @brief        Find the directory linked to extension
*
* @param [in]   uint8_t*: pointer to ext string
*
* @retval       uint8_t*: pointer to dir name; NULL if doesn't exist  
*
***********************************************************************************************************************/
uint8_t* findExtDir (uint8_t* ext)
{
  uint8_t   id, ip;

  if (ext == NULL)
  {
    return(NULL);
  }
  else
  {
    *ext = tolower(*ext);
    *(ext + 1)= tolower(*(ext + 1));
    *(ext + 2)= tolower(*(ext + 2));
  }

  for (id = 0; id < NUM_DIR; id++)
  {
    for (ip = 0; ip < FLASH_FAT_MAX_EXT_IN_DIR; ip++)
    {
      if (strncmp((char*)flashDirInfo[id].pExtInside[ip], (char*)ext, 3) == 0)
      {
        return ((uint8_t*)flashDirInfo[id].dirName);
      }
    }
  }
  return(NULL);
}

 
/**
*
* @brief        check if file exist 
*
* @param [in]   uint8_t*: pointer to file name
*
* @retval       uint16_t: lenght file if exist, 0 if doesn't exist or error  
*
***********************************************************************************************************************/
uint32_t fileExist (uint8_t* fileNameExt, fatDevice_e fatDevice)
{
  uint8_t*  fullFileName;
  FRESULT   res;
  char*     subDir;
  char*     pPath;
  char*     fileName;
  char*     ext;  

  fileName = (char*)malloc(strlen((char*)fileNameExt));
  strcpy((char*)fileName, (char*)fileNameExt);
    
  ext = strchr(fileName, '.');
  if (ext)
  {
    *ext = '\0';  // introduco terminatore per identificare il solo nome
    ext++;        // punto all'estensione
  }
  subDir = (char *)findExtDir((uint8_t*)ext);
  pPath = getFatDevicePath (fatDevice);
  if (subDir != NULL)
  {
    //subDir = (char *)dirFileName;
    fullFileName = (uint8_t*)calloc((size_t)strlen((char*)fileNameExt) + (size_t)strlen(subDir) + sizeof(USERPath) + 1, (size_t)1);
    /*         destination       source */
    strcpy((char *)fullFileName,      (char *)pPath);
    /*         destination       source */
    strcat((char *)fullFileName,      (char *)subDir);
    /*         destination       source */
    strcat((char *)fullFileName, (char *)"/");
    /*         destination       source */
    strcat((char *)fullFileName, (char *)fileNameExt);
  }
  else
  {
    fullFileName = (uint8_t*)calloc((size_t)strlen((char*)fileNameExt) + sizeof(USERPath), (size_t)1);
    /*         destination       source */
    strcpy((char *)fullFileName, (char *)pPath);
    /*         destination       source */
    strcat((char *)fullFileName, (char *)fileNameExt);
  }

  res = f_stat((TCHAR*)fullFileName, &Finfo);
  
  free(fileName);
  free(fullFileName); 
    
  if (res == FR_OK)
  { 
    return((uint32_t)Finfo.fsize);
  }
  else
  {
    return((uint32_t)0);
  }
}

/**
*
* @brief        check if a directory exist 
*
* @param [in]   uint8_t*: pointer to dir name
*
* @retval       FRESULT: FR_OK if directory exists 
*
***********************************************************************************************************************/
FRESULT dirExist (uint8_t* dirName, fatDevice_e fatDevice)
{
  FRESULT   res;
  uint8_t*  fullFileName;
  char*     pPath;

  res = FR_INT_ERR;
  if(osSemaphoreAcquire(fatSemaphore, portMAX_DELAY) == osOK)
  {
    pPath = getFatDevicePath (fatDevice);
    /* conservative memory allocation */
    fullFileName = (uint8_t*)calloc((size_t)strlen((char*)dirName) + sizeof(USERPath) + 1, (size_t)1);
    /*         destination       source */
    strcpy((char *)fullFileName,      (char *)pPath);
    /*         destination       source */
    strcat((char *)fullFileName,      (char *)dirName);
    /* check for directory */
    res = f_opendir(&Dir, (TCHAR*)fullFileName);

    free(fullFileName);
    osSemaphoreRelease(fatSemaphore); 
  }
  return(res);
}

/**
*
* @brief        check if a directory exist 
*
* @param [in]   uint8_t*: pointer to dir name
* @param [in]   uint8_t*: pointer to ext dir name content
*
* @retval       FRESULT: FR_OK directory and inside files removed 
*
***********************************************************************************************************************/
FRESULT dirRemove (uint8_t* dirName, uint8_t* ext)
{
  uint8_t*  fullFileName;
  char*     pPath;
  uint16_t  s1;
  FRESULT   res;
  char      *subDir, *pDirName;

  res = FR_INT_ERR;
  if(osSemaphoreAcquire(fatSemaphore, portMAX_DELAY) == osOK)
  {
    pPath = getFatDevicePath (FAT_DEVICE_FLASH);
    subDir = (char *)findExtDir((uint8_t*)ext);
    if (subDir != NULL)
    {
      /* conservative memory allocation */
      fullFileName = (uint8_t*)calloc((size_t)strlen((char*)dirName) + (size_t)strlen(subDir) + sizeof(USERPath) + 1, (size_t)1);
      /*         destination       source */
      strcpy((char *)fullFileName,      (char *)pPath);
      strcat((char *)fullFileName, (char *)subDir);
      if (dirName != NULL)
      {
        /*         destination       source */
        strcat((char *)fullFileName, (char *)"/");
        /*         destination       source */
        strcat((char *)fullFileName,      (char *)dirName);
      }
      /* check for directory */
      res = f_opendir(&Dir, (TCHAR*)fullFileName);
      if (res == FR_OK)
      {
        s1 = 0;
        for(;;) 
        {
          res = f_readdir(&Dir, &Finfo);
          if ((res != FR_OK) || !Finfo.fname[0]) break;
          if ((Finfo.fattrib & AM_DIR) != AM_DIR) 
          {
            pDirName = (char*)calloc(((size_t)strlen((char*)fullFileName) + (size_t)strlen((char*)Finfo.fname) + 2), (size_t)1);
            /*         destination       source */
            strcpy((char *)pDirName,      (char *)fullFileName);
            /*         destination       source */
            strcat((char *)pDirName, (char *)"/");
            /*         destination       source */
            strcat((char *)pDirName,      (char *)Finfo.fname);
            res = f_unlink((TCHAR*)pDirName);  /* Delete an existing file */
            free(pDirName);
            if (res == FR_OK)
            {
              s1++;
            }
            else
            {
              break;
            }
          }
        }
        if (res != FR_OK) 
        { 
          tPrintf("Error deleting directory: code=%d\n\r", (uint16_t)res);
        }
        else
        {
          res = f_unlink((TCHAR*)fullFileName);  /* Delete an existing directory */
          if (res == FR_OK)
          {
            tPrintf("Directory deleted (inside %d File deleted)\n\r", s1);
          }
        }
      }
      else
      {
        if (res == FR_NO_PATH)
        {
          /* dir not exit, already deleted */
          res = FR_OK;
        }
      }
      f_closedir(&Dir);
      free(fullFileName);
    }
    osSemaphoreRelease(fatSemaphore); 
  }
  return(res);
}

/**
*
* @brief        Create the semaphore to FAT access  
*
* @param [in]   none
*
* @retval       none  
*
***********************************************************************************************************************/
void  fatSemaphoreCreate (void)
{
  /* create a binary semaphore used for FAT access */
  fatSemaphore = osSemaphoreNew(1, 1, NULL);

  if( fatSemaphore == NULL )
  {
    _ErrorFat_Handler(__FILE__, __LINE__);
  }
}

/**
*
* @brief        give back  the semaphore to FAT access  
*
* @param [in]   none
*
* @retval       osSemaphoreId: the semaphore id  
*
***********************************************************************************************************************/
osSemaphoreId  getFatSemaphore (void)
{
  return(fatSemaphore);
}

/**
*
* @brief        Store info for new file uploaded from FTP   
*
* @param [in]   char*: string file full path
* @param [in]   uint32_t: file length
* @param [in]   uint32_t: data origin address in QSPI flash
*
* @retval       none  
*
***********************************************************************************************************************/
void storeInfoFileFtp (char* name, uint32_t len, uint32_t addr)
{
  char    c;
  uint8_t i;

  /*         destination       source */
  strcpy((char *)uplFileInfo.name, (char *)name);
  /* conviene convertire tutto in maiuscolo */
  i = 0;
  while (uplFileInfo.name[i])
  {
    c = uplFileInfo.name[i];
    uplFileInfo.name[i]=  toupper(c);
    i++;
  }
  uplFileInfo.length = len;
  uplFileInfo.address = addr;
}

/**
*
* @brief        put in the FAT the new file uploaded from FTP (all info in uplFileInfo strcture)  
*
* @param [in]   char*: string file full path
* @param [in]   uint32_t: file length
* @param [in]   uint32_t: data origin address in QSPI flash
*
* @retval       uint8_t: error status FALSE = no Errors = all OK 
*
***********************************************************************************************************************/
uint8_t moveFileFtpInFat (void)
{
	infoFw_st infoFw;
  uint32_t  cksum, addr, len, deltaBuffLen;
  uint16_t  ix;
  uint8_t   error;

  error = FALSE;
  /* check if a new fw file has been downloaded */
  if (strncmp(uplFileInfo.name, FW_SCU_PREFIX, strlen(FW_SCU_PREFIX)) == 0)
  {
    if (strstr(uplFileInfo.name, ".BIN") != NULL)
    {
      if(osSemaphoreAcquire(fatSemaphore, portMAX_DELAY) == osOK)
      {
        /* name and ext is referred a new FW file: checksum verify  */
        if (raw_sFLASH_Read((uint8_t*)&infoFw.ch16, uplFileInfo.address, sizeof(infoFw_st)) == HAL_OK)
        {
          deltaBuffLen = sizeof(buffWork); /* copy using 4K buffer */
          for (addr = uplFileInfo.address + MAX_FW_INFO_SIZE, len = 0U, cksum = 0U; len < infoFw.fwLen; )
          {
            if (raw_sFLASH_Read((uint8_t*)buffWork, addr, deltaBuffLen) == HAL_OK)
            {
              for (ix = 0; ix < deltaBuffLen; ix++)
              {
                cksum += buffWork[ix];
              }
            }
            len += deltaBuffLen;
            addr += deltaBuffLen; // next address for data 
            if ((infoFw.fwLen - len) < deltaBuffLen)
            {
              /* last chunk of file < 4K */
              deltaBuffLen = infoFw.fwLen - len;
            }
            refreshWD();
          } // end for 
          if (cksum != infoFw.ch16)
          {
            error = TRUE;
          }
        }
      } // end semaphore
      osSemaphoreRelease(fatSemaphore); 
    }
    else
    {
      error = TRUE;
    }
  }
  if (error == FALSE)
  {
    /* the FW directory must be empty */
    removeFilesInDir("FW");
    if (uplFileInfo.length != (uint32_t)FlashFatWriteFile(uplFileInfo.name, (char *)uplFileInfo.address, uplFileInfo.length, NULL, FAT_DEVICE_FLASH))
    {
      error = TRUE;
    }
  }
  return(error);
}

/**
*
* @brief        Get the start address for new firmware area in SDRAM  
*
* @param [in]   none
*
* @retval       uint8_t*: pointer to new FW in SDRAM area  
*
***********************************************************************************************************************/
__weak unsigned int* getNewFWsdramAddress(void)
{
#ifdef DA_FARE
  return ((unsigned int*)(NEW_FW_SDRAM_ADDRESS));
#else
  return ((unsigned int*)(0));
#endif
}


/**
*
* @brief        Set the flag for new FW available  
*
* @param [in]   none
*
* @retval       none  
*
***********************************************************************************************************************/
__weak void setNewFWavailable (unsigned int FWLength) 
{
  ;
}

/**
*
* @brief        Set flag on NVIC reset      
*
* @param [in]   none
*
* @retval       none 
*
***********************************************************************************************************************/
void setFlagForNvic(void)
{
  HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), BACKUP_NVIC_RESET_REG, BACKUP_NVIC_RESET_VAL);
}

/**
*
* @brief        Reset flag on NVIC reset      
*
* @param [in]   none
*
* @retval       none 
*
***********************************************************************************************************************/
void resetFlagForNvic(void)
{
  HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), BACKUP_NVIC_RESET_REG, BACKUP_NVIC_RESET_DEF_VAL);
}

/**
*
* @brief        check flag on NVIC reset      
*
* @param [in]   none
*
* @retval       uint8_t: TRUE if reset has been originated by NVIC_SystemReset call 
*
***********************************************************************************************************************/
uint8_t checkFlagForNvic(void)
{
  uint32_t  val;

  val = HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)getHandleRtc(), BACKUP_NVIC_RESET_REG);
  if (val == BACKUP_NVIC_RESET_VAL)
  {
    return ((uint8_t)TRUE);
  }
  return ((uint8_t)FALSE);
}

/**
*
* @brief        give back  the FAT free space  
*
* @param [in]   none
*
* @retval       uint32_t: num free bytes   
*
***********************************************************************************************************************/
uint32_t  getFatFreeSpace (void)
{
  long        p1;
  FATFS*      fs;
  uint32_t    temp;
  FRESULT     res;

  (void)FlashRead((uint32_t)0x40000, (uint8_t*)&temp, (uint32_t)2);
  // ONLY FOR DEBUG -->osDelay(100);
  res = f_getfree(USERPath, (DWORD*)&p1, &fs);
  temp = (uint32_t)0;
  if (res == FR_OK)
  {
    temp = (uint32_t)(p1 * fs->csize * BLOCK_SIZE);
  }
  return(temp);
}




/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void _ErrorFat_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}


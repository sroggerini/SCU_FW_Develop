/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>

#include "fatfs.h"
#include "rtcApi.h"

#pragma data_alignment=4
uint8_t buffWork[4096];	/* Working buffer */

#pragma data_alignment=2
char USERPath[4];   /* USER logical drive path */
FATFS USERFatFS;    /* File system object for USER logical drive */
FIL USERFile;       /* File object for USER */
uint8_t retUSER;    /* Return value for USER */

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */    

void MX_FATFS_Init(void) 
{
  /*## FatFS: Link the USER driver ###########################*/
  retUSER = FATFS_LinkDriver(&USER_Driver, USERPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */     
  /* USER CODE END Init */
}

/**
*
* @brief       Give back the pointer to device string path used for FATFS
*
* @param [in]  fatDevice_e: device identifier 
*  
* @retval      char*: pointer to device path string 
*  
****************************************************************/
char* getFatDevicePath (fatDevice_e fatDevice)
{
  char* pDevPath;
 
  switch (fatDevice)
  {
    case FAT_DEVICE_FLASH:
      pDevPath = USERPath;
      break;

    default:
      pDevPath = USERPath;
      break;
  }
  return(pDevPath);
}


/**
  * @brief  Gets Time from RTC 
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  /*
    Come torna la fat time? Così:
    bit31:25
    Year origin from the 1980 (0..127, e.g. 37 for 2017)
    bit24:21
    Month (1..12)
    bit20:16
    Day of the month(1..31)
    bit15:11
    Hour (0..23)
    bit10:5
    Minute (0..59)
    bit4:0
    Second / 2 (0..29, e.g. 25 for 50)  
    
  link: http://elm-chan.org/fsw/ff/doc/fattime.html
  */
  
  DWORD fatTime = 0;
  UpdateGlobalDT();
  fatTime |= (GlobalDT.Year - 1980) << 25;
  fatTime |= (GlobalDT.Month) << 21;
  fatTime |= (GlobalDT.Day) << 16;
  fatTime |= (GlobalDT.Hour) << 11;
  fatTime |= (GlobalDT.Minute) << 5;
  fatTime |= (GlobalDT.Second / 2);
  
  return fatTime;  
  /* USER CODE END get_fattime */  
}

/* USER CODE BEGIN Application */
/**
  * @brief  Create a string from fatTime value 
  * @param[in] uint32_t:   fat time value 
  * @param[in] char*:   string pointer where store the string time  
  * @retval none
  */
void  strRTCDateTime( char * str, uint32_t fatTime )
{
  uint32_t temp;
  
  temp = (uint32_t)(fatTime >> 25) & (uint32_t)(0x0000007F);
  uint16_t year   = (uint16_t)((uint32_t)1980 + temp);
  uint16_t month  = (fatTime >> 21) & (0x0000000F);
  uint16_t day    = (fatTime >> 16) & (0x0000001F);
  uint16_t hour   = (fatTime >> 11) & (0x0000001F);
  uint16_t minute = (fatTime >> 5)  & (0x0000003F);
  uint16_t second = ((fatTime >> 0) & (0x0000001F)) * 2;

  sprintf(str, "%04d-%02d-%02d %02d:%02d:%02d", year, month, day, hour, minute, second);
}

// Create a string from local time
// string str must be at least 20 characters long

char * strLocalTime( char * str )
{
  uint32_t locTime;

  locTime = get_fattime();
  strRTCDateTime(str, locTime);
  return(str);
}

/**
  * @brief  Create a string from time in msec  
  * @param[in] uint32_t:   time value in msec  
  * @param[in] char*:   string pointer where store the string time  
  * @retval none
  */
void strmSec2hms( char * str, uint32_t msec)
{
  uint32_t min, hour, sec;
  char * pe = str;

  sec = msec / 1000UL;
  hour = sec / 3600UL;
  sec %= 3600UL;
  min = sec / 60UL;
  sec %= 60UL;
  msec %= 1000UL; 

  if( hour > 0 )
  {
    sprintf(str, "%02dh ", hour); 
    pe = strchr( str, 0 );
  }
  if( min > 0 )
  {
    sprintf(pe, "%02dm ", min); 
    pe = strchr( pe, 0 );
  }
  if( sec > 0 || ( min == 0 && hour == 0 ))
  {
    sprintf(pe, "%02ds ", sec); 
    pe = strchr( pe, 0 );
  }
  if( msec > 0 || ( sec == 0 && min == 0 && hour == 0 ))
  {
    sprintf(pe, "%02dms", msec); 
  }
}
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

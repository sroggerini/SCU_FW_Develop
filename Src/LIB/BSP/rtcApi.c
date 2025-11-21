/**
* @file        rtcApi.c
*
* @brief       API for RTC - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: rtcApi.c 708 2025-02-28 17:54:37Z npiergi $
*
*     $Revision: 708 $
*
*     $Author: npiergi $
*
*     $Date: 2025-02-28 18:54:37 +0100 (ven, 28 feb 2025) $
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
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "main.h"
#include "rtcApi.h"
#include "eeprom.h"
#include "wrapper.h"

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#define   MAX_DELTA_TIME      ((uint32_t)15)  /*** max 15 sec of delta between RTC time and current unixTime */
/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 
static  legalPeriod_st  legalPeriod;
static  char            tickStr[10];

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 
struct DataAndTime_t GlobalDT;
RTC_DateTypeDef sTamperDate;  
RTC_TimeTypeDef sTamperTime;
uint8_t TamperDetected = FALSE;

/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/
extern RTC_HandleTypeDef hrtc;
extern uint32_t BKP_SRAM_UnixTimestamp;
extern infoStation_t  infoStation;

/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Function                             **
**                                                                          **
******************************************************************************
*/

extern unsigned char WriteOnEeprom (unsigned short Address, unsigned char *Buffer, unsigned short Length);


/*
*********************************** SCAME ************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
static void   rtcError_Handler    (void);

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/

/**
*
* @brief       GetDateTime_from_Unix
*
* @param [in]  Unix timestamp 
*  
* @retval      Date and time struct
*  
****************************************************************/

struct DataAndTime_t GetDateTime_from_Unix (uint32_t UnixTimestamp)  /* Ticket SCU-100 */
{ 
  
  time_t                time;
  struct tm             *pTimeinfo;
  struct DataAndTime_t   DT_Value;
  
  time = (time_t)UnixTimestamp;
  
  pTimeinfo = localtime (&time);
  /* Translate into Date and time format */
  DT_Value.Second = pTimeinfo->tm_sec;      
  DT_Value.Minute = pTimeinfo->tm_min;     
  DT_Value.Hour   = pTimeinfo->tm_hour;       
  DT_Value.Day    = pTimeinfo->tm_mday;        
  DT_Value.Month  = pTimeinfo->tm_mon + 1;  
  DT_Value.Year   = pTimeinfo->tm_year + 1900;
  DT_Value.DayWeek = (pTimeinfo->tm_wday == 0) ? 7 : pTimeinfo->tm_wday;
  /* get abilitazione ora legale      */  
  // eeprom_param_get(DST_EADD, (uint8_t *)&DT_Value.dstFlag, 1);
  DT_Value.dstFlag = infoStation.Time_Settings.dst;
  
  return (DT_Value);
  
}

/**
*
* @brief       DateTimeSet
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/
void DateTimeSet(struct DataAndTime_t *DT_Set)
{
  RTC_DateTypeDef       sdatestructure;
  RTC_TimeTypeDef       stimestructure;
  uint32_t              prevTimeSet;
  struct tm             structUnixTime = {0};
  uint32_t              currentUnixTime, newUnixTime, deltaSet, deltaSec, n32Bk;
  uint32_t              currCalp, currCalm, newCalm;
  struct DataAndTime_t  DT_Get;
	unsigned char         YearTemp, dataValid;


  
  /* Compute Unix time for new Date and time value */
  structUnixTime.tm_sec  = (int)DT_Set->Second;      
  structUnixTime.tm_min  = (int)DT_Set->Minute;      
  structUnixTime.tm_hour = (int)DT_Set->Hour;        
  structUnixTime.tm_mday = (int)DT_Set->Day;         
  structUnixTime.tm_mon  = (int)DT_Set->Month - 1;   
  structUnixTime.tm_year = (int)DT_Set->Year - 1900;
  structUnixTime.tm_wday = (DT_Set->DayWeek == 7) ? (int)0 : (int)(DT_Set->DayWeek - 1);;
  newUnixTime = (uint32_t)mktime((struct tm *)&structUnixTime);  

  /* unix time for current RTC value */
  currentUnixTime = newUnixTime;
  // xx eeprom_param_get(RTC_VALID_EADD, &dataValid, 1);
   if (infoStation.rtcValid == 0x01)
  {
    DateTimeGet(&DT_Get);
    structUnixTime.tm_sec  = (int)DT_Get.Second;
    structUnixTime.tm_min  = (int)DT_Get.Minute;
    structUnixTime.tm_hour = (int)DT_Get.Hour;
    structUnixTime.tm_mday = (int)DT_Get.Day;
    structUnixTime.tm_mon  = (int)DT_Get.Month - 1;
    structUnixTime.tm_year = (int)DT_Get.Year - 1900;
    structUnixTime.tm_wday = (DT_Get.DayWeek == (uint8_t)7) ? (int)0 : (int)(DT_Set->DayWeek - 1);
    currentUnixTime = (uint32_t)mktime((struct tm *)&structUnixTime);  
  }
  /* end set current RTC time */

  /* now set new time */
  YearTemp = (unsigned char)(DT_Set->Year - 2000);

  /*##-1- Configure the Date #################################################*/
  sdatestructure.Year = ((YearTemp / 10) << 4) + (YearTemp % 10);
  sdatestructure.Month = ((DT_Set->Month / 10) << 4) + (DT_Set->Month % 10);
  sdatestructure.Date = ((DT_Set->Day / 10) << 4) + (DT_Set->Day % 10);
  sdatestructure.WeekDay = DT_Set->DayWeek;
  
  if(HAL_RTC_SetDate(&hrtc,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    rtcError_Handler();
  }

  /*##-2- Configure the Time #################################################*/
  /* Set Time: 02:00:00 */
  stimestructure.Hours = ((DT_Set->Hour / 10) << 4) + (DT_Set->Hour % 10);
  stimestructure.Minutes = ((DT_Set->Minute / 10) << 4) + (DT_Set->Minute % 10);
  stimestructure.Seconds = ((DT_Set->Second / 10) << 4) + ((DT_Set->Second % 10) & 0x7F);
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;

  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET; 

  if (HAL_RTC_SetTime(&hrtc, &stimestructure, RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    rtcError_Handler();
  }
  /* calibration */
  prevTimeSet = getLastRtcSetTime();
  if ((prevTimeSet > MIN_UNIX_TIME_VAL) && (infoStation.rtcValid == 0x01))
  {
    deltaSet = (uint32_t)difftime(newUnixTime, prevTimeSet);

    if (deltaSet > MIN_TIME_BETWEEN_CALIB)
    {
      if (newUnixTime < currentUnixTime)
      {
        /* rtc goes fast: clock must be reduced */
        deltaSec = currentUnixTime - newUnixTime; // sec to be compensate 
        /* we use a compensation over a block of 32 sec */
        n32Bk = deltaSet / CAL_BLOCK_LEN;
        /* for each of this block we must compensate deltaSec */
        newCalm = (uint32_t)((FREQ_RTC * (float)deltaSec) / (float)n32Bk);
        getRtcCalValues(&currCalp, &currCalm);
        if (currCalp == RTC_SMOOTHCALIB_PLUSPULSES_SET)
        {
          if ((newCalm + currCalm) < MAX_CALM_VALUE)
          {
            setNewRtcCal (RTC_SMOOTHCALIB_PLUSPULSES_SET, newCalm + currCalm);
          }
          else
          {
            if (((newCalm + currCalm) - MAX_CALM_VALUE) < MAX_CALM_VALUE)
            {
              setNewRtcCal (RTC_SMOOTHCALIB_PLUSPULSES_RESET, ((newCalm + currCalm) - MAX_CALM_VALUE));
            }
            else
            {
              setNewRtcCal (RTC_SMOOTHCALIB_PLUSPULSES_RESET, (MAX_CALM_VALUE - 1));
            }
          }
        }
        else
        {
          if ((newCalm + currCalm) < MAX_CALM_VALUE)
          {
            setNewRtcCal (RTC_SMOOTHCALIB_PLUSPULSES_RESET, currCalm + newCalm);
          }
          else
          {
            /* no more calibration possible */
          }
        }
      }
      else
      {
        /* rtc goes slow: clock must be accelerated */
        deltaSec = newUnixTime - currentUnixTime; // sec to be compensate 
        /* we use a compensation over a block of 32 sec */
        n32Bk = deltaSet / CAL_BLOCK_LEN;
        /* for each of this block we must compensate deltaSec */
        newCalm = (uint32_t)((FREQ_RTC * (float)deltaSec) / (float)n32Bk);
        getRtcCalValues(&currCalp, &currCalm);
        if (currCalp == RTC_SMOOTHCALIB_PLUSPULSES_RESET)
        {
          if (newCalm < currCalm)
          {
            setNewRtcCal (RTC_SMOOTHCALIB_PLUSPULSES_RESET, currCalm - newCalm);
          }
          else
          {
            if ((newCalm - currCalm) < MAX_CALM_VALUE)
            {
              setNewRtcCal (RTC_SMOOTHCALIB_PLUSPULSES_SET, (MAX_CALM_VALUE - (newCalm - currCalm)));
            }
          }
        }
        else
        {
          if (currCalm > newCalm)
          {
            setNewRtcCal (RTC_SMOOTHCALIB_PLUSPULSES_SET, currCalm - newCalm);
          }
          else
          {
            /* no more calibration possible */
          }
        }
      }
    }
  }
  else
  {
    /* first setting */
    setFlagOnRtcBck(BACKUP_UNIX_SET_RTC, currentUnixTime);
  }
}

/**
*
* @brief       DateTimeGet
*
* @param [in]  struct DataAndTime_t *: pointer to struct where 
*        put current date and time 
*  
* @retval      none 
*  
****************************************************************/
void DateTimeGet(struct DataAndTime_t *DT_Get)
{

  UpdateGlobalDT();

  DT_Get->Year    = GlobalDT.Year; 
  DT_Get->Month   = GlobalDT.Month; 
  DT_Get->Day     = GlobalDT.Day;   
  DT_Get->Hour    = GlobalDT.Hour;  
  DT_Get->Minute  = GlobalDT.Minute;
  DT_Get->Second  = GlobalDT.Second;
  DT_Get->DayWeek = GlobalDT.DayWeek;
}

/**
*
* @brief       Get hour, minutes, seconds in a single string
*
* @param [in]  none 
*  
* @retval      char*:  pointer to formatted string  
*  
****************************************************************/
char* getHmsStr(void)
{

  
  UpdateGlobalDT();
  sprintf(tickStr, "%02d:%02d:%02d", GlobalDT.Hour, GlobalDT.Minute, GlobalDT.Second);
  return (tickStr);
}


/**
*
* @brief       Update gloabal structure for time and data 
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/
void UpdateGlobalDT(void)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;

  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

  GlobalDT.Year    = 2000 + sdatestructureget.Year;
  GlobalDT.Month   = sdatestructureget.Month;
  GlobalDT.Day     = sdatestructureget.Date;
  GlobalDT.DayWeek = sdatestructureget.WeekDay;
  GlobalDT.Hour    = stimestructureget.Hours;
  GlobalDT.Minute  = stimestructureget.Minutes;
  GlobalDT.Second  = stimestructureget.Seconds;
}

/**
*
* @brief       Get current local time Take in account Time zone 
*              and DST if present
*
* @param [in]  uint32_t: pointer whre to store the string for 
*        data
*  
* @retval      struct DataAndTime_t *: pointer to struct where 
*        put current date and time 
*  
****************************************************************/
struct DataAndTime_t* getCurrentLocalTime(uint32_t* pDTstring)
{
  uint32_t              rtcUT;
  time_t                currUT;
  struct tm             *pTimeinfo;

  rtcUT = (uint32_t)getCurrentUnixTime();
  
  currUT = (time_t)(rtcUT);


  /* update RTC when the delta time is more than the ammissible maximum **/
  pTimeinfo = localtime (&currUT);
  if (legalPeriod.tZone > (char)0)
  {
    rtcUT += (uint32_t)legalPeriod.tZone * (uint32_t)3600;  // add seconds for time zone 
  }
  else
  {
    rtcUT += (uint32_t)legalPeriod.tZone * (uint32_t)3600;  // add seconds for time zone 
    rtcUT -= (uint32_t)legalPeriod.tZone * (uint32_t)3600;  // subtract seconds for time zone 
  }

  /* get dst flag         */
  if (legalPeriod.oraLegFlag != (char)0)
  {
    /*  DST period  status    *****/
    if (legalPeriod.dstStatus > (uint8_t)0)
    {
      rtcUT += (uint32_t)3600; // ad 1h for DST
    }
  }

  currUT = (time_t)(rtcUT);

  /* update globat date and time structure  **/
  pTimeinfo = localtime (&currUT);

  GlobalDT.Second     = pTimeinfo->tm_sec;      
  GlobalDT.Minute     = pTimeinfo->tm_min;     
  GlobalDT.Hour       = pTimeinfo->tm_hour;       
  GlobalDT.Day        = pTimeinfo->tm_mday;        
  GlobalDT.Month      = pTimeinfo->tm_mon +  1;  
  GlobalDT.Year       = pTimeinfo->tm_year + 1900;
  GlobalDT.DayWeek    = (pTimeinfo->tm_wday == 0) ? 7 : pTimeinfo->tm_wday;
  GlobalDT.dstFlag    = pTimeinfo->tm_isdst;

  *pDTstring = (uint32_t)asctime (pTimeinfo);

  return((struct DataAndTime_t*)&GlobalDT);
}

/**
*
* @brief       Get day of week from date 
*
* @param [in]  struct DataAndTime_t *: pointer to struct where 
*        put current date and time 
*  
* @retval      uint8_t: day of week, 0=Sun, 1=Mon...6=Sat 
*  
****************************************************************/
uint8_t getDayOfWeek(struct DataAndTime_t * pLocDateTime)
{
  int                    d;             //Day     1-31
  int                    m;             //Month   1-12
  int                    y;             //Year    2013
  int                    weekD ;        // WeekDay ( Sunday - 0,Monday- 1,..... but for ST RTC Monday- 1, ....Sunday - 7)
  struct DataAndTime_t   locDateTime;

  if (pLocDateTime == NULL)
  {
    DateTimeGet(&locDateTime);
    pLocDateTime = (struct DataAndTime_t *)&locDateTime;
  }
  d = (int)pLocDateTime->Day;
  m = (int)pLocDateTime->Month;
  y = 2000 + (int)pLocDateTime->Year;
  weekD  = (d+=m<3?y--:y-2,23*m/9+d+4+y/4-y/100+y/400)%7;
  return ((uint8_t)(weekD));  
}

/**
*
* @brief       get current unix time 
*
* @param [in]  none  
*  
* @retval      uint32_t: the unix time  
*  
****************************************************************/
time_t getCurrentUnixTime (void)
{
  struct tm             structUnixTime = {0};
  time_t                currentUnixTime;
  struct DataAndTime_t  DT_Get;

  /* unix time for current RTC value */
  DateTimeGet(&DT_Get);
  structUnixTime.tm_sec  = (int)DT_Get.Second;
  structUnixTime.tm_min  = (int)DT_Get.Minute;
  structUnixTime.tm_hour = (int)DT_Get.Hour;
  structUnixTime.tm_mday = (int)DT_Get.Day;
  structUnixTime.tm_mon  = (int)DT_Get.Month - 1;
  structUnixTime.tm_year = (int)DT_Get.Year - 1900;
  currentUnixTime = mktime((struct tm *)&structUnixTime);  

  return(currentUnixTime);
}

/**
*
* @brief       DateTimeSet starting from unix time (when data comes from App)
*
* @param [in]  uint32_t: the current unix time  
*  
* @retval      none 
*  
****************************************************************/
void setDateTimeFromUnixT (uint32_t NewUT)  /* Ticket SCU-100 */
{
  
  uint32_t              rtcUT, deltaT;
  struct DataAndTime_t  DT_Set;
  time_t				        currUT;

  setLegalPeriod(NewUT);

  rtcUT = (uint32_t)getCurrentUnixTime();
  
  currUT = (time_t)(NewUT);

  /* check if there is difference from current time */
  if (rtcUT > currUT)
  {
    deltaT = rtcUT - currUT;
  }
  else
  {
    deltaT = currUT -rtcUT;
  }

  if (deltaT > MAX_DELTA_TIME)
  {
    /* Get Date and Time from unix timestamp */
    DT_Set = GetDateTime_from_Unix(currUT);     /* Ticket SCU-100 */         
    /* Set RTC with this values */
    DateTimeSet(&DT_Set);
      
  }
}

/**
*
* @brief  		  Set the legal time period  
*
* @param [in]  uint32_t: the current unix time  
*  
*  
* @retval       none
***********************************************************************************************************************/
void setLegalPeriod(uint32_t locUT)
{
  struct DataAndTime_t  DT_Current;
  time_t                currentUnixTime;
  struct tm             structUnixTime = {0};
  // signed char           tZone;
  time_t                currUT;
  uint8_t               i;

  if (locUT != (uint32_t)0)
  {
    currUT = (time_t)(locUT);
  }
  else
  {
    currUT = getCurrentUnixTime();
  }

  
  /* update globat date and time structure  **/
  DT_Current = GetDateTime_from_Unix(currUT);    /* Ticket SCU-100 */      

  /* get time zone         */
  // xx eeprom_param_get(TIME_ZONE_EADD, (uint8_t *)&tZone, 1);  
  
  for (i = 31, DT_Current.Month = 3; i >= 25; i--)
  {
    DT_Current.Day = i;
    if ((dayOfWeek_e)getDayOfWeek((struct DataAndTime_t *)&DT_Current) == DOMENICA)
    {
      structUnixTime.tm_sec  = (int)0;
      structUnixTime.tm_min  = (int)0;
      structUnixTime.tm_hour = (int)2;
      structUnixTime.tm_mday = (int)i;
      structUnixTime.tm_mon  = (int)DT_Current.Month - 1;
      structUnixTime.tm_year = (int)DT_Current.Year - 1900;
      currentUnixTime = (uint32_t)mktime((struct tm *)&structUnixTime);  
      if (infoStation.Time_Settings.TimeZone > (char)0)
      {
        currentUnixTime -= (uint32_t)infoStation.Time_Settings.TimeZone * (uint32_t)3600;  // subtract  seconds for time zone 
      }
      else
      {
        currentUnixTime += (uint32_t)infoStation.Time_Settings.TimeZone * (uint32_t)3600;  // add seconds for time zone 
      }
      legalPeriod.startDstUnixTime = currentUnixTime;
      break;
    }
  }

  for (i = 31, DT_Current.Month = 10; i >= 25; i--)
  {
    DT_Current.Day = i;
    if ((dayOfWeek_e)getDayOfWeek((struct DataAndTime_t *)&DT_Current) == DOMENICA)
    {
      structUnixTime.tm_sec  = (int)0;
      structUnixTime.tm_min  = (int)0;
      structUnixTime.tm_hour = (int)2;
      structUnixTime.tm_mday = (int)i;
      structUnixTime.tm_mon  = (int)DT_Current.Month - 1;
      structUnixTime.tm_year = (int)DT_Current.Year - 1900;
      currentUnixTime = (uint32_t)mktime((struct tm *)&structUnixTime);  
      if (infoStation.Time_Settings.TimeZone > (char)0)
      {
        currentUnixTime -= (uint32_t)infoStation.Time_Settings.TimeZone * (uint32_t)3600;  // subtract  seconds for time zone 
      }
      else
      {
        currentUnixTime += (uint32_t)infoStation.Time_Settings.TimeZone * (uint32_t)3600;  // add seconds for time zone 
      }
      legalPeriod.endDstUnixTime = currentUnixTime;
      break;
    }
  }
  /* get abilitazione ora legale      */
  // xx eeprom_param_get(DST_EADD, (uint8_t *)&legalPeriod.oraLegFlag, 1);
  legalPeriod.oraLegFlag = infoStation.Time_Settings.dst;    
  /* get dst status         */
  // xx eeprom_param_get(DST_STATUS_EADD, (uint8_t *)&legalPeriod.dstStatus, 1); 
  legalPeriod.dstStatus = infoStation.Time_Settings.DstStatus;
  /* get time zone         */
  // xx eeprom_param_get(TIME_ZONE_EADD, (uint8_t *)&legalPeriod.tZone, 1);
  legalPeriod.tZone = infoStation.Time_Settings.TimeZone;

  checkLegalPeriod((uint32_t)currUT);
}

/**
*
* @brief  		  check the legal time period change 
*
* @param [in]  uint32_t: the current unix time  
*  
*  
* @retval       none
***********************************************************************************************************************/
void checkLegalPeriod(uint32_t locUT)
{
  time_t                currUT;

  if (locUT != (uint32_t)0)
  {
    currUT = (time_t)(locUT);
  }
  else
  {
    currUT = getCurrentUnixTime();
  }

  if (legalPeriod.oraLegFlag != 0)
  {
    /* in this country DST accepted */
    if ((currUT > legalPeriod.startDstUnixTime) && (currUT < legalPeriod.endDstUnixTime))
    {
      if (legalPeriod.dstStatus == (uint8_t)0)
      {
        legalPeriod.dstStatus = 1;
       /* we are inside legal period */ 
       SCU_InfoStation_Set ((uint8_t *)&infoStation.Time_Settings.DstStatus, (uint8_t*)&legalPeriod.dstStatus, 1);  /*ex DST_STATUS_EADD*/
      }
    }
    else
    {
      if (legalPeriod.dstStatus != (uint8_t)0)
      {
        legalPeriod.dstStatus = 0;
        /* we are outside legal period */ 
        SCU_InfoStation_Set ((uint8_t *)&infoStation.Time_Settings.DstStatus, (uint8_t*)&legalPeriod.dstStatus, 1); /*ex DST_STATUS_EADD*/
      }
    }
  }
  else
  {
    /*  DST null status    *****/
    if (legalPeriod.dstStatus != (uint8_t)0)
    {
      legalPeriod.dstStatus = 0;
      /* we are outside legal period */ 
      SCU_InfoStation_Set ((uint8_t *)&infoStation.Time_Settings.DstStatus, (uint8_t*)&legalPeriod.dstStatus, 1); /*ex DST_STATUS_EADD*/
    }
  }
}

/**
  * @brief  Tamper event callback function
  * @param  RTC handle
  * @retval None
  */

void HAL_RTCEx_Tamper1EventCallback(RTC_HandleTypeDef *hrtc)
{
  
   /* In case of tamper detection, get date and time */
   /* NOTE: in timestamp registers, the value of the year is not saved */
   /* ref. User Manual chapter 17.4.14 */
   if (HAL_RTCEx_GetTimeStamp(hrtc, &sTamperTime, &sTamperDate, RTC_FORMAT_BIN) == HAL_OK)
      /* Show date and time */
      TamperDetected = TRUE;
   
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void rtcError_Handler(void)
{
  while (1)
  {
  }
}

/*************** END OF FILE ******************************************************************************************/


/*
 *  scame-devolo-greenphy - PLC Modem device
 *
 *  Copyright (C) 2010  Manuele Conti (manuele.conti@archimede-energia.com)
 *  Copyright (C) 2019  Bruno Zavettieri (bruno.zavettieri@archimede-energia.com)
 *  Copyright (C) 2019  Luca Valtellina (info@vlengineering.it)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * This code is made available on the understanding that it will not be
 * used in safety-critical situations without a full and competent review.
 */

/* For standard defines and includes */
#include <stdio.h>   // For snprintf
#include <string.h>  // For strlen
#include "homeplugdev_spi.h"
#include "homeplugdev.h"
#include "spi_drv.h"
#include "telnet.h"
#include "wrapper.h"
#include "scuMdb.h"
#include "rtcApi.h"

#include "EvsMng.h"
#include "PilotMng.h"
#include "PwmMng.h"

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
static   ISO15118Msg_st  ISO15118Msg;

static   DEBUG_ISO15118_st DEBUG_ISO15118;

static   homeplugdev_info info_xchange;

/***********    END ONLY FOR DEBUG END   ****************/

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local const                                   **
**                                                                          **
****************************************************************************** 
*/ 

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/

uint32_t timerCounter[timerLast];

/*    INFO        */
string_t hpgp_fw_ver;
/* CCS2 variables */
byte_t	 ccs2_hpgp_error;

/*  Input manager queue  declaration */
xQueueHandle ISO15118Queue = NULL;

ISO15118_Info_Device_st  ISO15118_Info_Device;
ISO15118_Status_st       ISO15118_Status;

/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/

/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Function                             **
**                                                                          **
******************************************************************************
*/

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
static uint32_t ISO15118_Manager (ISO15118Msg_st* pMsg);
static void board_handle_homeplugdev (void);

static uint16_t ISO15118_homeplugdev_info_host_get(uint8_t info_sel);
static void ISO15118_homeplugdev_info_host_init(void);
/*
*********************************** SCAME ************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/

/**    Check if EVcc IS is NULL 
*
* @brief       
*
* @param [in]  none
*  
* @retval      true/false 
*  
****************************************************************/

uint8_t is_EvccId_NULL (void)
{
  
  if (ISO15118_Info_Device.evccId[0] != '0')
    return false;
  if (ISO15118_Info_Device.evccId[1] != '0')
    return false;
  if (ISO15118_Info_Device.evccId[2] != ':')
    return false;
  if (ISO15118_Info_Device.evccId[3] != '0')
    return false;
  if (ISO15118_Info_Device.evccId[4] != '0')
    return false;
  if (ISO15118_Info_Device.evccId[5] != ':')
    return false;
  if (ISO15118_Info_Device.evccId[6] != '0')
    return false;
  if (ISO15118_Info_Device.evccId[7] != '0')
    return false;
  
  return true;   /* is NULL */
  
}

/**    Convert a string to a buffer 
*
* @brief       
*
* @param [in]  pvParameters
*  
* @retval      none 
*  
****************************************************************/

void mactoasc(const char mac[6], char *str)
{
	if (str == NULL)
		return;

  sprintf (str, "%.2x:%.2x:%.2x:%.2x:%.2x:%.2x",
              mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/**
 * @brief Converts an integer number to a string and writes it into a provided buffer,
 * optionally appending it to an existing prefix within that buffer.
 *
 * This function writes the formatted string directly into the 'buffer' provided.
 * The 'buffer' must be pre-allocated by the caller and be large enough to hold
 * the resulting string, including the null terminator.
 * The function does not perform dynamic memory allocation.
 *
 * @param buffer A pointer to the character array (buffer) where the result will be stored.
 * This buffer must be large enough.
 * @param maxLength The maximum allowed length for the resulting string, including the null terminator.
 * The function will not write beyond this limit.
 * @param number The integer number to convert to a string.
 * @return void. The result is written directly into 'buffer'.
 * If the formatted string would exceed maxLength, it will be truncated.
 */
void create_string_from_number_in_buffer(char* buffer, size_t maxLength, uint32_t number)
{
    size_t remaining_space;
    size_t current_len;
    int chars_written;
    
    // Basic check for buffer validity
    if (buffer == NULL || maxLength == 0) {
        // In a real application, you might want to log an error or assert here.
        // For void function, just return to prevent crash.
        return;
    }

    // Determine the starting point for writing the number
    // If the buffer already contains a "prefix", we want to append to it.
    // Otherwise, we write from the beginning of the buffer.
    current_len = strlen(buffer);

    // Ensure we don't try to write beyond the buffer's capacity
    if (current_len >= maxLength) {
        // Buffer is already full or has no space left for the number + null terminator.
        // You might want to clear the buffer or just return.
        buffer[0] = '\0'; // Clear the buffer to indicate failure/no space
        //fprintf(stderr, "Warning: Buffer too small for initial prefix. Clearing buffer.\n");
        return;
    }

    // Calculate remaining space for the number and null terminator
    remaining_space = maxLength - current_len;

    // Use snprintf to safely write the number into the remaining space
    // It appends to the current content of 'buffer' starting from 'buffer[current_len]'
    // snprintf will ensure not to write beyond 'maxLength' relative to the start of 'buffer'.
    chars_written = snprintf(buffer + current_len, remaining_space, "%d", number);

    // Check if snprintf encountered an error or if the string was truncated
    // If chars_written is negative, an encoding error occurred.
    // If chars_written >= remaining_space, the number part was truncated.
    if (chars_written < 0 || (size_t)chars_written >= remaining_space) {
        //fprintf(stderr, "Warning: Number string was truncated or an error occurred. Needed %d, available %zu.\n", chars_written, remaining_space);
        // The buffer will still contain whatever was written, but might be truncated.
        // You might choose to set buffer[0] = '\0' here as well, depending on desired error handling.
    }
}

/**
*
* @brief       Task to manage communication with ISO15118 board
*
* @param [in]  pvParameters
*  
* @retval      none 
*  
****************************************************************/

void  ISO15118Task (void * pvParameters)
{
  uint32_t        timeTick;

  /*-------- Creates an empty mailbox for monitor messages --------------------------*/
  ISO15118Queue = xQueueCreate(ISO15118_MAX_MESSAGE_NUM, sizeof(ISO15118Msg_st));
  configASSERT(ISO15118Queue != NULL);
  
  ISO15118_homeplugdev_info_host_init();
  
  DEBUG_ISO15118.maxCurrent = 9;

  timeTick = portMAX_DELAY;

  for (;;)
  {
    /* Wait for some event  */
    if (xQueueReceive(ISO15118Queue, (void *)&ISO15118Msg, timeTick) == pdPASS)
    {
      timeTick = ISO15118_Manager((ISO15118Msg_st *)&ISO15118Msg);
    }
    else
    {
      ISO15118Msg.taskEv = ISO15118_EV_TIMEOUT;
      timeTick = ISO15118_Manager((ISO15118Msg_st *)&ISO15118Msg);
    }
  }

}

/**
*
* @brief       Function used to manage communication with ISO15118 board
*
* @param [in]  pMsg: pointer to manage message containig event and value
*  
* @retval      
*  
****************************************************************/

uint32_t ISO15118_Manager (ISO15118Msg_st* pMsg)
{
  
  uint32_t  localTimeTick;
  
  uint8_t               mdbAddr;
  scuRwMapRegister_st*  pRwRegs;

  switch (pMsg->taskEv)
  {
    
    case ISO15118_EV_START:
        mdbAddr = getLogicalMdbAddrSem();
        pRwRegs = getRwMdbRegs(mdbAddr); 
        /* Init Software timers */
        memset(timerCounter, 0, sizeof(uint32_t) * timerLast);
        /* Init SPI bus, just to have spi1handle (speeed is @125KHz in SCU) */
	spi_init(1, SPI_CK_SPEED_125K, SPI_OPEN_MSTEN | SPI_OPEN_MODE8 | SPI_MODE_3);        
        
        /* ONLY FOR DEBUG START */
        DEBUG_ISO15118.maxCurrent = pRwRegs->scuSetRegister.maxTypCurr;
        DEBUG_ISO15118.maxPower = pRwRegs->scuSetRegister.pmPmax;
        /* ONLY FOR DEBUG END */
        
        ISO15118_Status.board_check_presence = true;
        
        ISO15118_Status.tracement = 1;   // ONLY FOR DEBUG 

        /* Start ISO15118 board polling */
        localTimeTick = ISO15118_TASK_TICK;          
      break;
      
    case ISO15118_EV_TIMEOUT:
        /* Manage communication with ISO15118 board every 60ms */
        board_handle_homeplugdev ();
        localTimeTick = ISO15118_TASK_TICK;
      break;
      
    default:
      break;
      
  }
  
  return localTimeTick;
}

/**
*
* @brief       Check if the structure Info.device is changed
*
* @param [in]  None
*  
* @retval      TRUE if changed - FALSE if NOT
*  
****************************************************************/

bool _IS_ISO15118_INFO_DEVICE_CHANGED (void)
{
  if (info.device.terminateOpenV2G != ISO15118_Info_Device.Term_openv2g ||
      info.device.stateOpenV2G != ISO15118_Info_Device.State_openv2g ||
      info.device.stateSlac != ISO15118_Info_Device.State_Slac ||
      info.device.stateSDP != ISO15118_Info_Device.State_SDP ||
      info.device.errorCode != ISO15118_Info_Device.Error_Code ||
      info.device.evDepartureTime != ISO15118_Info_Device.ev_Departure_Time ||
      info.device.evEnergyRequest != ISO15118_Info_Device.ev_Energy_Request ||
      info.device.serviceIdReq != ISO15118_Info_Device.serviceIdReq)
    
     return TRUE;
  
  return FALSE;
  
}

/**
*
* @brief       Function taht send and receive messages to/from 
*              ISO15118 board
*
* @param [in]  None
*  
* @retval      None
*  
****************************************************************/

static void board_handle_homeplugdev(void)
{
  static uint8_t st = 0U;
  static uint8_t hpgp_log = 0U;

  switch (st) 
  {
    case 0:
      if (timerCounter[timerCCSHomeplug] == 0) 
      {
        st = 1;
      }
      if ((hpgp_log == 0U) && (info.device.fw_ver[0] != 0x00)) 
      {
        hpgp_log = 1U;
        /* Write Info */
        tPrintf("FW_hpgp %s\n\r", hpgp_fw_ver);
      }
    break;
  
    case 1:
      /* Update Host data to send */
      info.host.presentCurrent = 0; // ONLY FOR DC 
      info.host.presentVoltage = 0; // ONLY FOR DC 
      info.host.maxCurrent = ISO15118_homeplugdev_info_host_get(ISO15118_MAXCURRENT);
      info.host.minCurrent = 0;  // ONLY FOR DC
      info.host.maxVoltage = 230; 
      info.host.minVoltage = 0 ; // ONLY FOR DC
//      info.host.maxPower = DEBUG_ISO15118.maxPower;    // ONLY FOR DC (70 is ONLY FOR DEBUG ) USARE maxPower ???
      info.host.maxPower = (((info.host.maxCurrent * 230) * 3)/100);    // ONLY FOR DC (70 is ONLY FOR DEBUG ) USARE maxPower ???
      info.host.freeService = 1; /* 1 = External payment - 0 = Plug&Charge */
      info.host.stopEVSE = ISO15118_homeplugdev_info_host_get(ISO15118_STOPEVSE);
      info.host.emergency = DEBUG_ISO15118.freeService;   /* Emergency stop (Suspension???) */
      info.host.powerLimit = 0; // ONLY FOR DC
//      info.host.stateCP = ISO15118_get_StateCP();   /* State of CP signal (ref. Stati CP in ISO15118-2_Info_struct.xls) */
      info.host.stateCP = ISO15118_homeplugdev_info_host_get(ISO15118_STATECP);   /* State of CP signal (ref. Stati CP in ISO15118-2_Info_struct.xls) */
      info.host.cableCheckRes = 0;   // ONLY FOR DC
      info.host.sessionId[0] = '\0'; // Reset string
      create_string_from_number_in_buffer((char *)info.host.sessionId, SESSIONID_STRING_LEN, getSessionId());
      if (info.host.evseId[0] == 0) 
        strcpy((char*)info.host.evseId, "1963");       
      info.host.evseTimeStamp = getCurrentUnixTime();
      info.host.paymentOption = 1; // External payment (1) - Plug&Charge (0)
      info.host.serviceID = 1;     // 1 service if PnC is not implemented
      if (info.host.serviceName[0] == 0x00) 
      {
        strcpy((char*)info.host.serviceName, "SCAME AC");
      }
      info.host.serviceCat = 0;    // Type of service provided: 0 = EVCharging 
      info.host.entransfMode = /* ONLY FOR DEBUG --> SET AS MONO ((infoStation.modePwr == MODE_MONO_PH_NO_PM) || (infoStation.modePwr == MODE_MONO_PH_PM)) ? 0 : 1 */ 0;
      /* Check to send */
      if (timerCounter[timerHPGP] == 0U) 
         homeplugdev_readwrite_info();
      /* Update errors counter */
      if (homeplugdev_error() != 0) 
      {
        if (ccs2_hpgp_error < HOMEPLUGDEV_THR_COMM_ERRORS) 
        {
          ccs2_hpgp_error++;
        }
      } 
      else if (ccs2_hpgp_error > 0U) 
      {
         ccs2_hpgp_error--;
      }

      /* Update Device received data to device variables */
      if (ccs2_hpgp_error >= HOMEPLUGDEV_THR_COMM_ERRORS) 
      {
        /* NOT PRESENT --> delete the task */
        // ONLY FOR DEBUG --> ISO15118_Status.board_detected = false;
        // ONLY FOR DEBUG --> tPrintf ("ISO15118 card: "ANSI_COLOR_RED "DISABLED" ANSI_COLOR_RESET"\n\r");       
        // ONLY FOR DEBUG --> vTaskDelete (NULL);
        // ONLY FOR DEBUG --> st = 0U;
        ccs2_hpgp_error = 0; // ONLY FOR DEBUG
        return;
      }
      
      /* Check ISO15118 card presence */
      if ((info.device.fw_ver[0] != '\0') && ISO15118_Status.board_check_presence)
      {
        /* Notify the card detection */
        ISO15118_Status.board_check_presence = false;        
        ISO15118_Status.board_detected = true;
        ISO15118_Status.check_communication = true;
        
        ISO15118_Status.tracement = 5;          // ONLY FOR DEBUG 
        
        tPrintf ("ISO15118 card: "ANSI_COLOR_GREEN "PRESENT" ANSI_COLOR_RESET"\n\r");
      }
      
      /* Check if info.device structure is changed */
      if (_IS_ISO15118_INFO_DEVICE_CHANGED())
      {
        ISO15118_Info_Device.Term_openv2g = info.device.terminateOpenV2G;
        ISO15118_Info_Device.State_openv2g = info.device.stateOpenV2G;
        ISO15118_Info_Device.State_Slac = info.device.stateSlac;
        ISO15118_Info_Device.State_SDP = info.device.stateSDP;
        ISO15118_Info_Device.Error_Code = info.device.errorCode;
        mactoasc((const char *)info.device.evccId, (char *)ISO15118_Info_Device.evccId);
        ISO15118_Info_Device.ev_Departure_Time = info.device.evDepartureTime;
        ISO15118_Info_Device.ev_Max_Current_Limit = info.device.evMaxCurrentLimit;
        ISO15118_Info_Device.ev_Min_Current = info.device.evMinCurrent;
        ISO15118_Info_Device.ev_Max_Voltage_Limit = info.device.evMaxVoltageLimit;
        ISO15118_Info_Device.ev_Energy_Request = info.device.evEnergyRequest;
        ISO15118_Info_Device.ev_Protocol = info.device.evProtocol;
        ISO15118_Info_Device.serviceIdReq = info.device.serviceIdReq;
        send_to_evs(EVS_ISO15118_INFO_DEVICE_UPDATE);  /* Send to evs_manager that informations are changed */
      }
      st = 0U;
    break;
  }
}

/**
*
* @brief       Timer management (based on SysTick so every 1ms)
*
* @param [in]  None
*  
* @retval      None
*  
****************************************************************/

void ISO15118_systick_callback(void)
{
  
  uint32_t tmp_tick = HAL_GetTick();
  
  /* Timers management */
  static uint8_t i = 0U;  
  
  for (i = 0U; i < timerLast; i++) 
  {
    if (timerCounter[i] > 0U)
      timerCounter[i]--;
  }

}

/**
*
* @brief       
*
* @param [in]  None
*  
* @retval      None
*  
****************************************************************/

void ISO15118_Task_SendEv (ISO15118Ev_e Event)
{
  ISO15118Msg_st    ISO15118Msg;

  /* start Ethernet Init */
  ISO15118Msg.taskEv = Event;
  configASSERT(xQueueSendToBack(ISO15118Queue, (void *)&ISO15118Msg, portMAX_DELAY) == pdPASS); 
  
}

/**
*
* @brief       
*
* @param [in]  None
*  
* @retval      None
*  
****************************************************************/

static void ISO15118_homeplugdev_info_host_init(void)
{
info_xchange.host.maxCurrent = ((M3T_CURRENT_MIN + 5) / 10);
info_xchange.host.stopEVSE = 0;
}

/**
*
* @brief       
*
* @param [in]  None
*  
* @retval      None
*  
****************************************************************/

static uint16_t ISO15118_homeplugdev_info_host_get(uint8_t info_sel)
{
switch (info_sel)
    {
    case ISO15118_MAXCURRENT:
        return info_xchange.host.maxCurrent;
        break;

    case ISO15118_STOPEVSE:
        return info_xchange.host.stopEVSE;
        break;

    case ISO15118_STATECP:
        {
        switch (s2_state_get())
            {
            case S2_STATE_A:
                return ccs2StateA;
                break;

            case S2_STATE_B:
            case S2_STATE_Bx:
                {
                if (is_pwm_CP_Active() == FALSE)
                    return ccs2StateB1;
                else
                    return ccs2StateB2;  /* EV is connected to the charging station with PWM active */
                }
                break;

            case S2_STATE_C:
                return ccs2StateC2;  /* EV is connected to the station and charging (no ventilation required) */    
                break;
      
            default:
                break;
            }

        /* CP not in A/B/C state, check now EVS state to see if it's in fault or unavailable */
        switch (evsModbusState)
            {
            case FAULTED_STATE:
            case UNVAILABLE_STATE:
                return ccs2StateF;  /* Charging station is not available */

            default:
                return FALSE;
            }
        }
        break;
    
    default:
        return 0;
        break;
    }
}

/**
*
* @brief       
*
* @param [in]  None
*  
* @retval      None
*  
****************************************************************/

void ISO15118_homeplugdev_info_host_set(uint8_t info_sel, uint16_t val)
{
switch (info_sel)
    {
    case ISO15118_MAXCURRENT:
        info_xchange.host.maxCurrent = val;
        break;

    case ISO15118_STOPEVSE:
        info_xchange.host.stopEVSE = val;
        break;

    case ISI15118_PAUSEEVSE:
        info_xchange.host.pauseEVSE = val;
        break;

    default:
        break;
    }
}

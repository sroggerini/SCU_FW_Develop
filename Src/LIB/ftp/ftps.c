/******************* (C) COPYRIGHT 2009 STMicroelectronics *********/
/**
* @file        ftps.c
*
* @brief       FTP server service - Implementation -
*
* @author      Nick
*
* @riskClass   A
*
* @moduleID
*
* @vcsInfo
*     $Id: ftps.c 599 2024-09-26 07:03:24Z stefano $
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
#ifdef FTP_SERVER
/*
**********************************Aesys **************************************
**                                                                          **
**           INCLUDE                                                        **
**                                                                          **
******************************************************************************
*/
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdarg.h>
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#else
#include "stm32h5xx_hal.h"
#endif
#include "cmsis_os.h"
#include "telnet.h"
#include "lwip/tcp.h"
#include "fatfs.h"
#include "ftps.h"

/*
**********************************Aesys **************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/


/*
**********************************Aesys **************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 


/*
**********************************Aesys **************************************
**                                                                          **
**                            Local const                                   **
**                                                                          **
****************************************************************************** 
*/ 

/*
**********************************Aesys **************************************
**                                                                          **
**                            Global  const                                 **
**                                                                          **
****************************************************************************** 
*/ 

/*
**********************************Aesys **************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 
//  array of parameters for each ftp thread
struct server_stru ss[FTP_NBR_CLIENTS];

/* FTP  queue  declaration */
xQueueHandle ftpQueue = NULL;
#ifdef CLIENTS_2
xQueueHandle ftpQueue2 = NULL;
#endif
/* Definitions for FTP connection thread id  */
osThreadId_t ftpConnServerTaskId;
const osThreadAttr_t ftpConnServerId_attributes = {
  .name = "FTP_CON_SERVER",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 11
};


#ifdef CLIENTS_2
osThreadId ftpConn2ServerTaskId;
const osThreadAttr_t ftpConnServer2Id_attributes = {
  .name = "FTP_CON2_SERVER",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 10
};
#endif

/*
**********************************Aesys **************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/

/*
**********************************Aesys **************************************
**                                                                          **
**                            External Function                             **
**                                                                          **
******************************************************************************
*/
extern void FtpServer_service (struct server_stru* pMsgRx);

/*
**********************************Aesys **************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
static xQueueHandle getFtpQueueHandle(uint8_t num);
static void         endFtpConnection (uint8_t ftpNum);

/**
*
* @brief       FTP connection thread
*
* @param [in]  void const *: not used  
*  
* @retval      none 
*  
****************************************************************/
void ftp_conn (void* pvParameters)
{
  struct server_stru msgRx;

  /*-------- Creates an empty mailbox for ftp messages --------------------------*/
  ftpQueue = xQueueCreate(NUM_BUFF_FTP_RX, sizeof(struct server_stru));
  configASSERT(ftpQueue != NULL);
  uint8_t ix;
  
  for (ix = 0; ix < FTP_NBR_CLIENTS; ix++)
  {
    resetftpDataConn(ix);
  }
  
  for (;;)
  {
    /* Wait for some event from ftp client side  */
    if (xQueueReceive(ftpQueue, (void *)&msgRx, portMAX_DELAY) == pdPASS)
    {
      FtpServer_service(&msgRx);
      endFtpConnection(msgRx.num);
      DEBUG_PRINT( "EndConn %d\r\n", msgRx.num);
      resetftpDataConn(msgRx.num);
      if (checkFlagForFileFromFTP() == TRUE)
      {
        /* the copy of the file received from FTP is made only when FTP connection is closed */
        setFlagOnRtcBck(BACKUP_DATA_QSPI_REG, BACKUP_DATA_QSPI_AVAILABLE);
      }
    }
    else
    {
      continue;
    }
  }
}

#ifdef CLIENTS_2
/**
*
* @brief       FTP connection thread number 2
*
* @param [in]  void const *: not used  
*  
* @retval      none 
*  
****************************************************************/
void ftp_conn2 (void* pvParameters)
{
  struct server_stru msgRx;

  /*-------- Creates an empty mailbox for ftp messages --------------------------*/
  ftpQueue2 = xQueueCreate(NUM_BUFF_FTP_RX, sizeof(struct server_stru));
  configASSERT(ftpQueue2 != NULL);

  for (;;)
  {
    /* Wait for some event from ftp client side  */
    if (xQueueReceive(ftpQueue2, (void *)&msgRx, portMAX_DELAY) == pdPASS)
    {
      FtpServer_service(&msgRx);
      endFtpConnection(msgRx.num);
      DEBUG_PRINT( "EndConn2 %d\r\n", msgRx.num);
      if (checkFlagForFileFromFTP() == TRUE)
      {
        /* the copy of the file received from FTP is made only when FTP connection is closed */
        setFlagOnRtcBck(BACKUP_DATA_QSPI_REG, BACKUP_DATA_QSPI_AVAILABLE);
      }
    }
    else
    {
      continue;
    }
  }
}
#endif

/**
*
* @brief       FTP main server thread
*
* @param [in]  void const *: not used  
*  
* @retval      none 
*  
****************************************************************/
void ftp_server (void* pvParameters)
{
  struct netconn*     ftpsrvconn;
  uint8_t             i;


  //  Initialize ftp thread' parameters for each thread
  for( i = 0; i < FTP_NBR_CLIENTS; i ++ )
  {
    ss[i].num = i;
    ss[i].ftpconn = NULL;
    ss[i].signalId = (FTP_SIGNAL_CLIENT_REQ << i);
  }

  /*  creation and definition of ftp connection  Task */
  ftpConnServerTaskId = osThreadNew(ftp_conn, NULL, &ftpConnServerId_attributes); 

#ifdef CLIENTS_2
  /*  creation and definition of ftp connection  Task number 2 */
  ftpConn2ServerTaskId = osThreadNew(ftp_conn2, NULL, &ftpConnServer2Id_attributes); 
#endif

  // Create the TCP connection handle
  ftpsrvconn = netconn_new( NETCONN_TCP );
  LWIP_ERROR( "http_server: invalid ftpsrvconn", (ftpsrvconn != NULL), return; );

  // Bind to port 21 (FTP) with default IP address
  //    and put the connection into LISTEN state
  netconn_bind( ftpsrvconn, NULL, FTP_SERVER_PORT );

  netconn_listen( ftpsrvconn );

  /* now we start with ntp client task also */
  //ntpStartNtpTask();

  while( TRUE )
  {
    // check every  200msec 
    osDelay(200);
    // Look for the first connection not used
    for( i = 0; i < FTP_NBR_CLIENTS; i++ )
    {
      if( ss[i].ftpconn == NULL )
        break;
    }
    if( i == FTP_NBR_CLIENTS )
    {
      // All connections occupied wait 300msec 
      osDelay(300);
    }
    else
    {
      if( netconn_accept( ftpsrvconn, &ss[i].ftpconn ) == ERR_OK )
      {
        DEBUG_PRINT( "Accept %d\r\n", i);

        // New client request, wake up the corresponding thread
        /* the message is correct: it can be sent to application layer */
        /*         destionation               source  */
        //memcpy((void*)&msgRx, (void*)&ss[i], sizeof(server_stru));
        configASSERT(xQueueSendToBack(getFtpQueueHandle(ss[i].num), (void *)&ss[i], portMAX_DELAY) == pdPASS);
      }
    }
  }
}

/**
*
* @brief        End ftp connection
*
* @param [in]   uint8_t: index connection
*
* @retval       none
*
***********************************************************************************************************************/
static void endFtpConnection (uint8_t ftpNum)
{
  dataClose(ftpNum);
  netconn_delete(ss[ftpNum].ftpconn);
  ss[ftpNum].ftpconn = NULL;
}

/**
*
* @brief        Get the pointer to master/slave Rx queue  
*
* @param [in]   none
*
* @retval       xQueueHandle: pointer to defined Rx MS queue
*
***********************************************************************************************************************/
static xQueueHandle getFtpQueueHandle(uint8_t num)
{
  xQueueHandle xQHandle;
  
  switch ((num))
  {
    case 0:
      xQHandle = ftpQueue;
      break;
#ifdef CLIENTS_2
    case 1:
      xQHandle = ftpQueue2;
      break;
#endif
    default:
      xQHandle = NULL;
      break;
  }
  return(xQHandle);
}
#endif
/*************** END OF FILE ******************************************************************************************/


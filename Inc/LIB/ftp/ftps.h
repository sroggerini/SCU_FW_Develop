/*
    FTP Server for STM32-E407 and ChibiOS
    Copyright (C) 2015 Jean-Michel Gallego

    See readme.txt for information

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _FTPS_H_
#define _FTPS_H_

#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/ip_addr.h"

#include "ff.h"

#include "telnet.h"
// to print messages for debugging
//#define DEBUG_PRINT(...)
#define DEBUG_PRINT(...) tPrintf( __VA_ARGS__ )
// to print commands received from clients and response of ftp server
#define COMMAND_PRINT(...)
// #define COMMAND_PRINT(...) tPrintf( __VA_ARGS__ )

#define FTP_VERSION              "FTP-2015-07-31"

#define FTP_USER                 "ftp2020"
#define FTP_PASS                 "2020"

#define FTP_SERVER_PORT          21
#define FTP_DATA_PORT            55600         // Data port in passive mode
#define FTP_TIME_OUT             10            // Disconnect client after 5 minutes of inactivity
#define FTP_PARAM_SIZE          (_MAX_LFN + 8 + 1)
#define FTP_CWD_SIZE            (_MAX_LFN + 8 + 1)  // max size of a directory name

// number of clients we want to serve simultaneously: in PANAMA application 1 is enough (typically 5)
#ifdef CLIENTS_2
#define FTP_NBR_CLIENTS          2
#else
#define FTP_NBR_CLIENTS          1
#endif

// signal position for client request connection
#define FTP_SIGNAL_CLIENT_REQ    ((int32_t)0x00000001)

#define NUM_BUFF_FTP_RX          ((uint8_t)6)

// size of file buffer for reading a file
#define FTP_BUF_SIZE             512
// System tick frequency: Frequency of the system timer that drives the system ticks
#define CH_CFG_ST_FREQUENCY                 1000


// define a structure of parameters for a ftp thread
struct server_stru
{
  uint8_t         num;
  // uint8_t fase;   // for debugging only
  struct netconn* ftpconn;
  //Nick binary_semaphore_t semrequest;
  uint32_t       signalId;
};

// Define a structure for passing messages
struct sdlog_stru
{
  char*   file;
  char*   line;
  uint8_t append;
};

enum dcm_type     // Data Connection mode:
{
  NOTSET  = 0,    //   not set
  PASSIVE = 1,    //   passive
  ACTIVE  = 2,    //   active
};

/**
  * @brief  FTP  structure definition
  */

typedef  struct
{
  struct    netconn * listdataconn, * dataconn, * ctrlconn;
  struct    netbuf  * inbuf;
            ip_addr_t ipclient;
            ip_addr_t ipserver;

  FIL                 file;
  FILINFO             finfo;

  char                parameters[FTP_PARAM_SIZE];   // parameters sent by client
  char                cwdName[FTP_CWD_SIZE];        // name of current directory
  char                cwdRNFR[FTP_CWD_SIZE];        // name of origin directory for Rename command
  char                path[FTP_CWD_SIZE];
  char                str[32];
  uint32_t            timeBeginTrans;
  uint32_t            bytesTransfered;
  char                buf[FTP_BUF_SIZE];           // data buffer for communication
  uint16_t            pbuf;
  uint16_t            dataPort;
  char                lfn[_MAX_LFN + 1];          // Buffer to store the LFN
  char                command[6];                   // command sent by client
  int8_t              cmdStatus;                    // status of ftp command connection
  enum dcm_type       dataConnMode;
  int8_t              nerr;
  uint8_t             num;
}ftpDataConn_st;


/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
void ftp_server         (void* pvParameters);
void resetftpDataConn   (uint8_t ix);
void dataClose          (uint8_t numConn);
char *getFtpPassword    (void);
char *getFtpUsername    (void);

#endif // _FTPS_H_

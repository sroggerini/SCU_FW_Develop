/******************* (C) COPYRIGHT 2020 SCAME  ******************************************************************/
/**
* @file        ftpserver.c
*
* @brief       FTP server service - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: ftpserver.c 599 2024-09-26 07:03:24Z stefano $
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

#include <string.h>
#include <ctype.h>
#include <stdlib.h>

#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#else
#include "stm32h5xx_hal.h"
#endif
#include "cmsis_os.h"
#include "ff.h"
#include "ftps.h"

#include "flashFat.h"
#include "fatfs.h"


/*
**********************************Aesys **************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#define   BUFFER_FILE_RAM_SIZE    ((uint32_t)(0x4000))
#define   START_ADDR_PARK_AREA    (bufferForFile)
#define   NUM_SECTOR_IN_RAM       ((uint32_t)(BUFFER_FILE_RAM_SIZE / BLOCK_SIZE))
 
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
static const char *ftpDefaultAccount[] = {FTP_USER, FTP_PASS};

/*
**********************************Aesys **************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 
//#pragma arm section rwdata   = "RW_SDRAM_DML"   
//static ftpDataConn_st ftpDataConn[FTP_NBR_CLIENTS] __attribute__((section ("RW_SDRAM_DML")));
ftpDataConn_st ftpDataConn[FTP_NBR_CLIENTS];

uint8_t   bufferForFile[BUFFER_FILE_RAM_SIZE];


/*
**********************************Aesys **************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/
extern struct server_stru ss[ FTP_NBR_CLIENTS ];
extern char   USERPath[4];                        /* USER logical drive path */


/*
**********************************Aesys **************************************
**                                                                          **
**                            External Function                             **
**                                                                          **
******************************************************************************
*/

/*
**********************************Aesys **************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
static char * int2strZ( char * s, uint32_t i, int8_t z );
static char * int2str( char * s, int32_t i, int8_t ls, uint8_t numConn);

// =========================================================
//
//              Send a response to the client
//
// =========================================================

void sendBegin(const char * s, uint8_t numConn)
{
  strncpy(ftpDataConn[numConn].buf, s, FTP_BUF_SIZE );
}

void sendCat(const char * s, uint8_t numConn)
{
  size_t len = FTP_BUF_SIZE - strlen(ftpDataConn[numConn].buf );
  strncat(ftpDataConn[numConn].buf, s, len );
}

void sendWrite( const char * s, uint8_t numConn)
{
  if (s != NULL)
  {
    sendBegin(s, numConn);
  }
  if( strlen(ftpDataConn[numConn].buf ) + 2 < FTP_BUF_SIZE )
    strcat(ftpDataConn[numConn].buf, "\r\n" );
  netconn_write(ftpDataConn[numConn].ctrlconn, ftpDataConn[numConn].buf, strlen(ftpDataConn[numConn].buf ), NETCONN_COPY );
  COMMAND_PRINT( ">%u> %s", ftpDataConn[numConn].num, ftpDataConn[numConn].buf );
}

void sendCatWrite( const char * s, uint8_t numConn)
{
  sendCat(s, numConn);
  sendWrite(NULL, numConn);
}

//  Convert an integer to string
//
//  Return pointer to string

char * i2str( int32_t i, uint8_t numConn)
{
  return int2str(ftpDataConn[numConn].str, i, 12, numConn);
}

// Create string YYYYMMDDHHMMSS from date and time
//
// parameters:
//    date, time
//
// return:
//    pointer to string

char * makeDateTimeStr( uint16_t date, uint16_t time, uint8_t numConn)
{
  int2strZ(ftpDataConn[numConn].str, (( date & 0xFE00 ) >> 9 ) + 1980, -5 );
  int2strZ(ftpDataConn[numConn].str + 4, ( date & 0x01E0 ) >> 5, -3 );
  int2strZ(ftpDataConn[numConn].str + 6, date & 0x001F, -3 );
  int2strZ(ftpDataConn[numConn].str + 8, ( time & 0xF800 ) >> 11, -3 );
  int2strZ(ftpDataConn[numConn].str + 10, ( time & 0x07E0 ) >> 5, -3 );
  int2strZ(ftpDataConn[numConn].str + 12, ( time & 0x001F ) << 1, -3 );
  return ftpDataConn[numConn].str;
}

// Calculate date and time from first parameter sent by MDTM command (YYYYMMDDHHMMSS)
//
// parameters:
//   pdate, ptime: pointer of variables where to store data
//
// return:
//    length of (time parameter + space) if date/time are ok
//    0 if parameter is not YYYYMMDDHHMMSS

int8_t getDateTime( uint16_t * pdate, uint16_t * ptime, uint8_t numConn)
{
  // Date/time are expressed as a 14 digits long string
  //   terminated by a space and followed by name of file
  if( strlen( ftpDataConn[numConn].parameters ) < 15 || ftpDataConn[numConn].parameters[ 14 ] != ' ' )
    return 0;
  for( uint8_t i = 0; i < 14; i++ )
    if( ! isdigit( ftpDataConn[numConn].parameters[ i ]))
      return 0;

  ftpDataConn[numConn].parameters[ 14 ] = 0;
  * ptime = atoi( ftpDataConn[numConn].parameters + 12 ) >> 1;   // seconds
  ftpDataConn[numConn].parameters[ 12 ] = 0;
  * ptime |= atoi( ftpDataConn[numConn].parameters + 10 ) << 5;  // minutes
  ftpDataConn[numConn].parameters[ 10 ] = 0;
  * ptime |= atoi( ftpDataConn[numConn].parameters + 8 ) << 11;  // hours
  ftpDataConn[numConn].parameters[ 8 ] = 0;
  * pdate = atoi( ftpDataConn[numConn].parameters + 6 );         // days
  ftpDataConn[numConn].parameters[ 6 ] = 0;
  * pdate |= atoi( ftpDataConn[numConn].parameters + 4 ) << 5;   // months
  ftpDataConn[numConn].parameters[ 4 ] = 0;
  * pdate |= ( atoi(ftpDataConn[numConn].parameters ) - 1980 ) << 9;       // years

  return 15;
}

// =========================================================
//
//             Get a command from the client
//
// =========================================================

// update variables command and parameters
//
// return: -4 time out
//         -3 error receiving data
//         -2 command line too long
//         -1 syntax error
//          0 command without parameters
//          >0 length of parameters

int8_t readCommand(uint8_t numConn)
{
  char   * pbuf;
  uint16_t buflen;
  int8_t   rc = 0;
  int8_t   i;
  char     car;

  ftpDataConn[numConn].command[0] = 0;
  ftpDataConn[numConn].parameters[0] = 0;
  ftpDataConn[numConn].nerr = netconn_recv(ftpDataConn[numConn].ctrlconn, &ftpDataConn[numConn].inbuf );
  if( ftpDataConn[numConn].nerr == ERR_TIMEOUT )
    return -4;
  if( ftpDataConn[numConn].nerr != ERR_OK )
    return -3;
  netbuf_data(ftpDataConn[numConn].inbuf, (void **) &pbuf, & buflen );
  if( buflen == 0 )
    goto deletebuf;
  i = 0;
  car = pbuf[0];
  do
  {
    if( ! isalpha(car))
      break;
    ftpDataConn[numConn].command[i++] = car;
    car = pbuf[i];
  }
  while( i < buflen && i < 4 );
  ftpDataConn[numConn].command[i] = 0;
  if( car != ' ' )
    goto deletebuf;
  do
    if( i > buflen + 2 )
      goto deletebuf;
  while( pbuf[i++] == ' ' );
  rc = i;
  do
    car = pbuf[ rc ++ ];
  while( car != '\n' && car != '\r' && rc < buflen );
  if( rc == buflen )
  {
    rc = -1;
    goto deletebuf;
  }
  if( rc - i - 1 >= FTP_PARAM_SIZE )
  {
    rc = -2;
    goto deletebuf;
  }
  strncpy( ftpDataConn[numConn].parameters, pbuf + i - 1, rc - i );
  ftpDataConn[numConn].parameters[rc - i] = 0;
  rc = rc - i;

  deletebuf:
  COMMAND_PRINT( "<%u< %s %s\r\n", ftpDataConn[numConn].num, ftpDataConn[numConn].command, ftpDataConn[numConn].parameters );
  netbuf_delete(ftpDataConn[numConn].inbuf);
  return rc;
}

// =========================================================
//
//               Functions for data connection
//
// =========================================================

uint8_t listenDataConn(uint8_t numConn)
{
  uint8_t ok = (uint8_t)TRUE;

  // If this is not already done, create the TCP connection handle
  //   to listen to client to open data connection
  if(ftpDataConn[numConn].listdataconn == NULL )
  {
    ftpDataConn[numConn].listdataconn = netconn_new( NETCONN_TCP );
    ok = (uint8_t)(ftpDataConn[numConn].listdataconn != NULL);
    if (ok)
    {
      // Bind listdataconn to port (FTP_DATA_PORT+num) with default IP address
      ftpDataConn[numConn].nerr = netconn_bind( ftpDataConn[numConn].listdataconn, IP_ADDR_ANY, ftpDataConn[numConn].dataPort );
      ok = (uint8_t)(ftpDataConn[numConn].nerr == ERR_OK);
    }
    if (ok)
    {
      // Put the connection into LISTEN state
      ftpDataConn[numConn].nerr = netconn_listen(ftpDataConn[numConn].listdataconn);
      ok = (uint8_t)(ftpDataConn[numConn].nerr == ERR_OK);
    }
  }
  if( ! ok )
    DEBUG_PRINT( "Error in listenDataConn()\r\n" );
  return ok;
}

uint8_t dataConnect(uint8_t numConn)
{
  ftpDataConn[numConn].nerr = ERR_CONN;

  if(ftpDataConn[numConn].dataConnMode == NOTSET )
  {
    DEBUG_PRINT( "No connecting mode defined\r\n" );
    goto error;
  }
  DEBUG_PRINT( "Connecting in %s mode\r\n",
               (ftpDataConn[numConn].dataConnMode == PASSIVE ? "passive" : "active" ));

  if (ftpDataConn[numConn].dataConnMode == PASSIVE )
  {
    if (ftpDataConn[numConn].listdataconn == NULL)
      goto error;
    // Wait for connection from client during 500 ms
    netconn_set_recvtimeout(ftpDataConn[numConn].listdataconn, (int32_t)(500));
    ftpDataConn[numConn].nerr = netconn_accept(ftpDataConn[numConn].listdataconn, &ftpDataConn[numConn].dataconn);
    if (ftpDataConn[numConn].nerr != ERR_OK )
    {
      DEBUG_PRINT( "Error in dataConnect(): netconn_accept\r\n" );
      goto error;
    }
  }
  else
  {
    //  Create a new TCP connection handle
    ftpDataConn[numConn].dataconn = netconn_new( NETCONN_TCP );
    if(ftpDataConn[numConn].dataconn == NULL)
    {
      DEBUG_PRINT( "Error in dataConnect(): netconn_new\r\n" );
      // goto delconn;
      goto error;
    }
    ftpDataConn[numConn].nerr = netconn_bind(ftpDataConn[numConn].dataconn, IP_ADDR_ANY, 0 ); //dataPort );
    //  Connect to data port with client IP address
    if(ftpDataConn[numConn].nerr != ERR_OK )
    {
      DEBUG_PRINT( "Error %u in dataConnect(): netconn_bind\r\n", abs(ftpDataConn[numConn].nerr ));
      // goto error;   // pas sûr !!!
      goto delconn;
    }
    ftpDataConn[numConn].nerr = netconn_connect(ftpDataConn[numConn].dataconn, &ftpDataConn[numConn].ipclient, ftpDataConn[numConn].dataPort );
    if(ftpDataConn[numConn].nerr != ERR_OK )
    {
      DEBUG_PRINT( "Error %u in dataConnect(): netconn_connect\r\n", abs(ftpDataConn[numConn].nerr ));
      // goto error;   // sûr !!!
      goto delconn;
    }
  }
  return (uint8_t)TRUE;

  delconn:
  if(ftpDataConn[numConn].dataconn != NULL )
  {
    netconn_delete(ftpDataConn[numConn].dataconn );
    ftpDataConn[numConn].dataconn = NULL;
  }

  error:
  sendWrite( "425 No data connection", numConn);
  return ((uint8_t)FALSE);
}

void dataClose(uint8_t numConn)
{
  ftpDataConn[numConn].dataConnMode = NOTSET;
  if(ftpDataConn[numConn].dataconn == NULL )
    return;
  netconn_close(ftpDataConn[numConn].dataconn );
  netconn_delete(ftpDataConn[numConn].dataconn );
  ftpDataConn[numConn].dataconn = NULL;
}

void dataWrite(const char * data, uint8_t numConn)
{
  netconn_write(ftpDataConn[numConn].dataconn, data, strlen( data ), NETCONN_COPY );
  // COMMAND_PRINT( data );
}

// =========================================================
//
//                  Functions on files
//
// =========================================================

// Make complete path/name from cwdName and parameters
//
// 3 possible cases:
//   parameters can be absolute path, relative path or only the name
//
// parameters:
//   fullName : where to store the path/name
//
// return:
//   TRUE, if done

uint8_t makePathFrom( char * fullName, char * param, uint8_t numConn)
{
  // Root or empty?
  if( ! strcmp( param, "/" ) || strlen( param ) == 0 )
  {
    strcpy( fullName, "/" );
    return (uint8_t)TRUE;
  }
  // If relative path, concatenate with current dir
  if( param[0] != '/' )
  {
    strcpy( fullName, ftpDataConn[numConn].cwdName );
    if( fullName[ strlen( fullName ) - 1 ] != '/' )
      strncat( fullName, "/", FTP_CWD_SIZE );
    strncat( fullName, param, FTP_CWD_SIZE );
  }
  else
    strcpy( fullName, param );
  // If ends with '/', remove it
  uint16_t strl = strlen( fullName ) - 1;
  if( fullName[ strl ] == '/' && strl > 1 )
    fullName[ strl ] = 0;
  if( strlen( fullName ) < FTP_CWD_SIZE )
    return TRUE;
  sendWrite( "500 Command line too long", numConn);
  return (uint8_t)FALSE;
}

uint8_t makePath(char * fullName, uint8_t numConn)
{
  return (uint8_t)makePathFrom( fullName, ftpDataConn[numConn].parameters, numConn);
}

void closeTransfer(uint8_t numConn)
{
  uint32_t deltaT = (uint32_t) ( HAL_GetTick() - ftpDataConn[numConn].timeBeginTrans );
  if( deltaT > 0 && ftpDataConn[numConn].bytesTransfered > 0 )
  {
    sendBegin( "226-File successfully transferred\r\n", numConn);
    sendCat( "226 ", numConn);
    sendCat( i2str( deltaT, numConn), numConn);
    sendCat( " ms, ", numConn);
    uint32_t bps;
    if( ftpDataConn[numConn].bytesTransfered < 0x7fffffff / CH_CFG_ST_FREQUENCY )
      bps = ( ftpDataConn[numConn].bytesTransfered * CH_CFG_ST_FREQUENCY ) / deltaT;
    else
      bps = ( ftpDataConn[numConn].bytesTransfered / deltaT ) * CH_CFG_ST_FREQUENCY;
    if( bps > 10000 )
    {
      sendCat(i2str(bps / 1000, numConn), numConn);
      sendCatWrite( " kbytes/s", numConn );
    }
    else
    {
      sendCat(i2str( bps, numConn), numConn);
      sendCatWrite( " bytes/s", numConn);
    }
  }
  else
    sendWrite ("226 File successfully transferred", numConn );
}

// Return TRUE if a file or directory exists
//
// parameters:
//   path : absolute name of file or directory

uint8_t fs_exists(char * path, uint8_t numConn)
{
  if( ! strcmp( path, "/" ) )
    return (uint8_t)TRUE;

  char *  path0 = path;

  return (uint8_t)f_stat( path0, &ftpDataConn[numConn].finfo ) == FR_OK;
}

// Open a directory
//
// parameters:
//   path : absolute name of directory
//
// return TRUE if opened

uint8_t fs_opendir( DIR * pdir, char * dirName )
{
  char * dirName0 = dirName;
  uint8_t ffs_result;

  ffs_result = f_opendir( pdir, dirName0 );
  return (uint8_t)(ffs_result == FR_OK);
}

// =========================================================
//
//                   Process a command
//
// =========================================================

uint8_t processCommand( char * command, char * parameters, uint8_t numConn)
{
  char*     pPath;
  
  pPath = getFatDevicePath (FAT_DEVICE_FLASH);
  
  if (f_chdrive(pPath) != FR_OK)
  {
    sendWrite( "500 Unknow command", numConn);
    return (uint8_t)TRUE;
  }
  /* mount file system if need */
  //(void) flashSDInit();

  ///////////////////////////////////////
  //                                   //
  //      ACCESS CONTROL COMMANDS      //
  //                                   //
  ///////////////////////////////////////

  //
  //  QUIT
  //
  if( ! strcmp( command, "QUIT" ))
    return FALSE;
  //
  //  PWD - Print Directory
  //
  else if( ! strcmp( command, "PWD" ) ||
           ( ! strcmp( command, "CWD" ) && ! strcmp( parameters, "." )))  // 'CWD .' is the same as PWD command
  {
    sendBegin( "257 \"", numConn);
    sendCat( ftpDataConn[numConn].cwdName, numConn);
    sendCatWrite( "\" is your current directory", numConn);
  }
  //
  //  CWD - Change Working Directory
  //
  else if( ! strcmp( command, "CWD" ))
  {
    if( strlen( parameters ) == 0 )
      sendWrite( "501 No directory name", numConn);
    else if( makePath(ftpDataConn[numConn].path, numConn))
      if( fs_exists(ftpDataConn[numConn].path, numConn))
      {
        strcpy(ftpDataConn[numConn].cwdName, ftpDataConn[numConn].path );
        sendWrite( "250 Directory successfully changed.", numConn);
      }
      else
        sendWrite( "550 Failed to change directory.", numConn);
  }
  //
  //  CDUP - Change to Parent Directory, 
  //
  else if( ! strcmp( command, "CDUP" ))
  {
    uint8_t ok = (uint8_t)FALSE;

    if( strlen(ftpDataConn[numConn].cwdName ) > 1 )  // do nothing if cwdName is root
    {
      // if cwdName ends with '/', remove it (must not append)
      if(ftpDataConn[numConn].cwdName[strlen(ftpDataConn[numConn].cwdName ) - 1] == '/' )
        ftpDataConn[numConn].cwdName[strlen(ftpDataConn[numConn].cwdName ) - 1 ] = 0;
      // search last '/'
      char * pSep = strrchr(ftpDataConn[numConn].cwdName, '/' );
      ok = (uint8_t)(pSep > ftpDataConn[numConn].cwdName);
      // if found, ends the string on its position
      if( ok )
      {
        * pSep = 0;
        ok = (uint8_t)fs_exists(ftpDataConn[numConn].cwdName, numConn );
      }
    }
    // if an error appends, move to root
    if( ! ok )
      strcpy(ftpDataConn[numConn].cwdName, "/");
    sendBegin( "200 Ok. Current directory is ", numConn);
    sendCatWrite(ftpDataConn[numConn].cwdName, numConn);
  }

  ///////////////////////////////////////
  //                                   //
  //    TRANSFER PARAMETER COMMANDS    //
  //                                   //
  ///////////////////////////////////////

  //
  //  MODE - Transfer Mode
  //
  else if( ! strcmp( command, "MODE" ))
  {
    if( ! strcmp( parameters, "S" ))
      sendWrite( "200 S Ok", numConn);
    // else if( ! strcmp( parameters, "B" ))
    //  sendWrite( "200 B Ok" );
    else
      sendWrite( "504 Only S(tream) is suported", numConn);
  }
  //
  //  STRU - File Structure
  //
  else if( ! strcmp( command, "STRU" ))
  {
    if( ! strcmp( parameters, "F" ))
      sendWrite( "200 F Ok", numConn);
    else
      sendWrite( "504 Only F(ile) is suported", numConn);
  }
  //
  //  TYPE - Data Type
  //
  else if( ! strcmp( command, "TYPE" ))
  {
    if( ! strcmp( parameters, "A" ))
      sendWrite( "200 TYPE is now ASCII", numConn);
    else if( ! strcmp( parameters, "I" ))
      sendWrite( "200 TYPE is now 8-bit binary", numConn);
    else
      sendWrite( "504 Unknow TYPE", numConn);
  }
  //
  //  PASV - Passive Connection management
  //
  else if( ! strcmp( command, "PASV" ))
  {
    if( listenDataConn(numConn))
    {
      dataClose(numConn);
      sendBegin( "227 Entering Passive Mode (", numConn );
      sendCat( i2str(ip4_addr1( &ftpDataConn[numConn].ipserver ), numConn), numConn); sendCat( ",", numConn);
      sendCat( i2str(ip4_addr2( &ftpDataConn[numConn].ipserver ), numConn), numConn); sendCat( ",", numConn);
      sendCat( i2str(ip4_addr3( &ftpDataConn[numConn].ipserver ), numConn), numConn); sendCat( ",", numConn);
      sendCat( i2str(ip4_addr4( &ftpDataConn[numConn].ipserver ), numConn), numConn); sendCat( ",", numConn);
      sendCat( i2str(ftpDataConn[numConn].dataPort >> 8, numConn), numConn); sendCat( ",", numConn);
      sendCat( i2str(ftpDataConn[numConn].dataPort & 255, numConn), numConn); sendCatWrite( ").", numConn);
      DEBUG_PRINT( "Data port set to %d\r\n", ftpDataConn[numConn].dataPort );
      ftpDataConn[numConn].dataConnMode = PASSIVE;
    }
    else
    {
      sendWrite( "425 Can't set connection management to passive", numConn);
      ftpDataConn[numConn].dataConnMode = NOTSET;
    }
  }
  //
  //  PORT - Data Port
  //
  else if( ! strcmp( command, "PORT" ))
  {
    uint8_t ip[4];
    uint8_t i;
    dataClose(numConn);
    // get IP of data client
    char * p = NULL;
    if( strlen( parameters ) > 0 )
    {
      p = parameters - 1;
      for( i = 0; i < 4 && p != NULL; i ++ )
      {
        ip[ i ] = atoi( ++ p );
        p = strchr( p, ',' );
      }
      // get port of data client
      if( i == 4 && p != NULL )
      {
        ftpDataConn[numConn].dataPort = 256 * atoi( ++ p );
        p = strchr( p, ',' );
        if( p != NULL )
          ftpDataConn[numConn].dataPort += atoi( ++ p );
      }
    }
    if( p == NULL )
    {
      sendWrite( "501 Can't interpret parameters", numConn);
      ftpDataConn[numConn].dataConnMode = NOTSET;
    }
    else
    {
      IP4_ADDR( &ftpDataConn[numConn].ipclient, ip[0], ip[1], ip[2], ip[3] );
      sendWrite( "200 PORT command successful", numConn);
      DEBUG_PRINT( "Data IP set to %u:%u:%u:%u\r\n", ip[0], ip[1], ip[2], ip[3] );
      DEBUG_PRINT( "Data port set to %d\r\n", ftpDataConn[numConn].dataPort);
      ftpDataConn[numConn].dataConnMode = ACTIVE;
    }
  }

  ///////////////////////////////////////
  //                                   //
  //        FTP SERVICE COMMANDS       //
  //                                   //
  ///////////////////////////////////////

  //
  //  LIST and NLST - List
  //
  else if( ! strcmp( command, "LIST" ) || ! strcmp( command, "NLST" ))
  {
    uint16_t nm = 0;
    DIR dir;

    if( ! fs_opendir( &dir, ftpDataConn[numConn].cwdName ))
    {
      sendBegin( "550 Can't open directory ", numConn);
      sendCatWrite(ftpDataConn[numConn].cwdName, numConn);
    }
    else if( dataConnect(numConn))
    {
      sendWrite( "150 Accepted data connection", numConn);
      for( ; ; )
      {
        if( f_readdir( &dir, &ftpDataConn[numConn].finfo ) != FR_OK ||
            ftpDataConn[numConn].finfo.fname[0] == 0 )
          break;
        if(ftpDataConn[numConn].finfo.fname[0] == '.')
          continue;
        if( ! strcmp( command, "LIST" ))
        {
          if(ftpDataConn[numConn].finfo.fattrib & AM_DIR)
            strcpy(ftpDataConn[numConn].buf, "+/" );
          else
          {
            strcpy(ftpDataConn[numConn].buf, "+r,s");
            strcat(ftpDataConn[numConn].buf, i2str(ftpDataConn[numConn].finfo.fsize, numConn));
          }
          strcat(ftpDataConn[numConn].buf, ",\t" );
          strcat(ftpDataConn[numConn].buf, ftpDataConn[numConn].lfn[0] == 0 ? ftpDataConn[numConn].finfo.fname : ftpDataConn[numConn].lfn );
        }
        else
          strcpy(ftpDataConn[numConn].buf, ftpDataConn[numConn].lfn[0] == 0 ? ftpDataConn[numConn].finfo.fname : ftpDataConn[numConn].lfn );
        strcat(ftpDataConn[numConn].buf, "\r\n");
        dataWrite(ftpDataConn[numConn].buf, numConn );
        nm ++;
      }
      sendWrite( "226 Directory send OK.", numConn);
      dataClose(numConn);
    }
    f_closedir (&dir);	/* Close an open directory */
  }
  //
  //  MLSD - Listing for Machine Processing (see RFC 3659)
  //
  else if( ! strcmp( command, "MLSD" ))
  {
    DIR dir;
    uint16_t nm = 0;

    if( ! fs_opendir( &dir, ftpDataConn[numConn].cwdName ))
    {
      sendBegin( "550 Can't open directory ", numConn);
      sendCatWrite(parameters, numConn);
    }
    else if( dataConnect(numConn))
    {
      sendWrite( "150 Accepted data connection", numConn);
      for( ; ; )
      {
        if( f_readdir( &dir, &ftpDataConn[numConn].finfo ) != FR_OK ||
            ftpDataConn[numConn].finfo.fname[0] == 0 )
          break;
        if(ftpDataConn[numConn].finfo.fname[0] == '.' )
          continue;
        strcpy(ftpDataConn[numConn].buf, "Type=" );
        strcat(ftpDataConn[numConn].buf, ftpDataConn[numConn].finfo.fattrib & AM_DIR ? "dir" : "file" );
        strcat(ftpDataConn[numConn].buf, ";Size=" );
        strcat(ftpDataConn[numConn].buf, i2str(ftpDataConn[numConn].finfo.fsize, numConn));
        if (ftpDataConn[numConn].finfo.fdate != 0)
        {
          strcat(ftpDataConn[numConn].buf, ";Modify=" );
          strcat(ftpDataConn[numConn].buf, makeDateTimeStr(ftpDataConn[numConn].finfo.fdate, ftpDataConn[numConn].finfo.ftime, numConn));
        }
        strcat(ftpDataConn[numConn].buf, "; " );
        strcat(ftpDataConn[numConn].buf, ftpDataConn[numConn].lfn[0] == 0 ? ftpDataConn[numConn].finfo.fname : ftpDataConn[numConn].lfn );
        strcat(ftpDataConn[numConn].buf, "\r\n" );
        dataWrite(ftpDataConn[numConn].buf, numConn);
        nm ++;
      }
      sendBegin( "226-options: -a -l\r\n", numConn);
      sendCat( "226 ", numConn);
      sendCat( i2str(nm, numConn), numConn);
      sendCatWrite( " matches total", numConn);
      dataClose(numConn);
    }
    f_closedir (&dir);	/* Close an open directory */
  }
  //
  //  DELE - Delete a File
  //
  else if( ! strcmp( command, "DELE" ))
  {
    if( strlen( parameters ) == 0 )
      sendWrite( "501 No file name", numConn);
    else if( makePath(ftpDataConn[numConn].path, numConn))
    {
      if( ! fs_exists(ftpDataConn[numConn].path, numConn))
      {
        sendBegin( "550 File ", numConn);
        sendCat( parameters, numConn);
        sendCatWrite( " not found", numConn);
      }
      else
      {
        uint8_t ffs_result = f_unlink(ftpDataConn[numConn].path );
        if( ffs_result == FR_OK )
        {
          sendBegin( "250 Deleted ", numConn);
          sendCatWrite( parameters, numConn);
        }
        else
        {
          sendBegin( "450 Can't delete ", numConn);
          sendCatWrite( parameters, numConn);
        }
      }
    }
  }
  //
  //  NOOP
  //
  else if( ! strcmp( command, "NOOP" ))
  {
    sendWrite( "200 Zzz...", numConn);
  }
  //
  //  RETR - Retrieve
  //
  else if( ! strcmp( command, "RETR" ))
  {
    if( strlen( parameters ) == 0 )
      sendWrite( "501 No file name", numConn);
    else if( makePath(ftpDataConn[numConn].path, numConn))
    {
      if( ! fs_exists(ftpDataConn[numConn].path, numConn))
      {
        sendBegin( "550 File ", numConn);
        sendCat( parameters, numConn);
        sendCatWrite( " not found", numConn);
      }
      else if( f_open( &ftpDataConn[numConn].file, ftpDataConn[numConn].path, FA_READ ) != FR_OK )
      {
        sendBegin( "450 Can't open ", numConn);
        sendCatWrite( parameters, numConn);
      }
      else if( dataConnect(numConn))
      {
        uint16_t nb;

        DEBUG_PRINT( "Sending %s\r\n", parameters );
        sendBegin( "150-Connected to port ", numConn);
        sendCat( i2str(ftpDataConn[numConn].dataPort, numConn), numConn);
        sendCat( "\r\n150 ", numConn);
        sendCat( i2str( f_size(&ftpDataConn[numConn].file), numConn), numConn);
        sendCatWrite( " bytes to download", numConn);
        ftpDataConn[numConn].timeBeginTrans = HAL_GetTick();
        ftpDataConn[numConn].bytesTransfered = 0;

        DEBUG_PRINT( "Start transfert\r\n" );
        while( f_read( &ftpDataConn[numConn].file, ftpDataConn[numConn].buf, FTP_BUF_SIZE, (UINT *) & nb ) == FR_OK && nb > 0 )
        {
          netconn_write(ftpDataConn[numConn].dataconn, ftpDataConn[numConn].buf, nb, NETCONN_COPY);
          ftpDataConn[numConn].bytesTransfered += nb;
          DEBUG_PRINT( "Sent %u bytes\r", ftpDataConn[numConn].bytesTransfered );
        }
        DEBUG_PRINT( "\n" );
        f_close(&ftpDataConn[numConn].file );
        closeTransfer(numConn);
        dataClose(numConn);
      }
    }
  }
  //
  //  STOR - Store
  //
  else if( ! strcmp( command, "STOR" ))
  {
    if( strlen( parameters ) == 0 )
      sendWrite( "501 No file name", numConn);
    else if( makePath(ftpDataConn[numConn].path, numConn))
    {
      FIL*          fp;
      osSemaphoreId semaphoreFat;
      FRESULT       res;

      semaphoreFat = getFatSemaphore();
      while (osSemaphoreAcquire(semaphoreFat, portMAX_DELAY) != osOK)
      {
        ;
      }
      fp = &ftpDataConn[numConn].file;

      res = f_open(fp, ftpDataConn[numConn].path, FA_CREATE_ALWAYS | FA_WRITE );
      //res = f_open(fp, (TCHAR*)"fwSCU.bin", FA_CREATE_ALWAYS | FA_WRITE );
      if(res != FR_OK )
      {
        sendBegin( "451 Can't open/create ", numConn);
        sendCatWrite( parameters, numConn);
        osSemaphoreRelease(semaphoreFat); 
      } 
      else if( ! dataConnect(numConn))
      {
        f_close( &ftpDataConn[numConn].file);
        osSemaphoreRelease(semaphoreFat); 
      }
      else
      {
        struct pbuf*  rcvbuf = NULL;
        void*         prcvbuf;
        uint16_t      buflen = 0;
        uint16_t      offLen = 0;
        uint16_t      copylen;
        int8_t        ferr = 0;
        UINT          nb;
        uint16_t      numData = 0;

        DEBUG_PRINT( "Receiving %s\r\n", parameters );
        sendBegin( "150 Connected to port ", numConn);
        sendCatWrite( i2str( ftpDataConn[numConn].dataPort, numConn), numConn);
        ftpDataConn[numConn].timeBeginTrans = HAL_GetTick();
        ftpDataConn[numConn].bytesTransfered = 0;

        do
        {
          ftpDataConn[numConn].nerr = netconn_recv_tcp_pbuf(ftpDataConn[numConn].dataconn, &rcvbuf );
          if( ftpDataConn[numConn].nerr != ERR_OK )
            break;
          prcvbuf = rcvbuf->payload;
          buflen = rcvbuf->tot_len;
          while( buflen > 0 )
          {
            if( buflen <= FTP_BUF_SIZE - offLen )
              copylen = buflen;
            else
              copylen = FTP_BUF_SIZE - offLen;
            buflen -= copylen;
            memcpy(ftpDataConn[numConn].buf + offLen, prcvbuf, copylen );
            prcvbuf = (void*)((uint32_t)prcvbuf + (uint32_t)copylen);
            offLen += copylen;
            if( offLen == FTP_BUF_SIZE )
            {
              if( ferr == 0 )
              {
                ferr = f_write( &ftpDataConn[numConn].file, ftpDataConn[numConn].buf, FTP_BUF_SIZE, (UINT *) & nb );
                numData++;
              }
              offLen = 0;
            }
            else
            {
              if((offLen != 0) && (offLen < FTP_BUF_SIZE) && (numData == 0))
              {
                if( ferr == 0 )
                {
                  numData++;
                }
              }
            }
            ftpDataConn[numConn].bytesTransfered += copylen;
          }
          pbuf_free( rcvbuf );
        }

        while( TRUE ); // ferr == 0 );
        DEBUG_PRINT( "Received %u bytes\r\n", ftpDataConn[numConn].bytesTransfered);

        if( (numData != 0) && (ferr == 0) )
        {
          /* copy the last chunk of the file */
          ferr = f_write( &ftpDataConn[numConn].file, ftpDataConn[numConn].buf, offLen, (UINT *)&nb);
          /* close  the file */
          f_close(&ftpDataConn[numConn].file );
          if (ferr == 0)
          {
            DEBUG_PRINT( "Written %u bytes on %s\r\n", ftpDataConn[numConn].bytesTransfered, (char *)ftpDataConn[numConn].parameters);
            if ((strstr((char *)ftpDataConn[numConn].path, (char *)".bin") != NULL) && (strstr((char *)ftpDataConn[numConn].path, (char *)"ScameSCU") != NULL))
            {
              /*****  new FW file has been dowloaded *********/
              setFlagOnRtcBck(BACKUP_DATA_QSPI_REG, BACKUP_DATA_QSPI_STORED);
            }
          }
        } 
        osSemaphoreRelease(semaphoreFat); 


        if( ftpDataConn[numConn].nerr != ERR_CLSD  )
        {
          sendBegin( "451 Requested action aborted: communication error ", numConn);
          sendCatWrite( i2str( abs( ftpDataConn[numConn].nerr ), numConn), numConn);
        }
        if( ferr != 0  )
        {
          sendBegin( "451 Requested action aborted: file error ", numConn);
          sendCatWrite( i2str( abs(ferr), numConn), numConn);
        }
        dataClose(numConn);
        closeTransfer(numConn);
      }
    }
  }
  //
  //  MKD - Make Directory
  //
  else if( ! strcmp( command, "MKD" ))
  {
    if( strlen( parameters ) == 0 )
      sendWrite( "501 No directory name", numConn);
    else if( makePath(ftpDataConn[numConn].path, numConn))
    {
      if( fs_exists(ftpDataConn[numConn].path, numConn))
      {
        sendBegin( "521 \"", numConn);
        sendCat( parameters, numConn);
        sendCatWrite( "\" directory already exists", numConn);
      }
      else
      {
        DEBUG_PRINT(  "Creating directory %s\r\n", parameters );
        uint8_t ffs_result = f_mkdir(ftpDataConn[numConn].path);

        strLocalTime(ftpDataConn[numConn].str);
        DEBUG_PRINT( "Date/Time: %s\r\n", ftpDataConn[numConn].str);


        if( ffs_result == FR_OK )
        {
          sendBegin( "257 \"", numConn);
          sendCat( parameters, numConn);
          sendCatWrite( "\" created", numConn);
        }
        else
        {
          sendBegin( "550 Can't create \"", numConn);
          sendCat( parameters, numConn);
          sendCatWrite( "\"", numConn);
        }
      }
    }
  }
  //
  //  RMD - Remove a Directory
  //
  else if( ! strcmp( command, "RMD" ))
  {
    if( strlen( parameters ) == 0 )
      sendWrite( "501 No directory name", numConn);
    else if( makePath(ftpDataConn[numConn].path, numConn), numConn)
    {
      DEBUG_PRINT(  "Deleting %s\r\n", ftpDataConn[numConn].path, numConn);
      if( ! fs_exists(ftpDataConn[numConn].path, numConn))
      {
        sendBegin( "550 Directory \"", numConn);
        sendCat( parameters, numConn);
        sendCatWrite( "\" not found", numConn);
      }
      else if( f_unlink(ftpDataConn[numConn].path) == FR_OK)
      {
        sendBegin( "250 \"", numConn);
        sendCat( parameters, numConn);
        sendCatWrite( "\" removed", numConn);
      }
      else
      {
        sendBegin ("501 Can't delete \"", numConn);
        sendCat (parameters, numConn);
        sendCatWrite ("\"", numConn);
      }
    }
  }
  //
  //  RNFR - Rename From
  //
  else if( ! strcmp( command, "RNFR" ))
  {
    ftpDataConn[numConn].cwdRNFR[ 0 ] = 0;
    if( strlen( parameters ) == 0 )
      sendWrite( "501 No file name", numConn);
    else if( makePath(ftpDataConn[numConn].cwdRNFR, numConn))
    {
      if( ! fs_exists(ftpDataConn[numConn].cwdRNFR, numConn))
      {
        sendBegin( "550 File ", numConn);
        sendCat(parameters, numConn);
        sendCatWrite( " not found", numConn);
      }
      else
      {
        DEBUG_PRINT( "Renaming %s\r\n", ftpDataConn[numConn].cwdRNFR );
        sendWrite( "350 RNFR accepted - file exists, ready for destination", numConn);
      }
    }
  }
  //
  //  RNTO - Rename To
  //
  else if( ! strcmp( command, "RNTO" ))
  {
    char sdir[ FTP_CWD_SIZE ];
    if( strlen(ftpDataConn[numConn].cwdRNFR ) == 0 )
      sendWrite( "503 Need RNFR before RNTO", numConn);
    else if( strlen( parameters ) == 0 )
      sendWrite( "501 No file name", numConn);
    else if( makePath(ftpDataConn[numConn].path, numConn))
    {
      if( fs_exists(ftpDataConn[numConn].path, numConn))
      {
        sendBegin( "553 ", numConn);
        sendCat( parameters, numConn);
        sendCatWrite( " already exists", numConn);
      }
      else
      {
        strcpy( sdir, ftpDataConn[numConn].path );
        char * psep = strrchr( sdir, '/' );
        uint8_t fail = (uint8_t)(psep == NULL);
        if( ! fail )
        {
          if( psep == sdir )
            psep ++;
          * psep = 0;
          fail = ! ( fs_exists(sdir, numConn) &&
                     ( ftpDataConn[numConn].finfo.fattrib & AM_DIR || ! strcmp( sdir, "/")));
          if( fail )
          {
            sendBegin( "550 \"", numConn);
            sendCat( sdir, numConn);
            sendCatWrite( "\" is not directory", numConn);
          }
          else
          {
            DEBUG_PRINT(  "Renaming %s to %s\r\n", ftpDataConn[numConn].cwdRNFR, ftpDataConn[numConn].path );
            if( f_rename(ftpDataConn[numConn].cwdRNFR, ftpDataConn[numConn].path ) == FR_OK )
              sendWrite( "250 File successfully renamed or moved", numConn);
            else
              fail = (uint8_t)TRUE;
          }
        }
        if( fail )
          sendWrite( "451 Rename/move failure", numConn);
      }
    }
  }
  //
  //  SYST
  //
  /*
  else if( ! strcmp( command, "SYST" ))
  {
    sendWrite( "215 UNIX Type: L8" );
  }
  */

  ///////////////////////////////////////
  //                                   //
  //   EXTENSIONS COMMANDS (RFC 3659)  //
  //                                   //
  ///////////////////////////////////////

  //
  //  FEAT - New Features
  //
  else if( ! strcmp( command, "FEAT" ))
  {
    sendBegin( "211-Extensions supported:\r\n", numConn) ;
    sendCat( " MDTM\r\n", numConn);
    sendCat( " MLSD\r\n", numConn);
    sendCat( " SIZE\r\n", numConn);
    sendCat( " SITE FREE\r\n", numConn);
    sendCatWrite( "211 End.", numConn);
  }
  //
  //  MDTM - File Modification Time (see RFC 3659)
  //
  else if( ! strcmp( command, "MDTM" ))
  {
    char * fname;
    uint16_t date, time;
    uint8_t gettime;

    gettime = getDateTime( &date, &time, numConn);
    fname = parameters + gettime;

    if( strlen( fname ) == 0 )
      sendWrite( "501 No file name", numConn);
    else if( makePathFrom(ftpDataConn[numConn].path, fname, numConn ))
    {
      if( ! fs_exists(ftpDataConn[numConn].path, numConn))
      {
        sendBegin( "550 File ", numConn);
        sendCat(fname, numConn);
        sendCatWrite( " not found", numConn);
      }
      else if( gettime )
      {
        ftpDataConn[numConn].finfo.fdate = date;
        ftpDataConn[numConn].finfo.ftime = time;
        if( f_utime(ftpDataConn[numConn].path, &ftpDataConn[numConn].finfo ) == FR_OK )
          sendWrite( "200 Ok", numConn);
        else
          sendWrite( "550 Unable to modify time", numConn);
      }
      else
      {
        sendBegin( "213 ", numConn);
        sendCatWrite( makeDateTimeStr(ftpDataConn[numConn].finfo.fdate, ftpDataConn[numConn].finfo.ftime, numConn), numConn);
      }
    }
  }
  //
  //  SIZE - Size of the file
  //
  else if( ! strcmp( command, "SIZE" ))
  {
    if( strlen( parameters ) == 0 )
      sendWrite( "501 No file name", numConn);
    else if( makePath(ftpDataConn[numConn].path, numConn))
    {
      if( ! fs_exists(ftpDataConn[numConn].path, numConn) || ftpDataConn[numConn].finfo.fattrib & AM_DIR )
        sendWrite( "550 No such file", numConn);
      else
      {
        sendBegin( "213 ", numConn);
        sendCatWrite(i2str(ftpDataConn[numConn].finfo.fsize, numConn), numConn);
        f_close(&ftpDataConn[numConn].file );
      }
    }
  }
  //
  //  SITE - System command
  //
  else if( ! strcmp( command, "SITE" ))
  {
    if( ! strcmp( parameters, "FREE" ))
    {
      FATFS * fs;
      DWORD free_clust;
      f_getfree( "0:", &free_clust, &fs );
      sendBegin( "211 ", numConn);
      sendCat( i2str(free_clust * fs->csize >> 11, numConn), numConn);
      sendCat( " MB free of ", numConn);
      sendCat( i2str((fs->n_fatent - 2) * fs->csize >> 11, numConn), numConn);
      sendCatWrite(" MB capacity", numConn);
    }
    else
    {
      sendBegin("500 Unknow SITE command ", numConn);
      sendCatWrite(parameters, numConn);
    }
  }
  //
  //  STAT - Status command
  //
  else if( ! strcmp( command, "STAT" ))
  {
    uint8_t i, ncli;
    for( i = 0, ncli = 0; i < FTP_NBR_CLIENTS; i ++ )
      if( ss[ i ].ftpconn != NULL )
        ncli ++;
    sendBegin( "211-FTP server status\r\n", numConn);
    sendCat( " Local time is ", numConn);
    sendCat( strLocalTime(ftpDataConn[numConn].str), numConn);
    sendCat( "\r\n ", numConn);
    sendCat( i2str(ncli, numConn), numConn);
    sendCat( " user(s) currently connected to up to ", numConn);
    sendCat( i2str(FTP_NBR_CLIENTS, numConn), numConn);
    sendCat( "\r\n You will be disconnected after ", numConn);
    sendCat( i2str(FTP_TIME_OUT, numConn), numConn);
    sendCat( " minutes of inactivity\r\n", numConn);
    sendCatWrite( "211 End.", numConn);
  }
  //
  //  Unknow command
  //
  else
    sendWrite( "500 Unknow command", numConn);
  return (uint8_t)TRUE;
}

// =========================================================
//
//                       Ftp server
//
// =========================================================

void FtpServer_service (struct server_stru* pMsgRx)
//void FtpServer_service( int8_t n, struct netconn *ctrlcn )
{
  uint16_t          dummy;
  ip_addr_t         ippeer;
  //struct sdlog_stru sdl;
  uint32_t          systemTimeBeginConnect;
  uint32_t          rtcBeginTime;
  uint8_t           n; 
  struct netconn*   ctrlcn;
  char              *pUser, *pPass;

  // variables initialization
  rtcBeginTime = get_fattime();
  systemTimeBeginConnect = HAL_GetTick();
  ctrlcn = pMsgRx->ftpconn;
  n = pMsgRx->num;
  
  strcpy(ftpDataConn[n].cwdName, "/");  //   Set the root directory

  ftpDataConn[n].cwdRNFR[ 0 ] = 0;
  ftpDataConn[n].num = n;
  ftpDataConn[n].ctrlconn = ctrlcn;
  ftpDataConn[n].listdataconn = NULL;
  ftpDataConn[n].dataconn = NULL;
  ftpDataConn[n].dataPort = FTP_DATA_PORT + n;
  ftpDataConn[n].cmdStatus = 0;
  ftpDataConn[n].dataConnMode = NOTSET;
  memcpy(ftpDataConn[n].finfo.fname, ftpDataConn[n].lfn, _MAX_LFN + 1);
  ftpDataConn[n].finfo.fsize = _MAX_LFN + 1;

  //  Get the local and peer IP
  netconn_addr(ctrlcn, &ftpDataConn[n].ipserver, &dummy );
  netconn_peer(ctrlcn, &ippeer, &dummy );

  sendBegin( "220---   Welcome to FTP Server!   ---\r\n", n);
  sendCat( "   ---  for LWIP & STM32F765  ---\r\n", n);
  sendCat( "   ---   by AESYS CPU1629   ---\r\n", n);
  sendCat( "220 --   Version ", n);
  sendCat( FTP_VERSION, n);
  sendCatWrite( "   --", n);

  DEBUG_PRINT( "Client connected!\r\n" );

  //  Wait for user name during 10 seconds
  netconn_set_recvtimeout(ftpDataConn[n].ctrlconn, (int32_t)(10 * 1000));
  if( readCommand(n) < 0 )
    goto close;
  if( strcmp(ftpDataConn[n].command, "USER" ))
  {
    sendWrite( "500 Syntax error", n);
    goto close;
  }
  pUser = getFtpUsername();
  if (pUser[0] == '\0')
  {
    pUser = (char*)ftpDefaultAccount[0];
  }
  if( strcmp(ftpDataConn[n].parameters, pUser))
  {
    sendWrite( "530 ", n);
    goto close;
  }
  sendWrite( "331 OK. Password required", n);

  //  Wait for password during 10 seconds
  if( readCommand(n) < 0 )
    goto close;
  if( strcmp(ftpDataConn[n].command, "PASS" ))
  {
    sendWrite( "500 Syntax error", n);
    goto close;
  }
  pPass = getFtpPassword();
  if (pPass[0] == '\0')
  {
    pPass = (char*)ftpDefaultAccount[1];
  }
  if( strcmp(ftpDataConn[n].parameters, pPass ))
  {
    sendWrite( "530 ", n);
    goto close;
  }
  sendWrite( "230 OK.", n);

  //  Wait for user commands
  //  Disconnect if FTP_TIME_OUT minutes of inactivity
  //netconn_set_recvtimeout(ftpDataConn[n].ctrlconn, (int32_t)( FTP_TIME_OUT * 60 * 1000 ));
  netconn_set_recvtimeout(ftpDataConn[n].ctrlconn, (int32_t)( FTP_TIME_OUT * 3 * 1000 ));
  while( TRUE )
  {
    int8_t err = readCommand(n);
    if( err == -4 ) // time out
      goto close;
    if( err < 0 )
      goto close;
    if( ! processCommand(ftpDataConn[n].command, ftpDataConn[n].parameters, n))
      goto bye;
  }

  bye:
  //sendWrite( "221 Goodbye", n);

  close:
  sendWrite( "221 Goodbye", n);
  //  Close the connections

  dataClose(n);
  if( ftpDataConn[n].listdataconn != NULL )
  {
    netconn_close(ftpDataConn[n].listdataconn );
    netconn_delete(ftpDataConn[n].listdataconn );
  }

  //  Write data to log
  uint32_t timeConnect = (uint32_t) ( HAL_GetTick() - systemTimeBeginConnect );
  strRTCDateTime(ftpDataConn[n].str, rtcBeginTime );
  strcpy(ftpDataConn[n].buf, "Connected at " );
  strcat(ftpDataConn[n].buf, ftpDataConn[n].str );
  strcat(ftpDataConn[n].buf, " for ");
  strmSec2hms(ftpDataConn[n].str, timeConnect);
  strcat(ftpDataConn[n].buf, ftpDataConn[n].str );
  strcat(ftpDataConn[n].buf, "\r\n" );
  //sdl.line = ftpDataConn[n].buf;
  strcpy(ftpDataConn[n].str, "/Log/" );
  strcat(ftpDataConn[n].str, ipaddr_ntoa( &ippeer ));
  strcat(ftpDataConn[n].str, ".log" );
  //sdl.file = ftpDataConn[n].str;
  //sdl.append = FALSE;
  //chMsgSend( tsdlog, (msg_t) & sdl );
#ifdef AESYS
  moveFileFromRootToFolder(FAT_DEVICE_FLASH);
#endif
  DEBUG_PRINT( "Client disconnected\r\n" );
}


// Convert a positive integer to string
//
// Parameters:
//   s: string where the conversion is made (must be large enough)
//   i: integer t convert
//   z: if >= 0, size of string s
//      if < 0, size of returned string
//              (must be <= than size of string s; leading space filled with '0')
//
// Return pointer to string

char * int2strZ( char * s, uint32_t i, int8_t z )
{
  char * psi = s + abs( z );

  * -- psi = 0;
  if( i == 0 )
    * -- psi = '0';
  for( ; i; i /= 10 )
    * -- psi = '0' + i % 10;
  if( z < 0 )
    while( psi > s )
      * -- psi = '0';
  return psi;
}

// Convert an integer to string
//
// Parameters:
//   s: string where the conversion is made (must be large enough)
//   i: integer t convert
//   ls: size of string s
//
// Return pointer to string

static char * int2str( char * s, int32_t i, int8_t ls, uint8_t numConn)
{
  if( i >= 0 )
    return int2strZ( s, i, ls );
  char * pstr = int2strZ( s + 1, - i, ls - 1 );
  * -- pstr = '-';
  return pstr;
}


void resetftpDataConn (uint8_t ix)
{
  ftpDataConn[ix].listdataconn = NULL;
  ftpDataConn[ix].dataconn = NULL;
  ftpDataConn[ix].ctrlconn = NULL;
  ftpDataConn[ix].inbuf = NULL;
  memset ((void*)&ftpDataConn[ix].ipclient, 0, sizeof(ip_addr_t));
  memset ((void*)&ftpDataConn[ix].ipserver, 0, sizeof(ip_addr_t));
  memset ((void*)&ftpDataConn[ix].file, 0, sizeof(FIL));
  memset ((void*)&ftpDataConn[ix].finfo, 0, sizeof(FILINFO));
  memset ((void*)ftpDataConn[ix].lfn, 0, _MAX_LFN + 1);
  ftpDataConn[ix].dataPort = 0;
  ftpDataConn[ix].cmdStatus = 0;
  memset ((void*)&ftpDataConn[ix].command[0], 0, (size_t)6);
  memset ((void*)ftpDataConn[ix].parameters, 0, FTP_PARAM_SIZE);
  memset ((void*)ftpDataConn[ix].cwdName, 0, FTP_CWD_SIZE);
  memset ((void*)ftpDataConn[ix].cwdRNFR, 0, FTP_CWD_SIZE);
  memset ((void*)ftpDataConn[ix].path, 0, FTP_CWD_SIZE);
  memset ((void*)ftpDataConn[ix].str, 0, 32);
  ftpDataConn[ix].timeBeginTrans = 0;
  ftpDataConn[ix].bytesTransfered = 0;
  ftpDataConn[ix].nerr = 0;
  ftpDataConn[ix].num = 0;
  memset ((void*)ftpDataConn[ix].buf, 0, FTP_BUF_SIZE);
  ftpDataConn[ix].pbuf = 0;
  ftpDataConn[ix].dataConnMode = NOTSET;
}  


/**
*
* @brief        Get the pointer to ftp user  
*
* @param [in]   none
*
* @retval       char*: pointer ftp user string 
*
***********************************************************************************************************************/
__weak char *getFtpUsername (void)
{
  return((char *)ftpDefaultAccount[0]);
  //return((char *)&_configuration.ftpUser[0]);
}

/**
*
* @brief        Get the pointer to ftp user  
*
* @param [in]   none
*
* @retval       char*: pointer ftp user string 
*
***********************************************************************************************************************/
__weak char *getFtpPassword (void)
{
  return((char *)ftpDefaultAccount[1]);
  //return((char *)&_configuration.ftpPass[0]);
}

#endif
/*************** END OF FILE ******************************************************************************************/


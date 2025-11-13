#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "usart.h"
#include "esp32.h"
#include "network_utils.h"
#include "cmsis_os2.h"
    
static const char *At_responseList[] = {
#define ITEM( code, value ) value,
  AT_STRING_RESPONSE_LIST
#undef ITEM
};

static const char *At_reportList[] = {
#define ITEM( code, value ) value,
  AT_STRING_REPORT_LIST
#undef ITEM
};

static const char *At_cmdList[] = {
#define ITEM( code, value ) value,
  AT_STRING_CMD_LIST
#undef ITEM
};

static const char *At_respGmr[] = {
#define ITEM( code, value ) value,
  AT_STRING_RESP_GMR_LIST
#undef ITEM
};

static const char *At_respLink[] = {
#define ITEM( code, value ) value,
  AT_STRING_RESP_LINK_LIST
#undef ITEM
};

static const char *At_linkType[] = {
#define ITEM( code, value ) value,
  AT_STRING_RESP_LINK_TYPE_LIST
#undef ITEM
};

static const char *At_serverType[] = {
#define ITEM( code, value ) value,
  AT_SERVER_TYPE_LIST
#undef ITEM
};

static char at_header[] = "AT+";
static char at_crlf[] = "\r\n";

static uint8_t espMem[ AT_PARSER_BUFFER_BYTESIZE ];
static uint8_t cmdResponseMem[ AT_PARSER_BUFFER_BYTESIZE ];

static AtEsp32 atEsp32parser;
#define parser ( &atEsp32parser )
#define usart ( &usart3 )

static int32_t At_receive( void );
static AtParserState At_analyzeLineData( void );
static AtCmdCode At_getCommandCode( Stream * const me );
static AtResponseResult At_getResponseResult( Stream * const me );
static AtReportCode At_getReportCode( Stream * const me );
static AtGmrCode At_getGmrResponseCode( Stream * const me );
static AtLinkCode At_getLinkResponseCode( Stream * const me );
static int32_t At_getRespArg( char *buff, uint32_t size );

/* Support function */
int32_t At_send_address( AtCmdCode code, AtCmdType type, uint32_t ip, uint32_t gateway, uint32_t netmask );

/* Format data function */
static void       At_stringToMac( char *buff, uint8_t mac[ 6 ] );
static uint32_t   At_stringToIp( char *buff );
static void       At_stringToString( char *buff, char *dst );
static AtLinkType At_stringToLinkTypeCode( char *buff );

static void At_lock( void )
{
  osMutexAcquire( parser->mutex, osWaitForever );
}

static void At_unlock( void )
{
  osMutexRelease( parser->mutex );
}

int32_t At_init( void )
{
  parser->state = AT_STATE_ANALIZE;
  parser->cmd_sent = AT_CMD_UNKNOWN;
  parser->response_code = AT_CMD_UNKNOWN;
  parser->response_len = 0;
  parser->response_result = AT_RESPONSE_UNKNOWN;
  parser->report_code = AT_REPORT_UNKNOWN;
  parser->response_link = AT_RESP_LINK_UNKNOWN;
  Stream_init( &parser->rxStream, espMem, AT_PARSER_BUFFER_BYTESIZE );
  Stream_init( &parser->cmdRespStream, cmdResponseMem, AT_PARSER_BUFFER_BYTESIZE );
  usart->init();
  parser->mutex = osMutexNew( NULL );
  return 0;
}

int32_t At_deinit( void )
{
  int32_t res;
  
  res = usart->deinit();
  return res;
}

void At_parser( void )
{
  uint32_t sleep;
  int32_t len;
  
  sleep = 0U;
  
  while( sleep == 0 )
  {
    At_receive();
    switch( parser->state )
    {
    case AT_STATE_ANALIZE:
      parser->state = At_analyzeLineData();
      break;
    case AT_STATE_RESP_GEN:
      switch( parser->response_result )
      {
      case AT_OK:
      case AT_ERROR:
      case AT_SEND_OK:
      case AT_SEND_FAIL:
        At_notify( AT_NOTIFY_RESPONSE, NULL );
        //sleep = 1U;
        break;
      default:
        // Do nothing
        break;
      }
      parser->state = AT_STATE_FLUSH;
      break;
    case AT_STATE_REPO_GEN:
      switch( parser->report_code )
      {
      case AT_WIFI_CONNECTED:
        At_notify( AT_NOTIFY_CONNECTED, NULL );
        break;
      case AT_WIFI_GOT_IP:
        At_notify( AT_NOTIFY_GOT_IP, NULL );
        break;
      case AT_WIFI_DISCONNECT:
        At_notify( AT_NOTIFY_DISCONNECT, NULL );
        break;
      case AT_READY:
        At_notify( AT_NOTIFY_READY, NULL );
        break;
      case AT_FAIL:
      case AT_BUSY_P:
      case AT_AT:
      case AT_ERR_CODE:
      default:
        // Do nothing
        break;
      }
      parser->state = AT_STATE_FLUSH;
      break;
    case AT_STATE_RESP_GMR:
      Stream_copy( &parser->cmdRespStream, &parser->rxStream, parser->response_len + 2 );
      parser->state = AT_STATE_ANALIZE;
      break;
    case AT_STATE_RESP_CMD:
      if( parser->response_code == AT_CMD_IPD )
      {
        Stream_copy( &parser->cmdRespStream, &parser->rxStream, parser->response_len + 1 );
        At_notify( AT_NOTIFY_LINK_RX_INIT, &(parser->ipd_rx) );
        parser->state = AT_STATE_RECV_DATA;
      }
      else if( parser->response_code == AT_CMD_CIPSENDL  )
      {
        // Not manage the response from long send command
        parser->state = AT_STATE_FLUSH;
      }
      else if( parser->response_code == AT_CMD_CWLAP )
      {
        Stream_copy( &parser->cmdRespStream, &parser->rxStream, parser->response_len + 2 );
        At_notify( AT_NOTIFY_SCAN_RESULT, NULL );
        parser->state = AT_STATE_ANALIZE;
      }
      else
      {
        Stream_copy( &parser->cmdRespStream, &parser->rxStream, parser->response_len + 2 );
        parser->state = AT_STATE_ANALIZE;
        if( parser->response_code == AT_CMD_STA_CONNECTED )
        {
          At_notify( AT_NOTIFY_STA_CONNECTED, NULL );
        }
        else if( parser->response_code == AT_CMD_DIST_STA_IP )
        {
          At_notify( AT_NOTIFY_DIST_STA_IP, NULL );
        }
        else if( parser->response_code == AT_CMD_STA_DISCONNECTED )
        {
          At_notify( AT_NOTIFY_STA_DISCONNECTED, NULL );
        }
        else if( parser->response_code == AT_CMD_CIUPDATE )
        {
          At_notify( AT_NOTIFY_UPDATE, NULL );
        }
      }
      break;
    case AT_STATE_RECV_DATA:
      {
        uint32_t p = (uint32_t)&parser->rxStream;
        At_notify( AT_NOTIFY_LINK_RX_DATA, &p );
        if( p == 0 )
        {
          parser->state = AT_STATE_ANALIZE;
        }
        else
        {
          parser->ipd_rx = p;
        }
      }
      break;
    case AT_STATE_RESP_LINK:
      Stream_copy( &parser->cmdRespStream, &parser->rxStream, parser->response_len + 2 );
      if( parser->response_link == AT_CONNECT )
      {
        At_notify( AT_NOTIFY_LINK_CONNECT, NULL );
      }
      else if( parser->response_link == AT_CLOSED )
      {
        At_notify( AT_NOTIFY_LINK_CLOSED, NULL );
      }
      parser->state = AT_STATE_ANALIZE;
      break;
    case AT_STATE_REQ_SEND:
      sleep = 1U;
      Stream_flush( &parser->rxStream, 1 );
      At_notify( AT_NOTIFY_REQ_SEND, NULL );
      parser->state = AT_STATE_ANALIZE;
      break;
    case AT_STATE_FLUSH:
      len = Stream_find( &parser->rxStream, at_crlf, 2 );
      if( len != -1 )
      {
        Stream_flush( &parser->rxStream, len + 2 );
      }
      parser->state = AT_STATE_ANALIZE;
      break;
    case AT_STATE_WAIT:
      sleep = 1;
      parser->state = AT_STATE_ANALIZE;
      break;
    }
  }
}

static int32_t At_receive( void )
{
  uint32_t n, cnt;
  
  Stream_lock( &parser->rxStream );
  
  cnt = 0;
  n = parser->rxStream.size - parser->rxStream.count;
  if( n > 0 )
  {
    cnt = usart->receive( &parser->rxStream.buff[ parser->rxStream.count ], n );
    parser->rxStream.count += cnt;
  }
  
  Stream_unlock( &parser->rxStream );

  return cnt;
}

static AtParserState At_analyzeLineData( void )
{
  char c;
  int32_t len;
  AtCmdCode cmd;
  
  while( Stream_getCount( &parser->rxStream ) > 0 )
  {
    c = ( char )Stream_getByte( &parser->rxStream );
    if( c == '+' )
    {
      cmd = At_getCommandCode( &parser->rxStream );
      if( cmd == AT_CMD_IPD )
      {
        // Fix response_len for +IPD report
        len = Stream_find( &parser->rxStream, ":", 1 );
      }
      else
      {
        len = Stream_find( &parser->rxStream, at_crlf, 2 );
      }
      if( len == -1 )
      {
        // The stream doesn't terminate with "\r\n" characters.
        return AT_STATE_WAIT;
      }
      parser->response_len = len;
      parser->response_code = cmd;
      if( cmd == AT_CMD_LINK_CONN )
      {
        parser->response_link = AT_CONNECT;
        return AT_STATE_RESP_LINK;
      }
      return AT_STATE_RESP_CMD;
    }
    else if( ( ( c >= 'A') && ( c <= 'Z') ) || ( ( c >= 'a') && ( c <= 'z') ) )
    {
      len = Stream_find( &parser->rxStream, at_crlf, 2 );
      if( len == -1 )
      {
        // The stream doesn't terminate with "\r\n" characters.
        return AT_STATE_WAIT;
      }
      else
      {
        parser->response_len = len;
        AtGmrCode gmrCode = At_getGmrResponseCode( &parser->rxStream );
        if( gmrCode != AT_GMR_UNKNOWN )
        {
          return AT_STATE_RESP_GMR;
        }
        else
        {
          parser->report_code = At_getReportCode( &parser->rxStream );
          if( parser->report_code == AT_REPORT_UNKNOWN )
          {
            parser->response_result = At_getResponseResult( &parser->rxStream );
            if( parser->response_result == AT_RESPONSE_UNKNOWN )
            {
              return AT_STATE_FLUSH;
            }
            else
            {
              return AT_STATE_RESP_GEN;
            }
          }
          else
          {
            if( parser->report_code == AT_AT )
            {
               return AT_STATE_FLUSH;
            }
            return AT_STATE_REPO_GEN;
          }
        }
      }
    }
    else if( ( c >= '0' ) && ( c <= '9' ) )
    {
      len = Stream_find( &parser->rxStream, at_crlf, 2 );
      if( len == -1 )
      {
        // The stream doesn't terminate with "\r\n" characters.
        return AT_STATE_WAIT;
      }
      else
      {
        parser->response_len = len;
        parser->response_link = At_getLinkResponseCode( &parser->rxStream );
        return AT_STATE_RESP_LINK;
      }
    }
    else if( c == '>' )
    {
      return AT_STATE_REQ_SEND;
    }
    Stream_flush( &parser->rxStream, 1 );
  }
  return AT_STATE_WAIT;
}

static AtCmdCode At_getCommandCode( Stream * const me )
{
  uint32_t size;
  AtCmdCode result;
    
  result = AT_CMD_UNKNOWN;
  size = ( sizeof( At_cmdList ) / sizeof ( At_cmdList[ 0 ] ) );
  for( uint32_t i = 0; i < size; i++ )
  {
    if( Stream_compare( me, 1, At_cmdList[ i ] ) == 0 )
    {
      result = ( AtCmdCode )i;
      break;
    }
  }
  if( result == AT_CMD_UNKNOWN )
  {
    if( Stream_compare( me, 1, "CIPUPDATE" ) == 0 )
    {
      result = AT_CMD_CIUPDATE;
    }
  }
  return result;
}

static AtResponseResult At_getResponseResult( Stream * const me )
{
  uint32_t size;
  AtResponseResult result;
  
  result = AT_RESPONSE_UNKNOWN;
  size = ( sizeof( At_responseList ) / sizeof( At_responseList[0] ) );
  for( uint32_t i = 0; i < size; i++ )
  {
    if( Stream_compare( me, 0, At_responseList[ i ] ) == 0 )
    {
      return ( AtResponseResult )i;
    }
  }
  return result;
}

static AtReportCode At_getReportCode( Stream * const me )
{
  uint32_t size;
  AtReportCode result;
  
  result = AT_REPORT_UNKNOWN;
  size = ( sizeof( At_reportList ) / sizeof( At_reportList[0] ) );
  for( uint32_t i = 0; i < size; i++ )
  {
    if( Stream_compare( me, 0, At_reportList[ i ] ) == 0 )
    {
      return ( AtReportCode )i;
    }
  }
  return result;
}

static AtGmrCode At_getGmrResponseCode( Stream * const me )
{
  uint32_t size;
  AtGmrCode result;
  
  result = AT_GMR_UNKNOWN;
  size = ( sizeof( At_respGmr ) / sizeof( At_respGmr[0] ) );
  for( uint32_t i = 0; i < size; i++ )
  {
    if( Stream_compare( me, 0, At_respGmr[ i ] ) == 0 )
    {
      return ( AtGmrCode )i;
    }
  }
  return result;
}

static AtLinkCode At_getLinkResponseCode( Stream * const me )
{
  uint32_t size;
  AtLinkCode result;
  
  result = AT_RESP_LINK_UNKNOWN;
  size = ( sizeof( At_respLink ) / sizeof( At_respLink[ 0 ] ) );
  for( uint32_t i = 0; i < size; i++ )
  {
    if( Stream_compare( me, 2, At_respLink[ i ] ) == 0 )
    {
      return ( AtLinkCode )i;
    }
  }
  return result;
}

static const char * At_getCmdByCode( AtCmdCode code )
{
  if( code < AT_CMD_NUM )
  {
    return At_cmdList[ code ];
  }
  return NULL;
}

uint32_t At_cmd_prepare( AtCmdCode code, AtCmdType type, char *out )
{
  const char *cmd;
  char cmdt;
  uint32_t size;
  
  cmd = At_getCmdByCode( code );
  
  switch( type )
  {
  case AT_CMD_QUERY:
    cmdt = '?';
    break;
  case AT_CMD_SET:
    cmdt = '=';
    break;
  default:
  case AT_CMD_EXECUTE:
    cmdt = '\0';
    break;
  }
  
  size = sprintf( out, "%s%s%c", at_header, cmd, cmdt );
  
  if( type == AT_CMD_EXECUTE )
  {
    size -= 1;
  }
  
  return size;
}

int32_t At_cmd_send( AtCmdCode code, char *buff, uint32_t size )
{
  int32_t sent;
  
  parser->cmd_sent = code;
  size += sprintf( &buff[ size ], "%s", at_crlf );
  sent = At_send_raw( ( uint8_t* )buff, size );
  if( sent == size )
    return 0;
  return -1;
}

int32_t At_send_raw( uint8_t *data, uint16_t len )
{
  return usart->send( data, len );
}

int32_t At_send_addressAp( AtCmdType type, uint32_t ip, uint32_t gateway, uint32_t netmask )
{
  return At_send_address( AT_CMD_CIPAP, type, ip, gateway, netmask );
}

int32_t At_send_addressStation( AtCmdType type, uint32_t ip, uint32_t gateway, uint32_t netmask )
{
  return At_send_address( AT_CMD_CIPSTA, type, ip, gateway, netmask );
}

int32_t At_send_address( AtCmdCode code, AtCmdType type, uint32_t ip, uint32_t gateway, uint32_t netmask )
{
  char out[ 66 ];
  uint32_t n;
  
  n = At_cmd_prepare( code, type, out );
  if( type == AT_CMD_SET )
  {
    n += sprintf( &out[ n ], "\"%d.%d.%d.%d\"", IP_ADDRESS( ip ) );
    if( gateway != NULL )
    {
      n += sprintf( &out[ n ], ",\"%d.%d.%d.%d\"", IP_ADDRESS( gateway ) );
      if( netmask != NULL )
      {
        n += sprintf( &out[ n ], ",\"%d.%d.%d.%d\"", IP_ADDRESS( netmask ) );
      }
    }
  }
  return At_cmd_send( code, out, n );
}

int32_t At_recv_ip( uint32_t *ip )
{
  int32_t res;
  char buff[ 18 ];
  
  At_lock();
  
  do
  {
    res = At_getRespArg( buff, sizeof( buff ) );
    if( res < 0 )
      break;
    if( res != 1 )
    {
      *ip = At_stringToIp( buff );
    }
  }
  while( res < 2 );
  
  if( res == 3 )
  {
    res = 0;
  } 
  else if( res == 2 )
  {
    res = 1;
  }
  
  At_unlock();
  
  return res;
}

int32_t At_send_multiConnection( AtCmdType type, uint8_t enable )
{
  char out[ 13 ];
  uint32_t n;
  
  n = At_cmd_prepare( AT_CMD_CIPMUX, type, out );
  if( type == AT_CMD_SET )
    n += sprintf( &out[ n ], "%d", enable == 0 ? 0 : 1 );
  return At_cmd_send( AT_CMD_CIPMUX, out, n );
}

int32_t At_send_sysstore( AtCmdType type, uint8_t enable )
{
  char out[ 16 ];
  uint32_t n;
  
  n = At_cmd_prepare( AT_CMD_SYSSTORE, type, out );
  if( type == AT_CMD_SET )
    n += sprintf( &out[ n ], "%d", enable == 0 ? 0 : 1 );
  return At_cmd_send( AT_CMD_SYSSTORE, out, n );
}

int32_t At_send_sysmsg( AtCmdType type, uint8_t setup )
{
  char out[ 13 ];
  uint32_t n;
  
  n = At_cmd_prepare( AT_CMD_SYSMSG, type, out );
  if( type == AT_CMD_SET )
    n += sprintf( &out[ n ], "%d", setup );
  return At_cmd_send( AT_CMD_SYSMSG, out, n );
}

int32_t At_send_versionInfo( void )
{
  char out[ 8 ];
  uint32_t n;
  
  n = At_cmd_prepare( AT_CMD_GMR, AT_CMD_EXECUTE, out );
  return At_cmd_send( AT_CMD_GMR, out, n );
}

int32_t At_recv_versionInfo( char *info, uint32_t size )
{
  int32_t n;
  
  At_lock();
  
  n = Stream_getCount( &parser->cmdRespStream );
  if( n > size )
    n = size;
  n = Stream_read( &parser->cmdRespStream, ( uint8_t* )info, n );
  info[ n ] = '\0';
  
  At_unlock();
  
  return n;
}

int32_t At_send_currentUart( AtCmdType type, AtUartConfig * const config )
{
  char out[ 30 ];
  int32_t n;
  
  n = At_cmd_prepare( AT_CMD_UART_CUR, type, out );
  if( type == AT_CMD_SET )
    n += sprintf( &out[ n ], "%d,%d,%d,%d,%d", 
                  config->baudrate, 
                  config->data_bits, 
                  config->stop_bits, 
                  config->parity, 
                  config->flow_control );
  return At_cmd_send( AT_CMD_UART_CUR, out, n );
}

int32_t At_send_test( void )
{
  char out[ 5 ];
  int32_t n;
  
  n = sprintf( out, "%s", At_cmdList[ AT_CMD_TEST ] );
  return At_cmd_send( AT_CMD_TEST, out, n );
}

int32_t At_send_echo( uint8_t enable )
{
  char out[ 7 ];
  int32_t n;
  
  n = sprintf( out, "%s%d", At_cmdList[ AT_CMD_ECHO ], enable == 0 ? 0 : 1 );
  return At_cmd_send( AT_CMD_TEST, out, n );
}

int32_t At_send_macAp( AtCmdType type, uint8_t mac[ 6 ] )
{
  char out[ 35 ];
  int32_t n;
  
  n = At_cmd_prepare( AT_CMD_CIPAPMAC, type, out );
  if( type == AT_CMD_SET )
    n += sprintf( &out[ n ], "\"%hhx:%hhx:%hhx:%hhx:%hhx:%hhx\"", MAC_ADDRESS( mac ) );
  return At_cmd_send( AT_CMD_CIPAPMAC, out, n );
}

int32_t At_send_macStation( AtCmdType type, uint8_t mac[ 6 ] )
{
  char out[ 35 ];
  int32_t n;
  
  n = At_cmd_prepare( AT_CMD_CIPSTAMAC, type, out );
  if( type == AT_CMD_SET )
    n += sprintf( &out[ n ], "\"%hhx:%hhx:%hhx:%hhx:%hhx:%hhx\"", MAC_ADDRESS( mac ) );
  return At_cmd_send( AT_CMD_CIPSTAMAC, out, n );
}

int32_t At_recv_mac( uint8_t mac[ 6 ] )
{
  int32_t res;
  char buff[ 20 ];
  
  At_lock();
  
  res = At_getRespArg( buff, sizeof( buff ) );
  if( res == 3 )
  {
    At_stringToMac( buff, mac );
    res = 0;
  }
  else
  {
    res = -1;
  }
  
  At_unlock();
  
  return res;
}

int32_t At_recv_macAndIp( uint8_t mac[ 6 ], uint32_t *ip )
{
  int32_t res;
  char buff[ 20 ];
  uint32_t arg;
  
  At_lock();
  
  arg = 0;
  do
  {
    res = At_getRespArg( buff, sizeof( buff ) );
    switch( arg )
    {
    case 0:
      At_stringToMac( buff, mac );
      break;
    case 1:
      *ip = At_stringToIp( buff );
      break;
    default:
      break;
    }
    arg++;
  }
  while( res < 2 );
  
  if( res == 3 )
  {
    res = 0;
  }
  else if( res == 2 )
  {
    res = 1;
  }
  
  At_unlock();
  
  return res;
}

int32_t At_send_mode( AtCmdType type, AtMode mode )
{
  char out[ 14 ];
  uint32_t n;
  
  n = At_cmd_prepare( AT_CMD_MODE, type, out );
  if( type == AT_CMD_SET )
    n += sprintf( &out[ n ], "%d", mode );
  return At_cmd_send( AT_CMD_MODE, out, n );
}

int32_t At_recv_mode( AtMode * const mode )
{
  char buff[ 2 ];
  int32_t res;
  
  At_lock();
  
  res = At_getRespArg( buff, sizeof( buff ) );
  if( res > 0 )
  {
    *mode = ( AtMode )( buff[0] - '0' );
    res = 0;
  }
  else
  {
    res = -1;
  }
  
  At_unlock();
  
  return res;
}

int32_t At_recv_response( void )
{
  int32_t res;
  
  At_lock();
  
  res = parser->response_result;
  
  At_unlock();
  
  return res;
}

static int32_t At_sprintstring(char *dst, const char *src)
{
  uint32_t i, j;
  
  for (i = 0, j = 0; i < strlen(src); i++, j++) {
    if (src[i] == ',' || src[i] == '\\' || src[i] == '"') {
      dst[j] = '\\';
      j++;
    }
    dst[j] = src[i];
  }
  return j;
}

int32_t At_send_confAp( AtCmdType type, AtApConfig * const config )
{
  char out[ 128 ];
  int32_t n;
  
  n = At_cmd_prepare( AT_CMD_CWSAP, type, out );
  if( type == AT_CMD_SET )
  {
    if( config->chl < AT_MIN_CHANNEL_ID || config->chl > AT_MAX_CHANNEL_ID )
    {
      return -1;
    }
    n += sprintf( &out[ n ], "\"");
    n += At_sprintstring( &out[ n ], config->ssid);
    n += sprintf( &out[ n ], "\",\"");
    n += At_sprintstring( &out[ n ], config->pass);
    n += sprintf( &out[ n ], "\",%d,%d,", config->chl, config->ecn );
    n += sprintf( &out[ n ], "%d,%d", config->max_conn, config->ssid_hidden );
  }
  return At_cmd_send( AT_CMD_CWSAP, out, n );
}

int32_t At_recv_confAp( AtApConfig * const config )
{
  int32_t res;
  char buff[ 64 + 1 ];
  uint32_t arg;
  
  At_lock();
  
  arg = 0;
  do
  {
    res = At_getRespArg( buff, sizeof( buff ) );
    
    if( res < 0 )
      break;
    
    switch( arg )
    {
    case 0:
      At_stringToString( buff, config->ssid );
      break;
    case 1:
      At_stringToString( buff, config->pass );
      break;
    case 2:
      config->chl = strtoul( buff, NULL, 10 );
      break;
    case 3:
      config->ecn = strtoul( buff, NULL, 10 );
      break;
    case 4:
      config->max_conn = strtoul( buff, NULL, 10 );
      break;
    case 5:
      config->ssid_hidden = buff[ 0 ] - '0';
      break;
    }
    arg++;
  }
  while( res < 2 );
  
  if( res > 0 )
  {
    res = 0;
  }
  
  At_unlock();
  
  return res;
}

int32_t At_send_getStatus( void )
{
  uint32_t n;
  char out[ 14 ];
  
  n = At_cmd_prepare( AT_CMD_CWSTATE, AT_CMD_QUERY, out );
  return At_cmd_send( AT_CMD_CWSTATE, out, n );
}

int32_t At_recv_getStatus( char *ssid )
{
  int32_t res, state;
  char buff[ 32 + 1 ];
  uint32_t arg;
  
  At_lock();
  
  arg = 0;
  do
  {
    res = At_getRespArg( buff, sizeof( buff ) );
    
    if( res < 0 )
      break;
    
    switch( arg )
    {
    case 0:
      state = buff[ 0 ] - '0';
      break;
    case 1:
      At_stringToString( buff, ssid );
      break;
    }
    arg++;
  }
  while( res < 2 );
  
  if( res > 0 )
  {
    res = state;
  }
  
  At_unlock();
  
  return res;
}

int32_t At_send_connectToAp( AtCmdType type, const char *ssid, const char *pass, const uint8_t mac[6] )
{
  char out[ 128 ];
  int32_t n;
  
  n = At_cmd_prepare( AT_CMD_CWJAP, type, out );
  if( type == AT_CMD_SET ) {
    n += sprintf( &out[ n ], "\"" );
    n += At_sprintstring( &out[ n ], ssid );
    n += sprintf( &out[ n ], "\",\"" );
    n += At_sprintstring( &out[ n ], pass );
    n += sprintf( &out[ n ], "\"" );
    if (mac != NULL) {
      if (!WIFI_CONFIG_MAC_IS_NULL(mac)) {
        n += sprintf( &out[ n ], ",\"%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx\"", MAC_ADDRESS(mac));
      }
    }
  }
  return At_cmd_send( AT_CMD_CWJAP, out, n );
}

int32_t At_recv_connectToAp( AtNetwork * const network )
{
  int32_t res;
  char buff[ 32 + 1 ];
  uint32_t arg;
  
  At_lock();
  
  arg = 0;
  do
  {
    res = At_getRespArg( buff, sizeof( buff ) );
    if( res < 0 )
    {
      break;
    }
    if( network == NULL )
    {
      res = ( buff[ 0 ] - '0' );
      At_unlock();
      return res;
    }
    switch( arg )
    {
    case 0:
      At_stringToString( buff, network->ssid );
      break;
    case 1:
      At_stringToMac( buff, network->bssid );
      break;
    case 2:
      network->chl = strtoul( buff, NULL, 10 );
      break;
    case 3:
      network->rssi = strtoll( buff, NULL, 10 );
      break;
    default:
      break;
    }
    arg++;
  }
  while( res < 2 );
  
  if( res == 3 )
  {
    res = 0;
  }
  else if( res == 2 )
  {
    res = 1;
  }
  
  At_unlock();
  
  return res;
}

int32_t At_send_disconnectFromAp( void )
{
  char out[ 11 ];
  uint32_t n;
  
  n = At_cmd_prepare( AT_CMD_CWQAP, AT_CMD_EXECUTE, out );
  return At_cmd_send( AT_CMD_CWQAP, out, n );
}

int32_t At_send_reconnToApOpt( uint16_t interval_s, uint16_t repeat_nb )
{
  char out[ 27 ];
  uint32_t n;
  
  n = At_cmd_prepare( AT_CMD_CWRECONNCFG, AT_CMD_SET, out );
  n += sprintf( &out[ n ], "%d,%d", interval_s, repeat_nb );
  return At_cmd_send( AT_CMD_CWRECONNCFG, out, n );
}

int32_t At_send_scanOpt( uint16_t printOpt, int8_t rssiFilter )
{
  char out[ 26 ];
  uint32_t n;
  
  n = At_cmd_prepare( AT_CMD_CWLAPOPT, AT_CMD_SET, out );
  n += sprintf( &out[ n ], "1,%d,%d" , printOpt, rssiFilter );  // The first parameter is 1, meaning that the result of the command AT+CWLAP will be ordered according to RSSI;
  return At_cmd_send( AT_CMD_CWLAPOPT, out, n );
}

int32_t At_send_scanAp( char *ssid )
{
  char out[ 14 + 32 ];
  uint32_t n;
  
  if( ssid != NULL )
  {
    n = At_cmd_prepare( AT_CMD_CWLAP, AT_CMD_SET, out );
    n += sprintf( &out[ n ], "\"%s\"", ssid );
  }
  else
  {
    n = At_cmd_prepare( AT_CMD_CWLAP, AT_CMD_EXECUTE, out );
  }
  return At_cmd_send( AT_CMD_CWLAP, out, n );
}

int32_t At_recv_scanAp( uint16_t printOpt, AtNetwork * const network )
{
  char buff[ 32 + 1 ];
  int32_t res;
  uint16_t arg;
  
  At_lock();
  
  arg = 0;
  do
  {
    res = At_getRespArg( buff, sizeof( buff ) );
    if( res < 0 )
      break;
    do
    {
      if( printOpt & ( 1U << arg ) )
      {
        break;
      }
      arg++;
    }
    while( arg < AT_SCAN_MAX_OPT_NUM );
    switch( ( 1U << arg ) )
    {
    case AT_SCAN_OPT_SHOW_ECV:
      network->ecn = strtoul( buff, NULL, 10 );
      break;
    case AT_SCAN_OPT_SHOW_SSID:
      At_stringToString( buff, network->ssid );
      break;
    case AT_SCAN_OPT_SHOW_RSSI:
      network->rssi = strtol( buff, NULL, 10 );
      break;
    case AT_SCAN_OPT_SHOW_MAC:
      At_stringToMac( buff, network->bssid );
      break;
    case AT_SCAN_OPT_SHOW_CHANNEL:
      network->chl = strtoul( buff, NULL, 10 );
      break;
    }
    arg++;
  } 
  while( res < 2 );
  
  if( res == 3 )
  {
    res = 0;
  }
  else if( res == 2 )
  {
    res = 1;
  }
  
  At_unlock();
  
  return res;
}

int32_t At_send_createConnection( uint8_t id, AtLinkType type, uint32_t ip, uint16_t port )
{
  char out[ 46 ];
  uint32_t n;
  
  n = At_cmd_prepare( AT_CMD_CIPSTART, AT_CMD_SET, out );
  if( type == AT_LINK_UDP )
  {
    n += sprintf( &out[ n ], "%hhu,\"UDP\",\"%hhu.%hhu.%hhu.%hhu\",%hu", id, IP_ADDRESS( ip ), port);
  }
  else
  {
    n += sprintf( &out[ n ], "%hhu,\"TCP\",\"%hhu.%hhu.%hhu.%hhu\",%hu", id, IP_ADDRESS( ip ), port);
  }
  return At_cmd_send( AT_CMD_CIPSTART, out, n );
}

int32_t At_recv_linkId( void )
{
  int32_t res, linkId;
  char buff[ 2 ];
  
  At_lock();
  
  linkId = -1;
  do
  {
    res = At_getRespArg( buff, sizeof( buff ) );
    if( res == 0 )
    {
      linkId = ( buff[ 0 ] - '0' );
    }
  }
  while( res < 2 );
  
  At_unlock();
  
  return linkId;
}

int32_t At_recv_linkInfo( AtLinkInfo *info )
{
  int32_t res;
  char buff[ 18 ];
  uint32_t arg;
  
  At_lock();
  
  arg = 0;
  do
  {
    res = At_getRespArg( buff, sizeof( buff ) );
    switch( arg )
    {
    case 0:
      info->status = buff[ 0 ] - '0';
      break;
    case 1:
      info->id = buff[ 0 ] - '0';
      break;
    case 2:
      info->type = At_stringToLinkTypeCode( buff );
      break;
    case 3:
      info->terminal = buff[ 0 ] - '0';
      break;
    case 4:
      info->r_ip = At_stringToIp( buff );
      break;
    case 5:
      info->r_port = strtoul( buff, NULL, 10 );
      break;
    case 6:
      info->l_port = strtoul( buff, NULL, 10 );
      break;
    }
    arg++;
  }
  while( res < 2 );
  
  if( res == 3 )
  {
    res = 0;
  }
  else if( res == 2 )
  {
    res = 1;
  }
  
  At_unlock();
  
  return res;
}

int32_t At_send_closeConnection( uint8_t id )
{
  char out[ 16 ];
  uint32_t n;
  
  n = At_cmd_prepare( AT_CMD_CIPCLOSE, AT_CMD_SET, out );
  n += sprintf( &out[ n ], "%d", id );
  return At_cmd_send( AT_CMD_CIPCLOSE, out, n );
}

int32_t At_send_data( uint8_t id, uint16_t len, uint32_t *ip, uint16_t *port )
{
  char out[ 20 ];
  uint32_t n;
  
  if( len > AT_SEND_LONG_MSG_BYTESIZE )
    n = At_cmd_prepare( AT_CMD_CIPSENDL, AT_CMD_SET, out );
  else
    n = At_cmd_prepare( AT_CMD_CIPSEND, AT_CMD_SET, out );
  n += sprintf( &out[ n ], "%d,%d", id, len );
  if( ( ip != NULL ) && ( port != NULL ) )
  {
    n += sprintf( &out[ n ], ",\"%d.%d.%d.%d,%d\"", IP_ADDRESS( *ip ), *port );
  }
  return At_cmd_send( AT_CMD_CIPSEND, out, n );
}

int32_t At_send_server( AtCmdType cmdType, AtServerMode serverMode, uint16_t port, AtServerType serverType, uint8_t caEnable )
{
  char out[ 33 ];
  uint32_t n;
  
  n = At_cmd_prepare( AT_CMD_CIPSERVER, cmdType, out );
  if( cmdType == AT_CMD_SET )
  {
    switch( serverMode )
    {
    case AT_SERVER_CREATE:
      n += sprintf( &out[ n ], "1,%hu,\"%s\",%d", port, At_serverType[ serverType ], ( caEnable == 0 ) ? 0 : 1 );
      break;
    case AT_SERVER_SHUTDOWN_AND_KEEP:
      n += sprintf( &out[ n ], "0,0" );
      break;
    case AT_SERVER_SHUTDOWN_AND_CLOSE:
      n += sprintf( &out[ n ], "0,1" );
      break;
    }
  }
  return At_cmd_send( AT_CMD_CIPSERVER, out, n );
}

int32_t At_send_serverMaxConn( AtCmdType type, uint8_t num )
{
  char out[ 24 ];
  uint32_t n;
  
  n = At_cmd_prepare( AT_CMD_CIPSERVERMAXCONN, type, out );
  if( type == AT_CMD_SET )
  {
    n += sprintf( &out[ n ], "%d", num );
  }
  return At_cmd_send( AT_CMD_CIPSERVERMAXCONN, out, n );
}

int32_t At_recv_ipd( uint8_t *id, uint16_t *len, uint32_t *ip, uint16_t *port )
{
  int32_t res;
  char buff[ 18 ];
  uint32_t arg;
  
  At_lock();
  
  arg = 0;
  do
  {
    res = At_getRespArg( buff, sizeof( buff ) );
    
    if( res < 0 )
      break;
    
    switch( arg )
    {
    case 0:
      if( id != NULL )
        *id = buff[ 0 ] - '0';
      break;
    case 1:
      if( len != NULL )
        *len = strtoul( buff, NULL, 10 );
      break;
    case 2:
      if( ip != NULL )
        *ip = At_stringToIp( buff );
      break;
    case 3:
      if( port != NULL )
        *port = strtoul( buff, NULL, 10 );
      break;
    default:
      break;
    }
    arg++;
    if( res == 1 )
    {
      res = 0;
      break;
    }
  }
  while( res >= 0 );
  
  At_unlock();
  
  return res;
}

int32_t At_send_update( uint8_t ota, const char *version, uint8_t nonblocking )
{
  char out[ 35 ];
  uint32_t n;
  
  n = At_cmd_prepare( AT_CMD_CIUPDATE, AT_CMD_SET, out );
  n += sprintf( &out[ n ], "%d", ota == 0 ? 0 : 1 );
  if( version != NULL )
  {
    n += sprintf( &out[ n ], ",\"%s\"", version );
  }
  if( nonblocking != 0 )
  {
    if( version != NULL )
    {
      n += sprintf( &out[ n ], ",,1" );
    }
    else
    {
      n += sprintf( &out[ n ], ",,,1" );
    }
  }
  return At_cmd_send( AT_CMD_CIUPDATE, out, n );
}

int32_t At_recv_update( void )
{
  char buff[ 4 ];
  int32_t res, state;
  
  At_lock();
  
  state = -1;
  do
  {
    res = At_getRespArg( buff, sizeof( buff ) );
    if( res < 0 )
      break;
    state = buff[ 0 ] - '0';
  }
  while( res < 2 );
  
  At_unlock();
  
  return state;
}

int32_t At_send_restore( void )
{
  char out[ 13 ];
  uint32_t n;
  
  n = At_cmd_prepare( AT_CMD_RESTORE, AT_CMD_EXECUTE, out );
  return At_cmd_send( AT_CMD_RESTORE, out, n );
}

int32_t At_send_sysflash( AtCmdType type, uint8_t op, const char *partition, uint32_t offset, uint32_t len )
{
  char out[ 128 ];
  uint32_t n;
  
  n = At_cmd_prepare( AT_CMD_SYSFLASH, type, out );
  if( type == AT_CMD_SET )
  {
    if (len == AT_FLASH_CLEAR_ALL)
      n += sprintf( &out[n], "%u,\"%s\"", op, partition);
    else
      n += sprintf( &out[n], "%u,\"%s\",%u,%u", op, partition, offset, len );
  }
  return At_cmd_send( AT_CMD_SYSFLASH, out, n );
}

static AtLinkType At_stringToLinkTypeCode( char *buff )
{
  uint32_t size;
  AtLinkType result;
  
  result = AT_LINK_UNKNOWN;
  size = ( ( sizeof( At_linkType ) / ( sizeof( At_linkType[ 0 ] ) ) ) );
  for( uint32_t i = 0; i < size; i++ )
  {
    if( strncmp( &buff[ 1 ], At_linkType[ i ], strlen(At_linkType[ i ] ) ) == 0 )
    {
      return ( AtLinkType )i;
    }
  }
  return result;
}

/**
 *  @return -2, buff too small
 *  @return -1, rx buffer empty
 *  @return 0, find argument and there is another argument (,)
 *  @return 1, find argument and there is another argument (:)
 *  @return 2, find argument and there is another line (\r\n+)
 *  @return 3, find argument and there is no new line (\r\nOK)
**/
static int32_t At_getRespArg( char *buff, uint32_t size )
{
  char triggers[] = { ',', ':', '\r' };
  uint8_t str, c, match;
  uint32_t i, t, tnum;
  int32_t res;
  
  c = Stream_getByte( &parser->cmdRespStream );
  if( c == '+' )
  {
    do
    {
      res = Stream_read( &parser->cmdRespStream, &c, 1 );
      if( res == 0 )
      {
        return -1;
      }
      if( c == ',' )
      {
        /* Handle +IPD resp */
        break;
      }
    }
    while( c != ':' );
  }
  
  i = 0;
  tnum = ( sizeof( triggers ) / sizeof( triggers[ 0 ] ) );
  str = 0;
  match = 0;
  
  do
  {
    if( i == size )
    {
      return -2;
    }
    Stream_read( &parser->cmdRespStream, &c, 1 );
    
    if( c == '"' )
    {
      str ^= 1U;
    }
    
    if( ( str == 0U ) && ( ( c == '(' ) || ( c == ')' ) ) )
    {
      // Ignore the parentheses outside the string
    }
    else
    {
      if( str == 0 )
      {
        for( t = 0; t < tnum; t++ )
        {
          if( c == triggers[ t ] )
          {
            c = '\0';
            match = 1;
            break;
          }
        }
      }
      buff[ i ] = c;
      i++;
    }
  }
  while( match == 0 );
  
  if( t == 2 )
  {
    Stream_flush( &parser->cmdRespStream, 1 );
    if( Stream_getCount( &parser->cmdRespStream ) > 0 )
    {
      c = Stream_getByte( &parser->cmdRespStream );
      if( c != '+' )
      {
        t = 3;
      }
    }
    else
    {
      t = 3;
    }
  }
  
  return t;
}

static void At_stringToMac( char *buff, uint8_t mac[ 6 ] )
{
  sscanf( buff, "\"%hhx:%hhx:%hhx:%hhx:%hhx:%hhx\"", &mac[ 5 ], &mac[ 4 ], &mac[ 3 ], &mac[ 2 ], &mac[ 1 ], &mac[ 0 ] );
}

static uint32_t At_stringToIp( char *buff )
{
  uint8_t str[ 4 ];
  sscanf( buff, "\"%hhu.%hhu.%hhu.%hhu\"", &str[ 3 ], &str[ 2 ], &str[ 1 ], &str[ 0 ] );
  return IP( str[ 3 ], str[ 2 ], str[ 1 ], str[ 0 ] );
}

static void At_stringToString( char *buff, char *dst )
{
  uint32_t size;
  
  size = strlen( buff ) - 2;
  strncpy( dst, &buff[ 1 ], size );
  dst[ size ] = '\0';
}
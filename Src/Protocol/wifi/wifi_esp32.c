#include <string.h>
#include <stdio.h>
#include "wifi_esp32.h"
#include "stm32f4xx_hal.h"
#include "esp32.h"

typedef struct
{
  WifiEventCallback callback;
  uint32_t flags;
  uint8_t connIds;
  osThreadId_t threadId;
  osEventFlagsId_t eventId;
  osMutexId_t mutexId;
  NetworkScanInfo *scan;
  uint32_t maxScanCount;
  uint32_t scanCount;
} WifiEsp32Control;

static char moduleInfo[ 180 ];
static Socket sockets[ WIFI_MAX_SOCKETS ];
static uint8_t socketsMem[ WIFI_MAX_SOCKETS ][ WIFI_SOCKET_BUFFER_BYTESIZE ];
static WifiEsp32Control wifiEsp32ctr;
#define pWifiCtr ( &wifiEsp32ctr )

const osThreadAttr_t wifiTask_attributes = {
  .name = "WifiTask",
  .stack_size = 128 * 5,
  //.priority = (osPriority_t) osPriorityNormal,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private const variables */
static const AtUartConfig wifiUartConfig = {
  .baudrate = WIFI_UART_DEF_BAUDRATE,
  .data_bits = WIFI_UART_DEF_DATABITS,
  .stop_bits = WIFI_UART_DEF_STOPBITS,
  .parity = WIFI_UART_DEF_PARITY,
  .flow_control = WIFI_UART_DEF_FLOWCONTROL
};

/* Interface functions */
static int32_t WifiEsp32_init( WifiEventCallback eventCallback );
static int32_t WifiEsp32_turnOn( void );
static int32_t WifiEsp32_turnOff( void );
static int32_t WifiEsp32_getVersion( Version * const version );
static int32_t WifiEsp32_enable( WifiInterface interface, WifiConfig * const config );
static int32_t WifiEsp32_reconnectionOpt( uint16_t interval_s, uint16_t repeat_nb );
static int32_t WifiEsp32_isConnect( void );
static int32_t WifiEsp32_getNetInfo( NetworkInfo * const info );
static int32_t WifiEsp32_getAddress( WifiInterface interface, NetworkAddress *address );
static int32_t WifiEsp32_disable( WifiInterface interface );
static int32_t WifiEsp32_scan( NetworkScanInfo scanInfo[], uint32_t maxScanInfo, char *ssid, int8_t rssi );
static int32_t WifiEsp32_socket( uint8_t type, uint8_t protocol );
static int32_t WifiEsp32_connect( int32_t socket, uint32_t ip, uint16_t port );
static int32_t WifiEsp32_send( int32_t socket, const void *buff, uint32_t len );
static int32_t WifiEsp32_sendTo( int32_t socket, const void *buff, uint32_t len, uint32_t *ip, uint16_t *port );
static int32_t WifiEsp32_bind( int32_t socket, uint32_t ip, uint16_t port );
static int32_t WifiEsp32_listen( int32_t socket, int32_t backlog );
static int32_t WifiEsp32_accept( int32_t socket, uint32_t *ip, uint16_t *port );
static int32_t WifiEsp32_recv( int32_t socket, void *buff, uint32_t len );
static int32_t WifiEsp32_recvFrom( int32_t socket, void *buff, uint32_t len, uint32_t *ip, uint16_t *port );
static int32_t WifiEsp32_close( int32_t socket );
static int32_t WifiEsp32_setSocketOpt( int32_t socket, int32_t opt, const void *value, uint32_t len );
static int32_t WifiEsp32_getSocketOpt( int32_t socket, int32_t opt, const void *value, uint32_t *len );
static int32_t WifiEsp32_update( Version * const version );
static int32_t WifiEsp32_restore( void );

/* Support function */
static int32_t WifiEsp32_lock( void );
static int32_t WifiEsp32_unlock( int32_t res );
static int32_t WifiEsp32_setupCommunication( void );
static int32_t WifiEsp32_setEcho( uint8_t enable );
static int32_t WifiEsp32_setupMessages( void );
static int32_t WifiEsp32_enableMultiConnection( void );
static int32_t WifiEsp32_disableStorage( void );
static int32_t WifiEsp32_getCurrentMac( WifiInterface interface, uint8_t mac[ 6 ] );
static int32_t WifiEsp32_getCurrentAddress( WifiInterface interface, uint32_t *ip, uint32_t *gateway, uint32_t *netmask );
static uint8_t WifiEsp32_getSocketId( void );
static void WifiEsp32_acceptId( uint8_t id );
static void WifiEsp32_freeId( uint8_t id );

static void WifiTask( void *argument )
{
  int32_t status;
  uint32_t timeout;
  uint32_t flags;
  
  status = At_init();
  if( status < 0 )
  {
    osThreadTerminate( osThreadGetId() );
  }
  
  //timeout = pdMS_TO_TICKS( WIFI_THREAD_POOLING_TIMEOUT );
  timeout = osWaitForever;
  
  for(;;)
  {
    flags = osThreadFlagsWait( WIFI_TASK_FLAGS, osFlagsWaitAny, timeout );
    if( ( flags & osFlagsError ) == 0 )
    {
      if( flags & WIFI_TASK_TERMINATE )
      {
        At_deinit();
        osThreadTerminate( osThreadGetId() );
      }
    }
    At_parser();
  }
}

static int32_t WifiEsp32_init( WifiEventCallback eventCallback )
{
  pWifiCtr->callback = eventCallback;
  
  if( pWifiCtr->flags & WIFI_FLAG_INIT )
    return wifiOK;
  
  pWifiCtr->flags = 0;
  pWifiCtr->eventId = osEventFlagsNew( NULL );
  if( pWifiCtr->eventId == NULL )
  {
    return wifiErrorResource;
  }
  
  pWifiCtr->threadId = osThreadNew( WifiTask, NULL, &wifiTask_attributes );
  if( pWifiCtr->threadId == NULL )
  {
    return wifiErrorResource;
  }
  
  pWifiCtr->mutexId = osMutexNew( NULL );
  if( pWifiCtr->mutexId == NULL )
  {
    return wifiErrorResource;
  }
  
  for ( uint32_t n = 0U; n < WIFI_MAX_SOCKETS; n++) {
    sockets[n].state = SOCKET_STATE_FREE;
  }
  pWifiCtr->connIds = 0U;
  
  pWifiCtr->flags |= WIFI_FLAG_INIT;
  
  pWifiCtr->scan = NULL;
  pWifiCtr->maxScanCount = 0;
  pWifiCtr->scanCount = 0;
  
  return wifiOK;
}

void At_notify( AtEvent event, void *arg )
{
  static volatile uint8_t rx_id;
  static volatile uint16_t rx_len;
  int32_t status;
  WifiStation station;
  AtLinkInfo linkInfo;
  uint8_t n;
  Socket *socket;
  uint16_t *u16, len;
  uint32_t *u32, addr;
  uint8_t link_id;
  AtNetwork network;
  
  switch( event )
  {
  case AT_NOTIFY_DATA_RX:
    osThreadFlagsSet( pWifiCtr->threadId, WIFI_TASK_RX_DATA );
    break;
  case AT_NOTIFY_TX_DONE:
    osEventFlagsSet( pWifiCtr->eventId, WIFI_EVENT_TX_DONE );
    break;
  case AT_NOTIFY_READY:
  case AT_NOTIFY_RESPONSE:
    osEventFlagsSet( pWifiCtr->eventId, WIFI_EVENT_RESPONSE );
    break;
  case AT_NOTIFY_CONNECTED:
    pWifiCtr->flags |= ( WIFI_FLAG_STATION_CONNECTED | WIFI_FLAG_STATION_ACTIVE );
    break;
  case AT_NOTIFY_GOT_IP:
    pWifiCtr->flags |= WIFI_FLAG_STATION_GOT_IP;
    break;
  case AT_NOTIFY_DISCONNECT:
    pWifiCtr->flags &= ~( WIFI_FLAG_STATION_CONNECTED | WIFI_FLAG_STATION_GOT_IP );
    break;
  case AT_NOTIFY_STA_CONNECTED:
    station.ip = 0;
    At_recv_mac( station.mac );
    if( pWifiCtr->callback != NULL )
      pWifiCtr->callback( WIFI_EVENT_STATION_CONNECTED, &station );
    break;
  case AT_NOTIFY_DIST_STA_IP:
    At_recv_macAndIp( station.mac, &station.ip );
    if( pWifiCtr->callback != NULL )
      pWifiCtr->callback( WIFI_EVENT_STATION_GOT_IP, &station );
    break;
  case AT_NOTIFY_STA_DISCONNECTED:
    station.ip = 0;
    At_recv_mac( station.mac );
    if( pWifiCtr->callback != NULL )
      pWifiCtr->callback( WIFI_EVENT_STATION_DISCONNECTED, &station );
    break;
  case AT_NOTIFY_LINK_CONNECT:
    if( ( pWifiCtr->flags & WIFI_FLAGS_CONN_INFO_ACTIVE ) != 0 )
    {
      // When a client connect to this server, received +LINK_CONN response
      status = At_recv_linkInfo( &linkInfo );
      if( status == 0 )
      {
        WifiEsp32_acceptId( linkInfo.id );
        if( linkInfo.terminal == 1U )
        {
          // Client link connect to server
          // Pick server from sockets
          for( n = 0; n < WIFI_MAX_SOCKETS; n++ )
          {
            if( sockets[ n ].state == SOCKET_STATE_SERVER )
            {
              socket = &sockets[ n ];
              do
              {
                // Pick first socket in backlog server that listens for a connection
                n = sockets[ n ].backlog;
                if( sockets[ n ].state == SOCKET_STATE_LISTEN )
                {
                  sockets[ n ].id = linkInfo.id;
                  sockets[ n ].accepted = false;
                  sockets[ n ].state = SOCKET_STATE_CONNECTED;
                  sockets[ n ].l_port = linkInfo.l_port;
                  sockets[ n ].r_port = linkInfo.r_port;
                  sockets[ n ].r_ip = linkInfo.r_ip;
                  break;
                }
              }
              while( sockets[ n ].backlog != socket->backlog );
              break;
            }
          }
          if( n != WIFI_MAX_SOCKETS )
          {
            // Notify connection
            osEventFlagsSet( pWifiCtr->eventId, WIFI_EVENT_WAIT_CONN_ACCEPT );
          }
        }
        else
        {
          // Client link connect
          // Pick socket that waiting client connection
          for( n = 0; n < WIFI_MAX_SOCKETS; n++ )
          {
            if( sockets[ n ].state == SOCKET_STATE_CONNECTREQ )
            {
              if( sockets[ n ].id == linkInfo.id )
              {
                sockets[ n ].state = SOCKET_STATE_CONNECTED;
                sockets[ n ].l_port = linkInfo.l_port;
                sockets[ n ].r_port = linkInfo.r_port;
                sockets[ n ].r_ip = linkInfo.r_ip;
                break;
              }
            }
          }
          if( n != WIFI_MAX_SOCKETS )
          {
            // Notify connection
            osEventFlagsSet( pWifiCtr->eventId, WIFI_EVENT_WAIT_CONN_OPEN( n ) );
          }
        }
      }
    }
    else
    {
      // When a client connect to this server, received <LINK_ID>,CONNECT response
      linkInfo.id = At_recv_linkId();
      WifiEsp32_acceptId( linkInfo.id );
      for( n = 0; n < WIFI_MAX_SOCKETS; n++ )
      {
        if( sockets[ n ].state == SOCKET_STATE_CONNECTREQ )
        {
          if( sockets[ n ].id == linkInfo.id )
          {
            sockets[ n ].state = SOCKET_STATE_CONNECTED;
            break;
          }  
        }
      }
      if( n != WIFI_MAX_SOCKETS )
      {
        // Notify connection
        osEventFlagsSet( pWifiCtr->eventId, WIFI_EVENT_WAIT_CONN_OPEN( n ) );
      }
      else
      {
        // No client is waiting connection. Create link connection for server.
        for( n = 0; n < WIFI_MAX_SOCKETS; n++ )
        {
          if( sockets[ n ].state == SOCKET_STATE_SERVER )
          {
            socket = &sockets[ n ];
            do
            {
              n = sockets[ n ].backlog;
              if( sockets[ n ].state == SOCKET_STATE_LISTEN )
              {
                sockets[ n ].id = linkInfo.id;
                sockets[ n ].state = SOCKET_STATE_CONNECTED;
                break;
              }
            }
            while( sockets[ n ].backlog != socket->backlog );
            break;
          }
        }
        if( n != WIFI_MAX_SOCKETS )
        {
          osEventFlagsSet (pWifiCtr->eventId, WIFI_EVENT_WAIT_CONN_ACCEPT);
        }
      }
    }
    break;
  case AT_NOTIFY_LINK_CLOSED:
    linkInfo.id = At_recv_linkId();
    WifiEsp32_freeId( linkInfo.id );
    for( n = 0; n < WIFI_MAX_SOCKETS; n++ )
    {
      if( sockets[ n ].id == linkInfo.id )
      {
        sockets[ n ].id = CONN_ID_INVALID;
        if( sockets[ n ].backlog == SOCKET_INVALID )
        {
          if( sockets[ n ].state == SOCKET_STATE_CLOSING )
          {
            sockets[ n ].state = SOCKET_STATE_FREE;
          }
          else
          {
            sockets[ n ].state = SOCKET_STATE_CLOSED;
          }
        }
        else
        {
          if( sockets[ n ].state == SOCKET_STATE_CLOSING ||
              ( sockets[ n ].state == SOCKET_STATE_CONNECTED && !sockets[ n ].accepted ) )
          {
            sockets[ n ].state = SOCKET_STATE_LISTEN;
          }
          else
          {
            sockets[ n ].state = SOCKET_STATE_CLOSED;
          }
        }
        break;
      }
    }
    if( n != WIFI_MAX_SOCKETS )
    {
      osEventFlagsSet( pWifiCtr->eventId, WIFI_EVENT_WAIT_CONN_CLOSE( n ) );
    }
    break;
  case AT_NOTIFY_REQ_SEND:
    osEventFlagsSet( pWifiCtr->eventId, WIFI_EVENT_TX_WAIT );
    break;
  case AT_NOTIFY_LINK_RX_INIT:
    u16 = ( uint16_t* )arg;
    
    *u16 = 0U;
    
    rx_id = SOCKET_INVALID;
    rx_len = 0;
    
    status = At_recv_ipd( &link_id, &len, NULL, NULL );
    if( status == 0 )
    {
      for( n = 0; n < WIFI_MAX_SOCKETS; n++ )
      {
        if( sockets[ n ].state == SOCKET_STATE_CONNECTED )
          if( sockets[ n ].id == link_id )
            break;
      }
      if( n != WIFI_MAX_SOCKETS )
      {
        socket = &sockets[ n ];
        
        rx_len = Stream_getAvailable( &socket->rx_stream );
        
        if( rx_len >= ( len + 2U ) )
        {
          rx_id = n;
          Stream_write( &socket->rx_stream, ( uint8_t* )&len, 2 );
          *u16 = len;
        }
      }
      rx_len = len;
    }
    break;
  case AT_NOTIFY_LINK_RX_DATA:
    u32 = ( uint32_t* )arg;
    addr = *u32;
    if( rx_id != SOCKET_INVALID )
    {
      socket = &sockets[ rx_id ];
      len = Stream_copy( &socket->rx_stream, ( Stream* )addr, rx_len );
    }
    else
    {
      len = Stream_flush( ( Stream* )addr, rx_len );
    }
    
    rx_len -= len;
    *u32 = rx_len;
    
    if( rx_id != SOCKET_INVALID )
    {
      if( rx_len == 0U )
      {
        osEventFlagsSet( pWifiCtr->eventId, WIFI_EVENT_WAIT_CONN_RX( rx_id ) );
      }
    }
    break;
  case AT_NOTIFY_UPDATE:
    status = At_recv_update();
    if( pWifiCtr->callback != NULL )
      pWifiCtr->callback( WIFI_EVENT_UPDATE, &status );
    break;
  case AT_NOTIFY_SCAN_RESULT:
    status = At_recv_scanAp( ( AT_SCAN_OPT_SHOW_ECV | AT_SCAN_OPT_SHOW_SSID | AT_SCAN_OPT_SHOW_RSSI | AT_SCAN_OPT_SHOW_MAC ), &network );
    if( status == 0 || status == 1 )
    {
      if( pWifiCtr->scan != NULL )
      {
        if( pWifiCtr->scanCount != pWifiCtr->maxScanCount )
        {
          NetworkScanInfo info;
        
          info.ecn = network.ecn;
          strncpy( info.ssid, network.ssid, NETWORK_INFO_SSID_BYTESIZE );
          info.rssi = network.rssi;
          memcpy( info.bssid, network.bssid, NETWORK_INFO_MAC_BYTESIZE );
          
          memcpy( &pWifiCtr->scan[pWifiCtr->scanCount], &info, sizeof(NetworkScanInfo) );
          pWifiCtr->scanCount += 1;
        }
      }
    }
    break;
  default:
    break;
  }
}

static int32_t WifiEsp32_wait( uint32_t event, uint32_t timeout_ms )
{
  uint32_t flags;
  
  if( timeout_ms == 0 )
    timeout_ms = osWaitForever;
  
  flags = osEventFlagsWait( pWifiCtr->eventId, event, osFlagsWaitAny, pdMS_TO_TICKS( timeout_ms ) );
  if( ( flags & osFlagsError ) == 0 )
  {
    return wifiOK;
  }
  else
  {
    if( flags == osFlagsErrorTimeout )
    {
      return wifiErrorTimeout;
    }
    else
    {
      return wifiError;
    }
  }
}

static int32_t WifiEsp32_lock( void )
{
  if( osMutexAcquire( pWifiCtr->mutexId, osWaitForever ) == osOK )
  {
    return wifiOK;
  }
  return wifiError;
}

static int32_t WifiEsp32_unlock( int32_t res )
{
  if( osMutexRelease( pWifiCtr->mutexId ) == osOK )
  {
    return res;
  }
  return wifiError;
}

static int32_t WifiEsp32_turnOn( void )
{
  int32_t res;
  
  if( pWifiCtr->flags & WIFI_FLAG_POWER )
    return wifiOK;
  
  res = WifiEsp32_lock();
  if( res != 0 )
    return wifiError;
  
  HAL_GPIO_WritePin( WIFI_ENABLE_GPIO_Port, WIFI_ENABLE_Pin, GPIO_PIN_SET );
  res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, osWaitForever );
   
  pWifiCtr->flags |= WIFI_FLAG_POWER;
  res = WifiEsp32_setupCommunication();
  
  if( res != AT_OK )
    return wifiError;
  
  res = WifiEsp32_setEcho( 0 );
  
  if( res != AT_OK )
    return wifiError;
  
  res = WifiEsp32_setupMessages();
  
  if( res != AT_OK )
    return wifiError;
  
  res = WifiEsp32_enableMultiConnection();
  
  if( res != AT_OK )
    return wifiError;
  
  res = WifiEsp32_disableStorage();
  
  if( res != AT_OK )
    return wifiError;
  
  res = WifiEsp32_unlock( res );
  
  return res;
}

static int32_t WifiEsp32_setupCommunication( void )
{
  int32_t res;
  AtUartConfig *config;
  
  config = ( AtUartConfig* )&wifiUartConfig;
  // Setup current
  res = At_send_currentUart( AT_CMD_SET, config );
  if( res == 0)
  {
    res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
    if( res == 0 )
    {
      res = At_recv_response();
      if( res == AT_OK )
      {
        // Test
        res = At_send_test();
        if( res == 0 )
        {
          res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
          if( res == 0 )
          {
            res = At_recv_response();
          }
        }
      }
    }
  }
  return res;
}

static int32_t WifiEsp32_setEcho( uint8_t enable )
{
  int32_t res;
  
  res = At_send_echo( enable );
  if( res == 0 )
  {
    res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
    if( res == 0 )
    {
      res = At_recv_response();
    }
  }
  return res;
}

static int32_t WifiEsp32_setupMessages( void )
{
  int32_t res;
  
  res = At_send_sysmsg( AT_CMD_SET, 3 );
  if( res == 0)
  {
    res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
    if( res == 0 )
    {
      res = At_recv_response();
      if( res == AT_OK )
        pWifiCtr->flags |= WIFI_FLAGS_CONN_INFO_ACTIVE;
      else
        pWifiCtr->flags &= ~WIFI_FLAGS_CONN_INFO_ACTIVE;
    }
  }
  return res;
}

static int32_t WifiEsp32_getCurrentMac( WifiInterface interface, uint8_t mac[ 6 ] )
{
  int32_t res;
  
  if( interface == WIFI_INTERFACE_AP )
  {
    res = At_send_macAp( AT_CMD_QUERY, NULL );
  }
  else
  {
    res = At_send_macStation( AT_CMD_QUERY, NULL );
  }
  if( res == 0 )
  {
    res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
    if( res == 0 )
    {
      res = At_recv_response();
      if( res == AT_OK )
      {
        res = At_recv_mac( mac );
      }
    }
  }
  return res;
}

static int32_t WifiEsp32_getCurrentAddress( WifiInterface interface, uint32_t *ip, uint32_t *gateway, uint32_t *netmask )
{
  int32_t res;
  
  if( interface == WIFI_INTERFACE_AP )
  {
    res = At_send_addressAp( AT_CMD_QUERY, NULL, NULL, NULL );
  }
  else
  {
    res = At_send_addressStation( AT_CMD_QUERY, NULL, NULL, NULL );
  }
  if( res == 0 )
  {
    res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
    if( res == 0 )
    {
      res = At_recv_response();
      if( res == AT_OK )
      {
        At_recv_ip( ip );
        At_recv_ip( gateway );
        At_recv_ip( netmask );
      }
    }
  }
  return res;
}

static int32_t WifiEsp32_enableMultiConnection( void )
{
  int32_t res;
  
  res = At_send_multiConnection( AT_CMD_SET, 1 );
  if( res == 0 )
  {
    res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
    if( res == 0 )
    {
      res = At_recv_response();
    }
  }
  return res;
}

static int32_t WifiEsp32_disableStorage( void )
{
  int32_t res;
  
  res = At_send_sysstore( AT_CMD_SET, 0 );
  if( res == 0 )
  {
    res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
    if( res == 0 )
    {
      res = At_recv_response();
    }
  }
  return res;
}

static int32_t WifiEsp32_turnOff( void )
{
  int32_t res;
  
  res = WifiEsp32_lock();
  if( res != 0 )
    return wifiError;
  
  osThreadFlagsSet( pWifiCtr->threadId, WIFI_TASK_TERMINATE );
  
  HAL_GPIO_WritePin(WIFI_ENABLE_GPIO_Port, WIFI_ENABLE_Pin, GPIO_PIN_RESET);
  pWifiCtr->flags &= ~WIFI_FLAG_POWER;
  
  res = WifiEsp32_unlock( res );
  
  return res;
}

static int32_t WifiEsp32_getVersion( Version * const version )
{
  int32_t res;
  
  res = WifiEsp32_lock();
  if( res != 0 )
    return wifiError;
  
  res = At_send_versionInfo();
  if( res == 0 )
  {
    res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
    if( res == 0 )
    {
      At_recv_versionInfo( moduleInfo, sizeof( moduleInfo ) );
      res = sscanf( moduleInfo, "AT version:%hhu.%hhu.%hhu.%hhu(", &version->major, &version->minor, &version->patch, &version->hotfix );
      if( res == 4 )
      {
        res = 0;
      }
    }
  }
  
  res = WifiEsp32_unlock( res );
  
  return res;
}

static int32_t WifiEsp32_enable( WifiInterface interface, WifiConfig * const config )
{
  int32_t res;
  AtMode mode;
  AtApConfig apConfig;
  
  if( config->ssid == NULL )
    return wifiErrorParameter;
  
  if( strlen( config->ssid ) == 0 )
    return wifiErrorParameter;
  
  res = WifiEsp32_lock();
  if( res != 0 )
    return wifiError;
  
  res = At_send_mode( AT_CMD_QUERY, NULL );
  if( res == 0 )
  {
    res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
    if( res == 0 )
    {
      res = At_recv_mode( &mode );
      if( res == 0 )
      {
        // Enable interface
        if( ( mode & ( 1U << interface ) ) == 0 )
        {
          mode |= ( 1U << interface );
          res = At_send_mode( AT_CMD_SET, mode );
          if( res == 0 )
          {
            res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
          }
        }
        // Change configuration
        if( interface == WIFI_INTERFACE_AP )
        {
          // Access point
          if( config->enc == WIFI_ENC_WEP )
          {
            return wifiErrorParameter;
          }
          
          res = At_send_confAp( AT_CMD_QUERY, NULL );
          if( res == 0 )
          {
            res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
            if( res == 0 )
            {
              res = At_recv_response();
              if( res == AT_OK )
              {
                res = At_recv_confAp( &apConfig );
                if( res == 0 )
                {
                  if( ( strcmp( apConfig.ssid, config->ssid ) == 0 ) &&
                      ( strcmp( apConfig.pass, config->pass ) == 0 ) && 
                      ( apConfig.chl == config->channel ) &&
                      ( apConfig.ecn == config->enc ) )
                  {
                    pWifiCtr->flags |= WIFI_FLAG_AP_ACTIVE;
                    res = -1;
                  }
                }
              }
            }
          }
          
          if( res == 0 )
          {
            apConfig.ecn = (uint8_t)config->enc;
            strcpy( apConfig.ssid, ( char* )config->ssid );
            strcpy( apConfig.pass, ( char* )config->pass );
            apConfig.chl = config->channel;
            apConfig.max_conn = WIFI_AP_MAX_CONNECTIONS;
            apConfig.ssid_hidden = 0;
            res = At_send_confAp( AT_CMD_SET, &apConfig );
            if( res == 0 )
            {
              res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_CONFIG_AP_TIMEOUT_MS );
              if( res == 0 )
              {
                res = At_recv_response();
                if( res == AT_OK )
                {
                  pWifiCtr->flags |= WIFI_FLAG_AP_ACTIVE;
                }
              }
            }
            else if( res == -1 )
            {
              res = wifiErrorParameter;
            }
          }
          else
          {
            res = 0;
          }
        }
        else
        {
          // Station
          res = At_send_connectToAp( AT_CMD_SET, config->ssid, config->pass, config->mac );
          if( res == 0 )
          {
            res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_CONNECT_TO_AP_TIMEOUT_MS );
            if( res == 0 )
            {
              res = At_recv_response();
              if( res == AT_ERROR )
              {
                res = At_recv_connectToAp( NULL );
              }
            }
          }
        }
      }
    }
  }
  
  res = WifiEsp32_unlock( res );
  
  return res;
}

static int32_t WifiEsp32_reconnectionOpt( uint16_t interval_s, uint16_t repeat_nb )
{
  int32_t res;
  
  res = WifiEsp32_lock();
  
  res = At_send_reconnToApOpt( interval_s, repeat_nb );
  if( res == 0 )
  {
    res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
    if( res == 0 )
    {
      res = At_recv_response();
    }
  }
  
  res = WifiEsp32_unlock( res );
  
  return res;
}

static int32_t WifiEsp32_isConnect( void )
{
  int32_t res;
  uint32_t flags;
  
  flags = pWifiCtr->flags & ( WIFI_FLAG_STATION_ACTIVE | WIFI_FLAG_STATION_CONNECTED | WIFI_FLAG_STATION_GOT_IP );
  
  if( flags == ( WIFI_FLAG_STATION_ACTIVE | WIFI_FLAG_STATION_CONNECTED | WIFI_FLAG_STATION_GOT_IP ) )
  {
    res = 1U;
  }
  else
  {
    res = 0U;
  }
  
  return res;
}

static int32_t WifiEsp32_getNetInfo( NetworkInfo * const info )
{
  int32_t res = wifiOK;
  AtNetwork network;
  
  res = WifiEsp32_lock();
  
  if( !WifiEsp32_isConnect() )
  {
    res = wifiError;
  }
  
  if( res == wifiOK )
  {
    res = At_send_connectToAp( AT_CMD_QUERY, NULL, NULL, NULL );
    if( res == 0 )
    {
      res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
      if( res == 0 )
      {
        res = At_recv_response();
        if( res == AT_OK )
        {
          res = At_recv_connectToAp( &network );
          if( res == 0 )
          {
            strcpy( info->ssid, network.ssid );
            memcpy( info->bssid, network.bssid, NETWORK_INFO_MAC_BYTESIZE );
            info->channel = network.chl;
            info->rssi = network.rssi;
          }
        }
      }
    }
  }
  
  res = WifiEsp32_unlock( res );
  
  return res;
}

static int32_t WifiEsp32_getAddress( WifiInterface interface, NetworkAddress *address )
{
  int32_t res;
  
  res = WifiEsp32_lock();
  
  res = WifiEsp32_getCurrentMac( interface, address->mac );
  if( res == 0 )
  {
    res = WifiEsp32_getCurrentAddress( interface, &address->ip, &address->gateway, &address->netmask );
  }
  
  res = WifiEsp32_unlock( res );
  
  return res;
}

static int32_t WifiEsp32_disable( WifiInterface interface )
{
  int32_t res;
  AtMode mode;
  
  res = WifiEsp32_lock();
  if( res != 0 )
    return wifiError;
  
  res = At_send_mode( AT_CMD_QUERY, NULL );
  if( res == 0 )
  {
    res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
    if( res == 0 )
    {
      res = At_recv_mode( &mode );
      if( res == 0 )
      {
        // Disconnect from network
        if( interface == WIFI_INTERFACE_STATION )
        {
          if( ( mode == AT_MODE_STATION ) || ( mode == AT_MODE_AP_STATION ) )
          {
            res = At_send_disconnectFromAp();
            if( res == 0 )
            {
              res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
              if( res == 0 )
              {
                res = At_recv_response();
              }
            }
          }
        }
        // Disable interface
        if( ( mode & ( 1U << interface ) ) != 0 )
        {
          mode &= ~( 1U << interface );
          res = At_send_mode( AT_CMD_SET, mode );
          if( res == 0 )
          {
            res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
            if( res == 0 )
            {
              res = At_recv_response();
              if( res == AT_OK )
              {
                if( interface == WIFI_INTERFACE_AP )
                  pWifiCtr->flags &= ~WIFI_FLAG_AP_ACTIVE;
              }
            }
          }
        }
      }
    }
  }
  
  res = WifiEsp32_unlock( res );
  
  return res;
}

static int32_t WifiEsp32_scan( NetworkScanInfo scanInfo[], uint32_t maxScanInfo, char *ssid, int8_t rssi )
{
  int32_t res, count;
  AtMode pastMode, mode;
  
  if( ( scanInfo == NULL ) || ( maxScanInfo == 0 ) )
  {
    return wifiErrorParameter;
  }
  else
  {
    res = WifiEsp32_lock();
    if( res != 0 )
      return wifiError;
    
    for( int i = 0; i < maxScanInfo; i++ )
    {
      memset( scanInfo[ i ].ssid, 0, NETWORK_INFO_SSID_BYTESIZE );
      memset( scanInfo[ i ].bssid, 0, NETWORK_INFO_MAC_BYTESIZE );
      scanInfo[ i ].rssi = 0;
      scanInfo[ i ].ecn = 0;
    }
    
    pWifiCtr->scan = scanInfo;
    pWifiCtr->maxScanCount = maxScanInfo;
    pWifiCtr->scanCount = 0;
    count = 0;
    
    res = At_send_scanOpt( ( AT_SCAN_OPT_SHOW_ECV | AT_SCAN_OPT_SHOW_SSID | AT_SCAN_OPT_SHOW_RSSI | AT_SCAN_OPT_SHOW_MAC ), rssi );
    if( res == 0 )
    {
      res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
      if( res == 0 )
      {
        
        res = At_send_mode( AT_CMD_QUERY, NULL );
        if( res == 0 )
        {
          res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
          if( res == 0 )
          {
            res = At_recv_mode( &mode );
            if( res == 0 )
            {
              pastMode = mode;
              mode |= ( 1U << WIFI_INTERFACE_STATION );
              res = At_send_mode( AT_CMD_SET, mode );
              if( res == 0 )
              {
                res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
              }
            }
          }
        }
        
        if( res == 0)
        {
          res = At_send_scanAp( ssid );
          if( res == 0 )
          {
            res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, osWaitForever );
            if( res == 0 )
            {
              res = At_recv_response();
              if( res == AT_OK )
              {
                count = pWifiCtr->scanCount;
                res = At_send_mode( AT_CMD_SET, pastMode );
                if( res == 0 )
                {
                  res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
                }
              }
            }
          }
          else
          {
            res = -1;
          }
        }
        else
        {
          res = -1;
        }
      }
    }
    
    pWifiCtr->scan = NULL;
    pWifiCtr->maxScanCount = 0;
    pWifiCtr->scanCount = 0;
    
    res = WifiEsp32_unlock( res );
    if( res == 0 )
      res = count;
  }
  return res;
}

static int32_t WifiEsp32_socket( uint8_t type, uint8_t protocol )
{
  int32_t res;
  uint32_t id;
  Socket *socket;
  
  if( ( ( type != SOCKET_TYPE_DATAGRAM ) && ( type != SOCKET_TYPE_STREAM ) ) ||
      ( ( protocol != SOCKET_PROTOCOL_TCP ) && ( protocol != SOCKET_PROTOCOL_UDP ) && ( protocol != SOCKET_PROTOCOL_TLS ) ) )
  {
    return SOCKET_EINVAL;
  }
  
  if( type == SOCKET_TYPE_DATAGRAM )
  {
    if( protocol != SOCKET_PROTOCOL_UDP )
      return SOCKET_EINVAL;
    if( protocol == 0 )
      protocol = SOCKET_PROTOCOL_UDP;
  }
  else
  {
    if( ( protocol != SOCKET_PROTOCOL_TCP ) && ( protocol != SOCKET_PROTOCOL_TLS ) )
      return SOCKET_EINVAL;
    if( protocol == 0 )
      protocol = SOCKET_PROTOCOL_TLS;
  }
  
  res = WifiEsp32_lock();
  if( res != 0 )
    return SOCKET_ERROR;
  
  for( id = 0; id < WIFI_MAX_SOCKETS; id++ )
  {
    if( sockets[ id ].state == SOCKET_STATE_FREE )
    {
      socket = &sockets[ id ];
      break;
    }
  }
  
  if( id == WIFI_MAX_SOCKETS )
    return SOCKET_ESOCK;
  
  memset( socket, 0, sizeof( Socket ) );
  
  socket->state = SOCKET_STATE_CREATED;
  socket->type = type;
  socket->protocol = protocol;
  socket->id = CONN_ID_INVALID;
  socket->server = SOCKET_INVALID;
  socket->backlog = SOCKET_INVALID;
  
  res = id;
  
  res = WifiEsp32_unlock( res );
  
  return res;
}

static int32_t WifiEsp32_connect( int32_t socket, uint32_t ip, uint16_t port )
{
  int32_t res;
  Socket *pSocket;
  AtLinkType type;
  
  if( ( socket < 0 ) || ( socket >= WIFI_MAX_SOCKETS ) )
    return SOCKET_EINVAL;
  
  if( ip == NULL )
    return SOCKET_EINVAL;
  
  if( port == 0 )
    return SOCKET_EINVAL;
  
  res = WifiEsp32_lock();
  if( res != wifiOK )
    return SOCKET_ERROR;
  
  pSocket = &sockets[ socket ];
  
  switch( pSocket->state )
  {
  case SOCKET_STATE_CREATED:
  case SOCKET_STATE_BOUND:
    break;
  case SOCKET_STATE_FREE:
    res = SOCKET_ESOCK;
    break;
  case SOCKET_STATE_LISTEN:
  case SOCKET_STATE_SERVER:
    res = SOCKET_EINVAL;
    break;
  case SOCKET_STATE_CONNECTREQ:
    res = SOCKET_EALREADY;
    break;
  case SOCKET_STATE_CONNECTED:
    res = SOCKET_EISCONN;
    break;
  case SOCKET_STATE_CLOSED:
    res = SOCKET_ETIMEDOUT;
    break;
  default:
    res = SOCKET_ERROR;
    break;
  }
  
  if( res == wifiOK )
  {
    pSocket->state = SOCKET_STATE_CONNECTREQ;
    pSocket->r_ip = ip;
    pSocket->r_port = port;
    pSocket->id = WifiEsp32_getSocketId();
    if( pSocket->protocol == SOCKET_PROTOCOL_UDP )
    {
      type = AT_LINK_UDP;
    }
    else
    {
      type = AT_LINK_TCP;
    }
    res = At_send_createConnection( pSocket->id, type, pSocket->r_ip, pSocket->r_port );
    if( res == 0 )
    {
      res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_CLIENT_CONNECT_TIMEOUT_MS );
      if( res == 0 )
      {
        res = At_recv_response();
        if( res == 0 )
        {
          pSocket->rx_len = 0;
          Stream_init( &pSocket->rx_stream, socketsMem[ socket ], WIFI_SOCKET_BUFFER_BYTESIZE );
        }
      }
    }
  }
  
  res = WifiEsp32_unlock( res );
  
  return res;
}

static uint8_t WifiEsp32_getSocketId( void )
{
  uint8_t bit;

  for( bit = 0; bit < CONN_ID_INVALID; bit++ )
  {
    if( ( pWifiCtr->connIds & ( 1U << bit ) ) == 0 )
      break;
  }
  return bit;
}

static void WifiEsp32_acceptId( uint8_t id )
{
  if( id < CONN_ID_INVALID )
  {
    pWifiCtr->connIds |= ( 1U << id );
  }
}

static void WifiEsp32_freeId( uint8_t id )
{
  if( id < CONN_ID_INVALID )
  {
    pWifiCtr->connIds &= ~( 1U << id );
  }
}

static int32_t WifiEsp32_send( int32_t socket, const void *buff, uint32_t len )
{
  return WifiEsp32_sendTo( socket, buff, len, 0, 0 );
}

static int32_t WifiEsp32_sendTo( int32_t socket, const void *buff, uint32_t len, uint32_t *ip, uint16_t *port )
{
  int32_t res, n, num;
  Socket *pSocket;
  uint32_t *r_ip;
  uint16_t *r_port;
  
  if( ( socket < 0 ) || ( socket >= WIFI_MAX_SOCKETS ) )
    return SOCKET_EINVAL;
  
  if( buff == NULL )
    return SOCKET_EINVAL;
  
  if( len == 0 )
    return SOCKET_EINVAL;
  
  res = WifiEsp32_lock();
  if( res != 0 )
    return SOCKET_ERROR;
  
  pSocket = &sockets[ socket ];
  
  switch( pSocket->state )
  {
  case SOCKET_STATE_CONNECTED:
    break;
  case SOCKET_STATE_FREE:
    res = SOCKET_ESOCK;
    break;
  case SOCKET_STATE_CREATED:
  case SOCKET_STATE_BOUND:
  case SOCKET_STATE_LISTEN:
  case SOCKET_STATE_SERVER:
  case SOCKET_STATE_CONNECTREQ:
  case SOCKET_STATE_CLOSING:
    res = SOCKET_ENOTCONN;
    break;
  case SOCKET_STATE_CLOSED:
    res = SOCKET_ECONNRESET;
  default:
    res = SOCKET_ERROR;
    break;
  }
  
  if( res == wifiOK )
  {
    r_ip = NULL;
    r_port = NULL;
    if( pSocket->type == SOCKET_TYPE_DATAGRAM )
    {
      if( ( ip != NULL ) && ( port != NULL ) )
      {
        r_ip = ip;
        r_port = port;
      }
    }
    
    res = At_send_data( pSocket->id, len, r_ip, r_port );
    
    if( res == 0 )
    {
      res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
    }
    
    if( res == 0 )
    {
      res = WifiEsp32_wait( WIFI_EVENT_TX_WAIT, WIFI_COMMAND_TIMEOUT_MS );
    }
    
    if( res != 0 )
    {
      res = wifiError;
    }
    else
    {
      n = 0;
      num = 0;
      while( len != 0 )
      {
        osEventFlagsClear( pWifiCtr->eventId, WIFI_EVENT_TX_DONE );
        n = At_send_raw( ( uint8_t* )buff + num, len );
        
        if( n < 0 )
        {
          res = SOCKET_ERROR;
          break;
        }
        
        num += n;
        len -= n;
        
        res = WifiEsp32_wait( WIFI_EVENT_TX_DONE, WIFI_COMMAND_TIMEOUT_MS );
        if( res != 0 )
        {
          res = SOCKET_ERROR;
          break;
        }
      }
      
      if( res == 0 )
      {
        res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, pSocket->tout_tx );
      }
      
      if( res == 0 )
      {
        res = At_recv_response();
        if( res == AT_SEND_OK )
        {
          res = num;
        }
        else if( res == AT_SEND_FAIL )
        {
          res = SOCKET_ERROR;
        }
        else
        {
          res = SOCKET_ERROR;
        }
      }
    }
  }
  
  res = WifiEsp32_unlock( res );
    
  return res;
}

static int32_t WifiEsp32_bind( int32_t socket, uint32_t ip, uint16_t port )
{
  int32_t res;
  Socket *pSocket;
  uint32_t n;
  
  if( ( socket < 0 ) || ( socket >= WIFI_MAX_SOCKETS ) )
    return SOCKET_EINVAL;
  
  if( port == 0 )
    return SOCKET_EINVAL;
  
  res = WifiEsp32_lock();
  if( res != wifiOK )
    return SOCKET_ERROR;
  
  pSocket = &sockets[ socket ];
  
  switch( pSocket->state )
  {
  case SOCKET_STATE_FREE:
    res = SOCKET_ESOCK;
    break;
  case SOCKET_STATE_CONNECTED:
    res = SOCKET_EISCONN;
    break;
  case SOCKET_STATE_BOUND:
    res = SOCKET_EINVAL;
    break;
  default:
    break;
  }
  
  if( res == 0 )
  {
    for( n = 0; n < WIFI_MAX_SOCKETS; n++ )
    {
      if( sockets[ n ].state >= SOCKET_STATE_BOUND )
      {
        if( sockets[ n ].l_port == port )
        {
          break;
        }
      }
    }
    if( n != WIFI_MAX_SOCKETS )
    {
      res = SOCKET_EADDRINUSE;
    }
    else
    {
      pSocket->l_ip = ip;
      pSocket->l_port = port;
      pSocket->state = SOCKET_STATE_BOUND;
    }
  }
  
  res = WifiEsp32_unlock( res );
  
  return res;
}

static int32_t WifiEsp32_listen( int32_t socket, int32_t backlog )
{
  int32_t res;
  Socket *pSocket;
  uint32_t n, p, cnt;
  int32_t nbFreeSockets;
  
  if( ( socket < 0 ) || ( socket >= WIFI_MAX_SOCKETS ) )
    return SOCKET_ESOCK;
  
  if( ( backlog <= 0 ) || ( backlog >= WIFI_MAX_SOCKETS ) )
    return SOCKET_EINVAL;
  
  res = WifiEsp32_lock();
  if( res != wifiOK )
    return SOCKET_ERROR;
  
  pSocket = &sockets[ socket ];
  
  if( pSocket->type != SOCKET_TYPE_STREAM )
    res = SOCKET_ENOTSUP;
  
  if( res == 0 )
  {
    switch( pSocket->state )
    {
    case SOCKET_STATE_FREE:
      res = SOCKET_ESOCK;
      break;
    case SOCKET_STATE_BOUND:
        break;
    case SOCKET_STATE_SERVER:
    case SOCKET_STATE_CREATED:
    case SOCKET_STATE_CONNECTED:
    case SOCKET_STATE_LISTEN:
    case SOCKET_STATE_CONNECTREQ:
    case SOCKET_STATE_CLOSING:
    case SOCKET_STATE_CLOSED:
    default:
      res = SOCKET_EINVAL;
      break;
    }
  }
  
  if( res == 0 )
  {
    nbFreeSockets = 0;
    for( n = 0; n < WIFI_MAX_SOCKETS; n++ )
    {
      if( sockets[ n ].state == SOCKET_STATE_FREE )
        nbFreeSockets += 1;
    }
    
    if( nbFreeSockets < backlog )
    {
      res = SOCKET_ENOTSUP;
    }
    else
    {
      res = At_send_serverMaxConn( AT_CMD_SET, backlog );
      if( res == 0 )
      {
        res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
        if( res == 0 )
        {
          res = At_recv_response();
        }
      }
    }
    
    if( res == 0 )
    {
      if( pSocket->protocol == SOCKET_PROTOCOL_TLS )
        res = At_send_server( AT_CMD_SET, AT_SERVER_CREATE, pSocket->l_port, AT_SERVER_TYPE_SSL, 0 );
      else
        res = At_send_server( AT_CMD_SET, AT_SERVER_CREATE, pSocket->l_port, AT_SERVER_TYPE_TCP, 0 );
      if( res == 0 )
      {
        res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
        if( res == 0 )
        {
          res = At_recv_response();
          if( res == AT_OK )
          {
            pSocket->state = SOCKET_STATE_SERVER;
            res = 0;
          }
        }
      }
    }
    
    if( res == 0 )
    {
      p = socket;
      cnt = backlog;
      for( n = 0; n < WIFI_MAX_SOCKETS; n++ )
      {
        if( sockets[ n ].state == SOCKET_STATE_FREE )
        {
          sockets[ p ].backlog = n;
          
          memcpy( &sockets[ n ], pSocket, sizeof( Socket ) );
          
          sockets[ n ].state = SOCKET_STATE_LISTEN;
          sockets[ n ].backlog = pSocket->backlog;
          sockets[ n ].server = socket;
          Stream_init( &sockets[ n ].rx_stream, socketsMem[ n ], WIFI_SOCKET_BUFFER_BYTESIZE );
          
          p = n;
          
          cnt--;
          if( cnt == 0 )
          {
            break;
          }
        }
      }
    }
  }
  
  res = WifiEsp32_unlock( res );
  
  return res;
}

static int32_t WifiEsp32_accept( int32_t socket, uint32_t *ip, uint16_t *port )
{
  int32_t res;
  Socket *pSocket;
  int32_t n;
  
  if( ( socket < 0 ) || ( socket >= WIFI_MAX_SOCKETS ) )
    return SOCKET_ESOCK;
  
  res = WifiEsp32_lock();
  if( res != 0 )
    return SOCKET_ERROR;
  
  res = SOCKET_ERROR;
  
  pSocket = &sockets[ socket ];
  
  if( pSocket->state != SOCKET_STATE_SERVER )
  {
    res = SOCKET_ESOCK;
  }
  else
  {
    do
    {
      bool found = false;
      n = pSocket->backlog;
      do
      {
        if( ( sockets[ n ].state == SOCKET_STATE_CONNECTED ) && 
            ( sockets[ n ].accepted == false ) )
        {
          found = true;
          sockets[ n ].accepted = true;
          if( ip != NULL )
          {
            *ip = sockets[ n ].r_ip;
          }
          if( port != NULL )
          {
            *port = sockets[ n ].r_port;
          }
          res = n;
        }
        n = sockets[ n ].backlog;
      }
      while( !found && ( n != pSocket->backlog ) );
      
      if( !found )
      {
        WifiEsp32_unlock( res );
        WifiEsp32_wait( WIFI_EVENT_WAIT_CONN_ACCEPT, osWaitForever );
      }
    }
    while( res == SOCKET_ERROR );
  }
  
  return res;
}

static int32_t WifiEsp32_recv( int32_t socket, void *buff, uint32_t len )
{
  return WifiEsp32_recvFrom( socket, buff, len, NULL, NULL );
}

static int32_t WifiEsp32_recvFrom( int32_t socket, void *buff, uint32_t len, uint32_t *ip, uint16_t *port )
{
  int32_t res, ex;
  Socket *pSocket;
  uint32_t cnt, n;
  uint8_t *pu8 = (uint8_t *)buff;
  
  if( ( socket < 0 ) || ( socket >= WIFI_MAX_SOCKETS ) )
    return SOCKET_ESOCK;
  
  if( buff == NULL )
    return SOCKET_EINVAL;
  
  res = WifiEsp32_lock();
  if( res != wifiOK )
    return SOCKET_ERROR;
  
  pSocket = &sockets[ socket ];
  
  if( pSocket->state != SOCKET_STATE_CONNECTED )
  {
    res = SOCKET_ERROR;
  }
  
  n = 0;
  
  while( res == 0 )
  {
    if( pSocket->state != SOCKET_STATE_CONNECTED )
    {
      res = SOCKET_ERROR;
      break;
    }
    if( pSocket->rx_len == 0 )
    {
      if( Stream_getCount( &pSocket->rx_stream ) >= 2U )
      {
        if( Stream_read( &pSocket->rx_stream, ( uint8_t* )&pSocket->rx_len, 2U ) != 2U )
        {
          res = SOCKET_ERROR;
        }
      }
    }
    
    if( pSocket->rx_len == 0 )
    {
      if( n != 0U )
      {
        res = ( int32_t )n;
      }
      else
      {
        WifiEsp32_unlock( res );
        
        ex = WifiEsp32_wait( WIFI_EVENT_WAIT_CONN_RX( socket ) | WIFI_EVENT_WAIT_CONN_CLOSE( socket ), pSocket->tout_rx );
        
        WifiEsp32_lock();
        
        if( ex != 0 )
        {
          if( ex == -1 )
          {
            res = SOCKET_EAGAIN;
          }
          else
          {
            res = SOCKET_ERROR;
          }
        }
        
        if( pSocket->state != SOCKET_STATE_CONNECTED )
        {
          res = SOCKET_ECONNRESET;
        }
      }
    }
    else if( ( pSocket->rx_len != 0 ) && ( len == 0 ) )
    {
      break;
    }
    else
    {
      cnt = len - n;
      
      if( cnt > pSocket->rx_len )
      {
        cnt = pSocket->rx_len;
      }
      
      cnt = Stream_read( &pSocket->rx_stream, &pu8[n], cnt );
      
      pSocket->rx_len -= cnt;
      n += cnt;
      
      if( n == len )
      {
        res = ( int32_t )n;
      }
    }
  }
  
  res = WifiEsp32_unlock( res );
  
  return res;
}

static int32_t WifiEsp32_close( int32_t socket )
{
  int32_t res;
  Socket *pSocket;
  uint32_t n;
  
  if( ( socket < 0 ) || ( socket >= WIFI_MAX_SOCKETS ) )
    return SOCKET_ESOCK;
  
  res = WifiEsp32_lock();
  if( res != 0 )
    return SOCKET_ERROR;
  
  WifiEsp32_wait( WIFI_EVENT_WAIT_CONN_CLOSE( socket ), 25 );
  
  pSocket = &sockets[ socket ];
  
  if( pSocket->state == SOCKET_STATE_FREE )
  {
    res = SOCKET_ESOCK;
  }
  else if( pSocket->state == SOCKET_STATE_SERVER )
  {
    res = At_send_server( AT_CMD_SET, AT_SERVER_SHUTDOWN_AND_CLOSE, 0, NULL, 0 );
    if( res == 0 )
    {
      res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
      if( res == 0 )
      {
        res = At_recv_response();
        if( res == AT_OK )
        {
          pSocket->state = SOCKET_STATE_FREE;
        }
      }
    }
    if( res == 0 )
    {
      for( n = 0; n < WIFI_MAX_SOCKETS; n++ )
      {
        if( sockets[ n ].server != SOCKET_INVALID )
        {
          sockets[ n ].server = SOCKET_INVALID;
          sockets[ n ].backlog = SOCKET_INVALID;
          sockets[ n ].state = SOCKET_STATE_FREE;
          Stream_uninit( &sockets[ n ].rx_stream );
        }
      }
    }
  }
  else if( ( pSocket->state == SOCKET_STATE_CLOSED ) ||
           ( pSocket->state == SOCKET_STATE_CONNECTREQ ) )
  {
    if( pSocket->backlog == SOCKET_INVALID )
    {
      pSocket->state = SOCKET_STATE_FREE;
      Stream_uninit( &pSocket->rx_stream );
    }
    else
    {
      pSocket->state = SOCKET_STATE_LISTEN;
    }
  }
  else
  {
    if ( ( pSocket->state > SOCKET_STATE_LISTEN ) && 
         ( pSocket->state <= SOCKET_STATE_CLOSING ) )
    {
      pSocket->state = SOCKET_STATE_CLOSING;
      res = At_send_closeConnection( pSocket->id );
      if( res == 0 )
      {
        res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
        if( res == 0 )
        {
          res = At_recv_response();
          if( res == AT_OK )
          {
            if( pSocket->backlog == SOCKET_INVALID )
              Stream_uninit( &pSocket->rx_stream );
          }
        }
        else
        {
          res = SOCKET_ERROR;
        }
      }
      if( pSocket->state == SOCKET_STATE_CLOSING )
        res = SOCKET_ERROR;
    }
  }
  
  res = WifiEsp32_unlock( res );
  
  return res;
}

static int32_t WifiEsp32_setSocketOpt( int32_t socket, int32_t opt, const void *value, uint32_t len )
{
  int32_t res;
  Socket *pSocket;
  uint32_t val;
  
  if( ( socket < 0 ) || ( socket >= WIFI_MAX_SOCKETS ) )
    return SOCKET_ESOCK;
  
  if( len != 4 )
    return SOCKET_EINVAL;
  
  res = WifiEsp32_lock();
  if( res != 0 )
    return SOCKET_ERROR;
  
  pSocket = &sockets[ socket ];
  
  if( pSocket->state == SOCKET_STATE_FREE )
    res = SOCKET_ESOCK;
  
  if( res == 0 )
  {
    val = *(__packed uint32_t*)value;
    switch( opt )
    {
    case SOCKET_OPT_FIONBIO:
      if( val == 0 )
      {
        pSocket->flags &= ~SOCKET_FLAG_NONBLOCK;
      }
      else
      {
        pSocket->flags |= SOCKET_FLAG_NONBLOCK;
      }
      break;
    case SOCKET_OPT_RCVTIMEO:
      pSocket->tout_rx = val;
      break;
    case SOCKET_OPT_SNDTIMEO:
      pSocket->tout_tx = val;
      break;
    case SOCKET_OPT_KEEPALIVE:
      if( val == 0 )
      {
        pSocket->flags &= ~SOCKET_FLAG_KEEPALIVE;
      }
      else
      {
        pSocket->flags |= SOCKET_FLAG_KEEPALIVE;
      }
      break;
    case SOCKET_OPT_TYPE:
    default:
      res = SOCKET_ERROR;
      break;
    }
  }
  
  res = WifiEsp32_unlock( res );
  
  return res;
}

static int32_t WifiEsp32_getSocketOpt( int32_t socket, int32_t opt, const void *value, uint32_t *len )
{
  int32_t res;
  Socket *pSocket;
  uint32_t *val;
  
  if( ( socket < 0 ) || ( socket >= WIFI_MAX_SOCKETS ) )
    return SOCKET_ESOCK;
  
  res = WifiEsp32_lock();
  if( res != 0 )
    return SOCKET_ERROR;
  
  pSocket = &sockets[ socket ];
  
  if( pSocket->state == SOCKET_STATE_FREE )
    res = SOCKET_ESOCK;
  
  if( res == 0 )
  {
    val = ( uint32_t* )value;
    *len = 4;
    switch( opt )
    {
    case SOCKET_OPT_FIONBIO:
      *val = ( ( pSocket->flags & SOCKET_FLAG_NONBLOCK ) == 0 ? 0 : 1 );
      break;
    case SOCKET_OPT_RCVTIMEO:
      *val = pSocket->tout_rx;
      break;
    case SOCKET_OPT_SNDTIMEO:
      *val = pSocket->tout_tx;
      break;
    case SOCKET_OPT_KEEPALIVE:
      *val = ( ( pSocket->flags & SOCKET_FLAG_KEEPALIVE ) == 0 ? 0 : 1 );
      break;
    case SOCKET_OPT_TYPE:
      *val = pSocket->type;
      break;
    default:
      res = SOCKET_ERROR;
      break;
    }
  }
  
  res = WifiEsp32_unlock( res );
  
  return res;
}



static int32_t WifiEsp32_update( Version * const version )
{
  int32_t res;
  char out[ 15 ];
  
  res = WifiEsp32_lock();
  if( res != 0 )
    return SOCKET_ERROR;
  
  sprintf( out, "v%hhu.%hhu.%hhu.%hhu", version->major, version->minor, version->patch, version->hotfix );
  res = At_send_update( 1, out, 0 );
  if( res == 0 )
  {
    res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_UPDATE_TIMEOUT_MS );
    if( res == 0 )
    {
      res = At_recv_response();
      if( res == AT_OK )
      {
        // In blocking mode, ESP32 auto restart. Wait ready report.
        WifiEsp32_wait( WIFI_EVENT_RESPONSE, osWaitForever );
      }
      else if( res == AT_ERROR )
      {
        res = -1;
      }
    }
  }

  res = WifiEsp32_unlock( res );
  
  return res;
}

static int32_t WifiEsp32_restore( void )
{
  int32_t res;
  
  res = WifiEsp32_lock();
  if( res != 0 )
    return SOCKET_ERROR;
  
  res = At_send_restore();
  if( res == 0 )
  {
    res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
    if( res == 0 )
    {
      res = At_recv_response();
      if( res == AT_OK )
      {
        res = 0;
      }
      else
      {
        res = -1;
      }
    }
  }
  
  res = WifiEsp32_unlock( res );
  
  return res;
}

static int32_t WifiEsp32_flash( const char *partition, const void *value, uint32_t len )
{
  int32_t res, num, n;
  
  res = WifiEsp32_lock();
  if( res != 0 )
    return wifiError;
  
  /* Clear partition */
  res = At_send_sysflash( AT_CMD_SET, AT_FLASH_OP_ERASE, partition, 0, AT_FLASH_CLEAR_ALL );
  if( res == 0 )
  {
    res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
    if( res == 0 )
    {
      res = At_recv_response();
      if( res == AT_OK )
      {
        /* Write partition and key-value */
        res = At_send_sysflash( AT_CMD_SET, AT_FLASH_OP_WRITE, partition, 0, len );
        if( res == 0 )
        {
          res = WifiEsp32_wait( WIFI_EVENT_TX_WAIT, WIFI_COMMAND_TIMEOUT_MS );
        }
        if( res != 0 )
        {
          res = wifiError;
        }
        else
        {
          n = 0;
          num = 0;
          while( len != 0 )
          {
            osEventFlagsClear( pWifiCtr->eventId, WIFI_EVENT_TX_DONE );
            n = At_send_raw( ( uint8_t* )value + num, len );
            
            if( n < 0 )
            {
              res = wifiError;
              break;
            }
            
            num += n;
            len -= n;
            
            res = WifiEsp32_wait( WIFI_EVENT_TX_DONE, WIFI_COMMAND_TIMEOUT_MS );
            if( res != 0 )
            {
              res = wifiError;
              break;
            }
          }
          
          if( res == 0 )
          {
            res = WifiEsp32_wait( WIFI_EVENT_RESPONSE, WIFI_COMMAND_TIMEOUT_MS );
          }
          
          if( res == 0 )
          {
            res = At_recv_response();
          }
        }
      }
    }
  }
  
  res = WifiEsp32_unlock( res );
  
  return res;
}

Wifi wifi = {
  WifiEsp32_init,
  WifiEsp32_turnOn,
  WifiEsp32_turnOff,
  WifiEsp32_getVersion,
  WifiEsp32_enable,
  WifiEsp32_reconnectionOpt,
  WifiEsp32_isConnect,
  WifiEsp32_getNetInfo,
  WifiEsp32_getAddress,
  WifiEsp32_disable,
  WifiEsp32_scan,
  WifiEsp32_socket,
  WifiEsp32_connect,
  WifiEsp32_send,
  WifiEsp32_sendTo,
  WifiEsp32_bind,
  WifiEsp32_listen,
  WifiEsp32_accept,
  WifiEsp32_recv,
  WifiEsp32_recvFrom,
  WifiEsp32_close,
  WifiEsp32_setSocketOpt,
  WifiEsp32_getSocketOpt,
  WifiEsp32_update,
  WifiEsp32_restore,
  WifiEsp32_flash
};
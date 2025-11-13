#ifndef __WIFI_H__
#define __WIFI_H__

#include <stdint.h>
#include <stdbool.h>
#include "network_utils.h"
#include "stream.h"

#define wifiOK               0
#define wifiError           -1
#define wifiErrorTimeout    -2
#define wifiErrorResource   -3
#define wifiErrorParameter  -4
#define wifiNotSupported    -5

typedef enum
{
  WIFI_INTERFACE_STATION,
  WIFI_INTERFACE_AP
} WifiInterface;

#define LIST_WIFI_ENCRYPTION                      \
  ITEM( WIFI_ENC_OPEN,          "OPEN" )          \
  ITEM( WIFI_ENC_WEP,           "WEP" )           \
  ITEM( WIFI_ENC_WPA_PSK,       "WPA-PSK" )       \
  ITEM( WIFI_ENC_WPA2_PSK,      "WPA2-PSK" )      \
  ITEM( WIFI_ENC_WPA_WPA2_PSK,  "WPA-WPA2-PSK" )

typedef enum
{
#define ITEM( enc, text ) enc,
  LIST_WIFI_ENCRYPTION
#undef ITEM
} WifiEncryption;

typedef struct
{
  const char *ssid;
  const char *pass;
  uint8_t mac[6];
  uint8_t channel;
  WifiEncryption enc;
} WifiConfig;

typedef struct
{
  uint8_t mac[ 6 ];
  uint32_t ip;
  uint32_t gateway;
  uint32_t netmask;
} NetworkAddress;

#define NETWORK_INFO_SSID_BYTESIZE 33
#define NETWORK_INFO_MAC_BYTESIZE MAC_BYTESIZE

typedef __packed struct
{
  uint8_t ecn;
  char ssid[ NETWORK_INFO_SSID_BYTESIZE ];
  uint8_t bssid[ NETWORK_INFO_MAC_BYTESIZE ];
  uint8_t channel;
  int8_t rssi;
} NetworkInfo;

typedef __packed struct
{
  uint8_t ecn;
  char ssid[ NETWORK_INFO_SSID_BYTESIZE ];
  uint8_t bssid[ NETWORK_INFO_MAC_BYTESIZE ];
  int8_t rssi;
} NetworkScanInfo;

typedef void ( *WifiEventCallback )( uint32_t event, void *args );

#define WIFI_EVENT_STATION_CONNECTED    ( 1U << 0 )
#define WIFI_EVENT_STATION_GOT_IP       ( 1U << 1 )
#define WIFI_EVENT_STATION_DISCONNECTED ( 1U << 2 )
#define WIFI_EVENT_UPDATE               ( 1U << 3 )

typedef struct
{
  uint8_t mac[ 6 ];
  uint32_t ip;
} WifiStation;

#define SOCKET_TYPE_STREAM    1U
#define SOCKET_TYPE_DATAGRAM  2U

#define SOCKET_PROTOCOL_UDP 1U
#define SOCKET_PROTOCOL_TCP 2U
#define SOCKET_PROTOCOL_TLS 3U

#define SOCKET_STATE_FREE       0U
#define SOCKET_STATE_CREATED    1U
#define SOCKET_STATE_BOUND      2U
#define SOCKET_STATE_LISTEN     3U
#define SOCKET_STATE_CONNECTREQ 4U
#define SOCKET_STATE_CONNECTED  5U
#define SOCKET_STATE_CLOSING    6U
#define SOCKET_STATE_CLOSED     7U
#define SOCKET_STATE_SERVER     8U
#define SOCKET_STATE_INVALID    0xFF

#define SOCKET_ERROR            (-1)         ///< Unspecified error
#define SOCKET_ESOCK            (-2)         ///< Invalid socket
#define SOCKET_EINVAL           (-3)         ///< Invalid argument
#define SOCKET_ENOTSUP          (-4)         ///< Operation not supported
#define SOCKET_ENOMEM           (-5)         ///< Not enough memory
#define SOCKET_EAGAIN           (-6)         ///< Operation would block or timed out
#define SOCKET_EINPROGRESS      (-7)         ///< Operation in progress
#define SOCKET_ETIMEDOUT        (-8)         ///< Operation timed out
#define SOCKET_EISCONN          (-9)         ///< Socket is connected
#define SOCKET_ENOTCONN         (-10)        ///< Socket is not connected
#define SOCKET_ECONNREFUSED     (-11)        ///< Connection rejected by the peer
#define SOCKET_ECONNRESET       (-12)        ///< Connection reset by the peer
#define SOCKET_ECONNABORTED     (-13)        ///< Connection aborted locally
#define SOCKET_EALREADY         (-14)        ///< Connection already in progress
#define SOCKET_EADDRINUSE       (-15)        ///< Address in use
#define SOCKET_EHOSTNOTFOUND    (-16)        ///< Host not found

#define SOCKET_OPT_FIONBIO        1          ///< Non-blocking I/O (Set only, default = 0);
#define SOCKET_OPT_RCVTIMEO       2          ///< Receive timeout in ms (default = 0);
#define SOCKET_OPT_SNDTIMEO       3          ///< Send timeout in ms (default = 0);
#define SOCKET_OPT_KEEPALIVE      4          ///< Keep-alive messages (default = 0);
#define SOCKET_OPT_TYPE           5          ///< Socket Type (Get only);

#define SOCKET_FLAG_NONBLOCK      (1U << 0)
#define SOCKET_FLAG_KEEPALIVE     (1U << 1)

typedef struct
{
  /* Configuration */
  uint8_t flags;
  uint8_t type;
  uint8_t protocol;
  uint32_t l_ip;
  uint16_t l_port;
  uint32_t r_ip;
  uint16_t r_port;
  volatile uint8_t state;
  volatile uint8_t id;
  uint8_t server;
  uint8_t backlog;
  volatile bool accepted;
  uint32_t tout_rx;
  uint32_t tout_tx;
  /* RX data layer */
  Stream rx_stream;
  uint32_t rx_len;
} Socket;

typedef struct
{
  uint8_t major;
  uint8_t minor;
  uint8_t patch;
  uint8_t hotfix;
} Version;

typedef struct
{
  int32_t ( *init )( WifiEventCallback eventCallback );
  int32_t ( *turnOn )( void );
  int32_t ( *turnOff )( void );
  int32_t ( *getVersion )( Version * const version );
  int32_t ( *enable )( WifiInterface interface, WifiConfig * const config );
  int32_t ( *reconnectionOpt )( uint16_t interval_s, uint16_t repeat_nb );
  int32_t ( *isConnect )( void );
  int32_t ( *getNetInfo )( NetworkInfo * const info );
  int32_t ( *getAddress )( WifiInterface interface, NetworkAddress *address );
  int32_t ( *disable )( WifiInterface interface );
  int32_t ( *scan )( NetworkScanInfo scanInfo[], uint32_t maxScanInfo, char *ssid, int8_t rssi );
  int32_t ( *socket )( uint8_t type, uint8_t protocol );
  int32_t ( *connect )( int32_t socket, uint32_t ip, uint16_t port );
  int32_t ( *send )( int32_t socket, const void *buff, uint32_t len );
  int32_t ( *sendTo )( int32_t socket, const void *buff, uint32_t len, uint32_t *ip, uint16_t *port );
  int32_t ( *bind )( int32_t socket, uint32_t ip, uint16_t port );
  int32_t ( *listen )( int32_t socket, int32_t backlog );
  int32_t ( *accept )( int32_t socket, uint32_t *ip, uint16_t *port );
  int32_t ( *recv )( int32_t socket, void *buff, uint32_t len );
  int32_t ( *recvFrom )( int32_t socket, void *buff, uint32_t len, uint32_t *ip, uint16_t *port );
  int32_t ( *close )( int32_t socket );
  int32_t ( *setSocketOpt )( int32_t socket, int32_t opt, const void *value, uint32_t len );
  int32_t ( *getSocketOpt )( int32_t socket, int32_t opt, const void *value, uint32_t *len );
  int32_t ( *update )( Version * const version );
  int32_t ( *restore )( void );
  int32_t ( *flash )( const char *partition, const void *value, uint32_t len );
} Wifi;

extern Wifi wifi;

const char * wifiEncryptionToStr( WifiEncryption enc );
#define WIFIENC2STR( enc ) wifiEncryptionToStr( enc )

int32_t Version_compare( Version *candidate, Version *current );

#endif
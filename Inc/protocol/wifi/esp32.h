#ifndef __ESP32_H__
#define __ESP32_H__

#include <stdint.h>
#include "stream.h"

#define AT_SEND_LONG_MSG_BYTESIZE 8192

#define AT_STRING_RESPONSE_LIST                 \
  ITEM( AT_OK,              "OK"              ) \
  ITEM( AT_ERROR,           "ERROR"           ) \
  ITEM( AT_SEND_OK,         "SEND OK"         ) \
  ITEM( AT_SEND_FAIL,       "SEND FAIL"       )

#define AT_STRING_REPORT_LIST                   \
  ITEM( AT_FAIL,            "FAIL"            ) \
  ITEM( AT_BUSY_P,          "busy p..."       ) \
  ITEM( AT_WIFI_CONNECTED,  "WIFI CONNECTED"  ) \
  ITEM( AT_WIFI_GOT_IP,     "WIFI GOT IP"     ) \
  ITEM( AT_WIFI_DISCONNECT, "WIFI DISCONNECT" ) \
  ITEM( AT_AT,              "AT"              ) \
  ITEM( AT_READY,           "ready"           ) \
  ITEM( AT_ERR_CODE,        "ERR CODE"        ) 
    
#define AT_STRING_RESP_GMR_LIST           \
  ITEM( AT_GMR_VERSION, "AT version"   )  \
  ITEM( AT_GMR_SDK,     "SDK version"  )  \
  ITEM( AT_GMR_TIME,    "compile time" )  \
  ITEM( AT_GMR_BIN,     "Bin version"  )

#define AT_STRING_CMD_LIST                              \
  ITEM( AT_CMD_IPD,               "IPD"               ) \
  ITEM( AT_CMD_GMR,               "GMR"               ) \
  ITEM( AT_CMD_UART_CUR,          "UART_CUR"          ) \
  ITEM( AT_CMD_TEST,              "AT"                ) \
  ITEM( AT_CMD_ECHO,              "ATE"               ) \
  ITEM( AT_CMD_MODE,              "CWMODE"            ) \
  ITEM( AT_CMD_CWSAP,             "CWSAP"             ) \
  ITEM( AT_CMD_CWJAP,             "CWJAP"             ) \
  ITEM( AT_CMD_CWQAP,             "CWQAP"             ) \
  ITEM( AT_CMD_CWRECONNCFG,       "CWRECONNCFG"       ) \
  ITEM( AT_CMD_CWSTATE,           "CWSTATE"           ) \
  ITEM( AT_CMD_CIPSTAMAC,         "CIPSTAMAC"         ) \
  ITEM( AT_CMD_CIPAPMAC,          "CIPAPMAC"          ) \
  ITEM( AT_CMD_CIPSTA,            "CIPSTA"            ) \
  ITEM( AT_CMD_CIPAP,             "CIPAP"             ) \
  ITEM( AT_CMD_CWLAPOPT,          "CWLAPOPT"          ) \
  ITEM( AT_CMD_CWLAP,             "CWLAP"             ) \
  ITEM( AT_CMD_CIPMUX,            "CIPMUX"            ) \
  ITEM( AT_CMD_CIPSERVER,         "CIPSERVER"         ) \
  ITEM( AT_CMD_CIPSERVERMAXCONN,  "CIPSERVERMAXCONN"  ) \
  ITEM( AT_CMD_SYSMSG,            "SYSMSG"            ) \
  ITEM( AT_CMD_CIPSTART,          "CIPSTART"          ) \
  ITEM( AT_CMD_CIPCLOSE,          "CIPCLOSE"          ) \
  ITEM( AT_CMD_CIPSENDL,          "CIPSENDL"          ) \
  ITEM( AT_CMD_CIPSEND,           "CIPSEND"           ) \
  ITEM( AT_CMD_STA_CONNECTED,     "STA_CONNECTED"     ) \
  ITEM( AT_CMD_DIST_STA_IP,       "DIST_STA_IP"       ) \
  ITEM( AT_CMD_STA_DISCONNECTED,  "STA_DISCONNECTED"  ) \
  ITEM( AT_CMD_LINK_CONN,         "LINK_CONN"         ) \
  ITEM( AT_CMD_CIUPDATE,          "CIUPDATE"          ) \
  ITEM( AT_CMD_RESTORE,           "RESTORE"           ) \
  ITEM( AT_CMD_SYSSTORE,          "SYSSTORE"          ) \
  ITEM( AT_CMD_SYSFLASH,          "SYSFLASH"          ) \
  ITEM( AT_CMD_CIFSR,             "CIFSR"             )
    
#define AT_STRING_RESP_LINK_LIST  \
  ITEM( AT_CONNECT, "CONNECT" )   \
  ITEM( AT_CLOSED,  "CLOSED"  )

#define AT_STRING_RESP_LINK_TYPE_LIST  \
  ITEM( AT_LINK_UDP, "UDP" )           \
  ITEM( AT_LINK_TCP, "TCP" )           \
  ITEM( AT_LINK_SSL, "SSL" )
    
#define AT_NOTIFY_EVENTS_LIST         \
  ITEM( AT_NOTIFY_DATA_RX           ) \
  ITEM( AT_NOTIFY_TX_DONE           ) \
  ITEM( AT_NOTIFY_READY             ) \
  ITEM( AT_NOTIFY_RESPONSE          ) \
  ITEM( AT_NOTIFY_CONNECTED         ) \
  ITEM( AT_NOTIFY_GOT_IP            ) \
  ITEM( AT_NOTIFY_DISCONNECT        ) \
  ITEM( AT_NOTIFY_STA_CONNECTED     ) \
  ITEM( AT_NOTIFY_DIST_STA_IP       ) \
  ITEM( AT_NOTIFY_STA_DISCONNECTED  ) \
  ITEM( AT_NOTIFY_LINK_CONNECT      ) \
  ITEM( AT_NOTIFY_LINK_CLOSED       ) \
  ITEM( AT_NOTIFY_LINK_RX_INIT      ) \
  ITEM( AT_NOTIFY_LINK_RX_DATA      ) \
  ITEM( AT_NOTIFY_REQ_SEND          ) \
  ITEM( AT_NOTIFY_UPDATE            ) \
  ITEM( AT_NOTIFY_SCAN_RESULT       )
    
typedef enum
{
#define ITEM( code, value ) code,
  AT_STRING_REPORT_LIST
#undef ITEM
AT_REPORT_UNKNOWN = 0xFF
} AtReportCode;

typedef enum
{
#define ITEM( code, value ) code,
  AT_STRING_RESPONSE_LIST
#undef ITEM
  AT_RESPONSE_UNKNOWN = 0xFF
} AtResponseResult;

typedef enum
{
#define ITEM( code, value ) code,
  AT_STRING_CMD_LIST
#undef ITEM
  AT_CMD_NUM,
  AT_CMD_UNKNOWN = 0xFF
} AtCmdCode;

typedef enum
{
#define ITEM( code, value ) code,
  AT_STRING_RESP_GMR_LIST
#undef ITEM
  AT_GMR_UNKNOWN = 0xFF
} AtGmrCode;

typedef enum
{
#define ITEM( code, value ) code,
  AT_STRING_RESP_LINK_LIST
#undef ITEM
  AT_RESP_LINK_UNKNOWN = 0xFF
} AtLinkCode;

typedef enum
{
  AT_CMD_EXECUTE,
  AT_CMD_QUERY,
  AT_CMD_SET
} AtCmdType;

typedef enum
{
  AT_MODE_RF_OFF,
  AT_MODE_STATION,
  AT_MODE_AP,
  AT_MODE_AP_STATION
} AtMode;

typedef struct
{
  uint32_t baudrate;
  uint8_t data_bits;
  uint8_t stop_bits;
  uint8_t parity;
  uint8_t flow_control;
} AtUartConfig;

#define AT_MIN_CHANNEL_ID  1
#define AT_MAX_CHANNEL_ID  13

typedef struct
{
  char ssid[ 32 + 1 ];
  char pass[ 64 + 1 ];
  uint8_t chl;
  uint8_t ecn;
  uint8_t max_conn;
  uint8_t ssid_hidden;
} AtApConfig;

typedef struct
{
  uint8_t ecn;
  char ssid[ 32 + 1 ];
  uint8_t bssid[ 6 ];
  uint8_t chl;
  int8_t rssi;
} AtNetwork;

typedef enum
{
#define ITEM( code, value ) code,
  AT_STRING_RESP_LINK_TYPE_LIST
#undef ITEM
  AT_LINK_UNKNOWN = 0xFF
} AtLinkType;

typedef struct
{
  uint8_t status;
  uint8_t id;
  AtLinkType type;
  uint8_t terminal;
  uint32_t r_ip;
  uint16_t r_port;
  uint16_t l_port;
} AtLinkInfo;

typedef enum
{
  AT_SERVER_SHUTDOWN_AND_KEEP,
  AT_SERVER_SHUTDOWN_AND_CLOSE,
  AT_SERVER_CREATE
} AtServerMode;

#define AT_SERVER_TYPE_LIST               \
  ITEM( AT_SERVER_TYPE_TCP,     "TCP"   ) \
  ITEM( AT_SERVER_TYPE_TCP_V6,  "TCPv6" ) \
  ITEM( AT_SERVER_TYPE_SSL,     "SSL"   ) \
  ITEM( AT_SERVER_TYPE_SSL_V6,  "SSLv6" )

typedef enum
{
#define ITEM( code, value ) code,
  AT_SERVER_TYPE_LIST
#undef ITEM
} AtServerType;

typedef enum
{
#define ITEM( event ) event,
  AT_NOTIFY_EVENTS_LIST
#undef ITEM
} AtEvent;

#define AT_STATES_LIST        \
  ITEM( AT_STATE_ANALIZE    ) \
  ITEM( AT_STATE_RESP_GEN   ) \
  ITEM( AT_STATE_REPO_GEN   ) \
  ITEM( AT_STATE_RESP_CMD   ) \
  ITEM( AT_STATE_RESP_GMR   ) \
  ITEM( AT_STATE_RESP_LINK  ) \
  ITEM( AT_STATE_RECV_DATA  ) \
  ITEM( AT_STATE_WAIT       ) \
  ITEM( AT_STATE_FLUSH      ) \
  ITEM( AT_STATE_REQ_SEND   )

typedef enum
{
#define ITEM( state ) state,
  AT_STATES_LIST
#undef ITEM
} AtParserState;

#define AT_PARSER_BUFFER_BYTESIZE 512

typedef struct
{
  Stream rxStream;
  Stream cmdRespStream;
  AtParserState state;
  AtCmdCode cmd_sent;
  AtCmdCode response_code;
  AtResponseResult response_result;
  AtReportCode report_code;
  uint16_t response_len;
  AtLinkCode response_link;
  uint16_t ipd_rx;
  void *mutex;
} AtEsp32;

int32_t At_init( void );

int32_t At_deinit( void );

void At_parser( void );

extern void At_notify( AtEvent event, void *arg );

int32_t At_send_currentUart( AtCmdType type, AtUartConfig * const config );

int32_t At_send_test( void );

int32_t At_send_echo( uint8_t enable );

int32_t At_send_macAp( AtCmdType type, uint8_t mac[ 6 ] );

int32_t At_send_macStation( AtCmdType type, uint8_t mac[ 6 ] );

int32_t At_recv_mac( uint8_t mac[ 6 ] );

int32_t At_recv_macAndIp( uint8_t mac[ 6 ], uint32_t *ip );

int32_t At_send_addressAp( AtCmdType type, uint32_t ip, uint32_t gateway, uint32_t netmask );

int32_t At_send_addressStation( AtCmdType type, uint32_t ip, uint32_t gateway, uint32_t netmask );

int32_t At_recv_ip( uint32_t *ip );

int32_t At_send_multiConnection( AtCmdType type, uint8_t enable );

int32_t At_send_sysstore( AtCmdType type, uint8_t enable );

int32_t At_send_sysmsg( AtCmdType type, uint8_t setup );

int32_t At_send_versionInfo( void );

int32_t At_recv_versionInfo( char *info, uint32_t size );

int32_t At_send_mode( AtCmdType type, AtMode mode );
  
int32_t At_recv_mode( AtMode * const mode );

int32_t At_recv_response( void );

int32_t At_send_confAp( AtCmdType type, AtApConfig * const config );

int32_t At_recv_confAp( AtApConfig * const config );

int32_t At_send_getStatus( void );

int32_t At_recv_getStatus( char *ssid );

int32_t At_send_connectToAp( AtCmdType type, const char *ssid, const char *pass, const uint8_t mac[6] );

int32_t At_recv_connectToAp( AtNetwork * const network );

int32_t At_send_disconnectFromAp( void );

int32_t At_send_reconnToApOpt( uint16_t interval_s, uint16_t repeat_nb );

#define AT_SCAN_OPT_SHOW_ECV      ( 1U << 0  )
#define AT_SCAN_OPT_SHOW_SSID     ( 1U << 1  )
#define AT_SCAN_OPT_SHOW_RSSI     ( 1U << 2  )
#define AT_SCAN_OPT_SHOW_MAC      ( 1U << 3  )
#define AT_SCAN_OPT_SHOW_CHANNEL  ( 1U << 4  )
#define AT_SCAN_OPT_SHOW_FREQOFF  ( 1U << 5  )
#define AT_SCAN_OPT_SHOW_FREQCAL  ( 1U << 6  )
#define AT_SCAN_OPT_SHOW_PAIR     ( 1U << 7  )
#define AT_SCAN_OPT_SHOW_GROUP    ( 1U << 8  )
#define AT_SCAN_OPT_SHOW_BGN      ( 1U << 9  )
#define AT_SCAN_OPT_SHOW_WPS      ( 1U << 10 )
#define AT_SCAN_MAX_OPT_NUM       10

int32_t At_send_scanOpt( uint16_t printOpt, int8_t rssiFilter );

int32_t At_send_scanAp( char *ssid );

int32_t At_recv_scanAp( uint16_t printOpt, AtNetwork * const network );

int32_t At_send_createConnection( uint8_t id, AtLinkType type, uint32_t ip, uint16_t port );

int32_t At_recv_linkId( void );

int32_t At_recv_linkInfo( AtLinkInfo *info );

int32_t At_send_closeConnection( uint8_t id );

int32_t At_send_data( uint8_t id, uint16_t len, uint32_t *ip, uint16_t *port );

int32_t At_send_raw( uint8_t *data, uint16_t len );

int32_t At_send_server( AtCmdType cmdType, AtServerMode serverMode, uint16_t port, AtServerType serverType, uint8_t caEnable );

int32_t At_send_serverMaxConn( AtCmdType type, uint8_t num );

int32_t At_recv_ipd( uint8_t *id, uint16_t *len, uint32_t *ip, uint16_t *port );

int32_t At_send_update( uint8_t ota, const char *version, uint8_t nonblocking );

int32_t At_recv_update( void );

int32_t At_send_restore( void );

#define AT_FLASH_OP_ERASE       0x00
#define AT_FLASH_OP_WRITE       0x01
#define AT_FLASH_OP_READ        0x02

#define AT_FLASH_PART_TYPE_CA   0x01
#define AT_FLASH_PART_TYPE_CERT 0x02
#define AT_FLASH_PART_TYPE_KEY  0x03

#define AT_FLASH_CLEAR_ALL  0xFFFFFFFF

int32_t At_send_sysflash( AtCmdType cmdType, uint8_t op, const char *partition, uint32_t offset, uint32_t len );

#endif
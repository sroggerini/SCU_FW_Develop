#ifndef __WIFI_ESP32_H__
#define __WIFI_ESP32_H__

#include <stdint.h>
#include "cmsis_os.h"
#include "wifi.h"

#define WIFI_THREAD_POOLING_TIMEOUT 20

#define WIFI_TASK_RX_DATA   0x01
#define WIFI_TASK_TERMINATE 0x02

#define WIFI_TASK_FLAGS ( WIFI_TASK_RX_DATA |   \
                          WIFI_TASK_TERMINATE )

#define WIFI_EVENT_TX_DONE              ( 1U << 0 )
#define WIFI_EVENT_RESPONSE             ( 1U << 1 )
#define WIFI_EVENT_TX_WAIT              ( 1U << 2 )
#define WIFI_EVENT_WAIT_CONN_ACCEPT     ( 1U << 3 )
#define WIFI_EVENT_WAIT_CONN_RX( n )    ( 1U << (  4 + ( n ) ) )
#define WIFI_EVENT_WAIT_CONN_OPEN( n )  ( 1U << (  9 + ( n ) ) )
#define WIFI_EVENT_WAIT_CONN_CLOSE( n ) ( 1U << ( 14 + ( n ) ) )

#define WIFI_FLAG_INIT                ( 1U << 0 )
#define WIFI_FLAG_POWER               ( 1U << 1 )
#define WIFI_FLAGS_CONN_INFO_ACTIVE   ( 1U << 2 )
#define WIFI_FLAG_AP_ACTIVE           ( 1U << 3 )
#define WIFI_FLAG_STATION_ACTIVE      ( 1U << 4 )
#define WIFI_FLAG_STATION_CONNECTED   ( 1U << 5 )
#define WIFI_FLAG_STATION_GOT_IP      ( 1U << 6 )

#define WIFI_COMMAND_TIMEOUT_MS                 1000
#define WIFI_COMMAND_CONFIG_AP_TIMEOUT_MS       2000
#define WIFI_COMMAND_CONNECT_TO_AP_TIMEOUT_MS   20000
#define WIFI_COMMAND_SCAN_AP_TIMEOUT_MS         5000
#define WIFI_COMMAND_CLIENT_CONNECT_TIMEOUT_MS  20000
#define WIFI_COMMAND_SEND_DATA_TIMEOUT_MS       5000
#define WIFI_COMMAND_UPDATE_TIMEOUT_MS          ( 3 * 60000 )

#define WIFI_UART_DEF_BAUDRATE    115200
#define WIFI_UART_DEF_DATABITS    8
#define WIFI_UART_DEF_STOPBITS    1
#define WIFI_UART_DEF_PARITY      0
#define WIFI_UART_DEF_FLOWCONTROL 0

#define WIFI_AP_MAX_CONNECTIONS 8

#define WIFI_MAX_SOCKETS            5
#define WIFI_SOCKET_BUFFER_BYTESIZE 2060

#define SOCKET_INVALID    0xFF
#define CONN_ID_INVALID   5

#define WIFI_ENABLE_Pin GPIO_PIN_15
#define WIFI_ENABLE_GPIO_Port GPIOD

#endif
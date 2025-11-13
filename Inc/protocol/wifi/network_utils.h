#ifndef __NETWORK_UTILS_H__
#define __NETWORK_UTILS_H__

#include <string.h>

#define MAC_BYTESIZE 6

#define MAC_ADDRESS( mac ) mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]

#define IP( a, b, c, d )  ( a << 24 | b << 16 | c << 8 | d )
#define IP_ADDRESS( ip ) ( ip >> 24 ) & 0xFF, ( ip >> 16 ) & 0xFF, ( ip >> 8 ) & 0xFF, ip & 0xFF
#define IP2STR( ip, str ) sprintf( str, "%hhu.%hhu.%hhu.%hhu", IP_ADDRESS( ip ) )

#define WIFI_CONFIG_MAC_SET_NULL(x) do { \
  x[0] = 0;                              \
  x[1] = 0;                              \
  x[2] = 0;                              \
  x[3] = 0;                              \
  x[4] = 0;                              \
  x[5] = 0;                              \
} while (0);

#define WIFI_CONFIG_MAC_IS_NULL(x) (x[0] == 0 && x[1] == 0 && x[2] == 0 && x[3] == 0 && x[4] == 0 && x[5] == 0)

#endif
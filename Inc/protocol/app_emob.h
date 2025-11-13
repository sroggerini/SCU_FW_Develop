#ifndef __APP_EMOB_H__
#define __APP_EMOB_H__

#include "cmsis_os.h"
#include "task.h"
#include "telnet.h"
#include "wifi.h"

#define LOG_SUCCESS ANSI_COLOR_GREEN "SUCCESS" ANSI_COLOR_RESET
#define LOG_FAIL ANSI_COLOR_RED "FAIL" ANSI_COLOR_RESET
#define LOG tPrintf

#define WIFI_TASK_POLLING_MS 1000

#define STATION_ADVERTISING_BYTESIZE      34
#define STATION_ADVERTISING_POLLING_MS    5000
#define STATION_ADVERTISING_IP            IP( 255, 255, 255, 255 )
#define STATION_ADVERTISING_PORT          4023
#define STATION_ADVERTISING_NAME_BYTESIZE 14

#define STATION_SERVER_PORT               333

typedef enum
{
  APPEMOB_NOTIFY_PROCEDURE_INIT,
  APPEMOB_NOTIFY_PROCEDURE_EXECUTE,
  APPEMOB_NOTIFY_PROCEDURE_RESULT_READ,
  APPEMOB_NOTIFY_PROCEDURE_ABORT,
  APPEMOB_NOTIFY_TRANSACTION_START,
  APPEMOB_NOTIFY_SESSION_START,
  APPEMOB_NOTIFY_SESSION_PAUSE,
  APPEMOB_NOTIFY_TRANSACTION_STOP,
  APPEMOB_NOTIFY_TRANSACTION_ABORT,
  APPEMOB_NOTIFY_PROCEDURE_MODE_CHANGED
} AppEmobEvent;

int32_t AppEmobTask_start( void );

int32_t AppEmobTask_stop( void );

int32_t AppEmobTask_setWifiChannel( uint8_t channel );

uint8_t AppEmobTask_getWifiChannel( void );

void AppEmob_notify( AppEmobEvent event, void *args );

void AppEmobTask_printWifiInfo( void );

/**
 * @brief Retrieves the module version information.
 *
 * This function populates the provided Version structure with
 * the current version of the module. It ensures that the caller
 * can access the version details for diagnostic or informational purposes.
 *
 * @param[out] version Pointer to a Version structure that will be
 *                     populated with the module's version information.
 *                     The pointer must not be NULL.
 *
 * @note Ensure the Version structure is properly allocated by the caller
 *       before invoking this function.
 */
void AppEmobTask_getModuleVersion( Version * const version );

/**
 * @brief Updates the module to the latest version by OTA mode.
 *
 * This function attempts to update the module firmware to the latest version by OTA mode.
 * It returns a status code indicating the outcome of the operation.
 *
 * @retval 0 The update completed successfully, and the module is now at the latest version.
 * @retval 1 The update was not initiated because the module is already at the latest version.
 * @retval < 0 The update failed due to an error (e.g., communication issues, hardware failure, or other errors).
 *
 * @note Ensure the system is ready for an update before calling this function.
 */
int32_t AppEmobTask_updateModule( void );

/**
 * @brief Retrieves the RSSI value of the connected WiFi network.
 *
 * This function returns the RSSI (Received Signal Strength Indicator) value
 * of the WiFi network the device is currently connected to. If the device
 * is not connected to a WiFi network, the function returns 0.
 *
 * @retval The RSSI value (in dBm) if connected to a WiFi network.
 * @retval 0 if not connected to a WiFi network.
 *
 * @note RSSI is typically a negative value; values closer to 0 indicate
 *       a stronger signal.
 */
int8_t AppEmobTask_getWifiRssi( void );

int32_t AppEmobTask_connectToWiFi( char *ssid, char *pass );

int32_t AppEmobTask_disconnectFromWiFi( void );

#endif
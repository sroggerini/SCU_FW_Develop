#include "app_emob.h"
#include "wrapper.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "eeprom.h"
#include "modbus.h"
#include "scuMdb.h"
#include "ExtInpMng.h"
#include "httpserver-socket.h"
#include "wrapper.h"
#include "transaction_register.h"
#include "AutotestMng.h"
#include "secure_area.h"

#define mdb ( &modbus )

static osThreadId_t appEmobTaskHandle;
static const osThreadAttr_t appEmobTask_attributes = {
  .name = "AppEmobTask",
  .stack_size = 128 * 7,
  .priority = (osPriority_t) osPriorityNormal,
};

static osThreadId_t advertisingTaskHandle;
static const osThreadAttr_t advertisingTask_attributes = {
  .name = "appAdvertisingTask",
  .stack_size = 128 * 5,
  .priority = (osPriority_t) osPriorityNormal,
};

static osThreadId_t serverTaskHandle;
static const osThreadAttr_t serverTask_attributes = {
  .name = "appServerTask",
  .stack_size = 128 * 7,
  .priority = (osPriority_t) osPriorityNormal,
};

static const osThreadAttr_t clientTask_attributes = {
  .name = "appClientTask",
  .stack_size = 128 * 10,
  .priority = (osPriority_t) osPriorityNormal,
};

static osThreadId_t appEmobProcedureTask;
static const osThreadAttr_t appEmobProcedureTask_attributes = {
  .name = "appEmobProcedureTask",
  .stack_size = 128 * 7,
  .priority = (osPriority_t) osPriorityNormal,
};

static osEventFlagsId_t procedureEventFlags;
static osMessageQueueId_t procedureQueue;

#define APPEMOB_ANTENNATEST_NETWORK_MAX_NUM   3
#define APPEMOB_ANTENNATEST_THRESHOLD         -70

#define APPEMOB_PROCEDURE_FLAG_EVSE_NOTIFY    ( 1U << 0 )
#define APPEMOB_PROCEDURE_FLAG_EVSE_STOP      ( 1U << 1 )
#define APPEMOB_PROCEDURE_FLAG_STOP           ( 1U << 2 )
#define APPEMOB_PROCEDURE_FLAG_ALL            ( APPEMOB_PROCEDURE_FLAG_EVSE_NOTIFY        | \
                                                APPEMOB_PROCEDURE_FLAG_EVSE_STOP          | \
                                                APPEMOB_PROCEDURE_FLAG_STOP               )

#define APPEMOB_STATION_SERIAL_MAX_LEN  9
#define APPEMOB_STATION_NAME_MAX_LEN    14
#define APPEMOB_TAG_UID_MAX_LEN         10
#define APPEMOB_AP_SSID_MAX_LEN         32
#define APPEMOB_AP_PASS_MAX_LEN         64

#define APPEMOB_FIRMWARE_DATA_BYTESIZE      2048
#define APPEMOB_FIRMWARE_OVERHEAD_BYTESIZE  4
#define APPEMOB_FIRMWARE_PACKET_BYTESIZE    ( APPEMOB_FIRMWARE_DATA_BYTESIZE + APPEMOB_FIRMWARE_OVERHEAD_BYTESIZE )

typedef __packed struct
{
  uint16_t id;
  uint8_t data[ APPEMOB_FIRMWARE_DATA_BYTESIZE ];
  uint16_t check;
} AppEmobFirmwarePacket;

const WifiConfig AP_FOR_MODULE_UPDATE = { .ssid = "LAB-EM", .pass = "SwissArmyKnife22", .mac = {0,0,0,0,0,0} };
const Version moduleFirmwareVer = { 3, 3, 0, 0 };

typedef __packed struct
{
  uint16_t rswCurr;
  uint16_t evsePwrMode;
  uint32_t maxPower;
  uint16_t maxTypCurr;
  uint16_t evseMod;
  uint8_t  mfwVer[LEN_BOARD_FW_VERSION_RO * 2];
  uint8_t  rfdVer[LEN_MIFARE_FW_VERSION_RO * 2];
  uint16_t mtType;
  uint16_t connectStatus;
  uint8_t  RoutSSID[LEN_ROUTER_NET_SSID_RW * 2];
  uint16_t schFlag1[20];
  uint16_t menuVisible;
  uint16_t pmImin;
  uint16_t pmPmax;
  uint16_t pmFlags;
  uint16_t pmHpower;
  uint16_t pmDset;
  uint16_t pmDmax;
  uint16_t pmMode;
  uint16_t hwChecks1;
  uint16_t hwChecks2;
  uint16_t hwActuators;
} AppEmobDataAccess;

#define APPEMOB_FLAG_WIFI_ACTIVE          ( 1U << 0 )
#define APPEMOB_FLAG_STATION_ACTIVE       ( 1U << 1 )
#define APPEMOB_FLAG_AP_CREATED           ( 1U << 2 )
#define APPEMOB_FLAG_STATION_CONNECTED    ( 1U << 3 )

typedef struct
{
  uint8_t connectorId;
  uint8_t connectorType;
  uint32_t serialStation;
  char stationName[ APPEMOB_STATION_NAME_MAX_LEN + 1 ];
  uint8_t stationMode;
  char uid[ APPEMOB_TAG_UID_MAX_LEN + 1 ];
  uint8_t localApChannel;
  /* Station data model */
  scuRoMapRegister_st *ro;
  scuRwMapRegister_st *rw;
  appMapRwRegister_st *app;
  /* Support data model */
  AppEmobDataAccess dataAccess;
} AppEmobDataModel;


#define APPEMOB_MAX_CLIENTS_NUM 2

#define APPEMOB_CLIENT_FLAG_LOGIN           ( 1U << 0 )
#define APPEMOB_CLIENT_FLAG_UPDATE_FW_REQ   ( 1U << 1 )

typedef enum
{
  CLIENT_OPERATION_NONE,
  CLIENT_OPERATION_TRANSACTION_REQUEST,
  CLIENT_OPERATION_FW_UPDATE
} ClientOperationType;

#define INVALID_CLIENT            -1
#define CLIENT_MAX_REQ_BYTESIZE   APPEMOB_FIRMWARE_PACKET_BYTESIZE
#define CLIENT_MAX_RES_BYTESIZE ( MODBUS_PDU_MAX_BYTESIZE + 7 ) // Modbus PDU max len + Modbus TCP header

typedef struct ClientCtl ClientCtl;
struct ClientCtl
{
  int32_t id;
  uint32_t ip;
  uint16_t port;
  uint32_t flags;
  uint8_t req[ CLIENT_MAX_REQ_BYTESIZE ];
  uint8_t res[ CLIENT_MAX_RES_BYTESIZE ]; 
  int32_t recLen;
};

typedef struct
{
  int32_t id;
  ClientCtl clients[ APPEMOB_MAX_CLIENTS_NUM ];
  uint32_t count;
  void *mutex;
} ServerCtl;

typedef struct
{
  /* Data model */
  AppEmobDataModel db;
  /* Control flags */
  uint32_t flags;
  /* Support update firmware */
  uint16_t pcktId;
  uint16_t maxPckt;
  uint8_t fwPacket[ APPEMOB_FIRMWARE_PACKET_BYTESIZE ];
  uint32_t fwPcktLen;
} AppEmobCtl;

typedef __packed struct
{
  uint16_t bytes;
  NetworkScanInfo ap[];
} AppScanApInfo;

static ServerCtl serverCtl;
#define server ( &serverCtl )

static AppEmobCtl appEmobCtl;
#define appemob ( &appEmobCtl )
#define db      ( &appEmobCtl.db )

static void WifiUserNotifyEvent( uint32_t event, void *args );

static void AdvertisingTask( void *argument );
static void AdvertisingTask_prepareMessage( char *msg );

static void ServerTask( void *argument );
static int32_t Server_init( ServerCtl * const s, int32_t id );
static int32_t Server_addClient( ServerCtl * const s, int32_t id, uint32_t ip, uint16_t port );

static void ClientTask( void *argument );
static int32_t ClientTask_accept( ClientCtl * const client );
static int32_t Client_init( ClientCtl * const c );
static int32_t Client_removeFromServer( ClientCtl * const c, ServerCtl * const s );

static int32_t processModbusRequest( ClientCtl * const client );
static int32_t processFirmwareRequest( ClientCtl * const client );

static void AppEmobProcedureTask( void *argument );

static char stationAdvertising[ STATION_ADVERTISING_BYTESIZE ];

extern sFLASH_Info sFLASH_Information;

uint16_t uint16_swapWord( uint16_t word )
{
  return ( word >> 8 ) | ( word << 8 );
}
#define UINT16_SWAP( w ) uint16_swapWord( w )

#define SCU_CONFIG_MODBUS_REGS_LIST                                     \
  ITEM( CONNECTOR_TYPES,            0x0000,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( HW_PRESENCE_FLAGS,          0x0001,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( ENERGY_METERS,	            0x0002,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( POWER_OUTAGE_TYPE,          0x0003,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( CONNECTOR_IDS,              0x0004,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( BOARD_SN,                   0x0005,	 8, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( PRODUCT_SN,                 0x000D,	 8, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( MASTER_HARDWARE_TYPE,       0x0015,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( UTC_DATE_TIME,              0x0016,	 9, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( POWER_MODULE_BRAND,	        0x001F,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( POWER_MODULE_MODEL,	        0x0020,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( POWER_MODULE_N,             0x0021,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( AUTOCONFIG_FUNCTION,        0x0022,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( EVSE_OPERATION_MODE,        0x0023,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( CHARGE_PROTOCOL_SELECTED,   0x0024,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( DISPLAY_DEFAULT_LANGUAGE,   0x0025,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( HW_CHECKS1,                 0x0026,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( HW_CHECKS2,                 0x0027,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( HW_ACTUATORS,               0x0028,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( LIM_MAX_CURRENT_DC,         0x0029,	 2, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( LIM_MIN_CURRENT_DC,         0x002B,	 2, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( LIM_MAX_VOLTAGE_DC,         0x002D,	 2, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( LIM_MIN_VOLTAGE_DC,         0x002F,	 2, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( MAX_TEMPORARY_POWER_DC,	    0x0031,	 2, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( LED_STRIP,                  0x0034,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( REMOTE_COMMANDS,            0x0035,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( LIM_MAX_POWER_DC,           0x0037,	 2, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( MAX_TEMPORARY_POWER_AC,	    0x003A,	 2, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( PM_MODE,                    0x003C,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( ENABLE_LOG,                 0x003D,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( AUTHORIZATION_FEEDBACK,     0x003E,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( TIMEZONE,                   0x003F,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( MENU_VISIBILITY,            0x0040,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( TIMEOUT_RANGE1,             0x0041,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( TIMEOUT_RANGE2,             0x0042,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( PRODUCT_CODE,               0x0043,	12, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( FAKE_PRODUCT_CODE,          0x004F,	10, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( CONNECTOR_N,                0x0059,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( DISPLAY_LANGUAGES,          0x005A,	 2, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( MAX_TYPICAL_CURRENT,        0x005C,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( MAX_SIMPLIFIED_CURRENT,     0x005D,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( CHARGE_BY_TIME,             0x005E,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( CHARGE_BY_ENERGY,           0x005F,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( PM_IMIN,                    0x0060,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( PM_PMAX,                    0x0061,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( PM_FLAGS,                   0x0062,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( PM_HPOWER,                  0x0063,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( PM_DSET,                    0x0064,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( PM_DMAX,                    0x0065,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( PIN_EXTERNAL_ENERGY_METER,  0x0066,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( LIM_MAX_POWER_AC,           0x0067,	 2, MODBUS_REGISTER_FLAG_RW )   

static const ModbusRegister scuConfigRegisters[] = {
#define ITEM( c, a, q, f) { .address = a, .quantity = q, .flags = f },
  SCU_CONFIG_MODBUS_REGS_LIST
#undef ITEM
};

#define SCU_UPDATE_MODBUS_REGS_LIST                              \
  ITEM( FILE_COMMAND,       0x0100,	 1, MODBUS_REGISTER_FLAG_RW  ) \
  ITEM( FILE_SIZE,          0x0101,	 2, MODBUS_REGISTER_FLAG_RW  ) \
  ITEM( FILE_PACKET_NUMBER, 0x0103,	 1, MODBUS_REGISTER_FLAG_RW  )

static const ModbusRegister scuUpdateRegisters[] = {
#define ITEM( c, a, q, f) { .address = a, .quantity = q, .flags = f },
  SCU_UPDATE_MODBUS_REGS_LIST
#undef ITEM
};

#define SCU_SETUP_MODBUS_REGS_LIST                               \
  ITEM( BOARD_FW_VERSION,   0x0300,	12, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( MIFARE_FW_VERSION,  0x030C,	10, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( BOARD_SN_OTP,	      0x0316,	 8, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( PLC_FW_VERSION,     0x031E,	12, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( PLC_MAC_ADDRESS,    0x032A,	10, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( HW_REVISION,	      0x0334,	 8, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( WIFI_MAC_ADDRESS,	  0x033C,	10, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( POWER_MODULE_SN,	  0x0346,	10, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( BLE_MAC_ADDRESS,    0x0350,	10, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( PRODUCT_SN_OTP,     0x035A,	 8, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( PRODUCT_CODE_OTP,	  0x0362,	12, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( BOOT_FW_VERSION,	  0x036E,	12, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( MODBUS_VERSION,	    0x037A,	 1, MODBUS_REGISTER_FLAG_R  )     

static const ModbusRegister scuSetupRegisters[] = {
#define ITEM( c, a, q, f) { .address = a, .quantity = q, .flags = f },
  SCU_SETUP_MODBUS_REGS_LIST
#undef ITEM
};

#define SCU_NOTIFY_MODBUS_REGS_LIST                                 \
  ITEM( NOTIFY_EVSE_PRESENCE,   0x0400,	 2, MODBUS_REGISTER_FLAG_R) \
  ITEM( NOTIFY_EVSE_CHANGE,     0x0402,	 2, MODBUS_REGISTER_FLAG_R) \
  ITEM( EVSE_ERRORS1,           0x0404,	 1, MODBUS_REGISTER_FLAG_R) \
  ITEM( EVSE_ERRORS2,           0x0405,	 1, MODBUS_REGISTER_FLAG_R) \
  ITEM( EVSE_CHANGE_REGISTERS,  0x0406,	32, MODBUS_REGISTER_FLAG_R)
    
static const ModbusRegister scuNotifyRegisters[] = {
#define ITEM( c, a, q, f) { .address = a, .quantity = q, .flags = f },
  SCU_NOTIFY_MODBUS_REGS_LIST
#undef ITEM
};

#define APPEMOB_ACCESS_MODBUS_REGS_LIST                             \
  ITEM( APPEMOB_EXCHANGE_AREA, 0x0500, 77, MODBUS_REGISTER_FLAG_R )
    
static const ModbusRegister appExchangeAreaRegisters[] = {
#define ITEM( c, a, q, f) { .address = a, .quantity = q, .flags = f },
  APPEMOB_ACCESS_MODBUS_REGS_LIST
#undef ITEM
};

#define SCU_EVENTS_MODBUS_REGS_LIST                                                 \
  ITEM( SOCKET_EVENT_FLAGS,                   0x0600,	 1, MODBUS_REGISTER_FLAG_R )  \
  ITEM( EVSE_CONNECTOR_STATUS,	              0x0601,	 1, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TEMPERATURE_SYSTEM,	                  0x0602,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TEMPERATURE_CCS_DCp,	                0x0604,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TEMPERATURE_CCS_DCn,	                0x0606,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TEMPERATURE_AUX1,	                    0x0608,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TEMPERATURE_AUX2,	                    0x060A,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( HUMIDITY_SYSTEM,	                    0x060C,	 1, MODBUS_REGISTER_FLAG_R )  \
  ITEM( ACCELEROMETER_EVENTS,	                0x060D,	 1, MODBUS_REGISTER_FLAG_R )  \
  ITEM( VOLTAGE_AC,	                          0x060E,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( CURRENT_AC_L1,	                      0x0610,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( CURRENT_AC_L2,	                      0x0612,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( CURRENT_AC_L3,	                      0x0614,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( POWER_DC,	                            0x0616,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( ACTIVE_POWER_AC,	                    0x0618,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( ACTIVE_POWER_AC_L1,	                  0x061A,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( REACTIVE_POWER_AC,	                  0x061C,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TOTAL_ACTIVE_ENERGY_AC_EV,	          0x061E,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TOTAL_ENERGY_DC_EV,	                  0x0620,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TOTAL_REACTIVE_ENERGY_AC_EV,	        0x0622,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TRANSACTION_ACTIVE_ENERGY_AC_EV,	    0x0624,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( ACTIVE_POWER_AC_L2,	                  0x0626,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TRANSACTION_REACTIVE_ENERGY_AC_EV,    0x0628,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( FREQUENCY_SUPPLY,                     0x062A,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TEMPERATURE_POWER_MODULE,             0x062C,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( FAN_SPEED,                            0x062E,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( VOLTAGE_DC,                           0x0630,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( CURRENT_DC,                           0x0632,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( POWER_MODULE_ERROR_DETAILS,	          0x0634,	 1, MODBUS_REGISTER_FLAG_R )  \
  ITEM( MAX_CURRENT_ROTATORY_SWITCH,	        0x0635,	 1, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TRANSACTION_ENERGY_DC_EV,	            0x0636,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( ACTIVE_POWER_AC_L3,                   0x0638,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( MAX_CURRENT_RESULT,                   0x063B,	 1, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TRANSACTION_TIME,                     0x063C,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TRANSACTION_COUNTDOWN,                0x063E,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( EV_BATTERY,                           0x0640,	 1, MODBUS_REGISTER_FLAG_R )  \
  ITEM( SESSION_ID,                           0x0641,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( EVSE_BOOT_EVENTS,                     0x0643,	 1, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TOTAL_ACTIVE_ENERGY_AC_GRID,	        0x0644,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TOTAL_REACTIVE_ENERGY_AC_GRID,	      0x0646,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TRANSACTION_ACTIVE_ENERGY_AC_GRID,	  0x0648,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TRANSACTION_REACTIVE_ENERGY_AC_GRID,  0x064A,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TOTAL_ENERGY_DC_GRID,                 0x064C,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TRANSACTION_ENERGY_DC_GRID,           0x064E,	 2, MODBUS_REGISTER_FLAG_R )  \
  ITEM( UID_AUTHORIZATION,                    0x0650,	24, MODBUS_REGISTER_FLAG_R )  \
  ITEM( TIME_INCHARGE,                        0x0668,	 2, MODBUS_REGISTER_FLAG_R )

static const ModbusRegister scuEventsRegisters[] = {
#define ITEM( c, a, q, f) { .address = a, .quantity = q, .flags = f },
  SCU_EVENTS_MODBUS_REGS_LIST
#undef ITEM
};
    
#define APP_EMOB_MODBUS_REGS_LIST                                           \
  ITEM( CONNECTOR_INIT_STATUS,        0x8200,	 1, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( CONNECTOR_OPERATION_STATUS,   0x8201,	 1, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( PROCEDURE_RESPONSE,		        0x8202,	 2, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( PM_TOTAL_POWER,		            0x8204,	 2, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( PM_L1_POWER,		              0x8206,	 2, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( PM_L2_POWER,		              0x8208,	 2, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( PM_L3_POWER,		              0x820A,	 2, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( APP_LOGIN_USER_TYPE,		      0x820C,	 1, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( CONNECTOR_CHECK_ACTIV_KEY,	  0x820D,	 3, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( CONNECTOR_NAME,		            0x8210,	 7, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( CONNECTOR_PIN,		            0x8217,	 3, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( ROUTER_NET_SSID,		          0x821A,	16, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( ROUTER_NET_PASSWORD,		      0x822A,	32, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( CONNECTOR_ROUTER_IP,		      0x824A,	 8, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( PROCEDURE_REQUEST,		        0x8252,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( SCHEDULE_FLAG1,		            0x8253,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( SCHEDULE_START1,		          0x8254,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( SCHEDULE_STOP1,	              0x8255,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( SCHEDULE_POWER1,		          0x8256,	 2, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( SCHEDULE_FLAG2,		            0x8258,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( SCHEDULE_START2,		          0x8259,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( SCHEDULE_STOP2,		            0x825A,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( SCHEDULE_POWER2,		          0x825B,	 2, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( SCHEDULE_FLAG3,		            0x825D,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( SCHEDULE_START3,		          0x825E,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( SCHEDULE_STOP3,		            0x825F,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( SCHEDULE_POWER3,		          0x8260,	 2, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( SCHEDULE_FLAG4,		            0x8262,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( SCHEDULE_START4,		          0x8263,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( SCHEDULE_STOP4,		            0x8264,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( SCHEDULE_POWER4,		          0x8265,	 2, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( CONNECTOR_CONNECTION_STATUS,  0x8267,	 1, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( MAX_POWER,		                0x8268,	 2, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( RANDOM_DELAY_FLAG,		        0x826A,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( UID_AUTHORIZATION_APP,		    0x826B,	 5, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( BT_SINAPSI_STATUS,		        0x8270,	 1, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( FW_UPDATE_NAME,		            0x8271,	 9, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( FW_UPDATE_LEN,		            0x827A,	 2, MODBUS_REGISTER_FLAG_RW ) \
  ITEM( FW_UPDATE_STATUS,		          0x827C,	 1, MODBUS_REGISTER_FLAG_R  ) \
  ITEM( EVSE_POWER_MODE,		          0x827D,	 1, MODBUS_REGISTER_FLAG_R  )

static const ModbusRegister appModbusRegisters[] = {
#define ITEM( c, a, q, f) { .address = a, .quantity = q, .flags = f },
  APP_EMOB_MODBUS_REGS_LIST
#undef ITEM
};

static ModbusTable appEmobModbusTables[] = {
  { .start = 0x0000,  .end = 0x0069,  .regs = scuConfigRegisters,       .regsCount = ( sizeof( scuConfigRegisters ) / sizeof( scuConfigRegisters[ 0 ] ) ),              .mem = NULL },
  { .start = 0x0100,  .end = 0x0104,  .regs = scuUpdateRegisters,       .regsCount = ( sizeof( scuUpdateRegisters ) / sizeof( scuUpdateRegisters[ 0 ] ) ),              .mem = NULL },
  { .start = 0x0300,  .end = 0x037B,  .regs = scuSetupRegisters,        .regsCount = ( sizeof( scuSetupRegisters ) / sizeof( scuSetupRegisters[ 0 ] ) )  ,              .mem = NULL },
  { .start = 0x0400,  .end = 0x0426,  .regs = scuNotifyRegisters,       .regsCount = ( sizeof( scuNotifyRegisters ) / sizeof( scuNotifyRegisters[ 0 ] ) ),              .mem = NULL },
  { .start = 0x0500,  .end = 0x054E,  .regs = appExchangeAreaRegisters, .regsCount = ( sizeof( appExchangeAreaRegisters ) / sizeof( appExchangeAreaRegisters[ 0 ] ) ),  .mem = NULL },
  { .start = 0x0600,  .end = 0x066A,  .regs = scuEventsRegisters,       .regsCount = ( sizeof( scuEventsRegisters ) / sizeof( scuEventsRegisters[ 0 ] ) ),              .mem = NULL },
  { .start = 0x8200,  .end = 0x827E,  .regs = appModbusRegisters,       .regsCount = ( sizeof( appModbusRegisters ) / sizeof( appModbusRegisters[ 0 ] ) ),              .mem = NULL }
};
    
typedef enum
{
#define ITEM( c, a, q, f ) c,
  SCU_CONFIG_MODBUS_REGS_LIST
#undef ITEM
#define ITEM( c, a, q, f ) c,
  SCU_UPDATE_MODBUS_REGS_LIST
#undef ITEM
#define ITEM( c, a, q, f ) c,
  SCU_SETUP_MODBUS_REGS_LIST
#undef ITEM
#define ITEM( c, a, q, f ) c,
  SCU_NOTIFY_MODBUS_REGS_LIST
#undef ITEM
#define ITEM( c, a, q, f ) c,
  APPEMOB_ACCESS_MODBUS_REGS_LIST
#undef ITEM
#define ITEM( c, a, q, f ) c,
  SCU_EVENTS_MODBUS_REGS_LIST
#undef ITEM
#define ITEM( c, a, q, f ) c,
  APP_EMOB_MODBUS_REGS_LIST
#undef ITEM
  APP_EMOB_MODBUS_REG_UNKNOWN
} AppEmobDataId;

typedef enum
{
  PROCEDURE_NONE = 0x00,
  PROCEDURE_CHECK_ACTIVATION_KEY,
  PROCEDURE_INIT_SOCKET,
  PROCEDURE_MODIFY_USER_PIN,
  PROCEDURE_LOGIN,
  PROCEDURE_ROUTER_CONNECTION,
  PROCEDURE_ROUTER_DISCONNECTION,
  PROCEDURE_MODIFY_STATION_NAME,
  PROCEDURE_UPDATE_FW,
  PROCEDURE_START_TRANSACTION,
  PROCEDURE_SUSPEND_TRANSACTION,
  PROCEDURE_RELEASE_SUSPENSION,
  PROCEDURE_STOP_TRANSACTION,
  PROCEDURE_MODIFY_SCHEDULE,
  PROCEDURE_TRANSACTION_REQUEST,
  PROCEDURE_FORCE_REBOOT,
  PROCEDURE_FACTORY_RESET,
  PROCEDURE_SINAPSI_BT_START,
  PROCEDURE_CHANGE_WORKING_MODE,
  PROCEDURE_SETTING_TIME,
  PROCEDURE_DELETE_TRANSACTION,
  PROCEDURE_SET_MAXIMUM_CURRENT,
  PROCEDURE_SET_POWER_MANAGEMENT,
  PROCEDURE_SET_CHARGE_BY_TIME,
  PROCEDURE_SET_LANGUAGES,
  PROCEDURE_SET_CHECKS_ACTUATORS,
  PROCEDURE_RESET_SCHEDULATIONS,
  PROCEDURE_SCAN_AP,
  PROCEDURE_UNKNOWN                                              
} ProcedureType;

typedef enum
{
  PROCEDURE_STATUS_NONE,
  PROCEDURE_STATUS_RUNNING,
  PROCEDURE_STATUS_TERMINATED
} ProcedureStatus;

typedef enum
{
  PROCEDURE_RESULT_NONE,
  PROCEDURE_RESULT_SUCCESS,
  PROCEDURE_RESULT_EXCEPTION
} ProcedureResult;

typedef enum
{
  PROCEDURE_EXCEPTION_NONE,
  PROCEDURE_EXCEPTION_INTERNAL_ERROR,
  PROCEDURE_EXCEPTION_INVALID,
  PROCEDURE_EXCEPTION_REJECTED,
  PROCEDURE_EXCEPTION_OCCUPIED,
  PROCEDURE_EXCEPTION_EXPIRED,
} ProcedureException;

typedef struct
{
  uint8_t procedure;
  uint8_t status;
  uint8_t result;
  uint8_t exception;
} ProcedureResultRegister;

typedef struct
{
  ClientCtl *client;
  ProcedureType type;
  ProcedureStatus status;
  ProcedureResult result;
  ProcedureException exception;
} Procedure;

static const uint8_t stationNonActive = '?';
static const uint8_t stationActive = '~';
static const char stationUdpCheck[] = "SCWBSK";

static void AppEmobTask( void *argument );

static void AppEmobTask_initDataModel( void );
static void AppEmobTask_initDataAccess( void );
static void AppEmobTask_connectWiFiAfterReset( void );

#define APPEMOB_PROCEDURE_TIMEOUT_MS  10000
#define APPEMOB_SOSPENSION_TIMEOUT_MS 40000
#define APPEMOB_AUTH_WAIT_TIMEOUT_MS  70000
#define APPEMOB_SCAN_AP_MAX_COUNT     30

static void AppEmobTask_startProcedure    ( Procedure * const procedure );
static void AppEmobTask_executeProcedure  ( Procedure * const procedure );
static void AppEmobTask_stopProcedure     ( Procedure * const procedure );
static void AppEmobTask_postProcedure     ( Procedure * const procedure );
static void AppEmobTask_clearProcedure    ( void );

static void AppEmobTask_initFirmwareUpdate( ClientCtl * const client );

static int32_t AppEmobTask_saveStationName( char *name, uint32_t len );
static int32_t AppEmobTask_checkStationName( char *name, uint32_t len );
static int32_t AppEmobTask_savePowerManagementParams( void );
static int32_t AppEmobTask_savePowerManagementFlags( uint16_t flags );
static int32_t AppEmobTask_savePowerManagementPmax( uint16_t power );
static int32_t AppEmobTask_savePowerManagementImin( uint16_t current );
static int32_t AppEmobTask_savePowerManagementHpow( uint16_t percent );
static int32_t AppEmobTask_savePowerManagementDset( uint16_t power );
static int32_t AppEmobTask_savePowerManagementDmax( uint16_t percent );
static int32_t AppEmobTask_savePowerManagementMode( uint16_t mode );
static int32_t AppEmobTask_savePowerManagementEmex( bool enable );

static int32_t AppEmobTask_filterApNetworks( NetworkScanInfo *aps, uint32_t len );

int32_t AppEmobTask_start( void )
{
  int32_t res = -1;
  
  if( ( appemob->flags & APPEMOB_FLAG_WIFI_ACTIVE ) == 0 )
  {
    appEmobTaskHandle = osThreadNew( AppEmobTask, &wifi, &appEmobTask_attributes );
    res = 0;
  }
  
  return res;
}

int32_t AppEmobTask_stop( void )
{
  int32_t res = -1;
  
  if( appemob->flags & APPEMOB_FLAG_WIFI_ACTIVE )
  {
    osThreadTerminate( appEmobTaskHandle );
    osThreadTerminate( advertisingTaskHandle );
    osThreadTerminate( serverTaskHandle );
    res = wifi.turnOff();
    if( res == 0 )
    {
      appemob->flags &= ~APPEMOB_FLAG_WIFI_ACTIVE;
    }
  }
  
  return res;
}

void AppEmobTask( void *argument )
{
  Wifi *pWifi;
  int32_t status;
  WifiConfig netConfig;
  NetworkInfo netInfo;
  NetworkAddress netAddress;
  
  pWifi = ( Wifi* )argument;
  
  status = pWifi->init( WifiUserNotifyEvent );
  if( status == wifiOK )
  {
    appemob->flags |= APPEMOB_FLAG_WIFI_ACTIVE;
    LOG( "Initialization Wi-Fi layer: " LOG_SUCCESS "\r\n" );
  }
  else
  {
    LOG( "Initialization Wi-Fi layer: " LOG_FAIL "\r\n" );
    goto wifi_kill;
  }
  
  status = pWifi->turnOn();
  if( status == wifiOK )
  {
    LOG( "Wi-Fi module turn on: " LOG_SUCCESS "\r\n" );
  }
  else
  {
    LOG( "Wi-Fi module turn on: " LOG_FAIL "\r\n" );
    goto wifi_kill;
  }
  
  status = pWifi->reconnectionOpt( 1, 0 );
  if( status != wifiOK )
  {
    LOG( "Wi-Fi reconnection to Ap enable: " LOG_FAIL "\r\n" );
  }
  
  /* Initialize data model */
  AppEmobTask_initDataModel();
  
  advertisingTaskHandle = osThreadNew( AdvertisingTask, pWifi, &advertisingTask_attributes );
  if( advertisingTaskHandle == NULL )
  {
    LOG( "Advertising task creation " LOG_FAIL "\r\n" );
  }
  
  serverTaskHandle = osThreadNew( ServerTask, pWifi, &serverTask_attributes );
  if( serverTaskHandle == NULL )
  {
    LOG( "Server task creation " LOG_FAIL "\r\n" );
  }
  
  appEmobProcedureTask = osThreadNew( AppEmobProcedureTask, NULL, &appEmobProcedureTask_attributes );
  if( appEmobProcedureTask == NULL )
  {
    LOG( "Procedure task creation " LOG_FAIL "\r\n" );
  }
  
  AppEmobTask_connectWiFiAfterReset();
  
  for(;;)
  {
    if( pWifi->isConnect() )
    {
      db->app->connectStatus = 1;
      if( ( appemob->flags & APPEMOB_FLAG_STATION_CONNECTED ) == 0 )
      {
        appemob->flags |= APPEMOB_FLAG_STATION_CONNECTED;
        status = pWifi->getNetInfo( &netInfo );
        if( status == 0 )
        {
          strncpy( ( char* )db->app->RoutSSID, netInfo.ssid, APPEMOB_AP_SSID_MAX_LEN );
          LOG( "Wifi interface STATION enabling: " LOG_SUCCESS "\r\n" );
          LOG( "Station connected to \"%s\", channel = %d, rssi = %ddBm, bssid = %02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx\r\n", netInfo.ssid, netInfo.channel, netInfo.rssi, MAC_ADDRESS(netInfo.bssid));
          status = pWifi->getAddress( WIFI_INTERFACE_STATION, &netAddress );
          if( status == 0 )
          {
            LOG( "Station IP %hhu.%hhu.%hhu.%hhu\r\n", IP_ADDRESS( netAddress.ip ) );
          }
          status = pWifi->disable( WIFI_INTERFACE_AP );
          if( status == 0 )
          {
            appemob->flags &= ~APPEMOB_FLAG_AP_CREATED;
            LOG( "Wifi interface AP disabling: " LOG_SUCCESS "\r\n" );
          }
        }
      }
    }
    else
    {
      db->app->connectStatus = 0;
      memset( ( char* )db->app->RoutSSID, 0, APPEMOB_AP_SSID_MAX_LEN );
      appemob->flags &= ~APPEMOB_FLAG_STATION_CONNECTED;
      if( ( appemob->flags & APPEMOB_FLAG_AP_CREATED ) == 0 )
      {
        char ssid[32];
        char pass[64];
        SecureArea_getLocalApSSID(ssid, sizeof(ssid));
        SecureArea_getLocalApPass(pass, sizeof(pass));
        netConfig.ssid = ssid;
        netConfig.pass = pass;
        WIFI_CONFIG_MAC_SET_NULL(netConfig.mac);
        netConfig.enc = WIFI_ENC_WPA2_PSK;
        netConfig.channel = db->localApChannel;
        status = pWifi->enable( WIFI_INTERFACE_AP, &netConfig );
        if( status == 0 )
        {
          appemob->flags |= APPEMOB_FLAG_AP_CREATED;
          LOG( "Wifi interface AP enabling: " LOG_SUCCESS "\r\n" );
          /* If autotest is active, notify the result of the test: if we arrive here, Flash, EEPROM, ioExp are OK  */
          notifyAutotestResult(AUTOTEST_EVENT_DIGITAL_IC, TEST_PASSED);
        }
      }
    }
    osDelay( pdMS_TO_TICKS( WIFI_TASK_POLLING_MS ) );
  }
  
wifi_kill:
  osThreadTerminate( osThreadGetId() );
}

static void AppEmobTask_initDataModel( void )
{
  db->connectorId = getLogicalMdbAddr();
  db->connectorType = getStationSocketType();
  db->serialStation = strtoul( getProductSerialNumber(), NULL, 10 );
  strncpy( db->stationName, getStationName(), APPEMOB_STATION_NAME_MAX_LEN );
  if( getActivationFlag() == 0 )
  {
    appemob->flags &= ~APPEMOB_FLAG_STATION_ACTIVE;
    db->app->connInitStat = 0;
  }
  else
  {
    appemob->flags |= APPEMOB_FLAG_STATION_ACTIVE;
    db->app->connInitStat = 1;
  }
  db->ro = getRoMdbRegs( db->connectorId );
  db->rw = getRwMdbRegs( db->connectorId );
  db->app = getAppMdbRwRegs( db->connectorId );
  /* Initialize Wi-Fi layer info */
  db->localApChannel = getWifiAccessPointChannelId();
  strncpy( ( char* )db->app->connName, db->stationName, APPEMOB_STATION_NAME_MAX_LEN );
  /* Connect station data model with Modbus interface */
  appEmobModbusTables[ 0 ].mem = &db->rw->scuSetRegister;
  appEmobModbusTables[ 1 ].mem = &db->rw->scuSetRegFwUpd;
  appEmobModbusTables[ 2 ].mem = &db->ro->scuMapRegInfoVer;
  appEmobModbusTables[ 3 ].mem = &db->ro->scuMapRegNotify;
  appEmobModbusTables[ 4 ].mem = &db->dataAccess;
  appEmobModbusTables[ 5 ].mem = &db->ro->scuMapRegStatusMeas;
  appEmobModbusTables[ 6 ].mem = db->app;
  /* Init Modbus layer */
  mdb->init( appEmobModbusTables, ( sizeof( appEmobModbusTables ) / sizeof( appEmobModbusTables[ 0 ] ) ) );
}

static void AppEmobTask_connectWiFiAfterReset( void )
{
  int32_t status;
  uint32_t wifiConnRetain;
  char ssid[32];
  char pass[64];
  
  SecureArea_getRemoteApSSID(ssid, sizeof(ssid));
  SecureArea_getRemoteApPass(pass, sizeof(pass));
  
  if( ( strlen( ssid ) != 0 ) && ( strlen( pass ) != 0 ) )
  {
    wifiConnRetain = 0;
    do
    {
      status = AppEmobTask_connectToWiFi( ssid, pass );
      if( status == wifiOK )
      {
        return;
      }
      else if( status == 2 )
      {
        /* Wrong password */
        LOG( "Connection to Wi-Fi \"%s\" error: " ANSI_COLOR_RED "WRONG PASSWORD" ANSI_COLOR_RESET "\r\n", ssid );
        break;
      }
      wifiConnRetain++;
      LOG( "Connection to Wi-Fi \"%s\" attempt %d with error %d: " LOG_FAIL "\r\n", ssid, wifiConnRetain, status );
    }
    while( wifiConnRetain < 3 );
  }
  status = AppEmobTask_disconnectFromWiFi();
  if( status == 0 )
  {
    LOG( "Wifi interface STATION disabling: " LOG_SUCCESS "\r\n" );
  }
  else
  {
    LOG( "Wifi interface STATION disabling: " LOG_FAIL "\r\n" );
  }
}

static void WifiUserNotifyEvent( uint32_t event, void *args )
{
  WifiStation *station;
  int32_t *state;
  
  switch( event )
  {
  case WIFI_EVENT_STATION_CONNECTED:
    station = ( WifiStation* )args;
    LOG( "Station %hhx:%hhx:%hhx:%hhx:%hhx:%hhx " ANSI_COLOR_GREEN "CONNECTED" ANSI_COLOR_RESET "\r\n", MAC_ADDRESS( station->mac ) );
    break;
  case WIFI_EVENT_STATION_GOT_IP:
    station = ( WifiStation* )args;
    LOG( "Station %hhx:%hhx:%hhx:%hhx:%hhx:%hhx " ANSI_COLOR_GREEN "GOT IP" ANSI_COLOR_RESET " %hhu.%hhu.%hhu.%hhu\r\n", MAC_ADDRESS( station->mac ), IP_ADDRESS( station->ip ) );
    break;
  case WIFI_EVENT_STATION_DISCONNECTED:
    station = ( WifiStation* )args;
    LOG( "Station %hhx:%hhx:%hhx:%hhx:%hhx:%hhx " ANSI_COLOR_RED "DISCONNECTED" ANSI_COLOR_RESET "\r\n", MAC_ADDRESS( station->mac ) );
    break;
  case WIFI_EVENT_UPDATE:
    state = ( int32_t* )args;
    LOG( "Wi-Fi module update: " );
    switch( *state )
    {
    case -1:
      LOG( "OTA fails in non-blocking mode" );
      break;
    case 1:
      LOG( "Server found" );
      break;
    case 2:
      LOG( "Connected to the server" );
      break;
    case 3:
      LOG( "Got the upgrade version" );
      break;
    case 4:
      LOG( "Upgrade done" );
      break;
    default:
      break;
    }
    LOG( "\r\n" );
    break;
  }
}

static void AdvertisingTask( void *argument )
{
  Wifi *pWifi;
  int32_t socket;
  int32_t status;
  uint8_t state = 0;
  
  pWifi = ( Wifi* )argument;
  
  for(;;)
  {
    switch( state )
    {
    case 0:
      socket = pWifi->socket( SOCKET_TYPE_DATAGRAM, SOCKET_PROTOCOL_UDP );
      if( socket < 0 )
      {
        LOG( "Advertising UDP socket creation: " LOG_FAIL "\r\n" );
        goto advertising_kill;
      }
      state = 1;
      break;
    case 1:
      status = pWifi->connect( socket, STATION_ADVERTISING_IP, STATION_ADVERTISING_PORT );
      if( status == wifiOK )
      {
        state = 2;
      }
      else
      {
        LOG( "Advertising UDP socket connection: " LOG_FAIL "\r\n" );
      }
      break;
    case 2:
      AdvertisingTask_prepareMessage( stationAdvertising );
      status = pWifi->send( socket, stationAdvertising, STATION_ADVERTISING_BYTESIZE );
      if( status != STATION_ADVERTISING_BYTESIZE )
      {
        LOG( "Advertising send message: " LOG_FAIL "\r\n" );
      }
      break;
    }
    osDelay( pdMS_TO_TICKS( STATION_ADVERTISING_POLLING_MS ) );
  }
  
advertising_kill:
  osThreadTerminate( osThreadGetId() );
}

static void AdvertisingTask_prepareMessage( char *msg )
{
  // Fix for iOS to save station on device db for autologin.
  memset( &msg[ 0 ], ' ', APPEMOB_STATION_NAME_MAX_LEN );
  strcpy( &msg[ 0 ], db->stationName );
  msg[ strlen( db->stationName ) ] = ' ';
  msg[ 14 ] = ( ( appemob->flags & APPEMOB_FLAG_STATION_ACTIVE ) ? stationActive : stationNonActive );
  sprintf( &msg[ 15 ], "%.*d", APPEMOB_STATION_SERIAL_MAX_LEN, db->serialStation );
  msg[ 24 ] = db->connectorType;
  msg[ 25 ] = db->ro->scuMapRegStatusMeas.ntfSktEvent & 0xFF;
  msg[ 26 ] = db->ro->scuMapRegStatusMeas.ntfChgStat & 0xFF;
  msg[ 27 ] = db->connectorId;
  sprintf( &msg[ 28 ], stationUdpCheck );
}

static void ServerTask( void *argument )
{
  Wifi *pWifi;
  int32_t status;
  int32_t id;
  uint32_t ip;
  uint16_t port;
  certificate_t *cert;
  
  pWifi = ( Wifi* )argument;
  
  cert = SecureArea_getServerKey();
  if (cert->content_len != 0) {
    status = pWifi->flash("server_key", cert, cert->content_len + sizeof(certificate_t));
    if (status != 0) {
      LOG( "Server key init: " LOG_FAIL "\r\n" );
    }
  }
  
  cert = SecureArea_getServerCert();
  if (cert->content_len != 0) {
    status = pWifi->flash("server_cert", cert, cert->content_len + sizeof(certificate_t));
    if (status != 0) {
      LOG( "Server certificate init: " LOG_FAIL "\r\n" );
    }
  }
  
  cert = SecureArea_getCaCert();
  if (cert->content_len != 0) {
    status = pWifi->flash("server_ca", cert, cert->content_len + sizeof(certificate_t));
    if (status != 0) {
      LOG( "Server CA certificate init: " LOG_FAIL "\r\n" );
    }
  }
  
  id = pWifi->socket( SOCKET_TYPE_STREAM, SOCKET_PROTOCOL_TLS );
  if( server < 0 )
  {
    LOG( "Server creation: " LOG_FAIL "\r\n" );
    goto server_kill;
  }
  
  status = Server_init( server, id );
  if( status != 0 )
  {
    LOG( "Server init: " LOG_FAIL "\r\n" );
    goto server_kill;
  }
  
  status = pWifi->bind( server->id, 0x00000000, STATION_SERVER_PORT );
  if( status != wifiOK )
  {
    LOG( "Server bind: " LOG_FAIL "\r\n" );
    goto server_kill;
  }
  
  status = pWifi->listen( server->id, APPEMOB_MAX_CLIENTS_NUM );
  if( status != wifiOK )
  {
    LOG( "Server listen: " LOG_FAIL "\r\n" );
    goto server_kill;
  }
  
  for(;;)
  {
    id = pWifi->accept( server->id, &ip, &port );
    if( id < 0 )
    {
      LOG( "Server accept client: " LOG_FAIL "\r\n" );
      goto server_kill;
    }
    else
    {
      status = Server_addClient( server, id, ip, port );
      if( status == 0 )
      {
        osThreadNew( ClientTask, &server->clients[ ( server->count - 1 ) ], &clientTask_attributes );
      }
    }
  }
server_kill:
  osThreadTerminate( osThreadGetId() );
}

static void Server_lock( ServerCtl * const s )
{
  osMutexAcquire( s->mutex, osWaitForever );
}

static void Server_unlock( ServerCtl * const s )
{
  osMutexRelease( s->mutex );
}

static int32_t Server_init( ServerCtl * const s, int32_t id )
{
  int32_t res;
  
  s->id = id;
  s->count = 0;
  for( uint32_t i = 0; i < APPEMOB_MAX_CLIENTS_NUM; i++ )
  {
    Client_init( &s->clients[ i ] );
  }
  s->mutex = osMutexNew( NULL );
  if( s->mutex == NULL )
  {
    res = -1;
  }
  else
  {
    res = 0;
  }
  return res;
}

static int32_t Client_init( ClientCtl * const c )
{
  c->id = INVALID_CLIENT;
  c->flags = 0;
  c->ip = 0;
  c->port = 0;
  memset( c->req, 0, CLIENT_MAX_REQ_BYTESIZE );
  memset( c->res, 0, CLIENT_MAX_RES_BYTESIZE );
  c->recLen = 0;
  return 0;
}

static int32_t Server_addClient( ServerCtl * const s, int32_t id, uint32_t ip, uint16_t port )
{
  int32_t res;
  uint32_t idx;
  ClientCtl *pClient;
  
  Server_lock( s );
  
  for( idx = 0; idx < APPEMOB_MAX_CLIENTS_NUM; idx++ )
  {
    if( s->clients[ idx ].id == INVALID_CLIENT )
      break;
  }
  if( idx != APPEMOB_MAX_CLIENTS_NUM )
  {
    res = 0;
  }
  else
  {
    res = -1;
  }
  
  /* It is possible to add new client */
  if( res == 0 )
  {
    pClient = &s->clients[ idx ];
    /* First client */
    if( s->count == 0 )
    {
      pClient->flags = APPEMOB_CLIENT_FLAG_LOGIN;
    }
    else
    {
      for( idx = 0; idx < APPEMOB_MAX_CLIENTS_NUM; idx++ )
      {
        if( s->clients[ idx ].ip == ip )
          break;
      }
      /* Same ip of connected client -> iOS second socket */
      if( idx != APPEMOB_MAX_CLIENTS_NUM )
      {
        /* Inherit flags of the first client (with firmware update) */
        pClient->flags = s->clients[ idx ].flags;
        s->clients[ idx ].flags &= ~APPEMOB_CLIENT_FLAG_UPDATE_FW_REQ;
      }
    }
    pClient->id = id;
    pClient->ip = ip;
    pClient->port = port;
    s->count++;
  }
  
  Server_unlock( s );
  
  return res;
}

static int32_t Client_removeFromServer( ClientCtl * const c, ServerCtl * const s )
{
  int32_t res;
  uint32_t idx;
  
  Server_lock( s );
  
  if( s->count > 0 )
  {
    for( idx = 0; idx < APPEMOB_MAX_CLIENTS_NUM; idx++ )
    {
      if( ( s->clients[ idx ].ip == c->ip ) && 
          ( s->clients[ idx ].port == c->port ) )
        break;
    }
    /* Client found */
    if( idx != APPEMOB_MAX_CLIENTS_NUM )
    {
      Client_init( &s->clients[ idx ] );
      s->count--;
      res = 0;
    }
    /* Client not found */
    else
    {
      res = -1;
    }
  }
  else
  {
    res = -1;
  }
  
  Server_unlock( s );
  
  return res;
}

static void ClientTask( void *argument )
{
  ClientCtl *client;
  int32_t len;
  Wifi *pWifi;
  uint32_t rec_timeout_ms;
  
  pWifi = &wifi;
  client = ( ClientCtl* )argument;
  rec_timeout_ms = 7000;
  
  LOG( "Socket %hhu.%hhu.%hhu.%hhu:%d" ANSI_COLOR_GREEN " CONNECT" ANSI_COLOR_RESET "\r\n", IP_ADDRESS( client->ip ), client->port );
  ClientTask_accept( client );
  pWifi->setSocketOpt( client->id, SOCKET_OPT_RCVTIMEO, &rec_timeout_ms, sizeof( rec_timeout_ms ) );
  do
  {
    len = pWifi->recv( client->id, client->req, sizeof( client->req ) );
    client->recLen = len;
    if( client->flags & APPEMOB_CLIENT_FLAG_UPDATE_FW_REQ )
    {
      processFirmwareRequest( client );
    }
    else
    {
      processModbusRequest( client );
    }
  }
  while( len >= 0 );
  len = pWifi->close( client->id );
  if( len == 0 )
  {
    LOG( "Socket %hhu.%hhu.%hhu.%hhu:%d" ANSI_COLOR_RED " DISCONNECT" ANSI_COLOR_RESET "\r\n", IP_ADDRESS( client->ip ), client->port );
    Client_removeFromServer( client, server );
    AppEmob_notify( APPEMOB_NOTIFY_PROCEDURE_ABORT, NULL );
    osThreadTerminate( osThreadGetId() );
  }
}

static int32_t ClientTask_accept( ClientCtl * const client )
{
  AppEmobTask_initDataAccess();
  return 0;
}

static void AppEmobTask_initDataAccess( void )
{
  char ssid[32];
  
  SecureArea_getRemoteApSSID(ssid, sizeof(ssid));
  db->dataAccess.rswCurr = db->ro->scuMapRegStatusMeas.rswCurr;
  db->dataAccess.maxTypCurr = db->rw->scuSetRegister.maxTypCurr;
  db->dataAccess.evsePwrMode = db->app->evsePwrMode;
  db->dataAccess.maxPower = db->app->maxPower;
  db->dataAccess.maxTypCurr = db->rw->scuSetRegister.maxTypCurr;
  db->dataAccess.evseMod = db->rw->scuSetRegister.evseMod;
  memcpy(db->dataAccess.mfwVer, db->ro->scuMapRegInfoVer.mfwVer, (LEN_BOARD_FW_VERSION_RO * 2));  // 12 registers      
  memcpy(db->dataAccess.rfdVer, db->ro->scuMapRegInfoVer.rfdVer, (LEN_MIFARE_FW_VERSION_RO * 2)); // 10 registers 
  db->dataAccess.mtType = db->rw->scuSetRegister.mtType;
  db->dataAccess.connectStatus = ( appemob->flags & APPEMOB_FLAG_STATION_CONNECTED ) ? 1 : 0;
  strcpy( ( char* )db->dataAccess.RoutSSID, ssid );
  memcpy(db->dataAccess.schFlag1, (uint8_t *)&db->app->schFlag1, 20 * 2);                      // 20 registers 
  db->dataAccess.menuVisible = db->rw->scuSetRegister.menuVisible;
  memcpy((uint16_t *)&db->dataAccess.pmImin,(uint16_t *)&db->rw->scuSetRegister.pmImin, 6 * 2);  // 6 registers
  db->dataAccess.pmMode = db->rw->scuSetRegister.pmMode;
  db->dataAccess.hwChecks1 = db->rw->scuSetRegister.hwChecks1;
  db->dataAccess.hwChecks2 = db->rw->scuSetRegister.hwChecks2;
  db->dataAccess.hwActuators = db->rw->scuSetRegister.hwActuators;
  eeprom_param_get( EVS_MODE_EADD, (uint8_t*)(&db->stationMode), 1 );
}

static int32_t processModbusRequest( ClientCtl * const client )
{
  int32_t status, len;
  uint8_t function;
  AppEmobDataId dataId;
  Procedure procedure;
  Wifi *pWifi;
  ProcedureResultRegister *reg;
  
  pWifi = &wifi;
  len = 0;
  
  /* Modbus layer */
  
  if( client->recLen > 0 )
  {
    status = mdb->parse( client->req, client->recLen );
    if( status == MODBUS_OK )
    {
      if( db->connectorId == mdb->unit() )
      {
        if( !mdb->exception() )
        {
          status = mdb->find();
          if( status != MODBUS_ERROR )
          {
            dataId = ( AppEmobDataId )status;
            /* Processing read/write registers from/to device memory */
            status = mdb->execute();
            if( status == MODBUS_OK )
            {
              function = mdb->function();
              /* Init App E-Mobility procedure */
              if( function == WRITE_MULTIPLE_REGISTERS )
              {
                if( dataId == PROCEDURE_REQUEST )
                {
                  procedure.client = client;
                  procedure.type = ( ProcedureType )db->app->procReq;
                  AppEmob_notify( APPEMOB_NOTIFY_PROCEDURE_INIT, &procedure );
                }
              }
              len = mdb->response( client->res, sizeof( client->res ) );
              if( len > 0 )
              {
                len = pWifi->send( client->id, client->res, len );
                if( len > 0 )
                {
                  if( function == WRITE_MULTIPLE_REGISTERS )
                  {
                    if( dataId == PROCEDURE_REQUEST )
                    {
                      procedure.client = client;
                      procedure.type = ( ProcedureType )db->app->procReq;
                      AppEmob_notify( APPEMOB_NOTIFY_PROCEDURE_EXECUTE, &procedure );
                    }
                  }
                  else if( function == READ_HOLDING_REGISTERS )
                  {
                    if( dataId == PROCEDURE_RESPONSE )
                    {
                      procedure.client = client;
                      reg = ( ProcedureResultRegister* )&client->res[9];
                      procedure.type = ( ProcedureType )reg->procedure;
                      procedure.status = ( ProcedureStatus )reg->status;
                      procedure.result = ( ProcedureResult )reg->result;
                      procedure.exception = ( ProcedureException )reg->exception;
                      AppEmob_notify( APPEMOB_NOTIFY_PROCEDURE_RESULT_READ, &procedure );
                    }
                  }
                  status = 0;
                }
              }
            }
          }
        }
      }
    }
  }
  return status;
}

void AppEmob_notify( AppEmobEvent event, void *args )
{
  uint32_t status;
  Procedure *procedure;
  
  switch( event )
  {
  case APPEMOB_NOTIFY_PROCEDURE_INIT:
    procedure = ( Procedure* )args;
    if( procedure->type == PROCEDURE_UPDATE_FW )
    {
      AppEmobTask_initFirmwareUpdate( procedure->client );
    }
    else
    {
      AppEmobTask_startProcedure( procedure );
    }
    break;
  case APPEMOB_NOTIFY_PROCEDURE_EXECUTE:
    procedure = ( Procedure* )args;
    status = osMessageQueuePut( procedureQueue, procedure, 0U, 0U );
    if( status != osOK )
    {
      LOG( "Start procedure: " LOG_FAIL "\r\n" );
    }
    break;
  case APPEMOB_NOTIFY_PROCEDURE_ABORT:
    osThreadResume( appEmobTaskHandle );
    osThreadResume( advertisingTaskHandle );
    AppEmobTask_clearProcedure();
    break;
  case APPEMOB_NOTIFY_PROCEDURE_RESULT_READ:
    {
      procedure = ( Procedure* )args;
      if( procedure->status == PROCEDURE_STATUS_TERMINATED )
      {
        AppEmobTask_postProcedure( procedure );
        AppEmobTask_clearProcedure();
      }
    }
    break;
  case APPEMOB_NOTIFY_TRANSACTION_START:
  case APPEMOB_NOTIFY_SESSION_START:
  case APPEMOB_NOTIFY_SESSION_PAUSE:
  case APPEMOB_NOTIFY_TRANSACTION_STOP:
  case APPEMOB_NOTIFY_PROCEDURE_MODE_CHANGED:
    osEventFlagsSet( procedureEventFlags, APPEMOB_PROCEDURE_FLAG_EVSE_NOTIFY );
    break;
  case APPEMOB_NOTIFY_TRANSACTION_ABORT:
    osEventFlagsSet( procedureEventFlags, APPEMOB_PROCEDURE_FLAG_EVSE_STOP );
    break;
  }
}

static uint32_t AppEmobProcedureTask_wait( uint32_t flag, uint32_t timeout )
{
  return osEventFlagsWait( procedureEventFlags, flag, osFlagsWaitAny, pdMS_TO_TICKS( timeout ) );
}

static void AppEmobProcedureTask( void *arguments )
{
  osStatus_t status;
  Procedure procedure;
  
  procedureQueue = osMessageQueueNew( 5, sizeof( Procedure ), NULL );
  if( procedureQueue == NULL )
  {
    LOG( "Procedure queue creation " LOG_FAIL "\r\n" );
    goto procedureTaskKill;
  }
  
  procedureEventFlags = osEventFlagsNew( NULL );
  if( procedureEventFlags == NULL )
  {
    LOG( "Procedure event flags creation " LOG_FAIL "\r\n" );
    goto procedureTaskKill;
  }
  
  for(;;)
  {
    status = osMessageQueueGet( procedureQueue, &procedure, NULL, osWaitForever );
    if( status == osOK )
    {
      AppEmobTask_executeProcedure( &procedure );
      AppEmobTask_stopProcedure( &procedure );
    }
  }
  
procedureTaskKill:
  osThreadTerminate( osThreadGetId() );
}

static void AppEmobTask_notifyProcedure( Procedure * const procedure )
{
  ProcedureResultRegister *reg;
  
  reg = ( ProcedureResultRegister* )db->app->procResp;
  reg->procedure = ( procedure->type & 0xFF );
  reg->status = procedure->status;
  reg->result = procedure->result;
  reg->exception = procedure->exception;
}

static void AppEmobTask_startProcedure( Procedure * const procedure )
{
  switch( procedure->type )
  {
  case PROCEDURE_ROUTER_CONNECTION:
  case PROCEDURE_ROUTER_DISCONNECTION:
    osThreadSuspend( appEmobTaskHandle );
  case PROCEDURE_TRANSACTION_REQUEST:
  case PROCEDURE_SCAN_AP:
    osThreadSuspend( advertisingTaskHandle );
    break;
  default:
    break;
  }
  db->app->connIdleStat = 1U;
  procedure->status = PROCEDURE_STATUS_RUNNING;
  procedure->result = PROCEDURE_RESULT_NONE;
  procedure->exception = PROCEDURE_EXCEPTION_NONE;
  AppEmobTask_notifyProcedure( procedure );
  osEventFlagsClear( procedureEventFlags, APPEMOB_PROCEDURE_FLAG_ALL );
}

static void AppEmobTask_executeProcedure( Procedure * const procedure )
{
  ProcedureException exception;
  uint32_t status;
  NetworkInfo netinfo;
  char *ssid, *pass;
  int32_t res;
  Wifi *pWifi;
  
  pWifi = &wifi;
  
  switch( procedure->type )
  {
  case PROCEDURE_CHECK_ACTIVATION_KEY:
    {
      if( SecureArea_verifyActivationKey(( char* )db->app->connActivKey) == 0 )
      {
        exception = PROCEDURE_EXCEPTION_NONE;
      }
      else
      {
        exception = PROCEDURE_EXCEPTION_REJECTED;
      }
      memset( db->app->connActivKey, 0, 5 );
    }
    break;
  case PROCEDURE_INIT_SOCKET:
    { 
      res = SecureArea_storeUserPin(( char* )db->app->connPin, 5);
      if( res == 0 )
      {
        memset( db->app->connPin, 0, 5 );
        res = AppEmobTask_saveStationName( ( char* )db->app->connName, APPEMOB_STATION_NAME_MAX_LEN );
        if( res == 0 )
        {
          res = setActivationFlag( 1 );
          if( res == 0 )
          {
            appemob->flags |= APPEMOB_FLAG_STATION_ACTIVE;
            db->app->connInitStat = 1;
            exception = PROCEDURE_EXCEPTION_NONE;
          }
        }
      }
      if( res != 0 )
      {
        exception = PROCEDURE_EXCEPTION_REJECTED;
      }
    }
    break;
  case PROCEDURE_MODIFY_USER_PIN:
    {
      res = SecureArea_storeUserPin(( char* )db->app->connPin, 5);
      if(res == 0)
      {
        exception = PROCEDURE_EXCEPTION_NONE;
      }
      else
      {
        exception = PROCEDURE_EXCEPTION_REJECTED;
      }
      memset( db->app->connPin, 0, 5 );
    }
    break;
  case PROCEDURE_LOGIN:
    {
      if( procedure->client->flags & APPEMOB_CLIENT_FLAG_LOGIN )
      {
        if( SecureArea_verifyUserPin( ( char* )db->app->connPin) == 0 )
        {
          exception = PROCEDURE_EXCEPTION_NONE;
        }
        else
        {
          exception = PROCEDURE_EXCEPTION_REJECTED;
        }
      }
      else
      {
        exception = PROCEDURE_EXCEPTION_OCCUPIED;
      }
      memset( db->app->connPin, 0, 5 );
    }
    break;
  case PROCEDURE_ROUTER_CONNECTION:
    {
      NetworkAddress address;
      
      ssid = ( char* )db->app->RoutSSID;
      pass = ( char* )db->app->RoutPass;
      res = 0;
      
      if( pWifi->isConnect() )
      {
        res = pWifi->getNetInfo( &netinfo );
        if( res == 0 )
        {
          if( strncmp( ssid, netinfo.ssid, 32 ) == 0 )
          {
            res = -1;
            exception = PROCEDURE_EXCEPTION_REJECTED;
          }
        }
      }
      if( res == 0 )
      {
        res = AppEmobTask_connectToWiFi(ssid, pass);
        if( res == 0 )
        {
          res = pWifi->getAddress( WIFI_INTERFACE_STATION, &address );
          if( res == 0 )
          {
            IP2STR( address.ip, ( char* )db->app->RoutIP );
          }
          if( res == 0 )
          {
            exception = PROCEDURE_EXCEPTION_NONE;
          }
          else
          {
            exception = PROCEDURE_EXCEPTION_REJECTED;
          }
        }
        else
        {
          exception = PROCEDURE_EXCEPTION_REJECTED;
        }
      }
      memset( pass, 0, APPEMOB_AP_PASS_MAX_LEN );
    }
    break;
  case PROCEDURE_ROUTER_DISCONNECTION:
    {
      if( pWifi->isConnect() )
      {
        res = pWifi->disable( WIFI_INTERFACE_STATION );
        if( res == 0 )
        {
          IP2STR( 0, (char *)db->app->RoutIP );
          res = SecureArea_resetRemoteAp();
          if( res == 0 )
          {
            exception = PROCEDURE_EXCEPTION_NONE;
          }
        }
      }
      else
      {
        exception = PROCEDURE_EXCEPTION_REJECTED;
      }
    }
    break;
  case PROCEDURE_MODIFY_STATION_NAME:
    {
      res = AppEmobTask_checkStationName( ( char* )db->app->connName, APPEMOB_STATION_NAME_MAX_LEN );
      if( res == 0 )
      {
        res = AppEmobTask_saveStationName( ( char* )db->app->connName, APPEMOB_STATION_NAME_MAX_LEN );
        if( res == 0 )
        {
          exception = PROCEDURE_EXCEPTION_NONE;
        }
      }
      if( res != 0 )
      {
        exception = PROCEDURE_EXCEPTION_INTERNAL_ERROR;
      }
    }
    break;
  case PROCEDURE_START_TRANSACTION:
    {
      if( ( db->rw->scuSetRegister.evseMod & 0x00FF ) == OP_MODE_PERSONAL )
      {
        strncpy( db->uid, ( char* )db->app->uidAuthApp, APPEMOB_TAG_UID_MAX_LEN );
        startPersonal = PERSONAL_BY_APP;
        db->ro->scuMapRegStatusMeas.uidAuth[0] = 0x0A;
        db->ro->scuMapRegStatusMeas.uidAuth[1] = UID_AUTHORIZATION_BY_APP;
        memcpy( &db->ro->scuMapRegStatusMeas.uidAuth[2], db->app->uidAuthApp, 10 );
        vTaskSuspend(RfidTaskHandle);
        rfidTaskStatus = RFID_TASK_SUSPENDED;
        send_to_evs(EVS_AUTH_START);
        status = AppEmobProcedureTask_wait( APPEMOB_PROCEDURE_FLAG_EVSE_NOTIFY | APPEMOB_PROCEDURE_FLAG_EVSE_STOP , APPEMOB_AUTH_WAIT_TIMEOUT_MS );
        if( status & APPEMOB_PROCEDURE_FLAG_EVSE_NOTIFY )
        {
          exception = PROCEDURE_EXCEPTION_NONE;
        }
        else if( status & APPEMOB_PROCEDURE_FLAG_EVSE_STOP )
        {
          memset( db->uid, 0, APPEMOB_TAG_UID_MAX_LEN );
          exception = PROCEDURE_EXCEPTION_EXPIRED;
        }
        else
        {
          exception = PROCEDURE_EXCEPTION_INTERNAL_ERROR;
        }
      }
      else
      {
        exception = PROCEDURE_EXCEPTION_REJECTED;
      }
    }
    break;
  case PROCEDURE_SUSPEND_TRANSACTION:
    {
      evseChargeStatusReg_e chargeStatus = ( evseChargeStatusReg_e )db->ro->scuMapRegStatusMeas.ntfChgStat;
      if( chargeStatus == CHARGING_STATE )
      {
        if( Scheduler_isDisabled() )
        {
          if( ( db->rw->scuSetRegister.evseMod & 0x00FF ) == OP_MODE_PERSONAL )
          {
            if( strncmp( db->uid, ( char* )db->app->uidAuthApp, APPEMOB_TAG_UID_MAX_LEN ) == 0 )
            {
              send_to_evs( EVS_APP_SUSPENDING );
              status = AppEmobProcedureTask_wait( APPEMOB_PROCEDURE_FLAG_EVSE_NOTIFY, APPEMOB_SOSPENSION_TIMEOUT_MS );
              if( status & APPEMOB_PROCEDURE_FLAG_EVSE_NOTIFY )
              {
                exception = PROCEDURE_EXCEPTION_NONE;
              }
              else
              {
                exception = PROCEDURE_EXCEPTION_INTERNAL_ERROR;
              }
            }
            else
            {
              exception = PROCEDURE_EXCEPTION_REJECTED;
            }
            memset( db->app->uidAuthApp, 0, LEN_UID_AUTHORIZATION_APP_RW * 2 );
          }
          else
          {
            send_to_evs( EVS_APP_SUSPENDING );
            status = AppEmobProcedureTask_wait( APPEMOB_PROCEDURE_FLAG_EVSE_NOTIFY, APPEMOB_SOSPENSION_TIMEOUT_MS );
            if( status & APPEMOB_PROCEDURE_FLAG_EVSE_NOTIFY )
            {
              exception = PROCEDURE_EXCEPTION_NONE;
            }
            else
            {
              exception = PROCEDURE_EXCEPTION_INTERNAL_ERROR;
            }
          }
        }
        else
        {
          exception = PROCEDURE_EXCEPTION_INVALID;
        }
      }
      else
      {
        exception = PROCEDURE_EXCEPTION_REJECTED;
      }
    }
    break;
  case PROCEDURE_RELEASE_SUSPENSION:
    {
      evseChargeStatusReg_e chargeStatus = ( evseChargeStatusReg_e )db->ro->scuMapRegStatusMeas.ntfChgStat;
      if( chargeStatus == SUSPENDED_EVSE_STATE )
      {
        if( Scheduler_isDisabled() )
        {
          if( ( db->rw->scuSetRegister.evseMod & 0x00FF ) == OP_MODE_PERSONAL )
          {
            if( strncmp( db->uid, ( char* )db->app->uidAuthApp, APPEMOB_TAG_UID_MAX_LEN ) == 0 )
            {
              send_to_evs( EVS_APP_RELEASE );
              status = AppEmobProcedureTask_wait( APPEMOB_PROCEDURE_FLAG_EVSE_NOTIFY, APPEMOB_PROCEDURE_TIMEOUT_MS );
              if( status & APPEMOB_PROCEDURE_FLAG_EVSE_NOTIFY )
              {
                exception = PROCEDURE_EXCEPTION_NONE;
              }
              else
              {
                exception = PROCEDURE_EXCEPTION_INTERNAL_ERROR;
              }
            }
            else
            {
              exception = PROCEDURE_EXCEPTION_REJECTED;
            }
            memset( db->app->uidAuthApp, 0, LEN_UID_AUTHORIZATION_APP_RW * 2 );
          }
          else
          {
            send_to_evs( EVS_APP_RELEASE );
            status = AppEmobProcedureTask_wait( APPEMOB_PROCEDURE_FLAG_EVSE_NOTIFY, APPEMOB_PROCEDURE_TIMEOUT_MS );
            if( status & APPEMOB_PROCEDURE_FLAG_EVSE_NOTIFY )
            {
              exception = PROCEDURE_EXCEPTION_NONE;
            }
            else
            {
              exception = PROCEDURE_EXCEPTION_INTERNAL_ERROR;
            }
          }
        }
        else
        {
          exception = PROCEDURE_EXCEPTION_INVALID;
        }
      }
      else
      {
        exception = PROCEDURE_EXCEPTION_REJECTED;
      }
    }
    break;
  case PROCEDURE_STOP_TRANSACTION:
    {
      evseChargeStatusReg_e chargeStatus = ( evseChargeStatusReg_e )db->ro->scuMapRegStatusMeas.ntfChgStat;
      if( ( chargeStatus == CHARGING_STATE          ) || 
          ( chargeStatus == SUSPENDED_EVSE_STATE    ) ||
          ( chargeStatus == SUSPENDED_EV_STATE      ) ||
          ( chargeStatus == SUSPENDED_NOPOWER_STATE ) )
      {
        if( ( db->rw->scuSetRegister.evseMod & 0x00FF ) == OP_MODE_PERSONAL )
        {
          if( strncmp( db->uid, ( char* )db->app->uidAuthApp, APPEMOB_TAG_UID_MAX_LEN ) == 0 )
          {
            send_to_evs( EVS_AUTH_STOP );
            status = AppEmobProcedureTask_wait( APPEMOB_PROCEDURE_FLAG_EVSE_NOTIFY, APPEMOB_PROCEDURE_TIMEOUT_MS );
            if( status & APPEMOB_PROCEDURE_FLAG_EVSE_NOTIFY )
            {
              memset( db->uid, 0, APPEMOB_TAG_UID_MAX_LEN );
              exception = PROCEDURE_EXCEPTION_NONE;
            }
            else
            {
              exception = PROCEDURE_EXCEPTION_INTERNAL_ERROR;
            }
          }
          else
          {
            exception = PROCEDURE_EXCEPTION_REJECTED;
          }
          memset( db->app->uidAuthApp, 0, LEN_UID_AUTHORIZATION_APP_RW * 2 );
        }
        else
        {
          send_to_evs( EVS_AUTH_STOP );
          status = AppEmobProcedureTask_wait( APPEMOB_PROCEDURE_FLAG_EVSE_NOTIFY, APPEMOB_PROCEDURE_TIMEOUT_MS );
          if( status & APPEMOB_PROCEDURE_FLAG_EVSE_NOTIFY )
          {
            exception = PROCEDURE_EXCEPTION_NONE;
          }
          else
          {
            exception = PROCEDURE_EXCEPTION_INTERNAL_ERROR;
          }
        }
      }
      else
      {
        exception = PROCEDURE_EXCEPTION_REJECTED;
      }
    }
    break;
  case PROCEDURE_MODIFY_SCHEDULE:
    {
      uint16_t *ptrToData;
      sck_schedule_t scheduleToSaved[ MAX_SCHEDULATION_NUMBER ];
      
      ptrToData = ( uint16_t* )( &db->app->schFlag1 );
      for( uint8_t id=0; id < MAX_SCHEDULATION_NUMBER; id++ )
      {
        scheduleToSaved[id].id = id;
        /* schFlag has bit 7 to enable/disable, bit 6 = dom, bit 5 = sab, bit 4 = ven... */
        scheduleToSaved[id].enable = (*ptrToData) & 0x80;
        scheduleToSaved[id].days = (*ptrToData) & 0x7F;    
        scheduleToSaved[id].start_hour = (*(++ptrToData) & 0xFF00) >> 8;
        scheduleToSaved[id].start_min = (*ptrToData) & 0x00FF;    
        scheduleToSaved[id].end_hour = (*(++ptrToData) & 0xFF00) >> 8;
        scheduleToSaved[id].end_min = (*ptrToData) & 0x00FF;
        scheduleToSaved[id].power = *((int32_t*)(++ptrToData));
        ptrToData = ptrToData + 2;
      }
      saveSchedulation( scheduleToSaved );
      Scheduler_scheduleCharge( scheduleToSaved );
      exception = PROCEDURE_EXCEPTION_NONE;
    }
    break;
  case PROCEDURE_TRANSACTION_REQUEST:
    {
      uint32_t size, count;
      uint8_t *data;
      
      osDelay(pdMS_TO_TICKS(500));
      
      count = TransactionRegister_getCount();
      size = count * sizeof( transaction_t );
      data = ( uint8_t* )malloc( size + 2 );
      if( data != NULL )
      {
        if( count != 0 )
          TransactionRegister_getTransaction( ( Transaction* )&data[ 2 ], count );
        data[ 0 ] = size >> 8;
        data[ 1 ] = size & 0x00FF;
        res = pWifi->send( procedure->client->id, data, size + 2 );
        if( res == ( size + 2 ) )
        {
          exception = PROCEDURE_EXCEPTION_NONE;
        }
        else
        {
          exception = PROCEDURE_EXCEPTION_INTERNAL_ERROR;
        }
        free( data );
      }
      else
      {
        exception = PROCEDURE_EXCEPTION_INTERNAL_ERROR;
      }
    }
    break;
  case PROCEDURE_FORCE_REBOOT:
    {
      exception = PROCEDURE_EXCEPTION_NONE;
    }
    break;
  case PROCEDURE_CHANGE_WORKING_MODE:
    {
      res = -1;
      if( db->ro->scuMapRegStatusMeas.ntfChgStat != CHARGING_STATE )
      {
        if( db->rw->scuSetRegister.evseMod != db->stationMode )
        {
          if( ( db->rw->scuSetRegister.evseMod & 0x00FF ) != OP_MODE_NET )
          {
            db->stationMode = db->rw->scuSetRegister.evseMod;
            EEPROM_Save_Config ( EVS_MODE_EADD, ( uint8_t* )&db->rw->scuSetRegister.evseMod, 1 );
            send_to_evs( EVS_AUTORIZATION_MODE );
            exception = PROCEDURE_EXCEPTION_NONE;
            res = 0;
          }
        }
      }
      if( res != 0 )
      {
        exception = PROCEDURE_EXCEPTION_REJECTED;
      }
    }
    break;
  case PROCEDURE_SETTING_TIME:
    {
      setCurrentTimestamp();
      exception = PROCEDURE_EXCEPTION_NONE;
    }
    break;
  case PROCEDURE_DELETE_TRANSACTION:
    {
      res = TransactionRegister_clearAll();
      if( res == 0 )
      {
        exception = PROCEDURE_EXCEPTION_NONE;
      }
      else
      {
        exception = PROCEDURE_EXCEPTION_INTERNAL_ERROR;
      }
    }
    break;
  case PROCEDURE_SET_MAXIMUM_CURRENT:
    {
      if( db->rw->scuSetRegister.maxTypCurr != 0 )
      {
        if( ( db->rw->scuSetRegister.maxTypCurr >= 6 ) && 
            ( db->rw->scuSetRegister.maxTypCurr <= 32 ) )
        {
          uint8_t tCurrent = db->rw->scuSetRegister.maxTypCurr;
          uint8_t sCurrent = db->rw->scuSetRegister.maxSimplCurr;
          if( tCurrent < 16 )
          {
            if( tCurrent < sCurrent )
            {
              EEPROM_Save_Config (M3S_CURRENT_EADD, &tCurrent, 1);
              db->rw->scuSetRegister.maxSimplCurr = tCurrent;
            }
          }
          else
          {
            sCurrent = 16;
            EEPROM_Save_Config (M3S_CURRENT_EADD, &sCurrent, 1);
            db->rw->scuSetRegister.maxSimplCurr = sCurrent;
          }
          EEPROM_Save_Config (M3T_CURRENT_EADD, &tCurrent, 1);
          exception = PROCEDURE_EXCEPTION_NONE;
        }
        else
        {
          exception = PROCEDURE_EXCEPTION_REJECTED;
        }
      }
      else
      {
        exception = PROCEDURE_EXCEPTION_INVALID;
      }
    }
    break;
  case PROCEDURE_SET_POWER_MANAGEMENT:
    {
      res = AppEmobTask_savePowerManagementParams();
      if( res == 0 )
      {
        exception = PROCEDURE_EXCEPTION_NONE;
      }
      else
      {
        exception = PROCEDURE_EXCEPTION_REJECTED;
      }
    }
    break;
  case PROCEDURE_SCAN_AP:
    {
      uint16_t bytes;
      
      osDelay(pdMS_TO_TICKS(500));
      AppScanApInfo *netScan = (AppScanApInfo *)malloc( sizeof( AppScanApInfo ) + ( APPEMOB_SCAN_AP_MAX_COUNT * sizeof( NetworkInfo ) ) );
      if( netScan != NULL )
      {
        res = pWifi->scan( netScan->ap, APPEMOB_SCAN_AP_MAX_COUNT, NULL, -100 );
        if( res > 0 )
        {
          res = AppEmobTask_filterApNetworks( netScan->ap, res );
          bytes = res * sizeof( NetworkInfo );
          netScan->bytes = (bytes << 8) | (bytes >> 8);
          res = pWifi->send( procedure->client->id, netScan, 2 + bytes );
          if( res == ( 2 + bytes ) )
            exception = PROCEDURE_EXCEPTION_NONE;
          else
            exception = PROCEDURE_EXCEPTION_INTERNAL_ERROR;
        }
        else if( res == 0 )
        {
          exception = PROCEDURE_EXCEPTION_NONE;
        }
        else
        {
          exception = PROCEDURE_EXCEPTION_INTERNAL_ERROR;
        }
        free( netScan );
      }
      else
      {
        exception = PROCEDURE_EXCEPTION_INTERNAL_ERROR;
      }
    }
    break;
  case PROCEDURE_SET_CHARGE_BY_TIME:
  case PROCEDURE_SET_LANGUAGES:
  case PROCEDURE_SET_CHECKS_ACTUATORS:
  case PROCEDURE_RESET_SCHEDULATIONS:
  case PROCEDURE_FACTORY_RESET:
  case PROCEDURE_SINAPSI_BT_START:
  case PROCEDURE_UNKNOWN:
  default:
    {
      exception = PROCEDURE_EXCEPTION_INTERNAL_ERROR;
    }
    break;
  case PROCEDURE_NONE:
    /* Not possible */
    break;
  }
  procedure->exception = exception;
}

static void NetworkScanInfo_swap (NetworkScanInfo *ap1, NetworkScanInfo *ap2) 
{
  NetworkScanInfo tmp;
  
  memcpy(&tmp, ap1, sizeof(NetworkScanInfo));
  memcpy(ap1, ap2, sizeof(NetworkScanInfo));
  memcpy(ap2, &tmp, sizeof(NetworkScanInfo));
}

static uint32_t NetworkScanInfo_countInstancesBySsid (NetworkScanInfo *ap, NetworkScanInfo *aps, uint32_t len)
{
  uint32_t cnt;

  cnt = 0;
  for (uint32_t i = 0; i < len; i++) {
    if (strcmp(ap->ssid, aps[i].ssid) == 0) {
      cnt += 1;
    }
  }

  return cnt;
}

static int32_t AppEmobTask_filterApNetworks( NetworkScanInfo *aps, uint32_t len )
{
  uint32_t win;
  
  // Reorder by SSID
  win = 1;
  for (uint32_t i = 1; i < len; i++) {
    uint32_t cnt = NetworkScanInfo_countInstancesBySsid(&aps[i], aps, win);
    if (cnt == 0) {
      if (i != win)
        NetworkScanInfo_swap(&aps[win], &aps[i]);
      win++;
    }
  }
  
  return win;
}

static void AppEmobTask_stopProcedure( Procedure * const procedure )
{
  procedure->status = PROCEDURE_STATUS_TERMINATED;
  if( procedure->exception == PROCEDURE_EXCEPTION_NONE )
  {
    procedure->result = PROCEDURE_RESULT_SUCCESS;
  }
  else
  {
    procedure->result = PROCEDURE_RESULT_EXCEPTION;
  }
  AppEmobTask_notifyProcedure( procedure );
  
  if( procedure->type == PROCEDURE_ROUTER_DISCONNECTION )
  {
    AppEmobTask_clearProcedure();
  }
}

static void AppEmobTask_postProcedure( Procedure * const procedure )
{
  Wifi *pWifi;
  
  pWifi = &wifi;
  
  switch( procedure->type )
  {
  case PROCEDURE_LOGIN:
    {
      if( procedure->exception == PROCEDURE_EXCEPTION_OCCUPIED )
      {
        LOG( "Login " LOG_FAIL ": another user is connected. Force close.\r\n" );
        pWifi->close( procedure->client->id );
      }
    }
    break;
  case PROCEDURE_TRANSACTION_REQUEST:
  case PROCEDURE_SCAN_AP:
    osThreadResume( advertisingTaskHandle );
    break;
  case PROCEDURE_FORCE_REBOOT:
    HAL_RTCEx_BKUPWrite( (RTC_HandleTypeDef*)getHandleRtc(), BACKUP_NVIC_RESET_REG, BACKUP_NVIC_RESET_VAL );
    HAL_NVIC_SystemReset();
    break;
  default:
    break;
  }
}

static void AppEmobTask_clearProcedure( void )
{
  Procedure procedure;
  
  db->app->connIdleStat = 0U;
  procedure.client = NULL;
  procedure.type = PROCEDURE_NONE;
  procedure.status = PROCEDURE_STATUS_NONE;
  procedure.result = PROCEDURE_RESULT_NONE;
  procedure.exception = PROCEDURE_EXCEPTION_NONE;
  AppEmobTask_notifyProcedure( &procedure );
}

static int32_t AppEmobTask_saveStationName( char *name, uint32_t len )
{
  int32_t res;
  
  res = setStationName( name, len );
  if( res == 0 )
  {
    strncpy( db->stationName, name, len );
  }
  return res;
}

static int32_t AppEmobTask_checkStationName( char *name, uint32_t len )
{
  int32_t res;
  
  res = -1;
  if( strlen( name ) > 0 )
  {
    res = 0;
  }
  
  return res;
}

static int32_t AppEmobTask_savePowerManagementParams( void )
{
  int32_t res;
  
  uint16_t pmFlags = db->rw->scuSetRegister.pmFlags;
  uint16_t pmPmax = db->rw->scuSetRegister.pmPmax;
  uint16_t pmImin = db->rw->scuSetRegister.pmImin;
  uint16_t pmHpower = db->rw->scuSetRegister.pmHpower;
  uint16_t pmDset = db->rw->scuSetRegister.pmDset;
  uint16_t pmDmax = db->rw->scuSetRegister.pmDmax;
  uint16_t pmMode = db->rw->scuSetRegister.pmMode;
  bool pmEmex = ( db->rw->scuSetRegister.hwChecks2 & 0x0002 ) ? true : false;
  
  res = AppEmobTask_savePowerManagementFlags( pmFlags );
  if( res == 0 )
  {
    res = AppEmobTask_savePowerManagementPmax( pmPmax );
    if( res == 0 )
    {
      res = AppEmobTask_savePowerManagementImin( pmImin );
      if( res == 0 )
      {
        res = AppEmobTask_savePowerManagementHpow( pmHpower );
        if( res == 0 )
        {
          res = AppEmobTask_savePowerManagementDset( pmDset );
          if( res == 0 )
          {
            res = AppEmobTask_savePowerManagementDmax( pmDmax );
            if( res == 0 )
            {
              res = AppEmobTask_savePowerManagementMode( pmMode );
              if( res == 0 )
              {
                res = AppEmobTask_savePowerManagementEmex( pmEmex );
              }
            }
          }
        }
      }
    }
  }
  return res;
}

static int32_t AppEmobTask_savePowerManagementFlags( uint16_t flags )
{
  uint8_t u8;
  
  eeprom_param_get( HIDDEN_MENU_ENB_EADD, (uint8_t *)&u8, 1 );
  if( flags & 0x0001 )
  {
    u8 |= HIDDEN_MENU_PMNG_ENB;
  }
  else
  {
    u8 &= ~HIDDEN_MENU_PMNG_ENB;
  }
  EEPROM_Save_Config ( HIDDEN_MENU_ENB_EADD, &u8, 1 );
  
  u8 = 0;
  if( flags & 0x0002 )
  {
    u8 = 1;
  }
  EEPROM_Save_Config ( PMNG_UNBAL_EADD, (uint8_t*)&u8, 1 );
  return 0;
}

static int32_t AppEmobTask_savePowerManagementPmax( uint16_t power )
{
  int32_t res;
  uint8_t u8;
  
  res = -1;
  if( ( power >= 30 ) && ( power <= 999 ) )
  {
    u8 = power & 0x00FF;
    EEPROM_Save_Config (PMNG_PWRLSB_EADD, (uint8_t*)&u8, 1);
    u8 = power >> 8;
    EEPROM_Save_Config (PMNG_PWRMSB_EADD, (uint8_t*)&u8, 1);
    res = 0;
  }
  return res;
}

static int32_t AppEmobTask_savePowerManagementImin( uint16_t current )
{
  int32_t res;
  uint8_t u8;
  
  res = -1;
  if( ( current >= 60 ) && ( current <= 630 ) )
  {
    if( current > 254 )
    {
      u8 = 255;
    }
    else
    {
      u8 = current;
    }
    EEPROM_Save_Config (PMNG_CURRENT_EADD, (uint8_t*)&u8, 1);
    res = 0;
  }
  return res;
}

static int32_t AppEmobTask_savePowerManagementHpow( uint16_t percent )
{
  int32_t res;
  uint8_t u8;
  
  res = -1;
  u8 = percent - 1;
  if( u8 <= 100 )
  {
    EEPROM_Save_Config (PMNG_MULTIP_EADD, (uint8_t*)&u8, 1);
    res = 0;
  }
  return res;
}

static int32_t AppEmobTask_savePowerManagementDset( uint16_t power )
{
  int32_t res;
  uint8_t u8;
  
  res = -1;
  if( ( power >= 1 ) && ( power <= 500 ) )
  {
    if( power > 254 )
    {
      u8 = 255;
    }
    else
    {
      u8 = power;
    }
    EEPROM_Save_Config (PMNG_ERROR_EADD, (uint8_t*)&u8, 1);
    res = 0;
  }
  return res;
}

static int32_t AppEmobTask_savePowerManagementDmax( uint16_t percent )
{
  int32_t res;
  uint8_t u8;
  
  res = -1;
  if( ( percent >= 1 ) && ( percent <= 100 ) )
  {
    u8 = percent;
    EEPROM_Save_Config (PMNG_DMAX_EADD, (uint8_t*)&u8, 1);
    res = 0;
  }
  return res;
}

static int32_t AppEmobTask_savePowerManagementMode( uint16_t mode )
{
  int32_t res;
  uint8_t u8;
  
  res = -1;
  if( mode <= 2 )
  {
    u8 = mode;
    EEPROM_Save_Config (PMNG_MODE_EADD, &u8, 1);
    res = 0;
  }
  return res;
}

static int32_t AppEmobTask_savePowerManagementEmex( bool enable )
{
  uint8_t u8;
  
  eeprom_param_get( CONTROL_BYTE2_EADD, (uint8_t*)&u8, 1 );
  if( enable )
  {
    u8 |= EMETER_EXT_CRL2;
  }
  else
  {
    u8 &= ~EMETER_EXT_CRL2;
  }
  EEPROM_Save_Config (CONTROL_BYTE2_EADD, (uint8_t*)&u8, 1);
  send_to_evs( EVS_EXTERNAL_EM_UPDATE );
  return 0;
}

static void AppEmobTask_initFirmwareUpdate( ClientCtl * const client )
{
  Wifi *pWifi;
  uint32_t timeout;
  ProcedureException exception;
  ProcedureStatus status;
  ProcedureResult result;
  ProcedureResultRegister *reg;
  
  pWifi = &wifi;
  reg = ( ProcedureResultRegister* )db->app->procResp;
  timeout = 0;
  
  if( ( db->app->fwUpdateLen > 0 ) && ( strlen( ( char* )db->app->fwUpdateName ) > 0 ) )
  {
    // Disabilito timeout per il client che fa la richiesta di aggiornamento firmware.
    pWifi->setSocketOpt( client->id, SOCKET_OPT_RCVTIMEO, &timeout, sizeof( timeout ) );
    // Disabilito l'advertising della stazione.
    osThreadSuspend( advertisingTaskHandle );
    // Disabilito i tentativi di riconnessione a un rete Wi-Fi.
    pWifi->reconnectionOpt( 0, 0 );
    // Cambio la funzione che processa i dati ricevuti dalla socket client con protocollo TCP grezzo.
    appemob->pcktId = 0;
    appemob->fwPcktLen = 0;
    appemob->maxPckt = db->app->fwUpdateLen / APPEMOB_FIRMWARE_DATA_BYTESIZE;
    memset( appemob->fwPacket, 0, APPEMOB_FIRMWARE_PACKET_BYTESIZE );
    // Imposto flag per cambiare funzione processamento pacchetti TCP
    client->flags |= APPEMOB_CLIENT_FLAG_UPDATE_FW_REQ;
    // Avvio task aggiornamento firmware.
    FirmwareUpdate_start( (char*)db->app->fwUpdateName, db->app->fwUpdateLen );
    // Controllo esito aggiornamento firmware
    status = PROCEDURE_STATUS_RUNNING;
    result = PROCEDURE_RESULT_NONE;
    exception = PROCEDURE_EXCEPTION_NONE;
  }
  else
  {
    status = PROCEDURE_STATUS_TERMINATED;
    result = PROCEDURE_RESULT_EXCEPTION;
    exception = PROCEDURE_EXCEPTION_INVALID;
  }
  reg->procedure = PROCEDURE_UPDATE_FW;
  reg->status = status;
  reg->result = result;
  reg->exception = exception;
}

static int32_t processFirmwareRequest( ClientCtl * const client )
{
  int32_t res;
  Wifi *pWifi;
  AppEmobFirmwarePacket *packet;
  uint16_t pckId, check;
  uint8_t reboot;
  
  pWifi = &wifi;
  
  reboot = 0;
  if( client->recLen > 0 )
  {
    memcpy( &appemob->fwPacket[ appemob->fwPcktLen ], client->req, client->recLen );
    appemob->fwPcktLen += client->recLen;
    if( appemob->fwPcktLen == APPEMOB_FIRMWARE_PACKET_BYTESIZE )
    {
      packet = (AppEmobFirmwarePacket* )appemob->fwPacket;
      pckId = UINT16_SWAP( packet->id );
      check = UINT16_SWAP( packet->check );
      if( ( pckId == appemob->pcktId ) &&
          ( check == 0xABCD ) )
      {
        /* Packet check success */
        res = FirmwareUpdate_put( packet->data, APPEMOB_FIRMWARE_DATA_BYTESIZE );
        if( res == 0 )
        {
          appemob->pcktId++;
          client->res[ 0 ] = ( uint8_t )( appemob->pcktId >> 8 );
          client->res[ 1 ] = ( uint8_t )( appemob->pcktId );
          
          pWifi->send( client->id, client->res, 2 );
          
          appemob->fwPcktLen = 0;
          if( appemob->pcktId == appemob->maxPckt )
          {
            res = FirmwareUpdate_end();
            reboot = 1U;
          }
        }
      }
    }
  }
  else
  {
    reboot = 1U;
  }
  
  if( reboot )
  {
    osDelay( pdMS_TO_TICKS( 500 ) );
    setFlagForNvic();
    NVIC_SystemReset();
  }
  
  return res;
}

int32_t AppEmobTask_setWifiChannel( uint8_t channel )
{
  WifiConfig config;
  int32_t res;
  char ssid[32];
  char pass[64];
  
  SecureArea_getLocalApSSID(ssid, sizeof(ssid));
  SecureArea_getLocalApPass(pass, sizeof(pass));
  
  config.ssid = ssid;
  config.pass = pass;
  config.enc = WIFI_ENC_WPA2_PSK;
  config.channel = channel;
  WIFI_CONFIG_MAC_SET_NULL(config.mac);
  res = wifi.enable( WIFI_INTERFACE_AP, &config );
  if( res == wifiOK )
  {
    res = saveWifiAccessPointChannelId( channel );
    if( res == 0 )
    {
      db->localApChannel = channel;
    }
  }
  return res;
}

uint8_t AppEmobTask_getWifiChannel( void )
{
  return db->localApChannel;
}

void AppEmobTask_printWifiInfo( void )
{
  int32_t status;
  Wifi *pWifi;
  NetworkInfo netInfo;
  NetworkAddress netAddress;
  Version version;
  char ssid[32];
  
  pWifi = &wifi;
  
  status = pWifi->getVersion( &version );
  if( status == wifiOK )
  {
    LOG( "ESP32 firmware AT version: %hhu.%hhu.%hhu.%hhu\r\n", version.major, version.minor, version.patch, version.hotfix );
  }
  
  if( pWifi->isConnect() )
  {
    status = pWifi->getNetInfo( &netInfo );
    if( status == 0 )
    {
      LOG( "Wifi interface STATION active.\r\n" );
      LOG( "Station connected to \"%s\", channel = %d, rssi = %ddBm, bssid = %02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx\r\n", netInfo.ssid, netInfo.channel, netInfo.rssi, MAC_ADDRESS(netInfo.bssid));
    }
    status = pWifi->getAddress( WIFI_INTERFACE_STATION, &netAddress );
    if( status == 0 )
    {
      LOG( "Station IP %hhu.%hhu.%hhu.%hhu\r\n", IP_ADDRESS( netAddress.ip ) );
    }
  }
  else
  {
    SecureArea_getLocalApSSID(ssid, sizeof(ssid));
    LOG( "Wifi interface ACCESS POINT active.\r\n" );
    LOG( "Station AP: \"%s\", channel = %d\r\n", ssid, db->localApChannel );
  }
}

int8_t AppEmobTask_getWifiRssi( void )
{
  int32_t status;
  int8_t rssi;
  Wifi *pWifi;
  NetworkInfo netInfo;
  
  pWifi = &wifi;
  
  rssi = 0;
  if( pWifi->isConnect() )
  {
    status = pWifi->getNetInfo( &netInfo );
    if( status == 0 )
    {
      rssi = netInfo.rssi;
    }
  }
  
  return rssi;
}

void AppEmobTask_getModuleVersion( Version * const version )
{
  Wifi *pWifi;
  
  pWifi = &wifi;
  
  version->major = 0;
  version->minor = 0;
  version->patch = 0;
  version->hotfix = 0;
  pWifi->getVersion( version );
}

int32_t AppEmobTask_updateModule( void )
{
  int32_t res;
  Wifi *pWifi;
  Version current, candidate;
  
  pWifi = &wifi;
  res = wifiOK;
  candidate = moduleFirmwareVer;
  
  // Check if module is OFF.
  if( ( appemob->flags & APPEMOB_FLAG_WIFI_ACTIVE ) == 0 )
  {
    res = pWifi->init( WifiUserNotifyEvent );
    if( res == wifiOK )
    {
      res = pWifi->turnOn();
      if( res == 0 )
      {
        LOG( "ESP32 firmware update: enable Wi-Fi layer\r\n" );
      }
    }
  }
  
  if( res == wifiOK )
  {
    res = pWifi->getVersion( &current );
    if( res == 0 )
    {
      res = Version_compare( &candidate, &current );
      if( res > 0 )
      {
        // Update required
        // Check if module is connected to a network.
        if( ( appemob->flags & APPEMOB_FLAG_STATION_CONNECTED ) == 0 )
        {
          res = pWifi->enable( WIFI_INTERFACE_STATION, (WifiConfig*)&AP_FOR_MODULE_UPDATE );
          if( res == 0 )
          {
            LOG( "ESP32 firmware update: connected to \"%s\"\r\n", AP_FOR_MODULE_UPDATE.ssid );
          }
        }
        else
        {
          res = wifiOK;
        }
        if( res == wifiOK )
        {
          // Terminate task
          if( ( appemob->flags & APPEMOB_FLAG_WIFI_ACTIVE ) != 0 )
          {
            osThreadTerminate( appEmobTaskHandle );
            osThreadTerminate( advertisingTaskHandle );
            osThreadTerminate( serverTaskHandle );
          }
          // Update module with candidate version.
          LOG( "ESP32 firmware update: loading...\r\n" );
          res = pWifi->update( &candidate ); 
          if( res == 0 )
          {
            LOG( "ESP32 firmware has been successfully updated\r\n");
          }
          else
          {
            LOG( "Error %d during ESP32 firmware update\r\n", res );
          }
          pWifi->restore();
        }
      }
      else
      {
        LOG( "ESP32 firmware is up-to-date\r\n" );
        res = 1;
      }
    }
  }
  
  return res;
}

int32_t AppEmobTask_disconnectFromWiFi( void )
{
  int32_t res;
  Wifi *pWifi;
  
  pWifi = &wifi;
  
  res = pWifi->disable( WIFI_INTERFACE_STATION );
  if(res == 0)
  {
    res = SecureArea_resetRemoteAp();
  }
  return res;
}

int32_t AppEmobTask_connectToWiFi( char *ssid, char *pass )
{
  int32_t res;
  WifiConfig netConfig;
  Wifi *pWifi;
  NetworkScanInfo *aps;
  
  pWifi = &wifi;
  netConfig.ssid = ssid;
  netConfig.pass = pass;
  WIFI_CONFIG_MAC_SET_NULL(netConfig.mac);
  
  aps = (NetworkScanInfo*)malloc( ( APPEMOB_SCAN_AP_MAX_COUNT * sizeof( NetworkScanInfo ) ) );
  if( aps == NULL )
    return -1;
  
  res = pWifi->scan( aps, APPEMOB_SCAN_AP_MAX_COUNT, NULL, -100 );
  if (res > 0) {
    res = AppEmobTask_filterApNetworks( aps, res );
    for (int i = 0; i < res; i++) {
      if (strcmp(aps[i].ssid, ssid) == 0) {
        memcpy(netConfig.mac, aps[i].bssid, 6);
        break;
      }
    }
  }
  
  res = pWifi->enable( WIFI_INTERFACE_STATION, &netConfig );
  if(res == 0)
  {
    res = SecureArea_storeRemoteAp(ssid, pass);
  }
  return res;
}
/**
* @file        blueMng.h
*
* @brief       Bluetooth manager  - Definition -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: blueMng.h 456 2024-04-19 12:42:30Z npiergi $
*
*     $Revision: 456 $
*
*     $Author: npiergi $
*
*     $Date: 2024-04-19 14:42:30 +0200 (ven, 19 apr 2024) $
*
*
* @copyright
*       Copyright (C) 2016 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

#ifndef __BLUEMNG_H 
#define __BLUEMNG_H

#include "cmsis_os2.h"
#include <string.h>
#include <time.h>

#include "EvsMng.h"

/********************* Bluetooth Task definitions ******************************/

#define BT_QUEUE_MAX_LENGTH 100

#define MAX_NUM_TRANSACTION_STORABLE            ((uint16_t)500) 
//#define MAX_NUM_TRANSACTION_STORABLE  ((uint16_t)32)

#define UID_TRANSACTION_FREE                    ((uint32_t)0x00000000)
#define UID_TRANSACTION_NET                     ((uint32_t)0x33333333)
#define UID_TRANSACTION_OCPP                    ((uint32_t)0x66666666)
#define UID_TRANSACTION_PERS_BY_CARD            ((uint32_t)0x88888888)
#define UID_TRANSACTION_PERS_BY_APP             ((uint32_t)0x99999999)
#define UID_TRANSACTION_DSO                     ((uint32_t)0xFFFFFFFF)
#define UNCHANGED_PARAMETER                     ((uint32_t)0xFFFFFFFF)

#define UID_SIZE                                ((uint16_t)10)


/************************* Socket definitions **********************************/
#define UNDEFINED  0x00
#define MAX_NAME_LENGTH 14

/* REGISTRO CONNECTOR_TYPE_RW modbus v19.2--> Addres 0x0000, 1 word */
typedef enum 
{
  PRESA_NESSUNA = 0x00,
  PRESA_TIPO_3A = 0x01,
  CONNETTORE_TETHERED_TIPO_1 = 0x02,
  CONNETTORE_TETHERED_TIPO_2 = 0x03,
  PRESA_SCHUKO = 0x04,
  PRESA_TIPO_2 = 0x05,
  CONNETTORE_TETHERED_TIPO_FF = 0x06,
  CONNETTORE_TETHERED_TIPO_AA = 0x07,
} sck_wiring_e;

/* Energy Meter definition */
enum _energy_meter
{
  UNKNOW =       (uint8_t)0,
  NONE =         (uint8_t)0, 
  TA =           (uint8_t)1,
  MONO_GAVAZZI = (uint8_t)2,
  MONO_ALGO2,
  MONO_LOVATO,
  TRI_GAVAZZI,
  TRI_ALGO2,
  TRI_LOVATO,
  MONO_SCAME,
  TRI_SCAME,
  TA_TRI,
  MONO_SINAPSI,
  MONO_PA775,
  TRI_PA775 = 0x0D,
};
typedef enum _energy_meter energy_meter_e;

/* Rfid Reader */
enum _rfid
{
  OTHER =                       0x00,
  MIFARE_MINI_4B =              0x01,
  MIFARE_MINI_7B =              0x02,
  MIFARE_1K_4B =                0x03,
  MIFARE_1K_7B =                0x04,
  MIFARE_4K_4B =                0x05,
  MIFARE_4K_7B =                0x06,
  MIFARE_ULTRALIGHT =           0x07,
  MIFARE_DESFIRE =              0x09,
  MIFARE_PROX =                 0x0B,
  MIFARE_PLUS_2K_SL2_4B =       0x21,
  MIFARE_PLUS_4K_SL2_4B =       0x22,
  MIFARE_PLUS_2K_SL2_7B =       0x23,
  MIFARE_PLUS_4K_SL2_7B =       0x24,
  MIFARE_PLUS_2K_SL03_4B =      0x31,
  MIFARE_PLUS_4K_SL03_4B =      0x32,
  MIFARE_PLUS_2K_SL03_7B =      0x33,
  MIFARE_PLUS_4K_SL03_7B =      0x34,
  CALYPSO =                     0xF0,
  VEHIACLE =                    0xF7
};
typedef enum _rfid rfid_e;

/* Socket state */
typedef enum 
{
  SCK_UNKNOWN = (uint8_t)0,
  SCK_DISABLED = 5,                     // No user allowed
  SCK_AUTH_WAIT = 6,                    // wait for user authorization / remote supervisor action
  SCK_SOCKET_AVAILABLE = 7,             // evs_mode == EVS_FREE_MODE
  SCK_EVSTATE_PLUG_WAIT = 10,
  SCK_BLOCK_UP_DELAY = 14,               // wait timeout before drive block up
  SCK_SOCKET_CHECK = 13,                 // inserted plug
  SCK_S2_WAITING = 20,                   // wait for S2 closing
  SCK_CHARGING = 22,                     // EV in charge
  SCK_SUSPENDING = 18,                   // forced suspension
  SCK_INTERRUPTING = 23,                 // wait for S2 opening or timeout
  SCK_PLUG_OUT = 27,                     // wait for EVS_PLUG_OUT
  SCK_RES_ERROR = 26,                    // show evs error
  SCK_ERROR_WAIT = 31
} sck_statevalue_e;

/* Socket error */
enum _sck_error
{
  ERROR_NONE = (uint8_t)0x00,
  ERR_RCDM_ANOM0,  
  ERR_LID_ANOM0,   
  ERR_VENT_ANOM0,  
  ERR_BLOCK_ANOM0, 
  ERR_NU04_ANOM0,  
  ERR_NU05_ANOM0,  
  ERR_MIRROR_ANOM0,
  ERR_RCBO_ANOM0,  
  ERR_CPSHORT_CRL1,    
  ERR_PPSHORT_CRL1,    
  ERR_CPLOST_CRL1,     
  ERR_PPLOST_CRL1,     
  ERR_VBUS_CRL1,       
  ERR_MIFARE_CRL1,     
  ERR_EMETER_INT_CRL1, 
  ERR_OVERCURRENT_CRL1,
  ERR_RECTIFIER_CRL2, 
  ERR_EMETER_EXT_CRL2
};
typedef enum _sck_error sck_error_e;

/* Socket state */
struct _sck_state
{
  sck_statevalue_e value;
  sck_error_e error;
};
typedef struct _sck_state sck_state_t;

/* Socket function mode */
enum _sck_mode
{
  FREE = (uint8_t)0,
  PERSONAL,
  NET,
  OCPP
};
typedef enum _sck_mode modeFun_e;

/* station Operative mode  */
typedef enum
{
  MODE_MONO_PH_NO_PM =     ((uint8_t)0x00), /** measure: Etot, Time, Pist, L1 */
  MODE_TRI_PH_NO_PM,                        /** measure: Etot, Time, Pist, L1, L2, L3 */
  MODE_MONO_PH_PM,                          /** measure: Etot, Time, Pist, Pest, L1 */
  MODE_TRI_PH_PM_BAL,                       /** measure: Etot, Time, Pist, Pest, L1, L2, L3, Pest1, Pest2, Pest3 */
  MODE_TRI_PH_PM_UMBAL                      /** measure: Etot, Time, Pist, Pest, L1, L2, L3 */
}modePwr_e;


/* Socket measures */
struct _sck_measures
{
  modePwr_e   modePwr;
  int32_t     Etot;
  uint32_t    duration;
  uint32_t    Pist;
  uint32_t    Pest;
  int32_t     currentL1;
  int32_t     currentL2;
  int32_t     currentL3;
  uint32_t    Pest1;
  uint32_t    Pest2;
  uint32_t    Pest3;
};
typedef struct _sck_measures sck_measures_t;
#define MEASURE_SIZE 41

/* Transaction */
typedef __packed struct
{
  time_t    start;                // Unix Time -->  8 bytes
  time_t    stop;                 // Unix Time -->  8 bytes
  //uint32_t  duration;             // sec       -->  4 bytes 
  uint32_t  active_energy;        // Wh        -->  4 bytes
  uint8_t   uid[UID_SIZE];        // string    --> 10 bytes
                                  //    Tot.   --> 30 bytes   
}transaction_t;

//#define TRANS_SIZE 26
#define TRANS_SIZE 34
//typedef struct _transaction transaction_t;

/* Transactions Register*/
struct _register
{
  unsigned int num;
  transaction_t *transactions;
};
typedef struct _register sck_register_t;

/* Socket Login */
#define USER_LENGTH 20
#define PASS_LENGTH 20

enum _auth_resp
{
  NO_AUTH = 0,
  AUTH_SETTED,
  AUTH_ACCEPTED,
  AUTH_REFUSED
};
typedef enum _auth_resp auth_resp_e;

struct _sck_auth
{
  char user[USER_LENGTH + 1];
  char pass[PASS_LENGTH + 1];
  auth_resp_e auth_state;
};
typedef struct _sck_auth sck_auth_t;

/* Socket schedulation */
#define MAX_SCHEDS 4
#define SCHED_BT_SIZE 11 //byte
typedef __packed struct
{
  uint8_t id;
  uint8_t days;
  uint8_t start_hour;
  uint8_t start_min;
  uint8_t end_hour;
  uint8_t end_min;
  int32_t power;
  uint8_t enable;
} sck_schedule_t;

/* Socket flag checks/actuators indexes definitions */
//enum _socketCheck
//{
//  RCDM = 0, // Residual Current Device Monitor
//  LIDC,     // Lid Closed
//  VENT,     // Presence Ventilation
//  BLCK,     // Latching System Block
//  RMEN,     // Remote Suspension
//  STOP,     // Stop Charge
//  MIRR,     // Mirror Contact
//  RCBO,     // Current Protection
//  CPSE,     // Short Circuit Control Pilot
//  PPSE,     // Short Circuit Plug Presence
//  CPLS,     // Lost Control Pilot
//  PPLS,     // Lost Plug Presence
//  VBUS,     // Main Break Down
//  MFRE,     // Mifare Reader
//  EMTR,     // Energy Meter
//  OVCE,     // Over Current Protection
//  RCTE,     // Presence Rectifier Diode
//};
//typedef enum _socketCheck SocketCheck_t;

/* Socket */
struct _socket
{
  char name[MAX_NAME_LENGTH];
  /* Info */
  char serial[16];
  char firmware[24];
  uint8_t wiring;
  energy_meter_e mt_int;
  energy_meter_e mt_ext;
  uint8_t rfid;
  int32_t max_current;
  int32_t max_power;
  /* State */
  uint8_t id;
  sck_state_t state;
  modeFun_e modeFun;
  modePwr_e modePwr;
  /* Measures */
  sck_measures_t measures;
  //uint8_t checks[2];
  //uint8_t actuators[2];
  sck_register_t reg;
  /* Authorization */
  sck_auth_t auth;
  /* Schedulation */
  sck_schedule_t scheds[MAX_SCHEDS];
};
typedef struct _socket socket_t;

/* Task Bluetooth */
enum _op_code
{
  UNKNOWN = 0,
  RESPONSE = 1,
  START_CHARGE = 2,
  START_SESSION = 3,
  STOP_CHARGE = 4,
  STOP_SESSION = 5,
  SUSPEND_CHARGE = 6,
  RELEASE_SUSPEND_CHARGE = 7,
  REQ_ACTIVE_TRANS = 8,
  ACTIVE_SCHEDULE = 9,
  DISABLE_SCHEDULE = 10,
  REQ_REGISTER = 11,
  GET_AUTH = 19,
  INIT_AUTH = 20,
  SET_AUTH = 21,
  REQ_AUTH = 23,
  REQ_CHANGE_AUTH = 24,
  SEND_APP_ID = 25
};
typedef enum _op_code op_code_e;

/************************* end Socket definitions **********************************/

/************************* Bluetooth Task definitions **********************************/
/* State of bluetooth task */
typedef enum
{
  INITIALIZE = 0,
  DISCONNECTED,
  DISCOVERABLE,
  IN_CONNECTION,
  CONNECTED,
  WAIT_ACK
} BluetoothState;

enum _sck_bt_req_type
{
  REQ_UNKNOWN = (uint8_t)0x00,
  READ_STATE,
  WRITE_STATE,
  UPDATE_MEASURES,
  READ_REGISTER,
  SEND_REGISTER,
  SEND_START_ACTIVE_TRANS,
  SEND_STOP_ACTIVE_TRANS,
  RETRY_SEND_LONG_CHAR,
  UPDATE_ID
};
typedef enum _sck_bt_req_type bt_req_type_e;

struct _bt_task_request
{
  bt_req_type_e type;
  sck_state_t state;
  sck_measures_t measures;
  uint16_t serv_handle;
  uint16_t char_handle;
};
typedef struct _bt_task_request BtTaskReq_t;

/******************* end Bluetooth Task definitions ****************************/


/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
void                  blueMngTask             (void * pvParameters);
osMessageQueueId_t    getBlueMngQueueHandle   (void);
BluetoothState *      getBtState              (void);

/* Bt Task send manager */
uint8_t requestToBtTask(BtTaskReq_t *req);
void busyBtTask();
void releaseBtTask();

void init_socket(socket_t *sck);
void init_measures(sck_measures_t *measures);

/* Socket Getter Functions */
char * get_sck_name();
char * get_sck_serial();
char * get_sck_firmware();
uint8_t get_sck_wiring();
uint16_t get_sck_emeters();
energy_meter_e get_sck_int_emeter();
energy_meter_e get_sck_ext_emeter();
uint8_t get_sck_rfid();
int32_t get_sck_max_current();
int32_t get_sck_max_power();
uint8_t get_sck_id();
sck_state_t get_sck_state();
modeFun_e get_sck_fun_mode();
modePwr_e get_sck_pwr_mode();
sck_measures_t * get_sck_measures();
sck_register_t * get_sck_register();
sck_auth_t * get_sck_auth();

/* Socket Setter Functions */
void set_sck_name               (char *name, uint8_t length);
void set_sck_mode               (modeFun_e mode);
void set_sck_max_current        (int32_t max_current);
void set_sck_state              (sck_state_t state);
void set_sck_id                 (int id);
void set_sck_pwr_mode           (modePwr_e currMode);
void set_sck_wiring             (uint8_t newWiring);

/* Authorization Functions */
void set_sck_auth(char *user, char *pass);
uint8_t check_sck_auth(char *user, char *pass);
void set_app_id(uint16_t id);

/* Socket Process Functions */
void start_charge();
uint8_t stop_charge();
void active_schedule();
void disable_schedule();
void lock_socket();
void unlock_socket();
void unknown_cmd();

/* Socket Transactions Register Functions */
void init_transaction_register(sck_register_t *reg, unsigned int n);
void populate_transaction_register(sck_register_t *reg); // Only for debug

/* Socket Schedulation Functions */
sck_schedule_t * get_schedulation();
void sched_received(uint8_t *data, uint8_t data_length);

#endif //  __BLUEMNG_H


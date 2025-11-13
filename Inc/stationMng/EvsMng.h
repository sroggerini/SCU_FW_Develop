/**
* @file        EvsMng.h
*
* @brief       Events manager - Definition -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: EvsMng.h 743 2025-04-17 12:56:36Z vania $
*
*     $Revision: 743 $
*
*     $Author: vania $
*
*     $Date: 2025-04-17 14:56:36 +0200 (gio, 17 apr 2025) $
*
*
* @copyright
*       Copyright (C) 2016 SCAME S.p.A. All rights reserved. This file is copyrighted and the property of Aesys
*       S.p.A.. It contains confidential and proprietary information. Any copies of this file (in whole or in part)
*       made by any method must also include a copy of this legend. Developed by:  SCAME S.p.A.
***********************************************************************************************************************/


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#ifndef _EVSMNG_H
#define _EVSMNG_H
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global include ------------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global defines ------------------------------------------------------------------------------------------------------------------------- //
#define RS485_ADD_MIN           (uint8_t)(1)    // minimo valore indirizzo SCU su bus RS-485
#define RS485_ADD_MAX           (uint8_t)(15)   // massimo valore indirizzo SCU su bus RS-485   /* RIVEDERE: mettere nel posto giusto @Nick */

#define EVS_ERROR_ARRAY_SIZE    (uint8_t)(3)

#define CONTROL_BYTE_0          (uint8_t)(0)
#define CONTROL_BYTE_1          (uint8_t)(1)
#define CONTROL_BYTE_2          (uint8_t)(2)
#define CONTROL_BYTE_3          (uint8_t)(3)

#define RCDM_CRL0               (uint8_t)(0x01) // abilitazione dei controlli
#define LID_CRL0                (uint8_t)(0x02) // abilitazione dei controlli
#define VENT_CRL0               (uint8_t)(0x04) // abilitazione dei controlli
#define BLOCK_CRL0              (uint8_t)(0x08) // abilitazione dei controlli
#define REMOTE_CRL0             (uint8_t)(0x10) // abilitazione dei controlli
#define PULS_CRL0               (uint8_t)(0x20) // abilitazione dei controlli
#define MIRROR_CRL0             (uint8_t)(0x40) // abilitazione dei controlli
#define RCBO_CRL0               (uint8_t)(0x80) // abilitazione dei controlli

#define RCDM_ANOM0              (RCDM_CRL0)
#define LID_ANOM0               (LID_CRL0)
#define VENT_ANOM0              (VENT_CRL0)
#define BLOCK_ANOM0             (BLOCK_CRL0)
#define NU04_ANOM0              (REMOTE_CRL0)
#define NU05_ANOM0              (PULS_CRL0)
#define MIRROR_ANOM0            (MIRROR_CRL0)
#define RCBO_ANOM0              (RCBO_CRL0)

#define EVS_RESETTABLE_ERROR0   (VENT_ANOM0 | BLOCK_ANOM0)

#define CPSHORT_CRL1            (uint8_t)(0x01) // abilitazione dei controlli
#define PPSHORT_CRL1            (uint8_t)(0x02) // abilitazione dei controlli
#define CPLOST_CRL1             (uint8_t)(0x04) // abilitazione dei controlli
#define PPLOST_CRL1             (uint8_t)(0x08) // abilitazione dei controlli
#define VBUS_CRL1               (uint8_t)(0x10) // abilitazione dei controlli
#define MIFARE_CRL1             (uint8_t)(0x20) // abilitazione dei controlli
#define EMETER_INT_CRL1         (uint8_t)(0x40) // abilitazione dei controlli
#define OVERCURRENT_CRL1        (uint8_t)(0x80) // abilitazione dei controlli

#define CPSHORT_ANOM1           (CPSHORT_CRL1)
#define PPSHORT_ANOM1           (PPSHORT_CRL1)
#define CPLOST_ANOM1            (CPLOST_CRL1)
#define PPLOST_ANOM1            (PPLOST_CRL1)
#define VBUS_ANOM1              (VBUS_CRL1)
#define MIFARE_ANOM1            (MIFARE_CRL1)
#define EMETER_INT_ANOM1        (EMETER_INT_CRL1)
#define OVERCURRENT_ANOM1       (OVERCURRENT_CRL1)

#define EVS_RESETTABLE_ERROR1   (OVERCURRENT_ANOM1 | CPSHORT_ANOM1 | PPSHORT_ANOM1 | CPLOST_ANOM1 | PPLOST_ANOM1)

#define RECTIFIER_CRL2          (uint8_t)(0x01) // abilitazione dei controlli
#define EMETER_EXT_CRL2         (uint8_t)(0x02) // abilitazione dei controlli
#define SINAPSI_CHN2_CRL2       (uint8_t)(0x04) // abilitazione dei controlli
#define PAUT_CRL2               (uint8_t)(0x08) // abilitazione dei controlli
#define NU25_CRL2               (uint8_t)(0x10) // abilitazione dei controlli
#define HGTP_CRL2               (uint8_t)(0x20) // abilitazione dei controlli
#define NU26_CRL2               (uint8_t)(0x40) // abilitazione dei controlli
#define SIN_RES_ERR_CRL2        (uint8_t)(0x80) // controllo azzeramento errori 

#define HGTP_CRL2_bit_pos       (uint8_t)(5)    // bit position

#define WIFI_CRL3               (uint8_t)(0x20) // controllo errori per wifi 

#define RECTIFIER_ANOM2         (RECTIFIER_CRL2)
#define EMETER_EXT_ANOM2        (EMETER_EXT_CRL2)
#define SINAPSI_CHN2_ANOM2      (SINAPSI_CHN2_CRL2)
#define NU23_ANOM2              (NU23_CRL2)
#define NU24_ANOM2              (NU24_CRL2)
#define NU25_ANOM2              (NU25_CRL2)
#define NU26_ANOM2              (NU26_CRL2)
#define NU27_ANOM2              (NU27_CRL2)

#define EVS_RESETTABLE_ERROR2   (RECTIFIER_ANOM2)

#define WIFI_ANOM3              (WIFI_CRL3)     //  anomalia funzionamento wifi 

#define NU00_ATT0               (uint8_t)(0x01)
#define NU01_ATT0               (uint8_t)(0x02)
#define NU02_ATT0               (uint8_t)(0x04)
#define BLOCK_ATT0              (BLOCK_CRL0)
#define PAUT_ATT0               (uint8_t)(0x10)
#define BBCK_ATT0               (uint8_t)(0x20)
#define CONTACT_ATT0            (MIRROR_CRL0)
#define RCBO_ATT0               (RCBO_CRL0)

#define EVS_TETHERED            (uint8_t)(0x20)
#define LID_OPEN_IN_CHARGE      (uint8_t)(0x04)
#define LID_CLOSE_IN_CHARGE     (uint8_t)(0x08)

#define SOCKET_TYPE_NULL        (uint8_t)(0x00)                         //  0 - 0x00: il tipo di presa non è stato impostato
#define SOCKET_T2_NO_LID        (uint8_t)(0xD2)                         //  1 - 0xD2: presa tipo 2 senza coperchio
#define SOCKET_T2_CLOSE_LID     (uint8_t)(0xD0 | LID_CLOSE_IN_CHARGE)   //  2 - 0xD8: presa tipo 2 coperchio chiuso in carica
#define SOCKET_T2_OPEN_LID      (uint8_t)(0xD0 | LID_OPEN_IN_CHARGE)    //  3 - 0xD4: presa tipo 2 coperchio aperto in carica
#define SOCKET_3C_OPEN_LID      (uint8_t)(0xD1 | LID_OPEN_IN_CHARGE)    //  4 - 0xD5: presa tipo 3C coperchio aperto in carica
#define SOCKET_3C_NO_LID        (uint8_t)(0xC0)                         //  5 - 0xC0: presa tipo 3C senza blocchi
#define SOCKET_3A_OPEN_LID      (uint8_t)(0x90 | LID_OPEN_IN_CHARGE)    //  6 - 0x94: presa tipo 3A coperchio aperto in carica
#define SOCKET_3A_NO_LID        (uint8_t)(0x80)                         //  7 - 0x80: presa tipo 3A senza blocchi
#define SOCKET_SK_CLOSE_LID     (uint8_t)(0x90 | LID_CLOSE_IN_CHARGE)   //  8 - 0x98: presa tipo schuko coperchio chiuso in carica
#define SOCKET_SK_NO_LID        (uint8_t)(0x81)                         //  9 - 0x81: presa tipo schuko senza blocchi
#define SOCKET_T1_TETHERED      (uint8_t)(0xC2 | EVS_TETHERED)          // 10 - 0xE2: presa tipo 1 cablata (PP sempre presente)
#define SOCKET_T2_TETHERED      (uint8_t)(0xC0 | EVS_TETHERED)          // 11 - 0xE0: presa tipo 2 cablata (PP sempre presente)

#define SOCKET_TYPE_MIN         (uint8_t)(1)
#define SOCKET_TYPE_MAX         (uint8_t)(11)

#define DISP_WAIT_POINTS        (uint8_t)(20)

#define CHARGE_BACKUP_MASK      (uint32_t)(0x000000FF)
#define SUSP_BACKUP_MASK        (uint32_t)(0x0000FF00)
#define CMODE_BACKUP_MASK       (uint32_t)(0x00FF0000)
#define VALID_BACKUP_MASK       (uint32_t)(0xFF000000)

#define CHARGE_BACKUP_SHIFT     (uint8_t)(0)
#define SUSP_BACKUP_SHIFT       (uint8_t)(8)
#define CMODE_BACKUP_SHIFT      (uint8_t)(16)

#define CHARGE_BACKUP_VAL       (uint32_t)(0x000000A1)
#define VALID_BACKUP_VAL        (uint32_t)(0xA7000000)

#define AUTH_START_SAVE             (uint32_t)(0xA5000001)
#define FREE_START_SAVE             (uint32_t)(0xA5000002)
#define AUTH_RESET_SAVE             (uint32_t)(0xA5000000)
#define M3T_CHARGE_SAVE             (uint32_t)(0xA5001100)
#define M3S_CHARGE_SAVE             (uint32_t)(0xA5002200)
#define NULL_CHARGE_SAVE            (uint32_t)(0xA5000000)

#define PERSONAL_BY_CARD         (uint8_t)(0x00)                         /* Personal mode charge started by card */
#define PERSONAL_BY_APP          (uint8_t)(0x01)                         /* Personal mode charge started by app  */

#define CTRL_HGTP_BIT            (uint8_t)(0x01) // abilitazione controllo over temperature


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global typedef ------------------------------------------------------------------------------------------------------------------------- //
typedef enum // possible value of charging_mode
{
CHARGING_MODE_NULL = 0,
M3T_CHARGING_MODE,
M3S_CHARGING_MODE
} charging_mode_en;

typedef enum // possible value of evs_mode
{
EVS_FREE_MODE = (uint8_t)(0x00),
EVS_PERS_MODE = (uint8_t)(0x01),
EVS_NET_MODE  = (uint8_t)(0x02),
EVS_OCPP_MODE = (uint8_t)(0x03),
EVS_MAX_MODE  = (uint8_t)(0x04)
} evs_mode_en;

#define EVS_AUTHORIZATION_MIN   (EVS_FREE_MODE)
#define EVS_AUTHORIZATION_MAX   (EVS_OCPP_MODE)

typedef enum // possible value of suspending_set
{
SUSPENDING_NULL     = (uint16_t)(0x0000),
REMOTE_SUSPENDING   = (uint16_t)(0x0002),
UART_SUSPENDING     = (uint16_t)(0x0004),
PMNG_SUSPENDING     = (uint16_t)(0x0008),
V230_SUSPENDING     = (uint16_t)(0x0010),
PEN_SUSPENDING      = (uint16_t)(0x0020),
SINAPSI_SUSPENDING  = (uint16_t)(0x0040),
HTS_SUSPENDING      = (uint16_t)(0x0080),
EMEX_SUSPENDING     = (uint16_t)(0x0100),
CHN2_SUSPENDING     = (uint16_t)(0x0200),
APP_SUSPENDING      = (uint16_t)(0x0400),
} suspending_en;

typedef enum // possible value of power_management_mode
{
PMNG_FULL           = (uint8_t)(0x00),
PMNG_ECO_SMART      = (uint8_t)(0x01),
PMNG_ECO_PLUS       = (uint8_t)(0x02),
PMNG_MODE_MASK      = (uint8_t)(PMNG_FULL | PMNG_ECO_SMART | PMNG_ECO_PLUS),
PMNG_NUM            = (uint8_t)(0x03),
PMNG_SINAPSI_SMART  = (uint8_t)(0x10)
} power_management_mode_en;

typedef enum // possible value of power_management_unbal
{
PMNG_UNBAL_OFF  = (uint8_t)(0x00),
PMNG_UNBAL_ON   = (uint8_t)(0x01)
} power_management_unbal_en; 

typedef enum // possible value of evs_iso15118_state
{
ISO15118_NULL = 0,      /* 00 */
ISO15118_WAIT,          /* 01 */
ISO15118_OK,            /* 02 */
ISO15118_NOK            /* 03 */
} iso15118_state_en;

typedef enum // possible value of evs_state
{
EVSTATE_NULL = 0,               /* 00 */ // not valid evs_state
EVSTATE_IDLE,                   /* 01 */ // idle evs_state
EVSTATE_INIT,                   /* 02 */ // first power on state from hw/fw reset: wait for block movements control [BlockMngTask]
EVSTATE_CPSET,                  /* 03 */ // wait stable volatage after CP set [+12V]
EVSTATE_DELAY,                  /* 04 */
EVSTATE_MODE_SEL,               /* 05 */ // dummy state
EVSTATE_DISABLED,               /* 06 */ // No user allowed
EVSTATE_AUTH_WAIT,              /* 07 */ // wait for user authorization / remote supervisor action
EVSTATE_SOCKET_AVAILABLE,       /* 08 */ // evs_mode == EVS_FREE_MODE
EVSTATE_AUTH_PENDING,           /* 09 */ // user authorization required
EVSTATE_AUTH_MISSED,            /* 10 */ // user authorization required failed
EVSTATE_PLUG_WAIT,              /* 11 */ // evs_mode = EVS_PERS_MODE, EVS_NET_MODE o EVS_OCPP_MODE
EVSTATE_LID_WAIT,               /* 12 */ // evs_mode = EVS_PERS_MODE, EVS_NET_MODE o EVS_OCPP_MODE [from EVSTATE_PLUG_WAIT to ]
EVSTATE_PLUG_CHECK,             /* 13 */ // wait for plug_presence = plug_configuration [see: PlugManager]
EVSTATE_SOCKET_CHECK,           /* 14 */ // check socket safety condition
EVSTATE_BLOCK_UP_DELAY,         /* 15 */ // wait timeout before drive block up
EVSTATE_BLOCK_DOWN_DELAY,       /* 16 */ // wait timeout before drive block down
EVSTATE_M3S_M3T_DETECT,         /* 17 */ // detect M3 standard / simplified
EVSTATE_RECTIFIER_CHECK,        /* 18 */ // check rectifier presence
EVSTATE_SUSPENDING,             /* 19 */ // forced suspension
EVSTATE_WAKEUP,                 /* 20 */ // ev wakeup transition
EVSTATE_S2_WAITING,             /* 21 */ // wait for S2 closing
EVSTATE_CONTACT_CLOSE_DELAY,    /* 22 */ // wait timeout before drive contact close
EVSTATE_CHARGING,               /* 23 */ // EV in charge
EVSTATE_INTERRUPTING,           /* 24 */ // wait for S2 opening or timeout
EVSTATE_F_STATE_ERROR,          /* 25 */ // 
EVSTATE_X1_STATE_ERROR,         /* 26 */ // 
EVSTATE_RES_ERROR,              /* 27 */ // show evs error
EVSTATE_PLUG_OUT,               /* 28 */ // wait for EVS_PLUG_OUT
EVSTATE_LID_ERROR,              /* 29 */ // wait for lid closing: plug not inserted
EVSTATE_PAUT_LID,               /* 30 */
EVSTATE_CLOSE_LID,              /* 31 */ // wait for lid closing: plug inserted
EVSTATE_AUTH_NEG,               /* 32 */ // authorization denied
EVSTATE_ERROR_WAIT,             /* 33 */ // wait for resettable error clear
EVSTATE_PEN_STATE,              /* 34 */ 
EVSTATE_V230_SUSPEND,           /* 35 */ 
EVSTATE_POWER_OFF,              /* 36 */ 
EVSTATE_LID_ERROR_DELAY,        /* 37 */
EVSTATE_ISO15118_WAIT,          /* 38 */
EVSTATE_ISO15118_WAKEUP         /* 39 */
} evs_state_en;

typedef enum // possible value of evs_modbus_state
{
MDBSTATE_STARTING = 0,          /* 00 */ // modbus state in according to par. 7.1.4.2
MDBSTATE_AVAILABLE,             /* 01 */
MDBSTATE_PREPARING,             /* 02 */
MDBSTATE_EV_CONNECTED,          /* 03 */
MDBSTATE_CHARGING,              /* 04 */
MDBSTATE_SUSPENDING_EV,         /* 05 */
MDBSTATE_SUSPENDING_EVSE,       /* 06 */
MDBSTATE_END_CHARGE,            /* 07 */
MDBSTATE_FAULTED,               /* 08 */
MDBSTATE_UNAVAILABLE,           /* 09 */
MDBSTATE_RESERVED,              /* 10 */
MDBSTATE_BOOTLOADER,            /* 11 */
MDBSTATE_SHUTDOWN,              /* 12 */
MDBSTATE_REBOOTING,             /* 13 */
MDBSTATE_SUSPENDED_EVSE_NOPOWER,/* 14 */
MDBSTATE_NUM_STATE,             /* 15 */
MDBSTATE_NULL_STATE             /* 16 */    // per init di evsModbusState e evsModbusState_old
}evs_modbus_state_en;

typedef enum
{
EVS_EVENT_NULL = 0,
EVS_EVENT_START,
EVS_BLOCK_STEADY,
EVS_AUTORIZATION_MODE,
SEM_AUTORIZATION_MODE,
EVS_PLUG_DETECTED,
EVS_PLUG_INSERTED,
EVS_PLUG_BACK,
EVS_PLUG_OUT,
EVS_xP_SHORT,
EVS_S2_STATE_UPDATE,  // 9
EVS_RECT_UPDATE,
EVS_PULS_STOP,
EVS_REMOTE_SUSPENDING,
EVS_REMOTE_RELEASE,
EVS_APP_SUSPENDING,
EVS_APP_RELEASE,
EVS_UART_SUSPENDING,
EVS_UART_RELEASE,
EVS_PMNG_SUSPENDING,
EVS_PMNG_RELEASE,
EVS_HTS_SUSPENDING,
EVS_HTS_RELEASE,
EVS_AUTH_START,
EVS_AUTH_REQUIRED,
EVS_AUTH_STOP,
EVS_AUTH_NEG,
EVS_RFID_ANOM1_UPDATE,
EVS_LID_UPDATE,
EVS_RCDM_OPEN,
EVS_MIRROR_ERROR,
EVS_OVERCURRENT_ERROR,
EVS_RCBO_CLOSE,
EVS_MIRROR_READ,
EVS_INTERNAL_EM_GOOD,
EVS_INTERNAL_EM_FAIL,
EVS_INTERNAL_EM_UPDATE,
EVS_EXTERNAL_EM_GOOD,
EVS_EXTERNAL_EM_FAIL,   // 0x22 = 34
EVS_EXTERNAL_EM_UPDATE,
EVS_ERROR_CAPTURE,
CPSET_TIM_EXPIRED,
EVS_SEC_TIM_EXPIRED,
ERROR_WAIT_TIM_EXPIRED,
CPPP_WAIT_TIM_EXPIRED,
EVS_STATE_TIM_EXPIRED,
EVS_POST_SUSPENDING_EXPIRED,
EVS_SUSPENDING_TIM_EXPIRED,
EVS_EVENT_PRINT,        // 0x2A = 42
EVS_V230_SUSPEND,       // main power off and backup active ---> Start emergency phase and suspend charging
EVS_V230_OFF,           // main power off ---> Start emergency phase with immediate stop charging and open socket 
EVS_V230_ON,            // main power on ---> come back from emergency phase
EVS_PEN_ALM_OFF,        // PEN alarm OFF                                    
EVS_PEN_ALM_ON,         // PEN alarm ON                                     
EVS_VENT_UPDATE,        // update ventilazione
EVS_SINAPSI_UPDATE,     // update info from Sinapsi (M1 chain 2)
EVS_WIFI_ANTENNA_ERROR,
EVS_SINAPSI_CHN2_FAIL,  // fail del canale 
EVS_SINAPSI_CHN2_GOOD,   // good del canale 
EVS_TIMESTAMP_TIM_EXPIRED,
LID_ERROR_DELAY_TIM_EXPIRED,
EVS_ISO15118_TIM_EXPIRED,
EVS_ISO15118_INFO_DEVICE_UPDATE
} EvsMngEvent_en;

typedef enum
{
EVS_STATE_TIM = 0,
CPSET_STATE_TIM,
EVS_SEC_TIM,
EVS_WAIT_TIM,
ERROR_WAIT_TIM,
CPPP_WAIT_TIM,
EVS_TPRINT_TIM,
EVS_UID_ERROR_TIM,
EVS_SUSPENDING_TIM,
EMETER_INT_ANOM1_TIM,
SBC_GSY_TIM,
EVS_TIMESTAMP_TIM,
LID_ERROR_DELAY_TIM,
EVS_POST_SUSPENDING_TIM,
EVS_ISO15118_TIM,
EVS_NUM_TIMER
} EvsMngTim_en;

/* queue info structure */
typedef __packed struct
{
EvsMngEvent_en      EvsMngEvent;
} EvsMngMsg_st;

#ifdef HW_MP28947

/***********     These definitions are just to avoid compiler errors       ***************/

enum info_host_param_sel {
    ISO15118_MAXCURRENT,
    ISO15118_STOPEVSE,
    ISO15118_STATECP,
    ISI15118_PAUSEEVSE,
    ISO15118_EMERGENCY
};

enum evs_iso15118_data_req_val {
	ISO15118_CHARGING_STOP          = 0x00,   /*  0U */
	ISO15118_CHARGING_START         = 0x01,   /*  1U */
	ISO15118_CHARGING_PAUSE         = 0x02,   /*  2U */
	ISO15118_V2GSEQUENCEPAUSE       = 0x1C,   /* 28U */
	ISO15118_V2GSEQUENCEEND         = 0x1D    /* 29U */
};

#define EVCCID_STRING_LEN		8

typedef struct
{
  
  uint8_t       State_openv2g;
  uint8_t       Term_openv2g;
  uint8_t       State_Slac;
  uint8_t       State_SDP;
  uint8_t       Error_Code;
  uint8_t       evccId[EVCCID_STRING_LEN];
  uint32_t      ev_Departure_Time;
  uint16_t      ev_Max_Current_Limit;
  uint16_t      ev_Min_Current;
  uint16_t      ev_Max_Voltage_Limit;
  uint16_t      ev_Energy_Request;
  uint8_t       ev_Protocol;
  uint8_t       serviceIdReq;
  
} ISO15118_Info_Device_st;

#endif

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
extern  uint8_t                 startPersonal;

extern  evs_modbus_state_en     evsModbusState;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global function prototypes ------------------------------------------------------------------------------------------------------------- //
void                  send_to_evs                     (uint8_t evs_event);
charging_mode_en      evs_charging_mode_get           (void);
void                  evs_error_set                   (uint8_t error_byte, uint8_t error_bit, uint8_t error_val);
uint8_t               evs_error_get                   (uint8_t *dst_ptr, uint8_t mifare_error_hide, uint8_t lid_error_hide, uint8_t emex_error_hide);
uint8_t               evs_suspending_set_get          (void);
void                  evs_control_save                (void);
void                  evs_chn2_init                   (void);
void                  evs_error_control               (uint8_t *control, uint16_t add, uint8_t num);
void                  evs_sec_get                     (uint8_t *dst_ptr);
evs_state_en          evs_state_get                   (void);
uint8_t               evs_mode_get                    (void);
uint8_t               evs_charging_resume_get         (void);
void                  evs_charging_resume_reset       (void);
uint8_t               evs_charging_resume_sem_get     (void);
void                  evs_charging_resume_sem_reset   (void);
uint32_t              evs_charge_status_get           (void);
void                  EvsMngTask                      (void *pvParameters);
uint8_t               vbus_anom1_save_get             (void);
void                  updateModbusErrorRegisters      (uint8_t* errorArray, uint8_t forceUpd);
void                  ledColorInUplaod                (void);
uint8_t               internal_chn2_error_enable_get  (void);
void                  internal_chn2_error_enable_reset(void);
uint8_t               chn2_error_enable_get           (void);
void                  chn2_error_enable_set           (void);
uint8_t               evs_gost_param_get              (void);
void                  ledColorPacketErrorUplaod       (void);
void                  ledColorUplaodFail              (void);
void                  ledColorUplaodOk                (void);
evs_modbus_state_en   getLastEvsStableStatus          (void);
uint32_t              evs_rtc_backup_get              (uint32_t reg);
void                  evs_reserved_set                (uint8_t val);
uint8_t               evs_reserved_get                (void);
void                  setWaitTimeValue                (uint16_t timeValue);
uint8_t               evs_iso15118_run_get            (void);
void                  ISO15118_homeplugdev_info_host_set(uint8_t info_sel, uint16_t val);

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#endif

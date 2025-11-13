// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//
//  File:           PersMng.h
//  Author:         Vania
//  Date:           06/11/2020
//
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#ifndef _PERSMNG_H
#define _PERSMNG_H
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global include ------------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global defines ------------------------------------------------------------------------------------------------------------------------- //
#define UID_DELETE_SEC          (uint8_t)(5)
#define UID_CONFIRM_SEC         (uint8_t)(5)
#define UPDATE_WLIST_SEC        (uint8_t)(20)
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global typedef ------------------------------------------------------------------------------------------------------------------------- //
typedef enum // possible value for card_auth_type output
{
NULL_AUTH           = (uint8_t)(0x00),
DATE_CHARGE_AUTH    = (uint8_t)(0x01),
CREDIT_CHARGE_AUTH  = (uint8_t)(0x02),
TIME_CHARGE_AUTH    = (uint8_t)(0x04),
ENERGY_CHARGE_AUTH  = (uint8_t)(0x08),
NO_LIMIT_AUTH       = (uint8_t)(0x10),
EXPIRED_CARD        = (uint8_t)(0x40),
CREDIT_EXHAUSTED    = (uint8_t)(0x80)
}card_auth_en;

typedef enum // possible value of pers_state
{
PERS_IDLE_STATE = 0,
PERS_MODE_INACTIVE,
PERS_MODE_ACTIVE,
PERS_MODE_UID_START,
PERS_MODE_UID_BUSY,
PERS_SHOW_DATA,
PERS_UID_CONFIRM,
PERS_UPDATE_WLIST,
PERS_UID_DELETE,
PERS_WLIST_DELETE,
PERS_WLIST_DELETED,
PERS_MODE_BLIND
}pers_state_en;

typedef enum
{
PERS_EVENT_NULL = 0,
PERS_START,
PERS_CARD_READ,
PERS_UID_RELEASE,
PERS_PULS_SET,
PERS_PULS_RESET,
PERS_AUTORIZATION_MODE,
PERS_DATA_TIM_EXPIRED,
PERS_CARD_CHARGE_EXPIRED,
PERS_RTC_BACKUP_SAVE,
PERS_RTC_BACKUP_START,
UID_CONFIRM_TIM_EXPIRED,
UPDATE_WLIST_TIM_EXPIRED,
UID_DELETE_TIM_EXPIRED,
PERS_BLIND_TIM_EXPIRED,
WLIST_DELETE_TIM_EXPIRED
}PersMngEvent_en;

typedef enum // Definizioni tipo carta per modbus_uid_manager
{
UID_MASTER = (uint8_t)(0x00),
UID_USER
}uid_type_en;

typedef enum // Definizioni codifica uid per modbus_uid_manager
{
HEX_UID = (uint8_t)(0x00),
ASCII_UID
}uid_code_en;

/* queue info structure */
typedef __packed struct
{
PersMngEvent_en    PersMngEvent;
}PersMngMsg_st;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global function prototypes ------------------------------------------------------------------------------------------------------------- //
void send_to_pers(uint8_t pers_event);
void pers_sec_get(uint8_t *dst_ptr);
void user_card_date_get(uint8_t *dst_ptr);
void user_card_credit_get(uint16_t *dst_ptr);
uint32_t user_card_time_decrease(void);
int32_t user_card_energy_get(void);
card_auth_en user_card_auth_get(void);
void modbus_uid_write(uid_type_en uid_type, uint8_t *uid_ptr, uid_code_en uid_code);
void manual_uid_factory(void);
void PersMngTask(void *pvParameters);
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#endif

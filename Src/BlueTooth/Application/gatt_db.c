/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
* File Name          : gatt_db.c
* Author             :
* Version            : V1.0.0
* Date               : 16-September-2015
* Description        : Functions to build GATT DB and handle GATT events.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "bluenrg_def.h"
#include "gatt_db.h"
#include "bluenrg_conf.h"
#include "bluenrg_gatt_aci.h"
#include "flashFat.h"

#include "blueMng.h"
#include "eeprom.h"

/** @brief Macro that stores Value into a buffer in Little Endian Format (2 bytes)*/
#define HOST_TO_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

/** @brief Macro that stores Value into a buffer in Little Endian Format (4 bytes) */
#define HOST_TO_LE_32(buf, val)    ( ((buf)[0] =  (uint8_t) (val)     ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[2] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[3] =  (uint8_t) (val>>24) ) )

/** @brief Macro that stores Value into a buffer in Little Endian Format (8 bytes) */
#define HOST_TO_LE_64(buf, val)    ( ((buf)[0] =  (uint8_t) (val)     ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[2] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[3] =  (uint8_t) (val>>24) ) , \
                                   ((buf)[4] =  (uint8_t) (val>>32) ) , \
                                   ((buf)[5] =  (uint8_t) (val>>40) ) , \
                                   ((buf)[6] =  (uint8_t) (val>>48) ) , \
                                   ((buf)[7] =  (uint8_t) (val>>56) ) )

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

/* Scame Socket Services */
#define COPY_INFO_SOCKET_SERVICE_UUID(uuid_struct)      COPY_UUID_128(uuid_struct,0x52,0xf3,0x03,0x34,0x8b,0xdc,0x11,0xeb,0x8d,0xcd,0x02,0x42,0xac,0x13,0x00,0x03)
#define COPY_STATE_SOCKET_SERVICE_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0x52,0xf3,0x0c,0x58,0x8b,0xdc,0x11,0xeb,0x8d,0xcd,0x02,0x42,0xac,0x13,0x00,0x03)

/* Scame Socket Characteristics */
#define COPY_SERIAL_CHAR_UUID(uuid_struct)              COPY_UUID_128(uuid_struct,0x52,0xf3,0x05,0x82,0x8b,0xdc,0x11,0xeb,0x8d,0xcd,0x02,0x42,0xac,0x13,0x00,0x03)
#define COPY_FIRMWARE_CHAR_UUID(uuid_struct)            COPY_UUID_128(uuid_struct,0x52,0xf3,0x06,0x72,0x8b,0xdc,0x11,0xeb,0x8d,0xcd,0x02,0x42,0xac,0x13,0x00,0x03)
#define COPY_ENERGYMETER_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x52,0xf3,0x07,0x62,0x8b,0xdc,0x11,0xeb,0x8d,0xcd,0x02,0x42,0xac,0x13,0x00,0x03)
#define COPY_RFID_CHAR_UUID(uuid_struct)                COPY_UUID_128(uuid_struct,0x52,0xf3,0x08,0x20,0x8b,0xdc,0x11,0xeb,0x8d,0xcd,0x02,0x42,0xac,0x13,0x00,0x03)
#define COPY_MAXCURRENT_CHAR_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x52,0xf3,0x08,0xd4,0x8b,0xdc,0x11,0xeb,0x8d,0xcd,0x02,0x42,0xac,0x13,0x00,0x03)
#define COPY_WIRING_CHAR_UUID(uuid_struct)              COPY_UUID_128(uuid_struct,0x52,0xf3,0x09,0x88,0x8b,0xdc,0x11,0xeb,0x8d,0xcd,0x02,0x42,0xac,0x13,0x00,0x03)
#define COPY_MAXPOWER_CHAR_UUID(uuid_struct)            COPY_UUID_128(uuid_struct,0xb1,0xc4,0xae,0xc2,0xa8,0x2d,0x11,0xeb,0xbc,0xbc,0x02,0x42,0xac,0x13,0x00,0x02)
#define COPY_PWRMODE_CHAR_UUID(uuid_struct)             COPY_UUID_128(uuid_struct,0xf5,0x1c,0x8a,0x9a,0xb2,0x3d,0x11,0xeb,0x85,0x29,0x02,0x42,0xac,0x13,0x00,0x03)

#define COPY_IDSOCKET_CHAR_UUID(uuid_struct)            COPY_UUID_128(uuid_struct,0x52,0xf3,0x0d,0x2a,0x8b,0xdc,0x11,0xeb,0x8d,0xcd,0x02,0x42,0xac,0x13,0x00,0x03)
#define COPY_STATEDESC_CHAR_UUID(uuid_struct)           COPY_UUID_128(uuid_struct,0x52,0xf3,0x0d,0xde,0x8b,0xdc,0x11,0xeb,0x8d,0xcd,0x02,0x42,0xac,0x13,0x00,0x03)
#define COPY_MODE_CHAR_UUID(uuid_struct)                COPY_UUID_128(uuid_struct,0x52,0xf3,0x0e,0x92,0x8b,0xdc,0x11,0xeb,0x8d,0xcd,0x02,0x42,0xac,0x13,0x00,0x03)

//#define COPY_DATA_CHAR_UUID(uuid_struct)                COPY_UUID_128(uuid_struct,0xf5,0x1c,0x87,0xfc,0xb2,0x3d,0x11,0xeb,0x85,0x29,0x02,0x42,0xac,0x13,0x00,0x03)

/* Scame Socket Register */
//#define COPY_REGISTER_SOCKET_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0xf5,0x1c,0x88,0xe2,0xb2,0x3d,0x11,0xeb,0x85,0x29,0x02,0x42,0xac,0x13,0x00,0x03)

/* Scame Socket Security */
#define COPY_SECURITY_SOCKET_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0xf5,0x1c,0x84,0xbe,0xb2,0x3d,0x11,0xeb,0x85,0x29,0x02,0x42,0xac,0x13,0x00,0x03)
// #define COPY_SECURITY_AUTH_STATE_UUID(uuid_struct)      COPY_UUID_128(uuid_struct,0xdf,0xe3,0xec,0x38,0xca,0x61,0x40,0xc8,0x87,0x43,0x5e,0x9f,0xd2,0x85,0x21,0x54)
#define COPY_SECURITY_CP_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0xf5,0x1c,0x85,0xa4,0xb2,0x3d,0x11,0xeb,0x85,0x29,0x02,0x42,0xac,0x13,0x00,0x03)
#define CP_RESP_SIZE 3
#define RESP_POSITIVE 0xAA
#define RESP_NEGATIVE 0xFF

/* Scame Socket Current Transaction Active */
#define COPY_TRANSACTION_SERVICE_UUID(uuid_struct)      COPY_UUID_128(uuid_struct,0xf5,0x1c,0x7d,0xb6,0xb2,0x3d,0x11,0xeb,0x85,0x29,0x02,0x42,0xac,0x13,0x00,0x03)
#define COPY_CURRENT_TRANS_CHAR_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0xf5,0x1c,0x80,0x86,0xb2,0x3d,0x11,0xeb,0x85,0x29,0x02,0x42,0xac,0x13,0x00,0x03)
#define COPY_MEASUREMENT_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0xf5,0x1c,0x81,0x94,0xb2,0x3d,0x11,0xeb,0x85,0x29,0x02,0x42,0xac,0x13,0x00,0x03)
#define COPY_REGISTER_CHAR_UUID(uuid_struct)            COPY_UUID_128(uuid_struct,0xf5,0x1c,0x89,0xbe,0xb2,0x3d,0x11,0xeb,0x85,0x29,0x02,0x42,0xac,0x13,0x00,0x03)
#define COPY_TRANS_CP_CHAR_UUID(uuid_struct)            COPY_UUID_128(uuid_struct,0xf5,0x1c,0x83,0xd8,0xb2,0x3d,0x11,0xeb,0x85,0x29,0x02,0x42,0xac,0x13,0x00,0x03)

/* Scame Socket Schedulation */
#define COPY_SCHEDULATION_SERVICE_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0xde,0xb2,0x93,0x7d,0x38,0x85,0x47,0x37,0x83,0x59,0x92,0x93,0xe6,0x53,0xdd,0xec)
#define COPY_SCHEDS_CHAR_UUID(uuid_struct)              COPY_UUID_128(uuid_struct,0x23,0x94,0x8b,0xe7,0x43,0xaf,0x46,0xd4,0xb2,0x11,0x4c,0x64,0xfc,0x62,0x23,0xc8)

uint16_t SocketInfoServHandle, SocketStateServHandle;

uint16_t SerialCharHandle, FirmwareCharHandle, EnergyMeterCharHandle;
uint16_t RfidCharHandle, MaxCurrentCharHandle, WiringCharHandle, MaxPowerCharHandle;
uint16_t PwrModeCharHandle;

uint16_t IdSocketCharHandle, StateDescCharHandle, FunModeCharHandle;

//uint16_t CmdDataCharHandle;
//uint16_t SocketRegisterServHandle;

/* Scame Socket Security Handles */
uint16_t SocketSecurityServHandle;
uint16_t /*AuthStateCharHandle,*/ SecurityCPCharHandle;

const uint8_t RESP_INIT_LOGIN_ACCEPTED[CP_RESP_SIZE] = 
{
  (uint8_t)RESPONSE,            // op_code
  (uint8_t)INIT_AUTH,           // op_code_resp
  (uint8_t)RESP_POSITIVE        // response_value
};
const uint8_t RESP_INIT_LOGIN_REFUSED[CP_RESP_SIZE] = 
{
  (uint8_t)RESPONSE,            // op_code
  (uint8_t)INIT_AUTH,           // op_code_resp
  (uint8_t)RESP_NEGATIVE        // response_value
};
const uint8_t RESP_SET_AUTH_ACCEPTED[CP_RESP_SIZE] = 
{
  (uint8_t)RESPONSE,            // op_code
  (uint8_t)SET_AUTH,           // op_code_resp
  (uint8_t)RESP_POSITIVE        // response_value
};
const uint8_t RESP_SET_AUTH_REFUSED[CP_RESP_SIZE] = 
{
  (uint8_t)RESPONSE,            // op_code
  (uint8_t)SET_AUTH,           // op_code_resp
  (uint8_t)RESP_NEGATIVE        // response_value
};
const uint8_t RESP_LOGIN_ACCEPTED[CP_RESP_SIZE] = 
{
  (uint8_t)RESPONSE,            // op_code
  (uint8_t)REQ_AUTH,            // op_code_resp
  (uint8_t)RESP_POSITIVE        // response_value
};
const uint8_t RESP_LOGIN_REFUSED[CP_RESP_SIZE] = 
{
  (uint8_t)RESPONSE,            // op_code
  (uint8_t)REQ_AUTH,            // op_code_resp
  (uint8_t)RESP_NEGATIVE        // response_value
};
const uint8_t RESP_CHANGE_AUTH_ACCEPTED[CP_RESP_SIZE] = 
{
  (uint8_t)RESPONSE,            // op_code
  (uint8_t)REQ_CHANGE_AUTH,     // op_code_resp
  (uint8_t)RESP_POSITIVE        // response_value
};
const uint8_t RESP_CHANGE_AUTH_REFUSED[CP_RESP_SIZE] = 
{
  (uint8_t)RESPONSE,            // op_code
  (uint8_t)REQ_CHANGE_AUTH,     // op_code_resp
  (uint8_t)RESP_NEGATIVE        // response_value
};

/* Scame Socket Current Transaction Active Handles */
uint16_t SocketTransServHandle;
uint16_t CurrentTransCharHandle, MeasurementCharHandle, RegisterCharHandle, TransCPCharHandle;

/* Scame Socket Schedulation Handles */
uint16_t SocketSchedServHandle;
uint16_t SchedulationCharHandle;

extern uint16_t gacc_service_handle, dev_name_char_handle;


//uint16_t HWServW2STHandle, EnvironmentalCharHandle, AccGyroMagCharHandle;
//uint16_t SWServW2STHandle, QuaternionsCharHandle;

uint16_t PersonalServHandle;
uint16_t PushbuttonCharHandle;
uint16_t LedCharHandle;

/* UUIDS */
Service_UUID_t service_uuid;
Char_UUID_t char_uuid;


/* GATT long update supports */
uint8_t sending = 0;
BtPacket_t pckt_queue[MAXPCKT];
int nPckts2send = 0;
int sendFromIdx = 0;

/* GATT long received supports */
uint8_t rec_buff[MAXBUFFBYTES];
int buff_index = 0;


/*-------------------------------*/
extern AxesRaw_t x_axes;
extern AxesRaw_t g_axes;
extern AxesRaw_t m_axes;

extern uint16_t connection_handle;
extern uint32_t start_time;

extern uint32_t counter;
extern uint32_t led_state;

extern uint8_t* device_name;
extern uint8_t device_name_length;

tBleStatus gatt_update(uint16_t serv_handle, uint16_t char_handle, uint8_t value_len, void *value);
tBleStatus gatt_long_update(uint16_t serv_handle, uint16_t char_handle, uint32_t value_len, void *value);
int create_pckt_queue(uint32_t value_len, void *value);

void packet_received(uint16_t handle, uint8_t *value, uint8_t value_length);
void gatt_long_received(uint16_t handle, uint8_t *value);

/**
 * @brief  Add the 'HW' service (and the Environmental and AccGyr characteristics).
 * @param  None
 * @retval tBleStatus Status
 */
//tBleStatus Add_HWServW2ST_Service(void)
//{
//  tBleStatus ret;
//  uint8_t uuid[16];
//
//  /* Add_HWServW2ST_Service */
//  COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);
//  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
//  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid.Service_UUID_128, PRIMARY_SERVICE,
//                          1+3*5, &HWServW2STHandle);
//  if (ret != BLE_STATUS_SUCCESS)
//    return BLE_STATUS_ERROR;
//
//  /* Fill the Environmental BLE Characteristc */
//  COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid);
//  uuid[14] |= 0x04; /* One Temperature value*/
//  uuid[14] |= 0x10; /* Pressure value*/
//  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
//  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
//                           2+2+4,
//                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
//                           ATTR_PERMISSION_NONE,
//                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
//                           16, 0, &EnvironmentalCharHandle);
//  if (ret != BLE_STATUS_SUCCESS)
//    return BLE_STATUS_ERROR;
//
//  /* Fill the AccGyroMag BLE Characteristc */
//  COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid);
//  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
//  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
//                           2+3*3*2,
//                           CHAR_PROP_NOTIFY,
//                           ATTR_PERMISSION_NONE,
//                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
//                           16, 0, &AccGyroMagCharHandle);
//  if (ret != BLE_STATUS_SUCCESS)
//    return BLE_STATUS_ERROR;
//
//  return BLE_STATUS_SUCCESS;
//}

/**
 * @brief  Add the SW Feature service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
//tBleStatus Add_SWServW2ST_Service(void)
//{
//  tBleStatus ret;
//  int32_t NumberOfRecords=1;
//  uint8_t uuid[16];
//
//  COPY_SW_SENS_W2ST_SERVICE_UUID(uuid);
//  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
//  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid.Service_UUID_128, PRIMARY_SERVICE,
//                          1+3*NumberOfRecords, &SWServW2STHandle);
//
//  if (ret != BLE_STATUS_SUCCESS) {
//    goto fail;
//  }
//
//  COPY_QUATERNIONS_W2ST_CHAR_UUID(uuid);
//  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
//  ret =  aci_gatt_add_char(SWServW2STHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
//                           2+6*SEND_N_QUATERNIONS,
//                           CHAR_PROP_NOTIFY,
//                           ATTR_PERMISSION_NONE,
//                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
//                           16, 0, &QuaternionsCharHandle);
//
//  if (ret != BLE_STATUS_SUCCESS) {
//    goto fail;
//  }
//
//  return BLE_STATUS_SUCCESS;
//
//fail:
//  return BLE_STATUS_ERROR;
//}

/**
* @brief  Add Scame Services using a vendor specific profile
* @param  None
* @retval tBleStatus Status
*/
tBleStatus Add_Scame_Services()
{
  tBleStatus ret;
  uint8_t uuid[16];
  
  /* =============================== Add Info Socket Service =============================== */
  COPY_INFO_SOCKET_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid.Service_UUID_128, PRIMARY_SERVICE,
                          1+2*8, &SocketInfoServHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* Add Serial number Characteristic */
  COPY_SERIAL_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SocketInfoServHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           16,
                           CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_DONT_NOTIFY_EVENTS,
                           16, 0, &SerialCharHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* Add Firmware version Characteristic */
  COPY_FIRMWARE_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SocketInfoServHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           24,
                           CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_DONT_NOTIFY_EVENTS,
                           16, 0, &FirmwareCharHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* Add Energy Meter Characteristic */
  COPY_ENERGYMETER_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SocketInfoServHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           2,
                           CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_DONT_NOTIFY_EVENTS,
                           16, 0, &EnergyMeterCharHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* Add RFID reader Characteristic */
  COPY_RFID_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SocketInfoServHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           1,
                           CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_DONT_NOTIFY_EVENTS,
                           16, 0, &RfidCharHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* Add Max Current Characteristic */
  COPY_MAXCURRENT_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SocketInfoServHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           4,
                           CHAR_PROP_READ|CHAR_PROP_WRITE,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &MaxCurrentCharHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* Add Wiring Characteristic */
  COPY_WIRING_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SocketInfoServHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           1,
                           CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_DONT_NOTIFY_EVENTS,
                           16, 0, &WiringCharHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* Add Max Power Characteristic */
  COPY_MAXPOWER_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SocketInfoServHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           4,
                           CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_DONT_NOTIFY_EVENTS,
                           16, 0, &MaxPowerCharHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* Add Power Mode Characteristic */
  COPY_PWRMODE_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SocketInfoServHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           1,
                           CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_DONT_NOTIFY_EVENTS,
                           16, 0, &PwrModeCharHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* =============================== Add State Socket Service =============================== */
  COPY_STATE_SOCKET_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid.Service_UUID_128, PRIMARY_SERVICE,
                          1+2*2+3, &SocketStateServHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* Add Id Socket Characteristic */
  COPY_IDSOCKET_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SocketStateServHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           1,
                           CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_DONT_NOTIFY_EVENTS,
                           16, 0, &IdSocketCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* Add State Socket Description Characteristic */
  COPY_STATEDESC_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SocketStateServHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           2,
                           CHAR_PROP_READ|CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_DONT_NOTIFY_EVENTS,
                           16, 0, &StateDescCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* Add Mode Characteristic */
  COPY_MODE_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SocketStateServHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           1,
                           CHAR_PROP_READ|CHAR_PROP_WRITE,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &FunModeCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* =============================== Add Register Socket Service =============================== */
//  COPY_REGISTER_SOCKET_SERVICE_UUID(uuid);
//  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
//  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid.Service_UUID_128, PRIMARY_SERVICE,
//                          1+3, &SocketRegisterServHandle);
//  
//  if (ret != BLE_STATUS_SUCCESS) {
//    goto fail;
//  }
//  
//  /* Add transactions data characteristics */
//  COPY_REGISTER_CHAR_UUID(uuid);
//  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
//  ret =  aci_gatt_add_char(SocketRegisterServHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
//                           20,
//                           CHAR_PROP_READ|CHAR_PROP_INDICATE,
//                           ATTR_PERMISSION_NONE,
//                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
//                           16, 1, &TransactionsCharHandle);
//  
//  if (ret != BLE_STATUS_SUCCESS) {
//    goto fail;
//  }
  
    /* ==================== Add Security Socket Service ===================== */
  COPY_SECURITY_SOCKET_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid.Service_UUID_128, PRIMARY_SERVICE,
                          1+3, &SocketSecurityServHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* Add Authorization State Characteristics */
//  COPY_SECURITY_AUTH_STATE_UUID(uuid);
//  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
//  ret =  aci_gatt_add_char(SocketSecurityServHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
//                           1 + HEADER_SIZE,
//                           CHAR_PROP_READ,
//                           ATTR_PERMISSION_NONE,
//                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
//                           16, 0, &AuthStateCharHandle);
//  
//  if (ret != BLE_STATUS_SUCCESS) {
//    goto fail;
//  }
  
  /* Add Security Control Point Characteristics */
  COPY_SECURITY_CP_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SocketSecurityServHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           52,
                           CHAR_PROP_WRITE|CHAR_PROP_INDICATE,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE,
                           16, 1, &SecurityCPCharHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
    /* =============== Add Transaction Socket Service =============== */
  COPY_TRANSACTION_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid.Service_UUID_128, PRIMARY_SERVICE,
                          1+2+3+3+3, &SocketTransServHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* Add Current Transaction Information Characteristic */
  COPY_CURRENT_TRANS_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SocketTransServHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           26,
                           CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_DONT_NOTIFY_EVENTS,
                           16, 1, &CurrentTransCharHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* Add Measurement Characteristic */
  COPY_MEASUREMENT_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SocketTransServHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           MEASURE_SIZE,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_DONT_NOTIFY_EVENTS,
                           16, 1, &MeasurementCharHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* Add Transactions Register Characteristics */
  COPY_REGISTER_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SocketTransServHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           2 * TRANS_SIZE,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_DONT_NOTIFY_EVENTS,
                           16, 1, &RegisterCharHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* Add Transaction Control Point Characteristic */
  COPY_TRANS_CP_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SocketTransServHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           3 + TRANS_SIZE,
                           CHAR_PROP_WRITE|CHAR_PROP_INDICATE,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE,
                           16, 1, &TransCPCharHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
    /* =============== Add Schedulation Socket Service =============== */
  COPY_SCHEDULATION_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid.Service_UUID_128, PRIMARY_SERVICE,
                          1+2, &SocketSchedServHandle);
  
  /* Add Schedulation Control Point Characteristic */
  COPY_SCHEDS_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(SocketSchedServHandle, UUID_TYPE_128, char_uuid.Char_UUID_128,
                           HEADER_SIZE + SCHED_BT_SIZE * MAX_SCHEDS,
                           CHAR_PROP_WRITE|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP|GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &SchedulationCharHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  return BLE_STATUS_SUCCESS;

fail:
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Update acceleration characteristic value
 * @param  AxesRaw_t structure containing acceleration value in mg.
 * @retval tBleStatus Status
 */
//tBleStatus Acc_Update(AxesRaw_t *x_axes, AxesRaw_t *g_axes, AxesRaw_t *m_axes)
//{
//  uint8_t buff[2+2*3*3];
//  tBleStatus ret;
//
//  HOST_TO_LE_16(buff,(HAL_GetTick()>>3));
//
//  HOST_TO_LE_16(buff+2,-x_axes->AXIS_Y);
//  HOST_TO_LE_16(buff+4, x_axes->AXIS_X);
//  HOST_TO_LE_16(buff+6,-x_axes->AXIS_Z);
//
//  HOST_TO_LE_16(buff+8,g_axes->AXIS_Y);
//  HOST_TO_LE_16(buff+10,g_axes->AXIS_X);
//  HOST_TO_LE_16(buff+12,g_axes->AXIS_Z);
//
//  HOST_TO_LE_16(buff+14,m_axes->AXIS_Y);
//  HOST_TO_LE_16(buff+16,m_axes->AXIS_X);
//  HOST_TO_LE_16(buff+18,m_axes->AXIS_Z);
//
//  ret = aci_gatt_update_char_value(HWServW2STHandle, AccGyroMagCharHandle,
//				   0, 2+2*3*3, buff);
//  if (ret != BLE_STATUS_SUCCESS){
//    PRINTF("Error while updating Acceleration characteristic: 0x%02X\n",ret) ;
//    return BLE_STATUS_ERROR ;
//  }
//
//  return BLE_STATUS_SUCCESS;
//}

/**
 * @brief  Update quaternions characteristic value
 * @param  SensorAxes_t *data Structure containing the quaterions
 * @retval tBleStatus      Status
 */
//tBleStatus Quat_Update(AxesRaw_t *data)
//{
//  tBleStatus ret;
//  uint8_t buff[2+6*SEND_N_QUATERNIONS];
//
//  HOST_TO_LE_16(buff,(HAL_GetTick()>>3));
//
//#if SEND_N_QUATERNIONS == 1
//  HOST_TO_LE_16(buff+2,data[0].AXIS_X);
//  HOST_TO_LE_16(buff+4,data[0].AXIS_Y);
//  HOST_TO_LE_16(buff+6,data[0].AXIS_Z);
//#elif SEND_N_QUATERNIONS == 2
//  HOST_TO_LE_16(buff+2,data[0].AXIS_X);
//  HOST_TO_LE_16(buff+4,data[0].AXIS_Y);
//  HOST_TO_LE_16(buff+6,data[0].AXIS_Z);
//
//  HOST_TO_LE_16(buff+8 ,data[1].AXIS_X);
//  HOST_TO_LE_16(buff+10,data[1].AXIS_Y);
//  HOST_TO_LE_16(buff+12,data[1].AXIS_Z);
//#elif SEND_N_QUATERNIONS == 3
//  HOST_TO_LE_16(buff+2,data[0].AXIS_X);
//  HOST_TO_LE_16(buff+4,data[0].AXIS_Y);
//  HOST_TO_LE_16(buff+6,data[0].AXIS_Z);
//
//  HOST_TO_LE_16(buff+8 ,data[1].AXIS_X);
//  HOST_TO_LE_16(buff+10,data[1].AXIS_Y);
//  HOST_TO_LE_16(buff+12,data[1].AXIS_Z);
//
//  HOST_TO_LE_16(buff+14,data[2].AXIS_X);
//  HOST_TO_LE_16(buff+16,data[2].AXIS_Y);
//  HOST_TO_LE_16(buff+18,data[2].AXIS_Z);
//#else
//#error SEND_N_QUATERNIONS could be only 1,2,3
//#endif
//
//  ret = aci_gatt_update_char_value(SWServW2STHandle, QuaternionsCharHandle,
//				   0, 2+6*SEND_N_QUATERNIONS, buff);
//  if (ret != BLE_STATUS_SUCCESS){
//    PRINTF("Error while updating Sensor Fusion characteristic: 0x%02X\n",ret) ;
//    return BLE_STATUS_ERROR ;
//  }
//
//  return BLE_STATUS_SUCCESS;
//}

/*******************************************************************************
* Function Name  : Read_Request_CB.
* Description    : Update the sensor valuse.
* Input          : Handle of the characteristic to update.
* Return         : None.
*******************************************************************************/
void Read_Request_CB(uint16_t handle, uint16_t offset)
{
  tBleStatus ret;
  
  if(handle == SerialCharHandle + 1)
  {
    //char_update_serial(socket->info.serial);
    //gatt_update(SocketInfoServHandle, SerialCharHandle, 16, get_sck_serial());
    aci_gatt_update_char_value(SocketInfoServHandle, SerialCharHandle,
                                   0, 16, get_sck_serial());
  }
  else if(handle == FirmwareCharHandle + 1)
  {
    //char_update_firmware(socket->info.firmware);
    //gatt_update(SocketInfoServHandle, FirmwareCharHandle, 24, get_sck_firmware());
    aci_gatt_update_char_value(SocketInfoServHandle, FirmwareCharHandle,
                                   offset, 24, get_sck_firmware());
  }
  else if(handle == EnergyMeterCharHandle + 1)
  {
    //char_update_energymeter(socket->info.energyMeter);
    uint16_t val = get_sck_emeters();
    //gatt_update(SocketInfoServHandle, EnergyMeterCharHandle, 2, &val_16);
    aci_gatt_update_char_value(SocketInfoServHandle, EnergyMeterCharHandle,
                                   0, 2, &val);
  }
  else if(handle == RfidCharHandle + 1)
  {
    //char_update_rfid(socket->info.rfidReader);
    uint8_t val = get_sck_rfid();
//    gatt_update(SocketInfoServHandle, RfidCharHandle, 1, &val_8);
    aci_gatt_update_char_value(SocketInfoServHandle, RfidCharHandle,
                                   0, 1, &val);
  }
  else if(handle == MaxCurrentCharHandle + 1)
  {
    //char_update_maxcurrent(socket->info.maxCurrent);
    int32_t value = get_sck_max_current();
//    gatt_update(SocketInfoServHandle, MaxCurrentCharHandle, 4, &value);
    aci_gatt_update_char_value(SocketInfoServHandle, MaxCurrentCharHandle,
                                   0, 4, &value);
  }
  else if(handle == WiringCharHandle + 1)
  {
    //char_update_wiring(socket->info.wiring);
    uint8_t val = get_sck_wiring();
//    gatt_update(SocketInfoServHandle, WiringCharHandle, 1, &val_8);
    aci_gatt_update_char_value(SocketInfoServHandle, WiringCharHandle,
                                   0, 1, &val);
  }
  else if(handle == MaxPowerCharHandle + 1)
  {
    //char_update_maxpower(socket->info.maxPower);
    int32_t value = get_sck_max_power();
    //gatt_update(SocketInfoServHandle, MaxPowerCharHandle, 4, &value);
    aci_gatt_update_char_value(SocketInfoServHandle, MaxPowerCharHandle,
                                   offset, 4, &value);
  }
  else if(handle == PwrModeCharHandle + 1)
  {
    uint8_t val = get_sck_pwr_mode();
    //gatt_update(SocketInfoServHandle, PwrModeCharHandle, 1, &val_8);
    aci_gatt_update_char_value(SocketInfoServHandle, PwrModeCharHandle,
                                   0, 1, &val);
  }
  else if(handle == IdSocketCharHandle + 1)
  {
    //char_update_id(socket->state.id);
    uint8_t val = get_sck_id();
    //gatt_update(SocketStateServHandle, IdSocketCharHandle, 1, &val_8);
    aci_gatt_update_char_value(SocketStateServHandle, IdSocketCharHandle,
                                   0, 1, &val);
  }
  else if(handle == StateDescCharHandle + 1)
  {
    //char_update_state_desc(socket->state.desc);
    sck_state_t state = get_sck_state();
    uint16_t val = ((uint8_t)state.value << 8)|((uint8_t)state.error);
    //gatt_update(SocketStateServHandle, StateDescCharHandle, 2, &val_16);
    aci_gatt_update_char_value(SocketStateServHandle, StateDescCharHandle,
                                   0, 2, &val);
  }
  else if(handle == FunModeCharHandle + 1)
  {
    //char_update_mode(socket->state.mode);
    uint8_t val = get_sck_fun_mode();
    //gatt_update(SocketStateServHandle, FunModeCharHandle, 1, &val_8);
    aci_gatt_update_char_value(SocketStateServHandle, FunModeCharHandle,
                                   0, 1, &val);
  }
//  else if(handle == RegisterCharHandle + 1)
//  {
//    char_update_register();
////    sck_register_t *reg = get_sck_register();
////    aci_gatt_update_char_value(SocketTransServHandle, RegisterCharHandle,
////                                   offset, reg->num * TRANS_SIZE, reg);
//  }
  else if(handle == CurrentTransCharHandle + 1)
  {
    sck_register_t *reg = get_sck_register();
    char_update_current_trans(&reg->transactions[reg->num]);
  }
  else if(handle == SchedulationCharHandle + 1)
  {
    sck_schedule_t sched[4];
    sck_schedule_t *ptr = get_schedulation();
    for(int i = 0; i < MAX_SCHEDS; i++)
    {
      memcpy(&sched[i], &ptr[i], SCHED_BT_SIZE);
      uint32_t val = sched[i].power;
      sched[i].power = 0;
      sched[i].power = ((val >> 24) & 0xFF) | ((val >> 8) & 0xFF00) | ((val << 24) & 0xFF000000) | ((val << 8) & 0xFF0000);
    }
    aci_gatt_update_char_value(SocketSchedServHandle, SchedulationCharHandle,
                                   offset, SCHED_BT_SIZE * MAX_SCHEDS, &sched);
  }

  if(connection_handle !=0)
  {
    ret = aci_gatt_allow_read(connection_handle);
    if (ret != BLE_STATUS_SUCCESS)
    {
      PRINTF("aci_gatt_allow_read() failed: 0x%02x\r\n", ret);
    }
  }
}

/*******************************************************************************
* Function Name  : Write_Request_CB.
* Description    : Update the sensor led state.
* Input          : Handle of the characteristic to update.
* Return         : None.
*******************************************************************************/
void Write_Request_CB(uint16_t handle, uint8_t *value, uint8_t value_length)
{
  tBleStatus ret;
  uint8_t write_status = 0x01, error_code;
  uint8_t restart = 0;
  
  // Only for debug
  if(handle == dev_name_char_handle + 1)
  {
    //PRINTF("Type of value correct -> allow write Name Character\n");
  }
  else if(handle == FunModeCharHandle + 1)
  {
    set_sck_mode((modeFun_e)value[0]);
    eeprom_param_set(EVS_MODE_EADD, &value[0], 1);
    restart = 1;
  }
  else if(handle == MaxCurrentCharHandle + 1)
  {
    if(value_length == 4)
    {
      int32_t currentLx = 0;
      memcpy(&currentLx, value, 4);
      set_sck_max_current(currentLx);
      currentLx = currentLx / 1000;
      eeprom_param_set(M3T_CURRENT_EADD, (uint8_t *)&currentLx, 1);
      restart = 1;
    }
  }
  else if(handle == SchedulationCharHandle + 1)
  {
    sched_received(value, value_length);
  }
  
  // Only for debug. SCU accept all write requests
  write_status = 0x00;
  error_code = ERR_CMD_SUCCESS;
  
  if(connection_handle !=0)
  { 
    ret = aci_gatt_write_response(connection_handle, handle, write_status, error_code, value_length, value);
    if (ret != BLE_STATUS_SUCCESS)
    {
      PRINTF("aci_gatt_update_char_value() failed: ret = 0x%02x, err = 0x%02x\r\n", ret, error_code);
    }
  }
  
  if(restart != 0)
  {
    osDelay(pdMS_TO_TICKS(2000));
    setFlagForNvic();
    NVIC_SystemReset();
  }
}

/*******************************************************************************
* Function Name  : Attribute_Modified_CB.
* Description    : Update the sensor parameters.
* Input          : Handle of the characteristic to update.
* Return         : None.
*******************************************************************************/
void Attribute_Modified_CB(uint16_t handle, uint8_t *value, uint8_t value_length)
{
  op_code_e opcode;
  char user[USER_LENGTH + 1] = {0};
  char pass[PASS_LENGTH + 1] = {0};
  uint8_t user_len, pass_len;
  
  char newuser[USER_LENGTH + 1] = {0};
  char newpass[PASS_LENGTH + 1] = {0};
  uint8_t newuser_len, newpass_len;
  
  sck_auth_t *auth;
  uint8_t resp[3];
  
  BtTaskReq_t req;
  
  if(handle == dev_name_char_handle + 1)
  {
    //char *received_name =(char *)value + 2;
    set_sck_name((char *)value, value_length);
  }
  else if(handle == TransCPCharHandle + 1)
  {
    opcode = (op_code_e)value[0];
    resp[0] = RESPONSE;
    resp[1] = opcode;
    switch(opcode)
    {
      case REQ_REGISTER:
        // received register request
        //send_register();
        req.type = SEND_REGISTER;
        osMessageQueuePut(getBlueMngQueueHandle(), &req, 0, 0);
        resp[2] = RESP_POSITIVE;
        break;
      case START_CHARGE:
        break;
      case STOP_CHARGE:
        resp[2] = stop_charge();
        break;
      case SUSPEND_CHARGE:
        send_to_evs(EVS_APP_SUSPENDING);
        resp[2] = RESP_POSITIVE;
        break;
      case RELEASE_SUSPEND_CHARGE:
        send_to_evs(EVS_APP_RELEASE);
        resp[2] = RESP_POSITIVE;
        break;
      case ACTIVE_SCHEDULE:
        break;
      case DISABLE_SCHEDULE:
        resp[2] = RESP_POSITIVE;
        break;
      case UNKNOWN:
      default:
        resp[2] = RESP_NEGATIVE;
        break;
    }
    aci_gatt_update_char_value(SocketTransServHandle, TransCPCharHandle,
                                   0, 3, &resp);
  }
  else if(handle == SecurityCPCharHandle + 1)
  { 
    opcode = (op_code_e)value[0];
    resp[0] = RESPONSE;
    resp[1] = opcode;
    uint16_t app_id = 0;
    switch(opcode)
    {
      case GET_AUTH:
        auth = get_sck_auth();
        resp[2] = auth->auth_state;
        break;
      case SET_AUTH:
        user_len = value[1];
        memcpy(user, value + 2, user_len);
        pass_len = value[2 + user_len];
        memcpy(pass, value + 2 + 1 + user_len, pass_len);
        set_sck_auth(user, pass);
        resp[2] = AUTH_ACCEPTED;
        break;
      case REQ_CHANGE_AUTH:
        user_len = value[1];
        memcpy(user, value + 2, user_len);
        pass_len = value[2 + user_len];
        memcpy(pass, value + 2 + 1 + user_len, pass_len);
        newuser_len = value[2 + user_len + 1 + pass_len];
        memcpy(newuser, value + 2 + 1 + user_len + pass_len + 1, newuser_len);
        newpass_len = value[2 + user_len + 1 + pass_len + newuser_len + 1];
        memcpy(newpass, value + 2 + 1 + user_len + pass_len + 1 + newuser_len + 1, newpass_len);
        if(check_sck_auth(user, pass))
        {
          if (strcmp(newuser, "") == 0)
          {
            set_sck_auth(user, newpass);
          } 
          else if (strcmp(newpass, "") == 0)
          {
            set_sck_auth(newuser, pass);
          }
          else 
          {
            set_sck_auth(newuser, newpass);
          }
          resp[2] = AUTH_ACCEPTED;
        }
        else
        {
          resp[2] = AUTH_REFUSED;
        }
        break;
      case REQ_AUTH:
        user_len = value[1];
        memcpy(user, value + 2, user_len);
        pass_len = value[2 + user_len];
        memcpy(pass, value + 2 + 1 + user_len, pass_len);
        if(check_sck_auth(user, pass))
        {
          resp[2] = AUTH_ACCEPTED;
        }
        else
        {
          resp[2] = AUTH_REFUSED;
        }
        break;
      case SEND_APP_ID:
        memcpy(&app_id, value + 1, 2);
        set_app_id(app_id);
        resp[2] = AUTH_ACCEPTED;
        break;
      case UNKNOWN:
      default:
        break;
    }
    gatt_update(SocketSecurityServHandle, SecurityCPCharHandle, 3, resp);
  }
//  else if(handle == RegisterCharHandle + 1)
//  {
//    uint16_t test = handle;
//  }
}

/*******************************************************************************
*       Custom functions
*******************************************************************************/

/**
 * @brief  Turn on all descriptors with notify property
 * @param  none
 * @retval tBleStatus Status
 */
tBleStatus turn_on_notify()
{
  tBleStatus result;
  
  uint8_t notify_on[2] = {NOTIFICATION, 0};
  uint8_t indicate_on[2] = {INDICATION, 0};
  
  /* Turn ON Notify State */
  result = aci_gatt_set_desc_value(SocketStateServHandle, 
                                   StateDescCharHandle, 
                                   StateDescCharHandle + 2,
                                   0,
                                   2,
                                   notify_on);
  
  /* Turn ON Notify Active Transaction Measurement Char */
  result = aci_gatt_set_desc_value(SocketTransServHandle, 
                                   MeasurementCharHandle, 
                                   MeasurementCharHandle + 2,
                                   0,
                                   2,
                                   notify_on);
  
  /* Turn ON Indicate Security Control Point Char */
  result = aci_gatt_set_desc_value(SocketSecurityServHandle, 
                                   SecurityCPCharHandle, 
                                   SecurityCPCharHandle + 2,
                                   0,
                                   2,
                                   indicate_on);
  
  /* Turn ON Notify Register Char */
  result = aci_gatt_set_desc_value(SocketTransServHandle, 
                                   RegisterCharHandle, 
                                   RegisterCharHandle + 2,
                                   0,
                                   2,
                                   notify_on);
  
   /* Turn ON Notify Transaction Control Point Char */
  result = aci_gatt_set_desc_value(SocketTransServHandle, 
                                   TransCPCharHandle, 
                                   TransCPCharHandle + 2,
                                   0,
                                   2,
                                   indicate_on);
  
  return result;
}

/**
 * @brief  Update serial number characteristic value
 * @param  none
 * @retval tBleStatus Status
 */
tBleStatus char_update_serial()
{
  return aci_gatt_update_char_value(SocketInfoServHandle, SerialCharHandle,
                                   0, 16, get_sck_serial());
}

/**
 * @brief  Update firmware version characteristic value
 * @param  none
 * @retval tBleStatus Status
 */
tBleStatus char_update_firmware()
{
  return aci_gatt_update_char_value(SocketInfoServHandle, FirmwareCharHandle,
                                   0, 24, get_sck_firmware());
}

/**
 * @brief  Update internal energy meter characteristic value
 * @param  none
 * @retval tBleStatus Status
 */
tBleStatus char_update_emeters()
{
  uint16_t val = get_sck_emeters();
  return aci_gatt_update_char_value(SocketInfoServHandle, EnergyMeterCharHandle,
                                 0, 2, &val);
}

/**
 * @brief  Update rfid reader characteristic value
 * @param  none
 * @retval tBleStatus Status
 */
tBleStatus char_update_rfid()
{
  uint8_t val = get_sck_rfid();
  return aci_gatt_update_char_value(SocketInfoServHandle, RfidCharHandle,
                             0, 1, &val);
}

/**
 * @brief  Update maximum current characteristic value
 * @param  none
 * @retval tBleStatus Status
 */
tBleStatus char_update_max_current()
{
  int32_t value = get_sck_max_current();
  return aci_gatt_update_char_value(SocketInfoServHandle, MaxCurrentCharHandle,
                                 0, 4, &value);
}

/**
 * @brief  Update wiring type characteristic value
 * @param  none
 * @retval tBleStatus Status
 */
tBleStatus char_update_wiring()
{
  uint8_t val = get_sck_wiring();
  return aci_gatt_update_char_value(SocketInfoServHandle, WiringCharHandle,
                                 0, 1, &val);
}

/**
 * @brief  Update maximum power characteristic value
 * @param  none
 * @retval tBleStatus Status
 */
tBleStatus char_update_max_power()
{
  int32_t value = get_sck_max_power();
  return aci_gatt_update_char_value(SocketInfoServHandle, MaxPowerCharHandle,
                                   0, 4, &value);
}

/**
 * @brief  Update power mode characteristic value
 * @param  none
 * @retval tBleStatus Status
 */
tBleStatus char_update_pwr_mode()
{
  uint8_t val = get_sck_pwr_mode();
  return aci_gatt_update_char_value(SocketInfoServHandle, PwrModeCharHandle,
                                   0, 1, &val);
}

/**
 * @brief  Update socket id characteristic value
 * @param  none
 * @retval tBleStatus Status
 */
tBleStatus char_update_id()
{
  uint8_t val = get_sck_id();
  return aci_gatt_update_char_value(SocketStateServHandle, IdSocketCharHandle,
                                   0, 1, &val);
}

/**
 * @brief  Update state characteristic value
 * @param  none
 * @retval tBleStatus Status
 */
tBleStatus char_update_state()
{
  sck_state_t state = get_sck_state();
  uint16_t val = ((uint8_t)state.value << 8)|((uint8_t)state.error);
  return aci_gatt_update_char_value(SocketStateServHandle, StateDescCharHandle,
                                 0, 2, &val);
}

/**
 * @brief  Update function mode characteristic value
 * @param  none
 * @retval tBleStatus Status
 */
tBleStatus char_update_fun_mode()
{
  uint8_t val = get_sck_fun_mode();
  return aci_gatt_update_char_value(SocketStateServHandle, FunModeCharHandle,
                                   0, 1, &val);
}

/**
 * @brief  Update Active Transaction Measurement Char
 * @param  sck_measures_t pointer
 * @retval tBleStatus Status
 */
tBleStatus char_update_measures(sck_measures_t *m)
{
  uint8_t size = 0;
  uint8_t value[41];
  
  memcpy(value, &m->modePwr, 1);
  memcpy(value + 1, &m->Etot, 4);
  memcpy(value + 5, &m->duration, 4);
  memcpy(value + 9, &m->Pist, 4);
  memcpy(value + 13, &m->currentL1, 4);
  
  switch(m->modePwr)
  {
  case MODE_MONO_PH_NO_PM:
    size = 17; /** measure: Etot, Time, Pist, L1 */
    break;
  case MODE_TRI_PH_NO_PM:
    size = 25; /** measure: Etot, Time, Pist, L1, L2, L3 */
    memcpy(value + 17, &m->currentL2, 4);
    memcpy(value + 21, &m->currentL3, 4);
    break;
  case MODE_MONO_PH_PM:
    size = 21; /** measure: Etot, Time, Pist, Pest, L1 */
    memcpy(value + 13, &m->Pest, 4);
    break;
  case MODE_TRI_PH_PM_BAL:
    size = 41; /** measure: Etot, Time, Pist, Pest, L1, L2, L3, Pest1, Pest2, Pest3 */
    memcpy(value + 13, &m->Pest, 4);
    memcpy(value + 21, &m->currentL2, 4);
    memcpy(value + 25, &m->currentL3, 4);
    memcpy(value + 29, &m->Pest1, 4);
    memcpy(value + 33, &m->Pest2, 4);
    memcpy(value + 37, &m->Pest3, 4);
    break;
  case MODE_TRI_PH_PM_UMBAL:
    size = 28; /** measure: Etot, Time, Pist, Pest, L1, L2, L3 */
    memcpy(value + 13, &m->Pest, 4);
    memcpy(value + 21, &m->currentL2, 4);
    memcpy(value + 25, &m->currentL3, 4);
    break;
  }
  
  return aci_gatt_update_char_value(SocketTransServHandle, MeasurementCharHandle,
                                   0, size, value);
  
  //return gatt_long_update(SocketTransServHandle, MeasurementCharHandle, size, &value);
}

/**
 * @brief  Update transaction characteristic value
 * @param  from: first transaction to send
 * @param  to: last transaction to send
 * @retval tBleStatus Status
 */
tBleStatus send_register()
{
  tBleStatus ret;
  sck_register_t *reg = get_sck_register();
  
//  uint8_t total_size = 5 * TRANS_SIZE;
//  
//  ret = aci_gatt_update_char_value_ext_IDB05A1(SocketTransServHandle, RegisterCharHandle,
//                                              NOTIFICATION, 2 * TRANS_SIZE,
//                                              0, total_size,
//                                              (uint8_t *)reg->transactions);
  
  if(reg->num > 0)
  {
    ret = gatt_long_update(SocketTransServHandle, RegisterCharHandle, 
                     reg->num * sizeof(transaction_t), reg->transactions);
  } 
  else 
  {
    uint8_t empty = 0;
    ret = gatt_update(SocketTransServHandle, RegisterCharHandle,
                1, &empty);
  }
  
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating characteristic value: 0x%04X\n",ret) ;
    return ret;
  }
  
  return BLE_STATUS_SUCCESS;
}

//tBleStatus char_update_register()
//{
//  tBleStatus ret;
//  
//  sck_register_t *reg = get_sck_register();
//  uint8_t reg_size = reg->num * sizeof(transaction_t);
//  
//  ret = aci_gatt_update_char_value_ext_IDB05A1(SocketTransServHandle, RegisterCharHandle,
//                                                NOTIFICATION, reg_size,
//                                                0, reg_size,
//                                                (uint8_t *)reg->transactions);
//  
//  if (ret != BLE_STATUS_SUCCESS){
//    PRINTF("Error while updating characteristic value: 0x%04X\n",ret) ;
//    return ret;
//  }
//  
//  return BLE_STATUS_SUCCESS;
//}

tBleStatus char_update_current_trans(transaction_t *trans)
{
  tBleStatus res;
  if(trans == NULL)
  {
    uint8_t tr = 0;
    res = aci_gatt_update_char_value(SocketTransServHandle, CurrentTransCharHandle,
                                   0, 1, &tr);
  }
  else 
  {
    res = aci_gatt_update_char_value(SocketTransServHandle, CurrentTransCharHandle,
                                   0, sizeof(transaction_t), trans);
  }
  return res;
}

void char_clear_current_trans()
{
  gatt_update(SocketTransServHandle, CurrentTransCharHandle, 
                   1, (uint8_t *)0);
}

tBleStatus char_update_auth_state(sck_auth_t *auth)
{
  tBleStatus ret;
  
  ret = gatt_update(SocketSecurityServHandle, SecurityCPCharHandle, CP_RESP_SIZE, (void *)RESP_SET_AUTH_ACCEPTED);
  
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating characteristic value: 0x%04X\n",ret) ;
    return ret;
  }
  
  return BLE_STATUS_SUCCESS;
}

tBleStatus char_update_auth_check(uint8_t accepted)
{
  tBleStatus ret;
  
  if(accepted == 1)
    ret = gatt_update(SocketSecurityServHandle, SecurityCPCharHandle, CP_RESP_SIZE, (void *)RESP_LOGIN_ACCEPTED);
  else
    ret = gatt_update(SocketSecurityServHandle, SecurityCPCharHandle, CP_RESP_SIZE, (void *)RESP_LOGIN_REFUSED);
  
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating characteristic value: 0x%04X\n",ret) ;
    return ret;
  }
  
  return BLE_STATUS_SUCCESS;
}

tBleStatus char_update_trans_cp(op_code_e request, uint8_t value, transaction_t *trans)
{
  uint8_t resp[3 + TRANS_SIZE];
  
  resp[0] = RESPONSE;
  resp[1] = (uint8_t)request;
  resp[2] = value;
  memcpy(resp + 3, trans, sizeof(transaction_t));
  return gatt_long_update(SocketTransServHandle, TransCPCharHandle, 3 + TRANS_SIZE, resp);
}

/**
 * @brief  Atomic function for update characteristic value
 * @param  serv_handle Handle of the characteristic's service to update
 * @param  char_handle Handle of the characteristic to update
 * @param  value_len Length of the characteristic value in octets
 * @param  value Characteristic value
 * @retval Value indicating success or error code.
 */
tBleStatus gatt_update(uint16_t serv_handle, uint16_t char_handle, uint8_t value_len, void *value)
{
  tBleStatus ret;
  
  // Same format for long update
//  uint8_t *packet = (uint8_t *)malloc((HEADER_SIZE + value_len)*sizeof(uint8_t));
//  packet[0] = 0;
//  packet[1] = LAST_PCKT;
//  memcpy(packet + 2, value, value_len);
  
//  ret = aci_gatt_update_char_value(serv_handle, char_handle,
//                                   0, value_len + HEADER_SIZE, packet);
  
    ret = aci_gatt_update_char_value(serv_handle, char_handle,
                                   0, value_len, value);
  
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error while updating characteristic value: 0x%04X\n",ret) ;
    return ret;
  }
  
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Atomic function for update characteristic value with length greater than 20 bytes.
 * @param  serv_handle Handle of the characteristic's service to update
 * @param  char_handle Handle of the characteristic to update
 * @param  value_len Length of the characteristic value in octets
 * @param  value Characteristic value
 * @retval Value indicating success or error code.
 */
tBleStatus gatt_long_update(uint16_t serv_handle, uint16_t char_handle, uint32_t value_len, void *value)
{ 
  tBleStatus ret;
  
  busyBtTask();
  sending = 1;
  sendFromIdx = 0;
  nPckts2send = create_pckt_queue(value_len, value);
  
  ret = send_packets_queue(serv_handle, char_handle);
  
  return ret;
}

int create_pckt_queue(uint32_t value_len, void *value)
{
  uint32_t num_packets = 0, size_last_pckt = 0, idx = 0;
  
  size_last_pckt = value_len % CONTENT_MAX_SIZE;
  num_packets = value_len / CONTENT_MAX_SIZE;
  
  for(idx = 0; idx < num_packets; idx++)
  {
    //pckt[0] = (uint8_t)idx;
    pckt_queue[idx].id = (uint8_t)idx;
    
    if(idx == num_packets - 1 && size_last_pckt == 0)
      //pckt[1] = (uint8_t)LAST_PCKT;
      pckt_queue[idx].isLast = LAST_PCKT;
    else
      //pckt[1] = (uint8_t)NOT_LAST_PCKT;
      pckt_queue[idx].isLast = NOT_LAST_PCKT;
    
    //memcpy(pckt + 2, (uint8_t *)value + (idx * 18), 18);
    memcpy(pckt_queue[idx].content, (uint8_t *)value + (idx * CONTENT_MAX_SIZE), CONTENT_MAX_SIZE);
    
    pckt_queue[idx].size = CONTENT_MAX_SIZE;
  }
  
  if(size_last_pckt != 0)
  {
    num_packets++;
    //pckt[0] = (uint8_t)idx;
    //pckt_queue[num_packets].id = (uint8_t)num_packets;
    pckt_queue[idx].id = (uint8_t)idx;
    //pckt[1] = (uint8_t)LAST_PCKT;
    //pckt_queue[num_packets].isLast = (uint8_t)LAST_PCKT;
    pckt_queue[idx].isLast = LAST_PCKT;
    //memcpy(pckt + 2, (uint8_t *)value + (value_len - size_last_pckt), size_last_pckt);
    //memcpy(pckt_queue[num_packets].content, (uint8_t *)value + (value_len - size_last_pckt), size_last_pckt);
    memcpy(pckt_queue[idx].content, (uint8_t *)value + (value_len - size_last_pckt), size_last_pckt);
    //pckt_queue[num_packets].size = size_last_pckt;
    pckt_queue[idx].size = size_last_pckt;
  }
  
  return num_packets;
}

tBleStatus send_packets_queue(uint16_t serv_handle, uint16_t char_handle)
{
  tBleStatus ret;
  BtTaskReq_t req;
  uint8_t packet[20];
  
  for(int i = sendFromIdx; i < nPckts2send; i++)
  {
    packet[0] = pckt_queue[i].id;
    packet[1] = pckt_queue[i].isLast;
    memcpy(packet + 2, pckt_queue[i].content, pckt_queue[i].size);
    //ret = gatt_update(serv_handle, char_handle, pckt_queue[i].size + HEADER_SIZE, packet);
    ret = aci_gatt_update_char_value(serv_handle, char_handle,
                                   0, pckt_queue[i].size + HEADER_SIZE, packet);
    if(ret != BLE_STATUS_SUCCESS)
    {
      // Manage errors
      if(ret == BLE_STATUS_INSUFFICIENT_RESOURCES)
      {
        // Bt queue is full: attempt a retrasmission
        sendFromIdx = i;
        req.type = RETRY_SEND_LONG_CHAR;
        req.serv_handle = serv_handle;
        req.char_handle = char_handle;
        osMessageQueuePut(getBlueMngQueueHandle(), &req, 10, 0);
      }
      return ret;
    }
  }
  // Reset queue
  releaseBtTask();
  sending = 0;
  sendFromIdx = 0;
  memset(pckt_queue, 0, nPckts2send * sizeof(BtPacket_t));
  nPckts2send = 0;
  return  BLE_STATUS_SUCCESS;
}

/**
 * @brief  Received one packet of 20 bytes for a char greather than 20 bytes
 * @param  handle Handle of the char value attribute
 * @param  value Characteristic value
 * @retval Value indicating end of trasmission (last packet)
 */
void packet_received(uint16_t handle, uint8_t *value, uint8_t value_length)
{
  uint8_t pckt[20];
  uint8_t id_pckt;
  uint8_t type_pckt;
  
  memcpy(pckt, value, value_length);
  memcpy(rec_buff + buff_index, value + HEADER_SIZE, CONTENT_MAX_SIZE);
  buff_index += CONTENT_MAX_SIZE;
  id_pckt = value[0];
  type_pckt = value[1];
  PRINTF("Packet received -> ID %d, TYPE %d\n", id_pckt, type_pckt);
  if(type_pckt == LAST_PCKT)
  {
    // all packets received
    gatt_long_received(handle, rec_buff);
    // clear buffer
    memset(rec_buff, 0, MAXBUFFBYTES);
    buff_index = 0;
  }
}

/**
 * @brief  Received a char with size greather than 20 bytes
 * @param  handle Handle of the char value attribute
 * @param  value Characteristic value
 */
void gatt_long_received(uint16_t handle, uint8_t *value)
{
//  REQ_AUTH variables
//  uint8_t name_length, pass_length;
//  // REQ_CHANGE_USER && REQ_CHANGE_PASS variables
//  uint8_t curr_length, newval_length;
//  char curr_val[21];
//  char new_val[21];
//  
//  if(handle == SecurityCPCharHandle + 1)
//  {
////    sck_auth_t rec_account;
////    sck_auth_t *sck_account = get_sck_account();
////    // Format rec_buff using Security request packet format
////    op_code_e op_code = (op_code_e)rec_buff[0];
////    switch(op_code)
////    {
////      case INIT_AUTH:
////        if(sck_account->auth_state == NO_AUTH)
////        {
////          // Login INITIALIZATION
////          gatt_update(SocketSecurityServHandle, SecurityCPCharHandle, CP_RESP_SIZE, (void *)RESP_INIT_LOGIN_ACCEPTED);
////        }
////        else
////        {
////          // Login NO INITIALIZATION
////          gatt_update(SocketSecurityServHandle, SecurityCPCharHandle, CP_RESP_SIZE, (void *)RESP_INIT_LOGIN_REFUSED);
////        }
////        break;
////      case SET_AUTH:
////        name_length = rec_buff[1];
////        pass_length = rec_buff[name_length + 2];
////        memcpy(rec_account.user, rec_buff + 2, name_length);
////        memcpy(rec_account.pass, rec_buff + name_length + 3, pass_length);
////        rec_account.user[name_length] = '\0';
////        rec_account.pass[pass_length] = '\0';
////        PRINTF("Login req:\nname = %s , pass = %s ?\n", rec_account.user, rec_account.pass);
////        strcpy(sck_account->user, rec_account.user);
////        strcpy(sck_account->pass, rec_account.pass);
////        sck_account->auth_state = AUTH_SETTED;
////        // SET AUTH accepted
////        gatt_update(SocketSecurityServHandle, SecurityCPCharHandle, CP_RESP_SIZE, (void *)RESP_SET_AUTH_ACCEPTED);
////        break;
////      case REQ_AUTH:
////        name_length = rec_buff[1];
////        pass_length = rec_buff[name_length + 2];
////        memcpy(rec_account.user, rec_buff + 2, name_length);
////        memcpy(rec_account.pass, rec_buff + name_length + 3, pass_length);
////        rec_account.user[name_length] = '\0';
////        rec_account.pass[pass_length] = '\0';
////        PRINTF("Login req:\nname = %s , pass = %s ?\n", rec_account.user, rec_account.pass);
////        if(check_login(&rec_account))
////        {
////          // Login ACCEPTED
////          gatt_update(SocketSecurityServHandle, SecurityCPCharHandle, CP_RESP_SIZE, (void *)RESP_LOGIN_ACCEPTED);
////        }
////        else
////        {
////          // Login REFUSED
////          gatt_update(SocketSecurityServHandle, SecurityCPCharHandle, CP_RESP_SIZE, (void *)RESP_LOGIN_REFUSED);
////        }
////        break;
////      case REQ_CHANGE_USER:
////      case REQ_CHANGE_PASS:
////        curr_length = rec_buff[1];
////        newval_length = rec_buff[curr_length + 2];
////        memcpy(curr_val, rec_buff + 2, curr_length);
////        memcpy(new_val, rec_buff + curr_length + 3, newval_length);
////        curr_val[curr_length] = '\0';
////        new_val[newval_length] = '\0';
////        if(op_code == REQ_CHANGE_USER)
////        {
////          PRINTF("Change User req:\ncurrent = %s , new = %s ?\n\r", curr_val, new_val);
////          if(strcmp(sck_account->user, curr_val) == 0)
////          {
////            // User CHANGED
////            PRINTF("User CHANGED\n\r");
////            // Change user
////            strcpy(sck_account->user, new_val);
////            // Save in memory
////            // @TODO
////            gatt_update(SocketSecurityServHandle, SecurityCPCharHandle, CP_RESP_SIZE, (void *)RESP_CHANGE_USER_ACCEPTED);
////          }
////          else
////          {
////            gatt_update(SocketSecurityServHandle, SecurityCPCharHandle, CP_RESP_SIZE, (void *)RESP_CHANGE_USER_REFUSED);
////          }
////        }
////        else if(op_code == REQ_CHANGE_PASS)
////        {
////          PRINTF("Change Pass req:\ncurrent = %s , new = %s ?\n\r", curr_val, new_val);
////          if(strcmp(sck_account->pass, curr_val) == 0)
////          {
////            // Password CHANGED
////            PRINTF("Password CHANGED\n\r");
////            // Change password
////            strcpy(sck_account->pass, new_val);
////            // Save in memory
////            // @TODO
////            gatt_update(SocketSecurityServHandle, SecurityCPCharHandle, CP_RESP_SIZE, (void *)RESP_CHANGE_PASS_ACCEPTED);
////          }
////          else
////          {
////            gatt_update(SocketSecurityServHandle, SecurityCPCharHandle, CP_RESP_SIZE, (void *)RESP_CHANGE_PASS_REFUSED);
////          }
////        }
////        break;
////      default:
////      case UNKNOWN:
////        break;
////    }
//  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

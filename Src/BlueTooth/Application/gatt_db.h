/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
* File Name          : gatt_db.h
* Author             :
* Version            : V1.0.0
* Date               : 16-September-2015
* Description        : Header file for gatt_db.c
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifndef GATT_DB_H
#define GATT_DB_H

#include <stdint.h>
#include <stdlib.h>
#include "bluenrg_def.h"

#include "blueMng.h"
#include "EvsMng.h"

/* GATT long update supports */
#define HEADER_SIZE     2       // bytes
#define CONTENT_MAX_SIZE    18      // bytes
#define NOT_LAST_PCKT   0
#define LAST_PCKT       1
#define MAXBUFFBYTES    512     // bytes
#define MAXPCKT         100     // packets

typedef struct {
  uint8_t id;
  uint8_t isLast;
  uint8_t content[CONTENT_MAX_SIZE];
  uint8_t size;                         // bytes
} BtPacket_t;

/**
 * @brief Number of application services
 */
#define NUMBER_OF_APPLICATION_SERVICES (2)

/**
 * @brief Define How Many quaterions you want to trasmit (from 1 to 3)
 *        In this sample application use only 1
 */
#define SEND_N_QUATERNIONS 1

/**
 * @brief Structure containing acceleration value of each axis.
 */
typedef struct {
  int32_t AXIS_X;
  int32_t AXIS_Y;
  int32_t AXIS_Z;
} AxesRaw_t;

enum {
  ACCELERATION_SERVICE_INDEX = 0,
  ENVIRONMENTAL_SERVICE_INDEX = 1
};

/** Documentation for C union Service_UUID_t */
typedef union Service_UUID_t_s {
  /** 16-bit UUID
  */
  uint16_t Service_UUID_16;
  /** 128-bit UUID
  */
  uint8_t Service_UUID_128[16];
} Service_UUID_t;

/** Documentation for C union Char_UUID_t */
typedef union Char_UUID_t_s {
  /** 16-bit UUID
  */
  uint16_t Char_UUID_16;
  /** 128-bit UUID
  */
  uint8_t Char_UUID_128[16];
} Char_UUID_t;

tBleStatus Add_HWServW2ST_Service(void);
tBleStatus Add_SWServW2ST_Service(void);
void Read_Request_CB(uint16_t handle, uint16_t offset);
tBleStatus BlueMS_Environmental_Update(int32_t press, int16_t temp);
tBleStatus Acc_Update(AxesRaw_t *x_axes, AxesRaw_t *g_axes, AxesRaw_t *m_axes);
tBleStatus Quat_Update(AxesRaw_t *q_axes);

void Write_Request_CB(uint16_t handle, uint8_t *value, uint8_t value_length);
void Attribute_Modified_CB(uint16_t handle, uint8_t *value, uint8_t value_length);

tBleStatus Add_Scame_Services(void);

tBleStatus turn_on_notify();
tBleStatus char_update_serial();
tBleStatus char_update_firmware();
tBleStatus char_update_emeters();
tBleStatus char_update_rfid();
tBleStatus char_update_max_current();
tBleStatus char_update_wiring();
tBleStatus char_update_max_power();
tBleStatus char_update_id();
tBleStatus char_update_state();
tBleStatus char_update_fun_mode();
tBleStatus char_update_pwr_mode();

tBleStatus char_update_current_trans(transaction_t *trans);

tBleStatus char_update_test(void *value);
//tBleStatus char_update_register();

tBleStatus send_register();

void char_clear_current_trans();

tBleStatus char_update_measures(sck_measures_t *m);

tBleStatus char_update_auth_state(sck_auth_t *auth);
tBleStatus char_update_auth_check(uint8_t accepted);

tBleStatus char_update_trans_cp(op_code_e request, uint8_t value, transaction_t *trans);

tBleStatus send_packets_queue(uint16_t serv_handle, uint16_t char_handle);

extern uint8_t Services_Max_Attribute_Records[];

#endif /* GATT_DB_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

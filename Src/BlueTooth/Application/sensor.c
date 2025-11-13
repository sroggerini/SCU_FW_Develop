/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
 * File Name          : sensor.c
 * Author             : AMS - VMA RF Application team
 * Version            : V1.0.0
 * Date               : 23-November-2015
 * Description        : Sensor init and sensor state machines
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "sensor.h"
#include "gatt_db.h"
#include "bluenrg_gap.h"
#include "bluenrg_gap_aci.h"
#include "hci_le.h"
#include "hci_const.h"
#include "bluenrg_aci_const.h"
#include "bluenrg_gatt_aci.h"

// Bluetooth Task includes
#include "blueMng.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define  ADV_INTERVAL_MIN_MS  20
#define  ADV_INTERVAL_MAX_MS  50
#define CONN_INTERVAL_MIN_MS  150
#define CONN_INTERVAL_MAX_MS  300

/** @brief Macro that stores Value into a buffer in Little Endian Format (2 bytes)*/
#define HOST_TO_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint8_t bdaddr[BDADDR_SIZE];
extern uint8_t bnrg_expansion_board;
__IO uint8_t set_connectable = 1;
__IO uint16_t connection_handle = 0;
__IO uint8_t  notification_enabled = FALSE;
__IO uint32_t connected = FALSE;

extern BluetoothState btState;

extern uint16_t EnvironmentalCharHandle;
extern uint16_t AccGyroMagCharHandle;

volatile uint8_t request_free_fall_notify = FALSE;

AxesRaw_t x_axes = {0, 0, 0};
AxesRaw_t g_axes = {0, 0, 0};
AxesRaw_t m_axes = {0, 0, 0};
AxesRaw_t q_axes[SEND_N_QUATERNIONS] = {{0, 0, 0}};

uint32_t counter = 0;
uint32_t led_state = 0;

/* Private function prototypes -----------------------------------------------*/
tBleStatus Set_DeviceConnectable(socket_t * socket);
void GAP_DisconnectionComplete_CB(void);
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
 * Function Name  : Set_DeviceConnectable.
 * Description    : Puts the device in connectable mode.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
tBleStatus Set_DeviceConnectable(socket_t * socket)
{
  tBleStatus ret;
  
  char local_name[MAX_NAME_LENGTH];
  strcpy(local_name + 1, socket->name);
  local_name[0] = AD_TYPE_COMPLETE_LOCAL_NAME;
  
  uint8_t company_id[2] = {ID_SCAME};
  
  uint8_t scu_adv_data[MAX_NAME_LENGTH+5] = {
    2,0x0A,0x08, /* 8 dBm */    // Trasmission Power          
    1+MAX_NAME_LENGTH,0x09      // Complete Name
  };
  
  memcpy(scu_adv_data + 5, socket->name, MAX_NAME_LENGTH);
  
  // Prepare ADV response data
  uint8_t state[2];
  state[0] = socket->state.value;
  state[1] = socket->state.error;
 
  uint8_t resp_data[] = {
    7, 0xFF,                    // Manifacturer Section
    company_id[1],              // Manifacturer Id (Reserved Id Free)        
    company_id[0], 
    socket->id,                 // Socket id
    socket->wiring,             // Socket wiring
    state[1],                   // Socket state
    state[0]
  };

  //hci_le_set_scan_resp_data(0, NULL);
  hci_le_set_scan_resp_data(sizeof(resp_data), resp_data);

  PRINTF("Set General Discoverable Mode.\n");

  ret = aci_gap_set_discoverable(ADV_IND,
                                (ADV_INTERVAL_MIN_MS*1000)/625,(ADV_INTERVAL_MAX_MS*1000)/625,
                                 STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE,
                                 sizeof(local_name), local_name, 0, NULL, 
                                 (CONN_INTERVAL_MIN_MS*1000)/1250, (CONN_INTERVAL_MAX_MS*1000)/1250);

  aci_gap_update_adv_data(MAX_NAME_LENGTH+5, scu_adv_data);

  if(ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("aci_gap_set_discoverable() failed: 0x%02x\r\n", ret);
  }
  else
    PRINTF("aci_gap_set_discoverable() --> SUCCESS\r\n");
  
  return ret;
} 

/**
 * @brief  Callback processing the ACI events.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
void user_notify(void * pData)
{
  hci_uart_pckt *hci_pckt = pData;
  /* obtain event packet */
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

  if(hci_pckt->type != HCI_EVENT_PKT)
    return;

  //PRINTF("pckt event: 0x%02X\n", event_pckt->evt);
  
  switch(event_pckt->evt){

  case EVT_DISCONN_COMPLETE:
    {
      GAP_DisconnectionComplete_CB();
    }
    break;

  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;
      //PRINTF("evt->subevent: 0x%02X\n", evt->subevent);
      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
          evt_le_connection_complete *cc = (void *)evt->data;
          GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;
      }
    }
    break;

  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
      //PRINTF("blue_evt->ecode: 0x%04X\n", blue_evt->ecode);
      switch(blue_evt->ecode){
      
      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        {
          evt_gatt_attr_modified_IDB05A1 *pr = (void*)blue_evt->data;
          Attribute_Modified_CB(pr->attr_handle, pr->att_data, pr->data_length);
        }
        break;
        
      case EVT_BLUE_GATT_READ_PERMIT_REQ:
        {
          evt_gatt_read_permit_req *pr = (void*)blue_evt->data;
          Read_Request_CB(pr->attr_handle, pr->offset);
        }
        break;
       
      case EVT_BLUE_GATT_WRITE_PERMIT_REQ:
        {
          evt_gatt_write_permit_req *pr = (void*)blue_evt->data;
          Write_Request_CB(pr->attr_handle, pr->data, pr->data_length);
        }
        break;
      }

    }
    break;
  }
}

/**
 * @brief  This function is called when the peer device gets disconnected.
 * @param  None
 * @retval None
 */
void GAP_DisconnectionComplete_CB(void)
{
  connected = FALSE;
  PRINTF("Disconnected\n");
  /* Make the device connectable again. */
  set_connectable = TRUE;
  notification_enabled = FALSE;
  // Change state for Bluetooth Task
  btState = DISCONNECTED;
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t Address of peer device
 * @param  uint16_t Connection handle
 * @retval None
 */
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{
  connected = TRUE;
  connection_handle = handle;

  PRINTF("Connected to device:");
  for(uint32_t i = 5; i > 0; i--){
    PRINTF("%02X-", addr[i]);
  }
  PRINTF("%02X\n", addr[0]);
  // Change state for Bluetooth Task
  btState = IN_CONNECTION;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

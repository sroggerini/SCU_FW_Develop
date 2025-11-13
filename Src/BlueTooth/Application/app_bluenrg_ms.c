/**
  ******************************************************************************
  * File Name          : app_bluenrg_ms.c
  * Description        : Source file
  *
  ******************************************************************************
  *
  * COPYRIGHT 2020 STMicroelectronics
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_bluenrg_ms.h"

#include "hci.h"
#include "hci_le.h"
#include "hci_tl.h"
#include "link_layer.h"
#include "sensor.h"
#include "gatt_db.h"

#include "compiler.h"
#include "bluenrg_utils.h"
#include "stm32l0xx_nucleo.h"
#include "bluenrg_gap.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_hal_aci.h"
#include "sm.h"
//#include "stm32l0xx_hal_tim.h"
#include "stm32f7xx_hal_tim.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private defines -----------------------------------------------------------*/
/**
 * 1 to send environmental and motion data when pushing the user button
 * 0 to send environmental and motion data automatically (period = 1 sec)
 */
#define USE_BUTTON 0

/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern AxesRaw_t x_axes;
extern AxesRaw_t g_axes;
extern AxesRaw_t m_axes;
extern AxesRaw_t q_axes;

extern volatile uint8_t set_connectable;
extern volatile int     connected;
/* at startup, suppose the X-NUCLEO-IDB04A1 is used */
uint8_t bnrg_expansion_board = IDB04A1;
uint8_t bdaddr[BDADDR_SIZE];
static volatile uint8_t user_button_init_state = 1;
static volatile uint8_t user_button_pressed = 0;

uint16_t gacc_service_handle, dev_name_char_handle, appearance_char_handle;

/* USER CODE BEGIN PV */
extern uint32_t counter;
extern uint32_t led_state;
extern uint32_t curCount;
extern uint32_t preCount;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void User_Process(void);
//static void User_Init(void);
//static void Set_Random_Environmental_Values(float *data_t, float *data_p);
//static void Set_Random_Motion_Values(uint32_t cnt);
//static void Reset_Motion_Values(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

#if PRINT_CSV_FORMAT
extern volatile uint32_t ms_counter;
/**
 * @brief  This function is a utility to print the log time
 *         in the format HH:MM:SS:MSS (DK GUI time format)
 * @param  None
 * @retval None
 */
void print_csv_time(void){
  uint32_t ms = HAL_GetTick();
  PRINT_CSV("%02ld:%02ld:%02ld.%03ld", ms/(60*60*1000)%24, ms/(60*1000)%60, (ms/1000)%60, ms%1000);
}
#endif

void MX_BlueNRG_MS_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN BlueNRG_MS_Init_PreTreatment */

  /* USER CODE END BlueNRG_MS_Init_PreTreatment */

  /* Initialize the peripherals and the BLE Stack */
  const char *name = "ChargePoint SCAME";

  uint8_t  bdaddr_len_out;
  uint8_t  hwVersion;
  uint16_t fwVersion;
  int ret;

  hci_init(user_notify, NULL);

  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  /*
   * Reset BlueNRG again otherwise we won't
   * be able to change its MAC address.
   * aci_hal_write_config_data() must be the first
   * command after reset otherwise it will fail.
   */
  hci_reset();
  HAL_Delay(100);

  PRINTF("HWver %d\nFWver %d\n", hwVersion, fwVersion);
  if (hwVersion > 0x30) { /* X-NUCLEO-IDB05A1 expansion board is used */
    bnrg_expansion_board = IDB05A1;
  }

  ret = aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS, BDADDR_SIZE, &bdaddr_len_out, bdaddr);

  if (ret) {
    PRINTF("Read Static Random address failed.\n");
  }

  if ((bdaddr[5] & 0xC0) != 0xC0) {
    PRINTF("Static Random address not well formed.\n");
    while(1);
  }
  
  /* Print address device */
  PRINTF("Device BLE address: ");
  for ( int i = BDADDR_SIZE-1; i > 0; i-- )
  {
    PRINTF("%02X-", bdaddr[i]);
  }
  PRINTF("%02X", bdaddr[0]);
  PRINTF("\n");

  /* GATT Init */
  ret = aci_gatt_init();
  if(ret){
    PRINTF("GATT_Init failed.\n");
  }

  /* GAP Init */
  if (bnrg_expansion_board == IDB05A1) {
    //ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &gacc_service_handle, &dev_name_char_handle, &appearance_char_handle);
    ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x11, &gacc_service_handle, &dev_name_char_handle, &appearance_char_handle);
  }
  else {
    ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &gacc_service_handle, &dev_name_char_handle, &appearance_char_handle);
  }
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("GAP_Init failed.\n");
  }

  /* Update device name */
  ret = aci_gatt_update_char_value(gacc_service_handle, dev_name_char_handle, 0,
                                   strlen(name), (uint8_t *)name);
  if (ret) {
    PRINTF("aci_gatt_update_char_value failed.\n");
    while(1);
  }

  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL,
                                     7,
                                     16,
                                     USE_FIXED_PIN_FOR_PAIRING,
                                     123456,
                                     BONDING);
  if (ret) {
    PRINTF("aci_gap_set_authentication_requirement failed.\n");
    while(1);
  }

  PRINTF("BLE Stack Initialized\n");
  
  /* Add my personal service */
  ret = Add_Scame_Services();
  if(ret == BLE_STATUS_SUCCESS) {
     PRINTF("BlueMS Personal service added successfully.\n");
  } else {
     PRINTF("Error while adding BlueMS Personal service: 0x%02x\r\n", ret);
     while(1);
  }

  /* Set output power level */
  ret = aci_hal_set_tx_power_level(1,7);

  /* USER CODE BEGIN BlueNRG_MS_Init_PostTreatment */
  
  /* USER CODE END BlueNRG_MS_Init_PostTreatment */
}

/*
 * BlueNRG-MS background task
 */
void MX_BlueNRG_MS_Process(void)
{
  /* USER CODE BEGIN BlueNRG_MS_Process_PreTreatment */

  /* USER CODE END BlueNRG_MS_Process_PreTreatment */

  User_Process();
  hci_user_evt_proc();

  /* USER CODE BEGIN BlueNRG_MS_Process_PostTreatment */

  /* USER CODE END BlueNRG_MS_Process_PostTreatment */
}

/**
 * @brief  Initialize User process.
 *
 * @param  None
 * @retval None
 */
//static void User_Init(void)
//{
//  
//}

/**
 * @brief  Process user input (i.e. pressing the USER button on Nucleo board)
 *         and send the updated acceleration data to the remote client.
 *
 * @param  None
 * @retval None
 */
static void User_Process(void)
{
  
}

/**
 * @brief  Set random values for all environmental sensor data
 * @param  float pointer to temperature data
 * @param  float pointer to pressure data
 * @retval None
 */
//static void Set_Random_Environmental_Values(float *data_t, float *data_p)
//{
//  *data_t = 27.0 + ((uint64_t)rand()*5)/RAND_MAX;     /* T sensor emulation */
//  *data_p = 1000.0 + ((uint64_t)rand()*80)/RAND_MAX; /* P sensor emulation */
//}

/**
 * @brief  Set random values for all motion sensor data
 * @param  uint32_t counter for changing the rotation direction
 * @retval None
 */
//static void Set_Random_Motion_Values(uint32_t cnt)
//{
//  /* Update Acceleration, Gyroscope and Sensor Fusion data */
//  if (cnt < 20) {
//    x_axes.AXIS_X +=  (10  + ((uint64_t)rand()*3*cnt)/RAND_MAX);
//    x_axes.AXIS_Y += -(10  + ((uint64_t)rand()*5*cnt)/RAND_MAX);
//    x_axes.AXIS_Z +=  (10  + ((uint64_t)rand()*7*cnt)/RAND_MAX);
//    g_axes.AXIS_X +=  (100 + ((uint64_t)rand()*2*cnt)/RAND_MAX);
//    g_axes.AXIS_Y += -(100 + ((uint64_t)rand()*4*cnt)/RAND_MAX);
//    g_axes.AXIS_Z +=  (100 + ((uint64_t)rand()*6*cnt)/RAND_MAX);
//    m_axes.AXIS_X +=  (3  + ((uint64_t)rand()*3*cnt)/RAND_MAX);
//    m_axes.AXIS_Y += -(3  + ((uint64_t)rand()*4*cnt)/RAND_MAX);
//    m_axes.AXIS_Z +=  (3  + ((uint64_t)rand()*5*cnt)/RAND_MAX);
//
//    q_axes.AXIS_X -= (100  + ((uint64_t)rand()*3*cnt)/RAND_MAX);
//    q_axes.AXIS_Y += (100  + ((uint64_t)rand()*5*cnt)/RAND_MAX);
//    q_axes.AXIS_Z -= (100  + ((uint64_t)rand()*7*cnt)/RAND_MAX);
//  }
//  else {
//    x_axes.AXIS_X += -(10  + ((uint64_t)rand()*3*cnt)/RAND_MAX);
//    x_axes.AXIS_Y +=  (10  + ((uint64_t)rand()*5*cnt)/RAND_MAX);
//    x_axes.AXIS_Z += -(10  + ((uint64_t)rand()*7*cnt)/RAND_MAX);
//    g_axes.AXIS_X += -(100 + ((uint64_t)rand()*2*cnt)/RAND_MAX);
//    g_axes.AXIS_Y +=  (100 + ((uint64_t)rand()*4*cnt)/RAND_MAX);
//    g_axes.AXIS_Z += -(100 + ((uint64_t)rand()*6*cnt)/RAND_MAX);
//    m_axes.AXIS_X += -(3  + ((uint64_t)rand()*7*cnt)/RAND_MAX);
//    m_axes.AXIS_Y +=  (3  + ((uint64_t)rand()*9*cnt)/RAND_MAX);
//    m_axes.AXIS_Z += -(3  + ((uint64_t)rand()*3*cnt)/RAND_MAX);
//
//    q_axes.AXIS_X += (200 + ((uint64_t)rand()*7*cnt)/RAND_MAX);
//    q_axes.AXIS_Y -= (150 + ((uint64_t)rand()*3*cnt)/RAND_MAX);
//    q_axes.AXIS_Z += (10  + ((uint64_t)rand()*5*cnt)/RAND_MAX);
//  }
//
//}

/**
 * @brief  Reset values for all motion sensor data
 * @param  None
 * @retval None
 */
//static void Reset_Motion_Values(void)
//{
//  x_axes.AXIS_X = (x_axes.AXIS_X)%2000 == 0 ? -x_axes.AXIS_X : 10;
//  x_axes.AXIS_Y = (x_axes.AXIS_Y)%2000 == 0 ? -x_axes.AXIS_Y : -10;
//  x_axes.AXIS_Z = (x_axes.AXIS_Z)%2000 == 0 ? -x_axes.AXIS_Z : 10;
//  g_axes.AXIS_X = (g_axes.AXIS_X)%2000 == 0 ? -g_axes.AXIS_X : 100;
//  g_axes.AXIS_Y = (g_axes.AXIS_Y)%2000 == 0 ? -g_axes.AXIS_Y : -100;
//  g_axes.AXIS_Z = (g_axes.AXIS_Z)%2000 == 0 ? -g_axes.AXIS_Z : 100;
//  m_axes.AXIS_X = (g_axes.AXIS_X)%2000 == 0 ? -m_axes.AXIS_X : 3;
//  m_axes.AXIS_Y = (g_axes.AXIS_Y)%2000 == 0 ? -m_axes.AXIS_Y : -3;
//  m_axes.AXIS_Z = (g_axes.AXIS_Z)%2000 == 0 ? -m_axes.AXIS_Z : 3;
//  q_axes.AXIS_X = -q_axes.AXIS_X;
//  q_axes.AXIS_Y = -q_axes.AXIS_Y;
//  q_axes.AXIS_Z = -q_axes.AXIS_Z;
//}

/**
  * @brief  BSP Push Button callback
  * @param  Button Specifies the pin connected EXTI line
  * @retval None.
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  /* Set the User Button flag */
  user_button_pressed = 1;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

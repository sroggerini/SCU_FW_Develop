/**
* @file        wrapper.h
*
* @brief       Wrapper tra station manager e low level HW - Definition -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: wrapper.h 749 2025-05-07 12:32:38Z stefano $
*
*     $Revision: 749 $
*
*     $Author: stefano $
*
*     $Date: 2025-05-07 14:32:38 +0200 (mer, 07 mag 2025) $
*
*
* @copyright
*       Copyright (C) 2016 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

#ifndef __WRAPPER_H 
#define __WRAPPER_H

/************************************************************
 * Include
 ************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#else
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_gpio_legacy.h"
#endif
#include "cmsis_os.h"
//#include "msg.h"
#include "EvsMng.h"
#include "Em_Task.h"
#include "EnergyMng.h"
#include "sbcSem.h"
#include "scheduleMng.h"
#include "app_emob.h"
#include "Event_log.h"    
#include "eeprom.h"    
/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
      
#define CHAR_NUM    (20)

#define WCENTER		        ((uint8_t)(0x81))
#define WLEFT		          ((uint8_t)(0x82))
#define WRIGHT		        ((uint8_t)(0x83))
#define WOFF		          ((uint8_t)(0x85))

#define DISP_LINE1	      ((uint8_t)(0x84))
#define DISP_LINE2	      ((uint8_t)(0x85))


#define DISP_FIRST_LINE	  ((uint8_t)1)
#define DISP_SECOND_LINE	((uint8_t)2)
#define DISP_FIRST_COLUMN ((uint8_t)1)

/* function code implemented for MODBUS */
#define   FUNCTION_READ_HOLDING_REG     ((uint8_t)3)
#define   FUNCTION_READ_INPUT_REG       ((uint8_t)4)
#define   FUNCTION_READ_EX_INPUT_REG    ((uint8_t)5)
#define   FUNCTION_WRITE_MULTIPLE_REG   ((uint8_t)16)
#define   FUNCTION_REPORT_SLAVE_ID_REG  ((uint8_t)17)
#define   FUNCTION_WRITE_SINGLE_REG     ((uint8_t)6)
#define   FUNCTION_ERROR_OCCURED        ((uint8_t)0x80)

/* Timeout  definition */
#define   TIMER_10MS                 pdMS_TO_TICKS(10)
#define   TIMER_50MS                 pdMS_TO_TICKS(50)
#define   TIMER_100MS                pdMS_TO_TICKS(100)
#define   TIMER_200MS                pdMS_TO_TICKS(200)
#define   TIMER_500MS                pdMS_TO_TICKS(500)
#define   TIMER_1SEC                 pdMS_TO_TICKS(1000)

#define   DELTA_CURR_MA              ((uint32_t)1000)
#define   DELTA_PWR_W                ((uint32_t)200)
#define   DELTA_ENRG_WH              ((uint32_t)100)
#define   DELTA_TRANS_TIME           ((uint32_t)5)

/*** define at fix SRAM location to activate MPU on this area ***/
#define INFO_ADDR           ((uint32_t)0x20056e90)
#define MPU_AREA            0x200
#define INFO_SIZE           sizeof(infoStation_t)
#define MPU_AREA_SIZE       MPU_REGION_SIZE_512B
#define RAM_REGION_NUMBER   MPU_REGION_NUMBER0

#define portMPU_REGION_READ_WRITE                MPU_REGION_FULL_ACCESS
#define portMPU_REGION_PRIVILEGED_READ_ONLY      MPU_REGION_PRIV_RO
#define portMPU_REGION_READ_ONLY                 MPU_REGION_PRIV_RO_URO
#define portMPU_REGION_PRIVILEGED_READ_WRITE     MPU_REGION_PRIV_RW

#define DUMMY_INFO_VAL                ((uint32_t)0xFFFFFFFF)
#define VALIDITY_PERIOD_COLLAUDO_USER ((uint32_t)7200)       /* 2h the validity from the first access in collaudo web account */  
#define CONF_DATA_OK                  ((uint16_t)0x00B0)
#define CONF_DATA_OK_MASK             ((uint16_t)0x00F0)
#define CONF_PASS_OK                  ((uint16_t)0x000A)
#define CONF_PASS_OK_MASK             ((uint16_t)0x000F)
#define CONF_DATA_AND_PASSWORD_MASK   ((uint16_t)(CONF_DATA_OK_MASK | CONF_PASS_OK_MASK))
#define CONF_DATA_AND_PASSWORD_OK     ((uint16_t)(CONF_DATA_OK | CONF_PASS_OK))
#define CONF_USER_PASS_CHANGED        ((uint16_t)0x0C00)
#define CONF_USER_PASS_CHANGED_MASK   ((uint16_t)0x0F00)

/* ***************  definizione I/O per gestione Ingressi Digitali  ************** */
typedef enum 
{  
  IT  = 0,                  // Italia
  FR,                       // Francia
  UK,                       // Inghilterra
  AS,                       // Arabia Saudita
  CZ,                       // Repubblica Ceca
  PULS1,                    // Pulsante1
  PULS2,                    // Pulsante2
  RCBO1,                    // RCBO1
  RCBO2,                    // RCBO2
  RCBO3,                    // RCBO3
  RCBO4,                    // RCBO4
  D,                        // Germania
  ES,                       // Spagna
  NUM_COUNTRY 
}countryId_e;


#define STATUS_PIN_LOW    ((uint8_t)0)    
#define STATUS_PIN_HIGH   ((uint8_t)1)   
 
#define R_ACT_STATUS_MASK ((uint16_t)0x40)
#ifdef GD32F4xx
/* In case of GD32F4xx, the PEN_ALM input is on U40.1 */
#define PEN_ALM_MASK      ((uint16_t)0x02)
#define PEN_ALM_Pos       ((uint16_t)0x01)
#else
/* In case of GD32F4xx, the PEN_ALM input is on U39.3 */
#define PEN_ALM_MASK      ((uint16_t)0x08)
#endif

#define IN1_MASK          ((uint16_t)0x01)
#define IN3_MASK          ((uint16_t)0x04)
#define IN4_MASK          ((uint16_t)0x08)
#define IN7_MASK          ((uint16_t)0x40)
#define IN8_MASK          ((uint16_t)0x80)

#define IsUpperChar(c)	  (((c)>='A')&&((c)<='Z'))
#define IsDigit(c)	      (((c)>='0')&&((c)<='9'))

#ifndef HW_MP28947

typedef enum 
{  
  IN0_DIODO_ADC_PIN_UP  = 0,       // PA3 = DIODO_ADC
  RCDM_IN_UP_PIN_UP,               // PA6 = RCDM --> DC Leakage
  IN2_PULSANTE_UP_PIN_UP,          // PB10= PULS/STOP
  IN5_BLK_UP_PIN_UP,               // PD0 = BLK_UP   
  IN6_BLK_DWN_PIN_UP,              // PC10 = BLK_DWN 
  SWPROG_PIN_UP,                   // PD14 = SW_PROG 
  IN9_RFID_DTC_PIN_UP,             // PB15 = RFID_DTC
  PEN_ALARM,                       // U39.IO3 = PEN_ALM
  R_ACT_STATUS_EXP1,               // U39.IO6 = R_ACT_STATUS
  IN1_REMOTE_EXP0,                 // U40.IO0 = REMOTE
  IN3_MIRROR_EXP0,                 // U40.IO2 = MIRROR
  IN4_RCBO_EXP0,                   // U40.IO3 = RCBO
  IN7_VENT_EXP0,                   // U40.IO6 = Ventilazione
  IN8_LID_EXP0,                    // U40.IO7 = LID
  NUM_INPUT_SCU 
}dIn_TypeDef;

#define NUM_INPUT_SCU_PIN_UP      ((uint16_t)IN9_RFID_DTC_PIN_UP + (uint16_t)1) 

#else

/* Hardware MP28947 doesn't have the IO expander, so only GPIO are managed */

typedef enum 
{  
  IN0_DIODO_ADC_PIN_UP  = 0,       // PD9 = DIODO_ADC
  RCDM_IN_UP_PIN_UP,               // PE7 = RCDM --> DC Leakage
  SWPROG_PIN_UP,                   // PD14 = SW_PROG 
  IN1_REMOTE_EXP0,                 // PB13 = REMOTE  
  IN3_MIRROR_EXP0,                 // PB9 = MIRROR
  IN9_RFID_DTC_PIN_UP,             // PD7 = RFID_DTC
  
  NUM_OF_GPIO_INPUT,        // number of GPIO managed by HW_MP28947
  
  IN2_PULSANTE_UP_PIN_UP,          // PB10= PULS/STOP
  IN5_BLK_UP_PIN_UP,               // PD0 = BLK_UP   
  IN6_BLK_DWN_PIN_UP,              // PC10 = BLK_DWN 
  PEN_ALARM,                       // U39.IO3 = PEN_ALM
  R_ACT_STATUS_EXP1,               // U39.IO6 = R_ACT_STATUS
  IN4_RCBO_EXP0,                   // U40.IO3 = RCBO
  IN7_VENT_EXP0,                   // U40.IO6 = Ventilazione
  IN8_LID_EXP0,                    // U40.IO7 = LID
  NUM_INPUT_SCU, 

}dIn_TypeDef;

#define NUM_INPUT_SCU_PIN_UP      NUM_OF_GPIO_INPUT

/* Not managed I/O */


#endif

typedef enum 
{  
  OUTBL1_P  = 0,            // PD8 = Comando motore +
  OUTBL1_M,                 // PD3 = Comando motore -
  CNTCT,                    // PD10 = Contattore   
#ifdef HW_MP28947
  CNTCT_PWM,                // PD15 = Comando contattore con segnale PWM
#endif  
  PWM_CP,                   // PC6 = PWM su CP
  SGCBOB,                   // PD7 = id per bobina di sgancio
  NUM_OUTPUT_SCU 
}dOut_TypeDef;

typedef enum 
{  
  GPIO1         = 0,    // bit 0: GPIO1 input
  IN_HWV,               // bit 1: versione HW   
  OUT_E_STATE,          // bit 2: is an output PIN (E_STATE)  
  IN_PEN_ALM,           // bit 3: input, PEN alarm  
  OUT_REM_ACT_ON,       // bit 4: is an output PIN (OUT_REM_ACT_ON)  
  OUT_REM_ACT_OFF,      // bit 5: is an output PIN (OUT_REM_ACT_OFF)  
  IN_REM_ACT_STATUS,    // bit 6: Automatic differential: Present / absent indication     
  GPIO0,                // bit 7: GPIO0 input/output 
  IN1_REMOTE,           // bit 0: IN1 input = REMOTE
  OUT_VMUTE,            // bit 1: Mute or power down audio amplifier   
  IN3_MIRROR,           // bit 2: IN3 input = MIRROR    
  IN4_RCBO,             // bit 3: IN3 input = RCBO  
  OUT_FIND_M,           // bit 4: is an output PIN (flag HW for linked SCU)  
  IN_N_FAULT,           // bit 5: is an input PIN (motor driver status)  
  IN7_VENT,             // bit 6: IN7 input = VENTilazione  
  IN8_LID_IN,           // bit 7: IN8 input = LID (coperchio)  
  NUM_IN_IOEXP 
}ioIn_TypeDef_e;

typedef enum 
{  
  E_STATE = 0,          // E_STATE Command
  REM_ACT_ON,           // Automatic differential: On Command    
  REM_ACT_OFF,          // Automatic differential: Off Command    
  V_MUTE,               // disable / enable audio amplifier     
  MIRROR_ENA,           // disable / enable mirror contact detection     
  NUM_OUT_IOEXP 
}ioOut_TypeDef_e;


typedef enum 
{ 
  VIN_ADC_IN  = 0,           // PB1 = ADC1_IN9 lettura analogica della tensione di ingresso
#ifndef HW_MP28947  
  PP_ADC_IN,                 // PB0 = ADC1_IN8 lettura analogica del PP
  SW_ADC_IN,                 // PC2 = ADC1_IN12 lettura analogica dello switch rotativo di corrente   
  UP_TEMP_IN,                // Internal temperature 
  TEMP_ADC_IN,               // PC3 = ADC1_IN13 lettura analogica della temperatura con NTC esterno
  IM_ADC_IN,                 // PA3 = ADC1_IN3 lettura analogica corrente nel motore blocco presa
#else
  UP_TEMP_IN,                // Internal temperature 
  TEMP_ADC_IN,               // PC3 = ADC1_IN13 lettura analogica della temperatura con NTC esterno
  TA23_TYPE_ADC_IN,          // PA4 = TA23TYPE lettura tipo di TA montato sulal scheda MP28947
  TA1_TYPE_ADC_IN,           // PB0 = TA1TYPE lettura tipo TA montato sulla scheda MP28072
#endif  
  NUM_ADC_SCU, 
  CP_ADC_IN                  // PA0 = ADC1_IN0 lettura analogica del CP
}adcIn_e;

typedef enum 
{  
  POS0_6A  = 0,           // Position 0 --> 6A
  POS1_10A,               // Position 1 --> 10A   
  POS2_13A,               // Position 2 --> 13A
  POS3_16A,               // Position 3 --> 16A  
  POS4_20A,               // Position 4 --> 20A    
  POS5_25A,               // Position 5 --> 25A   
  POS6_32A,               // Position 6 --> 32A 
  POS7_40A,               // Position 7 --> 40A  
  POS8_50A,               // Position 8 --> 50A  
  POS9_63A,               // Position 9 --> 63A  
  NUM_ROTARY_POS 
}rotaryPos_e;

/* ***************  definizione  gestione led RGB  ************** */
typedef enum
{
  LED_C_RED = 0,    // red Led    
  LED_B_GREEN,      // green Led         
  LED_A_BLU,        // blue Led
  NUM_LED
} ledIdx_e;

typedef enum
{
  LED_EVENT_OFF              = 0x00,        /* led status OFF                   */ 
  LED_EVENT_ON,                             /* led status ON                    */ 
  LED_EVENT_BLINK,                          /* led status Blinking              */ 
  LED_EVENT_OFF_ALL,                        /* led status all OFF               */ 
  LED_EVENT_BLINK_ALL,                      /* leds start Blinking at same time */ 
  LED_EVENT_SOFT_LIGHT                      /* leds on from 0 to on level slowly*/ 
} ledEvents_e;

/* ***************  definizione  gestione chain 2 sinapsi  ************** */
typedef enum
{
  TIMER_PMAX_T = 0,    // timer for P af Pmax    
  TIMER_SUSP_T,        // timer for forced suspension          
  TIMER_DIST_T,        // timer for detach power           
  NUM_SINAPSI_TIMER
} sinapsiIdx_e;

typedef enum
{
  SINAPSI_EVENT_START              = 0x00,        /* initilization                   */ 
  SINAPSI_EVENT_CHANGE,                           /* new info                        */ 
  SINAPSI_EVENT_PM_CONN,                          /* power managemen pronto          */ 
} sinapsiEvents_e;


/* REGISTRO EVSE_BOOT_EVENTS_RO --> Address = 0x0643, 1 word */
typedef enum
{
  REGULAR_BOOT                   = 0x0000,       /* Boot regolare SCU                     */
  FACTORY_RESET                  = 0x0001,       /* Reset di fabbrica SCU                 */ 
  ANOMALY_REBOOT                 = 0x0002,       /* Reboot anomalo SCU                    */
  REGULAR_REBOOT                 = 0x0003,       /* Reboot regolare SCU                   */
} bootReg_e;


/***************** definizione timer PWM CP    **************************/
#define TIM_CP_PWM                       TIM3
#define TIM_CP_PWM_CLK_ENABLE()          __HAL_RCC_TIM3_CLK_ENABLE();
#define CP_PWM_AF                        GPIO_AF2_TIM3
#define CP_PWM_TIM_CHx                   TIM_CHANNEL_1
#define CP_PWM_TIM_CHx_OUT_EN_MASK       ((uint32_t)0x00000001)
#define CP_PWM_TIM_CHx_OUT_LOW_MASK      ((uint32_t)0x00000002)

#define CP_PWM_PERIOD_VALUE              ((uint32_t)1000)
#define CP_PWM_MIN_DC                    ((uint32_t)50)   
/**
  * @brief PWM_CP --> PC6 TIM3 CH1
  */
#define CP_PWM_PIN                       PWM_CP_Pin
#define CP_PWM__GPIO_PORT                PWM_CP_GPIO_Port
#ifdef GD32F4xx
#define CP_PWM_GPIO_CLK_ENABLE()         __GPIOA_CLK_ENABLE()  
#define CP_PWM_GPIO_CLK_DISABLE()        __GPIOA_CLK_DISABLE()
#else
#define CP_PWM_GPIO_CLK_ENABLE()         __GPIOC_CLK_ENABLE()  
#define CP_PWM_GPIO_CLK_DISABLE()        __GPIOC_CLK_DISABLE()
#endif

/***************** definizione pin per 230Vac detector ***********************/
#define VAC230_DETECT_Pin                GPIO_PIN_8
#define VAC230_DETECT_GPIO_Port          GPIOC

/**
  * @brief WD V230 input pin  --> PC8 TIM8 CH3
  */
#define V230_WD_PIN                      VAC230_DET_Pin
#define V230_WD_GPIO_PORT                VAC230_DET_GPIO_Port
#define V230_WD_AF                       GPIO_AF3_TIM8
#define V230_WD_GPIO_CLK_ENABLE()        __GPIOC_CLK_ENABLE()  
#define V230_WD_GPIO_CLK_DISABLE()       __GPIOC_CLK_DISABLE()

/***************** definizione pin per motor driver STSPIN840 ***********************/
#define PHA_Pin                         GPIO_PIN_8
#define PHA_GPIO_Port                   GPIOD
#define PWMA_Pin                        GPIO_PIN_3
#define PWMA_GPIO_Port                  GPIOD

typedef enum
{
  UPG_LCD_ENABLED = ((uint8_t)0xAB),
  UPG_LCD_DWNL    = ((uint8_t)0xAC),
  UPG_LCD_DISABLED = ((uint8_t)0)
}upgLcd_e;

typedef enum
{
  LCD_NOT_PRESENT = ((uint8_t)FALSE),
  LCD_PRESENT     = ((uint8_t)TRUE),
  LCD_UNDEF       = ((uint8_t)0xFF)
}presenceLcd_e;

typedef enum
{
  CNNT_CHANGE_OFF =                ((uint8_t)0x00),
  CNNT_CHANGE_ONGOING =            ((uint8_t)0x01),
  CNNT_CHANGE_LCD_INIT_START =     ((uint8_t)0x02),
  CNNT_CHANGE_LCD_INIT_FINISHED =  ((uint8_t)0x03)
}lcdCntt_e;

typedef enum
{
  BATTERY_BACKUP_OFF  = ((uint8_t)0x00),
  BATTERY_BACKUP_ON   = ((uint8_t)0x01)
}batteryBackup_e;

typedef enum
{
  DISABLED  = ((uint8_t)0x00),
  ENABLED   = ((uint8_t)0x01)
}statusFlag_e;

#define MAX_NAME_LENGTH 14
#define MAX_SERIAL_LENGTH 8

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

/* station Operative mode  */
typedef enum
{
  MODE_MONO_PH_NO_PM =     ((uint8_t)0x00), /** measure: Etot, Time, Pist, L1 */
  MODE_TRI_PH_NO_PM,                        /** measure: Etot, Time, Pist, L1, L2, L3 */
  MODE_MONO_PH_PM,                          /** measure: Etot, Time, Pist, Pest, L1 */
  MODE_TRI_PH_PM_BAL,                       /** measure: Etot, Time, Pist, Pest, L1, L2, L3, Pest1, Pest2, Pest3 */
  MODE_TRI_PH_PM_UMBAL                      /** measure: Etot, Time, Pist, Pest, L1, L2, L3 */
}modePwr_e;

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

/* Socket function mode */
enum _sck_mode
{
  FREE = (uint8_t)0,
  PERSONAL,
  NET,
  OCPP
};
typedef enum _sck_mode modeFun_e;

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

/* Power manager parameters */
typedef __packed struct
{

  uint8_t  Mode;	
  uint8_t  Emeter;
  uint8_t  Unbal;
  uint8_t  Current;
  uint8_t  Error;	
  uint8_t  Multip;	
  uint8_t  Trange;	
  uint8_t  Dmax;	
  uint16_t Power;	
  
} Pmng_t;

/* Time limited charge */
typedef  __packed struct
{  
  
  uint8_t Mode;
  uint8_t Time;
  
} TCharge_t;

typedef  __packed struct
{
  
  uint8_t  Enabled;
  uint8_t  Visible;
  uint8_t  Pwd;
  
} Hidden_Menu_t;

typedef  __packed struct
{
  
  uint8_t  TimeZone;
  uint8_t  dst;
  uint8_t  TimeDstOffset;
  uint8_t  DstStatus;
  
} Time_Settings_t;

typedef  __packed struct 
{
  
  uint8_t  Enabled;
  uint8_t  Value;
  uint8_t  Delta;
  uint8_t  Hysteresis;
  
} Temp_Ctrl_t;

typedef __packed union
{
  struct
  {
  uint8_t Byte0;
  uint8_t Byte1;
  uint8_t Byte2;
  uint8_t Byte3;
  } Byte;                                /*!< Structure used for byte  access */
  uint32_t Word;                         /*!< Type      used for word access */
} controlByte_u;

/***************** definizione save config EEPROM info  ***********************/
#define   NUM_BACKUP_SAVE_EEPROM                ((uint16_t)3)

/* Transaction */
#define UID_SIZE ((uint16_t)10)
#define UID_TRANSACTION_FREE                    ((uint32_t)0x00000000)
#define UID_TRANSACTION_NET                     ((uint32_t)0x33333333)
#define UID_TRANSACTION_OCPP                    ((uint32_t)0x66666666)
#define UID_TRANSACTION_PERS_BY_CARD            ((uint32_t)0x88888888)
#define UID_TRANSACTION_PERS_BY_APP             ((uint32_t)0x99999999)
#define UID_TRANSACTION_DSO                     ((uint32_t)0xFFFFFFFF)
#define UNCHANGED_PARAMETER                     ((uint32_t)0xFFFFFFFF)
typedef __packed struct
{
  time_t    start;                // Unix Time -->  8 bytes
  time_t    stop;                 // Unix Time -->  8 bytes
  //uint32_t  duration;             // sec       -->  4 bytes 
  uint32_t  active_energy;        // Wh        -->  4 bytes
  uint8_t   uid[UID_SIZE];        // string    --> 10 bytes
                                  //    Tot.   --> 30 bytes   
}transaction_t;

/* Transactions Register*/
#define MAX_NUM_TRANSACTION_STORABLE            ((uint16_t)500) 
struct _register
{
  unsigned int num;
  transaction_t *transactions;
};
typedef struct _register sck_register_t;

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

struct _sck_state
{
  sck_statevalue_e value;
  sck_error_e error;
};
typedef struct _sck_state sck_state_t;

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
  sck_schedule_t scheds[MAX_SCHEDULATION_NUMBER];
};
typedef struct _socket socket_t;

typedef __packed struct
{
  char      fwVer[18];
  char      bootVer[6];
}fwBootVer_s;

typedef __packed union
{
  char          fwVer[24];
  fwBootVer_s   fwBootVer;
}fwInfoVersion_u;

/* Timeout range1: time between EV to EVSE  */
typedef __packed struct
{
  uint16_t  keyValue;
  uint16_t  timeRangeVal;
}toRange1_s;

#define   ACTIV_KEY_LENGTH              ((uint8_t)6)                            /* Activation key max number of bytes.                                          */              
#define   USER_PIN_LENGTH               ((uint8_t)6)                            /* User pin max number of bytes.                                                */        
#define   MAX_ROUTER_SSID_LENGTH        ((uint8_t)32)                           /* Router SSID max number of bytes                                              */
#define   MAX_ROUTER_PASS_LENGTH        ((uint8_t)64)                           /* Router password max number of bytes allowed                                  */
#define   INSTALLER_PIN_LENGTH          ((uint8_t)6)                            /* Installer pin max number of bytes                                            */        
#define   ITALY_PRODUCT_SN_LENGTH       ((uint8_t)10)                           /* Product serial number length.  100xxxxxx                                     */                                    
#define   PRODUCT_SN_LENGTH             ((uint8_t)16)                           /* Product serial number length.  100xxxxxx Sobem: 500044886/001                */                                    
#define   BOARD_SN_LENGTH               ((uint8_t)8)                            /* board serial number length.  00013440                                        */                                    
#define   PRODUCT_CODE_LENGTH           ((uint8_t)24)                           /* product code: 204.CA23B-T2T2W1 oppure 205.W36-B=205W236B                     */                                    
#define   FAKE_CODE_LENGTH              ((uint8_t)2 * 10)                       /* fake code: 204CA51FF                                                         */   

#define   WIFI_CONN_NAME_LEN            ((uint8_t)14)                           /* Length of connector name                                                     */

/* Flag to know if the initial configuration has finished */
#define   WIFI_ANTENNA_TEST_NOT_VALID  ((uint8_t)0xFF)                          /* Wifi antenna test for production (not a valid value)                         */
#define   WIFI_ANTENNA_TEST_PENDING    ((uint8_t)1)                             /* Wifi antenna test pending                                                    */
#define   WIFI_ANTENNA_TEST_DONE       ((uint8_t)2)                             /* Wifi antenna test done and pass succesfully                                  */
#define   WIFI_ANTENNA_TEST_TO_DO      ((uint8_t)3)                             /* Wifi antenna test for production has to done yet                             */
#define   ANTENNA_THRESHOLD            ((uint8_t)70)                            /* Wifi antenna threshold for test production                                   */

#define WIFI_AP_MIN_CHANNEL_ID  1
#define WIFI_AP_MAX_CHANNEL_ID  13

#define KEY_FOR_INFOSTATION_V0  ((char)0xA6)
#define KEY_FOR_INFOSTATION_V1  ((char)0xA8)      /* add checksum at the end  */
#define KEY_FOR_INFOSTATION_V2  ((char)0xAA)      /* renamed  productSn[10] in italyProductSn[10] and new productSn[16] at the end */
#define KEY_FOR_INFOSTATION_V3  ((char)0xAB)      /* added time range 1  at the end                                                */
#define KEY_FOR_INFOSTATION_V5  ((char)0xAE)      /* infoStation for RED: timeout for user collaudo and password for service and developper */
#define KEY_FOR_INFOSTATION_VX  KEY_FOR_INFOSTATION_V5

#define KEY_FOR_INFO_VALID      ((uint16_t)0xA8A6)

/* Station general parameters  */
typedef __packed struct 
{
  char                          name[MAX_NAME_LENGTH];
  /* Info */
  char                          serial[MAX_SERIAL_LENGTH];
  char                          firmware[24];
  sck_wiring_e                  wiring;
  int32_t                       max_current;
  int32_t                       max_currentSemp;
  energy_meter_e                emTypeInt;
  energy_meter_e                emTypeExt;
  evs_mode_en                   evs_mode;
  modePwr_e                     modePwr;
  uint8_t                       pmModeEn;
  power_management_unbal_en     pmUnbalEn;
  batteryBackup_e               batteryBackup;
  statusFlag_e                  v230MonFlag;
  /* Authorization */
  sck_auth_t                    auth;   
  /* Schedulation */
  sck_schedule_t                scheds[MAX_SCHEDULATION_NUMBER];
  /* Check */
  char                          key;  
  
  /* Added some parameters for Wi-Fi module to App communication */
  char                          reserved[ACTIV_KEY_LENGTH];                     /* Activation key inserted in user manual                       */
  char                          userPin[USER_PIN_LENGTH];                       /* User Pin selected by the app user                            */
  char                          routerSsid[MAX_ROUTER_SSID_LENGTH];             /* Router SSID of a home network                                */
  char                          routerPass[MAX_ROUTER_PASS_LENGTH];             /* Router Password of a home network                            */
  char                          installerPin[INSTALLER_PIN_LENGTH];             /* Installer Pin memorized in Eeprom                            */
  uint8_t                       socketActivatedFlag;                            /* Flag to check if the socket has been already initialized     */
  bootReg_e                     bootEvent;                                      /* Enum for boot SCU event                                      */                
  char                          italyProductSn[ITALY_PRODUCT_SN_LENGTH];        /* Product serial number 100xxxxxx                              */    
  uint16_t                      maxRandomDelay;                                 /* Max random delay value set for UK market (1800 in future)    */
  uint8_t                       esitoUpdateFw;                                  /* 1: esito update FW OK, 0: altro                              */
  uint8_t                       antennaPresence;                                /* 1 se antenna presente da test produzione, 0 non presente     */
  /* Added some parameters for debug  */
  uint8_t                       sinapsiRS485Errors;                             /* counter error communication to SINAPSI                       */
  /* Added some parameters for restore wifi module to factory parameter   */
  uint8_t                       keyForRestoreModule;                            /* Key for restore module 0xA9                                  */
  uint8_t                       restoreModule;                                  /* restore default parameter wifi module when 1                 */
  char                          productCode[PRODUCT_CODE_LENGTH];               /* Product Code: 204.CA23B-T2T2W1                               */    
  char                          fakeProductCode[FAKE_CODE_LENGTH];              /* fake Code: 204CA51FF                                         */    
  uint32_t                      sessionIdNum;                                   /* session identifier number                                    */
  uint8_t                       channelId;
  char                          productSn[PRODUCT_SN_LENGTH];                   /* Product serial number 100xxxxxx Sobem: 500044886/001         */  
  toRange1_s                    toRange1;                                       /* structure for store timeout from EV to EVSE                  */
  uint32_t                      passWebServiceHash;                             /* hash della password del web server per assistenza            */
  uint32_t                      passWebWiFiHash;                                /* hash della password per accesso wifi                         */
  uint32_t                      startTimeWebCollaudo;                           /* istante di prima attivazione web server al collaudo          */
  uint16_t                      confDataAndPassStatus;                          /* flag = 0xBBAA con tutti i dati di configurazione presenti    */
  uint32_t                      checksum;                                       /* checksum It is  the sum of all structure byte                */
  /*                    From there parameters comes from original eeprom_param_array[]                      */
  uint8_t                       socketEnable;                                   /* ex SOCKET_ENABLE_EADD    */
  uint8_t                       batteryConfig;                                  /* ex BATTERY_CONFIG_EADD   */
  uint8_t                       default_Lang;                                   /* ex LANG_DEFAULT_EADD     */
  uint8_t                       rs485Address;                                   /* ex RS485_ADD_EADD        */
  uint8_t                       rtcValid;                                       /* ex RTC_VALID_EADD        */
  uint8_t                       socketType;                                     /* ex SOCKET_TYPE_EADD      */
  controlByte_u                 controlByte;                                    /* ex CONTROL_BYTE0_EADD - CONTROL_BYTE1_EADD - CONTROL_BYTE2_EADD - CONTROL_BYTE3_EADD */
  uint8_t                       actuators;                                      /* ex ACTUATORS_EADD        */
  uint8_t                       blockDir;                                       /* ex BLOCK_DIR_EADD        */
  uint8_t                       persUidNum;                                     /* ex PERS_UIDNUM_EADD      */
  uint8_t                       persMaster;                                     /* ex PERS_MASTER_EADD      */ 
  controlByte_u                 LangConfig;                                     /* ex LANG_CONFIG0_EADD - LANG_CONFIG1_EADD - LANG_CONFIG2_EADD - LANG_CONFIG3_EADD */
  uint32_t                      TotalEnergy;                                    /* ex TOT_ENERGY0_EADD      */
  uint8_t                       StripLedType;                                   /* ex STRIP_LED_TYPE_EADD   */
  uint8_t                       LcdType;                                        /* ex LCD_TYPE_EADD         */
  uint16_t                      Energy_limit;                                   /* ex ENRG_LIMIT_EADD       */
  uint8_t                       Sinapsi_Installed;                              /* ex SINAPSI_INST_EADD     */
  uint8_t                       EmeterScu_Int;                                  /* ex EMETER_SCU_INT_EADD   */
  uint8_t                       Operative_mode;                                 /* ex OPERATIVE_MODE_EADD   */
  uint8_t                       semFlagControl;                                 /* ex SEM_FLAGS_CTRL_EADD   */
  uint8_t                       StationNominalPower;                            /* ex STATION_NOM_PWR_EADD  */
  uint8_t                       connectorNumber;                                /* ex CONNECTOR_NUMBER_EADD */
  uint8_t                       postSuspensionTime;                             /* ex POST_SUSP_TIME_EADD   */
  Pmng_t                        Pmng;                                           /* ex PMNG_MODE_EADD - PMNG_EMETER_EADD - PMNG_PWRLSB_EADD - PMNG_ERROR_EADD - PMNG_CURRENT_EADD - PMNG_MULTIP_EADD - PMNG_DMAX_EADD - PMNG_TRANGE_EADD */
  TCharge_t                     TCharge;                                        /* ex TCHARGE_MODE_EADD - TCHARGE_TIME_EADD */
  Hidden_Menu_t                 Hidden_Menu;                                    /* ex HIDDEN_MENU_VIS_EADD - HIDDEN_MENU_ENB_EADD */
  Time_Settings_t               Time_Settings;                                  /* ex TIME_ZONE_EADD - DST_EADD - TIME_DST_OFFSET_EADD - DST_STATUS_EADD */
  Temp_Ctrl_t                   Temp_Ctrl;                                      /* ex TEMP_CTRL_ENB_EADD - TEMP_CTRL_VAL_EADD - TEMP_DELTA_EADD - TEMP_HYSTERESIS_EADD */
  
}   infoStation_t;

/* Socket */
typedef struct 
{
  char name[MAX_NAME_LENGTH];
  /* Info */
  char serial[16];
  char firmware[24];
  uint8_t wiring;
  uint16_t mt_int;
  uint16_t mt_ext;
  uint8_t rfid;
  int32_t max_current;
  int32_t max_power;
  /* State */
  uint8_t id;
  uint16_t state;
  modeFun_e modeFun;
  modePwr_e modePwr;
} infoBle_t;

typedef __packed union
{
  uint32_t  w32;
  uint8_t   w32Byte[4];
}word32_u;

/* RW modbus area for Sinapsi - RSE */
typedef __packed struct
{
  uint16_t        iomCtrl;                   // IOM_CTRL,          83
  uint32_t        m2PaiiTs;                  // M2_PAII_TS,      
  uint16_t        m2Paii;                    // M2_PAII,           85
  uint32_t        m2PcTs;                    // M2_PC_TS,          
  uint16_t        m2Pc;                      // M2_PC,           
  uint32_t        m1PcTs;                    // M1_PC_TS,        
  uint16_t        m1Pc;                      // M1_PC,           
  uint32_t        m1PdTs;                    // M1_PD_TS,          90
  uint16_t        m1Pd;                      // M1_PD,           
  uint32_t        m1PapiTs;                  // M1_PAPI_TS,      
  uint16_t        m1Papi;                    // M1_PAPI,         
  uint32_t        m1PaiiTs;                  // M1_PAII_TS,      
  uint16_t        m1Paii;                    // M1_PAII,           95
  uint32_t        m1ResDistTs;               // M1_RES_DIST_TS,    
  uint16_t        m1TimeResDist;             // M1_TIME_RES_DIST,
  uint32_t        m1PfoTs;                   // M1_PFO_TS,       
  uint16_t        m1Pfo;                     // M1_PFO,          
  uint16_t        m1PmrTlim;                 // M1_PMRTLIM,        100
  uint32_t        m1PmrTlimTs;               // M1_PMRTLIM_TS,  
  uint32_t        m1SospRicTlimTs;           // M1_SOSP_RIC_TLIM,
  uint32_t        m1UtSync;                  // M1_UT_SYNC,        103
  uint16_t        setConfig;                 // SET_CONFIG_MODE,            
} rseSetReg_st;                                    

/* RW modbus area for Sinapsi - RSE */
typedef __packed struct
{
  uint16_t        m2Paii;                    // Potenza immessa istantanea  in Watt sul contatore M2 (votovoltaico) In assenza vale 0
  uint16_t        m2Pc;                      // Potenza contrattuale  in Watt sul contatore M2 (votovoltaico) In assenza vale 0           
  uint16_t        m1Pc;                      // Potenza contrattuale  in Watt           
  uint16_t        m1Pd;                      // Potenza disponibile  in Watt           
  int16_t         m1Papi;                    // Potenza istantanea prelevata in Watt         
  uint16_t        m1Paii;                    // potenza immessa Vale 0 in assenza di impianto fotovoltaico
  uint16_t        m1TimeResDistBool;         // vale 0 o 1: se = 1 distacco per esupero attivo e previsto tra 10 sec Se = 0 non attivo
  uint16_t        m1Pfo;                     // nuova fascia oraria (valida se != 0)          
  uint16_t        m1PmrTlim;                 // nuovo limite di potenza (valido se != 0 Se = 0 vale il limite contrattuale o PD)
  uint32_t        m1SospRicTlimBool;         // vale 0 o 1: se 1 è richiesto lo stato di sospensione permanente 0 stato di sospensione non più attivo
} sinapsiSetReg_st;                                    


typedef enum
{
  ACCOUNT_DEVELOPER = 0,
  ACCOUNT_COLLAUDO,
  ACCOUNT_SERVICE
} account_e;

typedef enum
{
  ACCESS_DENIED = 0,
  ACCESS_ALLOWED,
  ACCESS_CHANGE_PASS
} access_e;

/* Structure used to manage the different area in EEPROM involved in configuration parameter */
typedef struct {
  
  uint32_t               confParStartAddress;
  uint32_t               confParLen;
  
} confPar_st;


/* Structure used to manage the different area in EEPROM involved in configuration parameter */
typedef struct {
  uint16_t               confParIdLogicScu;
  confPar_st             confParEepromArray;
  confPar_st             confParInfoStation;
  confPar_st             confParBackupInfoStation;
  confPar_st             confParSerialCode;
  confPar_st             confParSerialFactoryCode;
  uint32_t               confParCkecksumControl;
} allConfPar_st;

/* Structure used to manage the different area in EEPROM involved in configuration parameter */
typedef struct {
  uint16_t               idLogicScu;
  uint8_t                confEepromParamArray[EEPROM_PARAM_NUM];
  infoStation_t          confInfoStation;
  infoStation_t          confBackupInfoStation;
  uint8_t                confSerialCode[END_SN_EE_ADDRES - PRD_CODE_EE_ADDRES + 1];
  uint8_t                confSerialFactoryCode[PRODUCT_SN_LENGTH + PRODUCT_CODE_LENGTH + FAKE_CODE_LENGTH];
  uint32_t               confRtcBackup[NUM_BACKUP_SAVE_EEPROM];
  uint32_t               ckecksumControl;
} blockConfPar_st;

typedef struct {
  blockConfPar_st        blockConfPar[SCU_NUM];
  uint32_t               globalCkecksumControl;
}areaConfPar_st;

/**
  * @brief  Check if DMA Mode is enabled for reception
  * @rmtoll CR3          DMAR          LL_USART_IsEnabledDMAReq_RX
  * @param  USARTx USART Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t USART_IsEnabledDMAReq_RX(USART_TypeDef *USARTx)
{
  return ((READ_BIT(USARTx->CR3, USART_CR3_DMAR) == (USART_CR3_DMAR)) ? 1UL : 0UL);
}


/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/
extern struct DataAndTime_t GlobalDT;

extern uint8_t  *pArea;

/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
/* primitive per gestione visualizzazioni LCD e Led RGB  */
void            DispUpdate              (uint8_t *line1, uint8_t align1, uint8_t *line2, uint8_t align2);
void            stopPWMlcd2x20          (void);
void            restartPWMlcd2x20       (void);
void            setLed                  (ledIdx_e led, ledEvents_e ledEvent, uint16_t blinkTime, uint8_t percentValue);
void            setPWM_Ledx             (ledIdx_e led, uint8_t percentValue);
void            setUpgradeLcd           (upgLcd_e status);
upgLcd_e        getUpgradeLcd           (void);
void            cnttStatusChange        (void);
void            setLedSoft              (ledIdx_e led, ledEvents_e ledEvent, uint16_t blinkTime, uint8_t percentValue, uint16_t iCurr, uint16_t iMax);
uint32_t        getLcdHwMng             (void);
void            setDateTimeWithTimeZone (struct DataAndTime_t* pLocDateTime);
void            timeOnLcd               (uint8_t lcdTime);
void            setLcdPresence          (presenceLcd_e status);
presenceLcd_e   getLcdPresence          (void);
void            setLedForReservedMode   (void);

/* primitive per gestione segnali digitali di input / output */
GPIO_PinState   getInputState       (dIn_TypeDef pinId);
void            setOutputState      (dOut_TypeDef pinId, GPIO_PinState PinState);

void            mirror_contact_check_enable (void);
void            mirror_contact_check_disable(void);

/* primitive per gestione segnale CP relativo alla presa  */
void            startPwmOnCP        (uint16_t dc);
void            stopPwmOnCP         (void);
void            init_CP_PWM         (void);
void            stopPwmOnLevel      (GPIO_PinState PinState);
uint16_t        getPwmOnCP          (void);


/* primitive per gestione segnali analogici  */
uint16_t        getADCmV            (adcIn_e ixADC);
rotaryPos_e     getRotarySwitchPos  (void);
uint32_t        getTAcurrent        (void);
void            sendMsgStartTa      (void);
void            sendMsgStopTa       (void);
int16_t         getUpTemp           (void);

/* primitive per gestione segnali relativi alla condizione di assenza rete  */
void            setFlagV230               (uint32_t status);
uint32_t        getFlagV230               (void);
void            setVac230PinAsIntrpt      (void);
void            SystemClock_PowerConfig   (statusFlag_e statusIn);
void            SystemClock_LSE_Enable    (void);
void            SystemClock_LSI_Enable    (void);
void            sbcPowerStatus            (statusFlag_e status);
void            sbcPowerControl           (statusFlag_e status);
void            setFirstLevelPwdnState    (void);
void            setSecondLevelPwdnState   (void);
void            setFlagV230Msg            (uint32_t statusIn);
uint32_t        getFlagV230Msg            (void);
void            setEmModelReg             (uint32_t statusIn, emEnum_e position);
uint32_t        getEmModelReg             (emEnum_e position);
statusFlag_e    getStationV230FlagStatus  (void);
uint16_t        vinPwrValue               (void);
uint8_t         checkVbusFlag             (void);
void            powerDownPin              (statusFlag_e statusIn);
uint16_t        getCounterZC              (void);


/* primitive per gestione memorizzazione e lettura dati EEPROM  */
unsigned char   ReadFromEeprom            (unsigned short Address, unsigned char *Buffer, unsigned short Length);
unsigned char   WriteOnEeprom             (unsigned short Address, unsigned char *Buffer, unsigned short Length);
void            restoreFactoryDefault     (void);

/* primitive per get e salvataggio autorizzazione socket Bluetooth */
sck_auth_t*     getAuthorization    (void);
void            saveAuthorization   (char *user, char *pass);

/* primitive per get e salvataggio schedulazione socket Bluetooth */
sck_schedule_t *  getSchedulationFromMemory (void);
void              saveSchedulation          (sck_schedule_t *schedulation);

/* primitive per gestione motore blocco / sblocco presa  */
void            setOutBL1_M         (void);
void            setOutBL1_P         (void);
void            brakePhase          (void);
uint8_t         gpio1IoExpWrite     (GPIO_PinState pinVal);
GPIO_PinState   gpio1IoExpRead      (void);
GPIO_PinState   ioExpRead           (uint8_t pinMask);
uint8_t         gpio0IoExpWrite     (GPIO_PinState pinVal);

/* primitive per gestione delle sessioni di ricarica   */
void            stopNewTransaction            (uint32_t activeEnergy);
void            startNewTransaction           (uint32_t uid);
void            setTransactionParam           (sck_measures_t *pMeasureSck, evs_state_en evStatus, uint8_t update, suspending_en suspending_type);
void            sendEventToSemMng             (sbcSemEvent_e eventMsg , uint16_t address); 


/* primitive per gestione informazioni della stazione  */
uint8_t*        getFwVer                        (void);
void            setGeneralStationParameters     (uint8_t Type);
char *          getStationName                  (void);
unsigned char   setStationName                  (char* stName, int length);
char*           getStationSerialNumber          (void);
sck_wiring_e    getStationSocketType            (void);
energy_meter_e  getStationEmType                (emEnum_e emPos);
energy_meter_e  getStationEmTypeInt             (void);
energy_meter_e  getStationEmTypeExt             (void);
int32_t         getStationMaxCurrentT           (void);
int32_t         getStationMaxCurrentS           (void);
void            setStationEmType                (energy_meter_e emType,  emEnum_e pos, EmeterType_en gsyEmType, EmeterType_en webEmType);
modeFun_e       getStationModeWorking           (void);
modePwr_e       getStationPowerModeWorking      (void);
sck_error_e     getErrorStateCoding             (void);
batteryBackup_e getStationBatteryBackupMode     (void);
void            setStationStatusInModbusMapApp  (uint16_t state);
uint16_t        getStationStatusFromModbusMap   ();
void            setStationEventInModbusMap      (uint16_t event); 
uint16_t        getStationStatusFromModbusMap   ();
rseSetReg_st*   getRWSinapsiInfo                (void);
uint16_t        cpuIdCheksum16                  (void);
uint8_t         getStationId                    (void);
uint32_t        getAddrSetting                  (void);
void            setFlagHwInfo                   (uint32_t statusIn, uint32_t mask);
uint8_t         getSW1flagAndReset              (void);
uint8_t         getSinapsiNewEnable             (void);
char*           getStationProductCodeString     (void);
void            sendChangeStatusToSemMng        (evs_modbus_state_en evStatus, uint8_t evsModbusState_force); 
void            init_i_measures                 (sck_measures_t *measures);

#ifdef COLLAUDO_PEN
/* primitive per gestione relè gig di collaudo PEN  */
void            setRelaysGigPen     (uint16_t rSetting);
#endif
void            setPenFilteringStatus       (statusFlag_e penSt);
void            startIoExpPenFiltering      (void);
void            ethMotorPowerOn             (void);

void init_measures(sck_measures_t *measures);
void initModbusRegisters(void);

extern 			    sck_measures_t               measureSck;
extern __no_init char                        OutOfPowerDownAt15V;

/* primitive per gestione download da GSY   */
void            resetCodeInfo                 (void);
xQueueHandle    getScuGsyDwldQueueHandle      (void);
void            setCodeLen                    (uint32_t codeLength);
uint8_t         getStatusDwnl                 (void);
void            lowLevelBrChange              (USART_TypeDef * pUart, uint32_t brr, uint8_t intf);
uint8_t         sbcPresence                   (void);

/* primitive per gestione informazioni SINAPSI sul contatore ENEL M1  */
uint32_t        getPotenzaContrattuale        (void);
uint32_t        getPotenzaDisponibile         (void);
uint32_t        getPotenzaPrelevata           (void);
uint32_t        getTempoDistacco              (void);
uint8_t         enaDurataPmaxT                (uint32_t* pPmaxT, uint32_t* pDurataPmax);
uint8_t         enaSospensioneTempoDistacco   (void);

void              setPMreadyForIom2G          (void);
sinapsiSetReg_st* getIom2Ginfo                (void);
void              startSinapsiIom2G           (void);
void              newSinapsiIom2GData         (void);
void              resetSinapsiIOM2CtrlReg     (void);
uint32_t          checkSinapsiTimer           (sinapsiIdx_e tId);
uint16_t          getSinapsiStatusActivation  (void);
statusFlag_e      getSinapsiEepromEn          (void);

/* primite per gestione informazione modulo Wi-Fi + App */
unsigned char   setUserPin                    (char* pin, uint8_t length);
char*           getUserPin                    (void);
unsigned char   setRouterSsid                 (char* ssid, uint8_t length);
char*           getRouterSsid                 (void);
unsigned char   setRouterPass                 (char* pass, uint8_t length);
char*           getRouterPass                 (void);
uint8_t         resetRouterSsid               (void);
uint8_t         resetRouterPass               (void);
uint8_t         setLabEmRouterSsid            (void);
void            setFastLabEmRouterSsid        (void);
unsigned char   setInstallerPin               (char* pass, uint8_t length);
char*           getInstallerPin               (void);
unsigned char   setActivationFlag             (uint8_t flag);
uint8_t         getActivationFlag             (void);
void            scheduleReceivedFromWifi      ();
unsigned char   setBootEvent                  (bootReg_e boot);
bootReg_e       getBootEvent                  (void);
unsigned char   setScuSerialNumberEeprom      (char* key, char* keyString);
unsigned char   setProductSerialNumberEeprom  (char* key, uint8_t length, uint8_t setAll);
void            resetEEpromAll                (void);
unsigned char   setScuHardwareVersionEeprom   (char* hwVerStr);
char*           getScuHWverFromEeprom         (void);
char            getCodeHwVersion              (void); 
char*           getProductSerialNumberEeprom  (void);
unsigned char   setMaxRandomDelay             (uint16_t delay);
char*           getMaxRandomDelay             (void);
unsigned char   setEsitoUpdateFw              (uint8_t esitoUpdateFw);
uint8_t         getEsitoUpdateFw              (void);
//void            updateStationEmType           (EmeterType_en emType,  emEnum_e pos);
//void            parserEmModbusToWeb           (energy_meter_e emCode);
wifiSbcMode_e   getWifiSbcEnv                 (void);
void            setWifiSbcEnv                 (wifiSbcMode_e status);
uint8_t         saveWifiAccessPointChannelId  (uint8_t id);
uint8_t         getWifiAccessPointChannelId   (void);


void            setFastBridge                 (statusFlag_e fbStatus);
statusFlag_e    getFastBridge                 (void);
uint32_t        getCounterSlaveDwnl           (void);
void            setCounterSlaveDwnl           (uint32_t value);
void            resetFlagV230Msg              (void);
uint8_t         getLogicalBlockPin            (void);

void            setCurrentTimestamp           ();

uint8_t         getStationSinapsiRS485Error   (void);
void            incStationSinapsiRS485Error   (void);
void            resetStationSinapsiRS485Error (void);

void            setScameActEnergyEepromOffset (uint32_t currActEnergy, uint8_t flagRW, uint8_t flagCharging, uint16_t* pEmFaul);
uint32_t        getScameActEnergyOffset       (void);
void            resetScameActEnergyE0Key      (void);
void            checkStartFwDwld              (char* pSrc, uint16_t lenRegByte, uint16_t index, uint8_t scuPhyAddr);
void            BKP_SRAM_UnixTimestamp_Save   (uint32_t Unixtimestamp);
char*           getStationFakeCodeCodeString  (void);
unsigned char   setStationFakeProductCode     (char* pCode, uint8_t length);
unsigned char   setStationProductCode         (char* pCode, uint8_t length);
void            setUIDinfoROmap               (void);
void            saveSessionId                 (uint32_t sessId);
uint32_t        recoverySessionId             (void);
void            sendDownloadMsg               (messageEv_e eventMsg);
unsigned char   setNumSocketFromFakeCode      (uint8_t num);
void            setParamFromAssignedAddr      (void);
void            forceSendEventToSemMng        (sbcSemEvent_e eventMsg , uint16_t address, uint16_t phyAddr); 
void            setScameInitActEnergy         (void);
void            SRAM_Check_DEFAULT_of_Code    (void);
void            SRAM_Param_DEFAULT_Set        (char *SRAM_ptr, uint8_t DEF_Value, uint8_t nChar);

uint8_t         memCpyInfoSt                  (uint8_t* pInfoSt, uint8_t* pData, uint16_t lenField);
uint32_t        findInfoStationCheksum        (uint8_t* pInfoSt);
void            saveTimeoutRange1             (uint16_t toVal);
uint8_t         Get_RST_Origin                (void);
uint32_t        getStationStartTimeWebCollaudo(void);
void            saveStartTimeWebCollaudo      (uint32_t unixTimeSession);
access_e        checkSviluper                 (char* pPassString, uint16_t length);
access_e        checkServicePassword          (char* pPassString, uint16_t length);
uint8_t         allConfDataAndPassword        (void);
uint8_t         setSerialReceivedFlag         (void);
void            setFlagForNvic                (void);
void            setRplOptionByte              (uint8_t rdpLevel);
void            BCD_to_PackedBCD              (uint8_t *pPackedBCD, uint8_t *pBCD, uint8_t nBCD);
void            PackedBCD_to_BCD              (uint8_t *pBCD, uint8_t *pPackedBCD, uint8_t nBCD);

#ifdef SIMULATORE_CARICHI
/* gestione per debug "carico elettronico"  */
void            setReleCarico                 (uint8_t* pMsg);
#endif

#define   RFID_TASK_SUSPENDED          ((uint8_t)0)                             /* RFID task is suspended by personal by app                                    */
#define   RFID_TASK_ACTIVE             ((uint8_t)1)                             /* RFID task is active                                                          */ 

extern          osThreadId_t          RfidTaskHandle;
extern          uint8_t               rfidTaskStatus;
extern          infoStation_t         infoStation;

#endif //  __WRAPPER_H


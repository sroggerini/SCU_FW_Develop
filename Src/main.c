 /**
* @file        tarManager.c
*
* @brief       main program for SCU  - Implementation -
*
* @author      Nick
*
* @riskClass   C 
*
* @moduleID  
*
* @vcsInfo
*     $Id: main.c 775 2025-07-14 11:32:59Z npiergi $
*
*     $Revision: 775 $
*
*     $Author: npiergi $
*
*     $Date: 2025-07-14 13:32:59 +0200 (lun, 14 lug 2025) $
*
*
* @copyright
*       Copyright (C) 2020 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

/************************************************************
 * Include
 ************************************************************/
#include "main.h"
#include "cmsis_os.h"
#include "prot_OnUsart.h"
#include "telnet.h"
#include "displayPin.h"
#include "ftps.h"
#include "InputsMng.h"
#include "ioExp.h"

/* includes for station manager */
#include "adcTask.h"
#include "ethInitTask.h"

#include "BlockMng.h"
#include "ContactMng.h"
#include "EnergyMng.h"
#include "EvsMng.h"
#include "ExtInpMng.h"
#include "EvsTimeMng.h"
#include "hts.h"
#include "LcdMng.h"
#include "Lcd.h"
#include "PilotMng.h"
#include "PwmMng.h"
#include "RfidMng.h"
#include "PersMng.h"
#include "monitorMng.h"

#include "eeprom.h"
#include "scuMdb.h"

#include "sbcGsy.h"
#include "TCP_socket.h"
#include "sinapsi.h"
#include "scheduleMng.h"
#include "AutotestMng.h"
#include "diffRiarm.h"
#include "hts.h"
#include "rtcApi.h"
#ifdef HW_MP28947
#include "metroTask.h"
#else
#include "lwip.h"
#endif
#ifndef GD32F4xx
#include "stm32h5xx_ll_rcc_legacy.h"
#endif
#include "transaction_register.h"
#include "httpserver-socket.h"
#include "wrapper.h"
#include "crc.h"
#ifndef HW_MP28947  
#include "homeplugdev.h"
#endif   

/*
***********************************SCAME**************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/

#define BIN_CANC_IN_MAIN  1
//#define REDUCE_CLOCK      1
#define WAKEUP_ON_TIMEOUT 1
//#define SPEAD_SPECTRUM    1 
#define MPU_ON_INFOSTATION 1

#define   MIN_VIN_TO_WORK     ((uint16_t)10000)   /* min Vin value to start the program */
#define   VIN_ATTENUATION     ((uint16_t)16)      /* Vin has in imput a 10 / 160 resistor partitor  */
#define   MIN_VIN_TETHERED    ((uint16_t)16000)   /* min Vin value in thethered to have coeherent state  */
#define   MIN_VIN_TO_RESTART  ((uint16_t)20000)   /* min Vin value to restart the program after powerdown */

/*
***********************************SCAME**************************************
**                                                                          **
**                            Local Const                                   **
**                                                                          **
****************************************************************************** 
*/ 

/*
***********************************SCAME**************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 


/*
***********************************SCAME**************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 
TIM_HandleTypeDef htimV230;


uint32_t calpValue, calmValue;

uint8_t cntDownLowPower, monV230Mng, hwVersion;

// Variabile globale configurazione di rete
NetworkConfiguration_t	NetworkConfiguration;

osSemaphoreId SPI1_semaphore;

static TIM_HandleTypeDef  htim6;

/** end declaration for demo voice  */

/** variable for current socket motor management ***/
static imState_e          imState;

static  emrgStCond_e           statoEmerg;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 4   
};

/* Definitions for debug uart Task  */
osThreadId_t protoUartTaskHandle;
const osThreadAttr_t protoUartTask_attributes = {
  .name = "PROTO_ON_UART_TASK",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 3  
};

/* Definitions for telnet Task  */
osThreadId_t fatMngTaskHandle;
const osThreadAttr_t fatMngTask_attributes = {
  .name = "FATMNG_TASK",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 5  
};

/* Definitions for digital input Task  */
osThreadId_t inputMngTaskHandle;
const osThreadAttr_t inputMngTask_attributes = {
  .name = "INPUTMNG_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 5    
};

/* Definitions for analog ADC   */
osThreadId_t adcMngTaskHandle;
const osThreadAttr_t adcMngTask_attributes = {
  .name = "ADC_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 3   
};

/* Definitions for ioexpander  Task  */
osThreadId_t ioexpMngTaskHandle;
const osThreadAttr_t ioexpMngTask_attributes = {
  .name = "IOEXP_MNG_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 4  
};

/* Definitions for Led RGB   Task  */
osThreadId_t ledMngTaskHandle;
const osThreadAttr_t ledMngTask_attributes = {
  .name = "LED_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 2  
};

/* Definitions for ethernet jump Task  */
osThreadId_t ethInitTaskHandle;
const osThreadAttr_t ethInitTask_attributes = {
  .name = "ETH_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 10   
};

/* Definitions for EVS state machine Task  */
osThreadId_t evsTaskHandle;
const osThreadAttr_t EvsMngTask_attributes = {
  .name = "EVS_TASK",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 6   
};

/* Definitions for socket block manager Task  */
osThreadId_t blockTaskHandle;
const osThreadAttr_t BlockMngTask_attributes = {
  .name = "BLOCK_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 3
};

/* Definitions for lcd manager Task  */
osThreadId_t lcdTaskHandle;
const osThreadAttr_t LcdMngTask_attributes = {
  .name = "LCD_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 6  
};

/* Definitions for plug manager Task  */
osThreadId_t PilotTaskHandle;
const osThreadAttr_t PilotMngTask_attributes = {
  .name = "PILOT_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 3  
};

/* Definitions for pwm manager Task  */
osThreadId_t PwmTaskHandle;
const osThreadAttr_t PwmMngTask_attributes = {
  .name = "PWM_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 3
};

/* Definitions for contact manager Task  */
osThreadId_t ContactTaskHandle;
const osThreadAttr_t ContactMngTask_attributes = {
  .name = "CONTACT_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 3
};

/* Definitions for contact manager Task  */
osThreadId_t ExtInpTaskHandle;
const osThreadAttr_t ExtInpMngTask_attributes = {
  .name = "EXTINP_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 4    
};

/* Definitions for evstime manager Task  */
osThreadId_t EvsTimeTaskHandle;
const osThreadAttr_t EvsTimeMngTask_attributes = {
  .name = "EVSTIME_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 2     
};

/* Definitions for eeprom manager Task  */
osThreadId_t EEpromTaskHandle;
const osThreadAttr_t EEpromMngTask_attributes = {
  .name = "EEPROM_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 8
};

/* Definitions for HTS manager Task  */
osThreadId_t HTSTaskHandle;
const osThreadAttr_t HTSMngTask_attributes = {
  .name = "HTS_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 5
};

/* Definitions for periodic manager Task  */
osThreadId_t periodicTaskHandle;
const osThreadAttr_t periodicMngTask_attributes = {
  .name = "PERIOD_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 2
};


/* Definitions for rfid manager Task  */
extern osThreadId_t RfidTaskHandle;
const osThreadAttr_t RfidMngTask_attributes = {
  .name = "RFID_TASK",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 4    
};

/* Definitions for rfid watchdog Task  */
extern osThreadId_t RfidWdgTaskHandle;
const osThreadAttr_t RfidWdgMngTask_attributes = {
  .name = "RFID_WDG_TASK",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 2     
};

/* Definitions for pers manager Task  */
osThreadId_t PersTaskHandle;
const osThreadAttr_t PersMngTask_attributes = {
  .name = "PERS_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 4
};

/* Definitions for energy manager Task  */
osThreadId_t EnergyTaskHandle;
const osThreadAttr_t EnergyMngTask_attributes = {
  .name = "ENERGY_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 7    
};

/* Definitions for activities monitor  Task  */
osThreadId_t monMngTaskHandle;
const osThreadAttr_t monMngTask_attributes = {
  .name = "MON_MNG_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 3    // Original value --> 3 - Maximum call chain --> 2
};

/* Definitions for SINAPSI Task  */
osThreadId_t sinapsiTaskHandle;
const osThreadAttr_t sinapsiTask_attributes = { 
  .name = "SINAPSI_TASK",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 3   
};

/* Definitions Schedulation Task  */
osThreadId_t schedTaskHandle;
const osThreadAttr_t schedTask_attributes = {
  .name = "SCHED_MNG_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 4          
};

/* Definitions for lcd low level manager Task  */
osThreadId_t lowLevelLcdTaskHandle;
const osThreadAttr_t lowLevelLcdMngTask_attributes = {
  .name = "HW_LCD_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 3            
};

/* Definitions Schedulation Task  */
osThreadId_t AutotestTaskHandle;
const osThreadAttr_t AutotestTask_attributes = {
  .name = "AUTOTEST_TASK",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 2            // Original value --> 4 - Maximum call chain --> 2
};

/* Definitions for activities diff retriggerable  Task  */
osThreadId_t diffRiarmTaskHandle;
const osThreadAttr_t diffRiarmTask_attributes = {
  .name = "DIFF_RIARM_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 3            // Original value --> 2 - Maximum call chain --> 3
};

/* Definitions for activities for ISO15118 Task  */
osThreadId_t ISO15118TaskHandle;
const osThreadAttr_t ISO15118Task_attributes = {
  .name = "ISO15118_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 3    
};

///* Definitions for Bluetooth Task  */
//osThreadId_t blueMngTaskHandle;
//const osThreadAttr_t blueMngTask_attributes = {
//  .name = "BLUETOOTH_TASK",
//  .priority = (osPriority_t) osPriorityBelowNormal,
//  .stack_size = configMINIMAL_STACK_SIZE * 9
//};


osSemaphoreId FlashSemaphoreHandle;
osSemaphoreId FatSemaphoreHandle;

// Signal for end initialization 
uint8_t  initUartDBG_End, initUart2End, initUart1End;

uint8_t	      RefreshWatchDog, enterInStop, immediateRestart, powerOnReset, flagV230OkFiltered, restartOnRemove;
infoV230_st   infoV230;

GPIO_InitTypeDef GPIO_InitStruct = {0}; 

static  uint32_t      flag230;
/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/

extern TIM_HandleTypeDef        htimV230;
extern ADC_HandleTypeDef        AdcVINHandle;
extern __no_init presenceLcd_e  lcdPresent;
extern uint8_t                  resetOrigin [4][8];
extern sFLASH_Info              sFLASH_Information;

/*
***********************************SCAME**************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
#ifdef BIN_CANC_IN_MAIN
static void removeFwBinFile (void);
#endif
#ifdef ELIMINATA
static uint8_t                  checkV230FromHwPin  (void);
#endif
static uint8_t                  checkStartV230Mng   (void);
static HAL_StatusTypeDef        TIM_V230_Config     (void);
#ifdef SPEAD_SPECTRUM
static void     spreadSpectrumConfig(void);
#endif
static void MPU_Config(void);
static void MPU_AccessPermConfig(void);
 
void StartDefaultTask(void *argument);
void LCD_IO_Init(LcdInitState State);
void    gotoxy          (uint8_t x, uint8_t y);
void    LcdData         (uint8_t Data); 


/*
***********************************SCAME**************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/

 
/**z
  * @brief  The application entry point.
  * @retval int    
  */  
 

int main(void) 
{
  uint32_t regRtc;
  uint16_t valVin; 
  /* USER CODE BEGIN 1  */
  
  /* USER CODE END 1    */ 
#ifdef NO_BOOT
  __enable_irq();
  /* Configure the system clock */
  SystemClock_PowerConfig(DISABLED);
#endif
     
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* USER CODE BEGIN Init */

  /* USER CODE END Init */ 
  statoEmerg = EMRG_ST_INIT;
  infoV230.lastMsgV230Sent = (uint32_t)0xFFFFFFFF;
  infoV230.statusV230 =  V230_INIT; 
  infoV230.statusSBC485 =  TASK_RS485_SBC_OFF; 
  flagV230OkFiltered = TRUE; 

  /* per adesso uso pulsante di prog per disabilitare power down  */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  /*Configure GPIO pins : SWPROG = PD14  */
  GPIO_InitStruct.Pin = SW_PROG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SW_PROG_GPIO_Port, &GPIO_InitStruct);

#ifdef GD32F4xx  
  if (*(__IO uint32_t *) RCC_BDCR_BDRST_BB != DISABLE) 
  {
    SystemClock_LSI_Enable();
    MX_RTC_Init();
  }
  /* per adesso uso pulsante di prog per disabilitare power down  */
  SystemClock_LSE_Enable();
  MX_RTC_Init();
#else
  /* Configure clock tree */
  SystemClock_Config();
  MX_RTC_Init();
#endif

#ifdef OUT_38KHZ
__HAL_RCC_GPIOC_CLK_ENABLE();
HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_LSE, RCC_MCODIV_1);
    while(1){;}
#endif
 
  /* per adesso uso pulsante di prog per disabilitare power down  */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  /*Configure GPIO pins : SWPROG = PD14  */
  GPIO_InitStruct.Pin = SW_PROG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SW_PROG_GPIO_Port, &GPIO_InitStruct);

#ifndef HW_MP28947  
  /* Configure the LCD_PWR_Pin as output LOW to put LCD in power on  (pull-up to 5V is external)*/
  HAL_GPIO_WritePin(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_RESET);

 /* Configure the LCD_PWR_Pin as output to put LCD in power ON  (pull-up to 5V is external)*/
  GPIO_InitStruct.Pin = LCD_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(LCD_PWR_GPIO_Port, &GPIO_InitStruct); 
#endif
  
#ifdef MPU_ON_INFOSTATION
  /* Set MPU regions */
  MPU_Config();
  MPU_AccessPermConfig();
#endif

  /* Check if the SW1 pression is for autotest entry */
  Check_for_Autotest();


  if ((HAL_GPIO_ReadPin(SW_PROG_GPIO_Port, SW_PROG_Pin) == GPIO_PIN_SET))
  {
    regRtc = getFlagV230(); 
    if (regRtc == BACKUP_230VAC_OFF_VAL)
    {
      /* reset proviene da una mancanza rete, quindi devo attivare la modalità di risparmio energetico "spinto" */
      /* for debug reset the 230 fault flag */
      //setFlagV230(BACKUP_230VAC_ON_VAL);

      /* enter in low power mode */
      StopMode_Measure();
    }
    else
    {
      if ((regRtc == BACKUP_WAIT_END_POWER) || (regRtc == BACKUP_RESTART_WAIT_END_POWER))
      {
        sbcPowerControl(DISABLED);

        /* control only the V230 hardware pin   */
        __HAL_RCC_GPIOC_CLK_ENABLE();
        /*Configure GPIO pins : V230 = PC8  */
        GPIO_InitStruct.Pin = VAC230_DET_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(VAC230_DET_GPIO_Port, &GPIO_InitStruct);

        __HAL_RCC_GPIOE_CLK_ENABLE();
        /*Configure GPIO pins : LED_C_RED = PE13  */
        GPIO_InitStruct.Pin = DLED_C_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(DLED_C_GPIO_Port, &GPIO_InitStruct);
        regRtc = 0;
        do
        {
          HAL_Delay(200);
          toggleHeartLed();
          valVin = vinPwrValue();
        } while (valVin < MIN_VIN_TO_RESTART);
        /* reset the msg flag */
        resetFlagV230Msg();
        /* attesa ritorno 230Vac: SCU dovrebbe spegnersi entro breve tempo ed al ritorno del 230 la flag è coerente  */
        setFlagV230(BACKUP_230VAC_ALREADY_ON_VAL);
        /** 230Vac come back before end residual power */
        /* reset uP and restart */
        setFlagForNvic();
        NVIC_SystemReset();
      }
    }
  }

/* per adesso attivo subito il clock reference 50MHz */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  regRtc = getFlagV230();
  if (regRtc == BACKUP_230VAC_OFF_V24_LOW)
  { 
    /*Configure SBC power and 50MHz Off   */
    HAL_GPIO_WritePin(SBC_PWR_GPIO_Port, SBC_PWR_Pin, GPIO_PIN_RESET);
    /*Configure GPIO pins : PB14  */
    GPIO_InitStruct.Pin = SBC_PWR_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SBC_PWR_GPIO_Port, &GPIO_InitStruct);
    /* Configure the system clock to use internal clock reference */
    SystemClock_PowerConfig(ENABLED);
  }
  else
  {
    /*Configure SBC power and 50MHz On   */
    HAL_GPIO_WritePin(SBC_PWR_GPIO_Port, SBC_PWR_Pin, GPIO_PIN_SET);
    /*Configure GPIO pins : PB14  */
    GPIO_InitStruct.Pin = SBC_PWR_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SBC_PWR_GPIO_Port, &GPIO_InitStruct);
  }

  if (LL_RCC_IsActiveFlag_PORRST() != (uint32_t)0)
  {
    SystemClock_PowerConfig(ENABLED);

#ifndef HW_MP28947  
#ifdef FORSE_NON_SEVE
    /* eliminata 08/07/2025, Nick, nella revisione assenza tensione su TFT E' eseguito un test in sbcPresence() più specifico */
    /** SBC and router OFF  OSC OFF **/
    HAL_GPIO_WritePin(SBC_PWR_GPIO_Port, SBC_PWR_Pin, GPIO_PIN_RESET);
    HAL_Delay(3000);
    /** SBC and router OFF  OSC OFF **/
    HAL_GPIO_WritePin(SBC_PWR_GPIO_Port, SBC_PWR_Pin, GPIO_PIN_SET);
#endif
#endif    
    HAL_Delay(30);    
    SystemClock_PowerConfig(DISABLED);
    /* reset the msg flag */
    resetFlagV230Msg();
    /* attesa ritorno 230Vac: SCU dovrebbe spegnersi entro breve tempo ed al ritorno del 230 la flag è coerente  */
    setFlagV230(BACKUP_230VAC_ALREADY_ON_VAL);
    powerOnReset = TRUE;
  }
  else
  {
    if (regRtc != BACKUP_230VAC_OFF_V24_LOW)
    {
      /* Configure the system clock */
      SystemClock_PowerConfig(DISABLED);
      //SystemClock_Config();
    }
  }

  /* Configure the SysTick to have interrupt in 1ms time basis*/
  if (HAL_SYSTICK_Config(SystemCoreClock / (1000 / uwTickFreq)) > 0U)
  {
    return HAL_ERROR;
  }

  /* only if Vin is more then MIN_VIN_TO_WORK the program can to start*/
  /*Configure Heart Led Port  */
#ifndef HW_MP28947  
  __HAL_RCC_GPIOH_CLK_ENABLE();
#else
  __HAL_RCC_GPIOA_CLK_ENABLE();
#endif  
  HAL_GPIO_WritePin(H_LED_GPIO_Port, H_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : H_LED_Pin */
  GPIO_InitStruct.Pin = H_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(H_LED_GPIO_Port, &GPIO_InitStruct);

  do
  {
    HAL_Delay(100);
    valVin = vinPwrValue();
    toggleHeartLed();
  } while (valVin < MIN_VIN_TO_WORK);
  
  /* Common initializations */  
  MX_GPIO_Init();
  
#ifdef MANAGE_IRC16M_OSC       // Fixed ticket SCU-22
  /* if HSI oscillator is enabled, put the external oscillator OFF */
  if ((RCC->CR & RCC_CR_HSION) == RCC_CR_HSION)   
    HAL_GPIO_WritePin(OSC_50MHz_ON_GPIO_Port, OSC_50MHz_ON_Pin, GPIO_PIN_RESET);             
#endif
  
#ifndef COLLAUDO_PEN 
  MX_TIM1_Init();  
#endif
  
#ifdef GD32F4xx  
/*  Initializations only for GD32F4xx */
  MX_DMA_Init();  /* NOTE: CLK enable for DMA are here, so this function must be called before the others */  
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  // Init CRC for crypto operations
  MX_CRC_Init();
#ifdef DEBUG_TRACE_PIN 
#ifndef HW_MP28947     
  DEBUG_Init_GPIO();
#endif  
#endif 
#else
  /* Initialization required only by STM32H5xx  */
  MX_GPDMA1_Init();
  MX_GPDMA2_Init();
  MX_I2C1_Init();
  MX_TAMP_RTC_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();
#endif
  
  /* Init timer for PWM on CNTCT contact */
#ifdef HW_MP28947  
#ifdef GD32F4xx  
  MX_TIM9_Init();  
#else
  MX_TIM15_Init();    
#endif  
  /* Set ADC window watchdog @20V to detect power down situation */
  triggerVINConfig();  
  /* Manage the board plugged as internal energy meter, is the MP28947-EM  */
  /* Init STPM with complete sequence and set the registers */
  METRO_Init();
#endif  
 
  /* USER CODE BEGIN 2 */
  
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

#ifdef DA_FARE
  setHwInfo();

  // Rimuovo il reset dal fisico ETH
  InitETH();
#endif

  /* definition and creation of I2C3 Semaphore (for EEPROM, IO expander) */
  i2c3SemaphoreCreate();
  
  // Inizializzo la QSPI
  FlashInit();

  // Initialize Eeprom
  EEpromManager_init();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
    
  /* create a binary semaphore used for flash access */
  FlashSemaphoreHandle = osSemaphoreNew(1, 1, NULL);
  
#ifndef GD32F4xx  
  /* definition and creation of Wifi/debug UART7 Semaphore */
  wifiDbgSemaphoreCreate();
#endif
    
  /* USER CODE END RTOS_SEMAPHORES */ 
#ifdef BIN_CANC_IN_MAIN
  removeFwBinFile();
#endif
  
  /* LOG message */
  EVLOG_Message (EV_INFO, "Power ON for %s", (char*)&resetOrigin[Get_RST_Origin()][0]);
        
  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
                                   
  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

#ifdef SPEAD_SPECTRUM
/**
  * @brief Spread spectrum configuration
  * @retval None
  */
static void spreadSpectrumConfig(void)
{

  /*** Fvco = 432MHz fPLL_In = 2MHz Fmod = 1KHz md = 2%                 */
  /*** PLLN = 432 / 2 = 216                                             */
  /*** MDEPER = fPLL_In / (4 x FMod) = 2M  / (4 x 1K) = 2000 /4 = 500   */
  /*** INCSTEP = (2^15 -1)x md x PLLN /(100 5 x MODEPER) =  (2^15 - 1) x 216 (100 x 5 x 500) = 56   */

  LL_RCC_PLL_ConfigSpreadSpectrum((uint32_t)500, (uint32_t)56, (uint32_t)LL_RCC_SPREAD_SELECT_CENTER);
  LL_RCC_PLL_SpreadSpectrum_Enable();
}
#endif


#ifdef GD32F4xx

/**
  * @brief System Clock Configuration
  * @retval None
  */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
   
  /*- Configure the needed RTc clock source */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
 
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /*##-1- Configure LSE as RTC clock source ###################################*/
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  { 
    Error_Handler();
  }
  
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;  // 16MHz / 8 = 2MHz
  RCC_OscInitStruct.PLL.PLLN = 240;    /* Max frequency supported by GD32F470 is 240MHz --> full speed set */
  RCC_OscInitStruct.PLL.PLLQ = 10;     /* Clock CK48M for SDIO and USB @24MHz */
  RCC_OscInitStruct.PLL.PLLP = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */ 
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  
}

#else

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Configure LSE Drive Capability
  *  Warning : Only applied when the LSE is disabled.
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

   /* Select SysTick source clock */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

   /* Re-Initialize Tick with new clock source */
  if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

#endif

/**
  * @brief  Initializes the CPU, AHB and APB busses clocks according to the specified
  *         parameters in the RCC_ClkInitStruct.
  * @param  RCC_ClkInitStruct pointer to an RCC_OscInitTypeDef structure that
  *         contains the configuration information for the RCC peripheral.
  * @param  FLatency FLASH Latency, this parameter depend on device selected
  *
  * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency
  *         and updated by HAL_RCC_GetHCLKFreq() function called within this function
  *
  * @note   The HSI is used (enabled by hardware) as system clock source after
  *         startup from Reset, wake-up from STOP and STANDBY mode, or in case
  *         of failure of the HSE used directly or indirectly as system clock
  *         (if the Clock Security System CSS is enabled).
  *
  * @note   A switch from one clock source to another occurs only if the target
  *         clock source is ready (clock stable after startup delay or PLL locked).
  *         If a clock source which is not yet ready is selected, the switch will
  *         occur when the clock source will be ready.
  *         You can use HAL_RCC_GetClockConfig() function to know which clock is
  *         currently used as system clock source.
  * @note   Depending on the device voltage range, the software has to set correctly
  *         HPRE[3:0] bits to ensure that HCLK not exceed the maximum allowed frequency
  *         (for more details refer to section above "Initialization/de-initialization functions")
  * @retval None
  */

#ifdef GD32F4xx

HAL_StatusTypeDef HAL_RCC_ClockConfigLowPower(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t FLatency)
{
  uint32_t tickstart = 0;

  /* Check Null pointer */
  if (RCC_ClkInitStruct == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_RCC_CLOCKTYPE(RCC_ClkInitStruct->ClockType));
  assert_param(IS_FLASH_LATENCY(FLatency));

  /* To correctly read data from FLASH memory, the number of wait states (LATENCY)
     must be correctly programmed according to the frequency of the CPU clock
     (HCLK) and the supply voltage of the device. */

  /* Increasing the CPU frequency */
  if (FLatency > __HAL_FLASH_GET_LATENCY())
  {
    /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
    __HAL_FLASH_SET_LATENCY(FLatency);

    /* Check that the new number of wait states is taken into account to access the Flash
    memory by reading the FLASH_ACR register */
    if (__HAL_FLASH_GET_LATENCY() != FLatency)
    {
      return HAL_ERROR;
    }
  }

  /*-------------------------- HCLK Configuration --------------------------*/
  if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_HCLK) == RCC_CLOCKTYPE_HCLK)
  {
    /* Set the highest APBx dividers in order to ensure that we do not go through
       a non-spec phase whatever we decrease or increase HCLK. */
    if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
    {
      MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_HCLK_DIV16);
    }

    if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
    {
      MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, (RCC_HCLK_DIV16 << 3));
    }

    /* Set the new HCLK clock divider */
    assert_param(IS_RCC_HCLK(RCC_ClkInitStruct->AHBCLKDivider));
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_ClkInitStruct->AHBCLKDivider);
  }

  /*------------------------- SYSCLK Configuration ---------------------------*/
  if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_SYSCLK) == RCC_CLOCKTYPE_SYSCLK)
  {
    assert_param(IS_RCC_SYSCLKSOURCE(RCC_ClkInitStruct->SYSCLKSource));

    /* HSE is selected as System Clock Source */
    if (RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSE)
    {
      /* Check the HSE ready flag */
      if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET)
      {
        return HAL_ERROR;
      }
    }
    /* PLL is selected as System Clock Source */
    else if (RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK)
    {
      /* Check the PLL ready flag */
      if (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET)
      {
        return HAL_ERROR;
      }
    }
    /* HSI is selected as System Clock Source */
    else
    {
      /* Check the HSI ready flag */
      if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET)
      {
        return HAL_ERROR;
      }
    }

    __HAL_RCC_SYSCLK_CONFIG(RCC_ClkInitStruct->SYSCLKSource);

    /* Get Start Tick*/
    tickstart = HAL_GetTick();

    while (__HAL_RCC_GET_SYSCLK_SOURCE() != (RCC_ClkInitStruct->SYSCLKSource << RCC_CFGR_SWS_Pos))
    {
      if ((HAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
  }

  /* Decreasing the number of wait states because of lower CPU frequency */
  if (FLatency < __HAL_FLASH_GET_LATENCY())
  {
    /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
    __HAL_FLASH_SET_LATENCY(FLatency);

    /* Check that the new number of wait states is taken into account to access the Flash
    memory by reading the FLASH_ACR register */
    if (__HAL_FLASH_GET_LATENCY() != FLatency)
    {
      return HAL_ERROR;
    }
  }

  /*-------------------------- PCLK1 Configuration ---------------------------*/
  if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
  {
    assert_param(IS_RCC_PCLK(RCC_ClkInitStruct->APB1CLKDivider));
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_ClkInitStruct->APB1CLKDivider);
  }

  /*-------------------------- PCLK2 Configuration ---------------------------*/
  if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
  {
    assert_param(IS_RCC_PCLK(RCC_ClkInitStruct->APB2CLKDivider));
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, ((RCC_ClkInitStruct->APB2CLKDivider) << 3));
  }

  /* Update the SystemCoreClock global variable */
  SystemCoreClock = HAL_RCC_GetSysClockFreq() >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];

  /* Configure the source of time base considering new system clocks settings*/
  //HAL_InitTick(uwTickPrio); sospesa

  return HAL_OK;
}

#endif

/**
  * @brief System Clock Configuration in low power
  * @retval None
  */

#ifdef GD32F4xx

void SystemClock_PowerConfig(statusFlag_e statusIn)
{

#ifdef MANAGE_IRC16M_OSC  
  uint32_t regRTC;
#endif
  
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  
#ifdef MANAGE_IRC16M_OSC  
  /* Check if we have to enable the IRC16M oscillator (HSI) */
  regRTC = HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)getHandleRtc(), BACKUP_HW_INFO);
  if (regRTC & IRC16M_OSC_VALID_FLAG)
      if (regRTC & IRC16M_OSC_ENABLE_FLAG)
         statusIn = ENABLED;    // enable the IRC16M (HSI)
#endif
  
  if (statusIn == ENABLED)      // HSI oscillator selected?
  {    
    __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);
    __HAL_RCC_PLL_DISABLE();
    __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSIState = RCC_CR_HSION;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8; // 16MHz / 8 = 2MHz
    RCC_OscInitStruct.PLL.PLLN = 240; /* Max frequency supported by GD32F470 is 240MHz --> full speed set */
  }
  else
  {
    __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);
    __HAL_RCC_PLL_DISABLE();
    __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);
#ifdef HW_MP28947    
    __HAL_RCC_HSI_ENABLE();/* enable HSI oscillator */
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET) {} // Wait till HSI is ready and stable
    __HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);   /* disable HSE oscillator */
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
#else
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
#endif    
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
#ifdef HW_MP28947    
    RCC_OscInitStruct.PLL.PLLM = 8; // 16MHz / 8 = 2MHz In Intek modificato da 8 a 16 --> 1MHz
#else
    RCC_OscInitStruct.PLL.PLLM = 25; // 50MHz / 25 = 2MHz
#endif    
    RCC_OscInitStruct.PLL.PLLN = 240; /* Max frequency supported by GD32F4xx is 240MHz In Intek modificato da 240MHz a 120MHz*/
  }
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 10;   /* Frequency for USB and SDIO is 24MHz */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
#ifndef COME_ERA
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // RCC_SYSCLK_DIV8;    from 216MHz to 27MHz
#else
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4; // RCC_SYSCLK_DIV4;    from 200MHz to 25MHz
#endif
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfigLowPower(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  
  if (statusIn == ENABLED) 
    __HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);   /* disable HSE oscillator */
  else
    __HAL_RCC_HSI_DISABLE();             /* disable HSI oscillator */

  /* starting from V4.1.6C we use the 4K backup SRAM to store total energy SCAME EM */
  __HAL_RCC_BKPSRAM_CLK_ENABLE();

#ifdef USE_BKSRAM
  HAL_PWREx_EnableBkUpReg();
#else
  /* 14/09/2023 per portare il consumo della batteria da 8uA a 1,2uA è necesario togliere l'alimentazione ai 4K di SRAM di backup */
  /* vanno tolte le variabile memorizzate nella section bkpsram (vedi wrapper.c, emSscameTotEnrg */
  if (HAL_PWREx_DisableBkUpReg() != HAL_OK)
  {
    HAL_Delay(30);
  }
#endif    
}

#else

void SystemClock_PowerConfig(statusFlag_e statusIn)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Configure LSE Drive Capability
  *  Warning : Only applied when the LSE is disabled.
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

#endif

/**
  * @brief System Clock Configuration for LSE enable
  * @retval None
  */

void SystemClock_LSE_Enable (void)
{
  
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
 
  /*##-1- Configure LSE as RTC clock source ###################################*/
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  { 
    Error_Handler();
  }
  
}

void SystemClock_LSI_Enable (void)
{
  
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
 
  /*##-1- Configure LSE as RTC clock source ###################################*/
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  { 
    Error_Handler();
  }
  
}


/**
  * @brief Reset backup registers
  * @retval None
  */

void BKP_ResetRegisters (void) 
{
   /* Force reset */
   __HAL_RCC_BACKUPRESET_FORCE();
   /* 100ms delay */
   HAL_Delay(100);
   /* Release reset */
   __HAL_RCC_BACKUPRESET_RELEASE();
   /* 100ms delay */
   HAL_Delay(100);
}

#ifndef USE_SLEEP_MODE_WHEN_VOLTAGE_IS_ABSENT 

/**
  * @brief Slow the clock to reduce the consumption
  * @param  None
  * @retval None
  */

void BACKUP_Mode_Enter (void)
{
    
  /* Select HSI as system clock */
  __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);
  /* Reduce AHB Clock to 1 MHz */
  MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV16);
  /* Reduce APB1 Clock to 1 MHz */
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV1);
  /* Reduce APB2 Clock to 1 MHz */
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1);
  /* Update the SystemCoreClock global variable */
  SystemCoreClock = HAL_RCC_GetSysClockFreq() >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];
  /* Configure the source of time base considering new system clocks settings*/
  HAL_InitTick(uwTickPrio); 
  
}

/**
  * @brief Draw the complete status bar, indicating the status of the Vin
  *        every step is about 3V.
  * @param  None
  * @retval None
  */

void BACKUP_Show_Status_Bar (void)
{ 
  /* Move to second row */
  gotoxy(1,2);
  /* Show the complete status bar */
  for (uint8_t x = 0; x < 20; x++)
    LcdData(0xFF);
}

/**
  * @brief Init GPIO used to pilot the LCD
  * @param  None
  * @retval None
  */

void BACKUP_Init_LCD_GPIO (void)
{
    
  lcdPresent = LCD_PRESENT;  
  /* Init also this variable used to generate delays for LCD management */
  minTimeEnVal = MIN_TIME_EN_INI_VAL;
  /* Init LCD's GPIO  */
  LCD_IO_Init(AS_BACKUP_MODE);  
  
}

/**
  * @brief Draw the progress bar, indicating the status of the Vin:
  * @param  Value of Vin
  * @param  Step for status bar
  * @retval None
  */

void BACKUP_Show_Progress_Bar (uint16_t Vin, uint8_t Vstep)
{
  
  uint8_t BarLenght, cnt;
  
  /* Compute the bar lenght according to the value of Vin reached */
  BarLenght = (((Vin - BACKUP_VIn_THRESHOLD_LOW) / 100) * Vstep) / 10;
  /* Check if at the end of the bar */
  if (BarLenght >= 20) return;
  /* Increment the lenght */
  BarLenght += 1;
  /* Move to the right position */
  gotoxy(BarLenght, 2); 
  /* Draw the spaces */
  for (cnt = BarLenght; cnt <= 20; cnt++)    
    LcdData(' ');

}

/**
  * @brief Manage the progress bar, indicating the status of the Vin:
  * @param  Value of Vin
  * @retval None
  */

void BACKUP_Manage_Progress_Bar(uint16_t Vin)
{
   static uint8_t Vstep = 0;
   
   /* Calculate the step of the progress bar, just the first time */
   if (Vstep == 0)
      Vstep = 20 / ((BACKUP_VIn_THRESHOLD_HIGH - BACKUP_VIn_THRESHOLD_LOW) / 1000);
   /* Show voltage bar on the display, only if present */
   BACKUP_Show_Progress_Bar(Vin, Vstep);
}

#endif

/**
  * @brief  Enter in low power/consumption mode
  * @param  None
  * @retval None
  */

void StopMode_Measure(void)
{

#ifndef USE_SLEEP_MODE_WHEN_VOLTAGE_IS_ABSENT  
  uint16_t Vin;
#endif
  
  /* this function must be called just after a uP reset so all peripheral are disabled and all clock are OFF  */
  
  //uint8_t   standByCnt;

  //standByCnt = 0;

  //while (standByCnt < 4)
  {
    /* the RTC is already programmed in this way: */
    /* Configure RTC prescaler and RTC data registers as follow:
    - Hour Format = Format 24
    - Asynch Prediv = Value according to source clock
    - Synch Prediv = Value according to source clock
    - OutPut = Output Disable
    - OutPutPolarity = High Polarity
    - OutPutType = Open Drain */ 
    /*## Configure the Wake up timer ###########################################*/
    /*  RTC Wakeup Interrupt Generation:
        Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI))
        Wakeup Time = Wakeup Time Base * WakeUpCounter 
                    = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI)) * WakeUpCounter
        ==> WakeUpCounter = Wakeup Time / Wakeup Time Base

    To configure the wake up timer to 20s the WakeUpCounter is set to 0xA017:
      RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16 
      Wakeup Time Base = 16 /(~32.768KHz) = ~0,488 ms
      Wakeup Time = ~1,5s = 0,488ms  * WakeUpCounter
      ==> WakeUpCounter = ~1500ms/0,488ms = 3074 = 0x0C02 
        Wakeup Time Base = 16 /(~32.768KHz) = ~0,488 ms
        Oppure
        Wakeup Time = ~20s = 0,488ms  * WakeUpCounter
        ==> WakeUpCounter = ~20s/0,488ms = 40983 = 0xA017 */

    //HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0xA017, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
    /* all HW not necessary in power down: first level (ETH, OSC, Audio, RFID) */
    setFirstLevelPwdnState();
    /* all remain HW (SBC and router) power OFF */
    setSecondLevelPwdnState();

#ifndef HW_MP28947
    /** adc for detect when the backup voltage level goes under treshold (< 13V)  */
    LcdPwrDwnContrast();
    /* start task del PWM */
    setPWMlcd2x20();
#endif
    
    setPWM_Ledx (LED_C_RED, (uint8_t)0);
    setPWM_Ledx (LED_B_GREEN, (uint8_t)0);
    setPWM_Ledx (LED_A_BLU, (uint8_t)0);

    triggerVINConfig();

    //__disable_irq();
    HAL_SuspendTick();
    
#ifdef GD32F4xx    
    EXTI->PR = 0xFFFFFFFF;
#else
    /* Clear rising edge pending register */
    EXTI->RPR1 = 0x1FFFF;
    /* Clear falling edge pending register */
    EXTI->FPR1 = 0x1FFFF;
#endif    

    /*Configure Heart Led Port  */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    HAL_GPIO_WritePin(H_LED_GPIO_Port, H_LED_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : LD2_Pin */
    GPIO_InitStruct.Pin = H_LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(H_LED_GPIO_Port, &GPIO_InitStruct);
 
    /** timer for detect the 230Vac come back  */
    /** se ci sono transizioni sul pin del V230 allora occorre uscire dallo sleep  */
    setVac230PinAsIntrpt();

    /* FLASH Deep Power Down Mode enabled */
    HAL_PWREx_EnableFlashPowerDown();

#ifdef WAKEUP_ON_TIMEOUT
    /** non serve, perchè ci affidamo al risveglio legato al fatto che la tensione scende sotto i 14V         */
#ifdef GD32F4xx
    HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 240, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);  // erano 70 col vecchio MP43055
#else
    HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 240, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0);  // erano 70 col vecchio MP43055
#endif    
#endif

#ifdef USE_SLEEP_MODE_WHEN_VOLTAGE_IS_ABSENT    
    /* Enter Sleep Mode */    
    HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);    
#else  
    
/******************************************************************************************************/
/****                   BACKUP CONDITION MANAGED REDUCING THE CLOCK FREQUENCY     *********************/
/******************************************************************************************************/
    
    /* Init GPIO used to pilot the LCD, only if present */
    if (lcdPresent) 
        BACKUP_Init_LCD_GPIO();
    /* Switch to underclock mode */
    BACKUP_Mode_Enter();    
    /* Check VIN voltage: when it reach 14V exit from the loop and wait till the board die */
    while (1)
    {
        /* Read Vin value */
        Vin = vinPwrValue();
        /* Check if is under 14V or 24V if 230V comes back */
        if ((Vin < (uint16_t)BACKUP_VIn_THRESHOLD_LOW) || 
            (Vin > (uint16_t)BACKUP_VIn_THRESHOLD_HIGH)) 
          break;
        else if (lcdPresent)
            /* Manage the status bar */
            BACKUP_Manage_Progress_Bar(Vin);
    }
     
    /* if Vin is lower than 14V, restore previous Clock configurations */
    SystemClock_Config();
    
#endif
    
#ifndef HW_MP28947
    /* Disable the interrupt for ADC Analog watchdog, it must be processed only one time after the power reach the low level trigger value */ 
    /* Disable the ADC Analog watchdog interrupt */
    __HAL_ADC_DISABLE_IT(&AdcVINHandle, ADC_IT_AWD);
#endif    
    
    /*Configure GPIO pins : LD2_Pin */
    GPIO_InitStruct.Pin = H_LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(H_LED_GPIO_Port, &GPIO_InitStruct);
    
    HAL_GPIO_WritePin(H_LED_GPIO_Port, H_LED_Pin,GPIO_PIN_SET);

    __enable_irq();
    
    /* Enable Ethernet Clock */
    //__HAL_RCC_ETH_CLK_ENABLE();

#ifdef WAKEUP_ON_TIMEOUT
    /* Disable Wake-up timer */
    if(HAL_RTCEx_DeactivateWakeUpTimer(&hrtc) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler(); 
    }
#endif
    /** when wakeup occured it is necessary to know the reason: V230 come back or backup voltage level under threshold  */
    if (getFlagV230() == BACKUP_230VAC_OFF_V24_LOW) 
    {
      HAL_ResumeTick();

      MX_GPIO_Init(); 
      
      /* per adesso attivo subito il clock reference 50MHz */
      __HAL_RCC_GPIOC_CLK_ENABLE();
            
      /*Configure PWR_DWN1L at non active value --> "1"   */
      HAL_GPIO_WritePin(PWRDWN1L_GPIO_Port, PWRDWN1L_Pin, GPIO_PIN_SET);

      /*Configure GPIO pins : LD2_Pin */
      GPIO_InitStruct.Pin = PWRDWN1L_Pin;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(PWRDWN1L_GPIO_Port, &GPIO_InitStruct);

#ifndef HW_MP28947      
      __HAL_RCC_GPIOD_CLK_ENABLE();
      /*Configure GPIO pins : OUTBL1_M_Pin OUTBL1_P_Pin */
      GPIO_InitStruct.Pin = OUTBL1_M_Pin|OUTBL1_P_Pin;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
#endif
      
      /* Check command type: oldest or newest depending on the motor driver attached */
      block_movement_set(BLOCK_UNLOCK);
      HAL_Delay(350);
      brakePhase();

            
      /* reset info for energy meter   */
      setEmModelReg(MASK_FOR_EM_INT_MODEL, INTERNAL_EM);
      setEmModelReg(MASK_FOR_EM_EXT_MODEL, EXTERNAL_EM);

      /* out from sleep on timeout: Vac 230 not present */
      /* set flag status to signal elapsed time and V230 doesn't come back: so open the block and wait the ending energy */
      /* set flag for open socket  */
      // setFlagV230Msg(MSG_230VAC_OFF_OPEN_SOCKET);
      //deInitSBCUsart();           // stop USART5 comunication to/from SBC
    }
    else
    {
      if (infoV230.edgeCounter != (uint16_t)0)
      {
        HAL_ResumeTick();
        /* out from sleep on edge on V230 pin: Vac 230 is come back */
        setFlagV230(BACKUP_230VAC_OK_FROM_PWDN); 
        /* set flag restart charging  */
        setFlagV230Msg(MSG_230VAC_ON_START_RECHARGE);
      }
      else
      {
        if (getFlagV230() == BACKUP_TIMEOUT_RTC)
        {
          HAL_ResumeTick();
        }
      }
    }
    powerOnReset = TRUE;
    //standByCnt++;

    /* reset flag for Em trap */
    setEmFaultTrap();


  }
}


/** @brief  Re-init I2C3 used for EEPROM and IO Exp    
  *         
  * @param  none
  * 
  * @retval none 
  */
void reinitI2CforEprom(void)
{
  HAL_I2C_DeInit(&hi2c3);
  MX_I2C3_Init();
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  uint32_t  regVal;
  uint16_t  tick1sec;

  getNetworkParameters(&NetworkConfiguration);

  tick1sec = 0;

  /** init for current socket motor management ***/
  imState = IM_IDLE;

  /** init countdown to low power mode  ***/
  cntDownLowPower = 4;
  /** init flag for monV230 monitor management   ***/
  monV230Mng = FALSE;

	// Inizializzo watchdog
#ifdef ACTIVE_IWDG
  MX_IWDG_Init();
  //fwUpgradeInitWD((uint8_t*)&hiwdg);
  /* lascio attivo il watchdog in debug, ma se eseguo uno stop si blocca anche il watchdog */
#ifdef GD32F4xx
  /* I don't know why with STM32H5xx this macro doesn't work */
  __HAL_DBGMCU_FREEZE_IWDG();
#endif  
#endif
  // Abilito il rinfresco del watchdog
  RefreshWatchDog = TRUE;
  immediateRestart = restartOnRemove = FALSE;

  /* set TIM3 for PWM on CP and leave low the pin */
  init_CP_PWM();

#ifdef V230_MON_ACTIVE
  if ((HAL_GPIO_ReadPin(SW_PROG_GPIO_Port, SW_PROG_Pin) == GPIO_PIN_SET))  // only for debug 
  {
    regVal = getFlagV230();
    if ((regVal == BACKUP_230VAC_OFF_V24_LOW))
    {
      /* salvo lo stato nel registro di backup di RTC */
      setFlagV230(BACKUP_WAIT_END_POWER);
    }
  }
#endif

  setUpgradeLcd(UPG_LCD_DISABLED);
  setLcdPresence(LCD_PRESENT);

  TransactionRegister_init();
  
  /* --------------------------------------- */

  /* USER CODE BEGIN 5 */
  /* definition and creation of debugTask and protocols on UART */
  protoUartTaskHandle = osThreadNew(protOnUartTask, NULL, &protoUartTask_attributes); 

  /* Initilaize the telnet module */  
  /* definition and creation of ethernet Task */  
  ethInitTaskHandle = osThreadNew(ethIniProcess, NULL, &ethInitTask_attributes); 
    
  /* definition and creation of digital input manager Task */
  inputMngTaskHandle = osThreadNew(inputMngTask, NULL, &inputMngTask_attributes); 

  /* definition and creation of ADC manager Task */
  adcMngTaskHandle = osThreadNew(adcGestTask, NULL, &adcMngTask_attributes); 

#ifndef HW_MP28947  
  /* definition and creation of digital input manager Task */
  ioexpMngTaskHandle = osThreadNew(ioMngTask, NULL, &ioexpMngTask_attributes); 
#endif
  
  /* definition and creation of led RGB  manager Task */
  ledMngTaskHandle = osThreadNew(ledMngTask, NULL, &ledMngTask_attributes); 

  /* definition and creation of EVS state machine manager Task */
  evsTaskHandle = osThreadNew(EvsMngTask, NULL, &EvsMngTask_attributes); 
  
  /* definition and creation of SOCKET BLOCK manager Task */
  blockTaskHandle = osThreadNew(BlockMngTask, NULL, &BlockMngTask_attributes); 

  /* definition and creation of LCD manager Task */
  lcdTaskHandle = osThreadNew(LcdMngTask, NULL, &LcdMngTask_attributes); 
  /* definition and creation of PLUG manager Task */
  PilotTaskHandle = osThreadNew(PilotMngTask, NULL, &PilotMngTask_attributes); 
  
  /* definition and creation of PWM manager Task */
  PwmTaskHandle = osThreadNew(PwmMngTask, NULL, &PwmMngTask_attributes); 
  
  /* definition and creation of CONTACT manager Task */
  ContactTaskHandle = osThreadNew(ContactMngTask, NULL, &ContactMngTask_attributes); 
  
  /* definition and creation of EXTINP manager Task */
  ExtInpTaskHandle = osThreadNew(ExtInpMngTask, NULL, &ExtInpMngTask_attributes); 
  
  /* definition and creation of EXTINP manager Task */
  EvsTimeTaskHandle = osThreadNew(EvsTimeMngTask, NULL, &EvsTimeMngTask_attributes); 
 
  /* definition and creation of EEPROM manager Task */
#ifdef OLD_EEPROM_MANAGEMENT  
  // xx REMOVED --> EEpromTaskHandle = osThreadNew(EEpromMngTask, NULL, &EEpromMngTask_attributes); 
#endif
  
  /* definition and creation of HTS manager Task */
  HTSTaskHandle = osThreadNew(HTSMngTask, NULL, &HTSMngTask_attributes); 
 
#ifdef TEST_EM_CLIMA
  /* definition and creation of periodic manager Task */
  periodicTaskHandle = osThreadNew(periodicMngTask, NULL, &periodicMngTask_attributes); 
#endif  
  /* definition and creation of RFID manager Task */
  RfidTaskHandle = osThreadNew(RfidMngTask, NULL, &RfidMngTask_attributes); 
   
  /* definition and creation of PERS manager Task */
  PersTaskHandle = osThreadNew(PersMngTask, NULL, &PersMngTask_attributes); 
 
  /* definition and creation of ENERGY manager Task */
  EnergyTaskHandle = osThreadNew(EnergyMngTask, NULL, &EnergyMngTask_attributes); 
 
  /* definition and creation of monitor activities manager Task */
  monMngTaskHandle = osThreadNew(monMngTask, NULL, &monMngTask_attributes); 

#ifndef HW_MP28947  
  /* definition and creation of sinapsi IOM2G  manager Task */
  sinapsiTaskHandle = osThreadNew(sinapsiMngTask, NULL, &sinapsiTask_attributes); 
#endif
  
  /* Initialize Schedule Manager Task */
  schedTaskHandle = osThreadNew(scheduleMngTask, NULL, &schedTask_attributes);

#ifndef HW_MP28947  
  /* definition and creation of retriggerable differential manager Task */
  diffRiarmTaskHandle = osThreadNew(diffRiarmTask, NULL, &diffRiarmTask_attributes); 
  /* definition and creation of retriggerable differential manager Task */
  ISO15118TaskHandle = osThreadNew(ISO15118Task, NULL, &ISO15118Task_attributes); 
#endif  
  /* Autotest reached? */
  if (Autotest)
    /* Initialize Autotest Task */
    AutotestTaskHandle = osThreadNew(AutotestMngTask, NULL, &AutotestTask_attributes);  
    
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);   

    //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    tick1sec++; 
    if (tick1sec == (uint16_t)10)
    {
      tick1sec = (uint16_t)0;
      if (getFastBridge() == ENABLED)
      {
        if(getCounterSlaveDwnl() == (uint32_t)0)
        {
          setFastBridge(DISABLED);
          sendDownloadMsg(SLAVE_END_DWNL); // scuGsyDwldTask()
        }
        setCounterSlaveDwnl((uint32_t)0);
      }
    }
    uint32_t flagAreaStatus = getFlagForQspiArea();
    switch (flagAreaStatus)
    {
      case BACKUP_DATA_QSPI_ERASE:
        break;

      case BACKUP_DATA_QSPI_STORED:
        /* no action: wait for ftp end connection */
        break;

      case BACKUP_DATA_QSPI_AVAILABLE:
        /* new file has been downloaded from FTP: if file is OK save it in the fat and restart for upgrade in the boot */
        tPrintf("New FW stored in QSPI Flash!\n\r");
        osDelay(2000);
        /* no error, so restart to give the control at boot for download in flash */
        /* disable all interrupts */
        SysTick->CTRL = 0;    //Disable Systick
        /* reset immediatly */
        setFlagForNvic();
        NVIC_SystemReset();
        break;

      default:
        /* here normally when flag is BACKUP_DATA_QSPI_DEF_VAL */
        break;
    }


    /*********** V230 CHECK AREA START *****************************************/
#ifdef V230_MON_ACTIVE
    switch (infoV230.statusV230)
    {
      case V230_INIT:
        /* we are at the reset */
        infoV230.edgeCounter = (uint16_t)0;
        infoV230.tickWindowCheck = (uint32_t)100;  /* 100msec the window on V230 check */
        infoV230.tickV230 = (uint32_t)2000;    /* 2ssec for  V230 OK */
        infoV230.tickCurrent = HAL_GetTick();
        infoV230.tickStop = infoV230.tickCurrent + infoV230.tickWindowCheck;
        setVac230PinAsIntrpt();
        infoV230.statusV230 = V230_EVAL;  /* suppose V230  present */
        break;

      case V230_EVAL:
        infoV230.tickCurrent = HAL_GetTick(); 
        if (infoV230.tickStop < infoV230.tickCurrent)
        {
          /* observe windows elapsed */
          infoV230.tickStop = infoV230.tickCurrent + infoV230.tickWindowCheck;  /* set next stop */
          if (infoV230.edgeCounter == (uint16_t)0)
          {
            /* any edge transaction has been observerd on V230 pin, so V230 isn't present */
            infoV230.statusV230 = V230_ABSENT;  /* V230 isn't present */
            flagV230OkFiltered = FALSE;
          }
          else
          {
            infoV230.statusV230 = V230_PRESENT;  /* V230 is present */
          }
          send_to_rfid(RFID_CONTROL_V230);                                       // start RfidMngTaskRFID_CONTROL_V230
          infoV230.edgeCounter = (uint16_t)0; /* reset the edge counter */
        }
      break;

      case V230_ABSENT:
        infoV230.tickCurrent = HAL_GetTick();
        if (infoV230.tickStop < infoV230.tickCurrent)
        {
          /* observe windows elapsed */
          infoV230.tickStop = infoV230.tickCurrent + infoV230.tickWindowCheck;  /* set next stop */
          if (infoV230.edgeCounter != (uint16_t)0)
          {
            /* some edge transaction has been observerd on V230 pin, so V230 is present */
            infoV230.statusV230 = V230_PRESENT;  /* V230 is present */
            flagV230OkFiltered = FALSE;
            infoV230.V230tickStop = infoV230.tickCurrent + infoV230.tickV230;  /* set next stop */
          }
          infoV230.edgeCounter = (uint16_t)0; /* reset the edge counter */
        }
        break;
        
      case V230_KO_WINDOW:
        infoV230.tickCurrent = HAL_GetTick();
        if (infoV230.tickStop < infoV230.tickCurrent)
        {
          /* ignore window elapsed */
          infoV230.tickStop = infoV230.tickCurrent + infoV230.tickWindowCheck;  /* set next stop */
          infoV230.statusV230 = V230_ABSENT;  /* we arrive from V230 absent  */
        }
        infoV230.edgeCounter = (uint16_t)0; /* reset the edge counter */
        break;

      case V230_PRESENT:
        infoV230.tickCurrent = HAL_GetTick();
        if (infoV230.tickStop < infoV230.tickCurrent)
        {
          /* observe windows elapsed */
          infoV230.tickStop = infoV230.tickCurrent + infoV230.tickWindowCheck;  /* set next stop */
          if (infoV230.edgeCounter == (uint16_t)0)
          {
            /* any edge transaction has been observerd on V230 pin, so V230 isn't present */
            infoV230.statusV230 = V230_KO_WINDOW;  /* V230 isn't present */
            flagV230OkFiltered = FALSE;
            /* overwrite tickStop with a 3 sec windows noise. In this window no more checks on V230 status is made  */
            infoV230.tickStop = infoV230.tickCurrent + (uint32_t)3000;    /* 3sec with  V230 KO At the end a new check */
          }
          infoV230.edgeCounter = (uint16_t)0; /* reset the edge counter */
        }
        else
        {
          if (flagV230OkFiltered == FALSE)
          {
            if (infoV230.V230tickStop < infoV230.tickCurrent)
            {
              flagV230OkFiltered = TRUE;      
            }
          }
        }

        break;

      default:
        break;
    }
#else
    infoV230.statusV230 = V230_PRESENT;  /* V230 is present */
#endif
    /*********** V230 CHECK AREA END   *****************************************/
        
  }
  /* USER CODE END 5 */ 
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  
#ifndef HW_MP28947
  uint32_t  i;
#endif  
  
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIMxV230) 
  {
    if (evs_state_get() == EVSTATE_CHARGING)
    {

#ifndef HW_MP28947
      
      /* Check command type: oldest or newest depending on the motor driver attached */
      
      // xx eeprom_param_get(BLOCK_DIR_EADD, &block_polarity, 1);

      switch (infoStation.blockDir & BLOCK_POLARITY_MASK)
      {
        case 0:         /* newest one */
        
          if (((HAL_GPIO_ReadPin(IN6_GPIO_Port, IN6_Pin) == GPIO_PIN_SET)) && ((HAL_GPIO_ReadPin(IN5_GPIO_Port, IN5_Pin) == GPIO_PIN_RESET)))
          {
            setOutBL1_M();
          }
          while (((HAL_GPIO_ReadPin(IN6_GPIO_Port, IN6_Pin) == GPIO_PIN_RESET)) && ((HAL_GPIO_ReadPin(IN5_GPIO_Port, IN5_Pin) == GPIO_PIN_SET)))
          {
            for (i = 0; i < 0x1000; i++)
            {
              ;
            }
          }
          for (i = 0; i < 0x1000; i++)
          {
            ;
          }
          brakePhase();
        break;
        
        case 1:         /* oldest one */

          if (((HAL_GPIO_ReadPin(IN6_GPIO_Port, IN6_Pin) == GPIO_PIN_SET)) && ((HAL_GPIO_ReadPin(IN5_GPIO_Port, IN5_Pin) == GPIO_PIN_RESET)))
          {
            setOutBL1_P();
          }
          while (((HAL_GPIO_ReadPin(IN6_GPIO_Port, IN6_Pin) == GPIO_PIN_RESET)) && ((HAL_GPIO_ReadPin(IN5_GPIO_Port, IN5_Pin) == GPIO_PIN_SET)))
          {
            for (i = 0; i < 0x10000; i++)
            {
              ;
            }
          }
          for (i = 0; i < 0x100000; i++)
          {
            ;
          }
        break;
      } 
#endif      
    }
  }
  else
  {
    if (htim->Instance == TIM14) {
      HAL_IncTick();
    }
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/* For STM32F5Hxx this function is defined in stm32h5xx_periph_init.c file */
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void refreshWD (void)
{
#ifdef ACTIVE_IWDG
  if (RefreshWatchDog)
  {
      HAL_IWDG_Refresh(&hiwdg);
  }
#endif
}

/**
  * @brief  get network parameters  
  *         
  * @param  NetworkConfiguration_t*: pointer where store data
  * 
  * @retval none
  */
void  getNetworkParameters(NetworkConfiguration_t* pNet)
{
  uint8_t   ContByte;
  uint32_t  TempXORChecksum;
  uint8_t*  pSrc;

  pSrc = (uint8_t*)pNet;
  // Il MAC Address a differenza del resto è sulla EEPROM a bordo
  ReadFromEeprom(NET_CONFIG_EEPROM_ADDRESS, pSrc, sizeof(NetworkConfiguration_t));
  // Calcolo il checksum
  for(ContByte = 0, TempXORChecksum = 0; ContByte < sizeof(NetworkConfiguration_t); ContByte++)
  {
    TempXORChecksum += (uint32_t)pSrc[ContByte];
  }
  if ((TempXORChecksum == (uint32_t)0) || ((uint8_t)0 != (uint8_t)TempXORChecksum))
  {
    /* set default net parameter */
    /* questo però è il mac address di STMicroelectronics !! */
    pNet->MACAddress[0] = 0x02; /* cambio 00 in 02 ovvero da MAC globale a localmente amministrato */
    pNet->MACAddress[1] = 0x80; /* usare NetworkConfiguration.MACAddress[]  ?*/
    pNet->MACAddress[2] = 0xE1;                                    
    pNet->MACAddress[3] = 0x00; /* \                                                                          */
    pNet->MACAddress[4] = 0x00; /*   Nik questi byte andrebbero incrementati per ogni SCU: Farlo in collaudo? */
    pNet->MACAddress[5] = 0x00; /* / oppure utilizzare serial number?                                         */
    /* default IP address  */
    pNet->IpAddress[0]  = 192;
    pNet->IpAddress[1]  = 168;
    pNet->IpAddress[2]  = 30;
    pNet->IpAddress[3]  = 127;
    /* default subnet mask address  */
    pNet->SubnetMask[0] = 255;
    pNet->SubnetMask[1] = 255;
    pNet->SubnetMask[2] = 255;
    pNet->SubnetMask[3] = 0U;
    /* default Gateway address  */
    pNet->Gateway[0]    = 192;
    pNet->Gateway[1]    = 168;
    pNet->Gateway[2]    = 30;
    pNet->Gateway[3]    = 1;
    /* default DHCP  */
    pNet->DHCP = (uint8_t)0;
    pNet->portHttp = HTTP_PORT;
    for(ContByte = 0, TempXORChecksum = 0; ContByte < (sizeof(NetworkConfiguration_t) - 1); ContByte++)
    {
      TempXORChecksum += pSrc[ContByte];
    }
    pNet->cksum = (uint8_t)((uint16_t)0x100 - (uint16_t)TempXORChecksum);
    WriteOnEeprom(NET_CONFIG_EEPROM_ADDRESS, pSrc, sizeof(NetworkConfiguration_t));
  }
}

/**
  * @brief  get the pointer to rtc structure 
  *         
  * @param  none
  * 
  * @retval RTC_HandleTypeDef*: pointer structure
  */
void* getHandleRtc(void)
{
   return((void*)&hrtc);
}

/**
*
* @brief        Set flag on RTC backup register      
*
* @param [in]   uint32_t: register backup Id
* @param [in]   uint32_t: data to write
*
* @retval       none 
*
***********************************************************************************************************************/
void setFlagOnRtcBck(uint32_t BackupRegister, uint32_t Data)
{
  HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)&hrtc, BackupRegister, Data);
}

/**
*
* @brief        Reset flag on RTC backup register      
*
* @param [in]   uint32_t: register backup Id
* @param [in]   uint32_t: data to write
*
* @retval       none 
*
***********************************************************************************************************************/
void resetFlagOnRtcBck(uint32_t BackupRegister, uint32_t Data)
{
  HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)&hrtc, BackupRegister, Data);
}

/**
*
* @brief        Set trigger level for TAMPER pin     
*
* @param [in]   RTC_HandleTypeDef: RTC peripheral
* @param [in]   uint32_t: LEVEL for trigger (LOW - HIGH)
*
* @retval       none 
*
***********************************************************************************************************************/
void TAMPER_Trigger_Level_Set(RTC_HandleTypeDef *hrtc, uint32_t LEVEL)
{
  RTC_TamperTypeDef sTamper = {0};

  /** Enable the RTC Tamper 1
  */
  sTamper.Tamper = RTC_TAMPER_1;
  sTamper.PinSelection = RTC_TAMPERPIN_DEFAULT;
  sTamper.Trigger = LEVEL; // RTC_TAMPERTRIGGER_HIGHLEVEL;
  sTamper.Filter = RTC_TAMPERFILTER_4SAMPLE;
  sTamper.SamplingFrequency = RTC_TAMPERSAMPLINGFREQ_RTCCLK_DIV32768;
  sTamper.PrechargeDuration = RTC_TAMPERPRECHARGEDURATION_1RTCCLK;
  sTamper.TamperPullUp = RTC_TAMPER_PULLUP_ENABLE;
  sTamper.TimeStampOnTamperDetection = RTC_TIMESTAMPONTAMPERDETECTION_ENABLE;
  if (HAL_RTCEx_SetTamper_IT(hrtc, &sTamper) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
*
* @brief        Check if TAMPER trigger level is set to LOW level     
*
* @param [in]   RTC_HandleTypeDef: RTC peripheral
*
* @retval       LOW / HIGH
*
***********************************************************************************************************************/

uint32_t _IS_TAMPER_TRIGGER_LEVEL_HIGH (RTC_HandleTypeDef *hrtc)
{
  return ((hrtc->Instance->TAFCR & RTC_TAFCR_TAMP1TRG) != 0);
}

/**
*
* @brief        get flag status on qspi data area      
*
* @param [in]   none
*
* @retval       uint32_t: the area status 
*
***********************************************************************************************************************/
uint32_t getFlagForQspiArea(void)
{
  uint32_t  val;

  val = HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)&hrtc, BACKUP_DATA_QSPI_REG);
  return (val);
}

/**
*
* @brief        get last  rtc setting time      
*
* @param [in]   none
*
* @retval       uint32_t: unix time 
*
***********************************************************************************************************************/
uint32_t getLastRtcSetTime(void)
{
  uint32_t  val;

  val = HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)&hrtc, BACKUP_UNIX_SET_RTC);
  return (val);
}


/**
*
* @brief        check flag on qspi flash data area erased      
*
* @param [in]   none
*
* @retval       uint8_t: TRUE if area is blank
*
***********************************************************************************************************************/
uint8_t checkFlagForQspiAreaErased(void)
{
  uint32_t  val;

  val = HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)&hrtc, BACKUP_DATA_QSPI_REG);
  if (val == BACKUP_DATA_QSPI_DEF_VAL)
  {
    return ((uint8_t)TRUE);
  }
  return ((uint8_t)FALSE);
}

/**
*
* @brief        check file uploaded flag on qspi flash data area      
*
* @param [in]   none
*
* @retval       uint8_t: TRUE if a new file is in the area
*
***********************************************************************************************************************/
uint8_t checkFlagForFileInQspiArea(void)
{
  uint32_t  val;

  val = HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)&hrtc, BACKUP_DATA_QSPI_REG);
  if (val == BACKUP_DATA_QSPI_AVAILABLE)
  {
    return ((uint8_t)TRUE);
  }
  return ((uint8_t)FALSE);
}

/**
*
* @brief        check if a file has been receoved from FTP      
*
* @param [in]   none
*
* @retval       uint8_t: TRUE if a new file is in the area
*
***********************************************************************************************************************/
uint8_t checkFlagForFileFromFTP(void)
{
  uint32_t  val;

  val = HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)&hrtc, BACKUP_DATA_QSPI_REG);
  if (val == BACKUP_DATA_QSPI_STORED)
  {
    return ((uint8_t)TRUE);
  }
  return ((uint8_t)FALSE);
}

/**
*
* @brief        Toggle the heart led status      
*
* @param [in]   none
*
* @retval       none 
*
***********************************************************************************************************************/
void toggleHeartLed(void)
{
  /* Heart bit      */
  HAL_GPIO_TogglePin(H_LED_GPIO_Port, H_LED_Pin);
}

/**
*
* @brief        Idle function       
*
* @param [in]   none
*
* @retval       none 
*
***********************************************************************************************************************/
void vApplicationIdleHook (void)
{
  uint16_t      valVin;
  statusFlag_e  flagVbus; 
  uint8_t       status;
  uint8_t       socket_type;

#ifdef HW_MP28947   
  // ONLY FOR DEBUG --> uint16_t      Key;
#endif  
  

  uint8_t  cnt, ix, v230ComeBack;
#ifdef ACTIVE_IWDG
  // Rinfresco watchdog: da spostare nel task Idle
  refreshWD();
#endif

  flag230 = getFlagV230();
  if (flag230 == BACKUP_230VAC_TRIGGER_OFF_VAL)
  {
    /* salvo lo stato nel registro di backup di RTC */
    setFlagV230(BACKUP_230VAC_OFF_VAL_ACTIVE);
    /* LCD oFF */
    //stopPWMlcd2x20();
    /* SBC power OFF */
    //sbcPowerStatus(OFF);
    /* SCU in low power  */
    //SystemClock_LowPowerConfig();

    /* check if battery backup is OFF */
    if (getStationBatteryBackupMode() == BATTERY_BACKUP_OFF) 
    {
      /* attesa ritorno 230Vac: SCU dovrebbe spegnersi entro breve tempo ed al ritorno del 230 la flag è coerente  */
      setFlagV230(BACKUP_230VAC_ALREADY_ON_VAL);
      /* set flag for open socket  */
      setFlagV230Msg(MSG_230VAC_OFF_OPEN_SOCKET);

      //GPIOA->ODR ^= (uint32_t)0x00000800; /* only for debug */

      /* reset info for energy meter   */
      setEmModelReg(MASK_FOR_EM_INT_MODEL, INTERNAL_EM);
      setEmModelReg(MASK_FOR_EM_EXT_MODEL, EXTERNAL_EM);
      flag230 = getFlagV230();
      tPrintf ("Flag 230 RTC: 0x%X\n\r", flag230);

      /**********************************  init intervento immediato sblocco presa ***************************************/
      /* segnalato dalla produzione che a volte non libera la presa  */
      /* anche se è fatto da Vania, accorcio i tempi e lo faccio subito qui 28/07/2022   */
      setOutputState(CNTCT, GPIO_PIN_RESET);            // diseccito bobina contattore
#ifdef HW_MP28947        
      setOutputState(CNTCT_PWM, GPIO_PIN_RESET);       // Eccito bobina contattore
#else       
      /* first of all unblock the socket, if it is blocked  When free PC10=IN6_PIN=0 PD0=IN5_Pin=1 When blocked PC10=IN6_PIN=1 PD0=IN5_Pin=0   **/
      if (((HAL_GPIO_ReadPin(IN6_GPIO_Port, IN6_Pin) == GPIO_PIN_SET)) && ((HAL_GPIO_ReadPin(IN5_GPIO_Port, IN5_Pin) == GPIO_PIN_RESET)))
      {
        setOutBL1_M();
        do
        {
          /** wait the unblock the socket: when that occures the sensors invert the original state  */
          HAL_Delay(100);
        } while (!((((HAL_GPIO_ReadPin(IN6_GPIO_Port, IN6_Pin) == GPIO_PIN_RESET)) && ((HAL_GPIO_ReadPin(IN5_GPIO_Port, IN5_Pin) == GPIO_PIN_SET)))));
        HAL_Delay(10);
        brakePhase();
      }
#endif      
      
      /**********************************  fine intervento immediato sblocco presa ***************************************/

      /* here wait if 230Vac come back... or wait ending power */
      HAL_Delay(500);
      /* suspend all tasks */
      // vTaskSuspendAll();  Non va fatto: devo lasciare attivo sia la gestione di SBC e di Vania 
      /* attesa ritorno 230 o disabilitazione flag quando il sensore è guasto */ 
      do
      {
        if ((infoV230.statusV230 == V230_PRESENT) || (checkVbusFlag() == 2))
        {
          /* reset if 230come back or the VBUS flag is disabled */
          setFlagV230(BACKUP_230VAC_ALREADY_ON_VAL);
          /** 230Vac come back before end residual power */
          HAL_Delay(1000);
          /* reset uP and restart */
          setFlagForNvic();
          NVIC_SystemReset();
        }
        HAL_Delay(100);
        refreshWD();
      } while (1);
      
    }
    else
    {
      /** invio evento di ingresso in emengenza: entro 3 sec devono concludersi le operazioni di sospensione */
      /** In caso contrario occorre allungare il tempo ma il risparmio energetico diminuisce                 */
      /*** set flag for put the EV in suspend mode This flag is managed in inputMng.c,600                    */
      setFlagV230Msg(MSG_230VAC_SUSPEND_RECHARGE);
      sendMainV230Info(MSG_230VAC_SUSPEND_RECHARGE);

#ifdef TEST24V
      startV24 = getADCmV(VIN_ADC_IN);
#endif
      //setOutputState(CNTCT, GPIO_PIN_RESET);                              // diseccito bobina contattore
      /* attesa 4 sec per simulare il tempo che SBC+router impiegano per informare Central station */
      for (cnt = 0, v230ComeBack = 0; cnt < cntDownLowPower; cnt++ )
      {
        for (ix = 0; ix < 50; ix++)   /* 50 * 20 = 1000msec = 1 sec */
        {
          /* every call take 20msec */
          if ((checkV230T20msecFromHwPin() != 0) && (infoV230.statusV230 != V230_KO_WINDOW))
          {
              setFlagV230(BACKUP_230VAC_SEND_RECHARGE); 
              /* set flag restart charging   */
              setFlagV230Msg(MSG_230VAC_ON_START_RECHARGE);
              sendMainV230Info(MSG_230VAC_ON_START_RECHARGE);
              /* 14/03/2023: occorre informare il gestore RFID del ritorno della tensione di rete */
              send_to_rfid(RFID_CONTROL_V230);                       // start RfidMngTaskRFID_CONTROL_V230
              v230ComeBack = 1;
              break;
          }
        }
        refreshWD();
        if (v230ComeBack == 1)
        {
          break;
        }
      }
      if (v230ComeBack == 0)
      {
        /* salvo lo stato nel registro di backup di RTC */
        setFlagV230(BACKUP_230VAC_OFF_VAL);

        /* start fast the first powe down level */
        setFirstLevelPwdnState();

        /* reset uP to go in power down */
        setFlagForNvic();
        NVIC_SystemReset();
      }
    }
  }
  else
  {
    /**** NON siamo in emergenza **** */
    if (flag230 == BACKUP_TIMEOUT_RTC)
    {
      /** timer for detect the 230Vac come back  */
      /* set flag to force V230 alarm   */
      //setFlagV230Msg(MSG_230VAC_OFF_OPEN_SOCKET);
      /* salvo lo stato nel registro di backup di RTC */
      setFlagV230(BACKUP_FAIL_BOARD);
    }
    else
    {
      if ((flag230 == BACKUP_230VAC_ON_VAL) || (flag230 == BACKUP_230VAC_SEND_RECHARGE) || 
           (flag230 == BACKUP_230VAC_OK_FROM_PWDN))
      {
        setFlagV230(BACKUP_230VAC_ALREADY_ON_VAL);
        if (flag230 == BACKUP_230VAC_OK_FROM_PWDN)
        {
          /* reset uP and restart: Necessario perchè non ripartiva il polling del pilota  */
          HAL_Delay(1000);
          setFlagForNvic();
          NVIC_SystemReset();
        }
      }
      else
      {
        if (flag230 == BACKUP_FAIL_BOARD) 
        {
            /* set station V230 control from   CONTROL_BYTE1_EADD bit  VBUS_CRL1  */
            // xx eeprom_param_get(CONTROL_BYTE1_EADD, (uint8_t *)&flagVbus, 1);
            flagVbus = infoStation.controlByte.Byte.Byte1;
            flagVbus = (((uint8_t)flagVbus & (uint8_t)VBUS_CRL1) == (uint8_t)VBUS_CRL1) ? ENABLED : DISABLED;
            if ((flagVbus == DISABLED) || (infoV230.statusV230 == V230_PRESENT))
            {
              setFlagV230(BACKUP_230VAC_ON_VAL);
              /* reset uP and restart */
              HAL_Delay(2000);
              setFlagForNvic();
              NVIC_SystemReset(); 
            }
        }
      }
    }
  }

  /* check if the monitor on V230 is active. If isn't active but it is enabled so start it*/
  if (checkStartV230Mng() == TRUE) 
  {
    status = checkVbusFlag();
    switch (statoEmerg)
    {
      case EMRG_ST_INIT:
        switch (status)
        {
          case EMRG_V230KO_VBUSENA: /* assenza tensione, flag VBUS abilitata */
            /* Verificare se si  proviene da un fail del circuito di rilevazione */ 
            /* set flag to inform the V230 is OFF    */
            setFlagV230Msg(MSG_230VAC_OFF_OPEN_SOCKET);  // EVS_V230_OFF
            tPrintf("Invio Assenza tensione!\n\r");
            
            if (!((flag230 == BACKUP_FAIL_BOARD) || (flag230 == BACKUP_TIMEOUT_RTC) || (flag230 == BACKUP_WAIT_END_POWER)))
            {
              /* se non si arriva da un falso allarme emergenza oppure la flag VBUS non è disabilitata */
              setFlagV230(BACKUP_230VAC_TRIGGER_OFF_VAL);   /* emergency condition detected -->V230 assente e flag VBUS attiva */
            }
            if (flag230 == BACKUP_WAIT_END_POWER) 
            {
              statoEmerg = EMRG_ST_END_PWR;
            }
            else
            {
              statoEmerg = EMRG_ST_V230KO_VBUSENA;
            }
            break;

          case EMRG_V230OK_VBUSENA: // presenza tensione, VBUS abilitata
            if (flag230 == BACKUP_FAIL_BOARD)
            {
              setFlagV230(BACKUP_230VAC_ON_VAL);
            }
            /** signal for normal condition   */
            setFlagV230Msg(MSG_230VAC_ON_START_RECHARGE);  // EVS_V230_ON
            tPrintf("Invio Presenza tensione!\n\r");
            if (getStationBatteryBackupMode() == BATTERY_BACKUP_OFF) 
            {
              if ((htimV230.Instance == NULL) || (htimV230.Instance->CR1 & (TIM_CR1_CEN) == 0))
              {
                /* we active the gard time to check V230 OFF: don't active again the timer becase an interrupt well be executed !!!*/
                (void)TIM_V230_Config();
              }
            }
            send_to_rfid(RFID_CONTROL_V230);  // ribadisco accensione RFID problema micro interruzioni ripetute sul 230V
            statoEmerg = EMRG_ST_V230OK_VBUSENA;
            break;

          case EMRG_VBUS_DIS: // VBUS disabilitata
            setFlagV230Msg(MSG_230VAC_ON_START_RECHARGE); // EVS_V230_ON
            statoEmerg = EMRG_ST_VBUS_DIS;
            break;
        }
        break;

      case EMRG_ST_V230KO_VBUSENA:  /* assenza tensione, flag VBUS abilitata */
        switch (status)
        {
          case EMRG_V230OK_VBUSENA: // presenza tensione, VBUS abilitata
            statoEmerg = EMRG_ST_INIT;
            break;

          case EMRG_VBUS_DIS: // VBUS disabilitata
            statoEmerg = EMRG_ST_INIT;
            break;
        }
        break;

      case EMRG_ST_V230OK_VBUSENA:  /* presenza tensione, flag VBUS abilitata */
        switch (status)
        {
          case EMRG_V230KO_VBUSENA: // assenza tensione, VBUS abilitata  Gestito ad interrupt 
            statoEmerg = EMRG_ST_INIT;
            break;

          case EMRG_VBUS_DIS: // VBUS disabilitata
            statoEmerg = EMRG_ST_INIT;
            break;
        }
        break;

      case EMRG_ST_VBUS_DIS:  /* flag VBUS disabilitata */
        switch (status)
        {
          case EMRG_V230KO_VBUSENA: // assenza tensione, VBUS abilitata 
            statoEmerg = EMRG_ST_INIT;
            break;

          case EMRG_V230OK_VBUSENA: // presenza tensione, VBUS abilitata
            statoEmerg = EMRG_ST_INIT;
            break;

          case EMRG_VBUS_DIS:
            // xx eeprom_param_get(SOCKET_TYPE_EADD, &socket_type, 1);
            if (infoStation.socketType & EVS_TETHERED)   
            {
              valVin =  VIN_ATTENUATION * getADCmV(VIN_ADC_IN);
              if (valVin < MIN_VIN_TETHERED)
              {
                /* set flag to inform the V230 is OFF    */
                setFlagV230Msg(MSG_230VAC_OFF_OPEN_SOCKET);  // EVS_V230_OFF
                tPrintf("Invio Assenza tensione Tethered!\n\r");
                statoEmerg = EMRG_ST_END_PWR;
              }
            }
            break;
        }
        break;

      case EMRG_ST_END_PWR: // assenza tensione, VBUS abilitata, attesa fine power 
        switch (status)
        {
          case EMRG_V230OK_VBUSENA: // presenza tensione, VBUS abilitata
            /* reset the msg flag */
            resetFlagV230Msg();
            /** 230Vac come back before end residual power */
            setFlagV230(BACKUP_230VAC_ALREADY_ON_VAL); 
            /* reset uP and restart */
            setFlagForNvic();
            NVIC_SystemReset();
            break;

          default:
            /* reset info for energy meter   */
            setEmModelReg(MASK_FOR_EM_INT_MODEL, INTERNAL_EM);
            setEmModelReg(MASK_FOR_EM_EXT_MODEL, EXTERNAL_EM);
            /* reset communications UART    */
#ifndef SOSPESA
            do
            {
              /* reset uP and restart */
              HAL_Delay(1000);
            } while (HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)getHandleRtc(), BACKUP_CHARGE_STATUS) != AUTH_RESET_SAVE);
            setFlagV230(BACKUP_RESTART_WAIT_END_POWER); 
            HAL_Delay(1000);
            setFlagForNvic();
            NVIC_SystemReset();
#endif
#ifdef SOSPESA
            deInitSBCUsart();           // stop USART5 comunication to/from SBC
            deInitRS485Usart();         // stop UART1 comunication to/from SCU and SINAPSI 
            valVin =  VIN_ATTENUATION * getADCmV(VIN_ADC_IN);
            if (valVin < MIN_VIN_TO_WORK)
            {
              HAL_Delay(100);
              toggleHeartLed();
            }
#endif
            break;
        }
        break;

    }
    checkStartSemSbcUartTask();
  }

  /*** here put the motor current socket management to stop the driver **/
  switch (imState)
  {
    case IM_IDLE:
#ifndef HW_MP28947      
      if (getADCmV(IM_ADC_IN) > IM_HIGH_THRSD)      
#endif
      {
        tPrintf("IM triggered!\n\r");
        send_to_block(BLOCK_IM_TRIGGERED); 
        imState = IM_TRIGGERED;
      }
        
      break;

    case IM_TRIGGERED:
#ifndef HW_MP28947      
      if (getADCmV(IM_ADC_IN) < IM_LOW_THRSD)
      {
        imState = IM_IDLE; 
      }
#endif        
      break;
  }

  if ((immediateRestart == TRUE) || (restartOnRemove == TRUE))
  {
    HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), BACKUP_CHARGE_STATUS, 0L);
    while(HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)getHandleRtc(), BACKUP_CHARGE_STATUS) != 0x00000000);
    if (getCollaudoParamWritten() == TRUE)
    {
      getIdModelFromStringAndParConfig((char *)getStationProductCodeString());
    }
    if (restartOnRemove == TRUE)
    {
      HAL_Delay(5000);
    }
    else
    {
      HAL_Delay(500);
    }
    /* reset uP and restart */
    setFlagForNvic();
    NVIC_SystemReset();
  }

#ifdef HW_MP28947  
  
  /* Tamper event detected?  */
  if (TamperDetected == TRUE)
  {
      /* NOTE: in timestamps registers, the year value is not saved */
    if( _IS_TAMPER_TRIGGER_LEVEL_HIGH(&hrtc) )
    {
      /* Tamper detected */
      TAMPER_Trigger_Level_Set (&hrtc, RTC_TAMPERTRIGGER_LOWLEVEL);
      TamperDetected = FALSE;
      tPrintf (ANSI_COLOR_RED"Tamper event detected on %0.2d-%0.2d-%0.2d ", sTamperDate.Date, sTamperDate.Month, GlobalDT.Year);
      tPrintf ("at %0.2d:%0.2d:%0.2d\n\r"ANSI_COLOR_RESET, sTamperTime.Hours, sTamperTime.Minutes, sTamperTime.Seconds);          
      EVLOG_Message (EV_INFO, "Tamper event detected");                                                                                 
    }
    else
    {
      /* Retrigger tamper */
      TAMPER_Trigger_Level_Set (&hrtc, RTC_TAMPERTRIGGER_HIGHLEVEL);      
      TamperDetected = FALSE;
    }
  } 
        
#else
  
  /* Check integrity of sensible data and manage errors */
  EEPROM_Check_Data_Idle();
    
#endif
                     
}

/**
*
* @brief        TIM7 configuration for V230 absence
*
* @param [out]  none
*
* @retval       none
*
***********************************************************************************************************************/
static HAL_StatusTypeDef TIM_V230_Config(void)
{
  uint32_t                uwPrescalerValue = 0;

  /* TIM7 clock enable */
  TIM_V230_CLK_ENABLE();

   
  /*##-1- Configure the TIM peripheral #######################################*/ 
  /* -----------------------------------------------------------------------
    In this example TIM6 input clock (TIM6CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1. ( / 4 see main.c where we have: RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;) 
      TIM6CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM6CLK = HCLK / 2 = SystemCoreClock / 2 
    To get TIM6 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM6CLK / TIM6 counter clock) - 1
    Prescaler = ((SystemCoreClock ) /10 KHz) - 1
  
  ----------------------------------------------------------------------- */  
  
  /* Compute the prescaler value to have TIM7 counter clock equal to 4KHz */
  uwPrescalerValue = ((SystemCoreClock / 2) / 4000) - 1;  /* 216000000 / 2 / 4000 -1 = 27000 - 1 = 26999*/
  
  /* Set TIMx instance */
  htimV230.Instance = TIMxV230;
   
  htimV230.Init.Prescaler     = uwPrescalerValue;
  htimV230.Init.Period        = TIMEOUT_V230_VALUE;     // now 50ms / Tick = 50ms * Ftick = 50ms * 4000 = 200
  htimV230.Init.ClockDivision = 0;                      // quindi aspettiamo 100ms prima di aprire i blocco presa
  htimV230.Init.CounterMode   = TIM_COUNTERMODE_UP;     // on timeout is made a call to: HAL_TIM_PeriodElapsedCallback (line 1761 about)
  htimV230.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if(HAL_TIM_Base_Init(&htimV230) == HAL_OK)
  {
    if (HAL_TIM_Base_Start_IT(&htimV230) == HAL_OK)
    {
      __HAL_TIM_CLEAR_FLAG(&htimV230, TIM_FLAG_UPDATE);
      /*Configure the TIM7 IRQ priority */
      HAL_NVIC_SetPriority(TIMxV230_IRQn, (uint32_t)10 ,0U);
      
      /* Enable the TIM7 global Interrupt */
      HAL_NVIC_EnableIRQ(TIMxV230_IRQn);

      __HAL_DBGMCU_UNFREEZE_TIM7();

      /* Start the TIM time Base generation in interrupt mode */
      return HAL_OK;
    }
  }
  /* Return function status */
  return HAL_ERROR;
}


#ifdef BIN_CANC_IN_MAIN
/**
*
* @brief        Remove bin file 
* @param [in]   none
*
* @retval       char*: pointer ftp user string 
* 
***********************************************************************************************************************/
static void removeFwBinFile (void)
{
  
  uint32_t FlashAddr;  
  uint8_t  nSector;
  
  /* Erase every sector in FW file area  (1MB) */
  for (nSector = 0; nSector < FLASH_FWFILE_NUM_64K_SECTOR; nSector++)
  {
    FlashAddr = EXT_SFLASH_NEW_FW_ADDR_START + (nSector * (uint32_t)FLASH_SECTOR_64K_SIZE);

    if (sFLASH_Erase_Block(FlashAddr) != HAL_OK)
       return;
  }

  /** set download flag at default value   ***/
  setFlagOnRtcBck(BACKUP_DATA_QSPI_REG, BACKUP_DATA_QSPI_FW_CHECKED);

}

#endif

/**
*
* @brief        set the new mac address       
*
* @param [in]   uint8_t*: pointer to mac string
*
* @retval       none  
*
***********************************************************************************************************************/
void setNewMac (unsigned char* tempMac)
{
  uint8_t   ContByte, TempXORChecksum;
  uint8_t*  pSrc;

  pSrc = (uint8_t*)&NetworkConfiguration;
  NetworkConfiguration.MACAddress[0] = tempMac[0]; 
  NetworkConfiguration.MACAddress[1] = tempMac[1]; 
  NetworkConfiguration.MACAddress[2] = tempMac[2];
  NetworkConfiguration.MACAddress[3] = tempMac[3]; 
  NetworkConfiguration.MACAddress[4] = tempMac[4]; 
  NetworkConfiguration.MACAddress[5] = tempMac[5];

  for(ContByte = 0, TempXORChecksum = 0; ContByte < (sizeof(NetworkConfiguration_t) - 1); ContByte++)
  {
    TempXORChecksum += pSrc[ContByte];
  }
  NetworkConfiguration.cksum = (uint8_t)((uint16_t)0x100 - (uint16_t)TempXORChecksum);
  WriteOnEeprom(NET_CONFIG_EEPROM_ADDRESS, pSrc, sizeof(NetworkConfiguration_t));
}

/**
*
* @brief        get the  mac address pointer      
*
* @param [in]   none
*
* @retval       uint8_t*: pointer to mac string  
*
***********************************************************************************************************************/
uint8_t* getMac (void)
{
  return ((uint8_t*)NetworkConfiguration.MACAddress);
}


/**
*
* @brief        set new calibration value for RTC      
*
* @param [in]   uint32_t: value for calp
* @param [in]   uint32_t: value for calm
*
* @retval       none  
*
***********************************************************************************************************************/
void setNewRtcCal (uint32_t calp, uint32_t calm)
{
  HAL_RTCEx_SetSmoothCalib(&hrtc, RTC_SMOOTHCALIB_PERIOD_32SEC,calp, calm);
  calmValue = calm;
  calpValue = calp;
  setFlagOnRtcBck(BACKUP_CALP_RTC, calp);
  setFlagOnRtcBck(BACKUP_CALM_RTC, calm);
}

/**
*
* @brief        get the calibration value       
*
* @param [in]   uint32_t*: pointer for calp
* @param [in]   uint32_t*: pointer for calm
*
* @retval       none  
*
***********************************************************************************************************************/
void getRtcCalValues (uint32_t* pCalp, uint32_t* pCalm)
{
  *pCalm = calmValue;
  *pCalp = calpValue;
}

/**
*
* @brief        get power on reset flag        
*
* @param [in]   none
*
* @retval       uint8_t  
*
***********************************************************************************************************************/
uint8_t getPowerOnResetFlag (void)
{
  return(powerOnReset);
}

/**
*
* @brief        set power on reset flag        
*
* @param [in]   none
*
* @retval       none  
*
***********************************************************************************************************************/
void setPowerOnResetFlag (uint8_t status)
{
  powerOnReset = status;
}


/**
*
* @brief        set the new count down to low power        
*
* @param [in]   uint8_t: count down value 
*
* @retval       none  
*
***********************************************************************************************************************/
void setNewCntDownLP (uint8_t cndDwn)
{
  cntDownLowPower = cndDwn;
}
#ifdef ELIMINATA
/**
*
* @brief        check if V230 come back using Hardware pin monitor.          
*
* @param [in]   none
*
* @retval       uint8_t: 1 if V230 come back  
*
***********************************************************************************************************************/
static uint8_t checkV230FromHwPin (void)
{
  uint8_t  result, numEdge;
  GPIO_PinState pinState, stateCurr;
  statusFlag_e  flagVbus; 


  numEdge = result = 0;

  stateCurr = HAL_GPIO_ReadPin(VAC230_DET_GPIO_Port, VAC230_DET_Pin);
  deInitSBCUsart();           // stop USART5 comunication to/from SBC
  deInitRS485Usart();         // stop USART1 comunication to/from RS485
  stopTimerAdcConv();         // stop timer for ADC conversion (timer and DMA)
  do
  {
    refreshWD();
    pinState = HAL_GPIO_ReadPin(VAC230_DET_GPIO_Port, VAC230_DET_Pin);
    switch (stateCurr)
    {
      case GPIO_PIN_RESET:
        if (pinState == GPIO_PIN_SET)
        {
          numEdge++;
          stateCurr = GPIO_PIN_SET;
          if (numEdge > 10)
          {
            /* 10 periodi = 20msc * 10 = 200msec*/
            result = 1;
          }
        }
        break;

      case GPIO_PIN_SET:
        if (pinState == GPIO_PIN_RESET)
        {
          stateCurr = GPIO_PIN_RESET;
        }
        break;
    }
    /* set station V230 control from   CONTROL_BYTE1_EADD bit  VBUS_CRL1  */
    eeprom_param_get(CONTROL_BYTE1_EADD, (uint8_t *)&flagVbus, 1);
    flagVbus = (((uint8_t)flagVbus & (uint8_t)VBUS_CRL1) == (uint8_t)VBUS_CRL1) ? ENABLED : DISABLED;
    if (flagVbus == DISABLED) result = 1;
    HAL_Delay(2);

  } while (result == 0);

  return (result);
}
#endif
/**
*
* @brief        check if V230 is present in 20msec period          
*
* @param [in]   none
*
* @retval       uint8_t: 1 if V230 come back  
*
***********************************************************************************************************************/
 uint8_t checkV230T20msecFromHwPin (void)
{
  uint8_t       numEdges, results;
  GPIO_PinState pinState, stateCurr;
  uint32_t      tim, timEnd;

  numEdges = results = 0;

  stateCurr = HAL_GPIO_ReadPin(VAC230_DET_GPIO_Port, VAC230_DET_Pin);
  tim = HAL_GetTick();
  timEnd = tim +(uint32_t)30;
  refreshWD();
  do
  {
    pinState = HAL_GPIO_ReadPin(VAC230_DET_GPIO_Port, VAC230_DET_Pin);
    switch (stateCurr)
    {
      case GPIO_PIN_RESET:
        if (pinState == GPIO_PIN_SET)
        {
          numEdges++;
          stateCurr = GPIO_PIN_SET;
          if (numEdges >=2)
          {
            /* 2 edge  detected in 20msec */
            results = 1;
          }
        }
        break;

      case GPIO_PIN_SET:
        if (pinState == GPIO_PIN_RESET)
        {
          stateCurr = GPIO_PIN_RESET;
        }
        break;
    }

    tim = HAL_GetTick();
  } while ((results == 0) && ((tim < timEnd)));

  return (results);
}

/**
*
* @brief        start the monitor on V230         
*
* @param [in]   none 
*
* @retval       none  
*
***********************************************************************************************************************/
void startV230Mng (void)
{
  monV230Mng = TRUE;
}

/**
*
* @brief        active a delayed NVIC restart          
*
* @param [in]   none 
*
* @retval       none  
*
***********************************************************************************************************************/
void activeImmediateReset (void)
{
  immediateRestart = TRUE;
}
	
/**
*
* @brief        active a delayed NVIC restart on Remove Socket (only for SEM)         
*
* @param [in]   none 
*
* @retval       none  
*
***********************************************************************************************************************/
void activeImmediateResetFromRemove (void)
{
  immediateRestart =restartOnRemove = TRUE;
}

/**
*
* @brief        check if is possible to start the monitor on V230         
*
* @param [in]   none 
*
* @retval       none  
*
***********************************************************************************************************************/
static uint8_t checkStartV230Mng (void)
{
  uint32_t ui_flag230; 

  if (getStatusDwnl() == TRUE)   /* if FW download is started, disable V230 check */
    return FALSE;
      
  ui_flag230 = getFlagV230(); 
  if ((monV230Mng == TRUE) && (ui_flag230 != BACKUP_230VAC_TRIGGER_OFF_VAL) && (infoV230.statusV230 != V230_EVAL))
  {
    /** monitor on 230 could be eneabled only after management OK and the alarm isn't set  */
    return (TRUE);
  }
  return(FALSE);
}

/**
*
* @brief        get current V230 status         
*
* @param [in]   none
*
* @retval       uint8_t  
*
***********************************************************************************************************************/
uint8_t isV230PresentValid (void)
{
  if (flagV230OkFiltered == TRUE) 
  {
    return(TRUE); 
  }
  else 
  {
    return(FALSE);
  }
}


/**
*
* @brief        check if sbc Uart can be started          
*
* @param [in]   none
*
* @retval       uint8_t  
*
***********************************************************************************************************************/
void checkStartSemSbcUartTask (void)
{
  uint32_t        regRtc; 
  statusFlag_e    flagVbus; 

  regRtc = getFlagV230();
  if (infoV230.statusSBC485 == TASK_RS485_SBC_OFF)
  {
    infoV230.statusSBC485 = TASK_RS485_SBC_ON;
    // xx eeprom_param_get(CONTROL_BYTE1_EADD, (uint8_t *)&flagVbus, 1);
    flagVbus = infoStation.controlByte.Byte.Byte1;
    flagVbus = (((uint8_t)flagVbus & (uint8_t)VBUS_CRL1) == (uint8_t)VBUS_CRL1) ? ENABLED : DISABLED;

    if ((infoV230.statusV230 == V230_PRESENT) || (flagVbus == DISABLED) || 
        (regRtc == BACKUP_FAIL_BOARD) || (regRtc == BACKUP_230VAC_TRIGGER_OFF_VAL))
    {
      if (regRtc == BACKUP_WAIT_END_POWER)
      {
        /* reset the msg flag */
        resetFlagV230Msg();
        /* attesa ritorno 230Vac: SCU dovrebbe spegnersi entro breve tempo ed al ritorno del 230 la flag è coerente  */
        setFlagV230(BACKUP_230VAC_ALREADY_ON_VAL);
      }
      startSemSbcUartTask();
    }
  }
}


/**
*
* @brief        get current emergency status         
*
* @param [in]   none
*
* @retval       emrgStCond_e  
*
***********************************************************************************************************************/
emrgStCond_e getEmergencyStatus (void)
{
  return(statoEmerg); 
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @brief  Init GPIO pin used for DEBUG
  *         
  * @param  None
  * @param  None
  * @retval None
  */
void DEBUG_Init_GPIO(void)
{
  /* GPIO0_uP and GPIO1_uP used as debug pin   */
  HAL_GPIO_WritePin(GPIO1_uP_GPIO_Port, GPIO1_uP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPIO1_uP_Pin PD1 */
  GPIO_InitStruct.Pin = GPIO1_uP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO1_uP_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIO0_uP_GPIO_Port, GPIO0_uP_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pins : GPIO0_uP_Pin PA11 */
  GPIO_InitStruct.Pin = GPIO0_uP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO0_uP_GPIO_Port, &GPIO_InitStruct);
  
}

/**
*
* @brief        Delete HTS Task  
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void deleteHtsTask (void)
{
  if( HTSTaskHandle != NULL )
  {
    deleteHtsTimer();
    vTaskDelete(HTSTaskHandle);
  }
}

/**
*
* @brief        Delete HTS Task  
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void startLcdTask (void)
{
  /* definition and creation of LCD manager Task */
  lowLevelLcdTaskHandle = osThreadNew(lowLevelLcdMngTask, NULL, &lowLevelLcdMngTask_attributes); 
}

#ifdef MPU_ON_INFOSTATION
/**
  * @brief  Configures the main MPU regions.
  * @param  None
  * @retval None
  */
void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable MPU */
  HAL_MPU_Disable();

  /* Configure RAM region as Region N 0, 512B of size and R/W region infoStatio ram */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = INFO_ADDR;
  MPU_InitStruct.Size = MPU_AREA_SIZE;
  MPU_InitStruct.AccessPermission = portMPU_REGION_READ_WRITE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = RAM_REGION_NUMBER;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  
  /* Enable MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  This function configure the access right using Cortex-M4 MPU regions.
  * @param  None
  * @retval None
  */
void MPU_AccessPermConfig(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Configure region for PrivilegedReadOnlyArray as REGION N 3, 32byte and R
     only in privileged mode */
  /* Disable MPU */
  HAL_MPU_Disable();

  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = INFO_ADDR;
  MPU_InitStruct.Size = MPU_AREA_SIZE;
  MPU_InitStruct.AccessPermission = portMPU_REGION_PRIVILEGED_READ_ONLY;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = RAM_REGION_NUMBER;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  
  /* Enable MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

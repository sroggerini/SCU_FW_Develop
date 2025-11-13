																			   /**
  ******************************************************************************
  * @file    telnet.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    11/20/2009
  * @brief   This file contains all the functions prototypes for the helloworld.c 
  *          file.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TELNET_H
#define __TELNET_H

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>
#include "ff.h"

/* Includes ------------------------------------------------------------------*/
/* Definition for TIMx clock resources */
#define TIM_WRB                            TIM8
#define TIM_WRB_CLK_ENABLE()              __HAL_RCC_TIM8_CLK_ENABLE()
#define DMA_WRB_CLK_ENABLE                __HAL_RCC_DMA2_CLK_ENABLE

#define TIM_WRB_OPM                        TIM8
#define TIM_WRB_OPM_CLK_ENABLE()           __HAL_RCC_TIM8_CLK_ENABLE()
   
/* Definition for TIMx's NVIC */
//#define TIM_WRB_IRQn                      TIM8_UP_TIM13_IRQn
//#define TIM_WRB_IRQHandler                TIM8_UP_TIM13_IRQHandler

#ifdef NO_DMA
#define TIM_WRB_IRQn                      TIM8_CC_IRQn
#define TIM_WRB_IRQHandler                TIM8_CC_IRQHandler
#else
#define TIM_WRB_DMA_IRQn                  DMA2_Stream2_IRQn
#define TIM_WRB_DMA_IRQHandler            DMA2_Stream2_IRQHandler
#endif   
   
   
#define TIM_WRB_CHANNEL_GPIO_PORT()       __HAL_RCC_GPIOH_CLK_ENABLE()

#define TIM_WRB_GPIO_PORT_CHANNEL1        GPIOH
#define TIM_WRB_GPIO_PIN_CHANNEL1         GPIO_PIN_10
#define TIM_WRB_GPIO_AF_CHANNEL1          ((uint32_t)0)

/* Definition for TIM_WRB's DMA TIM8-CCx */
#ifdef NO_WR_TIM_RESET
#define DMA_WRB_CHANNEL                   DMA_CHANNEL_7
#define TIM_WRB_CC1_DMA_INST              DMA2_Stream2
#define TIM_WRB_CCx_DMA_INST              TIM_WRB_CC1_DMA_INST
#define TIM_DMA_ID_CCx                    TIM_DMA_ID_CC1
#else
#define DMA_WRB_CHANNEL                   DMA_CHANNEL_0
#define TIM_WRB_CC2_DMA_INST              DMA2_Stream2
#define TIM_WRB_CCx_DMA_INST              TIM_WRB_CC2_DMA_INST
#define TIM_DMA_ID_CCx                    TIM_DMA_ID_CC2
#endif

/* Definition for WRITE status ---> PORT PI8 */
//#define BUS_WR_B_LOW()                    HAL_GPIO_WritePin(TIM_WRB_GPIO_PORT_CHANNEL1, TIM_WRB_GPIO_PIN_CHANNEL1, GPIO_PIN_RESET); 
//#define BUS_WR_B_HIGH()                   HAL_GPIO_WritePin(TIM_WRB_GPIO_PORT_CHANNEL1, TIM_WRB_GPIO_PIN_CHANNEL1, GPIO_PIN_SET); 
#define BUS_WR_B_LOW()                    (TIM_WRB_GPIO_PORT_CHANNEL1->BSRR=(uint32_t)TIM_WRB_GPIO_PIN_CHANNEL1<<16) 
#define BUS_WR_B_HIGH()                   (TIM_WRB_GPIO_PORT_CHANNEL1->BSRR=(uint32_t)TIM_WRB_GPIO_PIN_CHANNEL1) 
   

/* Definition for Chip select B ---> PORT 2*/
#define CS_B_PORT_CLK_ENABLE()            __HAL_RCC_GPIOH_CLK_ENABLE()
#define CS_B_GPIO_PORT                    GPIOH
#define CS_B_GPIO_PIN                     GPIO_PIN_11

#define CS_B_AF                           ((uint32_t)0)
//#define BUS_CS_B_LOW()                    HAL_GPIO_WritePin(CS_B_GPIO_PORT, CS_B_GPIO_PIN, GPIO_PIN_RESET) 
//#define BUS_CS_B_HIGH()                   HAL_GPIO_WritePin(CS_B_GPIO_PORT, CS_B_GPIO_PIN, GPIO_PIN_SET) 
#define BUS_CS_B_LOW()                    (CS_B_GPIO_PORT->BSRR=(uint32_t)CS_B_GPIO_PIN<<16) 
#define BUS_CS_B_HIGH()                   (CS_B_GPIO_PORT->BSRR=(uint32_t)CS_B_GPIO_PIN) 
#define BUS_CS_ACTIVE()                   ((CS_B_GPIO_PORT->IDR & CS_B_GPIO_PIN) == (uint32_t)GPIO_PIN_RESET)

/* Definition for Mux selector A/B */
#define SEL_AB_PORT_CLK_ENABLE()          __HAL_RCC_GPIOH_CLK_ENABLE();
#define SEL_AB_GPIO_PORT                  GPIOH
#define SEL_AB_GPIO_PIN                   GPIO_PIN_11
#define SEL_AB_AF                         ((uint32_t)0)
#define SEL_A_LOW()                       HAL_GPIO_WritePin(SEL_AB_GPIO_PORT, SEL_AB_GPIO_PIN, GPIO_PIN_RESET); 
#define SEL_B_HIGH()                      HAL_GPIO_WritePin(SEL_AB_GPIO_PORT, SEL_AB_GPIO_PIN, GPIO_PIN_SET); 

/* Definition for Enable Chip select B --> PORT 2 */
#define DYB_XN_PORT_CLK_ENABLE()          __HAL_RCC_GPIOG_CLK_ENABLE();
#define DYB_XN_GPIO_PORT                  GPIOG
#define DYB_XN_GPIO_PIN                   GPIO_PIN_3
#define DYB_XN_AF                         ((uint32_t)0)
//#define DYB_XN_ENA_LOW()                  HAL_GPIO_WritePin(DYB_XN_GPIO_PORT, DYB_XN_GPIO_PIN, GPIO_PIN_RESET); 
//#define DYB_XN_DIS_HIGH()                 HAL_GPIO_WritePin(DYB_XN_GPIO_PORT, DYB_XN_GPIO_PIN, GPIO_PIN_SET); 
#define DYB_XN_ENA_LOW()                  (DYB_XN_GPIO_PORT->BSRR=(uint32_t)DYB_XN_GPIO_PIN<<16) 
#define DYB_XN_DIS_HIGH()                 (DYB_XN_GPIO_PORT->BSRR=(uint32_t)DYB_XN_GPIO_PIN) 

#define DYB_X_PORT_CLK_ENABLE()           __HAL_RCC_GPIOD_CLK_ENABLE();
#define DYB_X_GPIO_PORT                   GPIOD
#define DYB_X_GPIO_PIN                    GPIO_PIN_13
#define DYB_X_AF                          ((uint32_t)0)
//#define DYB_X_DIS_LOW()                   HAL_GPIO_WritePin(DYB_X_GPIO_PORT, DYB_X_GPIO_PIN, GPIO_PIN_RESET); 
//#define DYB_X_ENA_HIGH()                  HAL_GPIO_WritePin(DYB_X_GPIO_PORT, DYB_X_GPIO_PIN, GPIO_PIN_SET); 
#define DYB_X_DIS_LOW()                   (DYB_X_GPIO_PORT->BSRR=(uint32_t)DYB_X_GPIO_PIN<<16) 
#define DYB_X_ENA_HIGH()                  (DYB_X_GPIO_PORT->BSRR=(uint32_t)DYB_X_GPIO_PIN) 

#define BUS_IDLE()                        (GPIOH->BSRR = ((uint32_t)0x000007FC))

#define __HAL_TIM_OPM_DISABLE(__HANDLE__)            ((__HANDLE__)->Instance->CR1&=~(TIM_CR1_OPM))
#define __HAL_TIM_WRB_DISABLE(__HANDLE__)            ((__HANDLE__)->Instance->CR1&=~(TIM_CR1_CEN))

//Telnet service port
#define TELNET_PORT 23

// Telnet Command Structure Definition
#define   IAC                     ((uint8_t)0xFF)
#define   WILL                    ((uint8_t)251)
#define   WONT                    ((uint8_t)252)
#define   DO                      ((uint8_t)253)
#define   DONT                    ((uint8_t)254)

// Telnet Options Definition
#define   ECHO                    ((uint8_t)1)
#define   SGA                     ((uint8_t)3)
#define   TERMINAL_TYPE           ((uint8_t)24)
#define   NEGOTIATE_WINDOW_SIZE   ((uint8_t)31)

/* Telenet Task state definition */
#define   STATE_UI_LOGIN          ((uint8_t)0)
#define   STATE_UI_USERNAME       ((uint8_t)1)
#define   STATE_UI_PASSWORD       ((uint8_t)2)
#define   STATE_UI_TRANSPARENT    ((uint8_t)3)
#define   STATE_UI_INIT_TELNET    ((uint8_t)4)

/* Telenet Task message Status Flag   */
#define   TELNET_RX               ((uint8_t)0)
#define   TELNET_TX               ((uint8_t)1)   
#define   TELNET_CLOSE            ((uint8_t)2)   

/* Telenet Task message size   */
#define   TELNET_MAX_MESSAGE_NUM  ((uint8_t)5)
#define   TELNET_MSG_SIZE         ((uint16_t)192)


// ***  State definition for console function 
#define  CONSOLE_IDLE             0
#define  COMANDI_BOARD_REPAIR     1  /*  comandi FAT su flash                                */
#define  COMANDO_DATA_ORA         2   /* stato cambiamenti data e ora                        */
#define  COMANDO_SD_CARD          3   /* comandi per SD card                                 */
#define  COMANDO_DML              4   /* comandi per protocollo DML                          */
#define  COMANDO_FTP              5   /* comandi per test FTP                                */
#define  COMANDO_FREE             6   /* comando libero                                      */
#define  COMANDO_IPCFG            7   /* comandi per protocollo IP4                          */
#define  COMANDO_DISPLAY          8   /* comandi per display IBIS                            */
#define  COMANDO_SET_IP_PAR       9   /* comandi per setting ip params                       */
#define  COMANDI_EEPROM           10  /* comando per EEPROM                                  */
#define  COMANDI_FLASH            11   /* comandi per flash QSPI                             */

#define  COMANDI_LCD              12  /* comando per test LCD                                */
#define  COMANDI_STATION_MNG      13  /* comandi per test funzioni della station manager     */
#define  COMANDI_EM_MNG           14  /* comando per test Energy meter                       */
#define  COMANDI_WIFI             15  /* comandi per controllo del modulo Wifi               */
#define  COMANDI_MODBUS           16  /* comando per test modbus SBC e SCU                   */

#define  COMANDO_SET_SN           19  /* comando per assegnare serial number                 */
#define  COMANDO_SHOW_ACT_KEY     20  /* comando per mostrare il codice di attivazione       */
#define  COMANDO_SHOW_NET_PAGE    21  /* comando per mostrare la pagina della rete Wi-Fi     */
#define  COMANDO_TRANSACTIONS_MNG 22  /* Comando per gestire le transazioni in QSPI FLASH    */
#define  COMANDO_SET_PRODUCT_SN   23  /* Comando per settare numero seriale del prodotto     */   
#define  COMANDO_SET_PRODUCT_CODE 24  /* Comando per settare numero seriale del prodotto     */ 
#define  COMANDO_SET_FAKE_CODE    25  /* Comando per settare fake product code               */ 
#define  COMANDO_SET_USER_PIN     28  /* Comando per settare il pin utente della app         */   
#define  COMANDO_SET_OSC_TYPE     30  /* Comando per impostare il tipo di oscillatore: IRC16M / HXTAL */      
#define  COMANDO_SET_HTS          31  /* Comando per impostare i parametri di controllo della temepratura */
#define  COMANDI_SRAM             32  /* comandi per SRAM uP                                */
#define  COMANDI_SEM              33  /* comandi per SEM                                    */
#define  COMANDO_VIEW_CONN_NUMBER 34  /* comando per visualizzare i connector number        */
#define  COMANDO_VIEW_FW_VERSIONS 35  /* comando per visualizzare le versioni fw delle SCU connesse a SEM */   
#define  COMANDO_EVLOG            36  /* comandi per visualizzare i comandi                 */
#define  COMANDO_VIEW_ASSIGNED    37  /* comando per visualizzare le SCU slave a cui la SCU master ha assegnato un indirizzo */   
    
    // ***  Opzioni di scelta menù  ***

#define  SEL_RET_MENU            'm'  /* ritorno al menu' di scelta iniziale                 */
#define  SEL_DATA_TIME           '1'  /* gestione comandi data&Time RTC                      */
#define  SEL_LCD_CMD             '2'  /* gestione comandi IP4                                */
#define  SEL_INFO_REPAIR_CMD     '3'  /* gestione comandi riparazione scheda                 */
#define  SEL_EE_CMD              '4'  /* gestione comandi EEprom                             */
#define  SEL_QSPI_CMD            '5'  /* comandi per flash QSPI                              */
#define  SEL_STATION_CMD         '6'  /* gestione comandi della station manager              */
#define  SEL_EM_CMD              '7'  /* gestione comandi per energy meter                   */
#define  SEL_MODBUS_CMD          '8'  /* gestione comandi sul modbus                         */
#define  SEL_IP4_CMD             '9'  /* gestione comandi IP4                                */   
   
#define  SEL_SRAM_UP             '0'  /* gestione comandi SRAM uP                            */
#define  SEL_RESET_SCU           'r'  /* reset immediato SCU                                 */
#define  SEL_SET_SN              's'  /* set serial number SCU                               */
#define  SEL_HTS_SENS            't'  /* set serial number SCU                               */

#define  SEL_RSE_CMD             'p'  /* gestione comandi sinapsi-RSE                        */

#define  SEL_ACT_KEY_CMD         'a'  /* set activation key for Wi-Fi App                    */   
#define  SEL_INSTAL_PIN          'i'  /* set installer pin for Wi-Fi App                     */
#define  SEL_PRODUCT_SN          'c'  /* set product serial number                           */
#define  SEL_PRODUCT_CODE        'd'  /* set product code                                    */
#define  SEL_FAKE_PRODUCT_CODE   'f'  /* set fake product code                               */   
#define  SEL_WIFI_CMD            'w'  /* set Wifi commands                                   */
#define  SEL_RANDOM_DELAY        'r'  /* set random delay value max.1800 sec                 */
#define  SEL_WIFI_ON_OFF         'o'  /* power on/off wifi module                            */
#define  SEL_WIFI_USER_PIN       'u'  /* set user pin for app                                */      
#define  SEL_RESET_ANT_TEST      'b'  /* reset antenna test flag for production              */
#define  SEL_RELAY_CMD           '@'  /* debug: comando rele di carico                       */
#define  SEL_SEM_CMD             'S'  /* gestione comandi SEM                                */
#define  SEL_EVLOG_CMD           'l'  /* gestione comandi EV LOG                             */
    
#define  CONSOLE_RX_MSG_LEN      ((uint16_t)128)
#define  CONSOLE_NUM_RX_MSG      ((uint16_t)2)
#define  TIMEOUT_WAIT_TO_SEND    ((uint16_t)300)   /* timeout to send a message on telnet     */

typedef struct
{
  uint8_t   bufferMsg[TELNET_MSG_SIZE];
  uint16_t  lenMsg; 
  uint8_t   telnetEv;
  uint8_t*  pBufferMsg;
} telnetMsg_st;

typedef struct
{
  uint8_t stato;
  uint8_t codCom;
  uint16_t idxMax;
  uint16_t idx;
} console_t;

typedef struct
{
  char*   nomeFile;
  char*   md5FileCk;
} testFile_t;

/* in the following structure, keep alòigned the head position !!! Nick 27/03/2023 */
typedef __packed struct 
{

  uint8_t             consoleMsgRx[CONSOLE_RX_MSG_LEN];

} consoleMng_st;

/* SCU index and relative address  */
typedef __packed struct
{
  uint8_t*         pConsolMsg;
  uint16_t         len;
}frameConsole_st;


/* list message to be managed   */
struct telnetMsgList{
  frameConsole_st       inMsg;
  struct telnetMsgList* next;
};

typedef struct telnetMsgList *telnetNodeMsg; /*Define node as pointer of msg type struct LinkedList */



extern  DIR            locDir;			  /* Directory object */  
extern  FILINFO        locFinfo;  
extern  FIL            locFile;			  /* File objects */  

/** @defgroup helloworld_Exported_Functions
  * @{
  */

uint16_t      tPrintf                 ( const char * format, ... );
void          telnetProcess           (void * pvParameters);
void          setConsoleIdle          (void);
void          consoleTask             (void * pvParameters);
xQueueHandle  getConsoleQueueHandle   (void);
void          putInConsoleQueueHandle (uint8_t* msgIn);
void          eraseAllBoardInfoEeprom (void);

/**
  * @}
  */



#ifdef __cplusplus
}
#endif

#endif /* __TELNET_H */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/


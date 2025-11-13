/**
* @file        rfidHalTask.h
*
* @brief       RFID I2C  protocol - Definition -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: rfidHalTask.h 76 2022-06-20 09:46:05Z npiergi $
*
*     $Revision: 76 $
*
*     $Author: npiergi $
*
*     $Date: 2022-06-20 11:46:05 +0200 (lun, 20 giu 2022) $
*
*
* @copyright
*       Copyright (C) 2016 SCAME S.p.A. All rights reserved. This file is copyrighted and the property of Aesys
*       S.p.A.. It contains confidential and proprietary information. Any copies of this file (in whole or in part)
*       made by any method must also include a copy of this legend. Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

#ifndef __RFID_TASK_H 
#define __RFID_TASK_H

/************************************************************
 * Include
 ************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include "stm32f7xx_hal.h"
#include "Em_uart.h"

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
/* definition for Task queue */
#define NUM_BUFF_SL030                  ((uint16_t)2)

/* definition for uP resource  */
#define I2CSMBx                         I2C1
#define HI2CSMB0                        hSmb0
#define SMBx_GPIO_AF                    GPIO_AF4_I2C1
#define SMBx_RCC_GPIOx_CLK_ENABLE       __HAL_RCC_GPIOB_CLK_ENABLE  

#define SL030_ADDR0 (uint8_t)(0xA0)  // SL030 default address: JP1 - open  JP2 - open
#define SL030_ADDR1 (uint8_t)(0xA2)  // SL030 default address: JP1 - open  JP2 - close
#define SL030_ADDR2 (uint8_t)(0xA4)  // SL030 default address: JP1 - close JP2 - open
#define SL030_ADDR3 (uint8_t)(0xA6)  // SL030 default address: JP1 - close JP2 - close

#define BLOCK_SIZE    16
#define FWVER_VSIZE   16
#define UIDN_VSIZE    8   // essendo lo UID più esteso lungo 7 byte, il primo byte di card_uid indica il tipo di card
#define SC_INFO       2   // settore residente info
#define BK_INFO       8   // blocco residente info

#define ERR_BIT       (uint8_t)(0x80)

#define MIFARE_IDLE   (uint8_t)(0x00)  // stabilizzazione grandezze elettriche
#define MIFARE_GFWV   (uint8_t)(0x01)  // get fw version and write KeyAA / SL030 detect
#define MIFARE_WKEY   (uint8_t)(0x02)  // write Key
#define MIFARE_SNIF   (uint8_t)(0x03)  // sniff mifare card
#define MIFARE_RIDW   (uint8_t)(0x04)  // lettura uid: scrittura comando
#define MIFARE_RIDR   (uint8_t)(0x05)  // lettura uid: lettura esito
//#define MIFARE_LGNW   (uint8_t)(0x06)  // login settore / scrittura settore: scrittura comando
#define MIFARE_LGNR   (uint8_t)(0x07)  // login settore: lettura esito
#define MIFARE_RBKW   (uint8_t)(0x08)  // lettura blocco: scrittura comando
#define MIFARE_RBKR   (uint8_t)(0x09)  // lettura blocco: lettura esito
#define MIFARE_BLND   (uint8_t)(0x0A)  // disabilitazione temporanea dello sniff
#define MIFARE_DTCE   (uint8_t)(ERR_BIT + 0x00)  // incoerenza / fallimento lettura fw version e scrittura Key 
#define MIFARE_POFF   (uint8_t)(ERR_BIT + 0x01)  // power OFF 

#define KEYAB_DEFT    (uint8_t)(0xFF)

#define SNIF_TMEOUT         pdMS_TO_TICKS((uint16_t)10000)
#define DTCE_TMEOUT         pdMS_TO_TICKS((uint16_t)1000)
//#define MINT_TMEOUT (uint16_t)(50)
#define MINT_TMEOUT         pdMS_TO_TICKS((uint16_t)250)
#define MNUM_TMEOUT         pdMS_TO_TICKS((uint16_t)50)
#define GFWV_TMEOUT         pdMS_TO_TICKS((uint16_t)10)
#define WKEY_TMEOUT         pdMS_TO_TICKS((uint16_t)100)
#define RIDR_TMEOUT         pdMS_TO_TICKS((uint16_t)32)
#define LGNR_TMEOUT         pdMS_TO_TICKS((uint16_t)32)
#define RBKW_TMEOUT         pdMS_TO_TICKS((uint16_t)10)
#define RBKR_TMEOUT         pdMS_TO_TICKS((uint16_t)12)
#define T30S_TMEOUT         pdMS_TO_TICKS((uint16_t)30000)
#define T1S_TMEOUT          pdMS_TO_TICKS((uint16_t)1000)
#define SET_CARD_TMEOUT     pdMS_TO_TICKS((uint16_t)10)
// --------------------------------------------------------------------------------------------------------------------------- //


// Definizioni per la rfidn SL030 board 
#define SMBSL030_ADDR		                SL030_ADDR0

#define MAX_DATA_LEN                    ((uint8_t)32)

/* definition for task management */
#define   NUM_BUFF_SL030_RX             ((uint8_t)2)

#define   MIN_SETUP_EM_ANSWER_TIME      ((uint32_t)5)
#define   MAX_SETUP_EM_ANSWER_TIME      ((uint32_t)30)
#define   MIDDLE_SETUP_EM_ANSWER_TIME   ((uint32_t)((MIN_SETUP_ANSWER_TIME + MAX_SETUP_ANSWER_TIME)/2))

#define   DEFAULT_STATUS_LEN            ((uint16_t)6)

#define   UID_TYPE_OFFSET               ((uint8_t)3)

/* definition for timer task  */
#define   DEFAULT_MIFARE_TIME           pdMS_TO_TICKS((uint16_t)5000)
#define   MIFARE_TIMER_GARD_TIME        pdMS_TO_TICKS((uint16_t)500)

// Definizioni per status  rfidn SL030 board 
typedef enum
{
  SL030_OPERATION_SUCCEED     = 0,    /* no error       */ 
  SL030_NO_ANSWER                     /* RFID not answering           */ 
} sl030StatusError_e;

// Definizioni per command rfidn SL030 board 
typedef enum
{
  SL030_SEL_MIFARE_CARD     = 1,    /* selection Mifare Card        */ 
  SL030_LOGIN_SECTOR,               /* login to a sector            */ 
  SL030_READ_DATA_BLK,              /* read a data block            */ 
  SL030_WRITE_DATA_BLK,             /* read a data block            */ 
  SL030_READ_VALUE_BLK,             /* read a value block           */ 
  SL030_WRITE_MS_KEY,               /* write master key (key A)     */ 
  SL030_GET_FW_VERSION      = 0xF0  /* request FW version           */ 
} sl030Command_e;

typedef enum
{
  SL030_ENA     = 0,       /* SL030 enable            */ 
  SL030_DIS,               /* SL030 disable           */ 
  SL030_CARD_DETECT        /* SL030 cart detected     */ 
} sl030Event_e;

typedef enum
{
  MIFARE_TIM = 0,    //     
  RFID_TIM,          //          
  MFPEND_TIM,        // timer per timeout di mfpend         
  MIFARE_NUM_TIMER
} mifareTim_e;

typedef __packed struct
{
  uint8_t         Len;                  /* number message bytes   */
  sl030Command_e  command;              /* command type           */
  uint8_t         dataCmd[MAX_DATA_LEN];/* data for command       */
}headerFormat_st;

/* define for R/W registers  */
typedef __packed struct
{
  uint8_t         Len;                  /* number message bytes   */
  sl030Command_e  command;              /* command type           */
  uint8_t         dataW[MAX_DATA_LEN];  /* max data to write*/      
} commandW_st;

typedef __packed struct
{
  headerFormat_st headerTx;             /* header format to send for parameter read */
  uint8_t         Len;                  /* number message bytes   */
  sl030Command_e  command;              /* command type           */
  uint8_t         status;               /* status  indication     */
  uint8_t         dataR[MAX_DATA_LEN];  /* max data to read       */      
} commandR_st;

typedef __packed union
{
  commandW_st   commandW;
  commandR_st   commandR;
} messageRW_u;

/* queue info structure */
typedef __packed struct
{
  messageRW_u         messageRW;
} formatRW_st;

/* queue info structure */
typedef __packed struct
{
  sl030Event_e        sl030Event;
} sl030Msg_st;

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 
extern uint8_t rfid_dis;

extern uint8_t sl030_fwver[];

extern uint8_t rfd_uid[];

extern uint8_t rfd_bki0[];
extern uint8_t rfd_bki1[];
extern uint8_t rfd_bki2[];
extern uint8_t rfd_bki3[];
// --------------------------------------------------------------------------------------------------------------------------- //


/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
void            rfidHalTask                   (void * pvParameters);
xQueueHandle    getRfidQueueHandle            (void);
void            setPowerRfid                  (uint8_t status);
uint8_t*        getRfidVersion                (void);


#endif //  __RFID_TASK_H


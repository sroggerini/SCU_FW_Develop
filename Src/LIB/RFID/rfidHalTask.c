/**
* @file        rfidTask.c
*
* @brief       RFID I2C  protocol - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: rfidHalTask.c 76 2022-06-20 09:46:05Z npiergi $
*
*     $Revision: 76 $
*
*     $Author: npiergi $
*
*     $Date: 2022-06-20 11:46:05 +0200 (lun, 20 giu 2022) $
*
*
* @copyright
*       Copyright (C) 2017 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

/************************************************************
 * Include
 ************************************************************/
//#include <main.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "stm32f7xx_hal.h"
#include "stm32f7xx_ll_rcc.h"
#include "cmsis_os.h"
#include "main.h"
#include "wrapper.h"
#include "rfidTask.h"
#include "displayPin.h"

/*
***********************************SCAME**************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#define UID_LMAX  10

#define MIF_1K4 (uint8_t)(0x01)  // Mifare 1k, 4 byte UID
#define MIF_1K7 (uint8_t)(0x02)  // Mifare 1k, 7 byte UID [1]
#define MIF_ULT (uint8_t)(0x03)  // Mifare UltraLight or NATG203[2], 7 byte UID
#define MIF_4K4 (uint8_t)(0x04)  // Mifare 4k, 4 byte UID
#define MIF_4K7 (uint8_t)(0x05)  // Mifare 4k, 7 byte UID [1]
#define MIF_DSF (uint8_t)(0x06)  // Mifare DesFire, 7 byte UID
#define MIF_OTH (uint8_t)(0x0A)  // Other
#define MIF_PLS (uint8_t)(0x0C)  // Mifare 4k, Plus 7 byte UID

#define TXNUM_MAX 3

#define KEYAA_TYPE  (uint8_t)(0xAA)
#define KEYBB_TYPE  (uint8_t)(0xBB)
#define KEY_TYPE    KEYAA_TYPE

#define BLCK_VSIZE    16

#define KEY_WRITE     (uint8_t)(0x12)  // write Key
#define FWVER_READ    (uint8_t)(0xF0)  // read fw version
#define UID_READ      (uint8_t)(0x01)  // read uid / check card presence
#define SEC_LOGIN     (uint8_t)(0x13)  // lgin with stored Key
#define BLOCK_READ    (uint8_t)(0x03)  // read block
#define BLOCK_WRITE   (uint8_t)(0x04)  // write block
// --------------------------------------------------------------------------------------------------------------------------- //


#define     TIMER_TICK_500                ((uint16_t)500)

/*
***********************************SCAME**************************************
**                                                                          **
**                            Local Const                                   **
**                                                                          **
****************************************************************************** 
*/ 
static uint8_t const code mifare_KeyAA[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};    // keyAA impianto


/*
***********************************SCAME**************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 

static formatRW_st        formatRW;  
static mngTaskInfo_st     mngTaskInfo;  

static I2C_HandleTypeDef  hSmb0;   // on I2C1

static                    TimerHandle_t   xMifareTimers[NUM_TIMER];
static uint8_t            fidx;        // indice general purpose usato per cicli ripetitivi
                          
static uint8_t            mifare_state;
static uint16_t           mifare_timer;
                          
static uint8_t            mfpend;        // flag uid in attesa di risposta da GS
static uint16_t           mfpend_timer;   // timer per timeout di mfpend
                          
static uint8_t            mftx_mnum;     // contatore tentativi di accesso a SL030
                          
static uint8_t            mfout;       // esito lettura uid
                          
static uint16_t           rfid_timer;
                          
static uint8_t            blnd_expired;

/*
***********************************SCAME**************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 
uint8_t       rfid_dis;

uint8_t       sl030_fwver[FWVER_VSIZE];

uint8_t       rfd_uid[UIDN_VSIZE];

uint8_t       rfd_bki0[BLCK_VSIZE];
uint8_t       rfd_bki1[BLCK_VSIZE];
uint8_t       rfd_bki2[BLCK_VSIZE];
uint8_t       rfd_bki3[BLCK_VSIZE];


/* rfid    queue  declaration */
xQueueHandle sl030Queue = NULL;

/*
***********************************SCAME**************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/


/*
***********************************SCAME**************************************
**                                                                          **
**                            External Function                             **
**                                                                          **
******************************************************************************
*/

/*
***********************************SCAME**************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
static uint8_t  ProcessRfidRequest      (formatRW_st* pSl030Msg);
static void     i2c1_Handler            (char * file, int line);
static void     MX_I2C1_Init            (void);
static uint8_t  ReadRfid                (commandR_st* TxBuffer, unsigned char *RxBuffer, unsigned short RxLength);
static uint8_t  WriteRfid               (commandW_st* TxBuffer, uint16_t TxLength); 
static void     powerPinRfidConfig      (void);
static void     mifareTimCallBack       (TimerHandle_t pxTimer);
static uint8_t  mifare_read_swVer       (void);

/*
***********************************SCAME**************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/

void rfidTask (void * pvParameters)
{
  uint32_t       timeTick;

  /*-------- Creates an empty mailbox for uart  messages --------------------------*/
  sl030Queue = xQueueCreate(NUM_BUFF_SL030, sizeof(sl030Msg_st));
  configASSERT(sl030Queue != NULL);

  /*-------- Creates all timer for the mifare task  --------------------------*/
  for (ix = (uint8_t)0; ix < (uint8_t)MIFARE_NUM_TIMER; ix++)
  {
    /* in this case we use one shot features */
    xMifareTimers[ix] = xTimerCreate("TimMifare", DEFAULT_MIFARE_TIME, pdFALSEE, (void*)(ix), mifareTimCallBack);
    configASSERT(xMifareTimers[ix] != NULL);
  }

  /*----- Initialization I2C1           -------------------------------------------*/

  MX_I2C1_Init();

  timeTick = portMAX_DELAY);

  /* configure and put in power-on RFID device */
  powerPinRfidConfig();
  /* configure all SW structure used by rfid manager */
  mifare_init();


  for (;;)
  {
    /* Wait for some event from SL030 interrupt  */
    if (xQueueReceive(sl030Queue, (void *)&formatRW, timeTick) == pdPASS)
    {
      ProcessRfidRequest(&formatRW);
    }
    else
    {
      // timeout 
    }
  }
}

/**
*
* @brief       Gestione del protocollo 
*
* @param [in]  formatRW*: puntatore al messaggio ricevuto  
*  
* @retval      uint8_t: error code 
*  
****************************************************************/
static uint8_t ProcessRfidRequest(sl030Msg_st* pSl030Msg)
{
  uint8_t     error, ix;

  error = (uint8_t)0;
  /* check for send message to RFID  */
  switch (mifare_state)
  {
    case MIFARE_SNIF: // attesa presentazione mifare card
      if (pSl030Msg->sl030Event == SL030_CARD_DETECT)   // message HW pin: card detected signal 
      {
        if ((!rfid_dis) && (smb0_enb))
        {
          rfid_enb = 0;
          card_dtcd = 0;

          mfout = mifare_read_uidr(rfd_uid);

          if (mfout == SL030_OPERATION_SUCCEED)                   // lettura uid: esito comando tag in
          {
            if (SNP_VCT2 & MFRE_ANOM2)
            {
//              msg_upd = 1;  // potrebbe essere il posto dove mettere una send? Nick
            }

            SNP_VCT2 &= ~ MFRE_ANOM2;
            mftx_mnum = 0;

            if ((!rfid_enb) || (!rfd_uid[0]))
            {
              smb0_enb = 1;
              mifare_uidrst();
              while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], SNIF_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
              mifare_state = MIFARE_SNIF;
            }
            else 
            {
              if (evs_mod == EVSM_MNET)
              {
                smb0_enb = 1;
                mfpend = 1;
                while ((xTimerChangePeriod (xMifareTimers[MFPEND_TIM], T30S_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
                SNP_VCT0 |= RFID_STAT0;
                blnd_expired = 0; 
                while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], T1S_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
                mifare_state = MIFARE_BLND;
              }
              else 
              {
                while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], LGNR_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
                mifare_state = MIFARE_LGNR;
              }
            }
          }
          else 
          {
            if (mfout == 2)                // lettura uid: esito comando no tag
            {
/*              if (SNP_VCT2 & MFRE_ANOM2)
                msg_upd = 1;*/

              SNP_VCT2 &= ~ MFRE_ANOM2;

              smb0_enb = 1;
              mftx_mnum = 0;
              while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], SNIF_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
              mifare_state = MIFARE_SNIF;
            }
            else if (mftx_mnum < TXNUM_MAX)
            {
              mftx_mnum ++;
              while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], MNUM_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
            }
            else
            {
              smb0_enb = 1;
              while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], DTCE_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
              mifare_state = MIFARE_DTCE;
            }
          }
        }       
      }
      break;

    default:
    break;
  }
  return(error);
}


/**
*
* @brief        Get the pointer to task queue
*
* @param [in]   none
*
* @retval       xQueueHandle: pointer to defined  queue
*
***********************************************************************************************************************/
xQueueHandle getRfidQueueHandle(void)
{
   return(sl030Queue);
}


/**
*
* @brief        Get the pointer to rfid string version
*
* @param [in]   none
*
* @retval       uint8_t: pointer to string
*
***********************************************************************************************************************/
uint8_t*  getRfidVersion(void)
{
  return(sl030_fwver);
}


/**
  * @brief I2C1 init function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  hSmb0.Instance = I2C1;
  hSmb0.Init.Timing = 0x20404768;
  hSmb0.Init.OwnAddress1 = 0;
  hSmb0.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hSmb0.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hSmb0.Init.OwnAddress2 = 0;
  hSmb0.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hSmb0.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hSmb0.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hSmb0) != HAL_OK)
  {
    i2c1_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hSmb0, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    i2c1_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hSmb0, 0) != HAL_OK)
  {
    i2c1_Handler(__FILE__, __LINE__);
  }

}

/**
*
* @brief       Read from RFID device (SL030)
*
* @param [in]  headerFormat_st*: pointer to header format  
*              unsigned char *: pointer where store read data
*              unsigned short: number of data to be read 
*  
* @retval      unsigned char: 0 when successfull read, error code otherwise 
*  
****************************************************************/
static uint8_t ReadRfid(commandR_st* TxBuffer, unsigned char *RxBuffer, unsigned short RxLength)
{

  // Aspetto che il canale I2C sia libero
  while (HAL_I2C_GetState(&HI2CSMB0) != HAL_I2C_STATE_READY)
  {
  }
  // Verifico che il dispositivo sia pronto
  if (HAL_I2C_IsDeviceReady(&HI2CSMB0, SMBSL030_ADDR, 10, 1000) == HAL_OK)
  {
    while (HAL_I2C_Master_Transmit(&HI2CSMB0, SMBSL030_ADDR,(uint8_t *)TxBuffer->headerTx, TxBuffer->Len, 100) != HAL_OK)
    {
      if (HAL_I2C_GetError(&HI2CSMB0) != HAL_I2C_ERROR_AF)
      {
        return(1);
      }
    }
    
    while (HAL_I2C_IsDeviceReady(&HI2CSMB0, SMBSL030_ADDR, 3, 100) != HAL_OK) 
    {
    }
    while (HAL_I2C_Master_Receive(&HI2CSMB0, SMBSL030_ADDR, (uint8_t *)RxBuffer, RxLength, 1000) != HAL_OK)
    {
      if (HAL_I2C_GetError (&HI2CSMB0) != HAL_I2C_ERROR_AF)
      {
        return(2);
      }
    } 
    return(0);
  }
  else
  {
    return(3);
  }
}

/**
*
* @brief       Write to  RFID device (SL030)
*
* @param [in]  commandW_st*: pointer to message format content
*              uint16_t: number of data to be write 
*  
* @retval      uint8_t: 0 when successfull write, error code 
*              otherwise
*  
****************************************************************/
static uint8_t WriteRfid(commandW_st* TxBuffer, uint16_t TxLength)
{

  // Aspetto che il canale I2C sia libero
  while (HAL_I2C_GetState(&HI2CSMB0) != HAL_I2C_STATE_READY)
  {
  }
  // Verifico che il dispositivo sia pronto
  if (HAL_I2C_IsDeviceReady(&HI2CSMB0, SMBSL030_ADDR, 10, 1000) == HAL_OK)
  {
    while (HAL_I2C_Master_Transmit(&HI2CSMB0, SMBSL030_ADDR,(uint8_t *)TxBuffer, TxLength, 100) != HAL_OK)
    {
      if (HAL_I2C_GetError(&HI2CSMB0) != HAL_I2C_ERROR_AF)
      {
        return(1);
      }
    }
  }
  else
  {
    return(3);
  }
}


/*
*
* @brief       Configure GPIO power RFID control (SL030)
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/
static void powerPinRfidConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /*Configure GPIO pin Output Level to 0 for power ON */
  HAL_GPIO_WritePin(SMB0_RFID_PWR_GPIO_Port, SMB0_RFID_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SMB0_RFID_PWR_Pin */
  GPIO_InitStruct.Pin = SMB0_RFID_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SMB0_RFID_PWR_GPIO_Port, &GPIO_InitStruct);
}

/*
*
* @brief       Configure GPIO power RFID control (SL030)
*
* @param [in]  uint8_t: ON / OFF   
*  
* @retval      none 
*  
****************************************************************/
void setPowerRfid(uint8_t status)
{
  if (status != OFF)
  {
    /*Configure GPIO pin Output Level to 0 for power ON */
    HAL_GPIO_WritePin(SMB0_RFID_PWR_GPIO_Port, SMB0_RFID_PWR_Pin, GPIO_PIN_RESET);
  }
  else
  {
    /*Configure GPIO pin Output Level to 1 for power OFF */
    HAL_GPIO_WritePin(SMB0_RFID_PWR_GPIO_Port, SMB0_RFID_PWR_Pin, GPIO_PIN_SET);
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void i2c1_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

/** ************************** MIFARE FUNTIONS ************************************* */

/**
*
* @brief       Lettura versione FW RFID SL030
*
* @param [in]  none  
* 
* @retval      unsigned char: 0 when successfull write, errors
*              number  otherwise
*  
****************************************************************/
static uint8_t mifare_read_swVer(void)
{
  commandR_st*  pCommandR;
  uint8_t       result, len, jx;

  /* init local variables */
  pCommandR = (commandR_st*)formatRW.messageRW.commandR;

  pCommandR->headerTx.Len = (uint8_t)1;                 // len = 1: command
  pCommandR->headerTx.command = SL030_GET_FW_VERSION;   // command
  pCommandR->Len = pCommandR->headerTx.Len + 1U;        // byte to be transmitted = 2: len-command

  /* il numero di byte da trasmettere per leggere il parametro, comprende il byte per la lunghezza  */
  result = ReadRfid(pCommandR, (uint8_t*)&pCommandR->Len, (uint16_t)MAX_DATA_LEN);

  if (result == (uint8_t)0)
  {
    len = pCommandR->Len;

    if ((len >= 2) && (len <= FWVER_VSIZE) && (pCommandR->command == SL030_GET_FW_VERSION) && (pCommandR->status == (uint8_t)SL030_OPERATION_SUCCEED))
    {
      for (jx = 0; jx < len - (uint8_t)2; ix++)
      {
        sl030_fwver[jx] = pCommandR->dataR[jx];
      }
      sl030_fwver[jx] = '\0';
  }
  return result;
}

/**
*
* @brief       scrittura e verifica di key in tutti i settori
*
* @param [in]  uint8_t: indirizzo SL030  
* @param [in]  unsigned char *: pointer to key string
* 
* @retval      unsigned char: 0 when successfull write, errors
*              number  otherwise
*  
****************************************************************/
#ifdef MAX_0
uint8_t max0_mifare_wrte_keyw(uint8_t add, uint8_t *key_vct)
{
  smb0_ack = 0;

  smb0_txvct[0] = add;            // add
  smb0_txvct[1] = 9;              // len
  smb0_txvct[2] = KEY_WRITE;      // command
  smb0_txvct[4] = KEY_TYPE;

  for (fidx = 0; fidx < 6; fidx++)
    smb0_txvct[fidx + 5] = *(key_vct + fidx);

  for (fidx = 0; fidx < 16; fidx++)
  {
    smb0_txvct[3] = fidx;         // sector

    smb0_rw = 0;                  // write
    smb0_len = smb0_txvct[1] + 1;
    smb0_busy = 1;

    /* il numero totale di byte da trasmettere, a parte l'indirizzo, è len + 1 */
    if (WriteRfid((commandW_st*)&smb0_txvct[1], (uint16_t)(smb0_len) == (uint8_t)0) 
    {
      /* no errors, check this writing */
      osDelay(60);

      smb0_rw = 1;        // read
      smb0_len = 1;       // valore fittizio, corretto in SMBUS0_ISR
      smb0_busy = 1;

      /* il numero di byte da trasmettere per leggere il parametro, a parte l'indirizzo, è 2: len-command */
      smb0_txvct[1] = (uint8_t)2; 
      result = ReadRfid((commandR_st*)&smb0_txvct[1], smb0_rxvct, (uint16_t)MAX_DATA_LEN);

      if ((smb0_rxvct[0] ^ 0x02) || (smb0_rxvct[1] ^ KEY_WRITE) || (smb0_rxvct[2]))
        smb0_ack = 0;
    }
  }

  return smb0_ack;
}
#else
uint8_t mifare_wrte_keyw(uint8_t add, uint8_t *key_vct)
{
  commandR_st*  pCommandR;
  commandW_st*  pCommandW;
  uint8_t       singleRes, result;

  /* init local variables */
  result = singleRes = (uint8_t)0;
  pCommandR = (commandR_st*)formatRW.messageRW.commandR;
  pCommandW = (commandR_st*)formatRW.messageRW.commandW;

  /* start  write and read messages */
  pCommandW->Len = 9;
  pCommandW->command = pCommandR->headerTx.command = KEY_WRITE;
  pCommandW->dataW[1] = pCommandR->headerTx.dataCmd[1] = KEY_TYPE;

  for (fidx = 0; fidx < 6; fidx++)
  {
    pCommandW->dataW[fidx + (uint8_t)2] = pCommandR->headerTx.dataCmd[] = *(key_vct + fidx);
  }

  for (fidx = 0; fidx < 16; fidx++)
  {
    pCommandW->dataW[0] = pCommandR->headerTx.dataCmd[0] = fidx;         // sector

    /* il numero totale di byte da trasmettere, a parte l'indirizzo, è len + 1 */
    if (WriteRfid(pCommandW, (uint16_t)((uint16_t)pCommandW->Len + (uint16_t)1)) == (uint8_t)0) 
    {
      /* no errors, check this writing */
      osDelay(60);

      /* il numero di byte da trasmettere per rileggere il parametro, a parte l'indirizzo, è 10: len-command-sector-type-key(6bytes) */
      pCommandR->headerTx.Len = pCommandW->Len; 
      pCommandR->Len = pCommandR->headerTx.Len + 1U;  // byte to be transmitted = 10: len-command-sector-type-key(6bytes)
      singleRes = ReadRfid(pCommandR, (uint8_t*)&pCommandR->Len, (uint16_t)MAX_DATA_LEN);

      if ((pCommandR->Len != (uint8_t)0x02) || (pCommandR->command != KEY_WRITE) || (pCommandR->status != 0) || (singleRes != 0))
      {
        /* an error occured */
        result++;
      }
    }
  }

  return (result);
}
#endif
// --------------------------------------------------------------------------------------------------------------------------- //

/**
*
* @brief       lettura uid
*
* @param [in]  uint8_t: indirizzo SL030  
* @param [in]  unsigned char *: pointer to key string
* 
* @retval      uint8_t: 0 when successfull read, 2 when no TAG, 1 otherwise
*  
****************************************************************/
uint8_t mifare_read_uidr(uint8_t *uid_vct)
{
#ifdef MAX_0
  uint8_t len;
  uint8_t ret;

  ret = 0;
  smb0_ack = 1;

  smb0_rw = 1;          // read
  smb0_len = 1;         // valore fittizio, corretto in SMBUS0_ISR
  smb0_busy = 1;

  smb0_start();

  len = smb0_rxvct[0];

  if ((smb0_ack) && (len >= 2) && (len <= UID_LMAX) && (smb0_rxvct[1] == UID_READ))
  {
    if ((len == 2) && (smb0_rxvct[2] == 1))
      ret = 2;
    else if (!smb0_rxvct[2])
    {
      for(fidx = 0; fidx < UIDN_VSIZE; fidx++)
        *(uid_vct + fidx) = 0x00;

      *uid_vct = smb0_rxvct[len];

      for(fidx = 0; fidx < (len - 3); fidx++)
        *(uid_vct + fidx + 1) = smb0_rxvct[fidx + 3];

      ret = 1;
    }
  }

  return ret;
#else
  commandR_st*  pCommandR;
  uint8_t       result, len;

  /* init local variables */
  pCommandR = (commandR_st*)formatRW.messageRW.commandR;

  pCommandR->headerTx.Len = (uint8_t)1;           // len = 1: command
  pCommandR->headerTx.command = UID_READ;         // command
  pCommandR->Len = pCommandR->headerTx.Len + 1U;  // byte to be transmitted = 2: len-command

  /* il numero di byte da trasmettere per leggere il parametro, comprende il byte per la lunghezza  */
  result = ReadRfid(pCommandR, (uint8_t*)&pCommandR->Len, (uint16_t)MAX_DATA_LEN);

  if (result == (uint8_t)0)
  {
    len = pCommandR->Len;

    if ((len >= 2) && (len <= UID_LMAX) && (pCommandR->command == UID_READ))
    {
      if ((len == 2) && (pCommandR->status == (uint8_t)1))
      {
        result = (uint8_t)2;  // no tag
      }
      else if (pCommandR->status == (uint8_t)0)
      {
        /* operation succeed : read the UIDN */
        for(fidx = 0; fidx < UIDN_VSIZE; fidx++)
        {
          *(uid_vct + fidx) = 0x00;
        }
        *uid_vct = pCommandR->dataR[len - UID_TYPE_OFFSET];

        for(fidx = 0; fidx < (len - 3); fidx++)
        {
          *(uid_vct + fidx + 1) = pCommandR->dataR[fidx];
        }
      }
    }
    else
    {
      result = (uint8_t)1;
    }
  }
  return result;
#endif
}
// --------------------------------------------------------------------------------------------------------------------------- //

/**
*
* @brief       login settore
*
* @param [in]  uint8_t: indirizzo SL030  
* @param [in]  uint8_t: sector Id  
* 
* @retval      uint8_t: 0 when successfull login, error code id  otherwise
*  
****************************************************************/
uint8_t mifare_lgin_secr(uint8_t add, uint8_t sec)
{
#ifdef MAX_0
  smb0_ack = 1;

  smb0_txvct[0] = add;          // add
  smb0_txvct[1] = 3;            // len
  smb0_txvct[2] = SEC_LOGIN;    // command
  smb0_txvct[3] = sec;          // sector
  smb0_txvct[4] = KEY_TYPE;

  smb0_rw = 0;          // write
  smb0_len = smb0_txvct[1] + 1;
  smb0_busy = 1;

  smb0_start();

  return smb0_ack;
#else
  commandR_st*  pCommandR;
  uint8_t       result, len;

  /* init local variables */
  pCommandR = (commandR_st*)formatRW.messageRW.commandR;

  pCommandR->headerTx.Len = 3;                    // len = 3: command-sector-type
  pCommandR->Len = pCommandR->headerTx.Len + 1U;  // byte to be transmitted = 4: len-command-sector-type
  pCommandR->headerTx.command = SEC_LOGIN;        // command
  pCommandR->headerTx.dataCmd[0] = sec;           // sector ID
  pCommandR->headerTx.dataCmd[1] = KEY_TYPE;      // kye type: 0xAA for KeyA 0xBB for KeyB
  pCommandR->Len = pCommandR->headerTx.Len + 1U;  // byte to be transmitted = 4: len-command-sector-type

  /* il numero di byte da trasmettere per leggere il parametro, comprende il byte per la lunghezza  */
  result = ReadRfid(pCommandR, (uint8_t*)&pCommandR->Len, (uint16_t)MAX_DATA_LEN);

  if (result == (uint8_t)0)
  {
    len = pCommandR->Len;

    if ((len != 2) || (pCommandR->command != SEC_LOGIN) || (pCommandR->status != (uint8_t)0x02))
    {
      result = (uint8_t)1;
    }
  }
  return result;
#endif
}
// --------------------------------------------------------------------------------------------------------------------------- //

/**
*
* @brief       lettura blocco nell' RFID device (SL030)
*
* @param [in]  uint8_t: indirizzo SL030  
* @param [in]  uint8_t: indice del blocco  
* @param [in]  unsigned char *: pointer where store read data
* 
* @retval      unsigned char: 0 when successfull read, error code otherwise 
*  
****************************************************************/
uint8_t mifare_read_blkr(uint8_t add, uint8_t blk, uint8_t* pData)
{
#ifdef MAX_0
  uint8_t   result;

  smb0_txvct[0] = add;            // add
  smb0_txvct[1] = 2;              // len
  smb0_txvct[2] = BLOCK_READ;     // command
  smb0_txvct[3] = blk;            // block

  smb0_rw = 0;                    // write
  smb0_len = smb0_txvct[1] + 1;
  smb0_busy = 1;

  /* il numero di byte da trasmettere per leggere il parametro, a parte l'indirizzo, è 3: len-command-index block */
  smb0_txvct[1] = smb0_len; 
  result = ReadRfid((commandR_st*)&smb0_txvct[1], pData, (uint16_t)MAX_DATA_LEN);

  return result;
#else
  commandR_st*  pCommandR;
  uint8_t       result;

  /* init local variables */
  pCommandR = (commandR_st*)formatRW.messageRW.commandR;

  pCommandR->headerTx.Len = 3;                // len = 3: len-command-index block
  pCommandR->headerTx.command = BLOCK_READ;   // command
  pCommandR->headerTx.dataCmd[0] = blk;       // block

  smb0_rw = 0;                    // write
  smb0_len = smb0_txvct[1] + 1;
  smb0_busy = 1;

  /* il numero di byte da trasmettere per leggere il parametro, comprende il byte per la lunghezza  */
  result = ReadRfid(pCommandR, pData, (uint16_t)MAX_DATA_LEN);

  return result;
#endif
}
// --------------------------------------------------------------------------------------------------------------------------- //

/**
*
* @brief        scrittura blocco
*
* @param [in]   uint8_t: blocco da scrivere
* @param [in]   uint8_t: contenuto da scrivere
*
* @retval       none
*
***********************************************************************************************************************/
void mifare_wrte_blkw(uint8_t add, uint8_t blk, uint8_t *blk_vct)
{
#ifdef MAX_0
  smb0_txvct[0] = add;            // add
  smb0_txvct[1] = 18;             // len
  smb0_txvct[2] = BLOCK_WRITE;    // command
  smb0_txvct[3] = blk;            // block

  smb0_rw = 0;                    // write
  smb0_len = smb0_txvct[1] + 1;
  smb0_busy = 1;

  for (fidx = 0; fidx < 16; fidx++)
    smb0_txvct[(fidx + 4)] = *(blk_vct + fidx);

  smb0_start();

#else
  commandW_st*  pCommandW;

  /* init local variables */
  pCommandW = (commandR_st*)formatRW.messageRW.commandW;

  /* start  write and read messages */     
  pCommandW->Len = 18;                 // len    
  pCommandW->command = BLOCK_WRITE;    // command
  pCommandW->dataW[0] = blk;           // block  

  for (fidx = 0; fidx < 16; fidx++)
    pCommandW->dataW[(fidx + (uint8_t)1)] = *((uint8_t*)((uint32_t)blk_vct + (uint32_t)fidx));

  /* il numero totale di byte da trasmettere, a parte l'indirizzo, è len + 1 */
  (void)WriteRfid(pCommandW, (uint16_t)((uint16_t)pCommandW->Len + (uint16_t)1)); 

#endif
}
// --------------------------------------------------------------------------------------------------------------------------- //

/**
*
* @brief        reset dati id
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void mifare_uidrst(void)
{
  SNP_VCT0 &= ~ RFID_STAT0;

  for (fidx = 0; fidx < UIDN_VSIZE; fidx++)
    rfd_uid[fidx] = 0x00;

  for (fidx = 0; fidx < BLCK_VSIZE; fidx++)
  {
    rfd_bki0[fidx] = 0x00;
    rfd_bki1[fidx] = 0x00;
    rfd_bki2[fidx] = 0x00;
    rfd_bki3[fidx] = 0x00;
  }
}
// --------------------------------------------------------------------------------------------------------------------------- //

/**
*
* @brief       inizializza variabili controllate da mifare_manager
*
* @param [in]  none   
*  
* @retval      none 
*  
****************************************************************/
void mifare_init(void)
{
  for(fidx = 0; fidx < FWVER_VSIZE; fidx++)
    sl030_fwver[fidx] = 0xFF;

  mifare_uidrst();

  mfout = 0;                    // reset esito lettura uid
  mfpend = 0;                   // reset flag uid in attesa di risposta da GS
  mftx_mnum = 0;                // reset contatore tentativi di accesso a SL030

  blnd_expired = 0;

  rfid_dis = 3;                 // forza primo ingresso in MIFARE_GFWV

  mifare_state = MIFARE_IDLE;
  /* set timer */
  while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], MINT_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
  while ((xTimerChangePeriod (xMifareTimers[RFID_TIM], DEFAULT_MIFARE_TIME, MIFARE_TIMER_GARD_TIME) != pdPASS)); 

  /* default mode is FREE */
  evs_mod = EVSM_FREE;
}
// --------------------------------------------------------------------------------------------------------------------------- //

/**
*
* @brief        callback to manager RFID timers   
*
* @param [in]   TimerHandle_t: the elapsed timer 
*
* @retval       none
*
***********************************************************************************************************************/
static void mifareTimCallBack (TimerHandle_t pxTimer)
{
  uint32_t          timer_id;

  /* find the  the timer Id */
  timer_id = (uint32_t)pvTimerGetTimerID(pxTimer);

  if ((mfpend) && (timer_id == (uint32_t)MFPEND_TIM))
	{
  	mifare_uidrst();
  	mfpend = 0;
	}

  if (timer_id == (uint32_t)RFID_TIM)
	{
    rfid_dis = 0;
	}

  /* check if timer exist and do actions   */
  if (timer_id == (uint32_t)MIFARE_TIM)
  {
    switch(mifare_state)
    {
      case MIFARE_IDLE: // attesa rfid_enb
      case MIFARE_GFWV: // lettura fw version
        if (mifare_read_swVer() == SL030_OPERATION_SUCCEED)
        {
          while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], WKEY_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
          smb0_enb = 1;
          mftx_mnum = 0;
          mifare_state = MIFARE_WKEY;
        }
        else 
        {
          if (mftx_mnum < TXNUM_MAX)
          {
            mftx_mnum ++;
            while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], MNUM_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
          }
          else
          {
            smb0_enb = 1;
            while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], DTCE_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
            mifare_state = MIFARE_DTCE;
          }
        }
        if (evs_mod == EVSM_MNET)
        {
          rfid_dis |= 1;			// per non resettare quando rfid_dis = 3
        }
        break;

      case MIFARE_WKEY: // scrittura keyAA
        if (mifare_wrte_keyw(SL030_ADDR0, mifare_KeyAA) == SL030_OPERATION_SUCCEED)  // scrittura andata a buon fine
        {
          while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], SNIF_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
          mifare_state = MIFARE_SNIF;
        }
        else if (mftx_mnum < TXNUM_MAX)
        {
          mftx_mnum ++;
          while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], MNUM_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
        }
        else
        {
          while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], DTCE_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
          mifare_state = MIFARE_DTCE;
        }
        break;

      case MIFARE_SNIF: // attesa presentazione mifare card
        if ((!rfid_dis) && (smb0_enb))
        {
          rfid_enb = 0;
          card_dtcd = 0;

          mfout = mifare_read_uidr(rfd_uid);

          if (mfout == SL030_OPERATION_SUCCEED)                   // lettura uid: esito comando tag in
          {
            if (SNP_VCT2 & MFRE_ANOM2)
            {
//              msg_upd = 1;  // potrebbe essere il posto dove mettere una send? Nick
            }

            SNP_VCT2 &= ~ MFRE_ANOM2;
            mftx_mnum = 0;

            if ((!rfid_enb) || (!rfd_uid[0]))
            {
              smb0_enb = 1;
              mifare_uidrst();
              while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], SNIF_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
              mifare_state = MIFARE_SNIF;
            }
            else 
            {
              if (evs_mod == EVSM_MNET)
              {
                smb0_enb = 1;
                mfpend = 1;
                while ((xTimerChangePeriod (xMifareTimers[MFPEND_TIM], T30S_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
                SNP_VCT0 |= RFID_STAT0;
                blnd_expired = 0; 
                while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], T1S_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
                mifare_state = MIFARE_BLND;
              }
              else 
              {
                while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], LGNR_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
                mifare_state = MIFARE_LGNR;
                if (mifare_lgin_secw(SL030_ADDR0, SC_INFO))// login settore: scrittura comando [EVSM_PERS || EVSM_FREE]
                {
                }
              }
            }
          }
          else 
          {
            if (mfout == 2)                // lettura uid: esito comando no tag
            {
//              if (SNP_VCT2 & MFRE_ANOM2)
//                msg_upd = 1;

              SNP_VCT2 &= ~ MFRE_ANOM2;

              smb0_enb = 1;
              mftx_mnum = 0;
              while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], SNIF_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
              mifare_state = MIFARE_SNIF;
            }
            else if (mftx_mnum < TXNUM_MAX)
            {
              mftx_mnum ++;
              while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], MNUM_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
            }
            else
            {
              smb0_enb = 1;
              while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], DTCE_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
              mifare_state = MIFARE_DTCE;
            }
          }
        }
        break;

      case MIFARE_LGNR:
        if (mifare_lgin_secr(SL030_ADDR0, SC_INFO) == SL030_OPERATION_SUCCEED) //login settore 2: lettura esito [EVSM_PERS || EVSM_FREE]
        {
          while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], RBKW_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
          mifare_state = MIFARE_RBKW;
        }
        else if (mftx_mnum < TXNUM_MAX)
        {
          mftx_mnum ++;
          while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], MNUM_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
        }
        else
        {
          smb0_enb = 1;
          while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], DTCE_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
          mifare_state = MIFARE_DTCE;
        }
        break;

      case MIFARE_RBKW: // lettura settore: lettura esito [EVSM_PERS || EVSM_FREE]
        if (mifare_read_blkr(SL030_ADDR0, BK_INFO, rfd_bki0) == SL030_OPERATION_SUCCEED)
        {
          smb0_enb = 1;
          SNP_VCT0 |= RFID_STAT0;  // // potrebbe essere il posto dove mettere una send a "personal.c"? Nick
          blnd_expired = 0;
          while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], T1S_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
          mifare_state = MIFARE_BLND;
        }
        else if (mftx_mnum < TXNUM_MAX)
        {
          mftx_mnum ++;
          while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], MNUM_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
        }
        else
        {
          smb0_enb = 1;
          while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], DTCE_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
          mifare_state = MIFARE_DTCE;
        }        
        break;

      case MIFARE_BLND: // disabilitazione temporanea dello sniff
        mifare_state = MIFARE_SNIF; // torno nello stato di attesa presentazione nuova card 
        break;

      case MIFARE_DTCE: // incoerenza / fallimento lettura fw version e scrittura Key
        setPowerRfid(OFF);

        if (ctrl_enb1 & MFRE_CRL1)
        {
          SNP_VCT2 |= MFRE_ANOM2;
        }
        while ((xTimerChangePeriod (xMifareTimers[MIFARE_TIM], T1S_TMEOUT, MIFARE_TIMER_GARD_TIME) != pdPASS)); 
        mifare_state = MIFARE_POFF;
        break;

      case MIFARE_POFF:
        setPowerRfid(ON);
        mifare_init();
        break;

      default:
        break;
    }
  } // end MIFARE_TIM

}



/*************** END OF FILE ******************************************************************************************/


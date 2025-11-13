/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Socket_RTOS/Inc/httpserver-socket.h
  * @author  MCD Application Team
  * @brief   header file for httpserver-socket.c
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HTTPSERVER_SOCKET_H__
#define __HTTPSERVER_SOCKET_H__

#define   TOTAL_K22_PIXEL           ((uint16_t)1976)
#define   K22_LED_PER_PIXEL         ((uint16_t)3)
#define   TOTAL_K22_LEDS            ((uint16_t)(TOTAL_K22_PIXEL * K22_LED_PER_PIXEL))

#define   TO_END_DWLD               ((uint16_t)1000)
#define   TO_END_CPLT_DWLD          ((uint16_t)48000)

#define   NUM_BUFF_WIFI_DWNL_RX     ((uint16_t)3)
//#define   NUM_BUFF_WIFI_DWNL_RX     ((uint16_t)1)
#define   DEF_WIFI_FILE_NAME        "ScameSCU_Vxxx.bin"

#define   LEN_OVERHEAD              ((uint32_t)4)   /* 2 byte per indice del pacchetto e 2 byte per CRC */
#define   WAIT_RESIDUAL_PACKET      ((uint32_t)3000)
#define   PACKET_FW_LEN             (2048) 
#define   PACKET_FW_TOT_DEFAULT_LEN (PACKET_FW_LEN + LEN_OVERHEAD)
#define   CODE_WIFI_LEN             ((uint16_t)(PACKET_FW_LEN + 16))

#define   NUM_BUFF_WIFI_UPG_RX      ((uint16_t)2)
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum
{
  FW_STATE_IDLE = 0,
  FW_STATE_RX,
  FW_DWNL_OTHER_RX
}fwState_e;

typedef enum
{
  WIFI_PK_LEN_DISCOVERY = 0,
  WIFI_PK_FULL_RECEIVED,
  WIFI_PK_PARTIAL_RECEIVED
}packetStatus_e;

typedef enum
{
  UPL_START = 0,
  UPL_DATA,
  UPL_DUMMY_EVENT
}wifiUplEv_e;

typedef enum
{
  ID_205_W11_B0 = 0,                            /* 0  205.W11-B0               */
  ID_205_W17_B0,                                /*    205.W17-B0/D0            */
  ID_205_W17_S0,                                /*    205.W17-S0/U0  CTRL1.4   */
  ID_205_T33_B0,                                /*    205.T33/37-B0/D0         */
  ID_205_T52_B0,                                /*    205.T52/62/74/85-B0/D0   */
  ID_205_T33_S0,                                /*    205.T33/37-S0/U0  CTRL   */
  ID_205_T52_S0,                                /* 6  205.T52/62/74/85-S0/U0   */ 
  ID_205_W36_B0,                                /*    205.W32/33/36/37-B0/D0   */
  ID_205_W85_B0,                                /*    205.W52/62/74/85-B0/D0   */
  ID_205_W36_S0,                                /*    205.W32/33/36/37-S0/U0   */
  ID_205_W85_S0,                                /* 10 205.W52/62/74/85-S0/U0   */

  ID_113_119_B0_D0,                             /* 11 205.T113/119-B0/D0       */
  ID_113_119_S0_U0,                             /* 12 205.T113/119-S0/U0       */
  ID_213_219_B0_D0,                             /* 13 205.T213/219-B0/D0       */
  ID_213_219_S0_U0,                             /* 14 205.T213/219-S0/U0       */
  ID_W113_119_B0_D0,                            /* 15 205.W113/119-B0/D0       */
  ID_W113_119_S0_U0,                            /* 16 205.W113/119-S0/U0       */
  ID_W213_219_B0_D0,                            /* 17 205.W213/219-B0/D0       */
  ID_W213_119_S0_U0,                            /* 18 205.W213/219-S0/U0       */

  ID_204_CA21B_T2T2,                            /* 19 204.CA21B/23B-T2T2       */
  ID_204_WD21B_T2T2,                            /* 20 204.WD21B/23B-T2T2       */
  ID_205_A33_DD,                                /* 21 205.A/33/52-62-BB/DD     */
  ID_TIC_LINKY,                                 /* 22 205.W319-B = TIC-LINKY   */
  ID_205_A33_UU,                                /* 23 205.A/33/52-62-UU        */
  ID_204_CA21B_T2xT2x,                          /* 24 204.CA21B/21B-T2xT2x     */
  ID_204_WD21B_T2xT2x,                          /* 25 204.WD21B/23B-T2xT2x     */
  ID_204_CA21B_UNUN,                            /* 26 204.CA21B-UNUN           */
  ID_204_UB21B_EB,                              /* 27 204.UB21B-EB             */
  ID_205_KB30_K,                                /* 28 205.KB30/50-B            */
  ID_205_Fx12_B_D,                              /* 29 205.205.Fx12-B/D  SOBEM  */
  ID_205_Fx33_BB_D,                             /* 30 205.Fx33-BB/D     SOBEM  */
  ID_205_Fx52_62_BB_DD,                         /* 31 205.Fx52/62-BB/DD SOBEM  */
  ID_29200_50,                                  /* 32 29200/50          SOBEM  */
  ID_29201_02_03_51_52_53,                      /* 33 29201/02/03/51/52/53 SOBEM  */
  ID_205_G37_BN_B0_DN,                          /* 34 205.G37-BN/B0/DN  SOBEM  */
  ID_205_G37_D0_G74_xx,                         /* 35 205.G37-D0/G74-xx SOBEM  */
  ID_28180_1_2_3,                               /* 36 205.Fx33-BB/D     SOBEM  */
  ID_204_CA_TFT_BCD,                            /* 37 204.CA49/59/67-B/C/D     */
  ID_204_CA_TFT_FGH,                            /* 38 204.CA49/59/67-F/G/H     */
  ID_204_CA_TFT_STU,                            /* 39 204.CA(TFT)-S/T/U        */
  ID_205_N_TFT_BCD,                             /* 40 205.N(TFT)-BCD           */
  ID_205_B33_DD,                                /* 41 205.B/33/52-62-DD        */
  ID_205_B33_UU,                                /* 42 205.B/33/52-62-UU        */
  ID_204_CA11B_T2,                              /* 43 204.CA11B/13B-T2T2       */
  ID_204_CA11B_T2x,                             /* 44 204.CA11B/13B-T2x        */
  ID_204_UB11B_EB,                              /* 45 204.UB11B-EB             */
  ID_204_CA_TFT_BCDT,                           /* 46 204.CA49/59/67-B/C/DT    */
  ID_END_MODEL
}idModel_e;



typedef __packed struct
{
  wifiUplEv_e         wifiUplEv;
  uint8_t*            pData;
  uint32_t            len;
} frameWifiUpl_st;

typedef __packed struct
{
  uint16_t    numBk;
  uint32_t    codeLen;
  uint32_t    currLen;
  uint32_t    currBuffIx;
  fwState_e   state;
  uint16_t    scuInDwldIdx;
} codeInfo_st;

typedef struct
{
  uint8_t  boot;
  uint32_t start;
  uint32_t size;
} FlashInfoFile;

typedef struct
{
  char scuPass[16];
  char ssid[32];
  char pass[64];
  char key[16];
} wifiWebInfo;

/* Exported variables --------------------------------------------------------*/
extern codeInfo_st   codeInfo;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void            http_server_socket_init           (void);
void            K22_httpd_init                    (void);
uint8_t         overlayPossible                   (void);
uint8_t         getThreshold                      (void);
void            send_to_fw_update_task            (frameWifiUpl_st pMsgToSend);
xQueueHandle    getScuWifiDwldQueueHandle         (void);
void            setBroadcastDownload              (uint8_t status);
uint8_t         getBroadcastDownload              (void);
void            setSlavaInDownload                (uint8_t status);
void            getIdModelFromStringAndParConfig  (char* pIdModel);
void            upgradeGeneralParameter           (idModel_e currModel, char currEmType, char currAddr);
int32_t         FirmwareUpdate_start              ( char *name, uint32_t size );
    
int32_t         FirmwareUpdate_put                ( uint8_t *data, uint32_t size );

int32_t         FirmwareUpdate_end                ( void );

void            setDowngradeStatus                (uint8_t dwngStatus);
uint8_t         getDowngradeStatus                (void);

#endif /* __HTTPSERVER_SOCKET_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

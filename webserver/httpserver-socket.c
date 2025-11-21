/******************* (C) COPYRIGHT 2024 SCAME  ******************************************************************/
/**
* @file        http_server.c
*
* @brief       Task for web server - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: httpserver-socket.c 25 2018-07-17 08:52:42Z nipi $
*
*     $Revision: 25 $
*
*     $Author: nipi $
*
*     $Date: 2018-07-17 10:52:42 +0200 (mar, 17 lug 2018) $
*
*
* @copyright
*       Copyright (C) 2024 Scame S.p.A. All rights reserved.
*       This file is copyrighted and the property of Scame S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  Scame S.p.A.
***********************************************************************************************************************/

/************************************************************
 * Include
 ************************************************************/
#include <stdio.h>
#include <ctype.h>

#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/sys.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"
#include "lwip/apps/fs.h"
#include "string.h"
#include "httpserver-socket.h"
#include "cmsis_os.h"
#include "TCP_socket.h"
#include "i2c.h"
#include "telnet.h"
#include "LcdMng.h"
#include "ethInitTask.h"
#include "adcTask.h"
#include "inputsMng.h"
#include "monitorMng.h"
#include "eeprom.h"
#include "EnergyMng.h"
#include "LcdMng.h"
#include "sbcGsy.h"
#include "prot_OnUsart.h"
#include "ledMng.h"
#include "displayPin.h"
#include "rtcApi.h"
#include "scuMdb.h"
#include "em_Task.h"
#include "flash_if.h"
#include "wrapper.h"
#include "ExtInpMng.h"
#include "RfidMng.h"
#include "ContactMng.h"
#include "sbcSem.h"
#include "cmox_crypto.h"
#include "configurator.h"
#include "secure_area.h"

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#define ATTIVA              1
#define DWL_TIMEOUT         1
#define SUSPEND_BEFORE_RED  1
#define SECURITY_PATTERN    1

// Indirizzo di partenza del FW da lanciare da boot
#define APPLICATION_ADDRESS_OLD_BOOT      ((uint32_t)0x08040000)
// boot size
#define SCAME_BOOT_SIZE             ((uint32_t)0x00008000)
// info FW offset  
#define OFFSET_INFO_FW_ADD          ((uint32_t)0x0000D000)

#define SECURE_AREA_SIZE            ((uint32_t)0x00004000)

#define SIGNATURE_AREA_SIZE         ((uint32_t)0x00000800) 

// Indirizzo di partenza del boot
#define BOOT_ADDRESS      ((uint32_t)0x08000000)
// Indirizzo massimo per il boot
#define BOOT_END_ADDRESS    ((uint32_t)(BOOT_ADDRESS + SCAME_BOOT_SIZE - (uint32_t)1))

// Indirizzo di partenza del FW da lanciare da boot
#define APPLICATION_ADDRESS     (BOOT_ADDRESS + SCAME_BOOT_SIZE + SECURE_AREA_SIZE )  /* 0x0800C000 */
#ifndef GD32F4xx
// Indirizzo massimo per il FW da lanciare da boot
#define APPLICATION_END_ADDRESS   ((uint32_t)0x0807FFFF)
// Indirizzo massimo per il FW da lanciare da boot (old)
#define APPLICATION_END_ADDRESS_OLD_BOOT    (uint32_t)0x0807FFFF
// Dimensione massima FW ST 0x0807.FFFF - 0x0800.8000 = 0x0007.8000 ovvero 512K - 32K = 480K
#define APPLICATION_MAX_SIZE   (APPLICATION_END_ADDRESS - APPLICATION_ADDRESS + 1)
// Dimensione massima FW+Boot  0x0808.0000 ovvero 512K 
#define APP_AND_BOOT_MAX_SIZE  (APPLICATION_END_ADDRESS - BOOT_ADDRESS + 1)
#else
// Indirizzo massimo per il FW da lanciare da boot
#define APPLICATION_END_ADDRESS   ((uint32_t)0x0809FFFF)
// Indirizzo massimo per il FW da lanciare da boot (old)
#define APPLICATION_END_ADDRESS_OLD_BOOT    (uint32_t)0x0809FFFF
// Dimensione massima FW GD 0x0809.FFFF - 0x0800.8000 = 0x0009.8000 ovvero 640K - 32K = 608K
#define APPLICATION_MAX_SIZE   (APPLICATION_END_ADDRESS - APPLICATION_ADDRESS + 1)
// Dimensione massima FW+Boot  0x080A.0000 ovvero 640K 
#define APP_AND_BOOT_MAX_SIZE  (APPLICATION_END_ADDRESS - BOOT_ADDRESS + 1)
#endif

// Indirizzo di partenza del FW in Flash
#define NEW_FW_ADDRESS      APPLICATION_ADDRESS                   /* 0x0800C000 */
// Indirizzo di partenza del FW da copiare vecchio boot
#define NEW_FW_ADDRESS_OLD_BOOT   (uint32_t)0x08080000

// Indirizzo di partenza del FW da copiare
#define NEW_FW_SDRAM_ADDRESS      ((uint32_t)0xC3100000)

// Indirizzo assoluto info sul FW in flash 
#define INFO_FLASH_ADDRESS			    (ORIGIN_ADDRESS + OFFSET_INFO_FW_ADD)   /* 0x0800D000 */

// Indirizzo relativo info sul FW  
#define INFO_OFFSET_ADDRESS			    (INFO_FLASH_ADDRESS - NEW_FW_ADDRESS) /* 0x0800D000 -  0x0800C000 = 0x1000*/

// Indirizzo massimo per il FW da copiare --> da settore 3 Add1=0x08018000 a settore 7 Add2= 0x080FFFFF (Add2-Add1)=E7FFF
#define NEW_FW_END_ADDRESS    ((uint32_t)0xC31E7FFF)

// Indirizzo massimo per il FW da copiare
#define NEW_FW_END_ADDRESS_OLD_BOOT   (uint32_t)0x080BFFFF

#define FUNTCTION_TO_LOAD   ((uint32_t)0x08017000)

// Dimensione massima FW (old boot/FW)
#define APPLICATION_MAX_SIZE_OLD_BOOT   (APPLICATION_END_ADDRESS_OLD_BOOT - APPLICATION_ADDRESS_OLD_BOOT + 1)

// Indirizzo relativo info TAG sul FW in flash (4 byte dopo la checksum )
#define INFO_OFFSET_TAG_ADDRESS	    (INFO_FLASH_ADDRESS + (uint32_t)(4) - NEW_FW_ADDRESS)  /* 0x0800D000 + 4 - 0x0800C000 = 0x1004*/

/* buffer len to verify checksum after complete upload */
#define  BUFF_FW_LEN   ((uint16_t)0x2000)

#define  TIMER_START_CODE      ((uint16_t)12000)

/* address for firmware version if present */
#define  FW_VERSION_OFFSET     0x00001000

/* address for boot version if present */
#define  BOOT_VERSION_OFFSET     0x00001000

/* size boot info */
#define     BOOT_INFO_SIZE          16
#define     BOOT_CKS_OFFSET         10


/* timeout to check the RTS485 status  */
#define     TIMER_RS485_CHECK      ((uint16_t)5000)

#define WEBSERVER_THREAD_PRIO    ( osPriorityAboveNormal )


/* UserID and Password definition *********************************************/
#define FILE_FOR_PWD          ((char *)"infoPwd.txt")
#define FILE_FOR_THR          ((char *)"infoThr.txt")
#define USERID                "scame"
#define PASSWORD              "Scame01"
#define MAX_PASS_LEN          ((uint8_t)32)
#define USERID_SERVICE        "service"
#define USERID_DEVELOPER      "developer"

#define LOGIN_SIZE   (20 + sizeof(USERID) + MAX_PASS_LEN)

#define OFFSET_SN_NEW    ((uint8_t)8)

/* file size for SCU firmware 512K = 524288 = 0x00080000 */

// Dimensione massima FW GD 0x0809.FFFF - 0x0800.8000 = 0x0009.8000 ovvero 640K - 32K = 608K
#define OFFSET_FILE_HTML     ((uint32_t)0x000000D2)
#define FILE_DOWNLOAD_SIZE   (APPLICATION_END_ADDRESS + OFFSET_FILE_HTML)
#define MIN_FILE_DWNL_SIZE   ((uint32_t)0x00040000)

/* upload file must have size < 10K Typically 50x50px bmp picture has size equal to 7654 bytes */
#define MAX_UPLOAD_FILE ((uint32_t)((uint32_t)10 * (uint32_t)1024))

#define OP_UPDATE_NONE              0x00
#define OP_UPDATE_MAIN_FW           0x01
#define OP_UPDATE_START_BIT         7
#define OP_UPDATE_WEB_START_BIT     6


#if LWIP_HTTPD_CUSTOM_FILES
#define GRAPHICS_   ((char*)"")
#else
#define GRAPHICS_   ((char*)"/graphics/")
#endif

// size of file buffer for reading a file
//#define HTTP_BUF_SIZE             ((uint16_t)0x8000)   /**** 0x8000=32K typical, 0x4000=16K only for debug */
#define   HTTP_BUF_SIZE             ((uint16_t)0x4000)

#define   NUM_BUFF_GSY_DWNL_RX                   ((uint16_t)1) 
#define   CODE_ERROR                             ((uint16_t)0xFFFF)
#define   CODE_STOP_DWNL                         ((uint16_t)0xFFFE)
#define   CODE_CKS_CODE_FAIL                     ((uint16_t)0xFFFD)
#define   CODE_OK_DWNL                           ((uint16_t)0xFFFC)

#define   TICK_PACKET_TO                         ((uint16_t)2000)

#define   NUM_PROD_CONF                          ((uint8_t)15)
#define   SIZE_CTRL_DATA                         ((uint8_t)5)

#ifdef MODBUS_TCP_EM_ETH
#define  MB_ADU_MAXSIZE                          ((uint16_t)32)
#define  NUM_BUFF_EM_LOVATO_ETH                  ((uint16_t)2)

#define  LOVATO_VL1_REG                          ((uint16_t)2)
#define  LOVATO_VL2_REG                          ((uint16_t)4)
#define  LOVATO_VL3_REG                          ((uint16_t)6)
#define  LOVATO_PA1_REG                          ((uint16_t)14)
#define  LOVATO_PA2_REG                          ((uint16_t)16)
#define  LOVATO_PA3_REG                          ((uint16_t)18)
#endif

/* Cryptographic defines */
#define SIGNATURE_SIZE CMOX_ECC_SECP256R1_SIG_LEN
uint8_t signature[SIGNATURE_SIZE];
/* SHA context */
cmox_sha256_handle_t sha256_ctx;
/* Computed data buffer */
uint8_t computed_hash[CMOX_SHA256_SIZE];
/* ECC context */
cmox_ecc_handle_t Ecc_Ctx;


typedef enum
{
  LoginPage = 0,
  mainPage,
  mainPagePOSTrx,
  FileUploadPage,
  UploadDonePage,
  helpPage,
  helpPagePOSTrx,
  postCkeckLogin,
  goHomePOSTrx,
  logoutPOSTrx,
  advanced,
  advancedPOSTrx,
  advRenameUplPOSTrx,
  advancedRename,
  advancedUpload,
  advancedPOSTviewRx,
  parameters,
  parametersPOSTrx,
  commandFailPage,
  advancedRemove,
  infoMacPOSTrx,
  infoHwConfRx,
  infoSpecFunc,
  modelConfFunc,
  sinapsiConf,
  passwordConf,
  wifiUpgrade,  
  changePassword,
} htmlpageState;

struct http_state
{
  char *file;
  u32_t left;
};

typedef  void (*pFunction)(void);

// Struttura utilizzata per la ricezione del nuovo firmware da attivare.
typedef struct
{
  unsigned char*  fwBuffer;
  unsigned int    fwPointer;
  unsigned int    checksum;
  unsigned char*  wdHandle;
  pFunction       upgradeFw;
  unsigned char   operation;
} FirwareUpgrade_t;

typedef __packed struct 
{  
  uint32_t            fwCheksum;
  uint8_t             strCheck[16];         
  uint8_t             fwVersion[16];         
  uint32_t            offsetAddress;
  uint32_t            fwLen;
}infoFwBoot_st;

typedef __packed struct
{
  uint16_t    emTipo:8;
  uint16_t    ledConf:2;
  uint16_t    idProduct:6;
} productConf_st;

typedef __packed union
{
  productConf_st           productConf;
  uint16_t                 productConf16;
} productConf_u;

#ifdef MODBUS_TCP_EM_LOVATO
typedef __packed struct
{
  uint32_t    vL1;
  uint32_t    vL2;
  uint32_t    vL3;
  uint32_t    pActiveL1;
  uint32_t    pActiveL2;
  uint32_t    pActiveL3;
} emLovatoToSem_st;
#endif


/*
*********************************** SCAME ************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
******************************************************************************
*/

u32_t               nPageHits = 0, nPageHitsWifi = 0;
unsigned char       SerNum[4];
FirwareUpgrade_t    fwUpgrade;
uint8_t             httpBuffer[HTTP_BUF_SIZE];
uint16_t            pIn, pInWifi,  pMax, pMaxWifi;
uint8_t             dataAvailable, numLoginError;
osThreadId_t        scuGsyDwldHandle, wifiUpgExpressifHandle;
xQueueHandle        scuGsyDwldQueue = NULL;
productConf_u       productConfUnion;
xQueueHandle        scuWifiDwldQueue = NULL;
xQueueHandle        wifiUpgExpressifQueue = NULL;

codeInfo_st         codeInfo;

infoFwBoot_st     infoFwRT;
uint32_t          totalLenRT, cksumRT, pAddrRT;

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
******************************************************************************
*/
static  htmlpageState htmlpage;
static  uint32_t      TotalReceived, ContentLengthOffset, BrowserFlag;

static  volatile uint32_t DataFlag;
static  volatile uint32_t size;
static  volatile uint32_t sFlashAddress;
static  volatile uint32_t TotalData;
static           char     pass[64];

static           char     *pFile, *pFileWrite;
static           char     *pTag1, *pRiga1, *pRiga2, *pRiga3, *pRiga4, *pRiga5, *pRiga6, *pRigaSwV, *pSinapsi, *pRigaBtV, *pTagFake, *pTemp;
static           char     *pHtmlData, *pPulsStop, *freeUse;
static    struct pbuf*    pBufHead;

static           char     *pCbox1[8], *pCbox2[8], *pCbox3[8], *pCbox1A[8], *pSock[12], *pEmeter[12], *pCboxGl[9], *pLang[9], *pHgpt[8];
static           char     *pAbPm[15],  *parPM[7],  *pLedS[4],  *pTime[4],  *pMode[4],  *pLcd[2], *pWiFi[7], *pOpMode[5], *pReadSp[4];
static           char     *pItypical, *pSempl, *pBackup, *pBusAdd, *pEmCode, *pAddMode, *pPresence[17];
static           char     *pHwConfData, *pSpecFuncData, *pSiConfData, *pSiBox[34], *pProdConfData, *pPassConf, *pWiFiUploading;


static  volatile uint8_t   resetpage;

static  uint8_t           httpFileFwName[32];
static  uint32_t          packetOverhead;

static  uint8_t           broadcastDwnl;

static  account_e         userType;
static  access_e          access;
static  uint8_t           downgradeStatus;

#ifdef MODBUS_TCP_EM_ETH
static struct       netconn *conn, *newconn;
static struct       netbuf *netbuf;
char                mb_req_buf[MB_ADU_MAXSIZE];
char                mb_repl_buf[MB_ADU_MAXSIZE];
emLovatoToSem_st    emLovatoToSem; 
#endif

#ifdef MODBUS_TCP_EM_LOVATO
xQueueHandle        emAnswMasterQueue = NULL;
#endif

#pragma data_alignment=4
__ALIGN_BEGIN static struct   fs_file heapFile; __ALIGN_END; /* heap file Buffer */


/*
*********************************** SCAME ************************************
**                                                                          **
**                            External  Variables                           **
**                                                                          **
******************************************************************************
*/

/* reserve 10K for HTTP message */
//extern  char     bufferFileWeb[4][0x2800];

extern unsigned short       EthernetTCP2Port;


extern uint32_t             counterSlaveDwnl;
extern emSinapsiInfo_st*    pEmSinapsiInfo;
extern frameEm_st           emMsg;
extern TimerHandle_t        xMdbUartTimers;

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local const                                   **
**                                                                          **
******************************************************************************
*/
/* Definitions for gsy download Task   */
const osThreadAttr_t scuGsyDwldTask_attributes = {
  .name = "GSY_DWLD_TASK",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 6
};
/* Definitions for WiFi Expressif upgrade Task   */
const osThreadAttr_t wifiUpgExpressifTask_attributes = {
  .name = "WIFI_UPG_TASK",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 2     
};


/* "\r\n--" */
static const char http_crnl_2[4] =  {0xd, 0xa, 0x2d, 0x2d};

/* "\r\n--" */
//static const char http_crlf_2[4] =  {0xd, 0xa,0xd,0xa};

/* "octet-stream" */
static const char octet_stream[14] = {0x6f, 0x63, 0x74, 0x65, 0x74, 0x2d, 0x73, 0x74, 0x72, 0x65, 0x61, 0x6d, 0x0d, };

/* Content Length */
static const char Content_Length[17] = {0x43, 0x6f, 0x6e, 0x74, 0x65, 0x6e, 0x74, 0x2d, 0x4c, 0x65, 0x6e, 0x67, 0x74, 0x68, 0x3a, 0x20, };

/* filename="ScameSCU_Vxx"  */
static const char filenameEq[] = "filename=\"";

static const char dwldDefFileName[] = "ScameSCU_X.bin";

static const char locPassScame[] = PASSWORD;

/* "Content-Type " */
//static const char Content_Type[] = "Content-Type: image/bmp\r\n\r\n";

static const char STR_ERROR_NONE[] =            "                    ";
static const char STR_ERR_RCDM_ANOM0[] =        " RCDM FAULT         ";
static const char STR_ERR_LID_ANOM0[] =         " LID FAULT          ";
static const char STR_ERR_VENT_ANOM0[] =        " VENT FAULT         ";
static const char STR_ERR_BLOCK_ANOM0[] =       " BLOCK FAULT        ";
static const char STR_ERR_NU04_ANOM0[] =        "                    ";
static const char STR_ERR_NU05_ANOM0[] =        "                    ";
static const char STR_ERR_MIRROR_ANOM0[] =      " MIRROR FAULT       ";
static const char STR_ERR_RCBO_ANOM0[] =        " RCBO FAULT         ";
static const char STR_ERR_CPSHORT_CRL1[] =      " CPS FAULT          ";
static const char STR_ERR_PPSHORT_CRL1[] =      " PPS FAULT          ";
static const char STR_ERR_CPLOST_CRL1[] =       " CPLOST FAULT       ";
static const char STR_ERR_PPLOST_CRL1[] =       " PPLOST FAULT       ";
static const char STR_ERR_VBUS_CRL1[] =         " VBUS FAULT         ";
static const char STR_ERR_MIFARE_CRL1[] =       " MIFARE FAULT       ";
static const char STR_ERR_EMETER_INT_CRL1[] =   " EM INT FAULT       ";
static const char STR_ERR_OVERCURRENT_CRL1[] =  " OVC FAULT          ";
static const char STR_ERR_RECTIFIER_CRL2[] =    " DIODO AC FAULT     ";
static const char STR_ERR_EMETER_EXT_CRL2[] =   " EMEX FAULT         ";

static const char* strErrors[] = { STR_ERROR_NONE,          STR_ERR_RCDM_ANOM0,       STR_ERR_LID_ANOM0,      STR_ERR_VENT_ANOM0,    STR_ERR_BLOCK_ANOM0,
                                   STR_ERR_NU04_ANOM0,      STR_ERR_NU05_ANOM0,       STR_ERR_MIRROR_ANOM0,   STR_ERR_RCBO_ANOM0,     STR_ERR_CPSHORT_CRL1,
                                   STR_ERR_PPSHORT_CRL1,    STR_ERR_CPLOST_CRL1,      STR_ERR_PPLOST_CRL1,    STR_ERR_VBUS_CRL1,      STR_ERR_MIFARE_CRL1,
                                   STR_ERR_EMETER_INT_CRL1, STR_ERR_OVERCURRENT_CRL1, STR_ERR_RECTIFIER_CRL2, STR_ERR_EMETER_EXT_CRL2};

static const uint8_t prodConfVal[ID_END_MODEL][SIZE_CTRL_DATA] = {
/*CTRL0, CTRL1,CTRL2,CTRL3,ATTUATIR see CONTROL_BYTE0_EADD and RCDM_CRL0 and more   */
  {0xBD, 0x9F, 0x01, 0x00, 0x48}, /*0  205.W11-B0                                   */          
  {0xBD, 0x9F, 0x01, 0x00, 0x48}, /*1  205.W17-B0/D0                                */
  {0xF5, 0x85, 0x01, 0x00, 0xC0}, /*2  205.W17-S0/U0  CTRL1.4=VBUS_CRL1=0           */  
  /* T33/... W32... change only number of led: 9 -- 6                               */        
  {0xBD, 0xFF, 0x01, 0x00, 0x48}, /*3  205.T33/37-B0/D0                             */          
  {0xBD, 0xFF, 0x01, 0x00, 0x48}, /*4  205.T52/62/74/85-B0/D0                       */          
  {0xF5, 0xE5, 0x03, 0x00, 0xC0}, /*5  205.T33/37-S0/U0  CTRL1.4=VBUS_CRL1=0        */          
  {0xF5, 0xE5, 0x03, 0x00, 0xC0}, /*6  205.T52/62/74/85-S0/U0  CTRL1.4=VBUS_CRL1=0  */          
  {0xBD, 0xFF, 0x01, 0x00, 0x48}, /*7  205.W32/33/36/37-B0/D0                       */          
  {0xBD, 0xFF, 0x01, 0x00, 0x48}, /*8  205.W52/62/74/85-B0/D0                       */          
  {0xF5, 0xE5, 0x03, 0x00, 0xC0}, /*9  205.W32/33/36/37-S0/U0  CTRL1.4=VBUS_CRL1=0  */          
  {0xF5, 0xE5, 0x03, 0x00, 0xC0}, /*10 205.W52/62/74/85-S0/U0  CTRL1.4=VBUS_CRL1=0  */          

  {0xBD, 0xDF, 0x03, 0x00, 0x48}, /*11 205.T113/119-B0/D0                           */          
  {0xF5, 0xC5, 0x03, 0x00, 0xC0}, /*12 205.T113/119-S0/U0  CTRL1.4=VBUS_CRL1=0      */          
  {0xBD, 0xDF, 0x03, 0x00, 0x48}, /*13 205.T213/219-B0/D0                           */          
  {0xF5, 0xC5, 0x03, 0x00, 0xC0}, /*14 205.T213/219-S0/U0  CTRL1.4=VBUS_CRL1=0      */          
  {0xBD, 0xDF, 0x03, 0x00, 0x48}, /*15 205.W113/119-B0/D0                           */          
  {0xF5, 0xC5, 0x03, 0x00, 0xC0}, /*16 205.W113/119-S0/U0  CTRL1.4=VBUS_CRL1=0      */          
  {0xBD, 0xDF, 0x03, 0x00, 0x48}, /*17 205.W213/219-B0/D0                           */          
  {0xF5, 0xC5, 0x03, 0x00, 0xC0}, /*18 205.W213/219-S0/U0  CTRL1.4=VBUS_CRL1=0      */     
       
  {0xBF, 0xFF, 0x01, 0x00, 0x48}, /*19 204.CA21B/21B-T2T2                           */          
  {0xBF, 0xFF, 0x01, 0x00, 0x48}, /*20 204.WD21B/23B-T2T2                           */   
  {0xBF, 0xFF, 0x01, 0x00, 0x48}, /*21 205.A/33/52-62-DD                          */          
  {0xBD, 0xDF, 0x03, 0x00, 0x48}, /*22 TIC-Linky                                    */          
  {0xFF, 0xF5, 0x03, 0x00, 0xC8}, /*23 205.A-B/33/52-62-UU  CTRL1.4=VBUS_CRL1=1     */          
  {0xF5, 0xF5, 0x01, 0x00, 0xC0}, /*24 ID_204_CA21B_T2xT2x  MirAtt, RCBOatt         */          
  {0xF5, 0xF5, 0x01, 0x00, 0xC0}, /*25 ID_204_WD21B_T2xT2x  MirAtt, RCBOatt         */          
  {0xBA, 0xF5, 0x00, 0x00, 0x48}, /*26 ID_204_CA21B_UNUN                            */          
  {0xBA, 0xF5, 0x00, 0x00, 0x48}, /*27 ID_204_UB21B_EB                              */          
  {0xB2, 0xF5, 0x00, 0x00, 0x48}, /*28 ID_205_KB30_K                                */          
  {0xBD, 0x9F, 0x01, 0x00, 0x48}, /*29 205.Fx12-B/D  SOBEM                      */   
  {0xBD, 0xFF, 0x01, 0x00, 0x48}, /*30 205.Fx33-BB/D     SOBEM                      */   
  {0xBD, 0xFF, 0x01, 0x00, 0x48}, /*31 205.Fx52/62-BB/DD SOBEM                      */   
  {0xBF, 0xFF, 0x01, 0x00, 0x48}, /*32 29200/50          SOBEM                      */   
  {0xBF, 0xFF, 0x01, 0x00, 0x48}, /*33 29201/02/03/51/52/53 SOBEM                   */   
  {0xBD, 0xFF, 0x01, 0x00, 0x48}, /*34 205.G37-BN/B0/DN  SOBEM                      */   
  {0xBD, 0xFF, 0x01, 0x00, 0x48}, /*35 205.G37-D0/G74-xx SOBEM                      */   
  {0xBD, 0xFF, 0x01, 0x00, 0x48}, /*36 205.Fx33-BB/D     SOBEM                      */   
  {0xBF, 0xFF, 0x01, 0x00, 0x48}, /*37 204.CA(TFT)-B/C/D                            */          
  {0xBF, 0xFF, 0x01, 0x00, 0x48}, /*38 204.CA(TFT)-F/G/H                            */          
  {0xF5, 0xF5, 0x01, 0x00, 0xC0}, /*39 204.CA(TFT)-S/T/U                            */          
  {0xBD, 0xFF, 0x01, 0x00, 0x48}, /*40 205.N(TFT)-BCD                               */          
  {0xBF, 0xFF, 0x03, 0x00, 0x48}, /*41 205.B/33/52-62-DD    CTRL1.4=VBUS_CRL1=1     */          
  {0xFF, 0xF5, 0x03, 0x00, 0xC8}, /*42 205.B/33/52-62-UU    CTRL1.4=VBUS_CRL1=1     */          
  {0xBF, 0xFF, 0x01, 0x00, 0x48}, /*43 204.CA11B/13B-T2T2                           */          
  {0xF5, 0xF5, 0x01, 0x00, 0xC0}, /*44 204.CA11B/13B-T2x                            */          
  {0xBA, 0xF5, 0x00, 0x00, 0x48}, /*45 ID_204_UB11B_EB                              */          
  {0xBD, 0xFF, 0x01, 0x00, 0x48}  /*46 204.CA(TFT)-B/C/D T                          */          
};                                                             
            
            
                                                   
/*                                                             
*********************************** SCAME ************************************
**                                                                          **
**                            External function                             **
**                                                                          **
******************************************************************************
*/

extern void                   setSysTickStatus              (uint16_t status);
extern areaConfPar_st  *      getPtrToConfParam             (void); 
                                                   
/*                                                             
*********************************** SCAME ************************************
**                                                                          **
**                            Local function                                **
**                                                                          **
******************************************************************************
*/

static uint32_t Parse_Content_Length(char *data, uint32_t len);

static void         conn_err        (void *arg, err_t err);
static void         send_data       (struct tcp_pcb *pcb, struct http_state *hs);
static err_t        http_poll       (void *arg, struct tcp_pcb *pcb);
static err_t        http_recv       (void *arg, struct tcp_pcb *pcb,  struct pbuf *p, err_t err);
static err_t        http_accept     (void *arg, struct tcp_pcb *pcb, err_t err);
static void         writePcb        (struct http_state *hs, struct fs_file *pFile,  struct pbuf *p, struct tcp_pcb *pcb);
static void         freeAllpbuf     (struct pbuf *p);
static struct pbuf* getNextBuf      (struct pbuf *p);
static char         CheckNewFW      (void);
static void         scuGsyDwldTask  (void * pvParameters);
static uint8_t      checksumOnBlock (uint8_t* pBuff, uint16_t len);
static void         checkAndProgBoot(void);
static char         CheckSignature(FlashInfoFile * const info);
static int8_t       compareVersions(char *current, char *next);

#ifdef MODBUS_TCP_EM_ETH
static void         copyString      (char* dest, char* src, unsigned num);
static void         tcp_thread      (void *arg);
static uint16_t     mb_process      (char *mb_repl_buf, char *mb_req_buf, uint16_t req_buf_len,  uint16_t regAddr); 
#endif
static void         sendMsgToStartUpgradeModule   (void);
static void         wifiUpgExpressifTask          (void * pvParameters);
static int          hex_to_int                    (char c); 
static char*        url_decode                    (const char* encoded_str);

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/


/**
  * @brief  Extract the Content_Length data from HTML data
  * @param  data : pointer on receive packet buffer
  * @param  len  : buffer length
  * @retval size : Content_length in numeric format
  */
static uint32_t Parse_Content_Length(char *data, uint32_t len)
{
  uint32_t i = 0, size = 0, S = 1;
  int32_t j = 0;
  char sizestring[6], *ptr;

  ContentLengthOffset = 0;

  /* find Content-Length data in packet buffer */
  for (i = 0; i < len; i++)
  {
    if (strncmp ((char*)(data + i), Content_Length, 16) == 0)
    {
      ContentLengthOffset = i + 16;
      break;
    }
  }

  /* read Content-Length value */
  if (ContentLengthOffset)
  {
    i = 0;
    ptr = (char*)(data + ContentLengthOffset);

    while(*(ptr + i) != 0x0d)
    {
      sizestring[i] = *(ptr + i);
      i++;
      ContentLengthOffset++;
    }

    if (i > 0)
    {
      /* transform string data into numeric format */
      for(j = i - 1; j >= 0; j--)
      {
        size += (sizestring[j] - '0') * S;
        S = S * 10;
      }
    }
  }

  return size;
}

/**
  * @brief  Copy max 64 byte from SDRAM to Ram using word align
  * @param  pSdram : pointer in SDRAM
  * @param  pRam : pointer in up RAM
  * @param  len  : data to write
  * @retval none
  */
__weak void cpyFromSdramToRam (void* pSdram, void* pRam, uint8_t len)
{
  uint32_t  rem;
  char      ramBuff[64];

  rem = ((uint32_t)pSdram % 4);

  pSdram = (void*)((uint32_t)pSdram - (uint32_t)rem);
  /*       destination       source     len  */
  memcpy((void*)ramBuff, (void*)pSdram, len + rem);
  /*       destination  source     len  */
  memcpy((void*)pRam, (void*)&ramBuff[rem], len + rem);
}




/************************ TCP SOCKET *****************************/

/**
  * @brief  callback function for handling connection errors
  * @param  arg: pointer to an argument to be passed to callback function
  * @param  err: LwIP error code
  * @retval None
  */
static void conn_err(void *arg, err_t err)
{
  struct http_state *hs;

  hs = arg;
  mem_free(hs);
}

/**
  * @brief  closes tcp connection
  * @param  pcb: pointer to a tcp_pcb struct
  * @param  hs: pointer to a http_state struct
  * @retval
  */
static void close_conn(struct tcp_pcb *pcb, struct http_state *hs)
{
  tcp_arg(pcb, NULL);
  tcp_sent(pcb, NULL);
  tcp_recv(pcb, NULL);
  mem_free(hs);
  tcp_close(pcb);
}


/**
  * @brief sends data found in  member "file" of a http_state struct
  * @param pcb: pointer to a tcp_pcb struct
  * @param hs: pointer to a http_state struct
  * @retval None
  */
static void send_data(struct tcp_pcb *pcb, struct http_state *hs)
{
  err_t err;
  u16_t len;

  /* We cannot send more data than space available in the send
     buffer */
  if (tcp_sndbuf(pcb) < hs->left)
  {
    len = tcp_sndbuf(pcb);
  }
  else
  {
    len = hs->left;
  }

  err = tcp_write(pcb, hs->file, len, 0);

  if (err == ERR_OK)
  {
    hs->file += len;
    hs->left -= len;
  }
}

/**
  * @brief callback function called after a successfull TCP data packet transmission
  * @param arg: pointer to an argument to be passed to callback function
  * @param pcb: pointer on tcp_pcb structure
  * @param len
  * @retval err : LwIP error code
  */
static err_t http_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
  struct http_state *hs;

  hs = arg;

  if (hs->left > 0)
  {
    send_data(pcb, hs);
  }
  else
  {
    close_conn(pcb, hs);

    if(resetpage == 1)
    {
      setFlagForNvic();
      /* Generate a software reset */
      NVIC_SystemReset();

      while(1);
    }
  }

  if (heapFile.data != NULL)
  {
    //fs_close(&heapFile);
  }

  return ERR_OK;
}


/**
  * @brief tcp poll callback function
  * @param arg: pointer to an argument to be passed to callback function
  * @param pcb: pointer on tcp_pcb structure
  * @retval err_t
  */
static err_t http_poll(void *arg, struct tcp_pcb *pcb)
{
  if (arg == NULL)
  {
    tcp_close(pcb);
  }
  else
  {
    send_data(pcb, (struct http_state *)arg);
  }

  return ERR_OK;
}

/**
  * @brief  send a message on HTTP
  * @param  arg: pointer to an argument structure to be passed to callback function
  * @param  fs_file: pointer to a file structure
  * @retval none
  */
static void writePcb(struct http_state *hs, struct fs_file *pFile,  struct pbuf *p, struct tcp_pcb *pcb)
{

  hs->file = (char*)pFile->data;
  hs->left = pFile->len;
  freeAllpbuf(p);

  /* send index.html page */
  send_data(pcb, hs);

  /* Tell TCP that we wish be to informed of data that has been
  successfully sent by a call to the http_sent() function. */
  tcp_sent(pcb, http_sent);
}

/**
  * @brief serve tcp connection
  * @param conn: connection socket
  * @param struct pbuf: see pbuf.h  line 186
  * @retval err_t: error code 0 if all OK
  */
static err_t http_recv(void *arg, struct tcp_pcb *pcb,  struct pbuf *p, err_t err)
{
  int32_t               i, normLen, len = 0;
  uint32_t              DataOffset;
  char                  *data, *ptr, *pAct, *pHgtp, *pSocket, *pEm, *pImin, *pLs, *pOm;
  char                  *pHpwr, *pDset, *pDmax, *pUnb, *pEmx, *pRfas, *pAbCT, *pMaxTric, *pBack, *pAddr, *pGMT, *pDST, *pSemFlags; 
  char                  *pDeT, *pModeOp, *pLcdType, *pWiFiMode, *pDiRiMode, *pMaxEnergy, *pWDef, *pSkPos, *pChId;
  struct                http_state *hs;
  char                  cnt, dotFake;
  uint64_t              checkFlag;
  uint8_t               result, in, userFound, passFound, *user;
  sck_error_e           sckState;
  wifiWebInfo*          pWifiWebInfo;

  hs = arg;
  pass[0] = '\0';
  strcat((char *)pass, GRAPHICS_);

  if (err == ERR_OK && p != NULL)
  {
    
    if (strncmp(p->payload, "GET /FwVersion", 14) == 0)
    {
      return configurator_get_version(arg, pcb, p, err);
    }
    else if (strncmp(p->payload, "POST /Config", 12) == 0) {
      err = configurator_init_config_reception(arg, pcb, p, err);
      if (err != ERR_OK)
        configurator_deinit_config_reception(arg, pcb, p, err);
      return err;
    }
    
    /* Inform TCP that we have taken the data */
    tcp_recved(pcb, p->tot_len);
    dataAvailable = (uint8_t)TRUE;
    pBufHead = p;

    while(dataAvailable == (uint8_t)TRUE)
    {
      if (hs->file == NULL)
      {
        data = p->payload;
        // len = p->tot_len;  /* total len, see struct pbuf */
        len = p->len;  // length of this buffer

        if (p->next != NULL)
        {
          tPrintf( "Tot_Len = %u Part Len = %u\r\n", p->tot_len, p->len);
          osDelay(100);
        }
        
        /*if(Configurator_handle(p) == 0)
        {
          heapFile.data = Configurator_getData();
          heapFile.len = Configurator_getDataSize();
          writePcb(hs, &heapFile, p, pcb);
          if(Configurator_rebootRequire() != 0)
          {
            activeImmediateReset();
          }
        }*/
        if(((strncmp((char *)data, "GET /login.html", 18) == 0) || (strncmp((char *)data, "GET / ", 6) == 0)) ||
           ((strncmp((char *)data, "GET /", 5) == 0) && (htmlpage == LoginPage)))
        {
          nPageHits = 0;
          pIn = (uint16_t)0;
          /* Load mainPage/index page */
          fs_open(&heapFile, "/login.html");
          /*send the login page (which is the index page) */
          htmlpage = LoginPage;
          writePcb(hs, &heapFile, p, pcb);
        }
        else if (((strncmp(data, "POST /checklog.cgi",18)==0) && (htmlpage == LoginPage)) || (htmlpage == postCkeckLogin))
        {
          user = (uint8_t*)httpFileFwName;

          /* parse packet for the username & password  */
          for (i=0, result = 0, userFound = 0, passFound = 0; i<len; i++)
          {
            if ((strncmp ((char*)(data+i), "username=", 9)==0) && (userFound == 0))
            {
              userFound = 1;
              while ((char)data[i] != (char)'=') i++;
              i++;
              in = 0;
              while (((char)data[i] != '&') && (in < MAX_PASS_LEN))
              {
               user[in] = (char)data[i];
               i++;
               in++;
              }
              if ( (strncmp((char*)user, (char *)USERID, in)==0) && (in != 0))
              {
                userType = ACCOUNT_COLLAUDO;
                continue;
              }
              else
              {
                if ( (strncmp((char*)user, (char *)USERID_DEVELOPER, in)==0) && (in != 0))
                {
                  userType = ACCOUNT_DEVELOPER;
                  continue;
                }
                else
                {
                  if ( (strncmp((char*)user, (char *)USERID_SERVICE, in)==0) && (in != 0))
                  {
                    userType = ACCOUNT_SERVICE;
                    continue;
                  }
                  else
                  {
                    numLoginError++;
                  }
                }
                break;
              }
            }
            else
            {
              if (userFound != 0)
              {
                access = ACCESS_ALLOWED;
                if (userFound == 1)
                {
                  if (userType == ACCOUNT_COLLAUDO)
                  {
                    /* the user try access with collaudo account: apply restriction */
                    DataOffset = getCurrentUnixTime(); normLen = (int32_t)getStationStartTimeWebCollaudo();
                    if (normLen == DUMMY_INFO_VAL) saveStartTimeWebCollaudo(DataOffset);
                    else if (normLen == (int32_t)0) access = ACCESS_DENIED; 
#ifdef SUSPEND_BEFORE_RED
                    else if ((DataOffset - normLen) > VALIDITY_PERIOD_COLLAUDO_USER) access = ACCESS_DENIED;
#endif
                  }
                }

                if ((strncmp ((char*)(data+i), "password=", 9)==0) && (passFound == 0) && (access == ACCESS_ALLOWED))
                {
                  passFound = 1;
                  while ((char)data[i] != (char)'=') i++;
                  i++;
                  in = 0;
                  while (((char)data[i] != (char)'&') && (in < MAX_PASS_LEN))
                  {
                   pass[in] = (char)data[i];
                   i++;
                   in++;
                  }
                  pass[in] = '\0';
                  /* convert %xx to special char */
                  pSiBox[0] = url_decode(pass);
                  if (pSiBox[0] != NULL)
                  {
                    /*         destination       source */
                    strcpy((char *)pass, (char *)pSiBox[0]);
                    free(pSiBox[0]);
                  }
                  in = (uint8_t)strlen(pass);
                  switch (userType)
                  {
                    case ACCOUNT_COLLAUDO:
                      if (access == ACCESS_ALLOWED)
                      {
                        /* there are the condition for "collaudo" account to enter in the server */
                        if (((strncmp((char*)pass, (char *)locPassScame, in)==0)) && (in != 0))
                          access = ACCESS_ALLOWED;
                        else
                          access = ACCESS_DENIED;
                      }
                      break;

                    case ACCOUNT_DEVELOPER:
                      access = checkSviluper(pass, (uint16_t)in);
                      break;

                    case ACCOUNT_SERVICE:
                      /* check if there are the condition for "service" account to enter in the server */
                      if(SecureArea_verifyScuPass(pass) == 0)
                        access = ACCESS_ALLOWED;
                      else
                        access = ACCESS_DENIED;
                      break;

                    default:
                      access = ACCESS_DENIED;
                      numLoginError++;
                      break;
                  }
                  if (access == ACCESS_ALLOWED)
                  {
                    numLoginError = 0;
                    result = 1;
                    htmlpage = mainPage;
                    fs_open(&heapFile, "/mainPage.html");
                    /* modify the tag inside html page */
                    /* modify the tag inside html page */
                    if (pTag1 == NULL)
                    {
                      pHtmlData = malloc(heapFile.len);

                      if (pHtmlData != NULL)
                      {
                        memcpy(pHtmlData, heapFile.data, heapFile.len);
                        pTag1     = strstr(pHtmlData, (char*)"02:80:E1");
                        pRiga1    = strstr(pHtmlData, (char*)"SCUriga#1");
                        pRiga2    = strstr(pHtmlData, (char*)"SCUriga#2");
                        pRiga3    = strstr(pHtmlData, (char*)"SCUriga#3");
                        pRiga4    = strstr(pHtmlData, (char*)"SCUriga#4");
                        pRiga5    = strstr(pHtmlData, (char*)"SCUriga#5");
                        pRiga6    = strstr(pHtmlData, (char*)"SCUriga#6");
                        pRigaSwV  = strstr(pHtmlData, (char*)"Vx.y.zza");
                        pRigaBtV  = strstr(pHtmlData, (char*)"Vw.ya");
                        pSinapsi  = strstr(pHtmlData, (char*)"disabl#d"); 
                        pPulsStop = strstr(pHtmlData, (char*)"disa#led"); 
                        pTagFake  = strstr(pHtmlData, (char*)"204.CA");
                        pAddMode  = strstr(pHtmlData, (char*)"di#abled");
                        pAddMode  = strstr(pHtmlData, (char*)"di#abled");
                        pSiBox[0] = strstr(pHtmlData, (char*)"hidden0");
                        pSiBox[1] = strstr(pHtmlData, (char*)"hidden1");
                        pSiBox[2] = strstr(pHtmlData, (char*)"SCAME77777");  // SSID
                        pSiBox[3] = strstr(pHtmlData, (char*)"SCAME88888");  // AP password
                        pSiBox[4] = strstr(pHtmlData, (char*)"KEY99");       // Actyvation key
                      }
                    }

                    if (pTag1 != NULL)
                    {
                      heapFile.data = (const char*)pHtmlData;
                      // Il MAC Address a differenza del resto è sulla EEPROM a bordo
                      eeprom_param_get(SERNUM_BYTE0_EADD, SerNum, 4);
                      /*                                      100xxxxxx       </strong> */
                      memcpy((void*)pTag1, (void*)"FFFFFFFF / 5000            ", (size_t)27);
                      if (SerNum[3] != 0xFF)
                      {
                        pTag1[1] = (SerNum[0] & 0x0F) + '0';
                        pTag1[0] = ((SerNum[0] >> 4) & 0x0F)  + '0';
                      }
                      if (SerNum[2] != 0xFF)
                      {
                        pTag1[3] = (SerNum[1] & 0x0F) + '0';
                        pTag1[2] = ((SerNum[1] >> 4) & 0x0F)  + '0';
                      }
                      if (SerNum[1] != 0xFF)
                      {
                        pTag1[5] = (SerNum[2] & 0x0F) + '0';
                        pTag1[4] = ((SerNum[2] >> 4) & 0x0F)  + '0';
                      }
                      if (SerNum[0] != 0xFF)
                      {
                        pTag1[7] = (SerNum[3] & 0x0F) + '0';
                        pTag1[6] = ((SerNum[3] >> 4) & 0x0F)  + '0';
                      }
                      freeUse = getProductSerialNumberEeprom();
                      if (freeUse[0] != 0xFF)
                      {
                        size = strlen(freeUse);
                        // Il PRD SN Address a differenza del resto è sulla EEPROM a bordo
                        memcpy((void*)&pTag1[11], (void*)freeUse, (size_t)size);
                      }
                      /* all blank in this field */
                      memset ((void*)pTagFake, ' ',35);
                      freeUse = getStationProductCodeString();
                      if (freeUse[0] != 0xFF)
                      {
                        size = strlen(freeUse);
                        // copy the product code like this: 204.CA23B-T2T2W1
                        memcpy((void*)pTagFake, (void*)freeUse, (size_t)size);
                      }
                      pTemp = (char*)((uint32_t)pTagFake + size + (uint32_t)1);
                      pass[0] = pass[2] = ' '; pass[1] = '/'; 
                      memcpy((void*)pTemp, (void*)pass, (size_t)3);
                      pTemp += (4);
#ifdef SECURITY_PATTERN
                      /* it is necessary to disable menù for RED paramenter */
                      memcpy((void*)pSiBox[0], (void*)"hidden ", (size_t)7);
                      memcpy((void*)pSiBox[1], (void*)"hidden ", (size_t)7);
#else
                      if ((strstr(freeUse, (char*)"W119") == NULL) && (strstr(freeUse, (char*)"W219")))
                      {
                        /* it is necessary to disable menù for RED paramenter */
                        memcpy((void*)pSiBox[0], (void*)"hidden ", (size_t)7);
                        memcpy((void*)pSiBox[1], (void*)"hidden ", (size_t)7);
                      }
                      else
                      {
                        (void)SecureArea_getLocalApSSID(pSiBox[2], (uint32_t)10);
                        (void)SecureArea_getLocalApPass(pSiBox[3], (uint32_t)10);
                        (void)SecureArea_getActivationKey(pSiBox[4],(uint32_t)5);
                      }
#endif                      
                      freeUse = getStationFakeCodeCodeString();
                      if (freeUse[0] != 0xFF)
                      {
                        size = strlen(freeUse);
                        // copy the fake code like this: 204.CA.51.FFFF
                        memcpy((void*)pTemp, (void*)freeUse, (size_t)size);
                      }
                    }
                    else
                    {
                      free(pHtmlData);
                      pHtmlData = NULL;
                    }

                    /* first LCD string */
                    ptr = getLineString((uint8_t)1, (uint8_t*)&cnt);
                    /*       destination       source     20   */
                    memcpy((void*)pRiga1, (void*)ptr, (size_t)cnt);

                    /* second LCD string */
                    ptr = getLineString((uint8_t)2, (uint8_t*)&cnt);
                    /*       destination       source     20   */
                    memcpy((void*)pRiga2, (void*)ptr, (size_t)cnt);

                    /* first scrolling line  LCD string */
                    ptr = getLineString((uint8_t)3, (uint8_t*)&cnt);
                    /*       destination       source     20   */
                    memcpy((void*)pRiga3, (void*)ptr, (size_t)cnt);

                    /* second scrolling line  LCD string */
                    ptr = getLineString((uint8_t)4, (uint8_t*)&cnt);
                    /*       destination       source     20   */
                    memcpy((void*)pRiga4, (void*)ptr, (size_t)cnt);

                    /* third scrolling line  LCD string */
                    ptr = getLineString((uint8_t)5, (uint8_t*)&cnt);
                    /*       destination       source     20   */
                    memcpy((void*)pRiga5, (void*)ptr, (size_t)cnt);

                    /* third scrolling line  LCD string: Error line */
                    sckState = getErrorStateCoding();
                    if (sckState == ERROR_NONE)
                    {
                      /* third scrolling line  LCD string */
                      ptr = getLineString((uint8_t)8, (uint8_t*)&cnt);
                      /*       destination       source     20   */
                      memcpy((void*)pRiga6, (void*)ptr, (size_t)cnt);
                    }
                    else
                    {
                      /*       destination       source                               20   */
                      memcpy((void*)pRiga6, (void*)strErrors[(uint8_t)sckState], (size_t)20);
                    }

                    /*       destination       source           8 = len("Vx.y.zza HWvAB")   */
                    memset ((void*)pRigaSwV, ' ', 8);
                    i = strlen((char*)getFwVer());
                    memcpy((void*)pRigaSwV, (void*)getFwVer(), (size_t)8);
                    memcpy((void*)&pRigaSwV[12], (void*)getScuHWverFromEeprom(), (size_t)2);
                    

                    /*       destination       source           5 = len("Vw.ya" or "Vw.y\0")   */
                    memcpy((void*)pRigaBtV, (void*)BOOT_ADDR_VER, (size_t)BOOT_VER_SIZE);
                    for (i = 0; i < BOOT_VER_SIZE; i++)
                    {
                      if (pRigaBtV[i] == '\0')
                      {
                        pRigaBtV[i] = ' ';
                      }
                    }

                    if (get_emeter_type(EXTERNAL_EM) == EMETER_SINAPSI) 
                    {
                      /*       destination       source           8 = len("        ")   */
                      memcpy((void*)pSinapsi, (void*)"        ", (size_t)8);
                    }
                    else
                    {
                      /*       destination       source           8 = len("disabl#d")   */
                      memcpy((void*)pSinapsi, (void*)"disabled", (size_t)8);
                    }

                    /* enable stop charge button only in FREE and when in charging */
                    if ((evs_mode_get() == EVS_FREE_MODE) && (evs_state_get() == EVSTATE_CHARGING))
                    {
                        memcpy((void*)pPulsStop, (void*)"        ", (size_t)8);
                    }
                    else
                    {
                      /*       destination       source           8 = len("disabl#d")   */
                      memcpy((void*)pPulsStop, (void*)"disabled", (size_t)8);
                    }
                    if (isSemMasterFz() == FALSE)
                    {
                      /*       destination       source           8 = len("di#abled")   */
                      memcpy((void*)pAddMode, (void*)"disabled", (size_t)8);
                    }

                    /* send page */
                    writePcb(hs, &heapFile, p, pcb);
                    break;
                  }
                  else
                  {
                    numLoginError++;
                  }
                }
              }
            }
          } // for i < len 
          if ((result == 0)  && (userFound == 1) && (htmlpage != changePassword))
          {
            htmlpage = LoginPage;
            /* reload index.html */
            fs_open(&heapFile, "/login.html"); 
            writePcb(hs, &heapFile, p, pcb);
            if (numLoginError >= 3)
            {
              suspendEthPollingCheck(ETH_EVENT_SUSPEND_TIME);    // stop check eth registers
              numLoginError = 0;
            }
          }
          else
          {
            if ((passFound == 0)  && (userFound == 0))
            {
              pbuf_free(p);
            }
          }
        }
        else if((strncmp((char *)data, "GET /mainPage.html", 18) == 0) || (strncmp((char *)data, "POST /mainPage.html", 19) == 0) ||
                (((strncmp((char *)data, "GET /checklog.cgi", 17) == 0)) && (htmlpage == mainPage)))
        {
          nPageHits = 0;
          pIn = (uint16_t)0;
          /* Load mainPage/index page */
          fs_open(&heapFile, "/mainPage.html");

          freeHtmlMemory();
          /* modify the tag inside html page */
          if (pTag1 == NULL)
          {
            pHtmlData = malloc(heapFile.len);

            if (pHtmlData != NULL)
            {
              memcpy(pHtmlData, heapFile.data, heapFile.len);
              pTag1 = strstr(pHtmlData, (char*)"02:80:E1");
              pRiga1 = strstr(pHtmlData, (char*)"SCUriga#1");
              pRiga2 = strstr(pHtmlData, (char*)"SCUriga#2");
              pRiga3 = strstr(pHtmlData, (char*)"SCUriga#3");
              pRiga4 = strstr(pHtmlData, (char*)"SCUriga#4");
              pRiga5 = strstr(pHtmlData, (char*)"SCUriga#5");
              pRigaSwV = strstr(pHtmlData, (char*)"Vx.y.zz");
              pRigaBtV = strstr(pHtmlData, (char*)"Vw.ya");
            }
          }

          if (pTag1 != NULL)
          {
            heapFile.data = (const char*)pHtmlData;
            // Il MAC Address a differenza del resto è sulla EEPROM a bordo
            eeprom_param_get(SERNUM_BYTE0_EADD, SerNum, 4);
            /*       destination       source     normLen  */
            /*                                      100xxxxxx       </strong> */
            memcpy((void*)pTag1, (void*)"FFFFFFFF / 5000            ", (size_t)27);
            if (SerNum[3] != 0xFF)
            {
              pTag1[1] = (SerNum[0] & 0x0F) + '0';
              pTag1[0] = ((SerNum[0] >> 4) & 0x0F)  + '0';
            }
            if (SerNum[2] != 0xFF)
            {
              pTag1[3] = (SerNum[1] & 0x0F) + '0';
              pTag1[2] = ((SerNum[1] >> 4) & 0x0F)  + '0';
            }
            if (SerNum[1] != 0xFF)
            {
              pTag1[5] = (SerNum[2] & 0x0F) + '0';
              pTag1[4] = ((SerNum[2] >> 4) & 0x0F)  + '0';
            }
            if (SerNum[0] != 0xFF)
            {
              pTag1[7] = (SerNum[3] & 0x0F) + '0';
              pTag1[6] = ((SerNum[3] >> 4) & 0x0F)  + '0';
            }
            freeUse = getProductSerialNumberEeprom();
            if (freeUse[0] != 0xFF)
            {
              size = strlen(freeUse);
              // Il PRD SN Address a differenza del resto è sulla EEPROM a bordo
              memcpy((void*)&pTag1[11], (void*)freeUse, (size_t)size);
            }
            /* enable stop charge button only in FREE and when in charging */
            if ((evs_mode_get() == EVS_FREE_MODE) && (evs_state_get() == EVSTATE_CHARGING))
            {
                memcpy((void*)pPulsStop, (void*)"        ", (size_t)8);
            }
            else
            {
              /*       destination       source           8 = len("disabl#d")   */
              memcpy((void*)pPulsStop, (void*)"disabled", (size_t)8);
            }
            /* all blank in product and fake code  field */
            memset ((void*)pTagFake, ' ',35);
            freeUse = getStationProductCodeString();
            if (freeUse[0] != 0xFF)
            {
              size = strlen(freeUse);
              // copy the product code like this: 204.CA23B-T2T2W1
              memcpy((void*)pTagFake, (void*)freeUse, (size_t)size);
            }
            pTemp = (char*)((uint32_t)pTagFake + size + (uint32_t)1);
            pass[0] = pass[2] = ' '; pass[1] = '/'; 
            memcpy((void*)pTemp, (void*)pass, (size_t)3);
            pTemp += (4);

            freeUse = getStationFakeCodeCodeString();
            if (freeUse[0] != 0xFF)
            {
              size = strlen(freeUse);
              // copy the fake code like this: 204.CA.51.FFFF
              memcpy((void*)pTemp, (void*)freeUse, (size_t)size);
            }
          }
          else
          {
            free(pHtmlData);
            pHtmlData = NULL;
          }

          /* first LCD string */
          ptr = getLineString((uint8_t)1, (uint8_t*)&cnt);
          /*       destination       source     20   */
          memcpy((void*)pRiga1, (void*)ptr, (size_t)cnt);

          /* second LCD string */
          ptr = getLineString((uint8_t)2, (uint8_t*)&cnt);
          /*       destination       source     20   */
          memcpy((void*)pRiga2, (void*)ptr, (size_t)cnt);

          /* first scrolling line  LCD string */
          ptr = getLineString((uint8_t)3, (uint8_t*)&cnt);
          /*       destination       source     20   */
          memcpy((void*)pRiga3, (void*)ptr, (size_t)cnt);

          /* second scrolling line  LCD string */
          ptr = getLineString((uint8_t)4, (uint8_t*)&cnt);
          /*       destination       source     20   */
          memcpy((void*)pRiga4, (void*)ptr, (size_t)cnt);

          /* third scrolling line  LCD string */
          ptr = getLineString((uint8_t)5, (uint8_t*)&cnt);
          /*       destination       source     20   */
          memcpy((void*)pRiga5, (void*)ptr, (size_t)cnt);

          /* forth scrolling line  LCD string: Error line */
          sckState = getErrorStateCoding();
          if (sckState == ERROR_NONE)
          {
            /* third scrolling line  LCD string */
            ptr = getLineString((uint8_t)8, (uint8_t*)&cnt);
            /*       destination       source     20   */
            memcpy((void*)pRiga6, (void*)ptr, (size_t)cnt);
          }
          else
          {
            /*       destination       source                               20   */
            memcpy((void*)pRiga6, (void*)strErrors[(uint8_t)sckState], (size_t)20);
          }

          memset ((void*)pRigaSwV, ' ', 8);
          /*       destination       source           8 = len("Vx.y.zza HW AB")   */
          memcpy((void*)pRigaSwV, (void*)getFwVer(), (size_t)7);

          memcpy((void*)&pRigaSwV[12], (void*)getScuHWverFromEeprom(), (size_t)2);

          /*       destination       source           5 = len("Vw.ya" or "Vw.y\0")   */
          memcpy((void*)pRigaBtV, (void*)BOOT_ADDR_VER, (size_t)BOOT_VER_SIZE);
          for (i = 0; i < BOOT_VER_SIZE; i++)
          {
            if (pRigaBtV[i] == '\0')
            {
              pRigaBtV[i] = ' ';
            }
          }

          writePcb(hs, &heapFile, p, pcb);
          /*send the main page (which is the index page) */
          htmlpage = mainPage;
        }
        /* Check if request to get aesys logo */
        else if (strncmp(data, "GET /graphics/beLogoUnif2.jpg", 29) == 0)
        {
          strcat((char *)pass, "beLogoUnif2.jpg");
          /*         destination       source */
          fs_open(&heapFile, pass);
          writePcb(hs, &heapFile, p, pcb);
        }
        /* Check if request to get aesys logo */
        else if (strncmp(data, "GET /graphics/logoSCAME.png", 27) == 0)
        {
          strcat((char *)pass, "logoSCAME.png");
          /*         destination       source */
          fs_open(&heapFile, pass);
          writePcb(hs, &heapFile, p, pcb);
        }
        else if ((strncmp(data, "POST /initUpload.cgi", 20) == 0) && (htmlpage == mainPage))
        {
          htmlpage = FileUploadPage;
          DataFlag = 0;
          fs_open(&heapFile, "/upload.html");
          writePcb(hs, &heapFile, p, pcb);
          /* init pointer to ext flash */
          sFlashAddress = (uint32_t)EXT_SFLASH_NEW_FW_ADDR_START;          
        }
        else if (((strncmp(data, "POST /infoSN.cgi", 16) == 0) && (htmlpage == mainPage)) || (htmlpage == infoMacPOSTrx))
        {
          htmlpage = infoMacPOSTrx;

          /* TO DO: per ora ripropongo pagina principale */
          /*
            HTML Form URL Encoded: application/x-www-form-urlencoded
                Form item: "descrId" = "11:22:33"
                    Key: descrId
                    Value: 11:22:33

              0000   64 65 73 63 72 49 64 3d 31 41 25 33 41 32 42 25  descrId=1A%3A2B%
              0010   33 41 33 43                                      3A3C        */
          /* parse packet for the octet-stream field */
          for (i = 0; i < len; i++)
          {
            if (strncmp ((char*)(data + i), "serialNum=", 10) == 0)
            {
              for (normLen = i; normLen < len; normLen++)
              {
                if (isdigit((int)data[normLen]) || (char)data[normLen] == (char)'F')
                {
                  break;
                }
              }
              if (((data[normLen + 0]) == 'F') && ((data[normLen + 1]) == 'F') && 
                  ((data[normLen + 2]) == 'F') && ((data[normLen + 3]) == 'F') && 
                  ((data[normLen + 4]) == 'F') && ((data[normLen + 5]) == 'F') && 
                  ((data[normLen + 6]) == 'F') && ((data[normLen + 7]) == 'F'))
              {
                /* reset SN at default value */
                SerNum[0] = SerNum[1] = SerNum[2] = SerNum[3] = (uint8_t)0xFF;     
                eeprom_param_set(SERNUM_BYTE0_EADD, SerNum, 4);
                setScuSerialNumberEeprom((char*)SerNum, (char*)&data[normLen]);
              }
              else
              {
                if ((isdigit (data[normLen + 0])) && (isdigit (data[normLen + 1])) && 
                    (isdigit (data[normLen + 2])) && (isdigit (data[normLen + 3])) &&
                    (isdigit (data[normLen + 4])) && (isdigit (data[normLen + 5])) &&
                    (isdigit (data[normLen + 6])) && (isdigit (data[normLen + 7])))
                {
                  SerNum[3] = 0; SerNum[3] = (data[normLen + 7] - '0'); SerNum[3] |= ((data[normLen + 6] - '0') << 4);  /* BCD, first two LSB digit  */
                  SerNum[2] = 0; SerNum[2] = (data[normLen + 5] - '0'); SerNum[2] |= ((data[normLen + 4] - '0') << 4);  
                  SerNum[1] = 0; SerNum[1] = (data[normLen + 3] - '0'); SerNum[1] |= ((data[normLen + 2] - '0') << 4);  
                  SerNum[0] = 0; SerNum[0] = (data[normLen + 1] - '0'); SerNum[0] |= ((data[normLen + 0] - '0') << 4);  /* BCD, first two MSB digit  */
                  eeprom_param_set(SERNUM_BYTE0_EADD, SerNum, 4);
                  /* save SCU SN also in reserved area in EEPROM */
                  if (setScuSerialNumberEeprom((char*)SerNum, (char*)&data[normLen]) != 0)
                  {
                    tPrintf( "Error writing SCU SN\r\n");
                  }
                }
              }
              /*                                                   789111111111122222222223333333   */
              /*                                                   789012345678901234567890123456   */
              /* the serial string is like that:  serialNum=00013540 <prdNum>100305019 <hwVer>8-D @<prdCode>....$<fakeCode>...#   */
              /*                                                   789111111111122222222223333333333   */
              /*                                                   789012345678901234567890123456789   */
              /* the serial string is like that:  serialNum=00013540 <prdNum>50004486/001 <hwVer>8-D @<prdCode>....$<fakeCode>...#   */
              cnt = 8;
              while ((data[normLen + cnt] != '>') && (cnt <= 16))
              {
                cnt++;
              }
              if (data[normLen + cnt] == '>')
              {
                cnt++;
                dotFake = 0;
                while (data[normLen + cnt + dotFake] != ' ')
                {
                  dotFake++;
                }
                if (setProductSerialNumberEeprom((char*)&data[normLen + cnt], (uint8_t)dotFake, (uint8_t)TRUE) != 0)
                {
                  tPrintf( "Error writing Product SN\r\n");
                }
              }

              while (data[normLen + cnt] != '>')
              {
                cnt++;
              }
              if (data[normLen + cnt] == '>')
              {
                cnt++;
                pass[0] = data[normLen + cnt];
                pass[1] = data[normLen + cnt + 2];
                if (setScuHardwareVersionEeprom((char*)pass) != 0)
                {
                  tPrintf( "Error writing HwVer!\r\n");
                }
              }

              /* the serial string is like that:  serialNum=00013540 <prdNum>100305019 <hwVer>8-D @<prdCode>....$<fakeCode>...#   */
              /*                                                   789111111111122222222223333333333   */
              /*                                                   789012345678901234567890123456789   */
              /* the serial string is like that:  serialNum=00013540 <prdNum>50004486/001 <hwVer>8-D @<prdCode>....$<fakeCode>...#   */
              /* get the product code and save it */
              while ((data[normLen + cnt] != '>') && (data[normLen + cnt] != '$'))
              {
                cnt++;
              }
              if (data[normLen + cnt] == '>')
              {
                size = 0;
                cnt++;
                while (data[normLen + cnt] != '$')
                {
                  pass[size] = data[normLen + cnt];
                  cnt++; size++;
                }
                if (size != 0) 
                {
                  pass[size] = '\0';;
                  size++;
                  setStationProductCode(pass, (uint8_t)size);
                }
              }
              /* get the fake code and save it */
              while ((data[normLen + cnt] != '>') && (data[normLen + cnt] != '#'))
              {
                cnt++;
              }
              if (data[normLen + cnt] == '>')
              {
                size = 0; dotFake = 0;
                cnt++;
                while (data[normLen + cnt] != '#')
                {
                  pass[size] = data[normLen + cnt];
                  if (data[normLen + cnt] == '.')
                  {
                    dotFake = (char)size;
                  }
                  cnt++; size++;
                }
                if (size != 0) 
                {
                  pass[size] = '\0';
                  dotFake++;
                  cnt = (char)size - (char)dotFake;
                  size++; 
                  setStationFakeProductCode(pass, (uint8_t)size);
                  if ((cnt == 0) || (cnt > 4))
                  {
                    cnt = 1;
                  }
                  (void)setNumSocketFromFakeCode((uint8_t)cnt);
                }

              }
              if ((pass[3] == '*') && (pass[5] == '*')) pass[30] = TRUE; else pass[30] = FALSE;

              /* the data and time is at the end string:   .. @<prdCode>....$<fakeCode>...#<parDeT><Data&Time>   */
              pDeT = (char*)strstr ((char*)data, "parDeT>");    
              if (pDeT != NULL)                                                  
              {             
                /*** now find set date and time Flag  */
                while (*pDeT != '<')
                {
                  pDeT++;
                }
                pDeT++; // point to numeric chrge in time string start
                cnt = 0;
                while (pDeT[cnt] != '>')
                {
                  cnt++;
                }
                pDeT[cnt] = '\0'; // put end string 
                /* set unix time value     */
                i = (int32_t)atoi (pDeT);
                if (i != 0)
                {
                  /* Save Date and Time informations received, into BKP SRAM region */
                  /* Save also the checksum */
                  BKP_SRAM_UnixTimestamp_Save(i);    /* Ticket SCU-100 */

                  setDateTimeFromUnixT(i);
                  // Aggiorno data e ora nella struttura globale 
                  UpdateGlobalDT();
                  cnt = (char)1;
                  // xx eeprom_array_set(RTC_VALID_EADD, (uint8_t*)&cnt, 1);
                  SCU_InfoStation_Set ((uint8_t *)&infoStation.rtcValid, (uint8_t*)&cnt, 1);          /* ex RTC_VALID_EADD */
                }
              }

              /* check if there is data for RED:   .. @<prdCode>....$<fakeCode>...#<parDeT><Data&Time><apName><ssidName....>   */
              pWDef = (char*)strstr ((char*)data, "apName>");    
              if (pWDef != NULL)                                                  
              {             
                /*** now find set SSID name starting  */
                while (*pWDef != '<')
                {
                  pWDef++;
                }
                pWDef++; 
                cnt = 0;
                pWifiWebInfo = (wifiWebInfo*)&httpBuffer[0];
                /* initialize the structure */
                memset ((void*)pWifiWebInfo, 0x00, sizeof(wifiWebInfo));
                /*       destination                   source          len   */
                memcpy((void*)pWifiWebInfo->ssid, (void*)&pWDef[0],  (size_t)10);
                /*       destination                   source          len   */
                memcpy((void*)pWifiWebInfo->pass, (void*)&pWDef[10], (size_t)10);
                /*       destination                   source          len   */
                memcpy((void*)pWifiWebInfo->key,  (void*)&pWDef[20], (size_t)5);
                if (strstr ((char*)pWDef, "Scame01>") != NULL) cnt = 7; else cnt = 10;
                /*       destination                   source          len   */
                memcpy((void*)pWifiWebInfo->scuPass,  (void*)&pWDef[25], (size_t)cnt);
                /* seave data in secure area */
                (void)Configurator_saveWifiWebInfo(pWifiWebInfo);
              }

              /* new: set here the configuration starting from product code 204.... or 205... */
              getIdModelFromStringAndParConfig((char *)getStationProductCodeString());

              /* we receive 205*W*17*S as fake code when we create a jolly SEM SCU board to use in replacement */
              if (pass[30] == TRUE)
              {
                /* remove MIRROR error */
                eeprom_param_get(CONTROL_BYTE0_EADD, (uint8_t *)pass, 1);
                pass[0] &= (~MIRROR_CRL0);
                /* now save in EEPROM check flag and actuator presence */
                // xx eeprom_array_set(CONTROL_BYTE0_EADD, (uint8_t *)pass, 1);
                SCU_InfoStation_Set ((uint8_t *)&infoStation.controlByte.Byte.Byte0, (uint8_t *)pass, 1);   /* ex CONTROL_BYTE0_EADD */
                /* the jolly address must be set */
                pass[5] = SCU_S_REPL_ADDR - 1;
                // xx eeprom_array_set(RS485_ADD_EADD, (uint8_t*)&pass[5], 1);
                SCU_InfoStation_Set ((uint8_t *)&infoStation.rs485Address, (uint8_t*)&pass[5], 1);        /* ex RS485_ADD_EADD */
                /* set operative mode SEM Slave and fixed addresss */
                pass[5] = SCU_SEM_S;
                // xx eeprom_array_set(OPERATIVE_MODE_EADD, (uint8_t*)&pass[5], 1);               
                SCU_InfoStation_Set ((uint8_t *)&infoStation.Operative_mode, (uint8_t*)&pass[5], 1);        /* ex OPERATIVE_MODE_EADD */
                setAddressType((uint8_t)SCU_FIXED_ADDR, TRUE);
              }

              writePcb(hs, &heapFile, p, pcb);
              /** restart the system by NVIC reset */
              activeImmediateReset();

              break;
            }
          }

          if (htmlpage == infoMacPOSTrx)
          {
            freeAllpbuf(p);
            return ERR_OK;
          }
        }
        else if ((strncmp(data, "POST /changePsw.html", 20) == 0) && (htmlpage == mainPage))
        {
          htmlpage = changePassword;
          fs_open(&heapFile, "/changePass.html");

          writePcb(hs, &heapFile, p, pcb);
        }
        else if ((strncmp(data, "POST /specFunc.html", 19) == 0) && (htmlpage == mainPage))
        {
          htmlpage = infoSpecFunc;
          DataFlag = 0;
          fs_open(&heapFile, "/specFunc.html");

          /* modify the tag inside html page */
          if (pLang[0] == NULL)
          {
            if ((pCbox1[0] != NULL) && (pHwConfData  != NULL))
            {
              if (pHwConfData != NULL) free(pHwConfData);
              pHwConfData = NULL;
              pCbox1[0] = NULL;
            }
            if ((pSiBox[0] != NULL) && (pSiConfData  != NULL))
            {
              if (pSiConfData != NULL) free(pSiConfData);
              pSiConfData = NULL;
              pSiBox[0] = NULL;
            }
            
            pSpecFuncData = malloc(heapFile.len);

            if (pSpecFuncData != NULL)
            {
              /*       destination       source     len   */
              memcpy(pSpecFuncData, heapFile.data, heapFile.len);

              pLang[0]  = strstr(pSpecFuncData, (char*)"check50");      // EN  in according to LcdLanguage_en
              pLang[1]  = strstr(pSpecFuncData, (char*)"check51");      // IT
              pLang[2]  = strstr(pSpecFuncData, (char*)"check52");      // FR
              pLang[3]  = strstr(pSpecFuncData, (char*)"check53");      // D
              pLang[4]  = strstr(pSpecFuncData, (char*)"check54");      // ES
              pLang[5]  = strstr(pSpecFuncData, (char*)"check55");      // P
              pLang[6]  = strstr(pSpecFuncData, (char*)"check56");      // RO
              pLang[7]  = strstr(pSpecFuncData, (char*)"check57");      // PL
              pLang[8]  = strstr(pSpecFuncData, (char*)"check58");      // SE
              
              pCboxGl[0]  = strstr(pSpecFuncData, (char*)"check60");      // EN
              pCboxGl[1]  = strstr(pSpecFuncData, (char*)"check61");      // IT
              pCboxGl[2]  = strstr(pSpecFuncData, (char*)"check62");      // FR
              pCboxGl[3]  = strstr(pSpecFuncData, (char*)"check63");      // D
              pCboxGl[4]  = strstr(pSpecFuncData, (char*)"check64");      // ES
              pCboxGl[5]  = strstr(pSpecFuncData, (char*)"check65");      // P
              pCboxGl[6]  = strstr(pSpecFuncData, (char*)"check66");      // RO
              pCboxGl[7]  = strstr(pSpecFuncData, (char*)"check67");      // PL
              pCboxGl[8]  = strstr(pSpecFuncData, (char*)"check68");      // SE

              pPresence[0] = strstr(pSpecFuncData, (char*)"chk30");      // none / block 
              pPresence[1] = strstr(pSpecFuncData, (char*)"disabl01");    // presenza socket 1 
              pPresence[2] = strstr(pSpecFuncData, (char*)"disabl02");    // presenza socket 2 
              pPresence[3] = strstr(pSpecFuncData, (char*)"disabl03");    // presenza socket 3 
              pPresence[4] = strstr(pSpecFuncData, (char*)"disabl04");    // presenza socket 4 
              pPresence[5] = strstr(pSpecFuncData, (char*)"disabl05");    // presenza socket 5 
              pPresence[6] = strstr(pSpecFuncData, (char*)"disabl06");    // presenza socket 6 
              pPresence[7] = strstr(pSpecFuncData, (char*)"disabl07");    // presenza socket 7 
              pPresence[8] = strstr(pSpecFuncData, (char*)"disabl08");    // presenza socket 8 
              pPresence[9] = strstr(pSpecFuncData, (char*)"disabl09");    // presenza socket 9 
              pPresence[10] = strstr(pSpecFuncData, (char*)"disabl10");   // presenza socket 10 
              pPresence[11] = strstr(pSpecFuncData, (char*)"disabl11");   // presenza socket 11 
              pPresence[12] = strstr(pSpecFuncData, (char*)"disabl12");   // presenza socket 12 
              pPresence[13] = strstr(pSpecFuncData, (char*)"disabl13");   // presenza socket 13 
              pPresence[14] = strstr(pSpecFuncData, (char*)"disabl14");   // presenza socket 14 
              pPresence[15] = strstr(pSpecFuncData, (char*)"disabl15");   // presenza socket 15
              pPresence[16] = strstr(pSpecFuncData, (char*)"disabl16");   // presenza socket 16

              pAbPm[0]  = strstr(pSpecFuncData, (char*)"check70");        // abilitazione Power Management 
              pAbPm[1]  = strstr(pSpecFuncData, (char*)"check71");        // abilitazione unbalance
              pAbPm[2]  = strstr(pSpecFuncData, (char*)"check72");        // abilitazione ricarica a fasce
              pAbPm[4]  = strstr(pSpecFuncData, (char*)"check73");        // abilitazione EMEX
              pAbPm[5]  = strstr(pSpecFuncData, (char*)"check75");        // abilitazione SINAPSI
              pAbPm[6]  = strstr(pSpecFuncData, (char*)"@ee");            // errori SINAPSI 
              pAbPm[7]  = strstr(pSpecFuncData, (char*)"check91");        // visible / not visible parameter 
              pAbPm[8]  = strstr(pSpecFuncData, (char*)"disab#ed");       // pm abil. ckeck box enab / disab.  
              

              pAbPm[3]  = strstr(pSpecFuncData, (char*)"check80");        // abilitazione Charge In Time
              pAbPm[9]  = strstr(pSpecFuncData, (char*)"check92");        // visibility Charge In Time
              pAbPm[10] = strstr(pSpecFuncData, (char*)"disa#led");       // charge time ckeck box enab / disab.  
              pAbPm[11] = strstr(pSpecFuncData, (char*)"check93");        // power management mode Full
              pAbPm[12] = strstr(pSpecFuncData, (char*)"check94");        // power management mode Eco
              pAbPm[13] = strstr(pSpecFuncData, (char*)"check95");        // power management mode Plus
              pAbPm[14] = strstr(pSpecFuncData, (char*)"check40");        // power management remote (SEM)

              parPM[0]  = strstr(pSpecFuncData, (char*)"@x");             // valore PMAX es:    21,5 [KW]   3..22   [KW]
              parPM[1]  = strstr(pSpecFuncData, (char*)"@y");             // valore Imin es:    6 [A]       6..63   [A]
              parPM[2]  = strstr(pSpecFuncData, (char*)"@z");             // valore Hpower es:  7 [%]       0..10   [%]
              parPM[3]  = strstr(pSpecFuncData, (char*)"@r");             // valore DSET es:    100         0..2000 [W]
              parPM[4]  = strstr(pSpecFuncData, (char*)"@w");             // valore DMAX es:    100[%]      0..100  [%]
              parPM[5]  = strstr(pSpecFuncData, (char*)"@t");             // valore T_RIC es:   60 [min]    0..360  [min]
              parPM[6]  = strstr(pSpecFuncData, (char*)"@v");             // valore ENRG_RIC es:10 [KWh]    0..100  [KWh]

              pTime[0] =  strstr(pSpecFuncData, (char*)"@g");             // time zone
              pTime[1] =  strstr(pSpecFuncData, (char*)"check81");        // DST (abilitazione ora legale
              pTime[2] =  strstr(pSpecFuncData, (char*)"check82");        // Time and date abilitazione
              pTime[3] =  strstr(pSpecFuncData, (char*)"Wed Feb 13 15:46:11 2013");     // current data and time field 

              pMode[0] =  strstr(pSpecFuncData, (char*)"check83");        // mode free
              pMode[1] =  strstr(pSpecFuncData, (char*)"check84");        // mode personal
              pMode[2] =  strstr(pSpecFuncData, (char*)"check85");        // mode Net
              pMode[3] =  strstr(pSpecFuncData, (char*)"check86");        // mode OCCP

              pLcd[0]  = strstr(pSpecFuncData, (char*)"check87");          // no LCD 
              pLcd[1]  = strstr(pSpecFuncData, (char*)"check88");          // 2x20 alpha LCD

              pWiFi[0]  = strstr(pSpecFuncData, (char*)"check89");          // no/yes WiFi 
              pWiFi[1]  = strstr(pSpecFuncData, (char*)"check90");          // no/yes Diff. Riarm 
              pWiFi[2]  = strstr(pSpecFuncData, (char*)"check96");          // no/yes wifi con SBC 
              pWiFi[3]  = strstr(pSpecFuncData, (char*)"@c");               // channel WiFi number 
              pWiFi[4]  = strstr(pSpecFuncData, (char*)"@WiFiVe");          // wiFi Expressif FW version 
              pWiFi[5]  = strstr(pSpecFuncData, (char*)"@tru");             // button FW upgrade presence 
              pWiFi[6]  = strstr(pSpecFuncData, (char*)"@db");              // wiFi Expressif signal level  
            }
          }

          if (pLang[0] == NULL)
          {
            if (pSpecFuncData != NULL) free(pSpecFuncData);
            pSpecFuncData = NULL;
          }
          else
          {
            heapFile.data = (const char*)pSpecFuncData;

            eeprom_param_get(LANG_DEFAULT_EADD, (uint8_t *)pass, 1);
            if (pass[0] < NUM_LANGUAGE)
            {
              /* set default language                                 */
              /*       destination                source        len   */
              memcpy((void*)pLang[pass[0]], (void*)"checked", (size_t)7);

              /* set power manager enabling                           */
              eeprom_param_get(HIDDEN_MENU_ENB_EADD, (uint8_t*)pass, 1);
              pass[0] &= HIDDEN_MENU_PMNG_ENB;

              if (pass[0] != (char)0)
              {
                memcpy((void*)pAbPm[0], (void*)"checked", (size_t)7);
              }
              else
              {
                memcpy((void*)pAbPm[0], (void*)"       ", (size_t)7);
              }
              /* get info for parameter visibulity                         */
              eeprom_param_get(HIDDEN_MENU_VIS_EADD, (uint8_t *)&pass[6], 1);
              if ((pass[6] & HIDDEN_MENU_PMNG_VIS) == (char)0)
              {
                memcpy((void*)pAbPm[8], (void*)"disabled", (size_t)8);  /* flag visibile non attiva --> disabilito check box abilitazione PM */
                memcpy((void*)pAbPm[7], (void*)"       ", (size_t)7);
              }
              else
              {
                memcpy((void*)pAbPm[7], (void*)"checked", (size_t)7);  /* flag "PM visibile" attiva; flag sul check box  */
              }

              /* set group language                                 */
              eeprom_param_get(LANG_CONFIG0_EADD, (uint8_t *)pass, 4);
              for (i = 0, len = 1, normLen = 0; i < (int32_t)NUM_LANGUAGE; i++, len = len << 1)
              {
                if (len >= (int32_t)0x100) 
                {
                  /* next language group byte */
                  len = 1;
                  normLen = i / (int32_t)8;
                }
                if ((pass[normLen] & (char)len) != 0)
                {
                  memcpy((void*)pCboxGl[i], (void*)"checked", (size_t)7);
                }
                else
                {
                  memcpy((void*)pCboxGl[i], (void*)"       ", (size_t)7);
                }
              }

              /* set presenza sockets                                 */
              if (isSemMasterFz() == TRUE)
              {
                /* è la master in SEM: allora questo pannello può essere visualizzato */
                memcpy((void*)pPresence[0], (void*)"block", (size_t)5);
                len = (int32_t)getSocketDiscovered();
                for (i = 1, normLen = 1, userFound = 0; i <= (int32_t)SCU_NUM; i++, normLen = normLen << 1, userFound++)
                {
                  if ((len & normLen) == 0)
                  {
                    memcpy((void*)pPresence[i], (void*)"disabled", (size_t)8);
                    memcpy((void*)((uint32_t)pPresence[i] +(uint32_t)16) , (void*)" \"", (size_t)2);
                  }
                  else
                  {
                    memcpy((void*)pPresence[i], (void*)"disabled", (size_t)8);
                    pass[8] = (char) getModbusAddrFromDevId(userFound);                   // recovery modbus address from device Id
                    pass[9] = (char)sprintf((char*)&pass[10], "%d\"", pass[8]); 
                    memcpy((void*)((uint32_t)pPresence[i] +(uint32_t)16), (void*)&pass[10], (size_t)pass[9]);    // the value is 16 position after "disabled"
                  }
                }
              }
              else
              {
                /* non è la master: allora questo pannello non deve essere visualizzato */
                memcpy((void*)pPresence[0], (void*)"none ", (size_t)5);
              }

              /* set abilitazione ricarica a fasce orarie             */
             eeprom_param_get(PMNG_TRANGE_EADD, (uint8_t *)pass, 1);
             if (pass[0] != (char)0)
             {
               memcpy((void*)pAbPm[2], (void*)"checked", (size_t)7);
             }
             else
             {
               memcpy((void*)pAbPm[2], (void*)"       ", (size_t)7);
             }
              /* get abilitazione EMEX energy meter esterno e SINAPSI activation flag    */
             eeprom_param_get(CONTROL_BYTE2_EADD, (uint8_t *)pass, 1);
             
             eeprom_param_get(HIDDEN_MENU_ENB_EADD, (uint8_t*)&pass[1], 1);
             
             if ((pass[0] & (char)EMETER_EXT_CRL2) != (char)0)
             {
               memcpy((void*)pAbPm[4], (void*)"checked", (size_t)7); 
             }
             else
             {
               memcpy((void*)pAbPm[4], (void*)"       ", (size_t)7); 
             }
             if ((pass[1] & (char)HIDDEN_MENU_SINAPSI) != (char)0)
             {
               memcpy((void*)pAbPm[5], (void*)"checked", (size_t)7);
             }
             else
             {
               memcpy((void*)pAbPm[5], (void*)"       ", (size_t)7);
             }
             if ((pass[1] & (char)HIDDEN_MENU_SEM_ENB) != (char)0)
             {
               memcpy((void*)pAbPm[14], (void*)"checked", (size_t)7);
             }
             else
             {
               memcpy((void*)pAbPm[14], (void*)"       ", (size_t)7);
             }

             len = sprintf((char*)&pass[10], "%3d", getStationSinapsiRS485Error());
             /*       destination      source            len   */
             memcpy ((void*)pAbPm[6], (void*)&pass[10],  len );


              /* set abilitazione unbalance per carichi trifase      */
             eeprom_param_get(PMNG_UNBAL_EADD, (uint8_t *)pass, 1);
             if (pass[0] != (char)0)
             {
               memcpy((void*)pAbPm[1], (void*)"checked", (size_t)7);
             }
             else
             {
               memcpy((void*)pAbPm[1], (void*)"       ", (size_t)7);
             }
              /* set abilitazione ricarica a tempo      */
             eeprom_param_get(TCHARGE_MODE_EADD, (uint8_t *)pass, 1); 
             if (pass[0] != (char)0)
             {
               memcpy((void*)pAbPm[3], (void*)"checked", (size_t)7);
             }
             else
             {
               memcpy((void*)pAbPm[3], (void*)"       ", (size_t)7);
             }
             /* get info for charge Time / energy parameter visibility                         */
             eeprom_param_get(HIDDEN_MENU_VIS_EADD, (uint8_t *)&pass[6], 1);
             if ((pass[6] & HIDDEN_MENU_TMEG_VIS) == (char)0)
             {
               memcpy((void*)pAbPm[10], (void*)"disabled", (size_t)8);  /* flag visibile non attiva --> disabilito check box abilitazione charge time */
               memcpy((void*)pAbPm[9],  (void*)"       ", (size_t)7);
             }
             else
             {
               memcpy((void*)pAbPm[9], (void*)"checked", (size_t)7);  /* flag "charge time visible" attiva; flag sul check box  */
             }

             /* get current power management mode */
             eeprom_param_get(PMNG_MODE_EADD, (uint8_t *)&pass[0], 1);
             pass[0] &=((char)PMNG_MODE_MASK);
             memcpy((void*)pAbPm[11],  (void*)"       ", (size_t)7);
             memcpy((void*)pAbPm[12],  (void*)"       ", (size_t)7);
             memcpy((void*)pAbPm[13],  (void*)"       ", (size_t)7);
             
             switch (pass[0])
             {
               case PMNG_ECO_PLUS:
                 memcpy((void*)pAbPm[12], (void*)"checked", (size_t)7);  /* power management mode Eco attiva; flag sul check box  */
                 break;
               case PMNG_ECO_SMART:
                 memcpy((void*)pAbPm[13], (void*)"checked", (size_t)7);  /* power management mode Plus attiva; flag sul check box  */
                 break;
               case PMNG_FULL:
               default:
                 memcpy((void*)pAbPm[11], (void*)"checked", (size_t)7);  /* power management mode Full attiva; flag sul check box  */
                 break;
             }

             /* get Potenza massima       */
             eeprom_param_get(PMNG_PWRLSB_EADD, (uint8_t *)pass, 2);
             i = ((int32_t)(pass[1]) << 8) + pass[0];
             i *= 100;
             if ((i < (int32_t)3000) || (i > (int32_t)99999))
             {
               i =(int32_t)3000;
             }
             memset ((void*)&pass[10], ' ', 6);
             /*       destination      source            len   */
             memcpy ((void*)parPM[0], (void*)&pass[10],  6 );
             if ((i % 1000) == 0)  // on web the power is in KW
             {
               /* no decimal */
               len = sprintf((char*)&pass[10], "%d\"", i / 1000);
             }
             else
             {
               /* decimal */
               len = sprintf((char*)&pass[10], "%d.%d\"", i / 1000, i % 1000);
             }
             /*       destination      source            len   */
             memcpy ((void*)parPM[0], (void*)&pass[10],  len );

             /* get corrente minima A * 10      */
             eeprom_param_get(PMNG_CURRENT_EADD, (uint8_t *)pass, 1);
             memset ((void*)&pass[10], ' ',6);
             i = (int32_t)pass[0];
             if ((i % 10) == 0)
             {
               /* no decimal */
               len = sprintf((char*)&pass[10], "%d\"", i / 10);
             }
             else
             {
               /* decimal */
               len = sprintf((char*)&pass[10], "%d.%d\"", i / 10, i % 10);
             }
             /*       destination      source            len   */
             memcpy ((void*)parPM[1], (void*)&pass[10], len );

             /* get hpower        */
             eeprom_param_get(PMNG_MULTIP_EADD, (uint8_t *)pass, 1);
             memset ((void*)&pass[10], ' ',6);
             pass[0]++;
             if ((pass[0] < (char)1) || (pass[0] > (char)10))
             {
               pass[0] = (char)2;
             }
             len = sprintf((char*)&pass[10], "%d\"", pass[0]);
             /*       destination      source            len   */
             memcpy ((void*)parPM[2], (void*)&pass[10], len );

             /* get DSET        */
             eeprom_param_get(PMNG_ERROR_EADD, (uint8_t *)pass, 1);
             memset ((void*)&pass[10], ' ',6);
             if (pass[0] > (char)20)
             {
               pass[0] = (char)0;
             }
             else
             {
               i = (int32_t)pass[0] * (int32_t)100;
             }
             len = sprintf((char*)&pass[10], "%d\"", i);
             /*       destination      source            len   */
             memcpy ((void*)parPM[3], (void*)&pass[10], len );

             /* get DMAX        */
             eeprom_param_get(PMNG_DMAX_EADD, (uint8_t *)pass, 1);
             memset ((void*)&pass[10], ' ',6);
             if (pass[0] > (char)100)
             {
               pass[0] = (char)10;
             }
             len = sprintf((char*)&pass[10], "%d\"", pass[0]);
             /*       destination      source            len   */
             memcpy ((void*)parPM[4], (void*)&pass[10], len );

             /* get CHARGE IN TIME         */
             eeprom_param_get(TCHARGE_TIME_EADD, (uint8_t *)pass, 1);
             memset ((void*)&pass[10], ' ',6);
             if (pass[0] > (char)12)
             {
               pass[0] = (char)0;
             }
             i = ((int32_t)(pass[0] * 30));
             len = sprintf((char*)&pass[10], "%d\" ", i);
             /*       destination      source            len   */
             memcpy ((void*)parPM[5], (void*)&pass[10], len );

             /* get ENERGY CHARGING         */
             eeprom_param_get(ENRG_LIMIT_EADD, (uint8_t *)pass, 1);
             memset ((void*)&pass[10], ' ',6);
             if (pass[0] > (char)100)
             {
               pass[0] = (char)0;
             }
             len = sprintf((char*)&pass[10], "%d\" ", pass[0]);
             /*       destination      source            len   */
             memcpy ((void*)parPM[6], (void*)&pass[10], len );

             /* get time zone         */
             eeprom_param_get(TIME_ZONE_EADD, (uint8_t *)pass, 1);
             memset ((void*)&pass[10], ' ',6);
             i = ((int32_t)(pass[0]));
             if ((i > (int32_t)12) || (i < (int32_t)-12))
             {
               pass[0] = (char)0;
             }
             i = ((int32_t)(pass[0]));
             len = sprintf((char*)&pass[10], "%d\"", i);
             /*       destination      source            len   */
             memcpy ((void*)pTime[0], (void*)&pass[10], len );

              /* get abilitazione ora legale      */
             eeprom_param_get(DST_EADD, (uint8_t *)pass, 1);
             if (pass[0] != (char)0)
             {
               memcpy((void*)pTime[1], (void*)"checked", (size_t)7);
             }
             else
             {
               memcpy((void*)pTime[1], (void*)"       ", (size_t)7);
             }
              /* set date and time       */
             eeprom_param_get(RTC_VALID_EADD, (uint8_t *)pass, 1);
             if (pass[0] != (char)1)
             {
               memcpy((void*)pTime[2], (void*)"checked", (size_t)7);
             }
             else
             {
               memcpy((void*)pTime[2], (void*)"       ", (size_t)7);
             }

             /* report current SCU data and time */
             (void)getCurrentLocalTime((uint32_t*)&packetOverhead);
             /*         destination       source                len */
             strncpy((char *)pTime[3], (char *)packetOverhead, (size_t)24);

             /* get operative mode       */
             eeprom_param_get(EVS_MODE_EADD, (uint8_t *)pass, 1);
             if (pass[0] <= EVS_OCPP_MODE)
             {
               /* reset radio button flag */
               for (i = 0; i < EVS_MAX_MODE; i++)
               {
                 memcpy((void*)pMode[i], (void*)"       ", (size_t)7);
               }
               /* set the current operative mode                       */
               /*       destination                source        len   */
               memcpy((void*)pMode[pass[0]], (void*)"checked", (size_t)7);
             }

             /* get lcd type  from LCD_TYPE_EADD   */
             eeprom_param_get(LCD_TYPE_EADD, (uint8_t *)pass, 1);
             pass[1] =  pass[0]; 
             pass[2] =  pass[0]; 
             pass[3] =  pass[0]; 
             pass[0] &= (uint8_t)LCD_TYPE_MASK;
             switch (pass[0])
             {
               case LCD_2X20:   
                 cnt = 1;  
                 break;

               default: 
               case LCD_TYPE_NULL:
                 cnt = 0;  
                 break;
             }
             for (i = 0; i < (uint8_t)LCD_TYPE_NUM; i++)
             {
               if ((char)i == cnt)
               {
                 /*       destination       source            len    */
                 memcpy((void*)pLcd[i], (void*)"checked", (size_t)7);
               }
               else
               {
                 /*       destination       source             len   */
                 memcpy((void*)pLcd[i], (void*)"       ", (size_t)7);
               }
             }

             /* get channel wifi number         */
             memset ((void*)pWiFi[3], ' ',4);
             i = (int32_t)getWifiAccessPointChannelId();
             len = sprintf((char*)&pass[10], "%d\"", i);
             /*       destination      source            len   */
             memcpy ((void*)pWiFi[3], (void*)&pass[10], len );

             /* get WiFi status   */
             pass[1] &= (uint8_t)WIFI_MASK;
             if (pass[1] == (uint8_t)0)
             {
               /* WiFi OFF and button upgrade not present and no version displayed*/
               /*       destination       source            len    */
               memcpy((void*)pWiFi[0], (void*)"       ", (size_t)7);
               /*       destination       source            len    */
               memcpy((void*)pWiFi[5], (void*)"none", (size_t)4);
               /*       destination       source            len    */
               memcpy((void*)pWiFi[4], (void*)"       ", (size_t)7);
             }
             else
             {
               /* WiFi ON and button upgrade present and version displayed */
               /*       destination       source            len    */
               memcpy((void*)pWiFi[0], (void*)"checked", (size_t)7);
               /*       destination       source            len    */
               memcpy((void*)pWiFi[5], (void*)"true", (size_t)4);
               
               /* get Expressif FW versione */
               AppEmobTask_getModuleVersion((Version*)&pass[20]);

               freeUse = pWiFi[4]; freeUse[1] = freeUse[3] = freeUse[5] = '.'; // version in this style: 3.3.0.0
               freeUse[0] = '0'+pass[20]; freeUse[2] = '0'+pass[21]; freeUse[4] = '0'+pass[22]; freeUse[6] = '0'+pass[23];
             }
             /* get channel wifi number         */
             memset ((void*)pWiFi[6], ' ',4);

             i = (int32_t)AppEmobTask_getWifiRssi();

             len = sprintf((char*)&pass[20], "%d", i);
             /*       destination      source            len   */
             memcpy ((void*)pWiFi[6], (void*)&pass[20], len );

      
             /* get WiFi&SBC status   */
             pass[3] &= (uint8_t)SBC_WIFI_MASK;
             if (pass[3] == (uint8_t)0)
             {
               /* WiFi&SBC OFF */
               /*       destination       source            len    */
               memcpy((void*)pWiFi[2], (void*)"       ", (size_t)7);
             }
             else
             {
               /* WiFi&SBC ON */
               /*       destination       source            len    */
               memcpy((void*)pWiFi[2], (void*)"checked", (size_t)7);
             }
      
             /* get DiRi status   */
             pass[2] &= (uint8_t)DIRI_MASK;
             if (pass[2] == (uint8_t)0)
             {
               /* DiRi OFF */
               /*       destination       source            len    */
               memcpy((void*)pWiFi[1], (void*)"       ", (size_t)7);
             }
             else
             {
               /* DiRi ON */
               /*       destination       source            len    */
               memcpy((void*)pWiFi[1], (void*)"checked", (size_t)7);
             }
                    
             writePcb(hs, &heapFile, p, pcb);
            } // end if NUM_LANGUAGE
          }
        }
        else if ((strncmp(data, "POST /hwConf.html", 17) == 0) && (htmlpage == mainPage))
        {
          htmlpage = infoHwConfRx;
          DataFlag = 0;
          fs_open(&heapFile, "/hwConf.html");

          /* modify the tag inside html page */
          if (pCbox1[0] == NULL)
          {
            if ((pLang[0] != NULL) && (pSpecFuncData  != NULL))
            {
              if (pSpecFuncData != NULL) free(pSpecFuncData);
              pSpecFuncData = NULL;
              pLang[0] = NULL;
            }
            if ((pSiBox[0] != NULL) && (pSiConfData  != NULL))
            {
              if (pSiConfData != NULL) free(pSiConfData);
              pSiConfData = NULL;
              pSiBox[0] = NULL;
            }

            pHwConfData = malloc(heapFile.len); 

            if (pHwConfData != NULL)
            {
              /*       destination       source     len   */
              memcpy(pHwConfData, heapFile.data, heapFile.len);
              pCbox1[0] = strstr(pHwConfData, (char*)"check01");    // pCbRcdm
              pCbox1[1] = strstr(pHwConfData, (char*)"check02");    // pCbLidc = 
              pCbox1[2] = strstr(pHwConfData, (char*)"check03");    // pCbVent = 
              pCbox1[3] = strstr(pHwConfData, (char*)"check04");    // pCbBclk = 
              pCbox1A[3] = strstr(pHwConfData, (char*)"check05");   // pCbBclkAct
              pCbox1[4] = strstr(pHwConfData, (char*)"check06");    // pCbRmen = 
              pCbox1[5] = strstr(pHwConfData, (char*)"check07");    // pCbPuls = 
              pCbox1[6] = strstr(pHwConfData, (char*)"check08");    // pCbMirr = 
              pCbox1A[6] = strstr(pHwConfData, (char*)"check09");   // pCbMirrAct
              pCbox1[7] = strstr(pHwConfData, (char*)"check10");    // pCbRcbo = 
              pCbox1A[7] = strstr(pHwConfData, (char*)"check11");   // pCbRcboAct
              pCbox2[0] = strstr(pHwConfData, (char*)"check12");      // pCbCpse
              pCbox2[1] = strstr(pHwConfData, (char*)"check13");      // pCbPpse
              pCbox2[2] = strstr(pHwConfData, (char*)"check14");      // pCbCpls
              pCbox2[3] = strstr(pHwConfData, (char*)"check15");      // pCbPpls
              pCbox2[4] = strstr(pHwConfData, (char*)"check16");      // pCbVbus
              pCbox2[5] = strstr(pHwConfData, (char*)"check17");      // pCbMfre
              pCbox2[6] = strstr(pHwConfData, (char*)"check18");      // pCbEmtr
              pCbox2[7] = strstr(pHwConfData, (char*)"check19");      // pCbOvce
              pCbox3[0] = strstr(pHwConfData, (char*)"check20");      // pCbRcte
              pHgpt[0]  = strstr(pHwConfData, (char*)"check22");      // hgtp
              pCbox1A[4] = strstr(pHwConfData, (char*)"check21");   // pCbPAutAtt

              pSock[0]  = strstr(pHwConfData, (char*)"check40");      // n/a
              pSock[1]  = strstr(pHwConfData, (char*)"check41");      // 2b
              pSock[2]  = strstr(pHwConfData, (char*)"check42");      // 2b_LC
              pSock[3]  = strstr(pHwConfData, (char*)"check43");      // 2b_LO
              pSock[4]  = strstr(pHwConfData, (char*)"check44");      // 3Cb_LO
              pSock[5]  = strstr(pHwConfData, (char*)"check45");      // 3C
              pSock[6]  = strstr(pHwConfData, (char*)"check46");      // 3Ab_LO
              pSock[7]  = strstr(pHwConfData, (char*)"check47");      // 3A
              pSock[8]  = strstr(pHwConfData, (char*)"check48");      // Sb_LC
              pSock[9]  = strstr(pHwConfData, (char*)"check49");      // S
              pSock[10] = strstr(pHwConfData, (char*)"check50");      // 1_CBL Tethered Type 1
              pSock[11] = strstr(pHwConfData, (char*)"check51");      // 2_CBL Tethered Type 2
              
              pEmeter[0]  = strstr(pHwConfData, (char*)"check30");      // n/a
              pEmeter[1]  = strstr(pHwConfData, (char*)"check31");      // TA
              pEmeter[2]  = strstr(pHwConfData, (char*)"check32");      // Mono Gavazzi
              pEmeter[3]  = strstr(pHwConfData, (char*)"check33");      // Mono Algo 2
              pEmeter[4]  = strstr(pHwConfData, (char*)"check34");      // TRI Gavazzi
              pEmeter[5]  = strstr(pHwConfData, (char*)"check35");      // TRI Algo2
              pEmeter[6]  = strstr(pHwConfData, (char*)"check37");      // TA TRI 
              pEmeter[7]  = strstr(pHwConfData, (char*)"check56");      // LOVATO MONO 
              pEmeter[8]  = strstr(pHwConfData, (char*)"check57");      // LOVATO TRI 
              pEmeter[9]  = strstr(pHwConfData, (char*)"check58");      // SCAME MONO 
              pEmeter[10] = strstr(pHwConfData, (char*)"check59");      // SCAME TRI 
              pEmeter[11] = strstr(pHwConfData, (char*)"disab#ed");     // check box to clear energy for Scame EM 

              pLedS[0]  = strstr(pHwConfData, (char*)"check52");          // 6 led 
              pLedS[1]  = strstr(pHwConfData, (char*)"check53");          // 9 led
              pLedS[2]  = strstr(pHwConfData, (char*)"check54");          // 12 led
              pLedS[3]  = strstr(pHwConfData, (char*)"check55");          // 18 led

              pItypical   = strstr(pHwConfData, (char*)"@x");           // min current in typical mode
              pSempl      = strstr(pHwConfData, (char*)"@y");           // min current in semplified mode

              pBackup    =  strstr(pHwConfData, (char*)"check36");      // Battery Backup flag

              pBusAdd   =   strstr(pHwConfData, (char*)"@a");           // SCU address:       1..16

              pEmCode   =   strstr(pHwConfData, (char*)"@d");           // Energy meter code in according to EmeterType_en

              pOpMode[0]    =  strstr(pHwConfData, (char*)"check38");   // mode EMUMAX0
              pOpMode[1]    =  strstr(pHwConfData, (char*)"check39");   // mode SEM Master Principale
              pOpMode[2]    =  strstr(pHwConfData, (char*)"check60");   // mode SEM Slave
              pOpMode[3]    =  strstr(pHwConfData, (char*)"check61");   // mode SEM Master secondario
              pOpMode[4]    =  strstr(pHwConfData, (char*)"check67");   // mode SEM isolated


              pReadSp[0]    = strstr(pHwConfData, (char*)"check62");  // Socket position H_SX 
              pReadSp[1]    = strstr(pHwConfData, (char*)"check63");  // Socket position H_DX 
              pReadSp[2]    = strstr(pHwConfData, (char*)"check64");  // Socket position L_SX 
              pReadSp[3]    = strstr(pHwConfData, (char*)"check65");  // Socket position L_DX 

              pAddMode      = strstr(pHwConfData, (char*)"check66");  // Address SCU fixed / adjustable 
              pProdConfData = strstr(pHwConfData, (char*)"205@01");   // Address to set the current product
            }
          }

          if (pCbox1[0] == NULL)
          {
            if (pHwConfData != NULL) free(pHwConfData);
            pHwConfData = NULL;
          }
          else
          {
            heapFile.data = (const char*)pHwConfData;

            eeprom_param_get(CONTROL_BYTE0_EADD, (uint8_t *)pass, 5);

            /* set RCDM check box and all bit on CONTROL_BYTE0   */
            for (i = 0, cnt = 0x01; i < 8; i++, cnt = cnt << 1)
            {
              if ((pass[0] & cnt) != 0)
              {
                /*       destination       source     20   */
                memcpy((void*)pCbox1[i], (void*)"checked", (size_t)7);
              }
              else
              {
                /*       destination       source     20   */
                memcpy((void*)pCbox1[i], (void*)"       ", (size_t)7);
              }
            }
            /* set CPSHORT check box and all bit on CONTROL_BYTE1   */
            for (i = 0, cnt = 0x01; i < 8; i++, cnt = cnt << 1)
            {
              if ((pass[1] & cnt) != 0)
              {
                /*       destination       source     20   */
                memcpy((void*)pCbox2[i], (void*)"checked", (size_t)7);
              }
              else
              {
                /*       destination       source     20   */
                memcpy((void*)pCbox2[i], (void*)"       ", (size_t)7);
              }
            }
            /* set RECTIFIER check box and all bit on CONTROL_BYTE2   */
            for (i = 0, cnt = 0x01; i < 1; i++, cnt = cnt << 1)
            {
              if ((pass[2] & cnt) != 0)
              {
                /*       destination       source     20   */
                memcpy((void*)pCbox3[i], (void*)"checked", (size_t)7);
              }
              else
              {
                /*       destination       source     20   */
                memcpy((void*)pCbox3[i], (void*)"       ", (size_t)7);
              }
            }

            eeprom_param_get(TEMP_CTRL_ENB_EADD, (uint8_t *)pass, 1);

            /* set HGPT check box and future bits    */
            for (i = 0, cnt = 0x01; i < 8; i++, cnt = cnt << 1)
            {
              if ((i == 0) && (pHgpt[i] != NULL))
              {
                if ((pass[0] & cnt) != 0)
                {
                  /*       destination       source     20   */
                  memcpy((void*)pHgpt[i], (void*)"checked", (size_t)7);
                }
                else
                {
                  /*       destination       source     20   */
                  memcpy((void*)pHgpt[i], (void*)"       ", (size_t)7);
                }
              }
            }

            /* set Actuators  check box     */
            for (i = 0, cnt = 0x01; i < 8; i++, cnt = cnt << 1)
            {
              if (((i == 3) || (i == 6) || (i == 7) || (i == 4)) && (pCbox1A[i] != NULL))
              {
                if ((pass[4] & cnt) != 0)
                {
                  /*       destination       source     20   */
                  memcpy((void*)pCbox1A[i], (void*)"checked", (size_t)7);
                }
                else
                {
                  /*       destination       source     20   */
                  memcpy((void*)pCbox1A[i], (void*)"       ", (size_t)7);
                }
              }
            }

            /* set socket type  from SOCKET_TYPE_EADD   */
            eeprom_param_get(SOCKET_TYPE_EADD, (uint8_t *)pass, 1);
            cnt = 0;
            switch (pass[0])
            {
              case SOCKET_TYPE_NULL: 
                cnt = 0;  
                break;
              case SOCKET_T2_NO_LID:   
                cnt = 1;  
                break;
              case SOCKET_T2_CLOSE_LID:
                cnt = 2;  
                break;
              case SOCKET_T2_OPEN_LID:
                cnt = 3;  
                break;
              case SOCKET_3C_OPEN_LID:
                cnt = 4;  
                break;
              case SOCKET_3C_NO_LID:
                cnt = 5;  
                break;
              case SOCKET_3A_OPEN_LID: 
                cnt = 6;  
                break;
              case SOCKET_3A_NO_LID:   
                cnt = 7;  
                break;
              case SOCKET_SK_CLOSE_LID:
                cnt = 8;  
                break;
              case SOCKET_SK_NO_LID:
                cnt = 9;  
                break;
              case SOCKET_T1_TETHERED: 
                cnt = 10;  
                break;
              case SOCKET_T2_TETHERED:
                cnt = 11;  
                break;
            }
            for (i = 0; i < 12; i++)
            {
              if ((char)i == cnt)
              {
                /*       destination       source            len   EMETER_MONO_PH_GAVAZZI */
                memcpy((void*)pSock[i], (void*)"checked", (size_t)7);
              }
              else
              {
                /*       destination       source             len   */
                memcpy((void*)pSock[i], (void*)"       ", (size_t)7);
              }
            }

            /* get energy meter type  from EMETER_INT_EADD and EMETER_SCU_INT_EADD   */
            eeprom_param_get(EMETER_SCU_INT_EADD, (uint8_t *)&pass[5], 1);
            eeprom_param_get(EMETER_INT_EADD, (uint8_t*)&pass, 1);
            if ((pass[0] == EMETER_THREE_PH_ALGO2) || (pass[0] == EMETER_MONO_PH_ALGO2))
            {
              if (pass[5] != EMETER_TYPE_NULL) pass[0] = pass[5];
            }

            cnt = 0;
            /*       destination       source             len   */
            memcpy((void*)pEmeter[11], (void*)"disabled", (size_t)8);
            switch (pass[0])
            {
              case EMETER_TYPE_NULL: 
                cnt = 0;  
                break;
              case EMETER_TAMP:   
                cnt = 1;  
                break;
              case EMETER_MONO_PH_GAVAZZI:
                cnt = 2;  
                break;
              case EMETER_MONO_PH_ALGO2:
                cnt = 3;  
                break;
              case EMETER_THREE_PH_GAVAZZI:
                cnt = 4;  
                break;
              case EMETER_THREE_PH_ALGO2:
                cnt = 5;  
                break;
              case EMETER_TAMP_3:   
                cnt = 6;  
                break;
              case EMETER_MONO_PH_LOVATO:   
                cnt = 7;  
                break;
              case EMETER_THREE_PH_LOVATO:   
                cnt = 8;  
                break;
              case EMETER_MONO_PH_SCAME:   
                cnt = 9;  
                break;
              case EMETER_THREE_PH_SCAME:   
                cnt = 10;  
                break;
            }
            if ((cnt == 9) || (cnt == 10))
            {
              /*       destination       source             len   */
              memcpy((void*)pEmeter[11], (void*)"enabled ", (size_t)8);
            }

            for (i = 0; i < 11; i++)
            {
              if ((char)i == cnt)
              {
                /*       destination       source            len   EMETER_MONO_PH_GAVAZZI */
                memcpy((void*)pEmeter[i], (void*)"checked", (size_t)7);
              }
              else
              {
                /*       destination       source             len   */
                memcpy((void*)pEmeter[i], (void*)"       ", (size_t)7);
              }
            }

            /* get energy meter code  currently detected by SCU    */
            pass[5] = (char)get_emeter_detected_type(INTERNAL_EM);
            len = sprintf((char*)&pass[10], "%X", pass[5]);
            /*       destination      source            len   */
            memcpy ((void*)pEmCode, (void*)&pass[10], len );
            

            /* set energy meter type  from STRIP_LED_TYPE_EADD   */
            eeprom_param_get(STRIP_LED_TYPE_EADD, (uint8_t *)pass, 1);
            cnt = 0;
            switch (pass[0])
            {
              case LED_STRIP_18:
                cnt = 3;  
                break;
              case LED_STRIP_09:   
                cnt = 1;  
                break;
              case LED_STRIP_12:
                cnt = 2;  
                break;
              default: 
              case LED_STRIP_06: 
                cnt = 0;  
                break;
            }
            for (i = 0; i < LED_STRIP_NUM; i++)
            {
              if ((char)i == cnt)
              {
                /*       destination       source            len    */
                memcpy((void*)pLedS[i], (void*)"checked", (size_t)7);
              }
              else
              {
                /*       destination       source             len   */
                memcpy((void*)pLedS[i], (void*)"       ", (size_t)7);
              }
            }

            /* get corrente massima in Modo3 standard [A]  from M3T_CURRENT_EADD   */
            eeprom_param_get(M3T_CURRENT_EADD, (uint8_t *)pass, 1);
            pass[10] = pass[11] = ' ';
            len = sprintf((char*)&pass[10], "%d", pass[0]);
            /*       destination      source            len   */
            memcpy ((void*)pItypical, (void*)&pass[10], len );

            /* get corrente massima in Modo3 standard [A]  from M3S_CURRENT_EADD   */
            eeprom_param_get(M3S_CURRENT_EADD, (uint8_t *)pass, 1);
            pass[10] = pass[11] = ' ';
            len = sprintf((char*)&pass[10], "%d", pass[0]);
            /*       destination      source            len   */
            memcpy ((void*)pSempl, (void*)&pass[10], len );

            /* set backup status flag   from BATTERY_CONFIG_EADD  */
            eeprom_param_get(BATTERY_CONFIG_EADD, (uint8_t *)pass, 1);
            if (pass[0] != (char)0)
            {
              /*       destination       source            len   BATTERY_CONFIG_EADD */
              memcpy((void*)pBackup, (void*)"checked", (size_t)7);

            }
            else
            {
              /*       destination       source            len   BATTERY_CONFIG_EADD */
              memcpy((void*)pBackup, (void*)"       ", (size_t)7);
            }

            /* get SCU address        */
            eeprom_param_get(RS485_ADD_EADD, (uint8_t *)pass, 1);
            pass[10] = pass[11] = ' ';
            if (pass[0] < SCU_NUM)
            {
              pass[0]++;  // physical address 0..15 logical address 1..16
            }
            else
            {
              if (pass[0]  != (SCU_S_REPL_ADDR - 1))
              {
                pass[0] = 88; // default for SCU address error
              }
              else
              {
                pass[0] = SCU_S_REPL_ADDR;
              }
            }
            len = sprintf((char*)&pass[10], "%2d", pass[0]);
            /*       destination      source            len   */
            memcpy ((void*)pBusAdd, (void*)&pass[10], len );

            /* set SCU opertive mode SEM = 0 / EMUMAX0 = 1       */
            eeprom_param_get(OPERATIVE_MODE_EADD, (uint8_t *)pass, 1);
            /*       destination       source             len   */
            memcpy((void*)pOpMode[0], (void*)"       ", (size_t)7);
            /*       destination       source             len   */
            memcpy((void*)pOpMode[1], (void*)"       ", (size_t)7);
            /*       destination       source             len   */
            memcpy((void*)pOpMode[2], (void*)"       ", (size_t)7);
            /*       destination       source             len   */
            memcpy((void*)pOpMode[3], (void*)"       ", (size_t)7);
            /*       destination       source             len   */
            memcpy((void*)pOpMode[4], (void*)"       ", (size_t)7);
            switch (pass[0])
            {
              case 0:  // SEM MASTER PRIMARIO
                /*       destination       source            len    */
                memcpy((void*)pOpMode[1], (void*)"checked", (size_t)7);
                break;

              case 2:  // SEM SLAVE
                /*       destination       source            len    */
                memcpy((void*)pOpMode[2], (void*)"checked", (size_t)7);
                break;

              case 3:  // SEM MASTER SECONDARIO
                /*       destination       source            len    */
                memcpy((void*)pOpMode[3], (void*)"checked", (size_t)7);
                break;

              case 4:  // SEM ISOLATED
                /*       destination       source            len    */
                memcpy((void*)pOpMode[4], (void*)"checked", (size_t)7);
                break;

              default: // EMUMAX0
                /*       destination       source            len    */
                memcpy((void*)pOpMode[0], (void*)"checked", (size_t)7);
                break;
            }
            /* set SCU socket position        */
            eeprom_param_get(LCD_TYPE_EADD, (uint8_t *)pass, 1);
            /*       destination       source             len   */
            memcpy((void*)pReadSp[0], (void*)"       ", (size_t)7);
            /*       destination       source             len   */
            memcpy((void*)pReadSp[1], (void*)"       ", (size_t)7);
            /*       destination       source             len   */
            memcpy((void*)pReadSp[2], (void*)"       ", (size_t)7);
            /*       destination       source             len   */
            memcpy((void*)pReadSp[3], (void*)"       ", (size_t)7);
            switch (pass[0] & SKT_POS_MASK)
            {
              case SKT_HIHG_DX:  
                /*       destination       source            len    */
                memcpy((void*)pReadSp[1], (void*)"checked", (size_t)7);
                break;

              case SKT_HIHG_SX:  
                /*       destination       source            len    */
                memcpy((void*)pReadSp[0], (void*)"checked", (size_t)7);
                break;

              case SKT_LOW_DX:  
                /*       destination       source            len    */
                memcpy((void*)pReadSp[3], (void*)"checked", (size_t)7);
                break;

              default: // SKT_LOW_SX
                /*       destination       source            len    */
                memcpy((void*)pReadSp[2], (void*)"checked", (size_t)7);
                break;
            }
          }

          /* set address mode from  SEM_FLAGS_CTRL_EADD  */
          eeprom_param_get(SEM_FLAGS_CTRL_EADD, (uint8_t *)pass, 1);
          if ((pass[0] & SCU_ADDR_MODE_MASK) == SCU_FIXED_ADDR)
          {
            /*       destination       source            len    */
            memcpy((void*)pAddMode, (void*)"checked", (size_t)7);

          }
          else
          {
            /*       destination       source            len    */
            memcpy((void*)pAddMode, (void*)"       ", (size_t)7);
          }

          freeUse = getStationProductCodeString();
          pass[0] = (char)strlen(freeUse);
          if ((pProdConfData != NULL) && (pass[0] < PRODUCT_CODE_LENGTH))
          {
            /*       destination            source            len    */
            memcpy((void*)pProdConfData, (void*)freeUse,    (size_t)pass[0]);
          }
          pProdConfData = NULL;
          

          writePcb(hs, &heapFile, p, pcb);
        }
        else if ((htmlpage == sinapsiConf) && ((strncmp(data, "POST /sinapsiCheck", 18) == 0)))
        {
          ptr = (char*)strstr ((char*)data, "check<");

          if (ptr != NULL)
          {
            while (*ptr != '<')
            {
              ptr++;
            }
            ptr++; // point to numeric sting start
            cnt = 0;
            while (ptr[cnt] != '>')
            {
              cnt++;
            }
            ptr[cnt] = '\0'; // put end string 
            checkFlag = (uint64_t)atol(ptr);

            /* flag not set means 1 for SINAPSI context    */
#ifdef COME_ERA
            pass[0] = ((char)(checkFlag)  == (char)0) ? (char)1 : (char)0;
            if (pEmSinapsiInfo->getConfigMode != (uint16_t)pass[0])
            {
              /* we write activation BT flag only when different  */
              sinapsiBTflagAct();
            }
#else
            pass[0] = (char)(checkFlag);
            manageBTactFlagSinapsi(pass[0]);
#endif
            cnt++;
            /*** now find Cfg Word value */
            ptr = (char*)strstr ((char*)&ptr[cnt], "CfgW<");
            while (*ptr != '<')
            {
              ptr++;
            }
            ptr++; // point to numeric configuration word
            cnt = 0;
            while (ptr[cnt] != '>')
            {
              cnt++;
            }
            ptr[cnt] = '\0'; // put end string 
            /* set configuration word     */
            i = (int32_t)atoi (ptr);
            sinapsiCWFlagSet(i);

            /* end action on sinapsi BT activation flag   */
            /* Load mainPage/index page */
            /* send uploaddone.html page */
            fs_open(&heapFile, "/uploaddone.html");
            writePcb(hs, &heapFile, p, pcb);
          }
        }
        else if (((strncmp(data, "POST /SinapsiConf.html", 22) == 0) && (htmlpage == mainPage)) ||
                 ((strncmp(data, "GET /SinapsiConf.html", 21) == 0) && (htmlpage == sinapsiConf))) 
        {
          htmlpage = sinapsiConf;
          DataFlag = 0;
          fs_open(&heapFile, "/sinapsiInfo.html");

          /* modify the tag inside html page */
          if (pSiBox[0] == NULL)
          {
            if ((pLang[0] != NULL) && (pSpecFuncData  != NULL))
            {
              if (pSpecFuncData != NULL) free(pSpecFuncData);
              pSpecFuncData = NULL;
              pLang[0] = NULL;
            }
            pSiConfData = malloc(heapFile.len); 

            if (pSiConfData != NULL)
            {
              /*       destination       source     len   */
              memcpy(pSiConfData, heapFile.data, heapFile.len);
              pSiBox[0] = strstr(pSiConfData, (char*)"@0a");            // Fw Ver
              pSiBox[1] = strstr(pSiConfData, (char*)"@U28072022");     // IOM2GSN
              pSiBox[2] = strstr(pSiConfData, (char*)"@c");             // IOM2GTP 
              pSiBox[3] = strstr(pSiConfData, (char*)"@d");             // BTACTR
              pSiBox[4] = strstr(pSiConfData, (char*)"@001");           // M1RxQ
              pSiBox[5] = strstr(pSiConfData, (char*)"@f");             // M1PLC 
              pSiBox[6] = strstr(pSiConfData, (char*)"@002");           // M2RxQ   
              pSiBox[7] = strstr(pSiConfData, (char*)"@h");             // M2PLC 
              pSiBox[8] = strstr(pSiConfData, (char*)"@003");           // BIU 
              pSiBox[9] = strstr(pSiConfData, (char*)"@000000");        // IOM2GDU 
              pSiBox[10] = strstr(pSiConfData, (char*)"@0001");         // M1CNT 
              pSiBox[11] = strstr(pSiConfData, (char*)"@0002");         // M1LOST 
              pSiBox[12] = strstr(pSiConfData, (char*)"@0003");         // M2CNT 
              pSiBox[13] = strstr(pSiConfData, (char*)"@0004");         // M2LOST 
              pSiBox[14] = strstr(pSiConfData, (char*)"@0:A1:A2:00:01:03");         // MAC 
              pSiBox[15] = strstr(pSiConfData, (char*)"@01");           // CONTRA  

              pSiBox[16] = strstr(pSiConfData, (char*)"@z00000001");    // M1TSPIMMIST 
              pSiBox[17] = strstr(pSiConfData, (char*)"@z000002");      // M1PIMMIST 
              pSiBox[18] = strstr(pSiConfData, (char*)"@z00000003");    // M2TSPIMMIST 
              pSiBox[19] = strstr(pSiConfData, (char*)"@z000004");      // M2PIMMIST 
              pSiBox[20] = strstr(pSiConfData, (char*)"@z005");         // M1PCONTR 
              pSiBox[21] = strstr(pSiConfData, (char*)"@z006");         // M1PDISP 
              pSiBox[22] = strstr(pSiConfData, (char*)"@z00000007");    // TSDIST 
              pSiBox[23] = strstr(pSiConfData, (char*)"@z008");         // DIST 
              pSiBox[24] = strstr(pSiConfData, (char*)"@z00000009");    // TSFASCIA  
              pSiBox[25] = strstr(pSiConfData, (char*)"@z010");         // FASCIA 
              pSiBox[26] = strstr(pSiConfData, (char*)"@z00000011");    // TSPAPREIST 
              pSiBox[27] = strstr(pSiConfData, (char*)"@z000012");      // PAPREIST 
              pSiBox[28] = strstr(pSiConfData, (char*)"@z000013");      // PAPREMAX 
              pSiBox[29] = strstr(pSiConfData, (char*)"@z000014");      // IAPREMAX 

              pSiBox[30] = strstr(pSiConfData, (char*)"check01");       // check configuration status  

              pSiBox[31] = strstr(pSiConfData, (char*)"@z00000012");    // CONFIGURATION WORD 
              pSiBox[32] = strstr(pSiConfData, (char*)"check02");       // check delibera 541  
              pSiBox[33] = strstr(pSiConfData, (char*)"@z000015");      // Pest, only for debug  
            }
          }

          if (pSiBox[0] == NULL)
          {
            if (pSiConfData != NULL) free(pSiConfData);
            pSiConfData = NULL;
          }
          else
          {
            heapFile.data = (const char*)pSiConfData;

            /**** IOM2G general data **** */
            sprintf(pass, "%d.%d", (uint8_t)(pEmSinapsiInfo->duFwVer / 256), (uint8_t)(pEmSinapsiInfo->duFwVer % 256)); 
            /*       destination       source     3   */
            memcpy((void*)pSiBox[0], (void*)pass, (size_t)3);

            /*       destination                                  source                 10   */
            memcpy((void*)pSiBox[1], (void*)emMsg.emModelInfo[EXTERNAL_EM].modelSN, (size_t)10);

            sprintf(pass, "%2d", emMsg.emModelInfo[EXTERNAL_EM].modelType); 
            /*       destination       source         2   */
            memcpy((void*)pSiBox[2], (void*)pass, (size_t)2);

            sprintf(pass, "%2d", pEmSinapsiInfo->getConfigMode); 
            /*       destination       source         2   */
            memcpy((void*)pSiBox[3], (void*)pass, (size_t)2);

            sprintf(pass, "%4d", pEmSinapsiInfo->m1PlccRssi); 
            /*       destination       source         4   */
            memcpy((void*)pSiBox[4], (void*)pass, (size_t)4);

            sprintf(pass, "%2d", pEmSinapsiInfo->m1PlcState); 
            /*       destination       source         2   */
            memcpy((void*)pSiBox[5], (void*)pass, (size_t)2);

            sprintf(pass, "%4d", pEmSinapsiInfo->m2PlccRssi); 
            /*       destination       source         4   */
            memcpy((void*)pSiBox[6], (void*)pass, (size_t)4);

            sprintf(pass, "%2d", pEmSinapsiInfo->m2PlcState); 
            /*       destination       source         2   */
            memcpy((void*)pSiBox[7], (void*)pass, (size_t)2);

            sprintf(pass, "%4d", pEmSinapsiInfo->biu); 
            /*       destination       source         4   */
            memcpy((void*)pSiBox[8], (void*)pass, (size_t)4);

            sprintf(pass, "%7d", pEmSinapsiInfo->duUpTime); 
            /*       destination       source         7   */
            memcpy((void*)pSiBox[9], (void*)pass, (size_t)7);

            sprintf(pass, "%5d", pEmSinapsiInfo->cfCountTotM1); 
            /*       destination       source         5   */
            memcpy((void*)pSiBox[10], (void*)pass, (size_t)5);

            sprintf(pass, "%5d", pEmSinapsiInfo->cfCountLostTotM1); 
            /*       destination       source         5   */
            memcpy((void*)pSiBox[11], (void*)pass, (size_t)5);

            sprintf(pass, "%5d", pEmSinapsiInfo->cfCountTotM2); 
            /*       destination       source         5   */
            memcpy((void*)pSiBox[12], (void*)pass, (size_t)5);

            sprintf(pass, "%5d", pEmSinapsiInfo->cfCountLostTotM2); 
            /*       destination       source         5   */
            memcpy((void*)pSiBox[13], (void*)pass, (size_t)5);

            sprintf(pass, "%02X:%02X:%02X:%02X:%02X:%02X", pEmSinapsiInfo->duMAC[0], pEmSinapsiInfo->duMAC[1], pEmSinapsiInfo->duMAC[2],
                                                           pEmSinapsiInfo->duMAC[3], pEmSinapsiInfo->duMAC[4], pEmSinapsiInfo->duMAC[5]);
            /*       destination       source         17   */
            memcpy((void*)pSiBox[14], (void*)pass, (size_t)17);

            sprintf(pass, "%3d", (pEmSinapsiInfo->costTrasf)); 
            /*       destination       source     3   */
            memcpy((void*)pSiBox[15], (void*)pass, (size_t)3);

            /**** IOM2G power Info **** */
            sprintf(pass, "%10d", pEmSinapsiInfo->potAttImmIstTs); 
            /*       destination       source     10   */
            memcpy((void*)pSiBox[16], (void*)pass, (size_t)10);

            size = (uint32_t)arrayTo64(pEmSinapsiInfo->potAttImmIst);
            sprintf(pass, "%8d", size); 
            /*       destination       source           8   */
            memcpy((void*)pSiBox[17], (void*)pass, (size_t)8);

            sprintf(pass, "%10d", pEmSinapsiInfo->potAttImmIstTsM2); 
            /*       destination       source     10   */
            memcpy((void*)pSiBox[18], (void*)pass, (size_t)10);

            size = (uint32_t)arrayTo64(pEmSinapsiInfo->potAttImmIstM2);
            sprintf(pass, "%8d", size); 
            /*       destination       source           8   */
            memcpy((void*)pSiBox[19], (void*)pass, (size_t)8);

            sprintf(pass, "%5d", pEmSinapsiInfo->potContrattuale); 
            /*       destination       source         5   */
            memcpy((void*)pSiBox[20], (void*)pass, (size_t)5);

            sprintf(pass, "%5d", pEmSinapsiInfo->potDisponibile); 
            /*       destination       source         5   */
            memcpy((void*)pSiBox[21], (void*)pass, (size_t)5);

            sprintf(pass, "%10d", pEmSinapsiInfo->tempoDistaccoTs); 
            /*       destination       source     10   */
            memcpy((void*)pSiBox[22], (void*)pass, (size_t)10);

            sprintf(pass, "%5d", pEmSinapsiInfo->tempoDistacco); 
            /*       destination       source          5   */
            memcpy((void*)pSiBox[23], (void*)pass, (size_t)5);

            sprintf(pass, "%10d", pEmSinapsiInfo->fasciaCorrenteTs); 
            /*       destination       source          10   */
            memcpy((void*)pSiBox[24], (void*)pass, (size_t)10);

            sprintf(pass, "%5d", pEmSinapsiInfo->fasciaCorrente); 
            /*       destination       source          5   */
            memcpy((void*)pSiBox[25], (void*)pass, (size_t)5);

            sprintf(pass, "%10d", pEmSinapsiInfo->potAttPreIstTs); 
            /*       destination       source     10   */
            memcpy((void*)pSiBox[26], (void*)pass, (size_t)10);

            //size = (uint32_t)arrayTo64(pEmSinapsiInfo->potAttPreIst);
            //sprintf(pass, "%8d", size); 

            sinapsiSetReg_st*     sinapsi_ptr;
            sinapsi_ptr = getIom2Ginfo();
            sprintf(pass, "%8d", sinapsi_ptr->m1Papi); 
            /*       destination       source           8   */
            memcpy((void*)pSiBox[33], (void*)pass, (size_t)8);

            size = (uint32_t)arrayTo64(pEmSinapsiInfo->potAttPreIst);
            sprintf(pass, "%8d", size); 
            /*       destination       source           8   */
            memcpy((void*)pSiBox[27], (void*)pass, (size_t)8);

            sprintf(pass, "%8d", pEmSinapsiInfo->potAttPreMax); 
            /*       destination       source           8   */
            memcpy((void*)pSiBox[28], (void*)pass, (size_t)8);

            sprintf(pass, "%8d", pEmSinapsiInfo->corAttPreMax); 
            /*       destination       source           8   */
            memcpy((void*)pSiBox[29], (void*)pass, (size_t)8);

#ifdef COME_ERA
            if (pEmSinapsiInfo->getConfigMode != (uint16_t)0)
            {
              /* when activation BT flag is != 0 SINAPSI isn't linked to main meter */
              /*       destination       source            len    */
              memcpy((void*)pSiBox[30], (void*)"       ", (size_t)7);
            }
            else
            {
              /* when activation BT flag is 0 SINAPSI is full operative */
              /*       destination          source            len    */
              memcpy((void*)pSiBox[30], (void*)"checked", (size_t)7);
            }
#else
            /* now the BT flag means restore initial condition. This means the check box is, by default, not active */
            /*       destination       source            len    */
            memcpy((void*)pSiBox[30], (void*)"       ", (size_t)7);
#endif
            sprintf(pass, "%10d", pEmSinapsiInfo->cfgDU); 
            /*       destination       source     10    */
            memcpy((void*)pSiBox[31], (void*)pass, (size_t)10);

            if (pEmSinapsiInfo->delibera541 != (uint16_t)0)
            {
              /*       destination          source            len    */
              memcpy((void*)pSiBox[32], (void*)"checked", (size_t)7);
            }
            else
            {
              /*       destination       source            len    */
              memcpy((void*)pSiBox[32], (void*)"       ", (size_t)7);
            }
          }
          if (pSiConfData != NULL) 
          {
            free(pSiConfData);
            pSiBox[0] = NULL;
            pSiConfData = NULL;
          }

          writePcb(hs, &heapFile, p, pcb);
        }
        else if ((strncmp(data, "GET /chgPsw.cgi", 15) == 0) && (htmlpage == changePassword))
        {
          ptr = (char*)strstr ((char*)data, "psw1=");
          while (*ptr != '=')
          {
            ptr++;
          }
          ptr++; // point to beginnning of service password
          cnt = 0;
          while (ptr[cnt] != '&')
          {
            pass[cnt] = ptr[cnt];
            cnt++;
          }
          pass[cnt] = '\0';
          pSiBox[0] = url_decode(pass);
          if (pSiBox[0] != NULL)
          {
            /*         destination       source */
            strcpy((char *)pass, (char *)pSiBox[0]);
            free(pSiBox[0]);
          }
          result =  (uint8_t)SecureArea_storeWebPassword(pass, (uint16_t)strlen(pass));
          if (result != 0) tPrintf( "Error storing data\r\n");

          fs_open(&heapFile, "/uploaddone.html");
          writePcb(hs, &heapFile, p, pcb);
          
          activeImmediateReset();
        }
        else if ((strncmp(data, "GET /style.css", 14) == 0) && ((htmlpage == infoHwConfRx) || (htmlpage == infoSpecFunc) || 
                                                                (htmlpage == modelConfFunc) || (htmlpage == sinapsiConf) ))
        {
          /* request css style  */
          fs_open(&heapFile, "/style.css");
          writePcb(hs, &heapFile, p, pcb);
        }
        else if ((strncmp(data, "GET /style2.css", 15) == 0) && ((htmlpage == passwordConf) || (htmlpage == mainPage)))
        {
          /* request css style  */
          fs_open(&heapFile, "/style2.css");
          writePcb(hs, &heapFile, p, pcb);
        }
        else if (((strncmp(data, "POST /gsyCheck", 14) == 0)) && (htmlpage == infoHwConfRx))
        {
          ptr = (char*)strstr ((char*)data, "check<");
          pHgtp = (char*)strstr ((char*)data, "hgtp<");
          pAct = (char*)strstr ((char*)data, "act<");
          pSocket = (char*)strstr ((char*)data, "skType<");
          pEm = (char*)strstr ((char*)data, "emCode<");
          pImin = (char*)strstr ((char*)data, "ITyp<");
          pDset = (char*)strstr ((char*)data, "ISem<");
          pBack = (char*)strstr ((char*)data, "Back<");
          pAddr = (char*)strstr ((char*)data, "iAddr<");
          pLs = (char*)strstr ((char*)data, "lsCode<");
          pOm = (char*)strstr ((char*)data, "moCode<");
          pWDef = (char*)strstr ((char*)data, "wDef<");
          pSkPos = (char*)strstr ((char*)data, "spCode<");
          pSemFlags = (char*)strstr ((char*)data, "semFlag<");
          pEmx = (char*)strstr ((char*)data, "emClear<");
          
          
          if ((ptr != NULL) && (pAct != NULL) && (pSocket != NULL) && (pBack != NULL) && (pAddr != NULL) && (pEm != NULL) && 
              (pOm != NULL) && (pWDef != NULL) && (pSkPos != NULL)  && (pSemFlags != NULL)  && (pHgtp != NULL))
          {
            while (*ptr != '<')
            {
              ptr++;
            }
            ptr++; // point to numeric sting start
            cnt = 0;
            while (ptr[cnt] != '>')
            {
              cnt++;
            }
            ptr[cnt] = '\0'; // put end string 
            checkFlag = (uint64_t)atol(ptr);

            /* set RCDM flag and all bit on CONTROL_BYTE0   */
            pass[0] = (char)(checkFlag);
            /* set CPSHORT flag and all bit on CONTROL_BYTE1   */
            pass[1] = (char)(checkFlag >> 8);
            /* set RECTIFIER flag and all bit on CONTROL_BYTE2   */
            pass[10] = (char)(checkFlag >> 16);
            eeprom_param_get(CONTROL_BYTE2_EADD, (uint8_t*)&pass[2], 1);
            pass[2] &= (~RECTIFIER_CRL2);
            if (pass[10] != 0)
            {
              pass[2] |= (RECTIFIER_CRL2);            
            }

            /* now we set the high temperature control bit */
            while (*pHgtp != '<')
            {
              pHgtp++;
            }
            pHgtp++; // point to numeric sting start
            cnt = 0;
            while (ptr[cnt] != '>')
            {
              cnt++;
            }
            pHgtp[cnt] = '\0'; // put end string 
            pass[10] = (uint8_t)atoi(pHgtp);
            eeprom_param_get(TEMP_CTRL_ENB_EADD, (uint8_t*)&pass[11], 1);
            pass[11] &= (~CTRL_HGTP_BIT);
            if ((pass[10] & CTRL_HGTP_BIT) != 0)
            {
              pass[11] |= (CTRL_HGTP_BIT);            
            }
            /* now save in EEPROM the control high temperature bit */
            // xx eeprom_array_set(TEMP_CTRL_ENB_EADD, (uint8_t *)&pass[11], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.Temp_Ctrl.Enabled, (uint8_t *)&pass[11], 1);          /* ex TEMP_CTRL_ENB_EADD */
            
            /* spare */
            pass[3] = (char)0;

            /* now find actuator string */
            while (*pAct != '<')
            {
              pAct++;
            }
            pAct++; // point to numeric sting start
            cnt = 0;
            while (pAct[cnt] != '>')
            {
              cnt++;
            }
            pAct[cnt] = '\0'; // put end string 
            checkFlag = (uint64_t)atol(pAct);
            /* set Actuators flag    */
            pass[4] = (char)(checkFlag);

            /* get current Post Authorization Mode */
            eeprom_param_get(ACTUATORS_EADD, (uint8_t*)&pass[5], 1);
            pass[5] &= PAUT_ATT0;
            if (pass[5] != (pass[4] & PAUT_ATT0))
            {
              /* a change on block mode has been made */
              activeImmediateReset();
            }

            /* now save in EEPROM check flag and actuator presence */
            // xx eeprom_array_set(CONTROL_BYTE0_EADD, (uint8_t *)pass, 5);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.controlByte.Byte.Byte0, (uint8_t *)pass, 5);       /* ex CONTROL_BYTE0_EADD */
            
            if ((rfid_state_get() == RFID_ERROR) && (pass[1] & MIFARE_CRL1))
                send_to_rfid(RFID_CONTROL_UPDATE);

            /* Set in modbus map */
            setHwChecks((pass[1] << 8) | pass[0], (pass[3] << 8) | pass[2]);
            pass[4] &= (~BBCK_ATT0);  // clear battery backup flag
            setHwActuators(pass[4]);

            /* now find socket value */
            while (*pSocket != '<')
            {
              pSocket++;
            }
            pSocket++; // point to numeric sting start
            cnt = 0;
            while (pSocket[cnt] != '>')
            {
              cnt++;
            }
            pSocket[cnt] = '\0'; // put end string 
            /* set socket code     */
            pass[5] = (char)strtoll (pSocket, NULL, 16);
            // xx eeprom_array_set(SOCKET_TYPE_EADD, (uint8_t*)&pass[5], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.socketType, (uint8_t*)&pass[5], 1);    /* ex SOCKET_TYPE_EADD */
            
            /* now find energy meter value */
            while (*pEm != '<')
            {
              pEm++;
            }
            pEm++; // point to numeric sting start
            cnt = 0;
            while (pEm[cnt] != '>')
            {
              cnt++;
            }
            pEm[cnt] = '\0'; // put end string 
            /* set energy meter code      */
            pass[5] = (char)strtoll (pEm, NULL, 16);
            // xx eeprom_array_set(EMETER_SCU_INT_EADD, (uint8_t*)&pass[5], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.EmeterScu_Int, (uint8_t*)&pass[5], 1);    /* ex EMETER_SCU_INT_EADD */
            switch (pass[5])
            {
              case EMETER_MONO_PH_SCAME:
              case EMETER_MONO_PH_LOVATO:
                pass[5] = EMETER_MONO_PH_ALGO2;
                break; 
              case EMETER_THREE_PH_SCAME:
              case EMETER_THREE_PH_LOVATO:
                pass[5] = EMETER_THREE_PH_ALGO2;
                break; 
              case EMETER_TYPE_NULL:
              case EMETER_TAMP:
              case EMETER_TAMP_3:
                send_to_evs(EVS_INTERNAL_EM_GOOD);
                break;
              default:
                break;
            }
            // xx eeprom_array_set(EMETER_INT_EADD, (uint8_t*)&pass[5], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.emTypeInt, (uint8_t*)&pass[5], 1);    /* ex EMETER_INT_EADD */
            setEvsePowerMode();
            
            /* now find corrente massima in Modo3 standard */
            while (*pImin != '<')
            {
              pImin++;
            }
            pImin++; // point to numeric string start
            cnt = 0;
            while (pImin[cnt] != '>')
            {
              cnt++;
            }
            pImin[cnt] = '\0'; // put end string 
            /* set I max Mode 3 standard     */
            pass[5] = (char)atoi (pImin);
            // xx eeprom_array_set(M3T_CURRENT_EADD, (uint8_t*)&pass[5], 1); 
            SCU_InfoStation_Set ((uint8_t *)&infoStation.max_current, (uint8_t*)&pass[5], 1);    /* ex M3T_CURRENT_EADD */
            pass[5] = (char)setNominalPower((uint16_t)pass[5]);
            // xx eeprom_array_set(STATION_NOM_PWR_EADD, (uint8_t*)&pass[5], 1); 
            SCU_InfoStation_Set ((uint8_t *)&infoStation.StationNominalPower, (uint8_t*)&pass[5], 1);    /* ex STATION_NOM_PWR_EADD */
            
            /* now find corrente massima in Modo3 semplificato */
            while (*pDset != '<')
            {
              pDset++;
            }
            pDset++; // point to numeric string start
            cnt = 0;
            while (pDset[cnt] != '>')
            {
              cnt++;
            }
            pDset[cnt] = '\0'; // put end string 
            /* set I max Mode 3 standard     */
            pass[5] = (char)atoi (pDset);
            // xx eeprom_array_set(M3S_CURRENT_EADD, (uint8_t*)&pass[5], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.max_currentSemp, (uint8_t*)&pass[5], 1);    /* ex M3S_CURRENT_EADD */
              
            /* now find backup flag  */
            while (*pBack != '<')
            {
              pBack++;
            }
            pBack++; // point to numeric string start
            cnt = 0;
            while (pBack[cnt] != '>')
            {
              cnt++;
            }
            pBack[cnt] = '\0'; // put end string 
            /* set backup flag: 1 = active       */
            pass[5] = (char)atoi (pBack);
            /* read current value for battery backup and actuators */
            eeprom_param_get(BATTERY_CONFIG_EADD, (uint8_t*)&pass[2], 1);
            eeprom_param_get(ACTUATORS_EADD, (uint8_t*)&pass[6], 1);
            pass[6] &= (~BBCK_ATT0);  // clear battery backup flag 
            /* save current value for battery backup */
            // xx eeprom_array_set(BATTERY_CONFIG_EADD, (uint8_t*)&pass[5], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.batteryConfig, (uint8_t*)&pass[5], 1);    /* ex BATTERY_CONFIG_EADD */  
           
            if (pass[5] != 0)
            {
              pass[6] |= (BBCK_ATT0); // set battery backup flag 
              /* the battery backup will be active */
              if (pass[2] == 0)
              {
                /* the current value for battery backup is "no active" */
                /* the battery backup mode has been activated: a restart it is necessary */
                activeImmediateReset();
              }
            }
            /* save current value for battery backup */
            // xx eeprom_array_set(ACTUATORS_EADD, (uint8_t*)&pass[6], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.actuators, (uint8_t*)&pass[6], 1);    /* ex ACTUATORS_EADD */
            setHwActuators(pass[6]);  // update hw actuators in the modbus map

            /* now find the SCU address on RS485 bus  */
            while (*pAddr != '<')
            {
              pAddr++;
            }
            pAddr++; // point to numeric string start
            cnt = 0;
            while (pAddr[cnt] != '>')
            {
              cnt++;
            }
            pAddr[cnt] = '\0'; // put end string 
            /* set SCU address    */
            pass[7] = (char)atoi (pAddr);
            pass[5] = (pass[7] - 1); // logical address in eeprom is in the range 0..15
            eeprom_param_get(RS485_ADD_EADD, (uint8_t*)&cnt, 1);
            if (cnt != pass[5])
            {
              /* a new address must be set */
              // xx eeprom_array_set(RS485_ADD_EADD, (uint8_t*)&pass[5], 1);
              SCU_InfoStation_Set ((uint8_t *)&infoStation.rs485Address, (uint8_t*)&pass[5], 1);          /* ex RS485_ADD_EADD */
              send_to_lcd(LCD_CURRENT_UPDATE);
              activeImmediateReset();
            }
            /* if physical address is 11..14 the connector position in fake code is 1..4 */
            if ((pass[7] >= (char)SCU_SLAVE_PRINCIPAL_ADDR) && (pass[7] <= (char)SCU_SLAVE_SECONDARY_MAX_ADDR))
            {
              pass[6] = pass[7] % 10;
              /* set index in fake code in the register */
              // xx eeprom_array_set(CONNECTOR_NUMBER_EADD, (uint8_t*)&pass[6], 1);
              SCU_InfoStation_Set ((uint8_t *)&infoStation.connectorNumber, (uint8_t*)&pass[6], 1);          /* ex CONNECTOR_NUMBER_EADD */
            }

            /* now find led strip value */
            while (*pLs != '<')
            {
              pLs++;
            }
            pLs++; // point to numeric sting start
            cnt = 0;
            while (pLs[cnt] != '>')
            {
              cnt++;
            }
            pLs[cnt] = '\0'; // put end string 
            /* set energy meter code     */
            pass[5] = (char)strtoll (pLs, NULL, 16);
            // xx eeprom_array_set(STRIP_LED_TYPE_EADD, (uint8_t*)&pass[5], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.StripLedType, (uint8_t*)&pass[5], 1);    /* ex STRIP_LED_TYPE_EADD */
            setNewCurrentLed();

            /* notify to EVS manager the changes  */
            send_to_evs(EVS_INTERNAL_EM_UPDATE);

            /* now find operative mode setting  value */
            while (*pOm != '<')
            {
              pOm++;
            }
            pOm++; // point to single numeric  char
            pass[5] = (char)1; /* default is EMUMAX0 mode --> GSY */
            if (*pOm == (char)'0')
            {
              pass[5] = (char)SCU_SEM_M; /* SEM master mode */
            }
            else
            {
              if (*pOm == (char)'2')
              {
                 pass[5] = (char)SCU_SEM_S; /* SEM slave mode */
              }
              else
              {
                if (*pOm == (char)'3')
                {
                   pass[5] = (char)SCU_SEM_MS; /* SEM master secondario  mode */
                }
                else
                {
                  if (*pOm == (char)'4')
                  {
                     pass[5] = (char)SCU_SEM_STAND_ALONE; /* SEM master in isolated mode */
                  }
                }
              }
            }
            /* set SCU opertive mode SEM = 0 / EMUMAX0 = 1. Read current status on pass[3]       */
            eeprom_param_get(OPERATIVE_MODE_EADD, (uint8_t *)&pass[3], 1);

            /* Reset Power management Remote if in GSY mode */
            if (pass[5] == SCU_GSY)
              /* Here reset the power management Remote flag */
              ResetPmRemoteSemFlag (0);              
              
            if (pass[3] != pass[5])
            {
              if (pass[5] == SCU_SEM_STAND_ALONE) 
              {
                  /* this SCU will operate in stand alone mode */
                  setEepromArrayIsolatedMode();
                  setAddressType((uint8_t)SCU_FIXED_ADDR, TRUE);
              }
              else
              {
                // xx eeprom_array_set(OPERATIVE_MODE_EADD, (uint8_t*)&pass[5], 1);
                SCU_InfoStation_Set ((uint8_t *)&infoStation.Operative_mode, (uint8_t*)&pass[5], 1);        /* ex OPERATIVE_MODE_EADD */
              }
              /* the SCU enviroment mode has been changed: a restart it is necessary */
              activeImmediateReset();
            }
            if (pass[5] != SCU_SEM_STAND_ALONE)  /* if SEM Sandalone mode is selected no more action are need*/
            {
              /*** now find if that it is a new default product parameters  */
              while (*pWDef != '<') 
              {
                pWDef++;
              }
              pWDef++; // point to numeric Diff. Riarm status
              cnt = 0;
              while (pWDef[cnt] != '>')
              {
                cnt++;
              }

              pWDef[cnt] = '\0'; // put end string 
              /* set wifi mode  value     */
              i = (int32_t)atoi (pWDef);
              pass[0] = (char)(i);
              pass[20] = FALSE;
              /* set new default if true       */
              if (pass[0] != 0)  
              {
                pass[20] = TRUE;
              }

              /* now find socket position code value */
              while (*pSkPos != '<')
              {
                pSkPos++;
              }
              pSkPos++; // point to numeric socket position code 
              cnt = 0;
              while (pSkPos[cnt] != '>')
              {
                cnt++;
              }
              pSkPos[cnt] = '\0'; // put end string 
              /* set socket position  value     */
              i = (int32_t)atoi (pSkPos);

              /* set SCU socket position. Read current status on pass[3]       */
              eeprom_param_get(LCD_TYPE_EADD, (uint8_t *)&pass[3], 1);

              switch ((uint8_t)i)
              {
                case SKT_HIHG_DX:
                case SKT_HIHG_SX:
                case SKT_LOW_DX:
                case SKT_LOW_SX:
                  pass[14] = ((pass[3] & (uint8_t)(~SKT_POS_MASK)) | (uint8_t)i);
                  // xx eeprom_array_set(LCD_TYPE_EADD, (uint8_t*)&pass[14], 1);
                  SCU_InfoStation_Set ((uint8_t *)&infoStation.LcdType, (uint8_t*)&pass[14], 1);     /* ex LCD_TYPE_EADD */
                  break;

                default:
                  break;
              }

              /* now find SEM flags  */
              if (pass[5] != SCU_GSY)
              {
              
                while (*pSemFlags != '<')
                {
                  pSemFlags++;
                }
                pSemFlags++; // point to numeric string start
                cnt = 0;
                while (pSemFlags[cnt] != '>')
                {
                  cnt++;
                }
                pSemFlags[cnt] = '\0'; // put end string 
                /* set backup flag: 1 = active       */
                pass[5] = (char)atoi (pSemFlags);
                
                /* read current value for SEM Flags */
                eeprom_param_get(SEM_FLAGS_CTRL_EADD, (uint8_t*)&pass[2], 1);
                pass[2] &= (~SCU_ADDR_MODE_MASK);
                pass[2] |= (pass[5] & SCU_ADDR_MODE_MASK);
                setAddressType((uint8_t)(pass[5] & SCU_ADDR_MODE_MASK), FALSE);
                /* save current value for SEM Flags */
                // xx eeprom_array_set(SEM_FLAGS_CTRL_EADD, (uint8_t*)&pass[2], 1);
                SCU_InfoStation_Set ((uint8_t *)&infoStation.semFlagControl, (uint8_t*)&pass[2], 1);        /* ex SEM_FLAGS_CTRL_EADD */
                /* upgrade modbus map */
                upgradeModbusHwConfig();
              
              }
              else
              {
                /* read current value for SEM Flags and put No fixed address */
                eeprom_param_get(SEM_FLAGS_CTRL_EADD, (uint8_t*)&pass[5], 1);
                pass[5] &= (~SCU_ADDR_MODE_MASK);
                setAddressType((uint8_t)(pass[5] & SCU_ADDR_MODE_MASK), FALSE);
                /* save current value for SEM Flags */
                // xx eeprom_array_set(SEM_FLAGS_CTRL_EADD, (uint8_t*)&pass[5], 1);
                SCU_InfoStation_Set ((uint8_t *)&infoStation.semFlagControl, (uint8_t*)&pass[5], 1);        /* ex SEM_FLAGS_CTRL_EADD */
              }

              while (*pEmx != '<')
              {
                pEmx++;
              }
              pEmx++; // point to numeric string start
              if (*pEmx != '0')
              {
                /* it necessary to reset active energy for EM Scame */
                /* clear energy value This is important for EM Scame */
                pass[10] = pass[11] = pass[12] = pass[13] = 0;
                // xx eeprom_array_set(TOT_ENERGY0_EADD, (uint8_t*)&pass[10], 4);
                SCU_InfoStation_Set ((uint8_t *)&infoStation.TotalEnergy, (uint8_t*)&pass[10], 4);       /* ex TOT_ENERGY0_EADD */
                HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), (uint32_t)BACKUP_EM_ENRG_ACT, 0L); 
                HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), (uint32_t)BACKUP_SCAME_TOTAL_ENRG, 0L); 
              }
            }

            if (pass[20] == TRUE)  
            {
              eeprom_default_set();
            }
            
            /* Send event to update eeprom */
            // send_to_eeprom(EEPROM_UPDATE); 

            /* end action on hwConf parameters  */
            /* Load mainPage/index page */
            /* send uploaddone.html page */
            fs_open(&heapFile, "/uploaddone.html");
            writePcb(hs, &heapFile, p, pcb);
          }
        }
        else if ((strncmp(data, "GET /hwCheckX", 13) == 0) && (htmlpage == infoHwConfRx))
        {
          /* end action on hwConf parameters  */
          /* Load mainPage/index page */
          fs_open(&heapFile, "/mainPage.html");
          writePcb(hs, &heapFile, p, pcb);
          return ERR_OK;
        }
        else if (((strncmp(data, "POST /gsySpFunc", 15) == 0)) && (htmlpage == infoSpecFunc))
        {
          ptr = (char*)strstr ((char*)data, "Lingua<");
          pAct = (char*)strstr ((char*)data, "gLingue<");
          pSocket = (char*)strstr ((char*)data, "abPM<");
          pEm = (char*)strstr ((char*)data, "parPMAX<");
          pImin = (char*)strstr ((char*)data, "parIMIN<");
          pHpwr = (char*)strstr ((char*)data, "parHPWR<");
          pDset = (char*)strstr ((char*)data, "parDSET<");
          pDmax = (char*)strstr ((char*)data, "parDMAX<");
          pUnb = (char*)strstr ((char*)data, "parUNB<");
          pEmx = (char*)strstr ((char*)data, "parEMX<");
          pRfas = (char*)strstr ((char*)data, "parRFAS<");
          pAbCT = (char*)strstr ((char*)data, "abCT<");
          pMaxTric = (char*)strstr ((char*)data, "parMAXTRIC<");
          pGMT = (char*)strstr ((char*)data, "parGMT<");
          pDST = (char*)strstr ((char*)data, "parDST<");
          pDeT = (char*)strstr ((char*)data, "parDeT<");
          pModeOp = (char*)strstr ((char*)data, "parPmodeOp<");
          pLcdType = (char*)strstr ((char*)data, "parTypeLcd<");
          pWiFiMode = (char*)strstr ((char*)data, "parWiFi<");
          pDiRiMode = (char*)strstr ((char*)data, "parDiRi<");
          pMaxEnergy = (char*)strstr ((char*)data, "parENRG<");
          pWDef = (char*)strstr ((char*)data, "wDef<");
#ifdef CHANGE_ID
          pPresSk = (char*)strstr ((char*)data, "pSck<");
#endif
          pChId = (char*)strstr ((char*)data, "wChId<");
          
          
          if ((ptr != NULL) && (pAct != NULL) && (pSocket != NULL) && (pEm != NULL) &&
              (pImin != NULL) && (pHpwr != NULL) && (pDset != NULL) && (pDmax != NULL) &&
              (pUnb != NULL) && (pEmx != NULL) && (pRfas != NULL) && (pAbCT != NULL) && (pMaxTric != NULL) &&
              (pGMT != NULL) && (pDST != NULL) && (pDeT != NULL) && (pModeOp != NULL) && (pLcdType != NULL) && 
              (pWiFiMode != NULL) && (pDiRiMode != NULL) && (pMaxEnergy != NULL) && (pWDef != NULL) && (pChId != NULL))
          {
            while (*ptr != '<')
            {
              ptr++;
            }
            ptr++; // point to numeric sting start
            cnt = 0;
            while (ptr[cnt] != '>')
            {
              cnt++;
            }
            ptr[cnt] = '\0'; // put end string 
            checkFlag = (uint64_t)atol(ptr);

            /* Translate Language set from modbus format to eeprom format */
            LANG_Modbus_to_EEprom_Translate((uint32_t)checkFlag);

            /*** now find group language string */
            while (*pAct != '<')
            {
              pAct++;
            }
            pAct++; // point to numeric sting start
            cnt = 0;
            while (pAct[cnt] != '>')
            {
              cnt++;
            }
            pAct[cnt] = '\0'; // put end string 
            checkFlag = (uint64_t)atol(pAct);
            pass[0] = pass[1] = pass[2] = pass[3] = (char)0;
            for (i = 0, len = 1, normLen = 0, cnt = 1; i < (int32_t)NUM_LANGUAGE; i++, len = len << 1, cnt = cnt << 1)
            {
              if (len >= ((int32_t)0x100 << (8 * normLen))) 
              {
                /* next language group byte */
                cnt = 1;
                normLen = i / (int32_t)8;
                pass[normLen] = 0;
              }
              if (((int32_t)checkFlag & len) != (int32_t)0)
              {
                pass[normLen] |= cnt; 
              }
            }
            /* save new group language *****/
            // xx eeprom_array_set(LANG_CONFIG0_EADD, (uint8_t *)pass, 4);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.LangConfig, (uint8_t *)pass, 4);       /* ex LANG_CONFIG0_EADD */
            /*** now find power management enable flag */

            while (*pSocket != '<')
            {
              pSocket++;
            }
            pSocket++; // point to PM enable / visible  flags 
            cnt = 0;
            while (pSocket[cnt] != '>')
            {
              cnt++;
            }
            pSocket[cnt] = '\0'; // put end string 
            /* set PMAX value: to have KW *10 this value, in W, must be dived by 100     */
            i = (int32_t)atoi (pSocket);

            eeprom_param_get(HIDDEN_MENU_ENB_EADD, (uint8_t *)&pass[0], 1);
#ifdef HIDDEN_MENU_SEM_ENB_RW
            pass[0] &= (~(HIDDEN_MENU_PMNG_ENB | HIDDEN_MENU_SEM_ENB));
            if (((char)(i) & (char)HIDDEN_MENU_PMNG_ENB) != (char)0)  /* bit 0 for incoming byte is PM flag */
            {
              pass[0] |= HIDDEN_MENU_PMNG_ENB; 
            }
            if (((char)(i) & (char)HTML_MENU_PMNG_SEM) != (char)0)  /* bit 6 for incoming byte is PM Remote by SEM */
            {
              pass[0] |= HIDDEN_MENU_SEM_ENB; 
            }
#else
            pass[0] &= (~(HIDDEN_MENU_PMNG_ENB));
            if (((char)(i) & (char)HIDDEN_MENU_PMNG_ENB) != (char)0)  /* bit 0 for incoming byte is PM flag */
            {
              pass[0] |= HIDDEN_MENU_PMNG_ENB; 
            }
#endif
            /* save PM enable flag *****/
            // xx eeprom_array_set(HIDDEN_MENU_ENB_EADD, (uint8_t*)&pass[0], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.Hidden_Menu.Enabled, (uint8_t*)&pass[0], 1);    /* ex HIDDEN_MENU_ENB_EADD */
            /* get PM visible flag *****/
            eeprom_param_get(HIDDEN_MENU_VIS_EADD, (uint8_t *)&pass[1], 1);
            pass[1] &= (~HIDDEN_MENU_PMNG_VIS);
            if (((char)(i) & (char)0x02) != (char)0)  /* bit 1, incoming byte is Visible flag  */
            {
              pass[1] |= (char)HIDDEN_MENU_PMNG_VIS;
            }
            /* save PM visible flag *****/
            // xx eeprom_array_set(HIDDEN_MENU_VIS_EADD, (uint8_t*)&pass[1], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.Hidden_Menu.Visible, (uint8_t*)&pass[1], 1);           /* ex HIDDEN_MENU_VIS_EADD */
            /* get current power management mode */
            eeprom_param_get(PMNG_MODE_EADD, (uint8_t *)&pass[0], 1);
            pass[0] &=(~(char)PMNG_MODE_MASK);

            i = (((char)(i >> (int32_t)4)) & (char)PMNG_MODE_MASK);
            pass[0] |= (char)(i);
            /* save received power management mode */
            // xx eeprom_array_set(PMNG_MODE_EADD, (uint8_t *)&pass[0], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Mode, (uint8_t *)&pass[0], 1);           /* ex PMNG_MODE_EADD */
            
            /* Update modbus register */
            setPmMode(pass[0]);

            /* get info for parameter visibulity                         */
            eeprom_param_get(HIDDEN_MENU_VIS_EADD, (uint8_t *)&pass[1], 1);
            pass[1] &= (~HIDDEN_MENU_PMNG_VIS);

            /*** now find PMAX value */
            while (*pEm != '<')
            {
              pEm++;
            }
            pEm++; // point to numeric PMAX string start
            cnt = 0;
            while (pEm[cnt] != '>')
            {
              cnt++;
            }
            pEm[cnt] = '\0'; // put end string 
            /* set PMAX value: to have KW *10 this value, in W, must be dived by 100     */
            i = (int32_t)atoi (pEm);
            i /= (int32_t)100;
            pass[0] = (char)(i);
            pass[1] = (char)( i / (int32_t)0x100);
            /* save PMAX value as KW * 10 unit measure  *****/
            // xx eeprom_array_set(PMNG_PWRLSB_EADD, (uint8_t*)&pass[0], 1);
            // xx eeprom_array_set(PMNG_PWRMSB_EADD, (uint8_t*)&pass[1], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Power, (uint8_t*)&i, 2);    /* ex PMNG_PWRLSB_EADD */

            /*** now find Imin value */
            while (*pImin != '<')
            {
              pImin++;
            }
            pImin++; // point to numeric Imin string start
            cnt = 0;
            while (pImin[cnt] != '>')
            {
              cnt++;
            }
            pImin[cnt] = '\0'; // put end string 
            /* set Imin value in [A * 10]     */
            i = (int32_t)atoi (pImin);
            pass[0] = (char)i;
            /* save Imin value   *****/
            // xx eeprom_array_set(PMNG_CURRENT_EADD, (uint8_t*)&pass[0], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Current, (uint8_t*)&pass[0], 1);    /* ex PMNG_CURRENT_EADD */
            
            /*** now find HPOWER value */
            while (*pHpwr != '<')
            {
              pHpwr++;
            }
            pHpwr++; // point to numeric Imin string start
            cnt = 0;
            while (pHpwr[cnt] != '>')
            {
              cnt++;
            }
            pHpwr[cnt] = '\0'; // put end string 
            /* set HPOWER value in the range 1..10     */
            i = (int32_t)atoi (pHpwr);
            pass[0] = (char)(i - 1);
            /* save HPOWER value   *****/
            // xx eeprom_array_set(PMNG_MULTIP_EADD, (uint8_t*)&pass[0], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Multip, (uint8_t*)&pass[0], 1);    /* ex PMNG_MULTIP_EADD */
            
            /*** now find DSET value */
            while (*pDset != '<')
            {
              pDset++;
            }
            pDset++; // point to numeric PMAX string start
            cnt = 0;
            while (pDset[cnt] != '>')
            {
              cnt++;
            }
            pDset[cnt] = '\0'; // put end string 
            /* set DSET value: to 0..2 store as * 10     */
            i = (int32_t)atoi (pDset);
            pass[0] = (char)( i / (int32_t)100);
            /* save DSET value as KW * 10 unit measure  *****/
            // xx eeprom_array_set(PMNG_ERROR_EADD, (uint8_t*)&pass[0], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Error, (uint8_t*)&pass[0], 1);     /* ex PMNG_ERROR_EADD */
            
            /*** now find DMAX value */
            while (*pDmax != '<')
            {
              pDmax++;
            }
            pDmax++; // point to numeric PMAX string start
            cnt = 0;
            while (pDmax[cnt] != '>')
            {
              cnt++;
            }
            pDmax[cnt] = '\0'; // put end string 
            /* set DMAX value in the range 1..100 %     */
            i = (int32_t)atoi(pDmax);
            pass[0] = (char)i;
            /* save DMAX value as percentile  *****/
            // xx eeprom_array_set(PMNG_DMAX_EADD, (uint8_t*)&pass[0], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Dmax, (uint8_t*)&pass[0], 1);            /* ex PMNG_DMAX_EADD */
            
            /*** now find unbalanced enable flag */
            while (*pUnb != '<')
            {
              pUnb++;
            }
            pUnb++; // point to Unbalanced enable flag 
            pass[0] = (char)0;
            if (*pUnb == (char)'1')
            {
              pass[0] = (char)1;
            }
            /* save Unbalance enable flag *****/
            // xx eeprom_array_set(PMNG_UNBAL_EADD, (uint8_t*)&pass[0], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Unbal, (uint8_t*)&pass[0], 1);         /* ex PMNG_UNBAL_EADD */
            
            /* Update modbus map */
            setPowerManagementRegisters (); 

            /*** now find EMEX enable flag */
            while (*pEmx != '<')
            {
              pEmx++;
            }
            pEmx++; // point to EMEX enable flag 
            cnt = 0;
            while (pEmx[cnt] != '>')
            {
              cnt++;
            }
            pEmx[cnt] = '\0'; // put end string 
            /*get the flag byte value  for EMEX alarm and SINAPSI     */
            i = (int32_t)atoi(pEmx);
            
            /* Update modbus register value */
            setPmEmexInModbus((uint8_t)i);

            eeprom_param_get(CONTROL_BYTE2_EADD, (uint8_t*)&pass[0], 1);
            eeprom_param_get(HIDDEN_MENU_ENB_EADD, (uint8_t *)&pass[3], 1);
            
            pass[1] = (char)i;

//            pass[0] &= (char)(~EMETER_EXT_CRL2);        // reset EMEX alarm status flag
            pass[0] &= (char)(~(EMETER_EXT_CRL2 | SINAPSI_CHN2_CRL2));        // reset EMEX alarm status flag
            pass[3] &= (char)(~HIDDEN_MENU_SINAPSI);    // reset pmng enable and SINAPSI flag 

            if (pass[1] & (char)EMETER_EXT_CRL2)
            {
//              pass[0] |= (char)EMETER_EXT_CRL2;         // set l'abilitazione allarme EM esterno
              pass[0] |= (char)(EMETER_EXT_CRL2 | SINAPSI_CHN2_CRL2);         // set l'abilitazione allarme EM esterno
            }
            if (pass[1] & (char)HIDDEN_MENU_SINAPSI)
            {
              pass[3] |= (char)HIDDEN_MENU_SINAPSI;      // set  SINAPSI
            } 

            /* save EMEX enable flag *****/
            // xx eeprom_array_set(CONTROL_BYTE2_EADD, (uint8_t*)&pass[0], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.controlByte.Byte.Byte2, (uint8_t*)&pass[0], 1);    /* ex CONTROL_BYTE2_EADD */
            /* save SINAPSI enable flag *****/
            // xx eeprom_array_set(HIDDEN_MENU_ENB_EADD, (uint8_t*)&pass[3], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.Hidden_Menu.Enabled, (uint8_t*)&pass[3], 1);    /* ex HIDDEN_MENU_ENB_EADD */

            /* save SINAPSI error counter  *****/
            if (pass[1] & SIN_RES_ERR_CRL2 )
            {
              /* for debug: clear sinapsi error */
              resetStationSinapsiRS485Error();
            }

            /*** now find fasce orarie enable flag */
            while (*pRfas != '<')
            {
              pRfas++;
            }
            pRfas++; // point to Unbalanced enable flag 
            pass[0] = (char)0;
            if (*pRfas == (char)'1')
            {
              pass[0] = (char)1;
            }
            /* save fasce orarie enable flag *****/
            // xx eeprom_array_set(PMNG_TRANGE_EADD, (uint8_t*)&pass[0], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Trange, (uint8_t*)&pass[0], 1);            /* ex PMNG_TRANGE_EADD */

            /*** now find charge in time enable flag */
            while (*pAbCT != '<')
            {
              pAbCT++;
            }
            pAbCT++; // point to charge in time enable flag 
            pass[0] = (char)0;
            if (((*pAbCT) & (char)0x01) != (char)0)  /* bit 0, incoming byte is charge time enable flag */
            {
              pass[0] = (char)1;
            }
            /* save charge in time enable flag *****/
            // xx eeprom_array_set(TCHARGE_MODE_EADD, (uint8_t*)&pass[0], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.TCharge, (uint8_t*)&pass[0], 1);    /* ex TCHARGE_MODE_EADD */

            /* get info for parameter charge time visibility                         */
            eeprom_param_get(HIDDEN_MENU_VIS_EADD, (uint8_t *)&pass[1], 1);
            pass[1] &= (~HIDDEN_MENU_TMEG_VIS);

            if (((*pAbCT) & (char)0x02) != (char)0) /* bit 1, incoming byte is Visible flag */
            {
              pass[1] |= (char)HIDDEN_MENU_TMEG_VIS;
            }
            /* save PM visible flag *****/
            // xx eeprom_array_set(HIDDEN_MENU_VIS_EADD, (uint8_t*)&pass[1], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.Hidden_Menu.Visible, (uint8_t*)&pass[1], 1);    /* ex HIDDEN_MENU_VIS_EADD */
            
            /* Update modbus MENU_VISIBILITY_RW 0x0040 register */
            setPmMenuVisibility((uint8_t)pass[1]);
            setChargeTimeVisibility((uint8_t)pass[1]);

            /*** now find charge in time value */
            while (*pMaxTric != '<')
            {
              pMaxTric++;
            }
            pMaxTric++; // point to numeric chrge in time string start
            cnt = 0;
            while (pMaxTric[cnt] != '>')
            {
              cnt++;
            }
            pMaxTric[cnt] = '\0'; // put end string 
            /* set charge in time value: to have 30 min step this value, in min, must be dived by 30     */
            i = (int32_t)atoi (pMaxTric);
            pass[0] = (char)( i / (int32_t)30);
            /* save charge in time value as 30 min step unit measure  *****/
            // xx eeprom_array_set(TCHARGE_TIME_EADD, (uint8_t*)&pass[0], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.TCharge, (uint8_t*)&pass[0], 1);    /* ex TCHARGE_TIME_EADD */
            
            /*** now find charge energy max value */
            while (*pMaxEnergy != '<')
            {
              pMaxEnergy++;
            }
            pMaxEnergy++; // point to numeric energy string start
            cnt = 0;
            while (pMaxEnergy[cnt] != '>')
            {
              cnt++;
            }
            pMaxEnergy[cnt] = '\0'; // put end string 
            /* set energy value in KWh Range 0...100     */
            i = (int32_t)atoi (pMaxEnergy);
            if (i > (int32_t)100)
            {
              i = (int32_t)0;
            }
            pass[0] = (char)(i);
            /* save energy value in KWh unit  *****/
            // xx eeprom_array_set(ENRG_LIMIT_EADD, (uint8_t*)&pass[0], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.Energy_limit, (uint8_t*)&pass[0], 1);    /* ex ENRG_LIMIT_EADD */
            
            /*** now find time zone value */
            while (*pGMT != '<')
            {
              pGMT++;
            }
            pGMT++; // point to numeric chrge in time string start
            cnt = 0;
            while (pGMT[cnt] != '>')
            {
              cnt++;
            }
            pGMT[cnt] = '\0'; // put end string 
            /* set time zone value     */
            i = (int32_t)atoi (pGMT);
            pass[0] = (char)(i);
            /* save time zone   *****/
            // xx eeprom_array_set(TIME_ZONE_EADD, (uint8_t*)&pass[0], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.Time_Settings.TimeZone, (uint8_t*)&pass[0], 1);    /* ex TIME_ZONE_EADD */
            
            /*** now find daylight enable flag  value */
            while (*pDST != '<')
            {
              pDST++;
            }
            pDST++; // point to numeric chrge in time string start
            cnt = 0;
            while (pDST[cnt] != '>')
            {
              cnt++;
            }
            pDST[cnt] = '\0'; // put end string 
            /* set time zone value     */
            i = (int32_t)atoi (pDST);
            pass[0] = (char)(i);
            /* save DST flag   *****/
            // xx eeprom_array_set(DST_EADD, (uint8_t*)&pass[0], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.Time_Settings.dst, (uint8_t*)&pass[0], 1);    /* ex DST_EADD */
            
            /*** now find set date and time Flag  */
            while (*pDeT != '<')
            {
              pDeT++;
            }
            pDeT++; // point to numeric chrge in time string start
            cnt = 0;
            while (pDeT[cnt] != '>')
            {
              cnt++;
            }
            pDeT[cnt] = '\0'; // put end string 
            /* set unix time value     */
            i = (int32_t)atoi (pDeT);
            if (i != 0)
            {
              /* Save Date and Time informations received, into BKP SRAM region */
              /* Save also the checksum */
              BKP_SRAM_UnixTimestamp_Save(i);    /* Ticket SCU-100 */
              
              setDateTimeFromUnixT(i);
              // Aggiorno data e ora nella struttura globale 
              UpdateGlobalDT();
              cnt = (char)1;
              // xx eeprom_array_set(RTC_VALID_EADD, (uint8_t*)&cnt, 1);
              SCU_InfoStation_Set ((uint8_t *)&infoStation.rtcValid, (uint8_t*)&cnt, 1);      /* ex RTC_VALID_EADD */
            }

            /*** now find operative mode flag  */
            while (*pModeOp != '<')
            {
              pModeOp++;
            }
            pModeOp++; // point to numeric chrge in time string start
            cnt = 0;
            while (pModeOp[cnt] != '>')
            {
              cnt++;
            }
            pModeOp[cnt] = '\0'; // put end string 
            /* set operative mode  value     */
            i = (int32_t)atoi (pModeOp);
            pass[0] = (char)(i);
            /* set operative mode       */
            if (pass[0] <= EVS_NET_MODE)  /* EVS_FREE_MODE = 0, EVS_PERS_MODE = 1, EVS_NET_MODE = 2 NON gestiti: EVS_OCPP_MODE = 3*/
            {
              /* set the current operative mode                       */
              // xx eeprom_array_set(EVS_MODE_EADD, (uint8_t*)&pass[0], 1);
              SCU_InfoStation_Set ((uint8_t *)&infoStation.evs_mode, (uint8_t*)&pass[0], 1);          /* ex EVS_MODE_EADD */
              send_to_evs(EVS_AUTORIZATION_MODE);
            }

            /*** now find LCD type  */
            while (*pLcdType != '<') 
            {
              pLcdType++;
            }
            pLcdType++; // point to numeric chrge in time string start
            cnt = 0;
            while (pLcdType[cnt] != '>')
            {
              cnt++;
            }
            pLcdType[cnt] = '\0'; // put end string 
            /* set operative mode  value     */
            i = (int32_t)atoi (pLcdType);
            pass[0] = (char)(i);
            /* get lcd type  from LCD_TYPE_EADD   */
            eeprom_param_get(LCD_TYPE_EADD, (uint8_t *)&pass[1], 1);
            pass[3] = (pass[1] & (char)((char)SBC_WIFI_MASK | (char)WIFI_MASK));   // save current WiFi&SBC status 

            pass[1] &= (~((uint8_t)LCD_TYPE_MASK | (uint8_t)WIFI_MASK));
            /* set operative mode       */
            if (pass[0] >= LCD_TYPE_NUM)  /* LCD_TYPE_NULL = 0, LCD_2X20 = 1 */
            {
              pass[0] = LCD_TYPE_NULL;
            }
            
            pass[1] &=~ LCD_TYPE_MASK;
            pass[1] |= pass[0];


            /*** now find WiFi Operative mode  */
            while (*pWiFiMode != '<') 
            {
              pWiFiMode++;
            }
            pWiFiMode++; // point to numeric chrge in time string start
            cnt = 0;
            while (pWiFiMode[cnt] != '>')
            {
              cnt++;
            }
            pWiFiMode[cnt] = '\0'; // put end string 
            /* set wifi mode  value     */
            i = (int32_t)atoi (pWiFiMode);
            pass[0] = ((char)(i) & (char)WIFI_MASK);
            /* set WiFi mode       */
            if ((pass[0] != WIFI_OFF) && (pass[0] != WIFI_ON))  /* WiFi OFF = 0, WiFi ON = 4 */
            {
              pass[0] = WIFI_OFF;
            }
            pass[1] &=~ WIFI_MASK;
            pass[1] |= pass[0];

            /* now set working mode between wifi and SBC */
            pass[0] = ((char)(i) & (char)SBC_WIFI_MASK);
            if ( pass[0] != 0)
            {
              /* set WiFi&SBC  and WiFi mode       */
              pass[1] |= (char)(((char)SBC_WIFI_MASK | (char)WIFI_MASK));
            }
            else
            {
              /* set WiFi&SBC  and WiFi mode       */
              pass[1] &= (char)(~(SBC_WIFI_MASK));
            }
            pass[2] = (pass[1] & (uint8_t)((char)SBC_WIFI_MASK | (char)WIFI_MASK));   // recovery the current WiFi and WiFi&SBC status 

            if (pass[3] != pass[2])
            {
              /* the WiFi&SBC mode has been changed: a restart it is necessary */
              activeImmediateReset();
            }

            /*** now find DiRi Operative mode  */
            while (*pDiRiMode != '<') 
            {
              pDiRiMode++;
            }
            pDiRiMode++; // point to numeric Diff. Riarm status
            cnt = 0;
            while (pDiRiMode[cnt] != '>')
            {
              cnt++;
            }
            pDiRiMode[cnt] = '\0'; // put end string 
            /* set wifi mode  value     */
            i = (int32_t)atoi (pDiRiMode);
            pass[0] = (char)(i);
            /* set WiFi mode       */
            if ((pass[0] != DIRI_OFF) && (pass[0] != DIRI_ON))  /* DiffRiarm OFF = 0, DiffRiarm ON = 8 */
            {
              pass[0] = DIRI_OFF;
            }

            if ((i ^ (pass[1] & DIRI_MASK)) != 0)
            {
              /* the diffRiarm mode has been changed: a restart it is necessary */
              activeImmediateReset();
            }

            pass[1] &=~ DIRI_MASK;
            pass[1] |= pass[0];
            

            /* set the current LCD type and WiFi Mode                        */
            // xx eeprom_array_set(LCD_TYPE_EADD, (uint8_t*)&pass[1], 1);
            SCU_InfoStation_Set ((uint8_t *)&infoStation.LcdType, (uint8_t*)&pass[1], 1);    /* ex LCD_TYPE_EADD */
            
            /* update status */
            send_to_evs(EVS_EXTERNAL_EM_UPDATE);

            /*** now find if that it is a new default product parameters  */
            while (*pWDef != '<') 
            {
              pWDef++;
            }
            pWDef++; // point to numeric Diff. Riarm status
            cnt = 0;
            while (pWDef[cnt] != '>')
            {
              cnt++;
            }
            pWDef[cnt] = '\0'; // put end string 
            /* set wifi mode  value     */
            i = (int32_t)atoi (pWDef);
            pass[0] = (char)(i);
            /* set new default if true       */
            if (pass[0] != 0)  
            {
              eeprom_default_set();
            }

#ifdef CHANGE_ID
            /*** now check if some socket has been removed   */
            if (getScuOpMode() == SCU_M_P)
            {
              while (*pPresSk != '<') 
              {
                pPresSk++;
              }
              pPresSk++; // point to numeric Diff. Riarm status
              cnt = 0;
              while (pPresSk[cnt] != '>')
              {
                cnt++;
              }
              pPresSk[cnt] = '\0'; // put end string 
              /* set wifi mode  vue     */ 
              i = (int32_t)atoi (pPresSk);
              setSocketDiscovered((uint16_t)i);
            }   
#endif
            /*** now find the channel id number   */
            while (*pChId != '<') 
            {
              pChId++;
            }
            pChId++; // point to numeric channel WiFi Id
            cnt = 0;
            while (pChId[cnt] != '>')
            {
              cnt++;
            }

            pChId[cnt] = '\0'; // put end string 
            /* set wifi channel number      */
            i = (int32_t)atoi (pChId);
            pass[0] = (char)(i);
            if(((uint8_t)i >= (uint8_t)WIFI_AP_MIN_CHANNEL_ID) && ((uint8_t)i <= (uint8_t)WIFI_AP_MAX_CHANNEL_ID))
            {
              AppEmobTask_setWifiChannel( ( uint8_t )i );
            }

                       
            /* upgrade modbus map */
            upgradeModbusHwConfig();

            /* Send event to update eeprom */
            // xx send_to_eeprom(EEPROM_UPDATE); 

            /* end action on hwConf parameters  */
            /* Load mainPage/index page */
            /* send uploaddone.html page */
            fs_open(&heapFile, "/uploaddone.html");
            writePcb(hs, &heapFile, p, pcb);
          }
        }
        else if ((((strncmp(data, "POST /upgWifi.html", 18) == 0)) && (htmlpage == infoSpecFunc)) ||
                 (((strncmp(data, "GET /upgWifi.html", 17) == 0)) && ((htmlpage == wifiUpgrade) || (htmlpage == modelConfFunc))))
        {
          if ((htmlpage == infoSpecFunc) || (htmlpage == modelConfFunc))
          {
            size = 0;
            pWiFiUploading = malloc(heapFile.len); 
            htmlpage = wifiUpgrade;
            /* request to start expressif upgrade module */
            sendMsgToStartUpgradeModule();
          }
          
          fs_open(&heapFile, "/upgWifi.html");


          if (pWiFiUploading != NULL)
          {
            /*       destination       source     len   */
            memcpy(pWiFiUploading, heapFile.data, heapFile.len);
            pWiFi[0] = strstr(pWiFiUploading, (char*)"@xx");
            if ((size >= 200) || (DataFlag != (uint32_t)0xFF))
            {
              if (DataFlag == 2)
              {
                size = 0;
                len = sprintf((char*)&pass[0], "%3d sec </span><span style=""color:red ""> ERROR!!!", size);
                activeImmediateReset();
              }
              else
              {
                len = sprintf((char*)&pass[0], "%3d sec </span><span style=""color:blue""> Restarting......", size);
                activeImmediateReset();
              }
              if (DataFlag != (uint32_t)0xFF)
              {
                activeImmediateReset();
              }
            }
            else
            {
              len = sprintf((char*)&pass[0], "%3d sec", size);
            }
            /*    destination      source     len   */
            memcpy(pWiFi[0], (char*)&pass[0], len);
            size += 5;  // refresh page, waiting end upgrade, is every 5 sec 
            heapFile.data = (const char*)pWiFiUploading;
          }

          writePcb(hs, &heapFile, p, pcb);
        }
        else if ((strncmp(data, "GET /hwCheckX", 13) == 0) && (htmlpage == infoHwConfRx))
        {
          /* end action on hwConf parameters  */
          /* Load mainPage/index page */
          fs_open(&heapFile, "/mainPage.html");
          writePcb(hs, &heapFile, p, pcb);
          return ERR_OK;
        }

        else if (((strncmp(data, "POST /upload.cgi", 16) == 0) || (DataFlag >= 1)) && (htmlpage == FileUploadPage) ||
                 ((strncmp(data, "POST /UpdateFW", 14) == 0) || (DataFlag >= 1)))
        {
          DataOffset = 0;

          /* POST Packet received */
          if (DataFlag == 0)
          {
            BrowserFlag = 0;
            TotalReceived = 0;

            /* parse packet for Content-length field */
            size = Parse_Content_Length(data, (uint32_t)(p->tot_len));

            /* some task must be stopped to avoid problem during uploading new firmware */
            send_to_contact(CONTACT_CONTROL_STOP);  // per evitare MIRROR ERROR e conseguente scatto bobina di sgancio

            suspendEthPollingCheck(ETH_EVENT_SUSPEND);   // stop check eth registers
            stopTimerAdcConv();         // stop timer for ADC conversion (timer and DMA)
            stopCompletePolling();      // stop polling on input signals
            deInitSBCUsart();           // stop USART5 comunication to/from SBC
            deInitRS485Usart();         // stop UART1 comunication to/from SCU and SINAPSI 
            sendMonMngMsg(MON_EV_HTTP, MON_START);            // start task monitor HTTP download 

            tPrintf( "Size %u bytes len=%u\r\n", size, len);
            nPageHits = 0;
            setUpgradeLcd(UPG_LCD_DWNL);
            putsxyDwnl_c(1, 1, 0);
            putsxyDwnl_c(1, 2, 1);
            ledColorInUplaod();
            osDelay(100);

            /* parse packet for the octet-stream field */
            for (i = 0; i < len; i++)
            {
              if (strncmp ((char*)(data + i), octet_stream, 13) == 0)
              {
                DataOffset = i + 16;
                osDelay(500);
                tPrintf( "Offset %u bytes len=%u\r\n", DataOffset, len);
                break;
              }
            }

            if ((size > MIN_FILE_DWNL_SIZE) && (size < FILE_DOWNLOAD_SIZE))
            {
              /* SCU hasn't SDRAM but we use the pointer as it was present: pointer to reserved area in SDRAM for incoming data file */
              pFile = (char*)NEW_FW_SDRAM_ADDRESS;
            }
            else
            {
              pFile = NULL;
            }

            if (pFile == NULL)
            {
              /* no area available, in this case reload upload page */
              htmlpage = FileUploadPage;
              fs_open(&heapFile, "/upload.html");
              writePcb(hs, &heapFile, p, pcb);
              DataFlag = 0;
              return ERR_OK;
            }

            pFileWrite = pFile;

            /* case of MSIE8 : we do not receive data in the POST packet */
            if (DataOffset == 0)
            {
              DataFlag++;
              BrowserFlag = 1;
              freeAllpbuf(p);
              return ERR_OK;
            }
            /* case of Mozilla Firefox : we receive data in the POST packet*/
            else
            {
              TotalReceived = len - (ContentLengthOffset + 4);
            }
          }

          if (((DataFlag == 1) && (BrowserFlag == 1)) || ((DataFlag == 0) && (BrowserFlag == 0)))
          {
            if ((DataFlag == 0) && (BrowserFlag == 0))
            {
              DataFlag++;
            }
            else if ((DataFlag == 1) && (BrowserFlag == 1))
            {
              /* parse packet for the octet-stream field */
              for (i = 0; i < len; i++)
              {
                if (strncmp ((char*)(data + i), octet_stream, 13) == 0)
                {
                  DataOffset = i + 16;
                  tPrintf( "Offset2 %u bytes len=%u\r\n", DataOffset, len);
                  osDelay(500);
                  break;
                }
              }

              /* parse packet for the filename field Example: filename="ScameSCU_V110.bin"*/
              cnt = strlen(filenameEq);

              for (i = 0; i < len; i++)
              {
                if (strncmp ((char*)(data + i), filenameEq, cnt) == 0)
                {
                  /* FW file name found */
                  i += cnt; // first FW file name char offset pointer
                  cnt = (char)0;

                  while (data[i + cnt] != '"')
                  {
                    httpFileFwName[cnt] = (uint8_t)data[i + cnt];
                    cnt++;
                  }

                  httpFileFwName[cnt] = '\0'; // end string
                  tPrintf( "FileName=%s\r\n", httpFileFwName);
                  osDelay(500);
                  break;
                }
              }

              TotalReceived += len;
              DataFlag++;
            }

            TotalData = 0;
          }
          /* DataFlag >1 => the packet is data only  */
          else
          {
            TotalReceived += len;
          }

          ptr = (char*)(data + DataOffset);
          len -= DataOffset;

          /* update Total data received counter */
          TotalData += len;

          /* check if last data packet */
          if (TotalReceived >= size)
          {
            /* if last packet need to remove the http boundary tag */
            /* parse packet for "\r\n--" starting from end of data */
            i = 4;

            while (strncmp ((char*)(data + p->tot_len - i), http_crnl_2 , 4) && (p->tot_len - i > 0))
            {
              i++;
            }

            /* http boundary tag NOT FOUND: search into the last data received */
            if (i == 0)
            {
              tPrintf( "Http tag not found in last packet!\r\n");
            }
            
            len -= i;
            TotalData -= i;

            /* write data in Flash */
            //if (len)
            {
              normLen = len;

              /* non è possibile scrivere oltre il limite del size previsto per la flash */
              if ((uint32_t)pFileWrite < (NEW_FW_SDRAM_ADDRESS + APP_AND_BOOT_MAX_SIZE))
              {
                if ((((uint32_t)pFileWrite) + (uint32_t)len) >= (NEW_FW_SDRAM_ADDRESS + APP_AND_BOOT_MAX_SIZE))
                {
                  normLen = (int32_t)(NEW_FW_SDRAM_ADDRESS + APP_AND_BOOT_MAX_SIZE) - (int32_t)pFileWrite;
                }

#ifdef USE_SDRAM
                /*       destination       source     normLen  */
                memcpy((void*)pFileWrite, (void*)ptr, normLen);
#else
                /* SCU without SDAM: use FAT on QSPI flash Write on FAT every HTTP_BUF_SIZE = 16K received bytes */
                pMax = pIn + (uint16_t)normLen;

                if (pMax < HTTP_BUF_SIZE)
                {
                  if (normLen != 0)
                  {
                    /*       destination             source     buffer size   */
                    memcpy((void*)&httpBuffer[pIn], (void*)ptr, normLen);
                  }
                  if (pMax != 0)
                  {
                    /* save last chunk of buffer */
                    result = FlashWrite(sFlashAddress, (uint8_t*)httpBuffer, pMax, pMax);
                    sFlashAddress += pMax;
                  }
                }
                else
                {
                  /*       destination             source     buffer size   */
                  memcpy((void*)&httpBuffer[pIn], (void*)ptr, (HTTP_BUF_SIZE - pIn));
                  result = FlashWrite(sFlashAddress, (uint8_t*)httpBuffer, HTTP_BUF_SIZE, HTTP_BUF_SIZE);
                  sFlashAddress += HTTP_BUF_SIZE;                  

                  if ((pMax - HTTP_BUF_SIZE) != (uint16_t)0)
                  {
                    /*       destination             source     buffer size   */
                    memcpy((void*)&httpBuffer[0], (void*)((uint32_t)ptr + (uint32_t)(HTTP_BUF_SIZE - pIn)), (pMax - HTTP_BUF_SIZE));
                    /* save last chunk of buffer */
                    result = FlashWrite(sFlashAddress, (uint8_t*)httpBuffer, (pMax - HTTP_BUF_SIZE), (pMax - HTTP_BUF_SIZE));
                    sFlashAddress +=  (pMax - HTTP_BUF_SIZE);
                  }

                }

#endif
                // Incremento contatore byte scritti
                fwUpgrade.fwPointer += (uint32_t)normLen;
              }

              pFileWrite = (char*)((uint32_t)pFileWrite + (uint32_t)len);
              //IAP_HTTP_writedata(ptr,len);
            }

            DataFlag = 0;
            i = 0;
            pFile = NULL;

            /* send uploaddone.html page */
            fs_open(&heapFile, "/uploaddone.html");
            writePcb(hs, &heapFile, p, pcb);
            sendMonMngMsg(MON_EV_HTTP, MON_STOP);            // stop task monitor HTTP download 
            if (result == HAL_OK)
            {
              char check = CheckNewFW();
              if ((check == (char)0) || (downgradeStatus == 0xAB))
              {
                /* Tell TCP that we wish be to informed of data that has been
                 successfully sent by a call to the http_sent() function. */
                htmlpage = UploadDonePage;
                /*new FW read back from FAT, is OK */
                tPrintf( "HTTP end: %06d / %06d\r\n", TotalReceived, size);
                /*****  new FW file has been dowloaded *********/
                osDelay(100);
                // Alzo il bit di avvio aggiornamento
                Set_Bit(fwUpgrade.operation, OP_UPDATE_WEB_START_BIT);
                /*****  new FW file has been dowloaded *********/
                setFlagOnRtcBck(BACKUP_DATA_QSPI_REG, BACKUP_DATA_QSPI_AVAILABLE);
              }
              else
              {
                if(check == 14)
                {
                  tPrintf( "Read back FW: verification of signature KO!\r\n");
                }
                else if(check == 15)
                {
                  tPrintf( "Read back FW: downgrade is not allowed!\r\n");
                }
                else
                {
                  tPrintf( "Read back FW: checsum KO!\r\n");
                }
                /* bug fix: when a wrong file is downloaded the system stalled, now a restart is generated */
                activeImmediateReset();
              }
              osDelay(200);
            }

          }
          /* not last data packet */
          else
          {
            /* write data in the file data area  */
            if (len)
            {
              normLen = len;

              /* non è possibile scrivere oltre il limite del size previsto per la flash */
              if ((uint32_t)pFileWrite < (NEW_FW_SDRAM_ADDRESS + APP_AND_BOOT_MAX_SIZE))
              {
                if ((((uint32_t)pFileWrite) + (uint32_t)len) >= (NEW_FW_SDRAM_ADDRESS + APP_AND_BOOT_MAX_SIZE))
                {
                  normLen = (int32_t)(NEW_FW_SDRAM_ADDRESS + APP_AND_BOOT_MAX_SIZE) - (int32_t)pFileWrite;
                }

#ifdef USE_SDRAM
                /*       destination        source     normLen  */
                memcpy((void*)pFileWrite, (void*)ptr, normLen);
#else
                /* SCU without SDAM: use FAT on QSPI flash */
                pMax = pIn + (uint16_t)normLen;

                if (pMax < HTTP_BUF_SIZE)
                {
                  /*       destination             source     buffer size   */
                  memcpy((void*)&httpBuffer[pIn], (void*)ptr, normLen);

                  pIn = pMax;

                  sendMonMngMsg(MON_EV_HTTP, MON_START);      // restart task monitor HTTP download 
                }
                else
                {
                  /*       destination             source     buffer size   */
                  memcpy((void*)&httpBuffer[pIn], (void*)ptr, (HTTP_BUF_SIZE - pIn));
                  result = FlashWrite(sFlashAddress, (uint8_t*)httpBuffer, HTTP_BUF_SIZE, HTTP_BUF_SIZE);
                  sFlashAddress += HTTP_BUF_SIZE;
                  if (result != HAL_OK)
                  {
                    tPrintf( "Ext flash write error\r\n" );
                  }
                  else
                  {
                    nPageHits++;
                    if (nPageHits & (uint32_t)0x0001)
                    {
                      putsxyDwnl_c(1, 2, (uint8_t)(nPageHits / 2));
                      tPrintf( "Rx bytes: %06d / %06d pMax=%d pIn=%d\r\n", TotalReceived, size, pMax, pIn);
                      osDelay(50);
                    }
                  }

                  if ((pMax - HTTP_BUF_SIZE) != (uint16_t)0)
                  {
                    /*       destination             source     buffer size   */
                    memcpy((void*)&httpBuffer[0], (void*)((uint32_t)ptr  + (uint32_t)(HTTP_BUF_SIZE - pIn)), (pMax - HTTP_BUF_SIZE));
                  }
                  pIn = (uint16_t)(pMax - HTTP_BUF_SIZE);
                }

#endif
                fwUpgrade.fwPointer += (uint32_t)normLen;
              }

              pFileWrite = (char*)((uint32_t)pFileWrite + (uint32_t)len);
              //IAP_HTTP_writedata(ptr,len);
            }
          }

          /* check if there is a next beffer in the queue */
          p = getNextBuf(p);
        }
        else if ((strncmp(data, "POST /uploadEnd.cgi", 19) == 0) &&
                 ((htmlpage == UploadDonePage) || (htmlpage == advanced) || (htmlpage == mainPage) || (htmlpage == commandFailPage) || (htmlpage == parameters)) )
        {
          htmlpage = mainPage;
          fs_open(&heapFile, "/mainPage.html");
          /* modify the tag inside html page  */
          writePcb(hs, &heapFile, p, pcb);
        }
        else if ((strncmp(data, "POST /reboot.html", 17) == 0))
        {
          HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), BACKUP_CHARGE_STATUS, 0L);
          while(HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)getHandleRtc(), BACKUP_CHARGE_STATUS) != 0x00000000);
          /** restart the system  */
          setFlagForNvic();
          NVIC_SystemReset();
        }
        else if ((strncmp(data, "POST /EEclear.html", 18) == 0))
        {
          resetEEpromAll();
          /** restart the system by NVIC reset */
          activeImmediateReset();
          /* send uploaddone.html page */
          fs_open(&heapFile, "/uploaddone.html");
          writePcb(hs, &heapFile, p, pcb);
        }
        else if ((strncmp(data, "POST /EEdef.html", 16) == 0))
        {
          restoreFactoryDefault();
          /** restart the system by NVIC reset */
          activeImmediateReset();
          /* send uploaddone.html page */
          fs_open(&heapFile, "/uploaddone.html");
          writePcb(hs, &heapFile, p, pcb);
        }
        else if ((strncmp(data, "POST /allDef.html", 17) == 0))
        {
          restoreFactoryDefaultForAll();
          /* send uploaddone.html page */
          fs_open(&heapFile, "/uploaddone.html");
          writePcb(hs, &heapFile, p, pcb);
        }
        else if ((strncmp(data, "POST /stpChg.cgi", 16) == 0))
        {
          if (evs_state_get() == EVSTATE_CHARGING)
          {
             send_to_evs(EVS_PULS_STOP);
          }

          htmlpage = mainPage;
          DataFlag = 0;
          fs_open(&heapFile, "/mainPage.html");

          if (pTag1 != NULL)
          {
            heapFile.data = (const char*)pHtmlData;
            //sprintf(pTag1, "00:11:BF:%02X:%02X:%02X", TempMAC[3], TempMAC[4], TempMAC[5]);
          }

          writePcb(hs, &heapFile, p, pcb);
        }
        else
        {
          /* Bad HTTP requests   goHomePOSTrx  */
          close_conn(pcb, hs);
          freeAllpbuf(p);
        }
      }
      else
      {
        /* Bad HTTP requests   goHomePOSTrx */
        freeAllpbuf(p);
        /* sul tasto back spesso finiva qui: se eseguo close connection la trasmissione della pagina html in corso */
        /* si blocca ed solo parzialmente visualizzata. Quindi ho commentato la close. Nick 30-05-2018             */
        //close_conn(pcb,hs);
      }
    }  // end while on  dataAvailable
  }
  else
  {
    if (err == ERR_OK && p == NULL)
    {
      /* received empty frame */
      close_conn(pcb, hs);
    }
  }

  return ERR_OK;
}

/**
*
* @brief        free all buffer linked in a message
*
* @param [in]   struct pbuf *: pointer to queue buffer
*
* @retval       none
*
***********************************************************************************************************************/
static void freeAllpbuf(struct pbuf *p)
{

  if (p != NULL)
  {
    pbuf_free(p);
  }

  dataAvailable = (uint8_t)FALSE;
}

/**
*
* @brief        get pointer a new buffer if exist
*
* @param [in]   struct pbuf *: pointer to queue buffer list
*
* @retval       struct pbuf *: pointer to new buffer
*
***********************************************************************************************************************/
static struct pbuf * getNextBuf(struct pbuf *p)
{
  struct pbuf *pNext;

  pNext = NULL;

  if (p != NULL)
  {
    if (p->next != NULL)
    {
      pNext = p->next;
    }
    else
    {
      pbuf_free(pBufHead);
      dataAvailable = (uint8_t)FALSE;
    }
  }
  else
  {
    dataAvailable = (uint8_t)FALSE;
  }

  return (pNext);
}


/**
  * @brief  callback function on TCP connection setup ( on port 80)
  * @param  arg: pointer to an argument structure to be passed to callback function
  * @param  pcb: pointer to a tcp_pcb structure
  * &param  err: Lwip stack error code
  * @retval err
  */
static err_t http_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
  struct http_state *hs;

  /* Allocate memory for the structure that holds the state of the connection */
  hs = mem_malloc(sizeof(struct http_state));

  if (hs == NULL)
  {
    return ERR_MEM;
  }

  /* Initialize the structure. */
  hs->file = NULL;
  hs->left = 0;

  /* Tell TCP that this is the structure we wish to be passed for our
     callbacks. */
  tcp_arg(pcb, hs);

  /* Tell TCP that we wish to be informed of incoming data by a call
     to the http_recv() function. */
  tcp_recv(pcb, http_recv);

  tcp_err(pcb, conn_err);

  tcp_poll(pcb, http_poll, 10);
  return ERR_OK;
}


/**
  * @brief  intialize HTTP webserver
  * @param  none
  * @retval None
  */
void K22_httpd_init(void)
{
  struct tcp_pcb *pcb;

  DataFlag = 0;
  size = 0;
  TotalReceived = 0;
  ContentLengthOffset = 0;
  BrowserFlag = 0;
  TotalData = 0;
#ifdef BUFFER_UP_RAM
  /* not used in SCAME application. The html file are stored in flash */
  heapFile.data = (char*)bufferFileWeb;
#endif
  heapFile.len = 0;
  pTag1 = pCbox1[0] = pLang[0] = NULL;
  pHtmlData = pHwConfData = pSpecFuncData = pSiConfData = pProdConfData = NULL;
  numLoginError = 0;

  downgradeStatus = 0;

  /*create new pcb*/
  pcb = tcp_new();
  /* start listening on port 8063   */
  tcp_bind(pcb, IP_ADDR_ANY, (uint16_t)EthernetTCP2Port);
  /* start listening on port 8063 */
  pcb = tcp_listen(pcb);
  /* define callback function for TCP connection setup */
  tcp_accept(pcb, http_accept);
  /* here we create the task to manager download FW from GSY */
  scuGsyDwldHandle = osThreadNew(scuGsyDwldTask, NULL, &scuGsyDwldTask_attributes); 
    /* here we create the task to manager upgrade FW on WiFi Expressif Module*/
  wifiUpgExpressifHandle = osThreadNew(wifiUpgExpressifTask, NULL, &wifiUpgExpressifTask_attributes); 

}

char getFlashFileInfo(FlashInfoFile * const fileInfo, infoFwBoot_st * const fwInfo)
{
  uint32_t sector;
  uint8_t *ptr;
  char err;
  
  for(sector = 0; sector < 512; sector++)
  {
    err = FlashRead(sector * BLOCK_SIZE, httpBuffer, BLOCK_SIZE);
    httpBuffer[BLOCK_SIZE] = 0;
    if(err == 0)
    {
      ptr = (uint8_t*)strstr((char*)httpBuffer, BOOT_CHECK_STR);
      if(ptr != NULL)
      {
        fileInfo->boot = 1U;
        if(sector > 0)
          fileInfo->start = ((sector - 1) * BLOCK_SIZE);
        else
          fileInfo->start = 0;
        err = FlashRead(fileInfo->start + SECTOR_BOOT_SIZE + SECURE_AREA_SIZE + 0x1000, (uint8_t*)fwInfo, sizeof(infoFwBoot_st));
        if(err == 0)
        {
          fileInfo->size = SECTOR_BOOT_SIZE + SECURE_AREA_SIZE + fwInfo->fwLen + SIGNATURE_AREA_SIZE;
        }
        return err;
      }
      ptr = (uint8_t*)strstr((char*)httpBuffer, FW_CHECK_STR);
      if(ptr != NULL)
      {
        fileInfo->boot = 0U;
        if(sector > 0)
          fileInfo->start = ((sector - 1) * BLOCK_SIZE);
        else
          fileInfo->start = 0;
        err = FlashRead(fileInfo->start + 0x1000, (uint8_t*)fwInfo, sizeof(infoFwBoot_st));
        if(err == 0)
        {
          fileInfo->size = fwInfo->fwLen + SIGNATURE_AREA_SIZE;
        }
        return err;
      }
    }
    else
    {
      return 1;
    }
  }
  
  return 1;
}

/**
*
* @brief       check if a new FW is present and tries to download it in the up Flash
*
* @param [in]  none  
*  
* @retval      char: 0 for no error  
*  
****************************************************************/
static char CheckNewFW(void)
{
  FlashInfoFile     flashFileInfo;
  infoFwBoot_st     infoFw;
  
  uint32_t          totalLen, cnt, len, cksum, pAddr;
  uint8_t           result;
  
  result = getFlashFileInfo(&flashFileInfo, &infoFw);
  if(result != 0)
  {
    return((char)12); /* error reading file */
  }
  
  result = CheckSignature(&flashFileInfo);
  if(result != 0)
  {
    return 14; /* error verification file */
  }
  
  sFlashAddress = EXT_SFLASH_NEW_FW_ADDR_START;
  if(flashFileInfo.boot)
  {
    /* il file caricato  è quello col boot */
    /* read firk 8K=0x2000 code (BUFF_FW_LEN)  */
    if (FlashRead(sFlashAddress, (void*)httpBuffer, BUFF_FW_LEN) != HAL_OK)
    {
      /* new FW corrupted */
      return((char)12); /* error reading file */
    }
    checkAndProgBoot();
    sFlashAddress += SECTOR_BOOT_SIZE + SECURE_AREA_SIZE;
  }
  
  /* Version control to prevent application downgrades */
  if(compareVersions((char*)FW_VERSION, (char*)infoFw.fwVersion) < 1)
  {
    /* Downgrade error */
    return((char)15);
  }
    
  /* read firk 8K=0x2000 code (BUFF_FW_LEN)  */
  if (FlashRead(sFlashAddress, (void*)httpBuffer, BUFF_FW_LEN) != HAL_OK)
  {
    /* new FW corrupted */
    return((char)12); /* error reading file */
  }
  
  /* check in advace on FW coerence */
  cksum = totalLen = pAddr = (uint32_t)0;
  len = BUFF_FW_LEN;
  do
  {
    
    refreshWD();
    
    for (cnt = 0; cnt < len; cnt++, pAddr++)
    {
      /* calcolo della checksum saltando i 4 byte di quella del fw corrente */
      if ((pAddr < INFO_OFFSET_ADDRESS) || (pAddr >= INFO_OFFSET_TAG_ADDRESS))
      {
        cksum += httpBuffer[cnt];
      }
    } 
    totalLen += cnt;
            
    /* read next 8K=0x2000 code (BUFF_FW_LEN)  */
    result = FlashRead (sFlashAddress + totalLen, (void*)httpBuffer, BUFF_FW_LEN);
    
    /* This check fixes the issue related to the FW update failed for checksum KO */
    /* This happened because the checksum was calculated reading over the file    */
    if ((totalLen + len) > infoFw.fwLen)
      len = infoFw.fwLen - totalLen;
    
} while ((totalLen < infoFw.fwLen) && (result == HAL_OK) && (len != 0));
  if (cksum == infoFw.fwCheksum)
  {
    return((char)0);
  }
  /*new FW read back from FAT is KO */
  tPrintf( " TotalLen: %06d\r\n fwLen: %06d\r\n", totalLen, infoFw.fwLen);
  osDelay(50);
  tPrintf( " cksFound: %06d\r\n cksFw: %06d\r\n", cksum, infoFw.fwCheksum);

  return((char)1); /* checksum */
}

/**
*
* @brief        Check if data store in external flash is valid.     
*
* @param [in]   FlashInfoFile info: informations of data store in external flash.
*
* @retval       0 valid
                1 not valid
*
***********************************************************************************************************************/
static char CheckSignature(FlashInfoFile * const info)
{
  char err;
  cmox_hash_retval_t hash_ret;
  cmox_ecc_retval_t retval;
  cmox_hash_handle_t *hash_ctx;
  uint32_t index;
  uint32_t fault_check = CMOX_ECC_AUTH_FAIL;
  uint8_t *pubkey;
  uint32_t pubkey_size;
  
  /* Construct a hash context that is configured to perform SHA256 digest operations */
  hash_ctx = cmox_sha256_construct(&sha256_ctx);
  if (hash_ctx == NULL)
  {
    return 1;
  }
  
  /* Get public key in BOOT */
  pubkey = (uint8_t*)(0x08007000);
  pubkey_size = CMOX_ECC_SECP256R1_PUBKEY_LEN;
  
  /* Initialize the hash context */
  hash_ret = cmox_hash_init(hash_ctx);
  if (hash_ret != CMOX_HASH_SUCCESS)
  {
    return 1;
  }
  
  /* Compute hash */
  /* Append the message to be hashed by chunks of readBytes Bytes */
  for (index = 0; index < (info->size - SIGNATURE_AREA_SIZE); index += 0x800)
  {
    err = FlashRead(info->start + index, httpBuffer, 0x800);
    if(err == 0)
    {
      hash_ret = cmox_hash_append(hash_ctx, httpBuffer, 0x800); /* Chunk of data to digest */
      if(hash_ret != CMOX_HASH_SUCCESS)
      {
        return 1;
      }
    }
    else
    {
      return 1;
    }
  }
  
  /* Generate the digest data */
  hash_ret = cmox_hash_generateTag(hash_ctx, computed_hash, NULL);
  if(hash_ret != CMOX_HASH_SUCCESS)
  {
    return 1;
  }

  err = FlashRead(info->start + index, signature, SIGNATURE_SIZE);
  if(err != 0)
  {
    return 1;
  }
  
  /* Construct a ECC context, specifying mathematics implementation and working buffer for later processing */
  cmox_ecc_construct(&Ecc_Ctx, CMOX_ECC256_MATH_FUNCS, httpBuffer, HTTP_BUF_SIZE);
  
  /* Verify signature */
  retval = cmox_ecdsa_verify(&Ecc_Ctx,                                  /* ECC context */
                             CMOX_ECC_CURVE_SECP256R1,                  /* SECP256R1 ECC curve selected */
                             pubkey, pubkey_size,                       /* Public key for verification */
                             computed_hash, CMOX_SHA256_SIZE,           /* Digest to verify */
                             signature, SIGNATURE_SIZE,                 /* Data buffer to receive signature */
                             &fault_check);                             /* Fault check variable: to ensure no fault injection occurs during this API call */
  
  /* Verify API returned value */
  if (retval != CMOX_ECC_AUTH_SUCCESS)
  {
    return 1;
  }
  
  /* Verify Fault check variable value */
  if (fault_check != CMOX_ECC_AUTH_SUCCESS)
  {
    return 1;
  }
  
  /* Cleanup context */
  cmox_ecc_cleanup(&Ecc_Ctx);
  cmox_hash_cleanup(hash_ctx);
  
  /* No more need of cryptographic services, finalize cryptographic library */
  cmox_finalize(NULL);
  
  return 0;
}

/**
*
* @brief       check if a new boot is present and tries to 
*              download it in the up Flash
*
* @param [in]  none  
*  
* @retval      none  
*  
****************************************************************/
static void checkAndProgBoot(void)
{
            uint32_t          totalLen, cnt, len, cksum, pAddr, pAddrF;
            uint8_t           bootInfo[16];
            uint8_t           *pBootVer;
            uint32_t          *pCkSum;
  volatile  uint16_t          FlashProtection;
            uint8_t           bootError, err, BlockSize, result;
            
  /* ricerca del boot tag string   */
  pBootVer = (uint8_t*)(BOOT_ADDRESS + BOOT_VERSION_OFFSET);

  /***      destination            source                                length      */
  memcpy((void*)bootInfo, (void*)&httpBuffer[BOOT_VERSION_OFFSET], sizeof(bootInfo));

#ifndef GD32F4xx 
  if ((strstr((char*)bootInfo, (char*)BOOT_CHECK_STR) != NULL) || 
      ((httpBuffer[BOOT_VERSION_OFFSET] == 'V') && (httpBuffer[BOOT_VERSION_OFFSET + 1] == '1') && (httpBuffer[BOOT_VERSION_OFFSET + 2] == '.')))
  {
    /* il file caricato contiene il boot */
    if (memcmp((void*)&httpBuffer[BOOT_VERSION_OFFSET], (void*)pBootVer, (size_t)4) == 0)
    {
      /* current version boot and new boot are the same: no update need */
      return;
    }
  }
#else
  if(strstr((char*)bootInfo, (char*)BOOT_CHECK_STR) != NULL)
  {
    bootInfo[4] = '\0';
    /* Version control to prevent bootloader downgrades */
    if(compareVersions((char*)pBootVer, (char*)bootInfo) < 1)
    {
      return;
    }
  }
#endif
  else
  {
    return;
  }
  /* if we arrive here in  httpBuffer[] there is the first boot packet to upload  */
  sFlashAddress = EXT_SFLASH_NEW_FW_ADDR_START;
  cksum = totalLen = pAddr = (uint32_t)0;
  len = BUFF_FW_LEN; // SCAME_BOOT_SIZE;
  do
  {
    for (cnt = 0; cnt < len; cnt++, pAddr++)
    {
       /* calcolo della checksum saltando i 4 byte di quella del fw corrente */
      if ((pAddr < BOOT_VERSION_OFFSET) || (pAddr >= (uint32_t)(BOOT_VERSION_OFFSET + BOOT_INFO_SIZE)))
      {
        cksum += httpBuffer[cnt];
      }
      if ((pAddr != 0) && ((pAddr % 0x1000) == 0))
      {
        cksum &= 0x00FFFFFF;
      }
    } 
    totalLen += cnt;
    /* read next 8K=0x2000 code (BUFF_FW_LEN)  */
    result = FlashRead (sFlashAddress + totalLen, (void*)httpBuffer, BUFF_FW_LEN);
  } while ((totalLen < SCAME_BOOT_SIZE) && (result == HAL_OK) && (len != 0));
  
  pCkSum = (uint32_t*)((uint32_t)bootInfo + (uint32_t)BOOT_CKS_OFFSET);
  if (cksum == *pCkSum)
  {
    /* disable sysTick so FreeRTOS cannot switch task  */
    setSysTickStatus(TRUE);
    /* disable all not necessary interrupts */
#ifdef GD32F4xx 	  
      for (int i = (int)WWDG_IRQn; i <= DMA2D_IRQn; i++)
      {
        HAL_NVIC_DisableIRQ((IRQn_Type)i);
      }
#else
      for (int i = (int)WWDG_IRQn; i <= LPTIM6_IRQn; i++)
      {
        HAL_NVIC_DisableIRQ((IRQn_Type)i);
      }
#endif	  

    bootError = (uint8_t)TRUE;
    do
    {
      FlashProtection = FLASH_If_GetWriteProtectionStatus();
      if(FlashProtection == FLASHIF_PROTECTION_NONE)
      {
        // Cancello l'area di flash interna (boot sector 32K escluso 0x08000000 - 0x08007FFF) 
        if (FLASH_If_Erase(BOOT_ADDRESS) == (uint16_t)0)
        {
          /* erase Flash done */
          bootError = (uint8_t)FALSE;
        }
      }
    } while (bootError == (uint8_t)TRUE);

    do
    {
      /* the received boot is OK: rewind the file to read it again and write to micro flash */
      sFlashAddress = EXT_SFLASH_NEW_FW_ADDR_START;
      bootError = (uint8_t)TRUE;
      err = 0; 
      totalLen = pAddr = cksum = (uint32_t)0;
      pAddrF = BOOT_ADDRESS;

      /** erase andata a buon fine  */
      /* read firk 8K=0x2000 code (BUFF_FW_LEN)  */
      if (FlashRead (sFlashAddress, (void*)httpBuffer, BUFF_FW_LEN) == HAL_OK)  
      {
        do
        {
          for (cnt = 0; cnt < len; cnt++, pAddr++)
          {
            /* calcolo della checksum saltando i 16 byte di info */
            if ((pAddr < BOOT_VERSION_OFFSET) || (pAddr >= (uint32_t)(BOOT_VERSION_OFFSET + BOOT_INFO_SIZE)))
            {
              cksum += httpBuffer[cnt];
            }
          } 

#ifdef GD32F4xx
          BlockSize = 4;   // 4 Bytes (32bit) in case of GD32F4xx 
#else
          BlockSize = 16;  // 16 Bytes (128bit) in case of STM32H5xx
#endif          
          /** flash erase OK: start programming  */
          /* programming 8K code block always */
          if (FLASH_If_Write(pAddrF, BOOT_END_ADDRESS, (uint32_t*)httpBuffer, (BUFF_FW_LEN / BlockSize)) == FLASHIF_OK)
          {
            if (memcmp((void*)pAddrF, (void*)httpBuffer, (size_t)BUFF_FW_LEN) != 0)
            {
              /* la copia in Flash non è andata a buon fine */
              err = 6;
            }
            else
            {
              pAddrF += (uint32_t)BUFF_FW_LEN;
            }
          }
          else
          {
            /* la scrittura in Flash non è andata a buon fine  */
            err = 6;
          }
          totalLen += cnt;
          /* read next 8K=0x2000 code (BUFF_FW_LEN)  */
          result = FlashRead (sFlashAddress + totalLen,(void*)httpBuffer, BUFF_FW_LEN); 
        } while ((totalLen < SCAME_BOOT_SIZE) && (result == HAL_OK) && (len != 0) && (err == 0));
        
        if (cksum == *pCkSum)
        {
          /* the new boot has been correctly programmed in the micro flash */
          bootError = (uint8_t)FALSE;
        }
      }
    } while (bootError == (uint8_t)TRUE);
    /* enable sysTick */
    setSysTickStatus(FALSE);
  }
}

static int8_t compareVersions(char *current, char *next)
{
  uint32_t n, m;
  
  n = strlen(current);
  m = strlen(next);
  if(m > n)
    n = m;
  return (int8_t)strncmp(next, current, n);
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
xQueueHandle getScuGsyDwldQueueHandle(void)
{
   return(scuGsyDwldQueue);
}

static void scuGsyDwldTask (void * pvParameters)
{
  uint32_t          timeTick, i;
  frameSbcRx_st*    pMsgRx;
  frameFwCode_st*   pFrameFwCode;
  areaConfPar_st*   pConfPar; 
  blockConfPar_st*  pTempBlockConfPar;
  uint16_t          codeError, cnt;
  uint8_t           sendAnsw;
  scuOpModes_e      scuMode;
  scuTypeModes_e    scuTypeModes;
  uint8_t           result;  
#ifdef NO_RESET
  scuOpModes_e    prevScuMode;
  scuTypeModes_e  prevScuTypeMode;
#endif
  /*-------- Creates an empty mailbox for uart  messages --------------------------*/
  scuGsyDwldQueue = xQueueCreate(NUM_BUFF_GSY_DWNL_RX, sizeof(frameSbcRx_st));
  configASSERT(scuGsyDwldQueue != NULL);
  

  //timeTick = pdMS_TO_TICKS(TIMER_TICK_500); 
  timeTick = portMAX_DELAY;
  pMsgRx = pMsgFrameSbcRx();

  codeInfo.codeLen = (uint32_t)0;
  codeInfo.numBk = (uint16_t)0;
  codeInfo.state = FW_STATE_IDLE;

  setFastBridge(DISABLED);
  setBroadcastDownload(FALSE);
  scuMode = getScuOpMode();
  scuTypeModes = getScuTypeMode();

  
  for (;;)
  {
    /* Wait for some event from Rx/Tx uart SBC (typically UART5)  */
    if (xQueueReceive(scuGsyDwldQueue, (void *)pMsgRx, timeTick) == pdPASS)
    {
      /* set default timeout  */
      timeTick = portMAX_DELAY;

      sendAnsw = 0;
      codeError = CODE_ERROR;
      scuMode = getScuOpMode();

      if ((pMsgRx->messageEv == UART_RX_START_DWLD) || (pMsgRx->messageEv == UART_RX_START_BRD_DWLD))
      {
        switch (codeInfo.state)
        {
          case FW_STATE_IDLE:
                        
            pIn = (uint16_t)0;
            
            sFlashAddress = (uint32_t)EXT_SFLASH_NEW_FW_ADDR_START;          
              
            send_to_contact(CONTACT_CONTROL_STOP);  // per evitare MIRROR ERROR e conseguente scatto bobina di sgancio

            /* some task must be stopped to avoid problem during uploading new firmware */
            suspendEthPollingCheck(ETH_EVENT_SUSPEND);   // stop check eth registers
#ifdef SUSPEND
            /*Configure PWR_DWN1L at  active value --> "0"   */
            HAL_GPIO_WritePin(PWRDWN1L_GPIO_Port, PWRDWN1L_Pin, GPIO_PIN_RESET);  // power down chip ETH, 
#endif
            stopTimerAdcConv();         // stop timer for ADC conversion (timer and DMA)
            stopCompletePolling();      // stop polling on input signals
            if (scuMode == SCU_EMUMAX0_M)
            {
              deInitRS485Usart();         // stop UART1 comunication to/from SCU and SINAPSI 
            }
            if (pMsgRx->messageEv == UART_RX_START_BRD_DWLD)
            {
              if (isSemMasterFz() == TRUE)
              {
                /* in a broadcast download the master must:
                   1) send a broadcast message for put all the slave in download mode
                   2) bridge the FW packet on RS485 for the slave
                   */
                upgradeFwSlaveBroadcast(MODBUS_BROADCAST_ADDR);
              }
            }
            else
            {
              if (scuMode == SCU_M_STAND_ALONE)
              {
                upgradeFwSlaveBroadcast(codeInfo.scuInDwldIdx);
              }
            }

            /* we try to create the new FW file */
            /*         destination       source */
            strcpy((char *)httpFileFwName, (char *)dwldDefFileName);
            {
              nPageHits = 0;
              setUpgradeLcd(UPG_LCD_DWNL);
              putsxyDwnl_c(1, 1, 0);
              putsxyDwnl_c(1, 2, 1);
              ledColorInUplaod();

              /** set the download flag at default value   ***/
              setFlagOnRtcBck(BACKUP_DATA_QSPI_REG, BACKUP_DATA_QSPI_DEF_VAL);

              /** FW code must by a multiple of PACKET_GSY_FW_LEN i.e 1024  ***/
              codeInfo.numBk = codeInfo.currBuffIx = (uint16_t)0;
              codeError = CODE_OK_DWNL;
              codeInfo.currLen = (uint32_t)0;
              /* init always the update data structure */
              fwUpgrade.fwPointer = (uint32_t)EXT_SFLASH_NEW_FW_ADDR_START;              
#ifdef DWL_TIMEOUT
              /* within 12sec a packet code must be received */
              timeTick = pdMS_TO_TICKS(TIMER_START_CODE); 
#endif
            }
#ifdef COME_ERA
            if (((scuMode == SCU_EMUMAX0_S) || (scuMode != SCU_M_P)) && (getBroadcastDownload() == TRUE))
#else
            if (((scuMode == SCU_EMUMAX0_S) || (isSemMasterFz() == FALSE)) && (getBroadcastDownload() == TRUE))
#endif
            {
              /* in broadcast mode no answer is possible for any slave but the master must respond to SBC */
              if (scuMode == SCU_EMUMAX0_M) 
              {
                sendAnsw = 1; 
              }
              else
              {
                sendAnsw = 0; 
                codeInfo.state = FW_STATE_RX;
              }
              if (isSemMode() == FALSE)  // in SEM the baude rate on RS485 is 230400 already
              {
                /* we are in broadcast FW download and the bridge on RS485 at 230400bps must be done when SCU is master */
                tPrintf("Broadcast SLAVE RS485 BR=");
                lowLevelBrChange(UART_SCU, UART_SBC_DWLD_BR, PROT_UART_SCU);    /* Fixed ticket SCU-73 */
              }
              if ((scuTypeModes == SCU_SEM_M) || (scuTypeModes == SCU_SEM_STAND_ALONE))
              {
                /* this change in the master must be done when the task is in FW_RX_STATE. That is checked inside lowLevelBrChange() routine */
                tPrintf("Cambio baud rate\n\r");
                osDelay(100);
                lowLevelBrChange(UART_SBC, UART_SBC_DWLD_BR, PROT_UART_SBC);
              }
            }
            else
            {
              sendAnsw = 1;
            }
            break;

          default:
            break;
        }
      }

      if (pMsgRx->messageEv == UART_RX_DWLD)
      {
        pFrameFwCode = (frameFwCode_st*)pMsgRx->messageRx.msgComplete;
        switch (codeInfo.state)
        {
          case FW_STATE_RX:
            /* non è possibile scrivere oltre il limite del size previsto per la flash */
            if (pMsgRx->totalLen == BUFFER_FW_PAYLOAD_CKS)
            {
              if ((checksumOnBlock(pMsgRx->messageRx.msgComplete, BUFFER_FW_PAYLOAD) == (uint16_t)0) && (pFrameFwCode->numPacket == codeInfo.numBk))
              {
                codeError = codeInfo.numBk;
                pMax = codeInfo.currBuffIx + PACKET_GSY_FW_LEN;
                codeInfo.currLen += PACKET_GSY_FW_LEN;

                if ((getBroadcastDownload() == TRUE) && ((scuMode == SCU_EMUMAX0_M) || (isSemMasterFz() == TRUE))) 
                {
                  /* the packet code must be sent on RS485 also */
                  txOnRs485Bus(pMsgRx->messageRx.msgComplete, (uint16_t)BUFFER_FW_PAYLOAD_CKS);
                  osDelay(50);
                }

                if ((pMax < HTTP_BUF_SIZE) && (codeInfo.currLen < codeInfo.codeLen))
                {
                  /*       destination             source     buffer size   */
                  memcpy((void*)&httpBuffer[codeInfo.currBuffIx], (void*)pFrameFwCode->dataFw, PACKET_GSY_FW_LEN);
                  codeInfo.currBuffIx += PACKET_GSY_FW_LEN;
                }
                else
                {
                  /*       destination                             source                                buffer size   */
                  memcpy((void*)&httpBuffer[codeInfo.currBuffIx], (void*)pFrameFwCode->dataFw, PACKET_GSY_FW_LEN);
                  result = FlashWrite(sFlashAddress, (uint8_t*)httpBuffer, pMax, pMax);
                  sFlashAddress += pMax;
                  codeInfo.currBuffIx = (uint16_t)0;
                  if (result != HAL_OK)
                  {
                    codeError = CODE_STOP_DWNL;
                  }
                  osDelay(500);
                }
                // Incremento contatore byte scritti
                fwUpgrade.fwPointer += (uint32_t)PACKET_GSY_FW_LEN;
                codeInfo.numBk++; // index for next block
              }
              else
              {
                ledColorPacketErrorUplaod();
                if (pFrameFwCode->numPacket == (codeInfo.numBk - 1))
                {
                  // precente ACK è andato perso: lo ritrasmetto 
                  codeError = codeInfo.numBk - 1; 
                }
                else
                {
                  codeError = CODE_CKS_CODE_FAIL;
                }
              }
              if ((scuMode == SCU_EMUMAX0_S || (scuMode == SCU_S_S)) && (getBroadcastDownload() == TRUE))
              {
                /* in broadcast mode no answer is possible */
                sendAnsw = 0;
                if (((codeInfo.currLen & (uint32_t)0x00003FFF) == (uint32_t)0) && (codeInfo.currLen != 0))
                {
                  nPageHits++;
                  if ((nPageHits % 2) == 0) putsxyDwnl_c(1, 2, (uint8_t)(nPageHits / 2));
                  tPrintf("Rx bytes=%6d / %d\n\r", codeInfo.currLen, codeInfo.codeLen);
                  xTimerReset (xMdbUartTimers, 0);                  
                }
              }
              else
              {
                sendAnsw = 1;
              }
              if ((codeInfo.currLen >= codeInfo.codeLen))
              {
                /* code download ended */
                /* close  the file and release the semaphore */
                timeTick = portMAX_DELAY;
                char check = CheckNewFW();
                if ((check == (char)0)  || (downgradeStatus == (uint8_t)0xAB))
                {
                  //deInitSBCUsart();           // stop USART5 comunication to/from SBC
                  /*new FW read back from FAT, is OK */
                  tPrintf( "Rx bytes=%06d bytes / %06d received\r\n", codeInfo.currLen, codeInfo.codeLen);
                  osDelay(20);
                  // Alzo il bit di avvio aggiornamento
                  Set_Bit(fwUpgrade.operation, OP_UPDATE_WEB_START_BIT);
                  /*****  new FW file has been dowloaded *********/
                  setFlagOnRtcBck(BACKUP_DATA_QSPI_REG, BACKUP_DATA_QSPI_AVAILABLE);
                  ledColorUplaodOk();
                  if (((scuMode == SCU_S_P) || (scuMode == SCU_S_S)) && (codeInfo.scuInDwldIdx != MODBUS_BROADCAST_ADDR))
                  {
                    /* after update will be necessary that a slave must inform the master about FW version   */
                    i = HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*) getHandleRtc(), BACKUP_HW_INFO);
                    /* set the flag */
                    i |= (NOTIFY_FW_UPD_FLAG);
                    HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), (uint32_t)BACKUP_HW_INFO, i);     
                  }
                  osDelay(2000);
                  /** restart the system to program new firmware */
                  activeImmediateReset();
                }
                else
                {
                  if(check == 14)
                  {
                    tPrintf( "Read back FW: verification of signature KO!\r\n");
                  }
                  else if(check == 15)
                  {
                    tPrintf( "Read back FW: downgrade is not allowed!\r\n");
                  }
                  else
                  {
                    tPrintf( "Read back FW: checsum KO!\r\n");
                  }
                  codeError = CODE_CKS_CODE_FAIL;
                  codeInfo.state = FW_STATE_IDLE;
                  /* bug fix: when a wrong file is downloaded the system stalled, now a restart is generated */
                  activeImmediateReset();
                }
              }
              else
              {
  #ifdef DWL_TIMEOUT
                timeTick = pdMS_TO_TICKS(TICK_PACKET_TO);
  #endif
              }
            }
            else
            {
              /* close  the file and release the semaphore */
              /** restart the system  */
              setFlagForNvic();
              NVIC_SystemReset();
            }
            break;

          case FW_DWNL_OTHER_RX:
            sendAnsw = 0;
            /* a new packet receveid: isn't for us so discard it. Do nothing */
            timeTick = TO_END_DWLD;
            break;

          case FW_STATE_IDLE:
            if (pMsgRx->totalLen == sizeof(blockConfPar_st))
            {
              for (cnt = 0, i = 0; cnt < sizeof(blockConfPar_st) - sizeof(i); cnt++)
              {
                i += pMsgRx->messageRx.msgComplete[cnt];
              }
              if (((blockConfPar_st*)&pMsgRx->messageRx.msgComplete[0])->ckecksumControl == i)
              {
                if (isSemMasterFz() == TRUE)
                {
                  /* in this case we manage the eeprom config data coming from slave to be used on slave board replacement */
                  pConfPar = getPtrToConfParam();
                  if (pConfPar == NULL)
                  {
                    pConfPar = (areaConfPar_st*)malloc(sizeof(areaConfPar_st));
                    memset((void*)pConfPar, 0xFF, sizeof(areaConfPar_st));
                    setPtrToConfParam((uint8_t*)pConfPar);
                  }
                  /* copy in SRAM the EEPROM area coming from slave. Only at the end the area will be stored in SPI Flash */
                  pTempBlockConfPar = (blockConfPar_st*)&pMsgRx->messageRx.msgComplete[0];
                  cnt = pTempBlockConfPar->idLogicScu;
                  if ((cnt > SCU_M_P_ADDR_LOG) && (cnt < SCU_NUM)) // cnt 1..15 the master, logic = 0, isn't involved 
                  {
                    /*             destination                                  source                            len                 */
                    memcpy((uint8_t *)&pConfPar->blockConfPar[cnt], (uint8_t *)pTempBlockConfPar, sizeof(blockConfPar_st));     
                    /* upgrade list of slave with eeprom info received */
                    setInfoEepromSlaveStatus(cnt);                        
                  }
                  /* there are 2 cases:                                                                                                         */
                  /* 1) the master is in discovery phase: so it wait all the slaves have sent the info and then save all infos in the SPI flash (at the and of getInfoSocket()) */
                  /* 2) the master isn't in discovery phase: this is the message from a slaves to update its eeprom infos in the SPI flash                                      */
                  if (getDiscoveryStatus() == (uint32_t)0)
                  {
                    /* here we are in case 2 */
                    if (saveSlaveParameters() != 0)
                    {
                      /* error occured: reset the master */
                      activeImmediateReset();
                    }
                  }
                  tPrintf("Received EEPROM info from Board %2d\n\r", cnt + 1);
                }
                else
                {
                  pConfPar = (areaConfPar_st*)&pMsgRx->messageRx.msgComplete[0];
                  cnt = pConfPar->blockConfPar[0].idLogicScu;
                  /* check if this is a replacement SCU slave board  (jolly SCU) */
                  if (SCU_S_REPL_ADDR == getStationId())
                  {
                    /* The board serial number must be preserved so overwrite old board serial */
                    eeprom_param_get(SERNUM_BYTE0_EADD, &pConfPar->blockConfPar[0].confEepromParamArray[SERNUM_BYTE0_EADD], 4);
                    eeprom_param_get(SCU_SN_EE_ADDRES,  &pConfPar->blockConfPar[0].confSerialCode[SCU_SN_EE_ADDRES - PRD_CODE_EE_ADDRES], 4);
                    /*             destination                                                           source                       len  */
                    memcpy((uint8_t *)&pConfPar->blockConfPar[0].confInfoStation.serial[0], (uint8_t *)getStationSerialNumber(), BOARD_SN_LENGTH);     
                    /*             destination                                                           source                       len  */
                    memcpy((uint8_t *)&pConfPar->blockConfPar[0].confBackupInfoStation.serial[0], (uint8_t *)getStationSerialNumber(), BOARD_SN_LENGTH);     
                    /* now write all previous data in the current EEPROM area */    
                    codeError = (uint16_t)WriteOnEeprom(EDATA_VALID_EADD, (uint8_t*)pConfPar->blockConfPar[0].confEepromParamArray, EEPROM_PARAM_NUM);
                    if (codeError == 0)
                    {
                      codeError = (uint16_t)WriteOnEeprom(SCU_GENERAL_INFO_EE_ADDRES, (uint8_t*)&pConfPar->blockConfPar[0].confInfoStation, sizeof (infoStation_t));
                      
                    }
                    if (codeError == 0)
                    {
                      codeError = (uint16_t)WriteOnEeprom(EDATA_BKP_SCU_EE_ADDRESS, (uint8_t*)&pConfPar->blockConfPar[0].confBackupInfoStation, sizeof (infoStation_t));
                    }
                    if (codeError == 0)
                    {
                      codeError = (uint16_t)WriteOnEeprom(PRD_CODE_EE_ADDRES, (uint8_t*)&pConfPar->blockConfPar[0].confSerialCode, 
                                                           sizeof (pConfPar->blockConfPar[0].confSerialCode));
                    }
                    if (codeError == 0)
                    {
                      codeError = (uint16_t)WriteOnEeprom(EDATA_DEFAULT_ID_CODES, (uint8_t*)&pConfPar->blockConfPar[0].confSerialFactoryCode, 
                                                           sizeof (pConfPar->blockConfPar[0].confSerialFactoryCode));
                    }
                    if (codeError == 0)
                    {
                      HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), (uint32_t)BACKUP_HW_INFO, pConfPar->blockConfPar[0].confRtcBackup[2]);     
                      HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), (uint32_t)BACKUP_EM_ENRG_ACT, pConfPar->blockConfPar[0].confRtcBackup[1]);     
                      HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), (uint32_t)BACKUP_EM_MODEL, pConfPar->blockConfPar[0].confRtcBackup[0]);     
                    }
                    if (codeError == 0)
                    {
                      tPrintf("Board address = %2d cloned!!\n\r", pConfPar->blockConfPar[0].confEepromParamArray[RS485_ADD_EADD] + 1);
                      /** restart the system by NVIC reset */
                      activeImmediateReset();
                    }
                  }
                }
              }
            }
            sendAnsw = 0;
            break;

          default:
            /* here for same action when the Fw packect isn't for us */
            break;
        }
      }
      if (pMsgRx->messageEv == SLAVE_IN_DWNL)
      {
        /* the master must:                                                           */
        /* - wait end transmission ACK to download to SBC                             */
        /* - stop unessenzial task                                                    */
        /* - change baude rate to 115200 on UART1 (SBC) and UART5 (RS485)             */
        /* - stop DMA bridge and start with a bridge byte per byte UART1_RX->UART5_TX */
        /* - start a monitor procedure to detect the end of slave downloading         */
        /* some task must be stopped to avoid problem during uploading new firmware   */
#ifdef COME_ERA
        if (scuMode == SCU_M_P)
#else
        if (isSemMasterFz() == TRUE)
#endif
        {
#ifdef NO_RESET
          prevScuMode = scuMode;
          prevScuTypeMode = getScuTypeMode();
#endif
          if (scuMode == SCU_M_STAND_ALONE)
          {
            upgradeFwSlaveBroadcast(codeInfo.scuInDwldIdx);
            osDelay(100);
          }
          /* we are in SEM and this is the master: use EMUMAX0 master in downloading phase */
          setScuOpMode(SCU_EMUMAX0_M);
          setScuTypeMode(SCU_GSY);
        }

        osDelay(10);
        suspendEthPollingCheck(ETH_EVENT_SUSPEND);   // stop check eth registers
#ifdef SOSPESA
        /*Configure PWR_DWN1L at  active value --> "0"   */
        HAL_GPIO_WritePin(PWRDWN1L_GPIO_Port, PWRDWN1L_Pin, GPIO_PIN_RESET);  // power down chip ETH, 
#endif
        stopTimerAdcConv();         // stop timer for ADC conversion (timer and DMA)
        stopCompletePolling();      // stop polling on input signals
        send_to_contact(CONTACT_CONTROL_STOP);  // per evitare MIRROR ERROR e conseguente scatto bobina di sgancio nel master mentre aggiorna uno slave

        setFastBridge(ENABLED);
        lowLevelBrChange(UART_SBC, UART_SBC_DWLD_BR, PROT_UART_SBC);    /* Fixed ticket SCU-73 */
        if ((scuMode < SCU_M_P))
        {
          lowLevelBrChange(UART_SCU, UART_SBC_DWLD_BR, PROT_UART_SCU);    /* Fixed ticket SCU-73 */
        }
      }

      if (pMsgRx->messageEv == SLAVE_END_DWNL)
      {
        osDelay(5000);
        /** restart the system */
        setFlagForNvic();
        NVIC_SystemReset();
      }
      

      if (pMsgRx->messageEv == OTHER_SLAVE_IN_DWNL)
      {
        codeInfo.state = FW_DWNL_OTHER_RX;
        sendAnsw = 0;
        /* a slave detect onother slave will start with download. This slave must:   */
        /* - ignore message sent at 230400bps on RS485                               */
        /* - wait for packet at 230400                                               */
        /* - avoid any disturbe over RS485                                           */
        /* - wait for end download to re-switch to 19200                             */
#ifdef NO_DISABLE_RS485
        lowLevelBrChange(UART_SCU, UART_SBC_DWLD_BR, PROT_UART_SCU);    /* Fixed ticket SCU-73 */
        timeTick = pdMS_TO_TICKS(TO_END_DWLD); 
#else
        timeTick = pdMS_TO_TICKS(TO_END_CPLT_DWLD); 
#endif
      }

      if (sendAnsw == 1)
      {
        /* At least 1ms of delay before the response, to let the MP40596 switching in RX mode (RS485) */
        /* NOTE: a delay of 2 produce a delay variable from 1 to 2ms */
        osDelay(pdMS_TO_TICKS(2));  

        if ((scuMode == SCU_EMUMAX0_S) || (scuMode == SCU_S_S))
        {
          /* if I am a slave the answer must be sent to SBC over RS485 bus */
          sendToSbcOnRS485(codeError);
        }
        else
        {
          /* if I am a master the answer must be sent to SBC over local UART  */
          sendToSbcOnUart(codeError);
        }
        if (((codeInfo.currLen & (uint32_t)0x00003FFF) == (uint32_t)0) && (codeInfo.currLen != 0))
        {
          nPageHits++;
          if ((nPageHits % 2) == 0) putsxyDwnl_c(1, 2, (uint8_t)(nPageHits / 2));
          tPrintf("Rx bytes=%6d / %d\n\r", codeInfo.currLen, codeInfo.codeLen);
          xTimerReset (xMdbUartTimers, 0);                            
        }
        if ((codeInfo.state == FW_STATE_IDLE)  && (codeError == CODE_OK_DWNL))
        {  
                  
          stopTimerAdcConv();         // stop timer for ADC conversion (timer and DMA)
          stopCompletePolling();      // stop polling on input signals
#ifdef SOSPESA
          deleteEmTask();             /* delete energy meter task                                */               
          deleteHtsTask();            // delete task and timer for temperature management 
#endif
          codeInfo.state = FW_STATE_RX;
 
          if (getScuTypeMode() == SCU_GSY)
          {
            tPrintf("Cambio baud rate\n\r");
            osDelay(100);
            if (scuMode == SCU_EMUMAX0_S)
            {
              lowLevelBrChange(UART_SCU, UART_SBC_DWLD_BR, PROT_UART_SCU);
            }
            else
            {
              if (getBroadcastDownload() == TRUE)
              {
                /* we are in broadcast FW download and the bridge on RS485 at 230400bps must be done when SCU is master */
                tPrintf("Broadcast RS485 BR=");
                lowLevelBrChange(UART_SCU, UART_SBC_DWLD_BR, PROT_UART_SCU);
                osDelay(10);
              }
              lowLevelBrChange(UART_SBC, UART_SBC_DWLD_BR, PROT_UART_SBC);
            }
          }
          else
          {
            if ((scuTypeModes == SCU_SEM_M) || (scuTypeModes == SCU_SEM_STAND_ALONE))
            {
              tPrintf("Cambio baud rate\n\r");
              osDelay(100);
              lowLevelBrChange(UART_SBC, UART_SBC_DWLD_BR, PROT_UART_SBC);
            }
          }
        }
      }
    }
    else
    {
      if (codeInfo.state == FW_DWNL_OTHER_RX)
      {
        tPrintf("Re-init UART end DWNL\n\r");
        codeInfo.state = FW_STATE_IDLE;
        timeTick = portMAX_DELAY;
        /* restore RS485 to default baude rate --> 19200  */
        uartReintialization(); 
      }
      else
      {
        ledColorUplaodFail();
        osDelay(5000);
        /** restart the system on download timeout error */
        setFlagForNvic();
        NVIC_SystemReset();
      }
    }
  }
}

/**
*
* @brief        Only for debug 
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void resetCodeInfo(void)
{
  /** only for initial debug ** */
  codeInfo.codeLen = (uint32_t)0;
  codeInfo.numBk = (uint16_t)0;
}

/**
*
* @brief        Only for debug 
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void setCodeLen(uint32_t codeLength)
{
  codeInfo.codeLen = codeLength;
}

/**
*
* @brief        Only for debug 
*
* @param [in]   none
*
* @retval       uint8_t: TRUE if downloading is running 
*
***********************************************************************************************************************/
uint8_t getStatusDwnl(void)
{
  if ((codeInfo.state == FW_STATE_RX) || (codeInfo.state == FW_DWNL_OTHER_RX) || (getFastBridge() == ENABLED))
  {
    return(TRUE);
  }
  return(FALSE);
}

/**
*
* @brief        CRC on 32bit as simples byte summing  
*
* @param [in]   uint8_t*: pointer to first data info
* @param [in]   uint16_t: data length 
*  
* @note         the 4 byte contening checkse at the end oh input buffer 
*
* @retval       uint8_t: 0 success (checksum verified)
*
***********************************************************************************************************************/
static uint8_t checksumOnBlock(uint8_t* pBuff, uint16_t len)
{
  uint32_t*       pCksVal;
  uint16_t        cnt;
  uint32_t        currCks;
  frameFwCode_st* pFrameFwCode;
  uint8_t*        pFrameForCks;

  pFrameFwCode = (frameFwCode_st*)pBuff;
  pFrameForCks = (uint8_t*)pFrameFwCode;

  pCksVal = (uint32_t*)(&pFrameFwCode->codeCheksum);
  for (cnt = (uint16_t)0, currCks = (uint32_t)0; cnt < len; cnt++)
  {
    currCks += pFrameForCks[cnt];
  }
  if (currCks == *pCksVal)
  {
    return((uint8_t)0);
  }
  return((uint8_t)1);
}

/**
*
* @brief        change in low level mode (fast) the uart baude rate   
*
* @param [in]   USART_TypeDef *: uart involved in the change
* @param [in]   uint32_t: code for baude rate 
*  
* @note         this is valid for sysClock = 216MHz 
*
* @retval       none
*
***********************************************************************************************************************/
void lowLevelBrChange(USART_TypeDef * pUart, uint32_t brr, uint8_t intf)
{

  /* disable the UART */
  pUart->CR1 &= ~USART_CR1_UE;

  /* change BRR register */
  switch (brr)
  {
#ifndef UART_SEM_SAME_BR
    case UART_SBC_SEM_BR:
    case UART_SBC_DWLD_BR:
    case UART_SBC_SEM_DWNL_BR:
#else
    case UART_SBC_SEM_BR:
    case UART_SBC_SEM_DWNL_BR:
#endif
      if (MX_PROT_UART_ChangeBaudeRate(intf, brr) == HAL_OK)
      {
        //pUart->BRR = (uint32_t)0x01D5;
        //tPrintf("115200\n\r");
        tPrintf("BR = %d\n\r", brr);
      }
      break;

    case (uint32_t)57600:
      if (MX_PROT_UART_ChangeBaudeRate(intf, brr) == HAL_OK)
      {
        //pUart->BRR = (uint32_t)0x01D5;
        tPrintf("57600\n\r");
      }
      //pUart->BRR = (uint32_t)0x03AA;
      //if (MX_PROT_UART_ChangeBaudeRate(PROT_UART_SBC, UART_SBC_SEM_BR) == HAL_OK)
      //tPrintf("57600\n\r");
      //osDelay(100);
      break;

    case (uint32_t)19200:
      pUart->BRR = (uint32_t)0x0AFD;
      break;

    
  }
  /* enable the UART */
  pUart->CR1 |= USART_CR1_UE;
}


/**
*
* @brief        Send to scuWifiDwldQueue queue a message  
*
* @param [in]   none
*
* @retval       none  
*
***********************************************************************************************************************/
void send_to_fw_update_task(frameWifiUpl_st msgToSend)
{
  configASSERT(xQueueSendToBack(getScuWifiDwldQueueHandle(), (void *)&msgToSend, portMAX_DELAY) == pdPASS);
}


/**
*
* @brief        Get the pointer to wifi upload task queue
*
* @param [in]   none
*
* @retval       xQueueHandle: pointer to defined  queue
*
***********************************************************************************************************************/
xQueueHandle getScuWifiDwldQueueHandle(void)
{
   return(scuWifiDwldQueue);
}

int32_t FirmwareUpdate_start( char *name, uint32_t size )
{
  char fatFileName[ 32 ];
  
  if( size <= MIN_FILE_DWNL_SIZE )
    return -1;
  
  if( size >= FILE_DOWNLOAD_SIZE )
    return -1;
  
  if( name != NULL )
  {
    if( strstr( name, ".bin" ) != NULL )
    {
      strcpy( fatFileName, name );
    }
    else
    {
      return -1;
    }
  }
  else
  {
    strcpy( fatFileName, DEF_WIFI_FILE_NAME );
  }
  
  
  strcpy( (char*)httpFileFwName, fatFileName );
  
  send_to_contact(CONTACT_CONTROL_STOP);  // per evitare MIRROR ERROR e conseguente scatto bobina di sgancio
  /* some task must be stopped to avoid problem during uploading new firmware */
  suspendEthPollingCheck(ETH_EVENT_SUSPEND);    // stop check eth registers
  stopTimerAdcConv();                           // stop timer for ADC conversion (timer and DMA)
  stopCompletePolling();                        // stop polling on input signals
  deInitSBCUsart();                             // stop USART5 comunication to/from SBC
  deInitRS485Usart();                           // stop UART1 comunication to/from SCU and SINAPSI    
  deleteEmTask();                               /* delete energy meter task                                */               
  setUpgradeLcd(UPG_LCD_DWNL);                  /* Update LCD informations with "UPLOADING..." (JAPPT-216) */ 
  putsxyDwnl_c(1, 1, 0);                        /* Clear the first row of LCD (JAPPT-216) */
  putsxyDwnl_c(1, 2, 0);                        /* Clear the second row of LCD (JAPPT-216) */
  ledColorInUplaod();
  
  codeInfo.codeLen = size;
  codeInfo.currLen = 0;
  codeInfo.currBuffIx = 0;
  memset( httpBuffer, 0xFF, HTTP_BUF_SIZE );
  
  return HAL_OK;
}

int32_t FirmwareUpdate_put( uint8_t *data, uint32_t size )
{
  uint8_t result;
  uint32_t num;
  
  result = 0;
  memcpy( (void*)&httpBuffer[codeInfo.currBuffIx], (void*)data, size );
  codeInfo.currLen += size;
  codeInfo.currBuffIx = ( codeInfo.currBuffIx + size ) % HTTP_BUF_SIZE;
  if( codeInfo.currBuffIx == 0 )
  {
    result = FlashWrite(codeInfo.currLen - HTTP_BUF_SIZE, (uint8_t*)httpBuffer, HTTP_BUF_SIZE, HTTP_BUF_SIZE);    
    memset( httpBuffer, 0xFF, HTTP_BUF_SIZE );
    num = codeInfo.currLen * 100 / codeInfo.codeLen;
    tPrintf( "Rx bytes: %d / %d %d%%\r\n", codeInfo.currLen, codeInfo.codeLen, num );
  }
  num = codeInfo.currLen * 20 / codeInfo.codeLen;
  putsxyDwnl_c( 1, 2, ( uint8_t )num );
  return result;
}

int32_t FirmwareUpdate_end( void )
{
  int32_t res;
  uint8_t result;
  
  res = 0;
  if( codeInfo.currBuffIx != 0 )
  {
    result = FlashWrite(codeInfo.codeLen - codeInfo.currBuffIx, (void*)httpBuffer, HTTP_BUF_SIZE, HTTP_BUF_SIZE);
    tPrintf( "Rx bytes: %d / %d 100%%\r\n", codeInfo.codeLen, codeInfo.codeLen );
  }
  if( result == HAL_OK )
  {    
    res = (int32_t)CheckNewFW();
    if ((res == (int32_t)0)  || (downgradeStatus == (uint8_t)0xAB))
    {
      /* Save in eeprom fw update result */
      setEsitoUpdateFw(0x01);
      /* new FW read back from FAT, is OK */
      tPrintf( "Firmware update download success!\r\n" );
      /*****  new FW file has been dowloaded *********/
      osDelay(100);
      // Alzo il bit di avvio aggiornamento
      Set_Bit(fwUpgrade.operation, OP_UPDATE_WEB_START_BIT);
      /*****  new FW file has been dowloaded *********/
      setFlagOnRtcBck(BACKUP_DATA_QSPI_REG, BACKUP_DATA_QSPI_AVAILABLE);
    }
    else
    {
      if(res == 14)
      {
        tPrintf( "Read back FW: verification of signature KO!\r\n");
      }
      else if(res == 15)
      {
        tPrintf( "Read back FW: downgrade is not allowed!\r\n");
      }
      else
      {
        tPrintf( "Read back FW: checsum KO! ErrCode = %d\r\n", res);
      }
    }
  }

  return res;
} 
      
/**
*
* @brief        set the download FW type     
*
* @param [in]   uint8_t: FW dowload broadcast TRUE or FALSE
*  
* @note         none 
*
* @retval       none
*
***********************************************************************************************************************/
void  setBroadcastDownload(uint8_t status)
{
  broadcastDwnl = status;
}

/**
*
* @brief        get the download FW type     
*
* @param [in]   none 
*  
* @note         none 
*
* @retval       uint8_t: FW dowload broadcast TRUE or FALSE
*
***********************************************************************************************************************/
uint8_t  getBroadcastDownload(void)
{
  return(broadcastDwnl);
}

/**
*
* @brief        set the id slave in download      
*
* @param [in]   uint8_t: id slave 2..16
*  
* @note         none 
*
* @retval       none
*
***********************************************************************************************************************/
void  setSlavaInDownload(uint8_t status)
{
  codeInfo.scuInDwldIdx = status;
}

/**
*
* @brief        free all allocated area by malloc      
*
* @param [in]   none
*  
* @note         none 
*
* @retval       none
*
***********************************************************************************************************************/
void freeHtmlMemory(void)
{
  if (pLang[0] != NULL ) 
  {
    pLang[0] = NULL;                                    /* always reset pointer for specialFunction.html page */
    if (pSpecFuncData != NULL) {free(pSpecFuncData); pSpecFuncData = NULL;}
  }
  if (pCbox1[0] != NULL ) 
  {
    pCbox1[0] = NULL;                                    /* always reset pointer for hwConf.html page */
    if (pHwConfData != NULL) {free(pHwConfData); pHwConfData = NULL;}
  }
  if (pSiBox[0] != NULL ) 
  {
    pSiBox[0] = NULL;                                    /* always reset pointer for sinapsiInfo.html page */
    if (pSiConfData != NULL) {free(pSiConfData); pSiConfData = NULL;}
  }

  if (pProdConfData != NULL) 
  {
    free(pProdConfData);                                  /* always reset pointer for modelConf.html page */
    pProdConfData = NULL;
  }

  if (pPassConf != NULL) 
  {
    free(pPassConf);
    pSiBox[0] = NULL;
    pPassConf = NULL;
  }
}

/**
*
* @brief        Task to manage the FW upgrade on Expressif Module
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
static void wifiUpgExpressifTask (void * pvParameters)
{
  frameWifiUpl_st   msgRx;

  /*-------- Creates an empty mailbox for wifi upgrade  messages --------------------------*/
  wifiUpgExpressifQueue = xQueueCreate(NUM_BUFF_WIFI_UPG_RX, sizeof(frameWifiUpl_st));
  configASSERT(wifiUpgExpressifQueue != NULL);
  
  for (;;)
  {
    /* Wait for some event from wifi upload    */
    if (xQueueReceive(wifiUpgExpressifQueue, (void *)&msgRx, portMAX_DELAY) == pdPASS)
    {
      if (msgRx.wifiUplEv == UPL_START)
      {

        int32_t res = AppEmobTask_updateModule();
        if( res == 0 )
        {
          DataFlag = (uint32_t)0;
        }
        else if( res == 1 )
        {
          // Aggiornamento non avviato perché il modulo  è già aggiornato all'ultima versione.
          DataFlag = (uint32_t)1;
        }
        else
        {
          DataFlag = (uint32_t)2;
        }
       }
    }

  }
}

/**
*
* @brief        Send the message to start FW upgrade on Expressif Module
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
static void sendMsgToStartUpgradeModule (void)
{
  frameWifiUpl_st   msgRx;

  DataFlag = (uint32_t)0xFF;

  msgRx.wifiUplEv = UPL_START;
  /* now send it the message to start */
  configASSERT(xQueueSendToBack(wifiUpgExpressifQueue, (void *)&msgRx, portMAX_DELAY) == pdPASS);  // wifiUpgExpressifTask()
}


#ifdef MODBUS_TCP_EM_ETH
/**
*
* @brief        manager TCP connection for modbus EM      
*
* @param [in]   void *: argument info
*  
* @note         none 
*
* @retval       none
*
***********************************************************************************************************************/
static void tcp_thread(void *arg) {
	err_t                 err, accept_err, result;
  frameSbcSem_st        tmpFrameSbcSem;  

	conn = netconn_new(NETCONN_TCP);
	if (conn != NULL) 
  {
		err = netconn_bind(conn, IP_ADDR_ANY, EM_LOVATO_PORT);
		if (err == ERR_OK) 
    {
			netconn_listen(conn);
			while (1) 
      {
				accept_err = netconn_accept(conn, &newconn);
				if (accept_err == ERR_OK) {
					while (netconn_recv(newconn, &netbuf) == ERR_OK) {
						do 
            {
							copyString(mb_req_buf, netbuf->p->payload, netbuf->p->len); // get the message from the client
							uint16_t buf_len = netbuf->p->len;
              uint16_t regAddr = mb_req_buf[9] + mb_req_buf[8] * 0x100; // Address to be read 
              result = ERR_OK;
              if (regAddr == ADDR_EM_LOVATO_V_SYS)
              {
                /* only a single request can be accepted, so delete pending request */
                xQueueReset(emAnswMasterQueue);
                /* ask the master for all meters values */
                readEmValuesFromMaster();
                /* wait for answer  */
        
                /* Wait for the answer within 500ms (for EM11 typical answering time is 40ms  */
                if (xQueueReceive(emAnswMasterQueue, (void *)&tmpFrameSbcSem, TIMER_500MS) == pdPASS) 
                {
                  if (tmpFrameSbcSem.sbcSemEvent == NOTIFY_MODBUS_RD_ACK)
                  {
                    buf_len = mb_process(mb_repl_buf, mb_req_buf, buf_len, regAddr); // process the data
                  }
                }
                else
                {
                  result = ERR_IF;
                }
              }
              else
              {
                buf_len = mb_process(mb_repl_buf, mb_req_buf, buf_len, regAddr); // process the data
              }

              if (result == ERR_OK) 
              {
                netconn_write(newconn, mb_repl_buf, buf_len, NETCONN_COPY); // send the message back to the client
              }
						} while (netbuf_next(netbuf) > 0);
						netbuf_delete(netbuf);
					}
					netconn_close(newconn);
					netconn_delete(newconn);
				}
			}
		} else {
			netconn_delete(conn);
		}
	}
}

/**
*
* @brief        init manager TCP connection for modbus EM      
*
* @param [in]   none
*  
* @note         none 
*
* @retval       none
*
***********************************************************************************************************************/
void tcpserver_init(void) 
{
  emAnswMasterQueue = xQueueCreate(NUM_BUFF_EM_LOVATO_ETH, sizeof(frameSbcSem_st));
  configASSERT(emAnswMasterQueue != NULL);

	sys_thread_new("tcp_thread", tcp_thread, NULL, 8*DEFAULT_THREAD_STACKSIZE,
			osPriorityNormal);
}

/**
*
* @brief        Move payload info in the buffer       
*
* @param [in]   none
*  
* @note         none 
*
* @retval       none
*
***********************************************************************************************************************/
static void copyString(char* dest, char* src, unsigned num)
{
	for(unsigned i = 0; i != num; ++i)
  {
		dest[i] = src[i];
	}
}

static uint16_t mb_process(char *mb_repl_buf, char *mb_req_buf, uint16_t req_buf_len, uint16_t regAddr) 
{
  uint16_t len;
  scuMapRegLovatoEmSim_st  *pScuMapRegLovatoEmSim;
  uint32_t                 tmp1, tmp2, tmp3;
  uint8_t                  cnt;

  pScuMapRegLovatoEmSim = getLovatoEthRegs();

  /* mirror data */
  mb_repl_buf[0] = mb_req_buf[0]; //Transaction identifier 
  mb_repl_buf[1] = mb_req_buf[1]; // Protocol identifier HH
  mb_repl_buf[2] = mb_req_buf[2]; // Protocol identifier HL
  mb_repl_buf[3] = mb_req_buf[3]; // Protocol identifier LH
  mb_repl_buf[4] = mb_req_buf[4]; // Protocol identifier LL
  mb_repl_buf[6] = mb_req_buf[6]; // Unit identifier 
  mb_repl_buf[7] = mb_req_buf[7]; // Function code 

  if (regAddr == LOVATO_VL1_REG)
  {
    /* data answer */
    mb_repl_buf[5] = 15;       // len data message
    mb_repl_buf[8] = 12;       // payload len
    
    /* payload data */
    mb_repl_buf[9] = mb_repl_buf[13] = mb_repl_buf[17] = 0;      // invio 22204  centesimi di Vrms 
    mb_repl_buf[10] = mb_repl_buf[14] = mb_repl_buf[18] = 0;       
    mb_repl_buf[11] = mb_repl_buf[15] = mb_repl_buf[19] = (pScuMapRegLovatoEmSim->sysVoltage / 10) / 0x100 ;       
    mb_repl_buf[12] = mb_repl_buf[16] = mb_repl_buf[20] = (pScuMapRegLovatoEmSim->sysVoltage / 10) % 0x100 ;;     
    len = 21;  
  }
  else if(regAddr == LOVATO_PA1_REG)
  {
    /* data answer */
    mb_repl_buf[5] = 15;       // len data message
    mb_repl_buf[8] = 12;       // payload len
    for (cnt = 0, tmp1 = tmp2 = tmp3 = 0; cnt < SCU_NUM; cnt++)
    {
      tmp1 += pScuMapRegLovatoEmSim->socketActivePower[cnt][0];  // add all L1 active power, one value for every socket 
      tmp2 += pScuMapRegLovatoEmSim->socketActivePower[cnt][1];  // add all L2 active power, one value for every socket 
      tmp3 += pScuMapRegLovatoEmSim->socketActivePower[cnt][2];  // add all L3 active power, one value for every socket 
    }
    mb_repl_buf[9]  = (uint8_t)(tmp1 >> 24);      // HH 
    mb_repl_buf[10] = (uint8_t)(tmp1 >> 16);      // HL      
    mb_repl_buf[11] = (uint8_t)(tmp1 >> 8);       // LH       
    mb_repl_buf[12] = (uint8_t)(tmp1);            // LL     
    mb_repl_buf[13] = (uint8_t)(tmp2 >> 24);      // HH 
    mb_repl_buf[14] = (uint8_t)(tmp2 >> 16);      // HL      
    mb_repl_buf[15] = (uint8_t)(tmp2 >> 8);       // LH       
    mb_repl_buf[16] = (uint8_t)(tmp2);            // LL     
    mb_repl_buf[17] = (uint8_t)(tmp3 >> 24);      // HH 
    mb_repl_buf[18] = (uint8_t)(tmp3 >> 16);      // HL      
    mb_repl_buf[19] = (uint8_t)(tmp3 >> 8);       // LH       
    mb_repl_buf[20] = (uint8_t)(tmp3);            // LL     
    len = 21;  
  }
  return(len);
}
#endif


/**
*
* @brief        Get the product code index starting from its string
*
* @param [in]   char*: pointer to product code string
*
* @retval       none
*
***********************************************************************************************************************/
void getIdModelFromStringAndParConfig (char* pIdModel)
{
  idModel_e idModel;
  char      currEmType, currPhyScuAddr, sbcPresent;
  char*     pHwVer;

  idModel = ID_205_W36_S0;
  sbcPresent = (char)FALSE;

  currEmType = (char)EMETER_MONO_PH_ALGO2;  /* default for mono energy meter */

  switch(pIdModel[4])
  {
      /* 21 205.A/33/52-62-BB/DD      */
      /* 23 205.A/33/52-62-SS/UU      */
    case 'A':
      if ((((pIdModel[5] == '3') && (pIdModel[6] == '3')) || ((pIdModel[5] == '5') && (pIdModel[6] == '2')) || ((pIdModel[5] == '6') && (pIdModel[6] == '2')))  
               && ((pIdModel[8] == 'B') || (pIdModel[8] == 'D'))) idModel = ID_205_A33_DD;
      if (pIdModel[5] != '3') 
      {
        sbcPresent = (char)TRUE;
      }
      break;

      /* 41 205.B/33/52-62-DD/BB      */
      /* 42 205.B/33/52-62-UU/SS      */
    case 'B':
      if ((((pIdModel[5] == '3') && (pIdModel[6] == '3')) || ((pIdModel[5] == '5') && (pIdModel[6] == '2')) || ((pIdModel[5] == '6') && (pIdModel[6] == '2')))  
               && ((pIdModel[8] == 'B') || (pIdModel[8] == 'D'))) idModel = ID_205_B33_DD;
      if ((((pIdModel[5] == '3') && (pIdModel[6] == '3')) || ((pIdModel[5] == '5') && (pIdModel[6] == '2')) || ((pIdModel[5] == '6') && (pIdModel[6] == '2')))  
               && (pIdModel[8] == 'S') || (pIdModel[8] == 'U')) idModel = ID_205_B33_UU;
      if (pIdModel[5] != '3')  
      {
        sbcPresent = (char)TRUE;
      }
      break;

      /* 19 204.CA21B/21B-T2T2       */
      /* 24 204.CA21B/21B-T2xT2x     */
      /* 26 204.CA21B-UNUN           */
      /* 37 204.CA49/59/67-B/C/D     */
      /* 38 204.CA49/59/67-F/G/H     */
      /* 39 204.CA(TFT)-S/T/U        */
      /* 43 204.CA11B/13B-T2         */
      /* 44 204.CA11B/13B-T2x        */
    case 'C':
      if ((pIdModel[10] == 'T') && (pIdModel[11] == '2') && (pIdModel[12] == 'T') && (pIdModel[13] == '2'))
      {
        idModel = ID_204_CA21B_T2T2; 
        if ((strstr((char*)&pIdModel[12], "A")) || (strstr((char*)&pIdModel[12], "E"))) sbcPresent = (char)TRUE;
      } 
      else if ((pIdModel[10] == 'T') && (pIdModel[11] == '2') && (pIdModel[13] == 'T') && (pIdModel[14] == '2')) 
      {
        idModel = ID_204_CA21B_T2xT2x; 
        if ((strstr((char*)&pIdModel[12], "A")) || (strstr((char*)&pIdModel[12], "E"))) sbcPresent = (char)TRUE;
      } 
      else if ((pIdModel[10] == 'U') && (pIdModel[11] == 'N') && (pIdModel[12] == 'U'))
      {
        idModel = ID_204_CA21B_UNUN; 
        if (strstr((char*)&pIdModel[12], "A")) sbcPresent = (char)TRUE;
      } 
      else if ((pIdModel[6] == '4') || (pIdModel[6] == '5') || (pIdModel[6] == '6'))
      {
        sbcPresent = (char)TRUE;
        if ((pIdModel[9] == 'B') || (pIdModel[9] == 'C') || (pIdModel[9] == 'D')) idModel = ID_204_CA_TFT_BCD;
        else if ((pIdModel[9] == 'F') || (pIdModel[9] == 'G') || (pIdModel[9] == 'H')) idModel = ID_204_CA_TFT_FGH;
        else if ((pIdModel[9] == 'S') || (pIdModel[9] == 'T') || (pIdModel[9] == 'U')) idModel = ID_204_CA_TFT_STU;
      } 
      else if (pIdModel[6] == '1')
      {
        if ((pIdModel[10] == 'T') && (pIdModel[11] == '2') && 
            (pIdModel[12] != '1') && (pIdModel[12] != '2') && (pIdModel[12] != '3') && (pIdModel[12] != '6')) idModel = ID_204_CA11B_T2;
        else if ((pIdModel[10] == 'T') && (pIdModel[11] == '2') &&
                 ((pIdModel[12] == '1') || (pIdModel[12] == '2') || (pIdModel[12] == '3') || (pIdModel[12] == '6'))) idModel = ID_204_CA11B_T2x;
        if ((strstr((char*)&pIdModel[10], "A")) || (strstr((char*)&pIdModel[10], "E"))) sbcPresent = (char)TRUE;
      } 
      break;

      /* 40 205.N(TFT)-BCD           */
    case 'N':
      idModel = ID_205_N_TFT_BCD;
      sbcPresent = (char)TRUE;
      break;

      /* 28 205.KB30/50-B          */
    case 'K':
      idModel = ID_205_KB30_K;
      if (pIdModel[6] == '5') 
      {
        sbcPresent = (char)TRUE;
      }
      break;

      /* 27 204.UB21B-EB             */
      /* 45 204.UB11B-EB             */
    case 'U':
      if (pIdModel[6] == '1') idModel = ID_204_UB11B_EB; else idModel = ID_204_UB21B_EB;
      if (strstr((char*)&pIdModel[8], "A")) sbcPresent = (char)TRUE;
      break;

      /* 0  205.W11-B0               */
      /* 1  205.W17-B0/D0            */
      /* 2  205.W17-S0/U0            */
      /* 7  205.W32/33/36/37-B0/D0   */
      /* 8  205.W52/62/74/85-B0/D0   */
      /* 9  205.W32/33/36/37-S0/U0   */
      /* 10 205.W52/62/74/85-S0/U0   */
      /* 15 205.W113/119-B0/D0       */
      /* 16 205.W113/119-S0/U0       */
      /* 17 205.W213/219-B0/D0       */
      /* 18 205.W213/219-S0/U0       */
      /* 20 204.WD21B/23B-T2T2       */
      /* 22 205.W319-B = TIC-LINKY   */
      /* 25 204.WD21B/23B-T2xT2x     */
    case 'W':
      if ((pIdModel[5] == '1') && (pIdModel[6] == '1') && (pIdModel[8] == 'B')) idModel = ID_205_W11_B0;
      else if ((pIdModel[5] == '1') && (pIdModel[6] == '7') && ((pIdModel[8] == 'B') || (pIdModel[8] == 'D'))) idModel = ID_205_W17_B0;
      else if ((pIdModel[5] == '1') && (pIdModel[6] == '7') && ((pIdModel[8] == 'S') || (pIdModel[8] == 'U'))) idModel = ID_205_W17_S0;
      else if ((pIdModel[5] == '3') && ((pIdModel[6] == '2') || (pIdModel[6] == '3') || (pIdModel[6] == '6') || (pIdModel[6] == '7')) && ((pIdModel[8] == 'B') || (pIdModel[8] == 'D'))) idModel = ID_205_W36_B0;

      else if ((((pIdModel[5] == '5') && (pIdModel[6] == '2')) || ((pIdModel[5] == '6') && (pIdModel[6] == '2')) || ((pIdModel[5] == '7') && (pIdModel[6] == '4')) || ((pIdModel[5] == '8') && (pIdModel[6] == '5')))  
               && ((pIdModel[8] == 'B') || (pIdModel[8] == 'D'))) idModel = ID_205_W85_B0;

      else if ((pIdModel[5] == '3') && ((pIdModel[6] == '2') || (pIdModel[6] == '3') || (pIdModel[6] == '6') || (pIdModel[6] == '7')) && ((pIdModel[8] == 'S') || (pIdModel[8] == 'U'))) idModel = ID_205_W36_S0;

      else if ((((pIdModel[5] == '5') && (pIdModel[6] == '2')) || ((pIdModel[5] == '6') && (pIdModel[6] == '2')) || ((pIdModel[5] == '7') && (pIdModel[6] == '4')) || ((pIdModel[5] == '8') && (pIdModel[6] == '5')))  
               && ((pIdModel[8] == 'S') || (pIdModel[8] == 'U'))) idModel = ID_205_W85_S0;

      else if (((pIdModel[5] == '1') && (pIdModel[6] == '1')) && ((pIdModel[7] == '3') || (pIdModel[7] == '9')) && ((pIdModel[9] == 'B') || (pIdModel[9] == 'D'))) idModel = ID_W113_119_B0_D0;  
      else if ((pIdModel[5] == '1') && (pIdModel[6] == '1') && ((pIdModel[7] == '3') || (pIdModel[7] == '9')) && ((pIdModel[9] == 'S') || (pIdModel[9] == 'U'))) idModel = ID_W113_119_S0_U0;  
      else if ((pIdModel[5] == '2') && (pIdModel[6] == '1') && ((pIdModel[7] == '3') || (pIdModel[7] == '9')) && ((pIdModel[9] == 'B') || (pIdModel[9] == 'D'))) idModel = ID_W213_219_B0_D0;  
      else if ((pIdModel[5] == '2') && (pIdModel[6] == '1') && ((pIdModel[7] == '3') || (pIdModel[7] == '9')) && ((pIdModel[9] == 'S') || (pIdModel[9] == 'U'))) idModel = ID_W213_119_S0_U0;  
      else if ((pIdModel[5] == 'D') && (pIdModel[11] == '2') && (pIdModel[12] == 'T')) 
        {
          idModel = ID_204_WD21B_T2T2; if (pIdModel[7] == '3') currEmType = (char)EMETER_THREE_PH_ALGO2;
          if ((strstr((char*)&pIdModel[12], "A")) || (strstr((char*)&pIdModel[12], "E"))) sbcPresent = (char)TRUE;
        }
      else if ((pIdModel[5] == 'D') && (pIdModel[11] == '2') && (pIdModel[12] != 'T')) 
        {
          idModel = ID_204_WD21B_T2xT2x; if (pIdModel[7] == '3') currEmType = (char)EMETER_THREE_PH_ALGO2;
          if ((strstr((char*)&pIdModel[12], "A")) || (strstr((char*)&pIdModel[12], "E"))) sbcPresent = (char)TRUE;
        }
      else if ((pIdModel[5] == '3') && (pIdModel[6] == '1')  && (pIdModel[7] == '9')) idModel = ID_TIC_LINKY;
      break;

      /* 3   205.T33/37-B0/D0         */
      /* 4   205.T52/62/74/85-B0/D0   */
      /* 5   205.T33/37-S0/U0         */
      /* 6   205.T52/62/74/85-S0/U0   */
      /* 11  205.T113/119-B0/D0       */ 
      /* 12  205.T113/119-S0/U0       */
      /* 13  205.T213/219-B0/D0       */
      /* 14  205.T213/219-S0/U0       */
    case 'T':
      if ((pIdModel[5] == '3') && ((pIdModel[6] == '3') || (pIdModel[6] == '7')) && ((pIdModel[8] == 'B') || (pIdModel[8] == 'D'))) idModel = ID_205_T33_B0;

      else if ((((pIdModel[5] == '5') && (pIdModel[6] == '2')) || ((pIdModel[5] == '6') && (pIdModel[6] == '2')) || ((pIdModel[5] == '7') && (pIdModel[6] == '4')) || ((pIdModel[5] == '8') && (pIdModel[6] == '5')))  
               && ((pIdModel[8] == 'B') || (pIdModel[8] == 'D'))) idModel = ID_205_T52_B0;

      else if ((pIdModel[5] == '3') && ((pIdModel[6] == '3') || (pIdModel[6] == '7')) && ((pIdModel[8] == 'S') || (pIdModel[8] == 'U'))) idModel = ID_205_T33_S0;

      else if ((((pIdModel[5] == '5') && (pIdModel[6] == '2')) || ((pIdModel[5] == '6') && (pIdModel[6] == '2')) || ((pIdModel[5] == '7') && (pIdModel[6] == '4')) || ((pIdModel[5] == '8') && (pIdModel[6] == '5')))  
               && ((pIdModel[8] == 'S') || (pIdModel[8] == 'U'))) idModel = ID_205_T52_S0;

      else if ((pIdModel[5] == '1')  && ((pIdModel[9] == 'B') || (pIdModel[9] == 'D'))) idModel = ID_113_119_B0_D0;
      else if ((pIdModel[5] == '1')  && ((pIdModel[9] == 'S') || (pIdModel[9] == 'U'))) idModel = ID_113_119_S0_U0;
      else if ((pIdModel[5] == '2')  && ((pIdModel[9] == 'B') || (pIdModel[9] == 'D'))) idModel = ID_213_219_B0_D0;
      else if ((pIdModel[5] == '2')  && ((pIdModel[9] == 'S') || (pIdModel[9] == 'U'))) idModel = ID_213_219_S0_U0;
      break;

      /*29 205.Fx12-B/D  SOBEM                          */    
      /*30 205.Fx33-BB/D     SOBEM                      */
      /*31 205.Fx52/62-BB/DD SOBEM                      */
    case 'F':
      if ((pIdModel[6] == '1') && ((pIdModel[9] == 'B') || (pIdModel[9] == 'D'))) idModel = ID_205_Fx12_B_D;
      else if ((pIdModel[6] == '3') && ((pIdModel[9] == 'B') || (pIdModel[9] == 'D'))) idModel = ID_205_Fx33_BB_D;
      else if (((pIdModel[6] == '5') || (pIdModel[6] == '6')) && ((pIdModel[9] == 'B') || (pIdModel[9] == 'D'))) 
      {
        idModel = ID_205_Fx52_62_BB_DD;
        sbcPresent = (char)TRUE;
      }
      break;

      /*34 205.G37-BN/B0/DN  SOBEM                      */
      /*35 205.G37-D0/G74-xx SOBEM                      */
    case 'G':
      if ((pIdModel[5] == '3') && ((pIdModel[8] == 'B') || ((pIdModel[8] == 'D') && (pIdModel[9] == 'N')))) idModel = ID_205_G37_BN_B0_DN;
      else if (((pIdModel[5] == '3') && ((pIdModel[8] == 'D') && (pIdModel[9] == '0'))) || (pIdModel[5] == '7')) 
      {
        idModel = ID_205_G37_D0_G74_xx;
        if (pIdModel[5] == '7') sbcPresent = (char)TRUE;
      }
      break;

      /*32 29200/50          SOBEM                      */
      /*36 205.Fx33-BB/D  --> 28180/1/2/3   SOBEM       */ 
    case '0':
      if ((pIdModel[1] == '9') && (pIdModel[2] == '2') && ((pIdModel[3] == '0') ||(pIdModel[3] == '5'))) idModel = ID_29200_50;
      if (pIdModel[1] == '8') idModel = ID_28180_1_2_3;

      break;

      /*33 29201/02/03/51/52/53 SOBEM                   */
      /*36 205.Fx33-BB/D  --> 28180/1/2/3   SOBEM                      */ 
    case '1':
    case '2':
    case '3':
      if ((pIdModel[1] == '9') && (pIdModel[2] == '2')) 
      {
        idModel = ID_29201_02_03_51_52_53;
        sbcPresent = (char)TRUE;
      }
      else
      {
        if (pIdModel[1] == '8') idModel = ID_28180_1_2_3;
      }
      break;
  }

  if ((strstr(pIdModel, "-D")) || (strstr(pIdModel, "3B")) || (strstr(pIdModel, "-U"))) currEmType = (char)EMETER_THREE_PH_ALGO2; /* default for three energy meter */

  /*** for WB, set a broadcast address, so the real address will be assigned in the general parameter setting function  ***/
  if (((uint8_t)idModel < (uint8_t)ID_204_CA21B_T2T2) || (idModel == ID_TIC_LINKY) || 
      (idModel == ID_205_G37_BN_B0_DN) || ((idModel == ID_205_G37_D0_G74_xx) && (sbcPresent == (char)FALSE)))
  {
    currPhyScuAddr = MODBUS_BROADCAST_ADDR; 
  }
  else
  {
    pHwVer = getScuHWverFromEeprom();
    /* the first time at startup we assign an address depending on product code */
    if (sbcPresent == (char)TRUE)
    {
      if (pHwVer[0] == '8') 
      {
        /* MP00078 model with SBC, so default is 1 = master SEM*/
        currPhyScuAddr = SCU_M_P_ADDR;
      }
      else
      {
        /* MP70433 default is 2 = slave primary SEM */
        currPhyScuAddr = SCU_M_P_ADDR + 1;
      }
    }
    else
    {
      /* MP00078 model without SBC, so default is 11 = slave secondary EMUMAX0 */
      currPhyScuAddr = SCU_S_S_COLLAUDO_ADDR;
    }
  }

  upgradeGeneralParameter(idModel, currEmType, currPhyScuAddr);

  /** restart the system by NVIC reset */
  activeImmediateReset();

}


/**
*
* @brief        Set the config parameter in according to product id 
*
* @param [in]   char*: pointer to product code string
*
* @retval       none
*
***********************************************************************************************************************/
void upgradeGeneralParameter (idModel_e currModel, char currEmType, char currAddr)
{

  uint16_t tmp16;
  
  /* id model product */
  pass[0] = (char)currModel;
  /* scu address RS485 */
  pass[17] = currAddr;
  /* em type  */
  pass[5] = currEmType;


  /* now save in EEPROM check flag and actuator presence */
  // xx eeprom_array_set(CONTROL_BYTE0_EADD, (uint8_t *)&prodConfVal[pass[0]][0], 5);
  SCU_InfoStation_Set ((uint8_t *)&infoStation.controlByte.Byte.Byte0, (uint8_t *)&prodConfVal[pass[0]][0], 5);     /* ex CONTROL_BYTE0_EADD */

  /* Set in modbus map */
  setHwChecks((prodConfVal[pass[0]][1] << 8) | prodConfVal[pass[0]][0], (prodConfVal[pass[0]][3] << 8) | prodConfVal[pass[0]][2]);
  setHwActuators(prodConfVal[pass[0]][4]);

  /* now save in EEPROM power management data  */
  /* get PM visible flag *****/
  eeprom_param_get(HIDDEN_MENU_VIS_EADD, (uint8_t *)&pass[10], 1);   
  /* get PM enable flag *****/
  eeprom_param_get(HIDDEN_MENU_ENB_EADD, (uint8_t*)&pass[11], 1);

  if ((pass[0] == (uint8_t)ID_205_W11_B0) || (pass[0] == (uint8_t)ID_205_W17_B0) ||                
      (pass[0] == (uint8_t)ID_205_W17_S0) || (pass[0] == (uint8_t)ID_204_CA21B_T2T2) || 
      (pass[0] == (uint8_t)ID_204_WD21B_T2T2) || (pass[0] == (uint8_t)ID_205_A33_DD) || (pass[0] == (uint8_t)ID_205_B33_DD) ||
      (pass[0] == (uint8_t)ID_205_B33_UU) || (pass[0] == (uint8_t)ID_205_B33_UU))
  {
    pass[10] &= (char)(~HIDDEN_MENU_PMNG_VIS);
    pass[11] &= (char)(~HIDDEN_MENU_PMNG_ENB);
    pass[2] = (char)(30);  /* 1-phase: set power to 3KW -> 3000 / 100 = 30*/
    pass[3] = (char)(0);
  }
  else
  {
    pass[10] |= (char)HIDDEN_MENU_PMNG_VIS;
    if (pass[0] == (uint8_t)ID_TIC_LINKY)
    {
      /* enable power management */
      pass[11] |= (char)(HIDDEN_MENU_PMNG_ENB);
    }
    else
    {
      pass[11] &= (char)(~HIDDEN_MENU_PMNG_ENB);
    }
    if ((pass[5] & (char)EMETER_THREE_PH) == (char)EMETER_THREE_PH)
    {
      /* three Phase station */
      pass[2] = (char)(60);  /* 3-phase: set power to 6KW -> 6000 / 100 = 60*/
      pass[3] = (char)(0);
      if (currModel == ID_205_Fx12_B_D)
      {
        /* reset info for energy meter   */
        setEmModelReg(EMETER_TAMP_3, INTERNAL_EM);
        // xx eeprom_array_set(EMETER_SCU_INT_EADD, (uint8_t*)&pass[5], 1);
        SCU_InfoStation_Set ((uint8_t *)&infoStation.EmeterScu_Int, (uint8_t*)&pass[5], 1);    /* ex EMETER_SCU_INT_EADD */
        SCU_InfoStation_Set ((uint8_t *)&infoStation.emTypeInt, (uint8_t*)&pass[5], 1);        /* ex EMETER_INT_EADD */
      }
    }
    else
    {
      pass[2] = (char)(30);  /* 1-phase: set power to 3KW -> 3000 / 100 = 30*/
      pass[3] = (char)(0);
      if (currModel == ID_205_Fx12_B_D)
      {
        /* reset info for energy meter   */
        setEmModelReg(EMETER_TAMP, INTERNAL_EM);
        // xx eeprom_array_set(EMETER_SCU_INT_EADD, (uint8_t*)&pass[5], 1);
        SCU_InfoStation_Set ((uint8_t *)&infoStation.EmeterScu_Int, (uint8_t*)&pass[5], 1);     /* ex EMETER_SCU_INT_EADD */
        SCU_InfoStation_Set ((uint8_t *)&infoStation.emTypeInt, (uint8_t*)&pass[5], 1);         /* ex EMETER_INT_EADD */
      }
    }
  }
  /* save PM visible flag *****/
  // xx eeprom_array_set(HIDDEN_MENU_VIS_EADD, (uint8_t*)&pass[10], 1);
  SCU_InfoStation_Set ((uint8_t *)&infoStation.Hidden_Menu.Visible, (uint8_t*)&pass[10], 1);     /* ex HIDDEN_MENU_VIS_EADD */
  /* save PM enable flag *****/
  // xx eeprom_array_set(HIDDEN_MENU_ENB_EADD, (uint8_t*)&pass[11], 1);
  SCU_InfoStation_Set ((uint8_t *)&infoStation.Hidden_Menu.Enabled, (uint8_t*)&pass[11], 1);     /* ex HIDDEN_MENU_ENB_EADD */
  /* save PMAX value as KW * 10 unit measure  *****/
  // xx eeprom_array_set(PMNG_PWRLSB_EADD, (uint8_t*)&pass[2], 1);
  // xx eeprom_array_set(PMNG_PWRMSB_EADD, (uint8_t*)&pass[3], 1);
  tmp16 = pass[2] | (pass[3] << 8);
  SCU_InfoStation_Set ((uint8_t *)&infoStation.Pmng.Power, (uint8_t*)&tmp16, 2);        /* ex PMNG_PWRLSB_EADD - PMNG_PWRMSB_EADD */

  pass[1] = (char)SOCKET_T2_NO_LID; /* 2B */
  pass[6] = (char)0;                /* battery backup OFF */
  if ((pass[0] == (uint8_t)ID_205_W17_S0)     || (pass[0] == (uint8_t)ID_205_T33_S0)    ||             
      (pass[0] == (uint8_t)ID_205_T52_S0)     || (pass[0] == (uint8_t)ID_205_W36_S0)    ||
      (pass[0] == (uint8_t)ID_205_W85_S0)     || (pass[0] == (uint8_t)ID_113_119_S0_U0) ||
      (pass[0] == (uint8_t)ID_213_219_S0_U0)  || (pass[0] == (uint8_t)ID_W113_119_S0_U0)||
      (pass[0] == (uint8_t)ID_W213_119_S0_U0) || (pass[0] == (uint8_t)ID_204_CA21B_T2xT2x) ||
      (pass[0] == (uint8_t)ID_205_A33_UU)     || (pass[0] == (uint8_t)ID_205_B33_UU) ||
      (pass[0] == (uint8_t)ID_204_WD21B_T2xT2x) || (pass[0] == (uint8_t)ID_204_CA_TFT_STU) ||
      (pass[0] == (uint8_t)ID_204_CA11B_T2x))         
  {                                                                                             
    pass[1] = (char)SOCKET_T2_TETHERED; /* 2_CBL */                                             
  }                                                                                             
  else 
  {
    if((pass[0] == (uint8_t)ID_204_CA21B_T2T2) || (pass[0] == (uint8_t)ID_204_WD21B_T2T2) || (pass[0] == (uint8_t)ID_205_A33_DD) || (pass[0] == (uint8_t)ID_205_B33_DD) ||
        (pass[0] == (uint8_t)ID_204_CA_TFT_FGH) || (pass[0] == (uint8_t)ID_204_CA11B_T2))
    {                                                                                             
      pass[1] = (char)SOCKET_T2_OPEN_LID; /* 2B_LO */                                             
    }  
    else
    {
      if((pass[0] == (uint8_t)ID_204_CA21B_UNUN) || (pass[0] == (uint8_t)ID_204_UB21B_EB) || (pass[0] == (uint8_t)ID_205_KB30_K) ||
         (pass[0] == (uint8_t)ID_204_UB11B_EB))
      {                               
        pass[1] = (char)SOCKET_SK_CLOSE_LID; /* SB_LC Schuko Plug Lock and Lid (Closed in Charge)*/                                             
      }  
      else
      {
        if((pass[0] == (uint8_t)ID_29200_50) || (pass[0] == (uint8_t)ID_29201_02_03_51_52_53))
        {                                                                                             
          pass[1] = (char)SOCKET_T2_CLOSE_LID; /* 2b_LC presa tipo 2 coperchio chiuso in carica*/                                             
        }  
      }
    }
  }
  if((pass[0] == (uint8_t)ID_204_CA21B_T2T2) || (pass[0] == (uint8_t)ID_204_WD21B_T2T2)    || (pass[0] == (uint8_t)ID_205_A33_DD)     ||
     (pass[0] == (uint8_t)ID_205_B33_DD)     || (pass[0] == (uint8_t)ID_205_A33_UU)        || (pass[0] == (uint8_t)ID_205_B33_UU)     || 
     (pass[0] == (uint8_t)ID_204_CA21B_T2xT2x) || (pass[0] == (uint8_t)ID_204_WD21B_T2xT2x)|| (pass[0] == (uint8_t)ID_204_CA21B_UNUN) || 
     (pass[0] == (uint8_t)ID_204_CA21B_T2T2) || (pass[0] == (uint8_t)ID_204_CA21B_T2xT2x)  || (pass[0] == (uint8_t)ID_204_CA21B_UNUN) || 
     (pass[0] == (uint8_t)ID_204_CA_TFT_BCD) || (pass[0] == (uint8_t)ID_204_CA_TFT_FGH)    || (pass[0] == (uint8_t)ID_204_CA_TFT_STU) ||
     (pass[0] == (uint8_t)ID_205_N_TFT_BCD) ||  (pass[0] == (uint8_t)ID_204_CA11B_T2x)     || (pass[0] == (uint8_t)ID_204_CA11B_T2))
  {
    pass[6] = (char)1;                  /* battery backup ON */                                 
  }
  // xx eeprom_array_set(SOCKET_TYPE_EADD, (uint8_t*)&pass[1], 1);                                    
  SCU_InfoStation_Set ((uint8_t*)&infoStation.socketType, (uint8_t*)&pass[1], 1);        /* ex SOCKET_TYPE_EADD */
  /* set backup flag: 1 = active      */                                                        
  // xx eeprom_array_set(BATTERY_CONFIG_EADD, (uint8_t*)&pass[6], 1);                                 
  SCU_InfoStation_Set ((uint8_t*)&infoStation.batteryConfig, (uint8_t*)&pass[6], 1);      /* ex BATTERY_CONFIG_EADD */
                                                                                                
  pass[6] = (char)LED_STRIP_06;                                                                 
  if((pass[0] == (uint8_t)ID_204_CA21B_T2T2) || (pass[0] == (uint8_t)ID_204_CA21B_T2xT2x) || (pass[0] == (uint8_t)ID_204_CA21B_UNUN) ||
     (pass[0] == (uint8_t)ID_204_CA_TFT_BCD) || (pass[0] == (uint8_t)ID_204_CA_TFT_FGH) || (pass[0] == (uint8_t)ID_204_CA_TFT_STU) ||
     (pass[0] == (uint8_t)ID_204_CA11B_T2x)  || (pass[0] == (uint8_t)ID_204_CA11B_T2))        
  {                                                                                             
    pass[6] = (char)LED_STRIP_18;                                                               
  }                                                                                             
  else                                                                                          
  {                                                                                             
    if ((pass[0] == (uint8_t)ID_113_119_B0_D0) || (pass[0] == (uint8_t)ID_113_119_S0_U0) ||     
        (pass[0] == (uint8_t)ID_213_219_B0_D0) || (pass[0] == (uint8_t)ID_213_219_S0_U0) ||
        (pass[0] == (uint8_t)ID_205_T33_B0) || (pass[0] == (uint8_t)ID_205_T52_B0) ||
        (pass[0] == (uint8_t)ID_205_T33_S0) || (pass[0] == (uint8_t)ID_205_T52_S0))
    {
      pass[6] = (char)LED_STRIP_09;
    }
    else
    {
      if ((pass[0] == (uint8_t)ID_205_A33_DD) || (pass[0] == (uint8_t)ID_205_A33_UU) || (pass[0] == (uint8_t)ID_205_B33_DD) || (pass[0] == (uint8_t)ID_205_B33_UU))    
      {
        pass[6] = (char)LED_STRIP_12;
      }
    }
  }
  // xx eeprom_array_set(STRIP_LED_TYPE_EADD, (uint8_t*)&pass[6], 1);
  SCU_InfoStation_Set ((uint8_t*)&infoStation.StripLedType, (uint8_t*)&pass[6], 1);    /* ex STRIP_LED_TYPE_EADD */
  setNewCurrentLed();

  /* set variable for sinapsi   *****/
  pass[9] = pass[6] = pass[7] = (char)0; 

  pass[9] = (char)EMETER_EXT_CRL2;          // set l'abilitazione allarme EM esterno per tutti i prodotti    
  if ((pass[0] == (uint8_t)ID_W213_219_B0_D0) || (pass[0] == (uint8_t)ID_W213_119_S0_U0))                
  {
    pass[9] = (char)(EMETER_EXT_CRL2 | SINAPSI_CHN2_CRL2);          // set l'abilitazione allarme chain2 per i prodotti SINAPSI    
  }
  /* save LCD type, if present  *****/
  pass[1] = (char)LCD_2X20;               /* LCD type is alpha 2x20 */
  pass[7] = (char)HIDDEN_MENU_PMNG_VIS;   /* default visible for PM menù  */                                                    
  if ((pass[0] == (uint8_t)ID_205_W17_B0)     || (pass[0] == (uint8_t)ID_205_W17_S0) ||            
      (pass[0] == (uint8_t)ID_W113_119_B0_D0) || (pass[0] == (uint8_t)ID_W113_119_S0_U0) || 
      (pass[0] == (uint8_t)ID_W213_219_B0_D0) || (pass[0] == (uint8_t)ID_W213_119_S0_U0) || 
      (pass[0] == (uint8_t)ID_113_119_B0_D0)  || (pass[0] == (uint8_t)ID_113_119_S0_U0)  ||
      (pass[0] == (uint8_t)ID_213_219_B0_D0)  || (pass[0] == (uint8_t)ID_213_219_S0_U0)  ||
      (pass[0] == (uint8_t)ID_205_W11_B0)     || (pass[0] == (uint8_t)ID_TIC_LINKY))
  {                                                                                             
    pass[1] = (char)LCD_TYPE_NULL; /* no LCD */                                                 
    if ((pass[0] == (uint8_t)ID_113_119_B0_D0) || (pass[0] == (uint8_t)ID_113_119_S0_U0) ||     
        (pass[0] == (uint8_t)ID_213_219_B0_D0) || (pass[0] == (uint8_t)ID_213_219_S0_U0) ||       
        (pass[0] == (uint8_t)ID_W113_119_B0_D0) || (pass[0] == (uint8_t)ID_W113_119_S0_U0) || 
        (pass[0] == (uint8_t)ID_W213_219_B0_D0) || (pass[0] == (uint8_t)ID_W213_119_S0_U0) ||
        (pass[0] == (uint8_t)ID_TIC_LINKY))
    {                                                                                           
      /* produt where wifi must be enabled */                                                   
      pass[1] |= (char)WIFI_MASK;                                                               
      if ((pass[0] == (uint8_t)ID_213_219_B0_D0) || (pass[0] == (uint8_t)ID_213_219_S0_U0) ||
          (pass[0] == (uint8_t)ID_W213_219_B0_D0) || (pass[0] == (uint8_t)ID_W213_119_S0_U0))     
      {                                                                                         
        /* sinapsi products */                                                                  
        pass[6] |= (char)HIDDEN_MENU_SINAPSI;      // set  SINAPSI                              
        /* for debug: clear sinapsi error */                                                    
        resetStationSinapsiRS485Error();                                                        
      }                                                                                         
    }                                                                                           
    if ((pass[0] == (uint8_t)ID_205_W17_B0) || (pass[0] == (uint8_t)ID_205_W17_S0) || (pass[0] == (uint8_t)ID_205_W11_B0))
    {
        pass[7] = (char)0;                                                       
    }
  }
  else
  {
    if ((pass[0] == (uint8_t)ID_204_CA_TFT_BCD) || (pass[0] == (uint8_t)ID_204_CA_TFT_FGH) || 
         (pass[0] == (uint8_t)ID_204_CA_TFT_STU) || (pass[0] == (uint8_t)ID_205_N_TFT_BCD) ||
         (pass[0] == (uint8_t)ID_204_UB21B_EB)   || (pass[0] == (uint8_t)ID_204_UB11B_EB))  
    {
      pass[1] = (char)LCD_TYPE_NULL; /* no LCD for product with TFT */                                                 
    }
  }

  /* 19 204.CA21B/21B-T2T2   20 204.WD21B/23B-T2T2  21 205.A-B/33/52-62-DD   23 ID_205_AB33_UU */    
  if (pass[0] >= (uint8_t)ID_204_CA21B_T2T2)            
  {        
    /* visible for PM menù is OFF  */
    pass[7] = (char)0;                                                       
  }
                                                                                           
  // xx eeprom_array_set(LCD_TYPE_EADD, (uint8_t*)&pass[1], 1);                                       
  SCU_InfoStation_Set ((uint8_t*)&infoStation.LcdType, (uint8_t*)&pass[1], 1);         /* ex LCD_TYPE_EADD */

  /* get current status for sinapsi and visibility */
  eeprom_param_get(CONTROL_BYTE2_EADD, (uint8_t*)&pass[11], 1);
  eeprom_param_get(HIDDEN_MENU_ENB_EADD, (uint8_t *)&pass[3], 1);
  /* get PM visible flag *****/
  eeprom_param_get(HIDDEN_MENU_VIS_EADD, (uint8_t *)&pass[8], 1);

  /* set abilitazione allarme EM esterno */
  //            pass[11] &= ((char)~EMETER_EXT_CRL2);
  pass[11] &= ((char)~(EMETER_EXT_CRL2 | SINAPSI_CHN2_CRL2));
  pass[11] |= (char)pass[9];           
  /* set SINAPSI */
  pass[3] &= ((char)~HIDDEN_MENU_SINAPSI);
  pass[3] |= (char)pass[6];         
  /* set visibility flag  */
  pass[8] &= ((char)~HIDDEN_MENU_PMNG_VIS);
  pass[8] |= (char)pass[7];           

  /* save EMEX enable flag *****/
  // xx eeprom_array_set(CONTROL_BYTE2_EADD, (uint8_t*)&pass[11], 1);
  SCU_InfoStation_Set ((uint8_t*)&infoStation.controlByte.Byte.Byte2, (uint8_t*)&pass[11], 1);     /* ex CONTROL_BYTE2_EADD */
  /* save SINAPSI enable flag *****/
  // xx eeprom_array_set(HIDDEN_MENU_ENB_EADD, (uint8_t*)&pass[3], 1);
  SCU_InfoStation_Set ((uint8_t*)&infoStation.Hidden_Menu.Enabled, (uint8_t*)&pass[3], 1);         /* ex HIDDEN_MENU_ENB_EADD */

  /* save PM visible flag *****/
  // xx eeprom_array_set(HIDDEN_MENU_VIS_EADD, (uint8_t*)&pass[8], 1);
  SCU_InfoStation_Set ((uint8_t*)&infoStation.Hidden_Menu.Visible, (uint8_t*)&pass[8], 1);         /* ex HIDDEN_MENU_VIS_EADD */
    
  /* now find corrente massima in Modo3 standard */
  /* default typical current is 32A  */
  pass[13] = 32; 
  if ((pass[0] == (uint8_t)ID_204_CA21B_UNUN) || (pass[0] == (uint8_t)ID_204_UB21B_EB) || (pass[0] == (uint8_t)ID_205_KB30_K)
      || (pass[0] == (uint8_t)ID_204_UB11B_EB))      
  { 
    /* only for this products the typical current is 16A */ 
    pass[13] = 16; 
  }

  // xx eeprom_array_set(M3T_CURRENT_EADD, (uint8_t*)&pass[13], 1); 
  SCU_InfoStation_Set ((uint8_t*)&infoStation.max_current, (uint8_t*)&pass[13], 1);       /* ex M3T_CURRENT_EADD */
  
  pass[13] = (char)setNominalPower((uint16_t)pass[13]);
  // xx eeprom_array_set(STATION_NOM_PWR_EADD, (uint8_t*)&pass[13], 1); 
  SCU_InfoStation_Set ((uint8_t*)&infoStation.StationNominalPower, (uint8_t*)&pass[13], 1);    /* ex STATION_NOM_PWR_EADD */
    
  /* corrente massima in Modo3 semplificato is 16A for all products */
  pass[13] = 16; 
  // xx eeprom_array_set(M3S_CURRENT_EADD, (uint8_t*)&pass[13], 1);
  SCU_InfoStation_Set ((uint8_t*)&infoStation.max_currentSemp, (uint8_t*)&pass[13], 1);     /* ex M3S_CURRENT_EADD */

  pass[14] = SKT_HIHG_SX; // set as default the socket position HIGH on the Left 
  /*** a default address 16 (logical 15) must be set if the current address in undefined ***/
  if (pass[17] == MODBUS_BROADCAST_ADDR)
  {
    pass[1] = 15; pass[17] = 16; pass[12] = (char)SCU_GSY;
    if ((pass[0] == (uint8_t)ID_205_T52_B0) || (pass[0] == (uint8_t)ID_205_T52_S0) ||       
        (pass[0] == (uint8_t)ID_205_W85_B0) || (pass[0] == (uint8_t)ID_205_W85_S0))
    { 
      /* product equipped with SBC must have logical address 0 (phisical 1)*/ 
      pass[1] = 0; pass[17] = 1; pass[12] = (char)SCU_SEM_M;
    }
    // xx eeprom_array_set(RS485_ADD_EADD, (uint8_t*)&pass[1], 1);
    SCU_InfoStation_Set ((uint8_t*)&infoStation.rs485Address, (uint8_t*)&pass[1], 1);    /* ex RS485_ADD_EADD */
  }
  else
  {
    pass[12] = (char)SCU_SEM_S;
    pass[1] = pass[17] - 1;
    // xx eeprom_array_set(RS485_ADD_EADD, (uint8_t*)&pass[1], 1);
    SCU_InfoStation_Set ((uint8_t*)&infoStation.rs485Address, (uint8_t*)&pass[1], 1);    /* ex RS485_ADD_EADD */
  }

  if (pass[0] < ID_204_CA21B_T2T2)
  {
    pass[6] = 1;  /* for WB index in fake code is always the first */
  }
  else
  {
    pass[6] = pass[17] % 10;   /* in production the sockets have default number 11,12,13,14 ID_204_CA21B_T2T2 */

    if ((pass[0] == (uint8_t)ID_205_B33_DD) || (pass[0] == (uint8_t)ID_205_B33_UU)  || (pass[0] == (uint8_t)ID_204_CA21B_T2xT2x)  || (pass[0] == (uint8_t)ID_204_CA21B_T2T2))
    {
      /* pillar with socket on high-left and high-right */
      if ((pass[17] == SCU_M_P_ADDR) || (pass[17] == SCU_S_S_COLLAUDO_ADDR)) pass[14] = SKT_HIHG_SX; else pass[14] = SKT_HIHG_DX;
    }
    if ((pass[0] == (uint8_t)ID_205_A33_DD) || (pass[0] == (uint8_t)ID_205_A33_UU))
    {
      /* pillar with socket on high-left and low-left */
      if ((pass[17] == SCU_M_P_ADDR) || (pass[17] == SCU_S_S_COLLAUDO_ADDR)) pass[14] = SKT_HIHG_DX; else pass[14] = SKT_LOW_DX;
    }
  }
  /* in this context the addres is the index for fake code */
  // xx eeprom_array_set(CONNECTOR_NUMBER_EADD, (uint8_t*)&pass[6], 1);
  SCU_InfoStation_Set ((uint8_t*)&infoStation.connectorNumber, (uint8_t*)&pass[6], 1);      /* ex CONNECTOR_NUMBER_EADD */

  /* set all parameter in according with SEM enviroment */
  if (pass[17] == SCU_M_P_ADDR)
  {
    pass[2] = (char)SCU_SEM_M;
    setAddressType((uint8_t)SCU_FIXED_ADDR, TRUE);
    setPmRemoteSemFlag(0);
  }
  else
  {
    if ((pass[0] < ID_204_CA21B_T2T2) || (pass[0] == ID_205_KB30_K))
    {
      /* we are in the WB enviroment with address 2..16 --> SCU_GSY (EMUMAX0) */
      pass[2] = (char)SCU_GSY;
      setAddressType((uint8_t)SCU_TEMPORARY_ADDR, TRUE);
    }
    else
    {
      /*address 2..4 --> SEM Slave or address 12..14 --> SCU_GSY (EMUMAX0) */
      if (pass[17] < SCU_S_S_COLLAUDO_ADDR) pass[2] = (char)SCU_SEM_S; else  pass[2] = (char)SCU_GSY;
      /* read current value for SEM Flags */
      eeprom_param_get(SEM_FLAGS_CTRL_EADD, (uint8_t*)&pass[1], 1);
      pass[1] &= (~SCU_ADDR_MODE_MASK);  /* this means "temporary" */
      pass[8] = (char)SCU_TEMPORARY_ADDR;
      if (((pass[17] > SCU_M_P_ADDR) && (pass[17] <= SCU_M_P_ADDR + 3)))
      {
        /*address 2..4 --> SEM Slave with fixed address */ 
        pass[1] |= SCU_FIXED_ADDR;
        pass[8] = (char)SCU_FIXED_ADDR;
        setPmRemoteSemFlag(0);
      }
      // xx eeprom_array_set(SEM_FLAGS_CTRL_EADD, (uint8_t*)&pass[1], 1);
      SCU_InfoStation_Set ((uint8_t*)&infoStation.semFlagControl, (uint8_t*)&pass[1], 1);          /* ex SEM_FLAGS_CTRL_EADD */
      setAddressType((uint8_t)pass[8], TRUE);
    }
  }
  if ((pass[12] == (char)SCU_GSY) && (getCollaudoRunning() == FALSE))
  {
    /* non siamo in SEM ma in EMUMAX0  */
    /* SCU emulazione MAX0 with GSY */
    pass[2] = (char)SCU_GSY;
    /* Here reset the power management Remote flag */
    ResetPmRemoteSemFlag (0);
  }
  // xx eeprom_array_set(OPERATIVE_MODE_EADD, (uint8_t*)&pass[2], 1);
  SCU_InfoStation_Set ((uint8_t*)&infoStation.Operative_mode, (uint8_t*)&pass[2], 1);      /* ex OPERATIVE_MODE_EADD */
  
  /* set SCU socket position. Read current status on pass[3]       */
  eeprom_param_get(LCD_TYPE_EADD, (uint8_t *)&pass[3], 1);

  switch ((uint8_t)pass[14])
  {
    case SKT_HIHG_DX:
    case SKT_HIHG_SX:
    case SKT_LOW_DX:
    case SKT_LOW_SX:
      pass[4] = ((pass[3] & (uint8_t)(~SKT_POS_MASK)) | (uint8_t)pass[14]);
      // xx eeprom_array_set(LCD_TYPE_EADD, (uint8_t*)&pass[4], 1);
      SCU_InfoStation_Set ((uint8_t*)&infoStation.LcdType, (uint8_t*)&pass[4], 1);          /* ex LCD_TYPE_EADD */
      break;

    default:
      break;
  }

  /* set the flag for complete reception serial number and code */
  if (setSerialReceivedFlag() == 0)
  {
    /* Write configurations in eeprom array */
    eeprom_ProductConfig_Param_Set();

    /* a new infoStation backup copy in EEPROM is need */
    BKP_SCU_Image_Store();

    /* set new default product parameters      */
    eeprom_default_set();
  }
}


/**
*
* @brief        converte da hex a int 
*
* @param [in]   char: hex char 
*
* @retval       int: int for the supplied char
*
***********************************************************************************************************************/
static int hex_to_int(char c) 
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    } else if (c >= 'a' && c <= 'f') {
        return 10 + (c - 'a');
    } else if (c >= 'A' && c <= 'F') {
        return 10 + (c - 'A');
    }
    return 0; // Se non è una cifra esadecimale valida, restituisce 0 (comportamento di default)
}

/**
*
* @brief        Decodifica una stringa URL 
*
* @param [in]   char*: pointer to received password string 
*
* @retval       char*: pointer to decoded string (converte %xx nel carattere speciale corrispondente)
*
***********************************************************************************************************************/
static char* url_decode(const char* encoded_str) 
{
    if (encoded_str == NULL) {
        return NULL;
    }

    size_t len = strlen(encoded_str);
    // Alloca memoria per la stringa decodificata.
    // Nel caso peggiore, la stringa decodificata può avere la stessa lunghezza di quella codificata
    // (se non ci sono % o +). Aggiungiamo +1 per il terminatore nullo '\0'.
    char* decoded_str = (char*) malloc(len + 1);
    if (decoded_str == NULL) {
        perror("Errore nell'allocazione di memoria per la stringa decodificata");
        return NULL;
    }

    size_t i = 0; // Indice per la stringa di input (encoded_str)
    size_t j = 0; // Indice per la stringa di output (decoded_str)

    while (i < len) {
        if (encoded_str[i] == '%') {
            // Controlla se ci sono abbastanza caratteri per un codice esadecimale valido (%XX)
            if (i + 2 < len && isxdigit(encoded_str[i+1]) && isxdigit(encoded_str[i+2])) {
                // Converte le due cifre esadecimali nel loro valore intero
                int high_nibble = hex_to_int(encoded_str[i+1]);
                int low_nibble = hex_to_int(encoded_str[i+2]);
                
                // Combina i nibble per ottenere il valore del carattere e lo aggiunge alla stringa decodificata
                decoded_str[j++] = (char)((high_nibble << 4) | low_nibble);
                i += 3; // Salta il '%', la prima cifra hex e la seconda cifra hex
            } else {
                // Sequenza '%' malformata (es. '%' alla fine o non seguita da 2 cifre hex)
                // Copia il '%' così com'è nella stringa decodificata e prosegue
                decoded_str[j++] = encoded_str[i++];
            }
        } else if (encoded_str[i] == '+') {
            // Gestisce il '+' come uno spazio (comune nei dati di form URL-encoded)
            decoded_str[j++] = ' ';
            i++;
        } else {
            // Copia il carattere così com'è nella stringa decodificata
            decoded_str[j++] = encoded_str[i++];
        }
    }
    decoded_str[j] = '\0'; // Termina la stringa decodificata con un carattere nullo

    return decoded_str;
}

/**
  * @brief  set the downgrade status   
  *         
  * @param  uint8_t: downgrade key status 
  * 
  * @retval none
  */
void setDowngradeStatus(uint8_t dwngStatus)
{
  downgradeStatus = dwngStatus;
}

/**
  * @brief  get the downgrade status   
  *         
  * @param  none 
  * 
  * @retval uint8_t: downgrade key status
  */
uint8_t getDowngradeStatus(void)
{
  return(downgradeStatus);
}



#ifdef MODBUS_TCP_EM_LOVATO
/**
*
* @brief        Get the pointer to task queue
*
* @param [in]   none
*
* @retval       xQueueHandle: pointer to defined  queue
*
***********************************************************************************************************************/
xQueueHandle getEmLovatoQueueHandle(void)
{
   return(emAnswMasterQueue);
}
#endif

/************************ (C) COPYRIGHT SCAME S.p.A. *****END OF FILE****/

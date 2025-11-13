/**
* @file        scuSem.h
*
* @brief       manager upgrade notify MDB register for SEM   - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: scuMdb.h 334 2023-11-06 11:11:45Z luca $
*
*     $Revision: 334 $
*
*     $Author: luca $
*
*     $Date: 2023-11-06 12:11:45 +0100 (lun, 06 nov 2023) $
*
*
* @copyright
*       Copyright (C) 2016 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/


#ifndef _SBCSEM_H
#define _SBCSEM_H
#include "Em_Task.h"
#include "sbcGsy.h"

// -------------------------- global defines --------------------------------------------------------------------------------- //

#define   SCU_NUM                           ((uint16_t)16)  /* massimo sono 16 SCU primarie/secondarie */

/* ----------- numero buffer nella coda di ricezione  -------------*/
#define   NUM_BUFF_SEM_SCU                  ((uint16_t)16) 
#define   NUM_BUFF_RS485_SEM                ((uint16_t)16) 
#define   NUM_BUFF_POLLING                  ((uint16_t)2) 
#define   NUM_OFF_LINE_RETRY                ((uint16_t)1)

#define   SIZE_MODBUS_MAP                   ((uint16_t)0x0800)  
     
#define   MODBUS_DATA_MAX_LEN               ((uint16_t)124)   /* 124 word --> 248 bytes < 253*/

#define   TIMEOUT_ACK_RESP                  ((uint16_t)500)   /* timeout ack for RD / WR operation */
#define   TIMEOUT_ACK_TRANSMITTER_RESP      ((uint16_t)100)   /* timeout ack for RD / WR operation for trnsmitter */
#define   NUM_TRANS_RETRY                   ((uint16_t)2)

//#define   WAIT_FOR_START_DISCOVERY          pdMS_TO_TICKS((uint16_t)4000)
#define   WAIT_FOR_START_DISCOVERY          pdMS_TO_TICKS((uint16_t)10000)
#define   WAIT_FOR_DISCOVERY_SLAVE          pdMS_TO_TICKS((uint16_t)1000)
#define   WAIT_FOR_NEXT_DISCOVERY           pdMS_TO_TICKS((uint16_t)2500)
#define   TIMEOUT_RD_REQ                    ((uint16_t)20)    /* min time between read request     */
#define   TICK_FOR_RTC_SYNCRO               pdMS_TO_TICKS((uint16_t)10000)
#define   WAIT_FOR_END_EDGE_ADDR            pdMS_TO_TICKS((uint16_t)100)
#define   WAIT_FOR_OPERATING_STATE          pdMS_TO_TICKS((uint16_t)3000)
#define   WAIT_TO_OPERATIVE_POST_ASS_ADDR   pdMS_TO_TICKS((uint16_t)5000)
#define   WAIT_FOR_EEPROM_WRITE             pdMS_TO_TICKS((uint16_t)200)

#ifndef POLLING_to_3s
/* Polling time is calculated as the sum of these 2 values and then doubled (x2) */
#define   ACTIVITY_PERIOD_CHECK_TIME        pdMS_TO_TICKS((uint32_t)8000)   
#define   START_PERIOD_CHECK_LIVE           pdMS_TO_TICKS((uint32_t)2000)
#else
#define   ACTIVITY_PERIOD_CHECK_TIME        pdMS_TO_TICKS((uint32_t)2000)
#define   START_PERIOD_CHECK_LIVE           pdMS_TO_TICKS((uint32_t)1000)
#endif

#define   STEP_PERIOD_CHECK_TIME            pdMS_TO_TICKS((uint32_t)20)
#define   TIMEOUT_NEXT_TX                   pdMS_TO_TICKS((uint32_t)2)
#define   TIMEOUT_ACK_POLLING               pdMS_TO_TICKS((uint32_t)20)   /* timeout ack in polling operation */
#define   SUSPEND_FOR_UPLOAD_TIME           pdMS_TO_TICKS((uint32_t)120000)

#define   WAIT_FOR_REQ_INFO                 pdMS_TO_TICKS((uint16_t)811)

#define   SCU_MASTER_MASK_BIT               ((uint32_t)0x00000001)

#define   NULL_RANDOM                       ((uint32_t)0xFFFFFFFFF)

/* define for supend / release management: in SEM is different from GSY */
#define   NUM_BUFF_REMOTE                   ((uint16_t)2) 
#define   TIMER_ACTION_AT_START             pdMS_TO_TICKS((uint32_t)200)   /* timeout for action after task starts  */


#define   PM_TIME_RANGE_FUNC_MASK           0x8000
#define   PM_TIME_RANGE_FUNC_BIT_POS        15

#define   REMOVE_CONNECTOR_ID_START         ((uint16_t)0xDD02)
#define   REMOVE_CONNECTOR_ID_END           ((uint16_t)0xDD10)

#define   NULL_ID                           ((uint16_t)0xFFFF)

typedef enum
{
  SBC_SEM_EVENT_RS485     = 0x0000,       /* Message for new event to process coming from RS485   */ 
  SBC_SEM_EVENT_UART5,                    /* Message for new event to process sending over RS485  */ 
  NOTIFY_TO_MASTER_TX,                    /* Message for notify to master any info                */ 
  NOTIFY_TO_SLAVE_TX,                     /* Message for notify to slave some info                */
  RECOVERY_INFO,                          /* Message for to get info on discovered socket         */
  NOTIFY_MODBUS_ACK,                      /* Message to notify an ACK                             */
  NOTIFY_TO_SLAVE_RX,                     /* Message for notify to slave to send us some info     */
  NOTIFY_MODBUS_RD_ACK,                   /* Message to notify an ACK to read operation           */
  NOTIFY_MODBUS_WR_ACK,                   /* Message to notify an ACK to write operation          */
  NOTIFY_START_TASK,                      /* the task can start                                   */
  SCU_EVENT_MSG_FROM_SBC_RD,              /* received a reading message from                      */ 
  SCU_EVENT_MSG_FROM_MASTER_RD,           /* received a reading message from SCU master           */ 
  POLLING_CHANGE_INFO,                    /* Message to change info by polling request            */
  SBC_SEM_EVENT_UART5_SEND_DWLD_CMD,      /* Message in SCU stand alone to start dwld on RS485    */ 
  SBC_SEM_TIMEOUT       = 0xFFFE,         /* Message for timeout                                  */ 
  SBC_SEM_DEF_EVENT     = 0xFFFF
} sbcSemEvent_e;

typedef enum
{
  SBC_SEM_INIT_DISCOVERY    = 0x0000,       /* initial state                                    */ 
  SBC_SEM_DISCOVERY_S,                      /* discovery slave state                            */ 
  SBC_SEM_RECOVERY_INFO,                    /* get info from master / slave in the chain        */ 
  SBC_SEM_ACK_RECOVERY_INFO,                /* received info from master / slave in the chain   */ 
  SBC_SEM_OPERATIVE,                        /* normal condition                                 */ 
  SBC_SEM_WAIT_WR_ACK,                      /* wait for write ack                               */ 
  SBC_SEM_WAIT_WR_MASTER_ACK,               /* wait for write ack from master                   */ 
  SBC_SEM_WAIT_WR_MASTER_POLL_ACK,          /* wait for write ack at polling from master        */ 
  SBC_SEM_WAIT_TO_START,                    /* wait for signal to start                         */ 
  SBC_SEM_WAIT_TO_REGISTER,                 /* wait for registration from master                */ 
  SBC_SEM_WAIT_RS485_ADDR,                  /* wait for rs485 assign from master                */ 
  SBC_SEM_CHECK_PREVIOUS_SCU,               /* check the presence of previuos discovered SCU    */ 
  SBC_SEM_WAIT_TO_BE_OPERATIVE,             /* wait to be entering in operative mode            */ 
  SBC_SEM_WAIT_WR_MASTER_ACK_STARTUP,       /* wait for write ack from master after a restart   */ 
  SBC_SEM_OPERATIVE_COLLAUDO,               /* when in testing wiyh automatic nachine           */
  SBC_SEM_BLANK_AFTER_ASS_ADDR,             /* after assigned affress no transmission on RS485  */
  SBC_SEM_DUMMY             = 0xFFFF
} sbcSemStates_e;

typedef enum
{
  SLAVE_INIT    = 0x0000,                   /* init state                                       */ 
  SLAVE_IDLE,                               /* wait polling timeout state                       */ 
  FIND_SLAVE,                               /* find first slave to be check                     */ 
  ASK_SLAVE,                                /* read request for a slave                         */ 
  CTRL_SLAVE,                               /* check slave answer                               */ 
  FW_DOWNLOAD,                              /* fw download ongoing                              */
  ADDING_SLAVE,                             /* check adding new slave                           */ 
  FIND_NEW_SLAVE                            /* check new slave in the system                    */ 
}pollingSlaveMng_e;

typedef enum
{
  NO_OPERATION      = 0x0000,                 /* No operation                                     */ 
  END_CHARGE        = 0x0001,                 /* Termine carica                                   */ 
  START_CHARGE      = 0x0002,                 /* Inizio  carica                                   */ 
  SOCKET_MOVING     = 0x0003,                 /* Movimento bloccco presa                          */ 
  MODE_AVAILABLE    = 0x0005,                 /* Cambio disponibilità a disponibile               */ 
  MODE_UNAVAILABLE  = 0x0006,                 /* Cambio disponibilità a indisponibile             */ 
  MODE_RESERVED     = 0x0007,                 /* Cambio disponibilità a riservato                 */ 
  SOFT_REBOOT       = 0x00FC,                 /* Reboot in sicurezza                              */ 
  HARD_REBOOT       = 0x00FD,                 /* Reboot immediato                                 */ 
  FACTORY_RESTORE   = 0x00FE                  /* Reset di fabbrica                                */ 
}remoteCmd_e;

typedef enum
{
  EVS_MODE_AVAILABLE    = 0x01,               /* Cambio disponibilità a disponibile               */ 
  EVS_MODE_UNAVAILABLE  = 0x00,               /* Cambio disponibilità a indisponibile             */ 
  EVS_MODE_RESERVED     = 0x02                /* Cambio disponibilità a reserved [NOT USED!]      */ 
}remoteCmdEvs_e;


/* ***************  definizione  gestione timer del task   ************** */
typedef enum
{
  TIMER_FIND_CONFIG = 0,    // timer for master / slave detection    
  TIMER_FOR_ACK,            // timer for master / slave ack answer    
  TIMER_FOR_TICK_RTC,       // timer tick for RTC to Modbus syncronization 
  TIMER_FOR_ACK_POLLING,    // timer tick to check answer on polling  
  TIMER_FOR_REQ_ADDR,       // timer tick to request an address to master   
  NUM_SBCSEM_TIMER
} sbcSemTimerIdx_e;


/* SCU index and relative address  */
typedef __packed struct
{
  uint16_t    index;
  uint16_t    rAddr;
}regSbcAddr_t;

/* SCU index and relative address  */
typedef __packed struct
{
  uint16_t    len;
  uint8_t*    pData;
}regDataToSend_t;

/* SCU index and relative address  */
typedef __packed struct
{
  sbcSemEvent_e    sbcSemEvent;
  regSbcAddr_t     data;
  regDataToSend_t  dataToSend;
  uint16_t         status;
  uint32_t         timeEntry;
}frameSbcSem_st;

/* list message to be managed   */
struct semMsgList{
  frameSbcSem_st    inMsg;
  struct semMsgList *next;
};

typedef struct semMsgList *nodeMsg; /*Define node as pointer of msg type struct LinkedList */

typedef __packed struct
{
  sbcSemStates_e      sbcSemStates;
  uint16_t            firstIdFree;
  uint16_t            dataVal;
  uint16_t            addrVal;
  uint16_t            logicIdSocket;
  uint32_t            discoveryMask;
  uint8_t*            pDataRd;
  uint16_t            scuInDwldIdx;
  uint32_t            activityStatus;
  uint16_t            unactiveSlave;
  uint16_t            pollingFlag;
  uint32_t            random;
  uint32_t            activeLastDiscovery;
  uint16_t            infoEepromSlaveToDo;
  uint8_t             addrModFree;
  uint8_t             offLine;
  uint8_t             sbcActive;
} sbcSemInfoMng_st;

typedef __packed struct
{
  pollingSlaveMng_e   pollStates;
  uint16_t            dataVal;
  uint16_t            addrVal;
  uint16_t            phyId;
  uint32_t            checkMask;
  uint8_t*            pDataRd;
  uint16_t            unactiveSlave;
  uint16_t            unreachableSlave;
  uint8_t             previousState[SCU_NUM];
  uint8_t             offLineCounter[SCU_NUM];
} pollingSlaveMng_st;

typedef enum
{
  REMOTE_INIT    = 0x0000,                  /* init state                                       */ 
  REMOTE_OPERATIVE,                         /* operative state                                  */ 
  REMOTE_EVS_AUTH,                          /* EVS_AUTH has been received                       */ 
  REMOTE_EVS_RELEASE,                       /* EVS_AUTH has been received                       */ 
  REMOTE_EVS_SUSPENDING,                    /* EVS_REMOTE_SUSPENDING has been sent              */ 
  REMOTE_EVS_WAIT_TO_STOP                   /* wait remote command to stop the charge           */ 
}remoteMng_e;

typedef enum
{
  REMOTE_TIMEOUT                = 0x00000,                 /* task timeout                     */ 
  REMOTE_EVS_AUTH_START         = EVS_AUTH_START,
  REMOTE_EVS_AUTH_STOP          = EVS_AUTH_STOP,
  REMOTE_EVS_REMOTE_SUSPENDING  = EVS_REMOTE_SUSPENDING,
  REMOTE_EVS_REMOTE_RELEASE     = EVS_REMOTE_RELEASE,
  REMOTE_MAX_POWER_CHANGE,      
  RETURN_TO_OPERATIVE      
}remoteMngEvent_e;


typedef __packed struct
{
  remoteMng_e         remoteStates;
  uint16_t            dataVal;
} remoteMng_st;

/* remote suspen / release message structure  */
typedef __packed struct
{
  remoteMngEvent_e currentEvent;
  uint16_t         currentData;
}frameRemote_st;



typedef enum
{
  RS485_SEM_IDLE              = 0x0000,       /* initial state                                          */ 
  RS485_SEM_OPERATIVE         = 0x0001,       /* normal condition                                       */ 
  RS485_SEM_MSG_TX            = 0x0002,       /* message ongong over RS485                              */ 
  RS485_SEM_MSG_WAITING       = 0x0003,       /* message waiting for RS485 busy                         */ 
  RS485_SEM_ACK_WAITING_WR    = 0x0004,       /* message waiting ACK for a previou WR  operation        */ 
  RS485_SEM_ACK_WAITING_RD    = 0x0005,       /* message waiting ACK for a previou RD operation         */ 
  RS485_SEM_DUMMY             = 0xFFFF
} rs485SemStates_e;

typedef __packed struct
{
  uint8_t     unitId;
  uint8_t     function;
  uint16_t    regAdd;
  uint16_t    numWords;
  uint8_t     numBytes;
} headerReqRWMR_st;



typedef __packed struct
{
  rs485SemStates_e      rs485SemStates;
  uint8_t*              pOriginData;
  headerReqRWMR_st      rs485RWmR;
  headerRHR_t           rs485Rd;   
  uint16_t              numRetry;   
} rs485SemInfoMng_st;

typedef __packed struct
{
  uint8_t     unitId;
  uint8_t     function;
  uint16_t    regAdd;
  uint16_t    numWords;
  uint8_t     numBytes;
  uint16_t    data[MODBUS_DATA_MAX_LEN];
} rs485RWmultipleReg_st;

/* socket presence in the chain   */
typedef __packed struct
{
  uint32_t    chainPresence;          // bit field 1 when a socket is present in that position
  uint32_t    livePresence;           // bit field 1 when a socket is present in that position run time
  uint32_t    keyPresence;            // a key for data structure validation
  uint32_t    activityStatus;         // bit field: 1 when a socket has been active in the last 5 sec (polling period)
  uint8_t     matrixConv[32];         // matrix conversion from RS485 address to Sem Idx connextor and viceversa 
  uint8_t     matrixIdConn[32];       // matrix conversion from RS485 address to id Connector (number on LCD) 
  uint32_t    assignedDeviceId;       // bit position for assigned device id 1 = device id assigned; 0 = device Id free
}socketPresence_t;

typedef enum    // possible value of wifi type --> reference in eeprom: LCD_TYPE_EADD bit 2
{
  HW_CHECK2_RCTE         = (uint16_t)(0x0001),
  HW_CHECK2_EM_EXT       = (uint16_t)(0x0002),
  HW_CHECK2_CHN2         = (uint16_t)(0x0004),
  HW_CHECK2_UNS0         = (uint16_t)(0x0008),
  HW_CHECK2_PEN          = (uint16_t)(0x0010),
  HW_CHECK2_HTEMP        = (uint16_t)(0x0020),
  HW_CHECK2_UNS1         = (uint16_t)(0x0040),
  HW_CHECK2_CHN2RES      = (uint16_t)(0x0080),
  HW_CHECK2_WIFI         = (uint16_t)(0x0100)
}hwCheck2_e;

typedef enum    // possible value of bit to notify a change
{
  UID_AUTHORIZATION_BIT         = (uint16_t)(0x0001),
  SOCKET_EVENT_FLAG_BIT         = (uint16_t)(0x0002),
  EVSE_CONNECTOR_STATUS_BIT_RO  = (uint16_t)(0x0004),
  EVSE_ERROR1_BIT_RO            = (uint16_t)(0x0008),
  EVSE_ERROR2_BIT_RO            = (uint16_t)(0x0010),
  SESSION_ID_BIT_RO             = (uint16_t)(0x0020),
  EVSE_BOOT_EVENT_BIT_RO        = (uint16_t)(0x0040),
  DISPLAY_DEF_LANGUAGE_BIT_RW   = (uint16_t)(0x0080),
  DISPLAY_LANGUAGES_BIT_RO      = (uint16_t)(0x0100),
  CONFIGURATIONS_BIT_RW         = (uint16_t)(0x0200)
}notifyBit_e;

typedef enum    // possible value of notification result
{
  SEND_NULL         = (uint8_t)(0x00),
  SEND_ACK,
  SEND_RETRY
}bitNotifyResult_e;

/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
void                  sbcSemGestTask                (void * pvParameters);
xQueueHandle          getSbcSemQueueHandle          (void);
void                  rs485SemGestTask              (void * pvParameters);
xQueueHandle          getRs485SemQueueHandle        (void);
uint8_t               txRS485Available              (void); 
void                  startSbcSemProcess            (void); 
void                  upgradeModbusHwConfig         (void); 
void                  upgradeModbusReg              (uint16_t errAddr);
void                  upgradeFwSlaveBroadcast       (uint8_t ixScu);
uint8_t               setInitState                  (uint8_t iniState);
void                  setNewMaxTempPowerSem         (void);
void                  setCurrentDateTimeInSem       (void);
GPIO_PinState         gpio0IoExpRead                (void);
void                  setAddressType                (uint8_t type, uint8_t clearPresence);
void                  HW_CHECKS_ACTUATORS_EEprom_Save (uint16_t rAddr);
void                  HW_PRESENCE_FLAG_EEprom_Save  (void);
void                  Print_Slave_FW_Version        (void);
void                  Print_Slave_Assigned          (void);
void                  resetCommandRemote            (void);
socketPresence_t*     getDefSocketInfoPtr           (void); 
uint32_t              getSocketDiscovered           (void); 
void                  setSocketDiscovered           (uint16_t maskPresence); 
void                  restoreFactoryDefaultForAll   (void); 
uint8_t               isModbusManagerActive         (void); 
uint8_t               getRemotePmFlag               (void); 
void                  restoreOperativeState         (void); 
void                  readEmValuesFromMaster        (void);
uint16_t              fromRs485ToSem                (uint16_t addrRS485);
void                  setPollingFlag                (uint8_t flagStatus); 
uint8_t               getIdNumberForLcd             (void); 
void                  setDevAlias                   (void);   
uint8_t               getNumSocketLcd               (uint8_t deviceId); 
uint8_t               readSocketPresence            (void); 
uint32_t              getPacketStatusNum            (void); 
uint8_t               allTaskAreOperative           (void);
uint8_t               getModbusAddrFromDevId        (uint8_t devId);
uint8_t               getAndsendAllSlaveParameters  (uint8_t idLogic);
//blockConfPar_st**     getPtrToConfParam             (void); 
void                  setPtrToConfParam             (uint8_t* ptr); 
uint32_t              getDiscoveryStatus            (void);
uint8_t               saveSlaveParameters           (void);
uint16_t              getInfoEepromSlaveStatus      (void); 
void                  setInfoEepromSlaveStatus      (uint16_t logicSlave); 


// --------------------------------------------------------------------------------------------------------------------------- //
#endif // _SBCSEM_H






/*************** END OF FILE ******************************************************************************************/




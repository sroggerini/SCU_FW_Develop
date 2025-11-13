/**
*   @file      handler_Modbus.h
*   @author    SCAME developers
*   @date      09 December 2021
*   @brief     Implements routines to handle Modbus protocol through SCU board
*   @note      
*
* @attention
*
*/

#ifndef HANDLER_MODBUS_H
#define HANDLER_MODBUS_H

/*******************************************************************************
* INCLUDE FILES:
*******************************************************************************/

#include "stdint.h"
#include "stdbool.h"
#include "metrology_usart.h"

/*******************************************************************************
* MACROS & CONSTANTS:
*****************************************************************************/

/* function code implemented for MODBUS */
#define   FUNCTION_READ_HOLDING_REG     ((uint8_t)3)
#define   FUNCTION_READ_INPUT_REG       ((uint8_t)4)
#define   FUNCTION_READ_EX_INPUT_REG    ((uint8_t)5)
#define   FUNCTION_WRITE_MULTIPLE_REG   ((uint8_t)16)
#define   FUNCTION_WRITE_SINGLE_REG     ((uint8_t)6)
#define   FUNCTION_ERROR_OCCURED        ((uint8_t)0x80)

#define   NBR_HEADER_BYTES_RD_RESPONSE            3
#define   NBR_HEADER_BYTES_WR_RESPONSE            6    

/* Commands implemented for calibration procedure */
#define   CALIBRATION_OFF_CMD                     0xA0    
#define   CALIBRATION_ON_CMD                      0xA1
#define   CALIBRATION_START_CMD                   0xA2      
/* Other commands */
#define   STORE_NEW_SN                            0x01
#define   RESET_SN                                0x02
#define   RESET_CALIB_FACTORS                     0x03
#define   RESET_ALL_NVM_DATA                      0x04
#define   STORE_CONFIGURATION                     0x05

/* Minimum number of bytes required to accept the query modbus: 
For Read function 0x04
   1 byte of Address 
   1 byte of Function
   2 bytes of Register address
   2 bytes for Number of words
   2 bytes of CRC
For Write function 0x10
   1 byte of Address 
   1 byte of Function
   2 bytes of Register address
   2 bytes for Number of words
   1 byte for Data counter
   next bytes are data to be written
*/
#define   MIN_NBR_OF_BYTES_FOR_QUERY_MODBUS       8    

/*******************************************************************************
* TYPES:
*******************************************************************************/

/* define error code for energy meters */
typedef enum
{
  EM_NO_ERROR     = 0,            /* no error                          */ 
  EM_REG_UNDEF,                   /* Register undefined                */ 
  EM_WRONG_ANSWER,                /* wrong answer                      */ 
  EM_NO_ANSWER                    /* EM not answering                  */ 
} emError_e;
 
#define LAST_ITEM                       ((uint16_t)0xFFFF)

// ALREADY DEFINED IN EM_TASK.h --> /* define EM type  */
// ALREADY DEFINED IN EM_TASK.h --> typedef enum
// ALREADY DEFINED IN EM_TASK.h --> {
// ALREADY DEFINED IN EM_TASK.h -->   EM_ALGO     = 0,                /* EM Gavazzi identifier             */ 
// ALREADY DEFINED IN EM_TASK.h -->   EM_GAVAZZI,                     /* EM Algodue identifier             */ 
// ALREADY DEFINED IN EM_TASK.h -->   EM_UNKNOW,                      /* EM unknow                         */ 
// ALREADY DEFINED IN EM_TASK.h -->   LAST_EM = EM_UNKNOW,            /* Last index reg                    */ 
// ALREADY DEFINED IN EM_TASK.h --> } emType_e;

/* define EM model  */
typedef enum
{
  EM_MONO_PH = 1,                 /* EM mono-phase AlgoDue identifier     */ 
  EM_THREE_PH = 3,                /* EM 3-phase AlgoDue identifier        */ 
} emPhase_e;

/* define for AlgoDue energy meter registers */

#define FIRST_ADDR_FOR_CONFIGURATION            ((uint16_t)0x0500)        /* First valid address for information registers   */
#define EC_AD_SN_REG                            ((uint16_t)0x0500)        /* Request to read Serial Number              */
#define EC_AD_MODEL_REG                         ((uint16_t)0x0505)        /* Request to read model                      */
#define EC_AD_BAUDERATE_REG                     ((uint16_t)0x0515)        /* Request to change baude rate               */
#define EC_EM_STATUS_REG                        ((uint16_t)0x0550)        /* Energy Meter status register               */
#define EC_COMMAND_REG                          ((uint16_t)0x0700)        /* Command register                           */
#define EC_REF_CALIB_MEAS_REG_V1                ((uint16_t)0x0702)        /* Reference measures for calibration process */
#define EC_REF_CALIB_MEAS_REG_L1                ((uint16_t)0x0704)        /* Reference measures for calibration process */
#define EC_REF_CALIB_MEAS_REG_V2                ((uint16_t)0x0706)        /* Reference measures for calibration process */
#define EC_REF_CALIB_MEAS_REG_L2                ((uint16_t)0x0708)        /* Reference measures for calibration process */
#define EC_REF_CALIB_MEAS_REG_V3                ((uint16_t)0x070A)        /* Reference measures for calibration process */
#define EC_REF_CALIB_MEAS_REG_L3                ((uint16_t)0x070C)        /* Reference measures for calibration process */
#define LAST_ADDR_FOR_CONFIGURATION             ((uint16_t)0x070C)        /* Last valid address for configuration registers   */

#define FIRST_ADDR_FOR_MEASURES                 ((uint16_t)0x0000)        /* First valid address for measure registers */
#define AD_VOLT_L1N_3PH_REG                     ((uint16_t)0x0000)        /* Voltage L1N for threephase                */
#define AD_VOLT_L2N_3PH_REG                     ((uint16_t)0x0002)        /* Voltage L2N for threephase                */
#define AD_VOLT_L3N_3PH_REG                     ((uint16_t)0x0004)        /* Voltage L3N for threephase                */
#define AD_SYS_VOLT_SIZE                        ((uint16_t)0x0003)        /* Size, in word, for voltage                */
#define AD_SYS_VOLT_1PH_REG                     ((uint16_t)0x000C)        /* Voltage when monophase                    */
#define AD_SYS_CURR_REG                         ((uint16_t)0x0016)        /* Current when monophase                    */
#define AD_CURR_L1_REG                          ((uint16_t)0x000E)        /* Current for phase L1                      */
#define AD_CURR_L2_REG                          ((uint16_t)0x0010)        /* Current for phase L2                      */
#define AD_CURR_L3_REG                          ((uint16_t)0x0012)        /* Current for phase L3                      */
#define AD_SYS_CURR_SIZE                        ((uint16_t)0x0003)        /* size, in word, for current                */
#define AD_COS_PHI_1PH_REG                      ((uint16_t)0x001B)        /* Request to read system cos PHI            */
#define AD_COS_PHI_3PH_REG                      ((uint16_t)0x001B)        /* Request to read system cos PHI trifase    */
#define AD_COS_PHI_SIZE                         ((uint16_t)0x0001)        /* size, in word, for cos Phi                */
#define AD_ACT_PWR_REG                          ((uint16_t)0x0025)        /* Request to read active power              */
#define AD_ACT_PWR_SIZE                         ((uint16_t)0x0004)        /* size, in word, for active power           */
#define AD_RACT_PWR_REG                         ((uint16_t)0x003D)        /* Request to read reactive power            */
#define AD_RACT_PWR_SIZE                        ((uint16_t)0x0004)        /* size, in word, for reactive power         */
#define AD_ACT_ENRG_REG                         ((uint16_t)0x0109)        /* Request to read active energy             */
#define AD_ACT_ENRG_SIZE                        ((uint16_t)0x0004)        /* size, in word, for active energy          */
#define AD_RACT_ENRG_REG                        ((uint16_t)0x0151)        /* Request to read reactive energy           */
#define AD_RACT_ENRG_SIZE                       ((uint16_t)0x0004)        /* size, in word, for reactive energy        */
#define AD_SACT_ENRG_REG                        ((uint16_t)0x0400)        /* Request to read session active energy     */
#define AD_SACT_ENRG_SIZE                       ((uint16_t)0x0004)        /* size, in word, for session active energy  */
#define AD_EXT_ACT_PWR_REG                      ((uint16_t)0x0025)        /* Request active power on ext em            */
#define AD_EXT_L1_ACT_PWR_REG                   ((uint16_t)0x001C)        /* Request to read active power on L1 phase  */
#define AD_EXT_L2_ACT_PWR_REG                   ((uint16_t)0x001F)        /* Request to read active power on L2 phase  */
#define AD_EXT_L3_ACT_PWR_REG                   ((uint16_t)0x0022)        /* Request to read active power on L3 phase  */
#define LAST_ADDR_FOR_MEASURES                  ((uint16_t)0x042A)        /* Last valid address valid for measure registers  */

#define EM_DEF_ADDR                             ((uint8_t)1)              /* Default address for Carlo Gavazzi / AlgoDUE  Meter               */
#define EM_DEF_EXT_ADDR                         ((uint8_t)2)              /* Default address for Carlo Gavazzi / AlgoDUE  external Meter      */

/* define identifier for all EM  */
typedef enum
{
  INTERNAL_EM     = 0,            /* Energy meter inside SCAME product                                */ 
  EXTERNAL_EM,                    /* Energy meter external (tipically in Power management enviroment) */ 
  EM_MAX_NUM
} emEnum_e;

/* define EM addresses  */
typedef enum
{
  EM_INT_ADDR     = EM_DEF_ADDR,      /* Internal Energy meter  address on RS485                          */ 
  EM_EXT_ADDR     = EM_DEF_EXT_ADDR   /* External Energy meter  address on RS485                          */ 
} emAddr_e;

/* define for EM map registers  */
typedef __packed struct
{
  uint16_t    regAdd;
  uint16_t    size;           /* number of word to be read/write for the specified reg address */  
  void*       value;          /* pointer to the value               */
  /* NOT USED IN ENERGY METER --> uint16_t    scale;  */        /* Scale of the data  */         
} emRegConfig_st;

/* define for EM map registers  */
// ALREADY DEFINED IN EM_TASK.h --> typedef __packed struct
// ALREADY DEFINED IN EM_TASK.h --> {
// ALREADY DEFINED IN EM_TASK.h -->   emType_e    type;           /* model val for a manufacturer  */
// ALREADY DEFINED IN EM_TASK.h -->   emPhase_e   emPhase;
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t    modelVal;
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     modelName[6];
// ALREADY DEFINED IN EM_TASK.h --> } emModelInfo_st;

/* Query Modbus received from the SCU board: these messages are received from the slave device (Energy Meter) */
/* These defines have been modified from the original one */

/* define for task request */
// ALREADY DEFINED IN EM_TASK.h --> typedef enum
// ALREADY DEFINED IN EM_TASK.h --> {
// ALREADY DEFINED IN EM_TASK.h -->   TX_TO_EM      = 0x00,      /* Request to send a msg to Energy Meter     */ 
// ALREADY DEFINED IN EM_TASK.h -->   RX_FROM_EM    = 0x01,      /* Request to take a msg from Energy Meter   */ 
// ALREADY DEFINED IN EM_TASK.h -->   EM_ERROR      = 0xFFFF     /* Error on uart   */ 
// ALREADY DEFINED IN EM_TASK.h --> } emMsgDir_e;


// ALREADY DEFINED IN EM_TASK.h --> typedef __packed struct
// ALREADY DEFINED IN EM_TASK.h --> {
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     unitId;
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     function;
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t    regAdd;
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t    value;
// ALREADY DEFINED IN EM_TASK.h --> } nodeRWsingleReg_st;

// ALREADY DEFINED IN EM_TASK.h --> typedef __packed struct
// ALREADY DEFINED IN EM_TASK.h --> {
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     unitId;
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     function;
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t    regAdd;
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t    numWords;
// ALREADY DEFINED IN EM_TASK.h --> } headerRIR_t;

// ALREADY DEFINED IN EM_TASK.h --> typedef __packed struct
// ALREADY DEFINED IN EM_TASK.h --> {
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     unitId;
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     function;
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t    regAdd;
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t    numBytes;
// ALREADY DEFINED IN EM_TASK.h --> } headerRHR_t;
 
// ALREADY DEFINED IN EM_TASK.h --> typedef __packed struct
// ALREADY DEFINED IN EM_TASK.h --> {
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     unitId;
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     function;
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t    regAdd;
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t    numWords;
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     numBytes;   /* Number of bytes in case of function 16 is 1 byte lenght */
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t    data[EM_MSG_MAX_LEN];
// ALREADY DEFINED IN EM_TASK.h --> } nodeRWmultipleReg_st;

// ALREADY DEFINED IN EM_TASK.h --> typedef __packed struct
// ALREADY DEFINED IN EM_TASK.h --> {
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     unitId;
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     function;
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t    regAdd;
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t    data;
// ALREADY DEFINED IN EM_TASK.h --> } nodeWsingleReg_st;

// ALREADY DEFINED IN EM_TASK.h --> typedef __packed union
// ALREADY DEFINED IN EM_TASK.h --> {
// ALREADY DEFINED IN EM_TASK.h -->   nodeRWsingleReg_st    nodeRWsingleReg;
// ALREADY DEFINED IN EM_TASK.h -->   headerRIR_t           nodeReadInputReg;
// ALREADY DEFINED IN EM_TASK.h -->   headerRHR_t           nodeReadHoldingReg;
// ALREADY DEFINED IN EM_TASK.h -->   nodeRWmultipleReg_st  nodeRWmultipleReg;
// ALREADY DEFINED IN EM_TASK.h -->   nodeWsingleReg_st     nodeWsingleReg;
// ALREADY DEFINED IN EM_TASK.h --> } messageRx_u;

// ALREADY DEFINED IN EM_TASK.h --> typedef __packed struct
// ALREADY DEFINED IN EM_TASK.h --> {
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t            totalLen;
// ALREADY DEFINED IN EM_TASK.h -->   messageRx_u         messageRx;   
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t            crc;
// ALREADY DEFINED IN EM_TASK.h -->   emMsgDir_e          emMsgDir;
// ALREADY DEFINED IN EM_TASK.h -->   emModelInfo_st      emModelInfo[EM_MAX_NUM];
// ALREADY DEFINED IN EM_TASK.h -->   bool                emMsgReady;
// ALREADY DEFINED IN EM_TASK.h --> } frameEm_st;

/* Answer to a Modbus query message type: this is transmitted from the slave device (Energy Meter) */
/* The name has been modified from the original */

/* define for answer modbus messages */

// ALREADY DEFINED IN EM_TASK.h --> typedef __packed struct
// ALREADY DEFINED IN EM_TASK.h --> {
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     unitId;
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     function;
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t    regAdd;
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t    numWords;
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t    crc;
// ALREADY DEFINED IN EM_TASK.h --> } nodeAnswWRmReg_st;

// ALREADY DEFINED IN EM_TASK.h --> typedef __packed struct
// ALREADY DEFINED IN EM_TASK.h --> {
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     unitId;
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     function;
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     numBytes;
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     data[EM_DATA_MAX_LEN];
// ALREADY DEFINED IN EM_TASK.h --> } headerAnswRIR_t;

// ALREADY DEFINED IN EM_TASK.h --> typedef __packed struct
// ALREADY DEFINED IN EM_TASK.h --> {
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     unitId;
// ALREADY DEFINED IN EM_TASK.h -->   uint8_t     function;
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t    regAdd;
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t    data;
// ALREADY DEFINED IN EM_TASK.h -->   uint16_t    crc;
// ALREADY DEFINED IN EM_TASK.h --> } nodeAnswWRsReg_st;

// ALREADY DEFINED IN EM_TASK.h --> typedef __packed union
// ALREADY DEFINED IN EM_TASK.h --> {
// ALREADY DEFINED IN EM_TASK.h -->   headerAnswRIR_t       nodeReadAnswInputReg;
// ALREADY DEFINED IN EM_TASK.h -->   nodeAnswWRmReg_st     nodeAnswWMmReg;
// ALREADY DEFINED IN EM_TASK.h -->   nodeAnswWRsReg_st     nodeAnswWRsReg;
// ALREADY DEFINED IN EM_TASK.h --> } messageTx_u;

typedef __packed struct
{
  uint16_t            totalLen;
  messageTx_u         messageTx; 
} frameTxEm_st;                         /* ex frameRxEm_st */


/* Exception type definitions */
#define ILLEGAL_FUNCTION_EXC            1
#define ILLEGAL_DATA_ADDRESS_EXC        2
#define ILLEGAL_DATA_VALUE_EXC          3
#define ILLEGAL_RESPONSE_LENGHT_EXC     4

/***************************************************************************************************/

typedef __packed union
{
  uint16_t  crcW;
  uint8_t   crcA[2];
  __packed struct 
  {
    uint8_t crcL;
    uint8_t crcH;
  }crcLH_st;
}crcMode_u;

typedef __packed union
{
  uint32_t  dataDW;
  uint8_t   dataD[4];
}dataAsDw_u;

/* remember endian in ST micro:       */
/* uint64_t --> 0x0103050702040608    */
/* address + 0  --> dataQ[0] = 08     */
/* address + 1  --> dataQ[1] = 06     */
/* address + 2  --> dataQ[2] = 04     */
/* address + 3  --> dataQ[3] = 02     */
/* address + 4  --> dataQ[4] = 07     */
/* address + 5  --> dataQ[5] = 05     */
/* address + 6  --> dataQ[6] = 03     */
/* address + 7  --> dataQ[7] = 01     */
typedef __packed union
{
  uint64_t  dataQW;
  uint8_t   dataQ[8];
}dataAsQw_u;

typedef __packed struct
{
  uint32_t    SysVoltage;
  int32_t     currentL1;
  int32_t     currentL2;
  int32_t     currentL3;
  uint32_t    neutralCurrent;
  int32_t     currentL;
  int16_t     cosPh1;
  int16_t     cosPh2;
  int16_t     cosPh3;
  int16_t     cosPh;
}emAlgoBk1_st;

typedef __packed struct
{
  uint16_t    activePower[3];
  int16_t     spare2[21];
  uint16_t    reactivePower[3];
}emAlgoBk2_st;


typedef __packed struct
{
  emAlgoBk1_st    emAlgoBk1;
  emAlgoBk2_st    emAlgoBk2;
  uint16_t        extActiveEnergy[3];
}emAlgoUserReg_st;

/* define states for the task  */
typedef enum
{
  STATE_IDLE     = 0,             /* initial state                          */ 
  EM_POLLING,                     /* opeartive EM, polling state            */ 
  EM_FAST_RESTORE,                /* fast operation after V230 come back    */ 
  EM_STOP_FOR_V230KO              /* stop operation after V230 goes off     */ 
} emStates_e;

typedef __packed struct
{
  int32_t   startActEnrgValue;
  int32_t   currentActEnrgValue;
  int32_t   sessionEnrgValue;
}sessActEnergy_t;


/*******************************************************************************
* GLOBAL VARIABLES:
*******************************************************************************/


/*******************************************************************************
* FUNCTION EXPORTED
*******************************************************************************/

void SCU_Modbus_Handler(void);    

emError_e getEmRegister(uint16_t regAddr, uint8_t size, uint8_t* pReadBuff, uint8_t emId, uint8_t flagRegCheck);
emError_e setEmRegister(uint16_t regAddr, uint16_t data, uint8_t emId, uint8_t flagRegCheck);

uint32_t __REV32 (uint32_t Value);

#endif /* HANDLER_MODBUS_H */

/******************************************************************************
* VARIABLE EXPORTED 
*******************************************************************************/

extern frameEm_st       emMsgRx;
extern frameTxEm_st     emMsgTx;

/**
  * @}
  */

/* End Of File */

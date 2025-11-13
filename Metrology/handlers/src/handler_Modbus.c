/**
*   @file      handler_Modbus.c
*   @author    SCAME developers
*   @date      09 December 2021
*   @brief     Implements routines to handle Modbus protocol through SCU board
*   @note      
*
* @attention
*
*/

/*******************************************************************************
* INCLUDE FILES:
*******************************************************************************/
#include <string.h>
#include <stdlib.h>
#include "metroTask.h"
#include "Handler_Modbus.h"
#include "handler_metrology.h"
#include "metrology_hal.h"
#include "handler_nvram.h"
#include "main.h"

/** @addtogroup GENERIC
  * @{
  */

/*******************************************************************************
* TYPES:
*******************************************************************************/

/*******************************************************************************
* CONSTANTS & MACROS:
*******************************************************************************/

/*                              Modbus map definition                   */

/* See reference Modbus.pdf related to Algo 2 device                    */

/* Information on Energy Counter and Communication module               */

static const emRegConfig_st   emRegister_Config[ ] __attribute__((aligned(4)))   = {
  
   /*****  ALGO DUE default  info register structure ***************/
   
   /* Reg Addr                   Size(word)         Value                        */
  {EC_AD_SN_REG,                 6U,                &em_Config.EC_Serial_Number, },     // @ 0x500     
  {EC_AD_MODEL_REG,              2U,                &em_Config.EC_Model,         },     // @ 0x505
  {EC_AD_BAUDERATE_REG,          1U,                &em_Config.Config,           },     // @ 0x515
  {EC_EM_STATUS_REG,             1U,                &em_Status.Value,            },     // @ 0x550 
  {EC_COMMAND_REG,               1U,                &em_Config.Command,          },     // @ 0x700
  {EC_REF_CALIB_MEAS_REG_V1,     2U,                &metroData.Calib.Vrms_Ref    },     // @ 0x702
  {EC_REF_CALIB_MEAS_REG_L1,     2U,                &metroData.Calib.Irms_Ref    },     // @ 0x704
  
 };

/* Calculate the number of registers inside the table */
const uint8_t EM_CONFIG_REG_nbr = (sizeof(emRegister_Config) / sizeof(emRegConfig_st));

/* Realtime Measures and Total counters                   */

// EM_MEASURES_REG_NUM

static const emRegConfig_st   emRegister_Measures[ ] __attribute__((aligned(4)))   = {
  
   /*****  ALGO DUE default  register structure ***************/
  
   /*  Reg Addr                 Size                    Value                                 Visible address   */
   {AD_VOLT_L1N_3PH_REG,        AD_SYS_VOLT_SIZE,       &em_Measures.Voltage_L1N,         },      // @ 0x00
   {AD_VOLT_L2N_3PH_REG,        AD_SYS_VOLT_SIZE,       &em_Measures.Voltage_L2N,         },      // @ 0x02
   {AD_VOLT_L3N_3PH_REG,        AD_SYS_VOLT_SIZE,       &em_Measures.Voltage_L3N,         },      // @ 0x04
   {AD_SYS_VOLT_1PH_REG,        AD_SYS_VOLT_SIZE,       &em_Measures.SystemVoltage,       },      // @ 0x0C  
   {AD_CURR_L1_REG,             AD_SYS_CURR_SIZE,       &em_Measures.Current_L1,          },      // @ 0x0E
   {AD_CURR_L2_REG,             AD_SYS_CURR_SIZE,       &em_Measures.Current_L2,          },      // @ 0x10
   {AD_CURR_L3_REG,             AD_SYS_CURR_SIZE,       &em_Measures.Current_L3,          },      // @ 0x12
   {AD_SYS_CURR_REG,            AD_SYS_CURR_SIZE,       &em_Measures.SystemCurrent,       },      // @ 0x16
   {AD_COS_PHI_1PH_REG,         AD_COS_PHI_SIZE,        &em_Measures.CosPhi_1PH,          },      // @ 0x1B
   {AD_COS_PHI_3PH_REG,         AD_COS_PHI_SIZE,        &em_Measures.CosPhi_3PH,          },      // @ 0x1B
   {AD_EXT_L1_ACT_PWR_REG,      AD_ACT_PWR_SIZE,        &em_Measures.Active_Pwr_L1,       },      // @ 0x1C
   {AD_EXT_L2_ACT_PWR_REG,      AD_ACT_PWR_SIZE,        &em_Measures.Active_Pwr_L2,       },      // @ 0x1F
   {AD_EXT_L3_ACT_PWR_REG,      AD_ACT_PWR_SIZE,        &em_Measures.Active_Pwr_L3,       },      // @ 0x22
   {AD_ACT_PWR_REG,             AD_ACT_PWR_SIZE,        &em_Measures.Active_Pwr_TOT,      },      // @ 0x25
   {AD_RACT_PWR_REG,            AD_RACT_PWR_SIZE,       &em_Measures.Reactive_Pwr_TOT,    },      // @ 0x3D
   {AD_ACT_ENRG_REG,            AD_ACT_ENRG_SIZE,       &em_Measures.Active_Energy_TOT,   },      // @ 0x109
   {AD_RACT_ENRG_REG,           AD_RACT_ENRG_SIZE,      &em_Measures.Reactive_Energy_TOT, },      // @ 0x151
   {AD_SACT_ENRG_REG,           AD_SACT_ENRG_SIZE,      &em_Measures.Sys_Active_Energy,   },      // @ 0x400
   
 };

/* Calculate the number of registers inside the table */
const uint8_t EM_MEASURE_REG_nbr = (sizeof(emRegister_Measures) / sizeof(emRegConfig_st));

/* Table of CRC values for high–order byte */
static const unsigned char auchCRCHi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low–order byte */
static const char auchCRCLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 
0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 
0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 
0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 
0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 
0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 
0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 
0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 
0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 
0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 
0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 
0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 
0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 
0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

/*******************************************************************************
* LOCAL DEFINES:
*******************************************************************************/

/*******************************************************************************
* GLOBAL VARIABLES:
*******************************************************************************/

/*******************************************************************************
* LOCAL VARIABLES:
*******************************************************************************/

/*******************************************************************************
* LOCAL FUNCTION PROTOTYPES:
*******************************************************************************/

extern UART_HandleTypeDef huart2;
extern const nvmLeg_t metroDefaultNvm;

/*******************************************************************************
*
*                       IMPLEMENTATION: Public functions
*
*******************************************************************************/

/**
* @brief                __REV32
* This function swap the HIGH/LOW 16bit part inside a 32bit variable
*       | HIGH16 | LOW16 | swapped to --> | LOW16 | HIGH16 |
* @param  uint32_t value to swap
* @retval uint32_t value swapped
*/

uint32_t __REV32 (uint32_t Value)
{
  
  uint32_t SwappedVal;
  
  SwappedVal = (Value & 0xFFFF0000) >> 16;
  SwappedVal |= (Value & 0x0000FFFF) << 16;   
    
  return SwappedVal;
}

/******************************************************************************
*
* @brief        Get the crc value in accordind to:
*               POLY => calculation polynomial of the CRC 16 = 1010 0000 0000 0001 1
*               (Generating polynomial = 1 + x2 + x 15 + x 16)
*  
* @param [in]   uint8_t*: point to start buffer
* @param [in]   len     : number of bytes involved in the crc
*
* @retval       uint8_t: the crc evaluate (mod 256)
*
*******************************************************************************/

uint16_t  crcEvaluation(uint8_t* puchMsg, uint16_t usDataLen)
{
  unsigned char uchCRCHi;   /* high byte of CRC initialized */
  unsigned char uchCRCLo;   /* low byte of CRC initialized */
  unsigned int  uIndex;          /* will index into CRC lookup table */

  uchCRCHi = 0xFF;
  uchCRCLo = 0xFF;
  while (usDataLen--) /* pass through message buffer */
  {
    uIndex = uchCRCLo ^ *(puchMsg++); /* calculate the CRC */
    uchCRCLo = (unsigned char)(uchCRCHi ^ (unsigned char)auchCRCHi[uIndex]);
    uchCRCHi = (unsigned char)auchCRCLo[uIndex] ;
  }
    return ((unsigned short)(uchCRCHi << 8 | uchCRCLo));
}

/**
* @brief                SCU Modbus Handler
* This function handle the Modbus communication with the SCU
* The EM is configured as SLAVE
* @param  Register address to find
* @retval Pointer to the location inside the table where the register is.
*/

uint16_t * EM_Check_Addr_In_Register_Table(int16_t RegAddr)
{
  
  uint8_t  cnt;
  uint16_t *pTable;  
  
  /* Check if is a MEASURE request */
  if ((RegAddr >= FIRST_ADDR_FOR_MEASURES) &&  (RegAddr <= LAST_ADDR_FOR_MEASURES))
  {
    /* Point to the Table */
    pTable = (uint16_t *)&emRegister_Measures[0];
    /* Find the address inside the table */
    for (cnt = 0; cnt < EM_MEASURE_REG_nbr; cnt++, pTable += sizeof(emRegConfig_st)/sizeof(uint16_t))
    {
      /* Find the register */
      if (*pTable == RegAddr)
        return pTable;
    }   
  }
  /* Check if is an INFO or CONFIGURATION request */
  if ((RegAddr >= FIRST_ADDR_FOR_CONFIGURATION) &&  (RegAddr <= LAST_ADDR_FOR_CONFIGURATION))
  {
    /* Point to the Table */
    pTable = (uint16_t *)&emRegister_Config[0];
    /* Find the address inside the table */
    for (cnt = 0; cnt < EM_CONFIG_REG_nbr; cnt++, pTable += sizeof(emRegConfig_st)/sizeof(uint16_t))
    {
      /* Find the register */
      if (*pTable == RegAddr)
        return pTable;
    }   
  }
  else
    return NULL;
    
  /* NOT A VALID register address */
  return NULL;
    
}

/**
* @brief                SCU Modbus Handler
* This function handle the Modbus communication with the SCU
* The EM is configured as SLAVE
* @param  None
* @retval None
*/

void SCU_Modbus_Handler(void)
{

    crcMode_u     crc;
    uint8_t*      pEnd;
    uint8_t*      ptr;
    uint8_t*      pRegValue8;
    uint16_t*     pRegValue16;
    uint16_t*     pRegTable;
    
    uint8_t       ModbusFunc;
    uint8_t       cnt;
    uint8_t       NumBytes;
    uint8_t       NumWords;
    uint16_t      RegAddr;
    uint16_t      Temp16;

    uint32_t      tmp_addr;
    
    frameEm_st*   pFrameRx;       
    
    /* Message received and ready for parsing? */
    if (emMsgRx.emMsgReady)
    {   
      emMsgRx.emMsgReady = false;
      /* Point to the frame received */
      pFrameRx = (frameEm_st*)&emMsgRx;
      /* Get Modbus function of the query */
      ModbusFunc = pFrameRx->messageRx.nodeRWsingleReg.function;       
      /* Check which type of message should be managed */
      switch (ModbusFunc)
      {
        
        case FUNCTION_READ_INPUT_REG:
          /* Point to the CRC received */
          pEnd  = (uint8_t*)((uint32_t)&pFrameRx->messageRx.nodeReadInputReg + sizeof(headerRIR_t));
          /* calculate CRC of the received message */
          crc.crcW = crcEvaluation ((uint8_t*)&pFrameRx->messageRx, pFrameRx->totalLen - sizeof(crc.crcW));
          /* CRC match ? */
          if ((crc.crcLH_st.crcL == *pEnd) && (crc.crcLH_st.crcH == *(pEnd + 1)))
          {
            /* Get the number of words */
            NumWords = __REV16(pFrameRx->messageRx.nodeReadInputReg.numWords);
            /* Get number of bytes to read */
            NumBytes = NumWords * 2;
            /* Get register address */
            RegAddr = __REV16(pFrameRx->messageRx.nodeReadInputReg.regAdd);  
            /* Message is OK, locate now the register inside the modbus map table */
            pRegTable = EM_Check_Addr_In_Register_Table(RegAddr);
            /* Register address found? */
            if (pRegTable != NULL)
            {
              /* Prepare the frame to transmit */              
              emMsgTx.messageTx.nodeReadAnswInputReg.unitId = em_Config.SlaveAddr;
              emMsgTx.messageTx.nodeReadAnswInputReg.function = FUNCTION_READ_INPUT_REG;  
              emMsgTx.messageTx.nodeReadAnswInputReg.numBytes = NumBytes;
              /* Point to the Table */
              pRegValue16 = (uint16_t *)((emRegConfig_st *)pRegTable)->value;
              /* Point to the beginning of the data to fill */
              ptr = (uint8_t *)&emMsgTx.messageTx.nodeReadAnswInputReg.data[0];

              /* Fill the data inside the query */
              for (cnt = 0; cnt < NumWords; cnt++)
              { 
                /* Fill the data, first the HIGH part and then the LOW */
                Temp16 = *pRegValue16++;
                /* HIGH part */
                *ptr++ = (Temp16 & 0xFF00) >> 8;
                /* LOW part */
                *ptr++ = Temp16 & 0x00FF;
              }
                       
              /* Calc the lenght of the message */
              emMsgTx.totalLen = NBR_HEADER_BYTES_RD_RESPONSE + emMsgTx.messageTx.nodeReadAnswInputReg.numBytes;
              /* Calculate CRC and append it to the message */
              crc.crcW = crcEvaluation ((uint8_t*)&emMsgTx.messageTx.nodeReadAnswInputReg.unitId, emMsgTx.totalLen);               
              /* Put CRC L inside the query first (ref. Modbus Protocol) */
              *ptr++ = crc.crcLH_st.crcL;
              /* Put CRC H inside the query as the last byte (ref. Modbus Protocol) */
              *ptr = crc.crcLH_st.crcH;
            }
            else
            {
              /* GIVE AN EXCEPTION */
              /* Prepare the frame to transmit */              
              emMsgTx.messageTx.nodeReadAnswInputReg.unitId = em_Config.SlaveAddr;
              /* Adjust the function type to match the requirements */
              emMsgTx.messageTx.nodeReadAnswInputReg.function |= 0x80;
              /* Set exception type */
              emMsgTx.messageTx.nodeReadAnswInputReg.numBytes = ILLEGAL_DATA_ADDRESS_EXC;
              /* Set the lenght of the message */
              emMsgTx.totalLen = NBR_HEADER_BYTES_RD_RESPONSE;
              /* Calculate CRC and append it to the message */
              crc.crcW = crcEvaluation ((uint8_t*)&emMsgTx.messageTx.nodeReadAnswInputReg.unitId, emMsgTx.totalLen);               
              /* Attach the CRC, LOW part first (ref. Modbus Protocol) */
              emMsgTx.messageTx.nodeReadAnswInputReg.data[0] = crc.crcLH_st.crcL;
              /* and then the HIGH part (ref. Modbus Protocol) */
              emMsgTx.messageTx.nodeReadAnswInputReg.data[1] = crc.crcLH_st.crcH;              
            }
            /* Send the response */
#ifdef GD32F3xx
            /* Since in GD32F3xx device the USART doesn't have the RS485 functionality, we need to manage 
               the DE pin manually. */
            HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
            /* Clear status flags of DMA tx */
            UART_EM_DMA->IFCR = DMA_IFCR_CGIF7 | DMA_IFCR_CTCIF7 | 
                                DMA_IFCR_CHTIF7 | DMA_IFCR_CTEIF7;              
#else
            /* Clear status flags of DMA tx */
            UART_EM_DMA->IFCR = DMA_IFCR_CGIF3 | DMA_IFCR_CTCIF3 | 
                                DMA_IFCR_CHTIF3 | DMA_IFCR_CTEIF3;                          
#endif            
            /* Enable Transfer complete interrupt */
            SET_BIT(huart2.Instance->CR1, USART_CR1_TCIE);              
            HAL_UART_Transmit_DMA(&huart2, &emMsgTx.messageTx.nodeReadAnswInputReg.unitId, emMsgTx.totalLen + sizeof(crc.crcW));
          }                      
          break;         
          
        case FUNCTION_WRITE_MULTIPLE_REG:
          /* Check if the message is right, evaluate CRC value */
          /* Point to the CRC received */
          pEnd  = (uint8_t*)((uint32_t)&pFrameRx->messageRx.nodeRWmultipleReg.data + (uint32_t)pFrameRx->messageRx.nodeRWmultipleReg.numBytes);
          /* calculate CRC of the received message */
          crc.crcW = crcEvaluation ((uint8_t*)&pFrameRx->messageRx, pFrameRx->totalLen - sizeof(crc.crcW));
          /* CRC match ? */
          if ((crc.crcLH_st.crcL == *pEnd) && (crc.crcLH_st.crcH == *(pEnd + 1)))
          {
            /* Parse the message received:              */
            /* Get register address                     */
            RegAddr = __REV16(pFrameRx->messageRx.nodeRWmultipleReg.regAdd);
            /* Get number of bytes to be written */
            NumWords = __REV16(pFrameRx->messageRx.nodeRWmultipleReg.numWords);
            /* Message is OK, locate now the register inside the modbus map table */
            pRegTable = EM_Check_Addr_In_Register_Table(RegAddr); 
            /* Register found */
            if (pRegTable != NULL)
            {
              /* Point to the local variables, the address is inside the local dictionary */
              pRegValue8 = (uint8_t *)((emRegConfig_st *)pRegTable)->value;
              /* Pointer to the received data */
              ptr = (uint8_t *)&pFrameRx->messageRx.nodeRWmultipleReg.data;
              /* Copy the received data into the local variables */
              for (cnt = 0; cnt < NumWords; cnt++, ptr += 2, pRegValue8 += 2)
              {
                /* Fit the right endianness order, HIGH part and then the LOW */
                *(pRegValue8 + 1) = *ptr;
                * pRegValue8 = *(ptr + 1);
              }            
              /* If received frame is at address 0x700, consider it as COMMAND */
              if (RegAddr == EC_COMMAND_REG)
              {
                /* Parse the command received */
                switch (em_Config.Command)
                {
                  case STORE_NEW_SN:                    /* 0x01 */
                      /* Store SN value */
                      memcpy(metroData.nvm->SerialNumber, em_Config.EC_Serial_Number, sizeof (em_Config.EC_Serial_Number));
                      /* Store the new SN value inside the FLASH */
                      NVM_Write(NVM_SAVE_LEG);
                    break;
                  case RESET_SN:                        /* 0x02 */
                    /* Reset SN values */
                      memcpy(em_Config.EC_Serial_Number, DEFAULT_SN, sizeof (em_Config.EC_Serial_Number));
                      memcpy(metroData.nvm->SerialNumber, em_Config.EC_Serial_Number, sizeof (em_Config.EC_Serial_Number));
                      /* Store the new SN value inside the FLASH */
                      NVM_Write(NVM_SAVE_LEG);                    
                    break;
                  case RESET_CALIB_FACTORS:             /* 0x03 */                                   
                      /* Mark not calibration status */
                      metroData.nvm->CalibStatus = DEVICE_NOT_CALIBRATED;
                      /* Mark as device not calibrated */
                      em_Status.Flags.Calibrated = false;                
                      /* Restore default calibration values */
                      metroData.nvm->V_CalibFact[METRO_PHASE_1] = metroData.nvm->I_CalibFact[METRO_PHASE_1] = 0x800;
                      metroData.nvm->data1[4] &= 0xFFFFF000; 
                      metroData.nvm->data1[4] |= metroData.nvm->V_CalibFact[METRO_PHASE_1];  /* dsp_cr5 = Voltage adjustment */    
                      metroData.nvm->data1[5] &= 0xFFFFF000;
                      metroData.nvm->data1[5] |= metroData.nvm->I_CalibFact[METRO_PHASE_1];  /* dsp_cr6 = Current adjustment */    
                      /* Set the device register */
                      /* Now send data to the external chip */
                      /* the address should be provided in 2 bytes format (16 bits by 16 bits) for STPM */
                      tmp_addr = STPM_DSPCTRL5;
                      /* Now Write the 2 U32 registers inside external chip for DSPCR5 to 7  */
                      Metro_HAL_Stpm_write(EXT1, (uint8_t*)&tmp_addr, 2, (uint32_t *)&metroData.nvm->data1[4], STPM_WAIT);                      
                      /* 3P system? */
                      if (metroData.nbPhase > 1)
                      {
                        /* Restore default calibration values for EXT2 device CHANNEL 1 (L2) */
                        metroData.nvm->V_CalibFact[METRO_PHASE_2] = metroData.nvm->I_CalibFact[METRO_PHASE_2] = 0x800;
                        metroData.nvm->data2[4] &= 0xFFFFF000; 
                        metroData.nvm->data2[4] |= metroData.nvm->V_CalibFact[METRO_PHASE_2];  /* dsp_cr5 = Voltage adjustment */    
                        metroData.nvm->data2[5] &= 0xFFFFF000;
                        metroData.nvm->data2[5] |= metroData.nvm->I_CalibFact[METRO_PHASE_2];  /* dsp_cr6 = Current adjustment */    
                        /* Restore default calibration values for EXT2 device CHANNEL 2 (L3) */
                        metroData.nvm->V_CalibFact[METRO_PHASE_3] = metroData.nvm->I_CalibFact[METRO_PHASE_3] = 0x800;
                        metroData.nvm->data2[6] &= 0xFFFFF000; 
                        metroData.nvm->data2[6] |= metroData.nvm->V_CalibFact[METRO_PHASE_3];  /* dsp_cr7 = Voltage adjustment */    
                        metroData.nvm->data2[7] &= 0xFFFFF000;
                        metroData.nvm->data2[7] |= metroData.nvm->I_CalibFact[METRO_PHASE_3];  /* dsp_cr8 = Current adjustment */    
                        /* Set the device register */
                        /* Now send data to the external chip */
                        /* the address should be provided in 2 bytes format (16 bits by 16 bits) for STPM */
                        tmp_addr = STPM_DSPCTRL5;
                        /* Now Write the 2 U32 registers inside external chip for DSPCR5 to 9  */
                        Metro_HAL_Stpm_write(EXT2, (uint8_t*)&tmp_addr, 4, (uint32_t *)&metroData.nvm->data2[4], STPM_WAIT);                                              
                      }
                      /* Store the new values inside the flash */
                      NVM_Write(NVM_SAVE_LEG);                   
                    break;
                  case RESET_ALL_NVM_DATA:              /* 0x04 */                                   
                      /* Restore default nvm data */
                      memcpy(metroData.nvm, &metroDefaultNvm, sizeof(nvmLeg_t));
                      /* Store the new values inside the flash */
                      NVM_Write(NVM_SAVE_LEG);                   
                    break;
#ifdef STORE_CONFIG                      
                  case STORE_CONFIGURATION:             /* 0x05 */
                      /* Store Configuration value */
                      memcpy(&metroData.nvm->Config, &em_Config.Config, sizeof (em_Config.Config));
                      /* Store config value inside the flash */
                      NVM_Write(NVM_SAVE_LEG);                   
                      /* Set TA type information */
                      EM_TA_type = ((em_Config.Config & ~EM_TA_CONFIG_MASK) >> EM_TA_CONFIG_BIT_POS) == 0 ? EM_TA_TYPE_SBT002 : EM_TA_TYPE_SDL007;
                    break;
#endif                    
                  case CALIBRATION_ON_CMD:              /* 0xA1 */
                      /* Calibration entry */
                      metroData.Calib.Flags.bit.ON = true;
                    break;
                  case CALIBRATION_START_CMD:           /* 0xA2 */
                      /* Calibration start */
                      metroData.Calib.Flags.bit.Start = true;
                    break;
                  case CALIBRATION_OFF_CMD:             /* 0xA0 */
                      /* Calibration exit */
                      metroData.Calib.Flags.bit.ON = false;
                    break;
                  default:
                    break;
                }              
              }
              /* Send the response, respect the order HI-LO, except for the CRC */
              emMsgTx.totalLen = NBR_HEADER_BYTES_WR_RESPONSE;
              emMsgTx.messageTx.nodeAnswWMmReg.unitId = em_Config.SlaveAddr;
              emMsgTx.messageTx.nodeAnswWMmReg.function = FUNCTION_WRITE_MULTIPLE_REG;  
              emMsgTx.messageTx.nodeAnswWMmReg.regAdd = __REV16(RegAddr);
              emMsgTx.messageTx.nodeAnswWMmReg.numWords = __REV16(NumWords);
              emMsgTx.messageTx.nodeAnswWMmReg.crc = crcEvaluation ((uint8_t*)&emMsgTx.messageTx.nodeAnswWMmReg.unitId, emMsgTx.totalLen);
#ifdef GD32F3xx
              /* Since in GD32F3xx device the USART doesn't have the RS485 functionality, we need to manage 
               the DE pin manually. */
              HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
              /* Clear status flags of DMA tx */
              UART_EM_DMA->IFCR = DMA_IFCR_CGIF7 | DMA_IFCR_CTCIF7 | 
                                  DMA_IFCR_CHTIF7 | DMA_IFCR_CTEIF7;              
#else              
              /* Clear status flags of DMA tx */
              UART_EM_DMA->IFCR = DMA_IFCR_CGIF3 | DMA_IFCR_CTCIF3 | 
                                  DMA_IFCR_CHTIF3 | DMA_IFCR_CTEIF3;              
#endif            
              /* Enable Transfer complete interrupt */
              SET_BIT(huart2.Instance->CR1, USART_CR1_TCIE);              
              HAL_UART_Transmit_DMA(&huart2, &emMsgTx.messageTx.nodeAnswWMmReg.unitId, emMsgTx.totalLen + sizeof(crc.crcW));            
            }
          }             
          break;
          
        default:
          
          break;
          
      }       
      
    }
    // PLACE HERE THE CODE FOR MODBUS HANDLER
}

/**
  * @}
  */

/* End Of File */

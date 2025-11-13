// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//
//    File:        RfidMng.h
//    Author:      Vania
//    Date:        06/11/2020
//
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#ifndef _RFIDMNG_H
#define _RFIDMNG_H
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global include ------------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#ifdef GD32F4xx 
#include "stm32f4xx_periph_init.h"
#else
#include "stm32h5xx_periph_init.h"
#endif

// -------------------------- global defines ------------------------------------------------------------------------------------------------------------------------- //
//#define CARD_UID_DIM		        (uint16_t)(16) /* Moved from 8 to 16 in order to acknoledge the UID of the ISODEP type cards */
#define CARD_UID_DIM		        (uint16_t)(8)
#define INFO_BLOCK_DIM                  (uint16_t)(16)

/* definition for uP resource  */
#define I2CSMBx                         I2C1
#define HI2CSMB0                        hi2c1
#define SMBx_GPIO_AF                    GPIO_AF4_I2C1
#define SMBx_RCC_GPIOx_CLK_ENABLE       __HAL_RCC_GPIOB_CLK_ENABLE  

#define CONTROL_STATUS_FLAG             ((uint8_t)0x01)
#define CONTROL_V230_FLAG               ((uint8_t)0x02)

#define CONTROL_START_FLAGS              (CONTROL_STATUS_FLAG | CONTROL_V230_FLAG)
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global typedef ------------------------------------------------------------------------------------------------------------------------- //
typedef enum // Definizioni per status rfidn SL030 board 
{
SL030_OPERATION_SUCCEED = (uint8_t)(0x00),         /* no error            */ 
SL030_OPERATION_FAILED,                            /* RFID not answering  */ 
}sl030_status_en;

typedef enum // Definizioni per command rfidn SL030 board
{
CMD_NULL          = (uint8_t)(0x00),
CMD_UID_READ      = (uint8_t)(0x01),    /* read uid / check card presence   */ 
CMD_BLOCK_READ    = (uint8_t)(0x03),    /* read a data block                */ 
CMD_BLOCK_WRITE   = (uint8_t)(0x04),    /* write block                      */ 
CMD_KEY_WRITE     = (uint8_t)(0x12),    /* write Key                        */ 
CMD_LOGIN         = (uint8_t)(0x13),    /* lgin with stored Key             */ 
CMD_FW_READ       = (uint8_t)(0xF0)     /* request FW version               */ 
}sl030_cmd_en;

typedef enum // tipo di carte supportate [sl030_card.card_type]
{
TYPE_NULL = (uint8_t)(0xFF),
MFMini_4B = (uint8_t)(0x01),
MFMini_7B = (uint8_t)(0x02),
MF1K_4B_UID = (uint8_t)(0x03),
MF1K_7B_UID = (uint8_t)(0x04),
MF4K_4B_UID = (uint8_t)(0x05),
MF4K_7B_UID = (uint8_t)(0x06),
MF_Ultralight = (uint8_t)(0x07),
ISODEP_NFCB = (uint8_t)(0x08),
MF_DESFire = (uint8_t)(0x09),
MF_PROX = (uint8_t)(0x0B),
MFPLUS2K_SL2_4B = (uint8_t)(0x21),
MFPLUS4K_SL2_4B = (uint8_t)(0x22),
MFPLUS2K_SL2_7B = (uint8_t)(0x23),
MFPLUS4K_SL2_7B = (uint8_t)(0x24),
MFPLUS2K_SL0_4B = (uint8_t)(0x31),
MFPLUS4K_SL0_4B = (uint8_t)(0x32),
MFPLUS2K_SL0_7B = (uint8_t)(0x33),
MFPLUS4K_SL0_7B = (uint8_t)(0x34),
Other = (uint8_t)(0x00)
}sl030_type_en;

typedef enum // possible value of rfid_state
{
RFID_STATE_IDLE = 0,
RFID_GET_SL030_FW,
RFID_KEY_WRITE,
RFID_CARD_SNIF,
RFID_SECTOR_LOGIN,
RFID_BLOCK8_READ,
RFID_BLOCK9_READ,
RFID_BLIND_STATE,
RFID_ERROR
}rfid_state_en;

typedef enum
{
RFID_EVENT_NULL = 0,
RFID_CONTROL_START,
RFID_CONTROL_STOP,
RFID_CARD_DETECT,
RFID_CARD_RELEASED,
RFID_CONTROL_UPDATE,
RFID_TIMEOUT,
RFID_CONTROL_V230,
RFID_WDG_START,  
RFID_WDG_TIMEOUT  
}RfidMngEvent_en;

/* queue info structure */
typedef __packed struct
{
RfidMngEvent_en    RfidMngEvent;
}RfidMngMsg_st;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global function prototypes ------------------------------------------------------------------------------------------------------------- //
void send_to_rfid(uint8_t rfid_event);
void send_to_WdgRfid(uint8_t rfid_event);
rfid_state_en rfid_state_get(void);
void rfid_anom1_update(void);
void rfid_uid_get(uint8_t *uid_ptr, sl030_type_en *type_ptr);
void rfid_new_block_get(uint8_t *dst_ptr, uint8_t block);
sl030_status_en rfid_sl030_data_push(sl030_cmd_en cmd, uint8_t *block_ptr, uint8_t block_num);
uint8_t* rfid_sl030_fw_get(void);
void RfidMngTask(void *pvParameters);
void setMifareFwVersion (char* MifarefwVersion);

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#endif

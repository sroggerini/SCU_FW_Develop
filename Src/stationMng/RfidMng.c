// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//
//  File:           RfidMng.c
//  Author:         Vania
//  Date:           06/11/2020
//
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local include -------------------------------------------------------------------------------------------------------------------------- //
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_rcc.h"
#else
#include "stm32h5xx_hal.h"
#include "stm32h5xx_ll_rcc.h"
#endif
#include "cmsis_os.h"

#include "displayPin.h"
#include "sbcGsy.h"

#include "eeprom.h"

#include "EvsMng.h"
#include "PersMng.h"
#include "RfidMng.h"
#include "telnet.h"
#include "scuMdb.h"

// #define EXCLUDE_RFID_OFF    

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local defines -------------------------------------------------------------------------------------------------------------------------- //
#define RFID_ERROR_MAX     (uint8_t)(4)

#define SL030_ADD_JP0      (uint8_t)(0xA0) // SL030 default address: JP1 - open  JP2 - open
#define SL030_ADD_JP1      (uint8_t)(0xA2) // SL030 default address: JP1 - open  JP2 - close
#define SL030_ADD_JP2      (uint8_t)(0xA4) // SL030 default address: JP1 - close JP2 - open
#define SL030_ADD_JP3      (uint8_t)(0xA6) // SL030 default address: JP1 - close JP2 - close

#define SL030_I2C_ADD      (SL030_ADD_JP0)

#define SL030_DATA_DIM     (uint16_t)(32)
#define SL030_FW_DIM       (uint16_t)(32)
#define SL030_KEYAA_DIM    (uint16_t)(6)
#define SL030_SECTOR_NUM   (uint16_t)(16)


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local typedef -------------------------------------------------------------------------------------------------------------------------- //
typedef enum // Definizioni per rfid_sl030_write_key
{
KEYAA_TYPE = (uint8_t)(0xAA),
KEYBB_TYPE = (uint8_t)(0xBB)
}sl030_key_en;

typedef enum // Definizioni per status rfid_power_config
{
RFID_OFF = 0,
RFID_ON
}rfid_power_en;

typedef __packed struct
{
uint8_t             len; // sl030 protocol payload message bytes
sl030_cmd_en        cmd; // sl030 protocol command type
}sl030_tx_header_st;

typedef __packed struct
{
uint8_t             len;    // sl030 protocol payload message bytes
sl030_cmd_en        cmd;    // sl030 protocol command type
sl030_status_en     status; // status indication
}sl030_rx_header_st;

typedef __packed struct
{
sl030_tx_header_st  tx_header;
uint8_t             tx_data[SL030_DATA_DIM]; // write data
sl030_rx_header_st  rx_header;
uint8_t             rx_data[SL030_DATA_DIM]; // read data
uint8_t             rw_len;
}sl030rw_cmd_st;

typedef __packed struct
{
sl030_type_en       card_type;
uint8_t             uid[CARD_UID_DIM];
}sl030_card_st;

typedef __packed struct
{
uint8_t b[INFO_BLOCK_DIM];
}block_array_st;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local constats ------------------------------------------------------------------------------------------------------------------------- //
static const uint8_t sl030_KeyAA[SL030_KEYAA_DIM] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local references ----------------------------------------------------------------------------------------------------------------------- //
xQueueHandle RfidMngQueue = NULL;

static RfidMngMsg_st        RfidMngMsg;

static sl030rw_cmd_st       sl030rw_cmd;

static rfid_state_en        rfid_state;

static uint8_t              rfid_enable;
static uint8_t              rfid_control_enable_old;

static uint8_t              rfid_error;
static uint8_t              rfid_error_counter;

static sl030_card_st        sl030_card;

static uint8_t              sl030_fw_array[SL030_FW_DIM];
static block_array_st       info_block_array[2];

static uint8_t              rfid_restart_Task = FALSE;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local function prototypes -------------------------------------------------------------------------------------------------------------- //
static xQueueHandle getRfidMngQueueHandle(void);

void i2c1_Handler(char *file, int line);
static void rfid_power_config(rfid_power_en status);

static uint32_t rfid_fail_manager(rfid_state_en *state);
static uint8_t rfid_sl030_read(sl030rw_cmd_st *src_ptr, uint8_t *dst_ptr, uint16_t dst_len);
static uint8_t rfid_sl030_write(sl030rw_cmd_st *src_ptr, uint8_t *dst_ptr, uint16_t dst_len, uint32_t delay);
static sl030_status_en rfid_sl030_data_pull(sl030_cmd_en cmd, uint8_t *dst_ptr, uint8_t aux_data);
static sl030_status_en rfid_sl030_write_key(uint8_t *key_ptr);
static sl030_status_en rfid_sl030_sector_login(uint8_t sector);
static void rfid_uid_reset(void);
static void RfidManager_init(void);
static uint32_t RfidManager(RfidMngMsg_st *pMsg);
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //


// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

extern void MX_I2C1_Init(void);
extern void I2C_ClearBusyFlagErratum (I2C_HandleTypeDef* hi2c);

extern osThreadAttr_t RfidMngTask_attributes;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  getRfidMngQueueHandle
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static xQueueHandle getRfidMngQueueHandle(void)
{
return(RfidMngQueue);
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  send_to_rfid
//
//  DESCRIPTION:    impacchetta l'evento da inviare a RfidMngTask
//  
//  INPUT:          valore di RfidMngEvent
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void send_to_rfid(uint8_t rfid_event)
{
RfidMngMsg_st    msgRfidSend;

msgRfidSend.RfidMngEvent = (RfidMngEvent_en)(rfid_event);
configASSERT(xQueueSendToBack(getRfidMngQueueHandle(), (void *)&msgRfidSend, portMAX_DELAY) == pdPASS);
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  rfid_power_config
//
//  DESCRIPTION:    configure GPIO power rfid control (SL030)
//
//  INPUT:          status: stato desiderato on/off
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void rfid_power_config(rfid_power_en status)
{
GPIO_InitTypeDef GPIO_InitStruct;

if (status == RFID_ON)
    {
    /*Configure GPIO pin Output Level to 0 for power ON */
    HAL_GPIO_WritePin(RFID_PWR_GPIO_Port, RFID_PWR_Pin, GPIO_PIN_RESET);
    }
#ifndef EXCLUDE_RFID_OFF
else    // if (status == RFID_OFF)
    {
    /*Configure GPIO pin Output Level to 1 for power OFF */
    HAL_GPIO_WritePin(RFID_PWR_GPIO_Port, RFID_PWR_Pin, GPIO_PIN_SET);
    }
#endif

/*Configure GPIO pins : RFID_PWR_Pin */
GPIO_InitStruct.Pin = RFID_PWR_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(RFID_PWR_GPIO_Port, &GPIO_InitStruct);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  MX_I2C1_Init
//
//  DESCRIPTION:    This function is executed in case of error occurrence.
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void i2c1_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
while(1)
    {
    }

/* USER CODE END Error_Handler_Debug */
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  rfid_fail_manager
//
//  DESCRIPTION:    gestione del fallimento dell'operazione sul lettore
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint32_t rfid_fail_manager(rfid_state_en *state)
{
uint32_t    newTimeTick;
uint8_t     rfid_control_enable;
evs_mode_en rfid_mode;

eeprom_param_get(EVS_MODE_EADD, (uint8_t*)(&rfid_mode), 1);

eeprom_param_get(CONTROL_BYTE1_EADD, &rfid_control_enable, 1);
rfid_control_enable &= MIFARE_CRL1;

if (rfid_control_enable == MIFARE_CRL1)
    {
    if (rfid_error_counter > RFID_ERROR_MAX)
        {
        rfid_error_counter = 0;
    
        if (rfid_mode != EVS_FREE_MODE)
            {
            rfid_error = 1;
            send_to_evs(EVS_RFID_ANOM1_UPDATE);
            /* LOG message */            
            EVLOG_Message(EV_ERROR, "RFID error");
            }

        *state = RFID_ERROR;
        newTimeTick = pdMS_TO_TICKS(1000);
        /* Reset I2C peripheral in order to exit from a stuck condition */
        I2C_ClearBusyFlagErratum(&HI2CSMB0);   // Fixed ticket SCU-42
        
        }
    else
        {
        rfid_error_counter ++;
        newTimeTick = pdMS_TO_TICKS(250);
        }
    }
else
    {
    *state = RFID_ERROR;                // con MIFARE_ANOM1 non abilitato, si tenta reset del lettore senza segnalare l'errore
    newTimeTick = pdMS_TO_TICKS(5000);
    }

return newTimeTick;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  rfid_sl030_read
//
//  DESCRIPTION:    driver per la lettura SL030
//
//  INPUT:          puntatore istruzioni per la lettura: src_ptr; puntatore destinatario dati: dst_ptr; numero byte da legger: dst_len
//
//  OUTPUT:         0 = successfull read; N = error code
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t rfid_sl030_read(sl030rw_cmd_st *src_ptr, uint8_t *dst_ptr, uint16_t dst_len)
{
  
    // Aspetto che il canale I2C sia libero
    while (HAL_I2C_GetState(&HI2CSMB0) != HAL_I2C_STATE_READY);
    
    /* Get tick start value */
    uint32_t Tickstart = HAL_GetTick();
      
    /* Transmit */
    while (HAL_I2C_Master_Transmit(&HI2CSMB0, SL030_I2C_ADD, (uint8_t*)&src_ptr->tx_header, src_ptr->rw_len, 100) != HAL_OK)
    {
        if (HAL_I2C_GetError(&HI2CSMB0) != HAL_I2C_ERROR_AF)
            return 1;
        /* Some error happens and 30ms timeout elapsed? */
        /* NOTE: I cannot delay more , otherwise GSY is not able to start */        
        if ((HAL_GetTick() - Tickstart) > 30)   /* Fixed ticket SCU-71 */
           return HAL_TIMEOUT;
     }

    osDelay(5);    /* 5ms of fixed delay */ 
    
    /* Get tick start value */
    Tickstart = HAL_GetTick();    
    
    /* Receive */
    while (HAL_I2C_Master_Receive(&HI2CSMB0, SL030_I2C_ADD, dst_ptr, dst_len, 1000) != HAL_OK)
    {
        if (HAL_I2C_GetError (&HI2CSMB0) != HAL_I2C_ERROR_AF)
            return 2;
        /* Some error happens and 200ms timeout elapsed? */
        if ((HAL_GetTick() - Tickstart) > 200)  /* Fixed ticket SCU-71 */
           return HAL_TIMEOUT;
    }

    return 0;
    
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  rfid_sl030_write
//
//  DESCRIPTION:    driver per la scrittura SL030
//
//  INPUT:          puntatore istruzioni per la scrittura: src_ptr; puntatore destinatario risposta: dst_ptr; numero byte da scrivere: dst_len;
//                  delay: intervallo di attesa fra l'operazione di scrittura e l'operazione di lettura dell'esito
//
//  OUTPUT:         0 = successfull read; N = error code
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t rfid_sl030_write(sl030rw_cmd_st *src_ptr, uint8_t *dst_ptr, uint16_t dst_len, uint32_t delay)
{
  
    // Aspetto che il canale I2C sia libero
    while (HAL_I2C_GetState(&HI2CSMB0) != HAL_I2C_STATE_READY);

    /* Get tick start value */
    uint32_t Tickstart = HAL_GetTick();
    
    /* Transmit */
    while (HAL_I2C_Master_Transmit(&HI2CSMB0, SL030_I2C_ADD, (uint8_t*)&src_ptr->tx_header, src_ptr->rw_len, 100) != HAL_OK)
    {
        if (HAL_I2C_GetError(&HI2CSMB0) != HAL_I2C_ERROR_AF)
            return 1;
        /* Some error happens and 200ms timeout elapsed? */
        if ((HAL_GetTick() - Tickstart) > 200)   /* Fixed ticket SCU-71 */
           return HAL_TIMEOUT;
        
    }

    osDelay(delay);    /* no errors, check this writing */

    /* Get tick start value */
    Tickstart = HAL_GetTick();    
    
    /* Receive */
    while (HAL_I2C_Master_Receive(&HI2CSMB0, SL030_I2C_ADD, dst_ptr, dst_len, 1000) != HAL_OK)
    {
        if (HAL_I2C_GetError (&HI2CSMB0) != HAL_I2C_ERROR_AF)
            return 2;
        /* Some error happens and 200ms timeout elapsed? */
        if ((HAL_GetTick() - Tickstart) > 200)   /* Fixed ticket SCU-71 */
           return HAL_TIMEOUT;
    }

    return 0;
    
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  rfid_sl030_data_push
//
//  DESCRIPTION:    funzione di alto livello per la scrittura su SL030
//
//  INPUT:          comando secondo protocollo SL030: cmd; puntatore blocco info da scrivere [info_block_array]: block_ptr; numero del blocco da scrivere: block_num
//
//  OUTPUT:         estio dell'operazione: rx_header.status
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
sl030_status_en rfid_sl030_data_push(sl030_cmd_en cmd, uint8_t *block_ptr, uint8_t block_num)
{
uint8_t          i;
sl030rw_cmd_st   *sl030_write_ptr = &sl030rw_cmd;

sl030_write_ptr->tx_header.len = (INFO_BLOCK_DIM + 2);                              // sl030 protocol len
sl030_write_ptr->tx_header.cmd = cmd;                                               // sl030 protocol command
sl030_write_ptr->tx_data[0] = block_num;

sl030_write_ptr->rw_len = (sl030_write_ptr->tx_header.len + 1);                     // number of bytes to send

for (i=0; i<INFO_BLOCK_DIM; i++)
    sl030_write_ptr->tx_data[(i + 1)] = *(block_ptr + i);

sl030_write_ptr->rx_header.len = 0;
sl030_write_ptr->rx_header.cmd = CMD_NULL;
sl030_write_ptr->rx_header.status = SL030_OPERATION_FAILED;

if (rfid_sl030_write(sl030_write_ptr, (uint8_t*)(&sl030_write_ptr->rx_header), sl030_write_ptr->rw_len, 30) == 0)
    {
    if ((sl030_write_ptr->rx_header.len == sl030_write_ptr->rw_len) && (sl030_write_ptr->rx_header.cmd == cmd) || (sl030_write_ptr->rx_header.status == SL030_OPERATION_SUCCEED))
        {
        for (i=0; i<INFO_BLOCK_DIM; i++)
            {
            if (sl030_write_ptr->rx_data[i] != *(block_ptr + i))
                sl030_write_ptr->rx_header.status = SL030_OPERATION_FAILED;
            }
        }
    }

return sl030_write_ptr->rx_header.status;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  rfid_sl030_data_pull
//
//  DESCRIPTION:    funzione di alto livello per la lettura su SL030
//
//  INPUT:          comando secondo protocollo SL030: cmd; puntatore array destinatario: dst_ptr; eventuale info aggiuntiva: aux_data
//
//  OUTPUT:         estio dell'operazione: rx_header.status
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static sl030_status_en rfid_sl030_data_pull(sl030_cmd_en cmd, uint8_t *dst_ptr, uint8_t aux_data)
{
uint8_t          i, offset, pull_byte_num;
sl030rw_cmd_st   *sl030_read_ptr = &sl030rw_cmd;

switch (cmd)
    {
    case CMD_FW_READ:
        {
        pull_byte_num = SL030_FW_DIM;
        sl030_read_ptr->tx_header.len = 1;                                          // sl030 protocol len
        }
        break;

    case CMD_UID_READ:
        {
        rfid_uid_reset();
        pull_byte_num = CARD_UID_DIM;
        sl030_read_ptr->tx_header.len = 1;                                          // sl030 protocol len
        }
        break;

    case CMD_BLOCK_READ:
        {
        pull_byte_num = INFO_BLOCK_DIM;
        sl030_read_ptr->tx_header.len = 2;                                          // sl030 protocol len
        sl030_read_ptr->tx_data[0] = aux_data;
        }
        break;

    default:
        {
        RfidManager_init();
        }
        break;
    }

sl030_read_ptr->tx_header.cmd = cmd;                                                // sl030 protocol command

sl030_read_ptr->rx_header.len = 0;
sl030_read_ptr->rx_header.cmd = CMD_NULL;
sl030_read_ptr->rx_header.status = SL030_OPERATION_FAILED;

for (i=0; i<SL030_DATA_DIM; i++)
    sl030_read_ptr->rx_data[i] = 0xFF;

sl030_read_ptr->rw_len = (sl030_read_ptr->tx_header.len + 1);                       // number of bytes to send

if (rfid_sl030_read(sl030_read_ptr, (uint8_t*)(&sl030_read_ptr->rx_header), (pull_byte_num + 3)) == 0)
    {
    if ((sl030_read_ptr->rx_header.len >= 2) && (sl030_read_ptr->rx_header.cmd == cmd) && (sl030_read_ptr->rx_header.status == SL030_OPERATION_SUCCEED))
        {
        if (sl030_read_ptr->rx_header.len > (pull_byte_num + 2))
            sl030_read_ptr->rx_header.len = (pull_byte_num + 2);
        
        if (cmd == CMD_UID_READ)
            {
            *dst_ptr = sl030_read_ptr->rx_data[(sl030_read_ptr->rx_header.len - 3)];  // card_type
            offset = 1;
            }
        else
            offset = 0;

        for (i=0; i<(sl030_read_ptr->rx_header.len - (2 + offset)); i++)
            *(dst_ptr + offset + i) = sl030_read_ptr->rx_data[i];
        }
    else if ((sl030_read_ptr->rx_header.len == 2) && (cmd == CMD_UID_READ) && (sl030_read_ptr->rx_header.status == 0x01))
        {
        *dst_ptr = TYPE_NULL;                                                       // card_type
        sl030_read_ptr->rx_header.status = SL030_OPERATION_SUCCEED;
        }
    else
        sl030_read_ptr->rx_header.status = SL030_OPERATION_FAILED;
    }

return sl030_read_ptr->rx_header.status;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  rfid_sl030_write_key
//
//  DESCRIPTION:    scrittura keyAA SL030 su tutti i settori
//
//  INPUT:          puntatore key: key_ptr [sl030_KeyAA]
//
//  OUTPUT:         estio dell'operazione: rx_header.status
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static sl030_status_en rfid_sl030_write_key(uint8_t *key_ptr)
{
uint8_t          i;
sl030rw_cmd_st   *sl030_write_ptr = &sl030rw_cmd;

sl030_write_ptr->tx_header.len = (SL030_KEYAA_DIM + 3);                             // sl030 protocol len
sl030_write_ptr->tx_header.cmd = CMD_KEY_WRITE;                                     // sl030 protocol command
sl030_write_ptr->tx_data[1] = KEYAA_TYPE;

sl030_write_ptr->rw_len = (sl030_write_ptr->tx_header.len + 1);                     // number of bytes to send

for (i=0; i<SL030_KEYAA_DIM; i++)
    sl030_write_ptr->tx_data[(i + 2)] = *(key_ptr + i);

for (i=0; i<SL030_SECTOR_NUM; i++)
    {
    sl030_write_ptr->rx_header.len = 0;
    sl030_write_ptr->rx_header.cmd = CMD_NULL;
    sl030_write_ptr->rx_header.status = SL030_OPERATION_FAILED;

    sl030_write_ptr->tx_data[0] = i;                                                // sector                

    if (rfid_sl030_write(sl030_write_ptr, (uint8_t*)(&sl030_write_ptr->rx_header), 3, 60) == 0)
        {
        if ((sl030_write_ptr->rx_header.len != 2) || (sl030_write_ptr->rx_header.cmd != CMD_KEY_WRITE) || (sl030_write_ptr->rx_header.status != SL030_OPERATION_SUCCEED))
            {
            sl030_write_ptr->rx_header.status = SL030_OPERATION_FAILED;
            break;
            }
        }
    }

return sl030_write_ptr->rx_header.status;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  rfid_sl030_sector_login
//
//  DESCRIPTION:    login per l'accesso al blocco della carta da leggere / scrivere
//
//  INPUT:          settore di appartenenza del blocco: sector
//
//  OUTPUT:         estio dell'operazione: rx_header.status
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static sl030_status_en rfid_sl030_sector_login(uint8_t sector)
{
sl030rw_cmd_st   *sl030_write_ptr = &sl030rw_cmd;

sl030_write_ptr->tx_header.len = 3;                                                 // sl030 protocol len
sl030_write_ptr->tx_header.cmd = CMD_LOGIN;                                         // sl030 protocol command
sl030_write_ptr->tx_data[0] = sector;
sl030_write_ptr->tx_data[1] = KEYAA_TYPE;

sl030_write_ptr->rw_len = (sl030_write_ptr->tx_header.len + 1);                     // number of bytes to send

sl030_write_ptr->rx_header.len = 0;
sl030_write_ptr->rx_header.cmd = CMD_NULL;
sl030_write_ptr->rx_header.status = SL030_OPERATION_FAILED;

if (rfid_sl030_write(sl030_write_ptr, (uint8_t*)(&sl030_write_ptr->rx_header), 3, 30) == 0)
    {
    if ((sl030_write_ptr->rx_header.len == 2) && (sl030_write_ptr->rx_header.cmd == CMD_LOGIN) && (sl030_write_ptr->rx_header.status == 0x02))
        sl030_write_ptr->rx_header.status = SL030_OPERATION_SUCCEED;
    }

return sl030_write_ptr->rx_header.status;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  rfid_state_get
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
rfid_state_en rfid_state_get(void)
{
return rfid_state;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  rfid_anom1_update
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         noe
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void rfid_anom1_update(void)
{
if (rfid_error == 1)
    evs_error_set(CONTROL_BYTE_1, MIFARE_ANOM1, 1);
else
    evs_error_set(CONTROL_BYTE_1, MIFARE_ANOM1, 0);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  rfid_uid_reset
//
//  DESCRIPTION:    reset uid e tipo carta
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void rfid_uid_reset(void)
{
uint8_t i;

for (i=0; i<CARD_UID_DIM; i++)
    sl030_card.uid[i] = 0x00;

sl030_card.card_type = TYPE_NULL;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  rfid_uid_get
//
//  DESCRIPTION:    ritorna uid e tipo carta letto e li resetta
//
//  INPUT:          puntatore destinatario: uid_ptr; puntatore tipo carta: type_ptr; 
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void rfid_uid_get(uint8_t *uid_ptr, sl030_type_en *type_ptr)
{
uint8_t i;

for (i=0; i<CARD_UID_DIM; i++)
    *(uid_ptr + i) = sl030_card.uid[i];

*type_ptr = sl030_card.card_type;

rfid_uid_reset();
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  rfid_new_block_get
//
//  DESCRIPTION:    ritorna il blocco letto e lo resetta
//
//  INPUT:          puntatore destinatario: uid_ptr; numero blocco: block
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void rfid_new_block_get(uint8_t *dst_ptr, uint8_t block)
{
uint8_t i;

for (i=0; i<INFO_BLOCK_DIM; i++)
    *(dst_ptr + i) = info_block_array[block].b[i];

for (i=0; i<INFO_BLOCK_DIM; i++)
    info_block_array[block].b[i] = 0xFF;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  rfid_sl030_fw_get
//
//  DESCRIPTION:    ritorna versione fw SL030
//
//  INPUT:          none
//
//  OUTPUT:         puntatore array
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t* rfid_sl030_fw_get(void)
{
  
  rfid_sl030_data_pull(CMD_FW_READ, sl030_fw_array, 0);  
  
return sl030_fw_array;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  RfidManager_init
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void RfidManager_init(void)
{
  rfid_enable = 0;  
  /* If Task restarts after a osThreadTerminate */ 
  if (rfid_restart_Task == TRUE)
  {
    /* Send the event to Rfid */
    send_to_rfid(RFID_CONTROL_START);
    rfid_enable = CONTROL_START_FLAGS;
  }
  else
  {
    rfid_error = 0;
    rfid_error_counter = 0;
  }
  rfid_restart_Task = FALSE;
  rfid_state = RFID_STATE_IDLE;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  RfidManager
//
//  DESCRIPTION:    gestione SL030: lettura carte mifare
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint32_t RfidManager(RfidMngMsg_st *pMsg)
{
uint32_t    newTimeTick = pdMS_TO_TICKS(5000);
uint8_t     rfid_control_enable, actuator_mode;
evs_mode_en rfid_mode;

newTimeTick = pdMS_TO_TICKS(1000);
eeprom_param_get(EVS_MODE_EADD, (uint8_t*)(&rfid_mode), 1);

if ((isSemMode() == TRUE) && (rfid_mode == EVS_PERS_MODE) && (getCollaudoRunning() == FALSE))
    {
    if ((evs_state_get() == EVSTATE_DISABLED) || (evs_state_get() == EVSTATE_AUTH_WAIT) || (evs_state_get() == EVSTATE_SOCKET_AVAILABLE))
        {
        rfid_mode = EVS_FREE_MODE;
        SCU_InfoStation_Set ((uint8_t *)&infoStation.evs_mode, (uint8_t*)(&rfid_mode), 1);  /* ex EVS_MODE_EADD */
        send_to_evs(SEM_AUTORIZATION_MODE);
        send_to_pers(PERS_AUTORIZATION_MODE);
        }
    }

eeprom_param_get(CONTROL_BYTE1_EADD, &rfid_control_enable, 1);
rfid_control_enable &= MIFARE_CRL1;

if (((rfid_control_enable == 0) || (rfid_mode == EVS_FREE_MODE)) && (rfid_error == 1))
    {
    rfid_error = 0;
    send_to_evs(EVS_RFID_ANOM1_UPDATE);
    }

if (pMsg->RfidMngEvent == RFID_CONTROL_STOP)
    RfidManager_init();
else if (pMsg->RfidMngEvent == RFID_CONTROL_START)                                  // comando di start da EvsManager
    rfid_enable |= CONTROL_STATUS_FLAG;
else if (pMsg->RfidMngEvent == RFID_CONTROL_V230)                                   // comando di start da main [switch (infoV230.statusV230)]
    rfid_enable |= CONTROL_V230_FLAG;

switch (rfid_state)
    {
    case RFID_STATE_IDLE:
        {
        if ((rfid_enable == CONTROL_START_FLAGS) && (checkVbusFlag() != EMRG_V230KO_VBUSENA))   // ricevuti entrmabi i comadi di start e la tensione 230 è corretta  
            {
            rfid_power_config(RFID_OFF);                                            // reset del lettore rfid
            osDelay(200);
            rfid_power_config(RFID_ON);
            /* A little delay before sending the request, this is required by SCAME RFID board (the issue is that at startup, DTC pin is detected by SCU) */            
            osDelay(10);  
            rfid_state = RFID_GET_SL030_FW;
//            newTimeTick = pdMS_TO_TICKS(100);
            newTimeTick = pdMS_TO_TICKS(300);
            }
        else
            newTimeTick = pdMS_TO_TICKS(5000);                                      // assegnazione di sicurezza [per non bloccare il task in nessun caso]
        }
        break;

    case RFID_GET_SL030_FW:
        {
//      if (pMsg->RfidMngEvent == RFID_TIMEOUT)     // stato solo temporizzato
        if (rfid_sl030_data_pull(CMD_FW_READ, sl030_fw_array, 0) == SL030_OPERATION_SUCCEED)
            {
            tPrintf("RFID FW VER = %s\n\r", sl030_fw_array);
            setMifareFwVersion((char*)sl030_fw_array);   /* Copy rfid fw version in modbus map (Fixed ticket SCU-84) */
            rfid_error_counter = 0;
            rfid_error = 0;
            send_to_evs(EVS_RFID_ANOM1_UPDATE);                                     // reset [eventuale] di MIFARE_ANOM1
            rfid_state = RFID_KEY_WRITE;
            newTimeTick = pdMS_TO_TICKS(5);
            }
        else
            newTimeTick = rfid_fail_manager(&rfid_state);
        }
        break;

    case RFID_KEY_WRITE:
        {
//      if (pMsg->RfidMngEvent == RFID_TIMEOUT)     // stato solo temporizzato
        if (rfid_sl030_write_key((uint8_t*)(sl030_KeyAA)) == SL030_OPERATION_SUCCEED)
            {
            rfid_error_counter = 0;
            rfid_state = RFID_CARD_SNIF;
            newTimeTick = pdMS_TO_TICKS(5000);
            }
        else
            newTimeTick = rfid_fail_manager(&rfid_state);
        }
        break;

    case RFID_CARD_SNIF:
        {
        if ((pMsg->RfidMngEvent == RFID_TIMEOUT) || (pMsg->RfidMngEvent == RFID_CARD_DETECT))
            {
            if (rfid_sl030_data_pull(CMD_UID_READ, (uint8_t *)(&sl030_card), 0) == SL030_OPERATION_SUCCEED)
                {
                rfid_error_counter = 0;

                if (sl030_card.card_type == TYPE_NULL)                              // nessuna carta presente
                    newTimeTick = pdMS_TO_TICKS(5000);                              // si ricarica lo snif
                else
                    {
                    if (((rfid_mode == EVS_NET_MODE) || (rfid_mode == EVS_OCPP_MODE)) && (getCollaudoRunning() == FALSE))
                        {
                        tPrintf("UID = %02x %02x %02x %02x\n\r", sl030_card.uid[0], sl030_card.uid[1], sl030_card.uid[2], sl030_card.uid[3]);
                        
                        if (gsy_connected_get() == 1)
                            {
                            eeprom_param_get(ACTUATORS_EADD, &actuator_mode, 1);
                            actuator_mode &= PAUT_ATT0;

                            if (evs_gost_param_get() == 1)
                                gsy_quick_polling_update((BUSY_OUTLET | RFID_PENDING), 1);
                            else                            
                                send_to_evs(EVS_AUTH_REQUIRED);
                            }

                        rfid_state = RFID_BLIND_STATE;
                        newTimeTick = pdMS_TO_TICKS(1000);
                        }
                    else if (sl030_card.card_type == MFMini_4B)        // ((rfid_mode == EVS_PERS_MODE) || (rfid_mode == EVS_FREE_MODE))
                        {
                        rfid_state = RFID_SECTOR_LOGIN;
                        newTimeTick = pdMS_TO_TICKS(10);
                        }
                    else if (sl030_card.card_type == ISODEP_NFCB)      // This is the card type detected when a smartphone is tapped on the reader 
                        {
                        newTimeTick = pdMS_TO_TICKS(1000);             // Fixed ticket SCU-49 "Gestione NFC secondo il protocollo ISODEP_NCFB" 
                        send_to_pers(PERS_CARD_READ);                      
                        }
                    }
                }
            else
                newTimeTick = rfid_fail_manager(&rfid_state);
            }
        }
        break;

    case RFID_SECTOR_LOGIN:
        {
//      if (pMsg->RfidMngEvent == RFID_TIMEOUT)     // stato solo temporizzato
        if (rfid_sl030_sector_login(2) == SL030_OPERATION_SUCCEED)                  // login sector 2
            {
            rfid_error_counter = 0;
            rfid_state = RFID_BLOCK8_READ;
            newTimeTick = pdMS_TO_TICKS(10);
            }
        else
            newTimeTick = rfid_fail_manager(&rfid_state);
        }
        break;

    case RFID_BLOCK8_READ:
        {
//      if (pMsg->RfidMngEvent == RFID_TIMEOUT)     // stato solo temporizzato
        if (rfid_sl030_data_pull(CMD_BLOCK_READ, info_block_array[0].b, 8) == SL030_OPERATION_SUCCEED)   // read block 8
            {
            rfid_error_counter = 0;
            rfid_state = RFID_BLOCK9_READ;
            newTimeTick = pdMS_TO_TICKS(10);
            }
        else
            newTimeTick = rfid_fail_manager(&rfid_state);
        }
        break;

    case RFID_BLOCK9_READ:
        {
//      if (pMsg->RfidMngEvent == RFID_TIMEOUT)     // stato solo temporizzato
        if (rfid_sl030_data_pull(CMD_BLOCK_READ, info_block_array[1].b, 9) == SL030_OPERATION_SUCCEED)   // read block 9
            {
            rfid_error_counter = 0;
            rfid_state = RFID_BLIND_STATE;
            newTimeTick = pdMS_TO_TICKS(1000);
            send_to_pers(PERS_CARD_READ);
            }
        else
            newTimeTick = rfid_fail_manager(&rfid_state);
        }
        break;

    case RFID_BLIND_STATE:
        {
//      if (pMsg->RfidMngEvent == RFID_TIMEOUT)     // stato solo temporizzato
        if (pMsg->RfidMngEvent == RFID_CARD_RELEASED)
            {
            rfid_state = RFID_CARD_SNIF;
            newTimeTick = pdMS_TO_TICKS(1500);
            }
        }
        break;

    case RFID_ERROR:
        {
        if (pMsg->RfidMngEvent == RFID_CONTROL_UPDATE)
            {
            if (rfid_control_enable_old == 0)
                newTimeTick = pdMS_TO_TICKS(250);
            }
        else if (pMsg->RfidMngEvent == RFID_TIMEOUT)
            {
            rfid_state = RFID_STATE_IDLE;
            send_to_rfid(RFID_CONTROL_START);
            newTimeTick = pdMS_TO_TICKS(250);
            }
        }
        break;

    default:
        {
        RfidManager_init();
        newTimeTick = pdMS_TO_TICKS(250);
        }
        break;
    }

rfid_control_enable_old = rfid_control_enable;

return (newTimeTick);
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  RfidMngTask
//                  
//  DESCRIPTION:    Handle sl030 rfid reader/writer
//                  
//  INPUT:          none
//                  
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void RfidMngTask(void *pvParameters)
{
uint32_t timeTick;

/* init task */

/*-------- Creates an empty mailbox for RfidMngTask messages --------------------------*/
RfidMngQueue = xQueueCreate(2, sizeof(RfidMngMsg_st));
configASSERT(RfidMngQueue != NULL);

MX_I2C1_Init();
RfidManager_init();
timeTick = portMAX_DELAY;

/* start task */

for (;;)
    {
    /* Wait for some event from SW */
    if (xQueueReceive(RfidMngQueue, (void *)&RfidMngMsg, timeTick) == pdPASS)
        {
        timeTick = RfidManager(&RfidMngMsg);
        }
    else
        {
        /* Wait for possible handled timeout */
        RfidMngMsg.RfidMngEvent = RFID_TIMEOUT;
        timeTick = RfidManager(&RfidMngMsg);
        }
    }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

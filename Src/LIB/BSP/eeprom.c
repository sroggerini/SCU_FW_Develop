// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//
//  File:           eeprom.c
//  Author:         Vania
//  Date:           06/11/2020
//
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local include -------------------------------------------------------------------------------------------------------------------------- //
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#else
#include "stm32h5xx_hal.h"
#endif

#include "BlockMng.h"
#include "EnergyMng.h"
#include "EvsMng.h"
#include "ExtInpMng.h"
#include "LcdMng.h"
#include "PersMng.h"
#include "RfidMng.h"

#include "eeprom.h"

#include "i2c.h"
#include "wrapper.h"
#include "ledMng.h"
#include "scuMdb.h"
#include "sbcSem.h"
#include "scheduleMng.h"
#include "string.h"
#include "transaction_register.h"
#include "secure_area.h"

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local defines -------------------------------------------------------------------------------------------------------------------------- //
#define EEPROM_NULL_UPDATE      (uint8_t)(0x00)
#define EEPROM_CONFIG_UPDATE    (uint8_t)(0x01)
#define EEPROM_MASTER_UPDATE    (uint8_t)(0x02)
#define EEPROM_USERMAP_UPDATE   (uint8_t)(0x04)
#define EEPROM_USERID_UPDATE    (uint8_t)(0x08)
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local typedef -------------------------------------------------------------------------------------------------------------------------- //
typedef enum // eeprom_param_array board default value [eeprom_param_board_val]
{
EDATA_VALID_EDEF    = (EDATA_DEFAULT_PRG),                                            // 0  - 
EDATA_NUM_EDEF      = (EEPROM_PARAM_NUM),                                           // 1  - 
SERNUM_BYTE0_EDEF   = (uint8_t)(0xFF),                                              // 2  - 
SERNUM_BYTE1_EDEF   = (uint8_t)(0xFF),                                              // 3  - 
SERNUM_BYTE2_EDEF   = (uint8_t)(0xFF),                                              // 4  - 
SERNUM_BYTE3_EDEF   = (uint8_t)(0xFF),                                              // 5  - 
SOCKET_ENABLE_EDEF  = (uint8_t)(1),                                                 // 6  - presa abilitata
BATTERY_CONFIG_EDEF = (uint8_t)(0),                                                 // 7  - modalità di funzionamento in presenza di batteria tampone
LANGUAGE_EDEF       = (LANGUAGE_ITA),                                               // 8  - lingua di default display lcd
RS485_ADD_EDEF      = (uint8_t)(RS485_ADD_MAX),                                     // 9  - indirizzo SCU su bus RS-485 (16 a display)
RTC_VALID_EDEF      = (uint8_t)(0),                                                 // 10 - rtc non configurato
EVS_MODE_EDEF       = (EVS_FREE_MODE),                                              // 11 - modalità di funzionamento della EVS
M3T_CURRENT_EDEF    = (uint8_t)(32),                                                // 12 - corrente massima Evs [A]
M3S_CURRENT_EDEF    = (uint8_t)(16),                                                // 13 - corrente massima in Modo3 semplificato [A]
SOCKET_TYPE_EDEF    = (SOCKET_T2_TETHERED),                                         // 14 - tipo presa, socket type
EMETER_INT_EDEF     = (EMETER_TYPE_NULL),                                           // 15 - tipo misuratore di energia interno disabilitato
#ifdef HW_MP28947
CONTROL_BYTE0_EDEF  = (MIRROR_CRL0 | RCDM_CRL0 | REMOTE_CRL0),                      // 16 - abilitazione controlli byte 0
CONTROL_BYTE1_EDEF  = (CPLOST_CRL1 | CPSHORT_CRL1 | OVERCURRENT_CRL1 | EMETER_INT_CRL1),              // 17 - abilitazione controlli byte 1
CONTROL_BYTE2_EDEF  = (EMETER_EXT_CRL2 | RECTIFIER_CRL2),                           // 18 - abilitazione controlli byte 2
#else
CONTROL_BYTE0_EDEF  = (RCBO_CRL0 | PULS_CRL0 | REMOTE_CRL0 | VENT_CRL0),            // 16 - abilitazione controlli byte 0
CONTROL_BYTE1_EDEF  = (CPLOST_CRL1 | CPSHORT_CRL1),                                 // 17 - abilitazione controlli byte 1
CONTROL_BYTE2_EDEF  = (EMETER_EXT_CRL2 | RECTIFIER_CRL2),                           // 18 - abilitazione controlli byte 2
#endif
CONTROL_BYTE3_EDEF  = (uint8_t)(0x00),                                              // 19 - abilitazione controlli byte 3
ACTUATORS_EDEF      = (CONTACT_ATT0),                                               // 20 - configurazione attuatori contattore, blocchi presa e bobina di sgancio
BLOCK_DIR_EDEF      = (uint8_t)(0x01),                                              // 21 - configurazione direzione blocchi presa
PERS_UIDNUM_EDEF    = (uint8_t)(0),                                                 // 22 - personal: numero UID registrati [EVS_PERS_MODE]
PERS_MASTER_EDEF    = (uint8_t)(0),                                                 // 23 - personal: flag UID master registrato [EVS_PERS_MODE]
PMNG_MODE_EDEF      = (PMNG_FULL),                                                  // 24 - power management: configurazione power management
PMNG_EMETER_EDEF    = (EMETER_TYPE_NULL),                                           // 25 - power management: tipo misuratore di energia esterno disabilitato
PMNG_PWRLSB_EDEF    = (uint8_t)(30),                                                // 26 - power management: potenza domestica installata LSB [MSB + LSB = 3KW]
PMNG_PWRMSB_EDEF    = (uint8_t)(0),                                                 // 27 - power management: potenza domestica installata MSB
PMNG_ERROR_EDEF     = (uint8_t)(5),                                                 // 28 - power management: errore ammesso nella regolazione di potenza [KW * 10]
PMNG_CURRENT_EDEF   = (uint8_t)(60),                                                // 29 - power management: corrente minima di ricarica del veicolo prima di andare in sospensione [ampere]
PMNG_MULTIP_EDEF    = (uint8_t)(0),                                                 // 30 - power management: fattore moltiplicativo per la potenza minima per uscire da no-power
PMNG_DMAX_EDEF      = (uint8_t)(40),                                                // 31 - power management: fattore moltiplicativo della potenza per la sospensione immediata della ricarica
PMNG_TRANGE_EDEF    = (uint8_t)(0),                                                 // 32 - power management: abilitazione della ricarica con potenza variabile per fasce orarie [fisse]
TCHARGE_MODE_EDEF   = (uint8_t)(0),                                                 // 33 - configurazione ricarica a tempo /* RIVEDERE */
TCHARGE_TIME_EDEF   = (uint8_t)(2),                                                 // 34 - numero step di 30 minuti in ricarica a tempo
PMNG_PWDB0_EDEF     = (char)('0'),                                                  // 35 - password menù nascosto byte 0
PMNG_PWDB1_EDEF     = (char)('0'),                                                  // 36 - password menù nascosto byte 1
PMNG_PWDB2_EDEF     = (char)('0'),                                                  // 37 - password menù nascosto byte 2
PMNG_UNBAL_EDEF     = (PMNG_UNBAL_OFF),                                             // 38 - power management: flag di consenso per carichi sbilanciati in trifase
LANG_CONFIG0_EDEF   = (uint8_t)(LANGUAGE_POL_BIT | LANGUAGE_RUM_BIT | 
                                LANGUAGE_POR_BIT | LANGUAGE_ESP_BIT | 
                                LANGUAGE_DEU_BIT | LANGUAGE_FRA_BIT | 
                                LANGUAGE_ITA_BIT | LANGUAGE_ENG_BIT),               // 39 -Configurazione lingue abilitate byte 0
LANG_CONFIG1_EDEF       = (uint8_t)(LANGUAGE_SVE_BIT),                              // 40 - Configurazione lingue abilitate byte 1
LANG_CONFIG2_EDEF       = (uint8_t)(0),                                             // 41 - Configurazione lingue abilitate byte 2
LANG_CONFIG3_EDEF       = (uint8_t)(0),                                             // 42 - Configurazione lingue abilitate byte 3
TOT_ENERGY0_EDEF        = (uint8_t)(0),                                             // 43 - energia attiva totale erogata byte 0
TOT_ENERGY1_EDEF        = (uint8_t)(0),                                             // 44 - energia attiva totale erogata byte 1
TOT_ENERGY2_EDEF        = (uint8_t)(0),                                             // 45 - energia attiva totale erogata byte 2
TOT_ENERGY3_EDEF        = (uint8_t)(0),                                             // 46 - energia attiva totale erogata byte 3
STRIP_LED_TYPE_EDEF     = (uint8_t)(LED_STRIP_06),                                  // 47 - tipo di strip led utilizzata
TIME_ZONE_EDEF          = (uint8_t)(1),                                             // 48 - time zone +1 per Italia
DST_FLAG_EDEF           = (uint8_t)(1),                                             // 59 - abilitazione ora legale 1 per Italia (ora legale attiva)
TIME_DST_OFFSET_EDEF    = (uint8_t)(1),                                             // 50 - offset ora legale 1 per Italia (entità ora legale)
DST_DST_EDEF            = (uint8_t)(0),                                             // 51 - ora legale stato corrente 
#ifdef HW_MP28947
LCD_TYPE_EDEF           = ((uint8_t)LCD_NULL | (uint8_t)WIFI_ON),                   // 52 - LCD not present and WIFI enabled
#else
LCD_TYPE_EDEF           = (uint8_t)(LCD_2X20),                                      // 52 - tipo LCD 
#endif
HIDDEN_MENU_VIS_EDEF	= (HIDDEN_MENU_NULL),                                       // 53 - visiblità dei setup [menu nascosto] -> nessun setup visible
HIDDEN_MENU_ENB_EDEF	= (HIDDEN_MENU_NULL),                                       // 54 - abilitazione dei setup [menu nascosto] -> nessun setup abilitato
ENRG_LIMIT_EDEF         = (uint8_t)(10),                                            // 55 - massima energia erogata per la ricarica a energia limitata [KWh]: 0 -> NO LIMIT
SINAPSI_CONFIG_EDEF     = (uint8_t)(0),                                             // 56 - flag di avvenuta configurazione e verifica della prima installazione sinapsi
EMETER_SCU_INT_EDEF     = (EMETER_TYPE_NULL),                                       // 57 -
OPERATIVE_MODE_EDEF     = (uint8_t)(1),                                             // 58 - modo operativo EMUMAX0 = 1 / SEM = 0
TEMP_CTRL_ENB_EDEF      = (uint8_t)(1),                                             // 59 -
TEMP_CTRL_VAL_EDEF      = (uint8_t)(70),                                            // 60 -
TEMP_DELTA_EDEF         = (uint8_t)(0),                                             // 61 -
TEMP_HYSTERESIS_EDEF    = (uint8_t)(2),                                             // 62 -
RS485_ADD_ALIAS_EDEF    = RS485_ADD_EDEF,                                           // 63 - Alias = default address
SEM_FLAGS_CTRL_EDEF     = (uint8_t)(0x01),                                          // 64 - bit0=1=Fixed Address
NOMINAL_POWER_EDEF      = (uint8_t)(74),                                            // 65 - potenza nominale della stazione 7,4KW --> 74 [KW * 10]
CONNECTOR_NUMBER_EDEF   = (uint8_t)(1),                                             // 66 - default connector number 1..4
POST_SUSP_TIME_EDEF     = (uint8_t)(0x00)                                           // 68 - Default is not used 
}eeprom_param_default_val_en;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

typedef struct
{
  uint8_t   key;
  uint8_t   parNum;
}infoEE_t;

// -------------------------- local constats ------------------------------------------------------------------------------------------------------------------------- //
static const uint8_t eeprom_param_board_val[EEPROM_PARAM_NUM] = {EDATA_VALID_EDEF,    EDATA_NUM_EDEF,       SERNUM_BYTE0_EDEF,     SERNUM_BYTE1_EDEF,
                                                                 SERNUM_BYTE2_EDEF,   SERNUM_BYTE3_EDEF,    SOCKET_ENABLE_EDEF,    BATTERY_CONFIG_EDEF,
                                                                 LANGUAGE_EDEF,       RS485_ADD_EDEF,       RTC_VALID_EDEF,        EVS_MODE_EDEF,
                                                                 M3T_CURRENT_EDEF,    M3S_CURRENT_EDEF,     SOCKET_TYPE_EDEF,      EMETER_INT_EDEF,
                                                                 CONTROL_BYTE0_EDEF,  CONTROL_BYTE1_EDEF,   CONTROL_BYTE2_EDEF,    CONTROL_BYTE3_EDEF,
                                                                 ACTUATORS_EDEF,      BLOCK_DIR_EDEF,       PERS_UIDNUM_EDEF,      PERS_MASTER_EDEF,
                                                                 PMNG_MODE_EDEF,      PMNG_EMETER_EDEF,     PMNG_PWRLSB_EDEF,      PMNG_PWRMSB_EDEF,
                                                                 PMNG_ERROR_EDEF,     PMNG_CURRENT_EDEF,    PMNG_MULTIP_EDEF,      PMNG_DMAX_EDEF,
                                                                 PMNG_TRANGE_EDEF,    TCHARGE_MODE_EDEF,    TCHARGE_TIME_EDEF,     PMNG_PWDB0_EDEF,
                                                                 PMNG_PWDB1_EDEF,     PMNG_PWDB2_EDEF,      PMNG_UNBAL_EDEF,       LANG_CONFIG0_EDEF,
                                                                 LANG_CONFIG1_EDEF,   LANG_CONFIG2_EDEF,    LANG_CONFIG3_EDEF,     TOT_ENERGY0_EDEF,
                                                                 TOT_ENERGY1_EDEF,    TOT_ENERGY2_EDEF,     TOT_ENERGY3_EDEF,      STRIP_LED_TYPE_EDEF,  
                                                                 TIME_ZONE_EDEF,      DST_FLAG_EDEF,        TIME_DST_OFFSET_EDEF,  DST_DST_EDEF,        
                                                                 LCD_TYPE_EDEF,       HIDDEN_MENU_VIS_EDEF, HIDDEN_MENU_ENB_EDEF,  ENRG_LIMIT_EDEF,
                                                                 SINAPSI_CONFIG_EDEF, EMETER_SCU_INT_EDEF,  OPERATIVE_MODE_EDEF,   TEMP_CTRL_ENB_EDEF,
                                                                 TEMP_CTRL_VAL_EDEF,  TEMP_DELTA_EDEF,      TEMP_HYSTERESIS_EDEF,  RS485_ADD_ALIAS_EDEF,
                                                                 SEM_FLAGS_CTRL_EDEF, NOMINAL_POWER_EDEF,   CONNECTOR_NUMBER_EDEF, POST_SUSP_TIME_EDEF};

static const uint8_t eeprom_param_master_Iso[EEPROM_PARAM_NUM] = {EDATA_VALID_EDEF,    EDATA_NUM_EDEF,       SERNUM_BYTE0_EDEF,    SERNUM_BYTE1_EDEF,
                                                                 SERNUM_BYTE2_EDEF,   SERNUM_BYTE3_EDEF,    0,                     BATTERY_CONFIG_EDEF,  /* socket disabled */
                                                                 LANGUAGE_EDEF,       0,                    RTC_VALID_EDEF,        EVS_MODE_EDEF,        /* fixed addres 1 */
                                                                 M3T_CURRENT_EDEF,    M3S_CURRENT_EDEF,     SOCKET_TYPE_EDEF,      EMETER_INT_EDEF,
                                                                 0,                   0,                    0,                     0,                    /* no control enabled */
                                                                 0,                   BLOCK_DIR_EDEF,       PERS_UIDNUM_EDEF,      PERS_MASTER_EDEF,     /* no actuator */
                                                                 PMNG_MODE_EDEF,      PMNG_EMETER_EDEF,     PMNG_PWRLSB_EDEF,      PMNG_PWRMSB_EDEF,
                                                                 PMNG_ERROR_EDEF,     PMNG_CURRENT_EDEF,    PMNG_MULTIP_EDEF,      PMNG_DMAX_EDEF,
                                                                 PMNG_TRANGE_EDEF,    TCHARGE_MODE_EDEF,    TCHARGE_TIME_EDEF,     PMNG_PWDB0_EDEF,
                                                                 PMNG_PWDB1_EDEF,     PMNG_PWDB2_EDEF,      PMNG_UNBAL_EDEF,       LANG_CONFIG0_EDEF,
                                                                 LANG_CONFIG1_EDEF,   LANG_CONFIG2_EDEF,    LANG_CONFIG3_EDEF,     TOT_ENERGY0_EDEF,
                                                                 TOT_ENERGY1_EDEF,    TOT_ENERGY2_EDEF,     TOT_ENERGY3_EDEF,      STRIP_LED_TYPE_EDEF,  /* NO LCD */
                                                                 TIME_ZONE_EDEF,      DST_FLAG_EDEF,        TIME_DST_OFFSET_EDEF,  DST_DST_EDEF,        
                                                                 LCD_NULL     ,       HIDDEN_MENU_VIS_EDEF, HIDDEN_MENU_ENB_EDEF,  ENRG_LIMIT_EDEF,
                                                                 SINAPSI_CONFIG_EDEF, EMETER_SCU_INT_EDEF,  SCU_SEM_STAND_ALONE,   TEMP_CTRL_ENB_EDEF,   /* operative mode = SCU_SEM_STAND_ALONE = 4*/
                                                                 TEMP_CTRL_VAL_EDEF,  TEMP_DELTA_EDEF,      TEMP_HYSTERESIS_EDEF,  RS485_ADD_ALIAS_EDEF,
                                                                 SEM_FLAGS_CTRL_EDEF, NOMINAL_POWER_EDEF,   CONNECTOR_NUMBER_EDEF, POST_SUSP_TIME_EDEF};
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
                                                                 
// -------------------------- local references ----------------------------------------------------------------------------------------------------------------------- //
xQueueHandle EEpromMngQueue = NULL;

#ifdef OLD_EEPROM_MANAGEMENT
static EEpromMngMsg_st        EEpromMngMsg;
#endif

static uint8_t                eeprom_param_array[EEPROM_PARAM_NUM];
static uint8_t                eeprom_master_uid_array[CARD_UID_DIM];
static uint8_t                eeprom_user_map_array[USER_MAP_EEDIM];
#ifdef OLD_EEPROM_MANAGEMENT
static uint8_t                eeprom_user_uid_array[CARD_UID_DIM];


static uint8_t                eeprom_do_reset;
static uint8_t                eeprom_uc_reset;

static uint8_t                eeprom_busy;
static uint8_t                eeprom_field_set;

static uint16_t               eadd_value;
static uint16_t               eadd_start;
static uint16_t               eadd_stop;

static uint8_t                *earray_ptr;

static uint8_t                eeprom_update_field;

static uint16_t               eeprom_user_uid_eadd;
#endif

/* Init structure */
SRAM_SCU_Check_t SRAM_SCU_Check = {.BKP_Store = FALSE, .ErrorCnt = 0, .Param = NULL}; 
 
osSemaphoreId EEprom_semaphore;

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local function prototypes -------------------------------------------------------------------------------------------------------------- //
void EEpromManager_init(void);
static void eeprom_read_all(void);
#ifdef OLD_EEPROM_MANAGEMENT 
static uint32_t EEpromManager(EEpromMngMsg_st *pMsg);
#endif
static uint8_t rs485AutoAddress(void);

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

EEprom_BKP_Reg_Copy_st   EEprom_BKP_Reg_Copy;

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //


// -------------------------- external variables ----------------------------------------------------------------------------------------------------------------------//
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

extern uint8_t                 scuAddr;
extern infoStation_t           infoStation;
extern appMapRwRegister_st     appMapRwRegister[SCU_NUM];
extern socketPresence_t        socketPresence;

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  getEEpromMngQueueHandle
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

#ifdef OLD_EEPROM_MANAGEMENT
xQueueHandle getEEpromMngQueueHandle(void)
{
return(EEpromMngQueue);
}
#endif

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  send_to_eeprom
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#ifdef OLD_EEPROM_MANAGEMENT
void send_to_eeprom(uint8_t eeprom_event)
{
EEpromMngMsg_st    msgEEpromSend;

msgEEpromSend.EEpromMngEvent = (EEpromMngEvent_en)(eeprom_event);
configASSERT(xQueueSendToBack(getEEpromMngQueueHandle(), (void *)&msgEEpromSend, portMAX_DELAY) == pdPASS);
}
#endif

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  eeprom_busy_get
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#ifdef OLD_EEPROM_MANAGEMENT
uint8_t eeprom_busy_get(void)
{
return eeprom_busy;
}
#endif

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  eeprom_default_set
//
//  DESCRIPTION:    The current parameter in EEPROM will be new default product parameters
//
//  INPUT:          - pointer in RAM to currents parameters 
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void eeprom_default_set(void)
{
  WriteOnEeprom(EDATA_DEFAULT_EADD, eeprom_param_array, EEPROM_PARAM_NUM);
  WriteOnEeprom(EDATA_DEFAULT_SKT_PRESENCE, (uint8_t*)getDefSocketInfoPtr(), sizeof(socketPresence_t));
  WriteOnEeprom(EDATA_DEFAULT_ID_CODES, (uint8_t*)infoStation.productSn, sizeof(infoStation.productSn));
  WriteOnEeprom(EDATA_DEFAULT_ID_CODES + sizeof(infoStation.productSn), (uint8_t*)infoStation.productCode, sizeof(infoStation.productCode));
  WriteOnEeprom(EDATA_DEFAULT_ID_CODES + sizeof(infoStation.productSn) + sizeof(infoStation.productCode), (uint8_t*)infoStation.fakeProductCode, sizeof(infoStation.fakeProductCode));
  manual_uid_factory();
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  eeprom_board_default
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void eeprom_board_default(void)
{
uint8_t param = EDATA_BOARD_PRG;

WriteOnEeprom(EDATA_VALID_EADD, &param, 1);

TransactionRegister_clearAll();
osDelay(100);
setFlagForNvic();
NVIC_SystemReset();
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  Eeprom_Master_User_card_Force_Reset
//
//  DESCRIPTION:    Force User and Master card reset during the restore to DEFAULT procedure
//
//  INPUT:          none;  
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

void Eeprom_Master_User_card_Force_Reset (void)           /* Fixed ticket SCU-76 */ 
{
  uint8_t   Master_card_reg;          
  
  eeprom_param_get(PERS_MASTER_EADD, &Master_card_reg, 1);  
  Master_card_reg &=~ (0x10 | 0x01);
  eeprom_param_array[PERS_MASTER_EADD] = Master_card_reg;
  WriteOnEeprom((EDATA_DEFAULT_EADD + PERS_MASTER_EADD), &Master_card_reg, 1);   
    
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  eeprom_param_set
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void eeprom_param_set(uint16_t eadd, uint8_t *src_ptr, uint8_t num)
{

#ifdef MODIFICA_EEPROM_MNG
  
uint8_t i, *update_array;
uint16_t j = 0;

if (eadd < MASTER_UID_EEOFFSET)
    {
    j = eadd;
    update_array = eeprom_param_array;
    // xx eeprom_update_field |= EEPROM_CONFIG_UPDATE;
    }
else if (eadd < USER_MAP_EEOFFSET)
    {
    update_array = eeprom_master_uid_array;
    // xx eeprom_update_field |= EEPROM_MASTER_UPDATE;
    }
else if (eadd < USER_UID_EEOFFSET)
    {
    update_array = eeprom_user_map_array;
    // xx eeprom_update_field |= EEPROM_USERMAP_UPDATE;
    }
else
    {
    update_array = eeprom_user_uid_array;
    // xx eeprom_user_uid_eadd = eadd;
    // xx eeprom_update_field |= EEPROM_USERID_UPDATE;
    }

for (i=0; i<num; i++)
    *(update_array + j + i) = *(src_ptr + i);

// xx eeprom_field_set = 1;

Station_Cfg_Update (eadd, &infoStation, src_ptr);

/* Update the backup image for SCU data if needed */
if (SRAM_SCU_Check.BKP_Store)
  BKP_SCU_Image_Store();  

// xx send_to_eeprom(EEPROM_UPDATE);

#else

  switch (eadd)
  {
    case EDATA_VALID_EADD:
      
      break;
    case EDATA_NUM_EADD:        
      break;
    case SERNUM_BYTE0_EADD:     
    case SERNUM_BYTE1_EADD:     
    case SERNUM_BYTE2_EADD:     
    case SERNUM_BYTE3_EADD:     
      break;
    case SOCKET_ENABLE_EADD:    
      break;
    case BATTERY_CONFIG_EADD:   
      break;
    case LANG_DEFAULT_EADD:     
      break;
    case RS485_ADD_EADD:        
      break;
    case RTC_VALID_EADD:        
      break;
    case EVS_MODE_EADD:         
      break;
    case M3T_CURRENT_EADD:      
      break;
    case M3S_CURRENT_EADD:      
      break;
    case SOCKET_TYPE_EADD:      
      break;
    case EMETER_INT_EADD:       
      break;
    case CONTROL_BYTE0_EADD:    
    case CONTROL_BYTE1_EADD:    
    case CONTROL_BYTE2_EADD:    
    case CONTROL_BYTE3_EADD:    
      break;
    case ACTUATORS_EADD:        
      break;
    case BLOCK_DIR_EADD:        
      break;
    case PERS_UIDNUM_EADD:      
      break;
    case PERS_MASTER_EADD:      
      break;
    case PMNG_MODE_EADD:        
      break;
    case PMNG_EMETER_EADD:      
      break;
    case PMNG_PWRLSB_EADD:      
      break;
    case PMNG_PWRMSB_EADD:      
      break;
    case PMNG_ERROR_EADD:       
      break;
    case PMNG_CURRENT_EADD:     
      break;
    case PMNG_MULTIP_EADD:      
      break;
    case PMNG_DMAX_EADD:        
      break;
    case PMNG_TRANGE_EADD:      
      break;
    case TCHARGE_MODE_EADD:     
      break;
    case TCHARGE_TIME_EADD:     
      break;
    case PMNG_PWDB0_EADD:       
      break;
    case PMNG_PWDB1_EADD:       
      break;
    case PMNG_PWDB2_EADD:       
      break;
    case PMNG_UNBAL_EADD:       
      break;
    case LANG_CONFIG0_EADD:     
      break;
    case LANG_CONFIG1_EADD:     
      break;
    case LANG_CONFIG2_EADD:     
      break;
    case LANG_CONFIG3_EADD:     
      break;
    case TOT_ENERGY0_EADD:      
      break;
    case TOT_ENERGY1_EADD:      
    case TOT_ENERGY2_EADD:      
    case TOT_ENERGY3_EADD:      
      break;
    case STRIP_LED_TYPE_EADD:   
      break;
    case TIME_ZONE_EADD:        
      break;
    case DST_EADD:              
      break;
    case TIME_DST_OFFSET_EADD:  
      break;
    case DST_STATUS_EADD:       
      break;
    case LCD_TYPE_EADD:         
      break;
    case HIDDEN_MENU_VIS_EADD:  
      break;
    case HIDDEN_MENU_ENB_EADD:  
      break;
    case ENRG_LIMIT_EADD:       
      break;
    case SINAPSI_INST_EADD:     
      break;
    case EMETER_SCU_INT_EADD:   
      break;
    case OPERATIVE_MODE_EADD:   
      break;
    case TEMP_CTRL_ENB_EADD:    
      break;
    case TEMP_CTRL_VAL_EADD:    
      break;
    case TEMP_DELTA_EADD:       
      break;
    case TEMP_HYSTERESIS_EADD:  
      break;
    case RS485_ADD_ALIAS_EADD:  
      break;
    case SEM_FLAGS_CTRL_EADD:   
      break;
    case STATION_NOM_PWR_EADD:  
      break;
    case CONNECTOR_NUMBER_EADD: 
      break;
    case POST_SUSP_TIME_EADD:   
      break;
    default:
      break;
     
  }


#endif



}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  eeprom_array_set
//
//  DESCRIPTION:    Set eeprom_array in RAM, are the parameters to store in
//                  eeprom
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#ifdef OLD_EEPROM_MANAGEMENT
void eeprom_array_set(uint16_t eadd, uint8_t *src_ptr, uint8_t num)
{
  
  uint8_t i, *update_array;
  uint16_t j = 0;
  
  /* The region is about the parameters in eeprom? */ 
  if (eadd < MASTER_UID_EEOFFSET)   
  {
      /* Prepare the array pointers to update */
      j = eadd;
      update_array = eeprom_param_array;
      eeprom_update_field |= EEPROM_CONFIG_UPDATE;
      /* Update the array in RAM */
      for (i = 0; i < num; i++)
        *(update_array + j + i) = *(src_ptr + i);
  
      eeprom_field_set = 1;      
  }
    
}
#endif

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  eeprom_param_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void eeprom_param_get(uint16_t eadd, uint8_t *dst_ptr, uint8_t num)
{
uint8_t i;

  for (i=0; i<num; i++)
  {
      *(dst_ptr + i) = eeprom_param_array[(eadd + i)];
  }
#ifdef IGNORE_VBUS
  if (eadd == CONTROL_BYTE1_EADD) *dst_ptr &= (~VBUS_CRL1);
#endif
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  eeprom_master_uid_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void eeprom_master_uid_get(uint8_t *dst_ptr)
{
uint8_t i;

for (i=0; i<CARD_UID_DIM; i++)
    *(dst_ptr + i) = eeprom_master_uid_array[i]; 
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  eeprom_uid_map_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void eeprom_uid_map_get(uint8_t *map_ptr)
{
uint8_t i;

for (i=0; i<USER_MAP_EEDIM; i++)
    *(map_ptr + i) = eeprom_user_map_array[i];
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  eeprom_uid_reg_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void eeprom_uid_reg_get(uint16_t uid_add, uint8_t *uid_ptr)
{
ReadFromEeprom((USER_UID_EEOFFSET + uid_add), uid_ptr, CARD_UID_DIM);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  eeprom_ProductConfig_Param_Set
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void eeprom_ProductConfig_Param_Set (void)
{
  WriteOnEeprom(EDATA_VALID_EADD, eeprom_param_array, EEPROM_PARAM_NUM);
}


/**
*
* @brief        Automatic RS485 bus address 
*
* @param [out]  none
*
* @retval       uint8_t: address assigned 
*
*******************************************************************************/
static uint8_t rs485AutoAddress(void)
{
  uint8_t  addr, ck8;

  addr = (uint8_t)0xFF;

  if (getAddrSetting() == 0) 
  {
    /* no address setting so autodetect it */
    if (sbcPresence())
    {
      /* in this case the address on RS485 bus if 1 */
      addr = 0;  /* this means address 1 on RS485 bus */
    }
    else
    {
      ck8 = (uint8_t)cpuIdCheksum16();
      addr = (((0x0F & ck8) + ((0xF0 & ck8) >>4)) & 0x0F);
      if (addr == 0)
      {
        addr++;
      }
    }
    setFlagHwInfo((RS485_ADD_SET | KEY_FOR_RS485_ADD), MASK_FOR_RS485_ADD_SET);
    /* a new address must be set */
    //eeprom_param_set(RS485_ADD_EADD, (uint8_t*)&addr, 1);
    //WriteOnEeprom(RS485_ADD_EADD, (uint8_t*)&addr, 1);
  }
  return (addr);
}

/**
*
* @brief        BKP_SCU_Image_Store 
*
*               Create BKP image for SCU data 
*
* @param [out]  none
*
* @retval       none
*
*******************************************************************************/

void BKP_SCU_Image_Store (void)
{
    uint16_t ee_data;
    /* Write infoStation data in BKP area */
    WriteOnEeprom (EDATA_BKP_SCU_EE_ADDRESS, (uint8_t *)&infoStation, sizeof (infoStation_t));        
    /* Write infoStation data in BKP area */
    WriteOnEeprom (EDATA_BKP_SEM_EE_ADDRESS, (uint8_t *)&socketPresence, sizeof (socketPresence_t));                    
    /* Save scuAddr for SEM management */
    WriteOnEeprom(EDATA_BKP_RS485_ADDR_EE_ADDRESS, (uint8_t*)&scuAddr, 1);    
    /* Write signature */
    ee_data = EDATA_BKP_SCU_SIGNATURE;
    WriteOnEeprom (EDATA_BKP_SCU_EE_ADDRESS + sizeof (infoStation_t), (uint8_t *)&ee_data, sizeof (uint16_t));    
    SRAM_SCU_Check.BKP_Store = FALSE;
    
    tPrintf ("BKP image stored in eeprom\r\n");
    
    EVLOG_Message (EV_INFO, "BKP image for infostation and SEM data stored in eeprom");      
    
}

/**
*
* @brief        Addr_MatrixConv_IsNull 
*
*              Check if Matrix conversion struct is NUL (all values to 0xFF) 
*
* @param [out]  none
*
* @retval       TRUE/FALSE
*
*******************************************************************************/

static bool Addr_MatrixConv_IsNull (void)
{
  uint8_t cnt, cnt1;
  
  /* check matrixConv */
  for (cnt = 0; cnt < SCU_NUM; cnt++)
    if (socketPresence.matrixConv[cnt] != 0xFF)
      break;
  
  /* check matrixIdConn */
  for (cnt1 = 0; cnt1 < SCU_NUM; cnt1++)
    if (socketPresence.matrixIdConn[cnt1] != 0xFF)
      break;  
  
  /* Check if the array are not initialized */
  if ((cnt == SCU_NUM) && (cnt1 == SCU_NUM))
    return TRUE;
  return FALSE;  
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  eeprom_read_all
//
//  DESCRIPTION:    read all eeprom data
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void eeprom_read_all(void)
{
uint8_t     valid, ee_data;
uint16_t    i, error, ee_data16;
infoEE_t    infoEE;
uint8_t     *pBuff;

uint8_t Product_SN_temp[PRODUCT_SN_LENGTH], Product_Code_temp[PRODUCT_CODE_LENGTH], FakeProduct_Code_temp[FAKE_CODE_LENGTH];
                                           
if(osSemaphoreAcquire(EEprom_semaphore, portMAX_DELAY) == osOK)
{

  if (ReadFromEeprom(EDATA_VALID_EADD, (uint8_t*)&infoEE, 2) != osOK)
  {
    /* Impossible to read the key --> reset uP and restart */
    EVLOG_Message (EV_ERROR, "Error reading the EEPROM, force a RESET");      
    setFlagForNvic();
    NVIC_SystemReset();            
  }
      
  valid = infoEE.key; 
  
  (void)rs485AutoAddress();

  /* Check the key and parameter number to find if the dataset is old */  
  if ((infoEE.key == EDATA_VALID_PRG) && 
      ((infoEE.parNum >= EEPROM_PARAM_NUM_V421) && (infoEE.parNum <= EEPROM_PARAM_NUM_V423))) 
  {
    
    EVLOG_Message (EV_INFO, "Data in EEPROM have a different format");      

    /* is necessary to move RFID CARD info in new location */
    if (infoEE.parNum == EEPROM_PARAM_NUM_V421) i = RFID_CARD_SIZE_V421;
    if (infoEE.parNum > EEPROM_PARAM_NUM_V421) i = RFID_CARD_SIZE_V423;
    /* read current data */
    pBuff = malloc(i);
    ReadFromEeprom(SCU_MASTER_UID_EEOFFSET_v4_2_1_ADDRES, pBuff, i);
    /* and save it in new location */
    WriteOnEeprom(MASTER_UID_EEOFFSET, pBuff, i);
  
    if (infoEE.parNum == EEPROM_PARAM_NUM_V421)
    {
      /* In previous version, the general info parameters are stored in a different area in eeprom, this generate 
      an incompatibility during the update.
      So we need to test if at address 0x200 there are some parameters stored 
      Fixed ticket SCU-77                                                            */
      ReadFromEeprom(SCU_GENERAL_INFO_EE_v4_2_1_ADDRES, pBuff, sizeof(infoStation_t));
      /* and save it in new location */
      WriteOnEeprom(SCU_GENERAL_INFO_EE_ADDRES, pBuff, sizeof(infoStation_t));
    }
    free(pBuff);
  }
  
  /* Check validity key */
  if (valid == EDATA_VALID_PRG)                                                             // ALL DATA VALID
  {
    
      EVLOG_Message (EV_INFO, "Data in EEPROM are valid");      

      ReadFromEeprom(MASTER_UID00_EADD, eeprom_master_uid_array, CARD_UID_DIM);
      ReadFromEeprom(USER_MAP00_EADD, eeprom_user_map_array, USER_MAP_EEDIM);
  
      ReadFromEeprom(EDATA_NUM_EADD, &ee_data, 1);
  
      /* number of parameter stored is a subset of the actual ? */
      if (ee_data < EEPROM_PARAM_NUM)
      {          
          /* Read the entire SUBSET */
          ReadFromEeprom(EDATA_NUM_EADD, &eeprom_param_array[EDATA_NUM_EADD], ee_data);   
          /* Set the new set of params with a default value */
          for (i=ee_data; i<EEPROM_PARAM_NUM; i++)
              eeprom_param_array[i] = eeprom_param_board_val[i];
          /* Realign the number of parameters */
          eeprom_param_array[EDATA_NUM_EADD] = EEPROM_PARAM_NUM;          
          /* Validate data */
          eeprom_param_array[EDATA_VALID_EADD] = EDATA_VALID_PRG;
          /* Store the new set of params */
          WriteOnEeprom(EDATA_VALID_EADD, &eeprom_param_array[EDATA_VALID_EADD], EEPROM_PARAM_NUM);
      }
      else
        /* Read the entire SET */
        ReadFromEeprom(EDATA_VALID_EADD, &eeprom_param_array[EDATA_VALID_EADD], EEPROM_PARAM_NUM);   
      
      /* Read backup area for SCU data backup */
      ReadFromEeprom(EDATA_BKP_SCU_EE_ADDRESS + sizeof (infoStation_t), (uint8_t *)&ee_data16, sizeof (uint16_t));
      /* Check if BKP image is present or not */
      if (ee_data16 != EDATA_BKP_SCU_SIGNATURE)
        SRAM_SCU_Check.BKP_Store = TRUE;                  /* The BKP image for SCU data is missing, create it! */
      
  }
  else if (valid == EDATA_FACTORY_PRG)                                                    // FACTORY RESTORE  
  {
       
      EVLOG_Message (EV_INFO, "Data in EEPROM have been reset to the FACTORY values");      

      error = 0;  
      
      if (ReadFromEeprom(EDATA_DEFAULT_EADD, eeprom_param_array, EEPROM_PARAM_NUM) != osOK)    // la lettura default di prodotto non va
        error = 1;
      
      if (error || ((eeprom_param_array[EDATA_VALID_EADD] == 0xFF ) && (eeprom_param_array[EDATA_NUM_EADD] == 0xFF)))   /* if there is an error reading DEFAULT or data are not initialized, do it (fixed tcket SCU-75) */
      {
          for (i=0; i<EEPROM_PARAM_NUM; i++)
              eeprom_param_array[i] = eeprom_param_board_val[i];                /* Fill the structure with DEFAULT data */
      }
        
      eeprom_param_array[BLOCK_DIR_EADD] = BLOCK_DIR_EDEF;
        
      do
      {
        error = (uint8_t)0;
    
        pBuff = (uint8_t*)malloc(EDATA_LAST_BYTE);   // cancella da EDATA_VALID_EADD a EDATA_DEFAULT_EADD escluso
        memset((void*)pBuff, 0xFF, EDATA_LAST_BYTE); 

        if (WriteOnEeprom((uint16_t)EDATA_VALID_EADD, pBuff, (uint16_t)EDATA_LAST_BYTE) == 0)
        {
        if (ReadFromEeprom(EDATA_VALID_EADD, pBuff, EDATA_DEFAULT_EADD) == osOK)    // rileggo la eeprom
            {
            for (i = 0; i < EDATA_LAST_BYTE; i++)
                {
                if (pBuff[i] != (uint8_t)0xFF)
                    error++;
                }
            }
        }
      } while (error != (uint8_t)0);
  
      /* clear energy value This is important for EM Scame */
      // xx eeprom_array_set(TOT_ENERGY0_EADD, (uint8_t*)&eeprom_param_board_val[TOT_ENERGY0_EADD], 4);
      EEPROM_Save_Config (TOT_ENERGY0_EADD, (uint8_t*)&eeprom_param_board_val[TOT_ENERGY0_EADD], 4);
      /* Store data in eeprom */
      WriteOnEeprom(EDATA_VALID_EADD, eeprom_param_array, EEPROM_PARAM_NUM);
  
#ifdef COME_ERA
      if (eeprom_param_array[PERS_MASTER_EADD] & 0x01)
        ReadFromEeprom((EDATA_DEFAULT_EADD + MASTER_UID00_EADD), eeprom_master_uid_array, CARD_UID_DIM);
      else
      {
        for (i=0; i<USER_MAP_EEDIM; i++)
          eeprom_master_uid_array[i] = 0xFF;
      }
  
      WriteOnEeprom(MASTER_UID00_EADD, eeprom_master_uid_array, CARD_UID_DIM);
  
      if (eeprom_param_array[PERS_MASTER_EADD] & 0x10)
      {
        ReadFromEeprom((EDATA_DEFAULT_EADD + USER_MAP00_EADD), eeprom_user_map_array, USER_MAP_EEDIM);
  
        ReadFromEeprom((EDATA_DEFAULT_EADD + USER_UID_EEOFFSET), pBuff, USER_UID_BYTE_NUM);    // rileggo user uid factory default
        WriteOnEeprom(USER_UID_EEOFFSET, pBuff, USER_UID_BYTE_NUM);
      }
      else
      {
        for (i=0; i<USER_MAP_EEDIM; i++)
          eeprom_user_map_array[i] = 0x00;
      }
#else
      /* 23/04/2025 Nick: factory restore must delete master card UID and all UID user card */
      eeprom_param_array[PERS_MASTER_EADD] = eeprom_param_array[EVS_MODE_EADD] = eeprom_param_array[PERS_UIDNUM_EADD] = 0;
      for (i=0; i<CARD_UID_DIM; i++) 
      {
        eeprom_master_uid_array[i] = 0xFF;
      }
      for (i=0; i<USER_MAP_EEDIM; i++) 
      {
        eeprom_user_map_array[i] = 0x00;
      }
      WriteOnEeprom(MASTER_UID00_EADD, eeprom_master_uid_array, CARD_UID_DIM);           
#endif  
  
      WriteOnEeprom(USER_MAP00_EADD, eeprom_user_map_array, USER_MAP_EEDIM);
      ee_data = 0;
      WriteOnEeprom(PERS_UIDNUM_EADD, &ee_data, 1);     // cancello in eeprom il numero UID registrati - Fixed SCU-76
      WriteOnEeprom(SOCKETS_PRESENCE_EE_ADDRES, (uint8_t*)getDefSocketInfoPtr(), sizeof(socketPresence_t));
            
      /* Restore factory ID copying them from EDATA_DEFAULT_ID_CODES address */
      ReadFromEeprom(EDATA_DEFAULT_ID_CODES, (uint8_t*)Product_SN_temp, sizeof(Product_SN_temp));
      ReadFromEeprom(EDATA_DEFAULT_ID_CODES + sizeof(Product_SN_temp), (uint8_t*)Product_Code_temp, sizeof(Product_Code_temp));
      ReadFromEeprom(EDATA_DEFAULT_ID_CODES + sizeof(Product_SN_temp) + sizeof(Product_Code_temp), (uint8_t*)FakeProduct_Code_temp, sizeof(FakeProduct_Code_temp));
      WriteOnEeprom(PRD_SN_EE_ADDRES, (uint8_t*)Product_SN_temp, sizeof(Product_SN_temp));
      WriteOnEeprom(PRD_CODE_EE_ADDRES, (uint8_t*)Product_Code_temp, sizeof(Product_Code_temp));
      memcpy(infoStation.productSn, Product_SN_temp, sizeof(Product_SN_temp));
      memcpy(infoStation.productCode, Product_Code_temp, sizeof(Product_Code_temp));
      memcpy(infoStation.fakeProductCode, FakeProduct_Code_temp, sizeof(FakeProduct_Code_temp));
      /* APP has to be activated */
      infoStation.socketActivatedFlag = 0;

      /* Update full infostation structure in EEPROM */
      WriteOnEeprom(SCU_GENERAL_INFO_EE_ADDRES, (uint8_t*)&infoStation, sizeof(infoStation));
      
      SRAM_SCU_Check.BKP_Store = TRUE;  /* The BKP image for SCU data must be updated */
  
  }
  else    // if ((valid == 0xFF) || (valid == EDATA_BOARD_PRG))                                    // DEFAULT DATA RESTORE 
  {
    
      EVLOG_Message (EV_INFO, "Data in EEPROM has been reset to the DEFAULT values");      

      if (ReadFromEeprom(SERNUM_BYTE0_EADD, &eeprom_param_array[SERNUM_BYTE0_EADD], 4) != osOK)    // recupero numero di serie
      {
        for (i=0; i<4; i++)
          eeprom_param_array[(SERNUM_BYTE0_EADD + i)] = eeprom_param_board_val[(SERNUM_BYTE0_EADD + i)];
      }
      
      do
      {
        error = (uint8_t)0;
    
        pBuff = (uint8_t*)malloc(EDATA_DEFAULT_EADD);   // cancella da EDATA_VALID_EADD a EDATA_DEFAULT_EADD escluso
        memset((void*)pBuff, 0xFF, EDATA_DEFAULT_EADD); 
    
        if (WriteOnEeprom((uint16_t)EDATA_VALID_EADD, pBuff, (uint16_t)EDATA_DEFAULT_EADD) == 0)
        {
          if (ReadFromEeprom(EDATA_VALID_EADD, pBuff, EDATA_DEFAULT_EADD) == osOK)    // rileggo la eeprom
          {
            for (i = 0; i < EDATA_DEFAULT_EADD; i++)
            {
              if (pBuff[i] != (uint8_t)0xFF)
                error++;
            }
          }
        }
      } while (error != (uint8_t)0);
  
      
      /* Overwrite eeprom_param_array with DEFAULT data , exclude serial number */
      for (i=0; i<EEPROM_PARAM_NUM; i++)
      {
        if ((i < SERNUM_BYTE0_EADD) || (i > SERNUM_BYTE3_EADD))
          eeprom_param_array[i] = eeprom_param_board_val[i];
      }
      /* Write on EEPROM */ 
      if (WriteOnEeprom(EDATA_VALID_EADD, eeprom_param_array, EEPROM_PARAM_NUM) == osOK)
      {     
        ee_data = EDATA_VALID_PRG;   /* NO ERRORS, try to write the validation value */
        if (WriteOnEeprom(EDATA_VALID_EADD, &ee_data, 1) == osOK)
          eeprom_param_array[EDATA_VALID_EADD] = EDATA_VALID_PRG;  /* NO ERRORS --> block validated */
        else
          error = TRUE;
      }
      else
          error = TRUE;
  
      /* if error writing default data */
      if (error)
      {
        /* reset uP and restart */
        setFlagForNvic();
        NVIC_SystemReset();        
      }
  
      for (i=0; i<CARD_UID_DIM; i++)
          eeprom_master_uid_array[i] = 0xFF;
  
      WriteOnEeprom(USER_MAP00_EADD, eeprom_master_uid_array, CARD_UID_DIM);
  
      for (i=0; i<USER_MAP_EEDIM; i++)
          eeprom_user_map_array[i] = 0x00;
  
      WriteOnEeprom(USER_MAP00_EADD, eeprom_user_map_array, USER_MAP_EEDIM);
      
      SRAM_SCU_Check.BKP_Store = TRUE;  /* The BKP image for SCU data must be updated */
  
  }
  
  evs_control_save();
  
  /* Sinapsi enabled? init variables */
  if (getSinapsiEepromEn() == ENABLED)
    evs_chn2_init();
  
  do
  {
    ;
  } while (readSocketPresence() != osOK);
    
  /* Check if matrixConv in socket presence is NULL for some reason */
  if (Addr_MatrixConv_IsNull())
  {
    /* set default value for socket presence SEM structure (baco hard fault dovuto alla corruzione di freeRTos in getLogicalMdbAddrSem()  */
    /* return NULL_ID calling fromRs485ToSem()  when colling setGeneralStationParameters() at line 829                                    */
    memcpy ((uint8_t*)&socketPresence, (uint8_t*)getDefSocketInfoPtr(), sizeof(socketPresence_t));
    WriteOnEeprom(SOCKETS_PRESENCE_EE_ADDRES, (uint8_t*)getDefSocketInfoPtr(), sizeof(socketPresence_t));     
  }
  
  /* Restore SCU type working mode  */
  setScuTypeModeFromEeprom();  
  setGeneralStationParameters(valid);
  Scheduler_scheduleCharge(getSchedulationFromMemory());
  setNominalPower(eeprom_param_array[M3T_CURRENT_EADD]);
  SecureArea_init();
  
  /* Update the backup image for SCU data if needed */
  if (SRAM_SCU_Check.BKP_Store)
    BKP_SCU_Image_Store();  
}

/* release the semaphore */
osSemaphoreRelease(EEprom_semaphore); 

}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  EEpromManager_init
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

void EEpromManager_init(void)
{
#ifdef OLD_EEPROM_MANAGEMENT 
eeprom_uc_reset = 0;
eeprom_do_reset = 0;
eeprom_busy = 0;
eeprom_field_set = 0;
eeprom_update_field = EEPROM_NULL_UPDATE;
#endif
/* Create semaphore */
EEprom_semaphore = osSemaphoreNew(1, 1, NULL);

eeprom_read_all();
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  EEpromManager
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#ifdef OLD_EEPROM_MANAGEMENT 
static uint32_t EEpromManager(EEpromMngMsg_st *pMsg)
{
uint8_t     eget_value;
uint32_t    newTimeTick;
uint8_t*    pEEarea;
uint16_t    bkLen, eeIdx;
uint8_t     eeArea[128];

eadd_stop = (uint16_t)0;  /* init address stop */

if (pMsg->EEpromMngEvent == EEPROM_UPDATE)
    {
    eeprom_busy = 1;

    if (eeprom_field_set == 1)
        {
        if (eeprom_update_field & EEPROM_MASTER_UPDATE)             // gli aggiornamenti dei campi sottostanti sono posticipati
            {
            earray_ptr = eeprom_master_uid_array;
            eadd_start = MASTER_UID00_EADD;
            eadd_stop = (MASTER_UID00_EADD + CARD_UID_DIM);
            }
        else if (eeprom_update_field & EEPROM_USERMAP_UPDATE)       // gli aggiornamenti dei campi sottostanti sono posticipati
            {
            earray_ptr = eeprom_user_map_array;
            eadd_start = USER_MAP00_EADD;
            eadd_stop = (USER_MAP00_EADD + USER_MAP_EEDIM);
            }
        else if (eeprom_update_field & EEPROM_USERID_UPDATE)        // l'aggiornamento del campo sottostante è posticipato
            {
            earray_ptr = eeprom_user_uid_array;
            eadd_start = eeprom_user_uid_eadd;
            eadd_stop = (eeprom_user_uid_eadd + CARD_UID_DIM);
            }
        else    // if (eeprom_update_field & EEPROM_CONFIG_UPDATE)  // aggiornamento a più bassa priorità temporale
            {
            earray_ptr = eeprom_param_array;
            eadd_start = EDATA_VALID_EADD;
            eadd_stop = (EDATA_VALID_EADD + EEPROM_PARAM_NUM);
            }
    
        eadd_value = eadd_start;
        eeprom_field_set = 0;
        }

    }
  
        
if ((eadd_stop != (uint16_t)0) && (eadd_stop > eadd_start))
    {
    eeIdx = 0;
    bkLen = (uint16_t)(eadd_stop - eadd_start);
    pEEarea = (uint8_t*)eeArea;
  
    if (ReadFromEeprom(eadd_value, pEEarea, bkLen) == osOK)
        {
        do
            {
            eget_value = pEEarea[eeIdx];
            
            if (eget_value == *earray_ptr)                  // (eadd_value - eadd_start) = riallineamento fra indirizzo reale eeprom e earray_ptr
                {
                if (eeprom_do_reset == 1)
                {
                  setFlagForNvic();
                  NVIC_SystemReset();
                }

                eadd_value ++;
                earray_ptr ++;
                eeIdx++;

                if (eadd_value == eadd_stop)                // finito scroll degli indirizzi
                    {
                    if (eeprom_update_field & EEPROM_MASTER_UPDATE)
                        eeprom_update_field &=~ EEPROM_MASTER_UPDATE;
                    else if (eeprom_update_field & EEPROM_USERMAP_UPDATE)
                        eeprom_update_field &=~ EEPROM_USERMAP_UPDATE;
                    else if (eeprom_update_field & EEPROM_USERID_UPDATE)
                        eeprom_update_field &=~ EEPROM_USERID_UPDATE;
                    else    // if (eeprom_update_field & EEPROM_CONFIG_UPDATE)
                        eeprom_update_field &=~ EEPROM_CONFIG_UPDATE;
                
                    if (eeprom_update_field == EEPROM_NULL_UPDATE)
                        {
                        eeprom_busy = 0;
                
                        if (eeprom_uc_reset == 1)
                        {
                          setFlagForNvic();
                          NVIC_SystemReset();
                        }
                        }
                    else
                        {
                        eeprom_field_set = 1;
                        send_to_eeprom(EEPROM_UPDATE);
                        }
                
                    newTimeTick = portMAX_DELAY;
                    }
                else
                    {
                    }
                }
            else
                {
                WriteOnEeprom(eadd_value, earray_ptr, 1);   // (eadd_value - eadd_start) = riallineamento fra indirizzo reale eeprom e earray_ptr
                pEEarea[eeIdx] = *earray_ptr;               // store new value in malloc area also

                if (eadd_value == EDATA_VALID_EADD)
                    eeprom_do_reset = 1;
                }
            }while (eadd_value != eadd_stop);
        }
    else
        {
        newTimeTick = pdMS_TO_TICKS(100);               // errore di lettura: si ritenta dopo 100ms
        }
    } 

/* Update the backup image for SCU data if needed */
if (SRAM_SCU_Check.BKP_Store)
  BKP_SCU_Image_Store();  

/* Check if a wifi procedure is ongoing in order to skip the InitModbusRegister function call inside the function below  (Fixed ticket SCU-82) */
if ((eeprom_busy == 0) && (InitModbusRegisters_Stop == FALSE))
{
  setGeneralStationParameters(EDATA_VALID_PRG);
}
else
  /* free the semaphore */
  InitModbusRegisters_Stop = FALSE;        


return newTimeTick;
}

#endif

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  EEpromMngTask
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void eeprom_uc_reset_set(void)
{
#ifdef OLD_EEPROM_MANAGEMENT  
  eeprom_uc_reset = 1;
#else
  setFlagForNvic();
  NVIC_SystemReset();
#endif
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

#ifdef OLD_EEPROM_MANAGEMENT

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  EEpromMngTask
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void EEpromMngTask(void *pvParameters)
{
uint32_t timeTick;

/* init task */

/*-------- Creates an empty mailbox for EEpromMngTask messages --------------------------*/
EEpromMngQueue = xQueueCreate(4, sizeof(EEpromMngMsg_st));  
configASSERT(EEpromMngQueue != NULL);

/* Create semaphore */
EEprom_semaphore = osSemaphoreNew(1, 1, NULL);

timeTick = portMAX_DELAY;
send_to_eeprom(EEPROM_EVENT_INIT);

/* start task */

for (;;)
    {
    /* Wait for some event from SW */
    if (xQueueReceive(EEpromMngQueue, (void *)&EEpromMngMsg, timeTick) == pdPASS)
        {
        if (EEpromMngMsg.EEpromMngEvent == EEPROM_EVENT_INIT)
        {
            EEpromManager_init();
        }
        else
            {
            timeTick = EEpromManager(&EEpromMngMsg);
            }
        }
    else
        {
        /* Wait for possible handled timeout */
        EEpromMngMsg.EEpromMngEvent = EEPROM_TIMEOUT;
        timeTick = EEpromManager(&EEpromMngMsg);
        }
    }
}

#endif

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
/**
*
* @brief       get eeprom array value 
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
uint8_t*  getEepromArray(void) 
{
  return(eeprom_param_array);
}

/**
*
* @brief       get eeprom array value 
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  setEepromArrayIsolatedMode(void) 
{
  uint8_t   i, SerNum[4];

  eeprom_param_get(SERNUM_BYTE0_EADD, SerNum, 4);

  for (i=0; i<EEPROM_PARAM_NUM; i++)
      eeprom_param_array[i] = eeprom_param_master_Iso[i];                /* Fill the structure with DEFAULT data for isolated mode */

  EEPROM_Save_Config (SERNUM_BYTE0_EADD, SerNum, 4);
}

/**
*
* @brief       SCU_Data_Check
*
* @param [in]  none
*  
* @retval      error on check type (ERROR_ON_SRAM_DATA - NO_ERROR_ON_SRAM_DATA)
*  
****************************************************************/
error_check_data_en SCU_Data_Check(void)
{

  uint8_t cnt, Product_SN_SOBEM = false;

  /* #1 --> Check RS485 address data */
  SRAM_SCU_Check.Param = ScuADDR;
  if (scuAddr >= SCU_NUM)
    return ERROR_ON_SRAM_DATA;
  /* Check infoStation data */
  
  /* #2 --> Check SCU SN data */
  SRAM_SCU_Check.Param = Serial;  
  for (cnt = 0; cnt < BOARD_SN_LENGTH; cnt++)
  {
     /* check if is a number */
    if ((infoStation.serial[cnt] >= '0') && (infoStation.serial[cnt] <= '9'))
      continue;
    if (infoStation.serial[cnt] == 'F')     /* DEFAULT value */
      continue;
    if ((infoStation.serial[cnt] == 0) && (cnt == BOARD_SN_LENGTH - 1)) /* Termination char */
      continue;
    return ERROR_ON_SRAM_DATA;
  }
    
  /* Check User pin only if Wifi is enabled (Espressif module is enabled) */
  if (HAL_GPIO_ReadPin(WIFI_EN_GPIO_Port, WIFI_EN_Pin) == GPIO_PIN_SET)
  {
    
    /* Only if socket has been activated, check user Pin, otherwise is filled with 0xFF */
    if (infoStation.socketActivatedFlag == TRUE)
    {
      /* #3 --> Check User pin data */
      SRAM_SCU_Check.Param = UserPin;        
      for (cnt = 0; cnt < USER_PIN_LENGTH; cnt++)
      {
        /* check if is a number */
        if ((infoStation.userPin[cnt] >= '0') && (infoStation.userPin[cnt] <= '9'))  
          continue;
        if ((infoStation.userPin[cnt] == 0) && (cnt == USER_PIN_LENGTH - 1))    /* Termination char */
          continue;
        return ERROR_ON_SRAM_DATA;
      }
    }
    
    /* Check Router ssid data (only if connected to a router)*/
    if (appMapRwRegister[scuAddr].connectStatus == 1 )
    {
      SRAM_SCU_Check.Param = RouterSsid;        
      for (cnt = 0; cnt < MAX_ROUTER_SSID_LENGTH; cnt++)
      {
        /* #4 --> check if ssid is empty or corrupted */
        if (infoStation.routerSsid[cnt] == 0xFF)  
          return ERROR_ON_SRAM_DATA;    
        if (infoStation.routerSsid[cnt] == 0)  /* Termination char */
          break;      
      }
      
      SRAM_SCU_Check.Param = RouterPass;        
      for (cnt = 0; cnt < MAX_ROUTER_PASS_LENGTH; cnt++)
      {
        /* #5 --> check if is password is empty */
        if (infoStation.routerPass[cnt] == 0xFF)  
          return ERROR_ON_SRAM_DATA;
        if (infoStation.routerPass[cnt] == 0)  /* Termination char */
          break;      
      }
    }   
  }
  
  /* #6 --> Check Product SN data */
  SRAM_SCU_Check.Param = ProductSN;          
  for (cnt = 0; cnt < PRODUCT_SN_LENGTH; cnt++)
  {
     /* check if is a number */
    if ((infoStation.productSn[cnt] >= '0') && (infoStation.productSn[cnt] <= '9'))
      continue;
    if ((infoStation.productSn[cnt] == 0) && (cnt == PRODUCT_SN_LENGTH - 1)) /* Termination char */
      continue;
    if (infoStation.productSn[0] == '5')
      Product_SN_SOBEM = true;
    if ((infoStation.productSn[9] == '/') && (Product_SN_SOBEM == true))
      continue;
    return ERROR_ON_SRAM_DATA;
  }
  /* #7 --> Check Product code data */
  SRAM_SCU_Check.Param = ProductCode;          
  for (cnt = 0; cnt < PRODUCT_CODE_LENGTH; cnt++)
  {
    /* Check Ascii char admitted */
    if ((infoStation.productCode[cnt] >= '0') && (infoStation.productCode[cnt] <= '9'))      /* Numbers from 0 t0 9*/
      continue;
    else if ((infoStation.productCode[cnt] >= 'A') && (infoStation.productCode[cnt] <= 'Z')) /* Letters from A to Z */
      continue;
    if (infoStation.productCode[cnt] == '\0')   /* DEFAULT value */
      continue;
    else if ((infoStation.productCode[cnt] == '.') || (infoStation.productCode[cnt] == '-'))   /* '.' or '-' */
      continue;
    else
      return ERROR_ON_SRAM_DATA;    
  }

  /* #8 --> Symplified current value */
  SRAM_SCU_Check.Param = SimplCurr;
  /* Value must be max 16A */
  if (infoStation.max_currentSemp > VALUE_OF_SIMPLCURR)
      return ERROR_ON_SRAM_DATA;        
  
  /* #9 --> Typical current value */
  SRAM_SCU_Check.Param = TypCurr;
  /* Value must be max 32A */
  if (infoStation.max_current > VALUE_OF_TIPCURR)
      return ERROR_ON_SRAM_DATA;        
  
  /* NO ERRORS on SCU data */
  return NO_ERROR_ON_SRAM_DATA;
}

/**
*
* @brief       SEM_Data_Check
*
* @param [in]  none
*  
* @retval      error on check type (ERROR_ON_SRAM_DATA - NO_ERROR_ON_SRAM_DATA)
*  
****************************************************************/
error_check_data_en SEM_Data_Check(void)
{
  
  uint8_t cnt;
  
  /* Only if we are in SEM mode */
  if (isSemMode())
  {
    /* #8 --> Check Fake Product code data */
    SRAM_SCU_Check.Param = FakeProductCode;              
    for (cnt = 0; cnt < FAKE_CODE_LENGTH; cnt++)
    {
      /* Check Ascii char admitted */
      if ((infoStation.fakeProductCode[cnt] >= '0') && (infoStation.fakeProductCode[cnt] <= '9'))      /* Numbers from 0 t0 9*/
        continue;
      else if ((infoStation.fakeProductCode[cnt] >= 'A') && (infoStation.fakeProductCode[cnt] <= 'Z')) /* Letters from A to Z */
        continue;
      else if (infoStation.fakeProductCode[cnt] == '.')   /* '.' */
        continue;
      if (infoStation.fakeProductCode[cnt] == '\0')  /* DEFAULT value */
        continue;
      else
        return ERROR_ON_SRAM_DATA;    
    }
    
    /* #9 --> Check matrix conversion */
    SRAM_SCU_Check.Param = MatrixConv;              
    for (cnt = 0; cnt < SCU_NUM; cnt++)
    {
      if ((socketPresence.matrixConv[cnt] >= 1) && (socketPresence.matrixConv[cnt] <= 32))        
        continue;
      if (socketPresence.matrixConv[cnt] == 0xFF)  /* DEFAULT value */
        continue;      
      return ERROR_ON_SRAM_DATA;          
    }

    /* #10 --> Check matrix Id connector */
    SRAM_SCU_Check.Param = MatrixIdConn;              
    for (cnt = 0; cnt < SCU_NUM; cnt++)
    {
      if ((socketPresence.matrixIdConn[cnt] >= 1) && (socketPresence.matrixIdConn[cnt] <= 32))        
        continue;
      if (socketPresence.matrixIdConn[cnt] == 0xFF)  /* DEFAULT value */
        continue;      
      return ERROR_ON_SRAM_DATA;          
    }

  }
  
  /* All the check passes */
  SRAM_SCU_Check.Param = Param_NULL; 
  
  return NO_ERROR_ON_SRAM_DATA;  
}

/**
*
* @brief       Data_Integrity_Check
*
* @param [in]  chek type (ON_SCU_DATA - ON_SEM_DATA - ON_ALL_DATA)
*  
* @retval      error on check type (ERROR_CHECKING_DATA_SCU - ERROR_CHECKING_DATA_SEM)
*  
****************************************************************/

error_check_data_en Data_Integrity_Check (sram_check_type_en checkType)
{
  
    /* Check infostation data only if EEPROM has been initialized */
    if (infoStation.key != EDATA_VALID_PRG)
      return NO_SRAM_DATA_CHECKED;
     
    /* Check data integrity */
    switch (checkType)
    {
      case ON_SCU_DATA:
        if (SCU_Data_Check() == ERROR_ON_SRAM_DATA)
          return ERROR_CHECKING_DATA_SCU;
        break;
        
      case ON_SEM_DATA:
        if (SEM_Data_Check() == ERROR_ON_SRAM_DATA)
          return ERROR_CHECKING_DATA_SEM;
        break;
        
      case ON_ALL_DATA:
        if (SCU_Data_Check() == ERROR_ON_SRAM_DATA)
          return ERROR_CHECKING_DATA_SCU;
        if (SEM_Data_Check() == ERROR_ON_SRAM_DATA)
          return ERROR_CHECKING_DATA_SEM;        
        break;
        
      default:
        break;
    }
    
    return NO_ERROR_ON_SRAM_DATA;
}


/**
*
* @brief       SRAM_Param_Show_Corrupted_Value
*              Force DEFAULT value for a specific parameter 
*              in the check list.
*
* @param [in]  Parameter number in the check list
*  
* @retval      none 
*  
****************************************************************/

void SRAM_Param_Show_Corrupted_Value (SRAM_SCU_Check_List_e Param)
{
  /* Select which parameter value must be forced to DEFAULT */
  switch (Param)
  {
    case ScuADDR:
      tPrintf ( "CORRUPTED VALUE of ScuADDR --> %d\r\n" , scuAddr);      
      break;
    case Serial:
      tPrintf ( "CORRUPTED VALUE of SerialNumber --> %s\r\n" , infoStation.serial);      
      break;
    case UserPin:
      tPrintf ( "CORRUPTED VALUE of User pin --> %s\r\n" , infoStation.userPin);      
      break;    
    case RouterSsid:
      tPrintf ( "CORRUPTED VALUE of Router ssid --> %s\r\n" , infoStation.routerSsid);      
      break;    
    case RouterPass:
      tPrintf ( "CORRUPTED VALUE of router password --> %s\r\n" , infoStation.routerPass);      
      break;    
    case ProductSN:
      tPrintf ( "CORRUPTED VALUE of Product SN --> %s\r\n" , infoStation.productSn);      
      break;    
    case ProductCode:
      tPrintf ( "CORRUPTED VALUE of Product Code --> %s\r\n" , infoStation.productCode);      
      break;    
    case FakeProductCode:
      tPrintf ("CORRUPTED VALUE of Fake Product Code --> %s\r\n" , infoStation.fakeProductCode);      
      break;    
    case SimplCurr:
      tPrintf ("CORRUPTED VALUE of Simplified current --> %d\r\n" , infoStation.max_currentSemp);  
      break;    
    case TypCurr:
      tPrintf ("CORRUPTED VALUE of Typical current --> %d\r\n" , infoStation.max_current);      
      break;    
    
    default:
      break;
  }
}

/**
*
* @brief       SRAM_Param_DEFAULT_Force
*              Force DEFAULT value for a specific parameter 
*              in the check list.
*
* @param [in]  Parameter number in the check list
*  
* @retval      none 
*  
****************************************************************/

void SRAM_Param_DEFAULT_Force (SRAM_SCU_Check_List_e Param)
{
  infoStation_t*      pInfoStation;
  uint8_t             infoSet;

  pInfoStation = (infoStation_t*)malloc(sizeof(infoStation_t));
  /* make a copy of current structure values */
  memcpy ((void*)pInfoStation, (void*)&infoStation, sizeof(infoStation_t));

  infoSet = FALSE;

  /* Select which parameter value must be forced to DEFAULT */
  switch (Param)
  {
    case ScuADDR:
      scuAddr = (SCU_NUM / 2) - 1;  /* default address is 15 (16 in webUI) */
      break;
    case Serial:
      /* Force DEFAULT value into the param */
      SRAM_Param_DEFAULT_Set (&pInfoStation->serial[0], 'F', BOARD_SN_LENGTH);    
      infoSet = TRUE;
      break;
    case UserPin:
      /* Force DEFAULT value into the param */
      SRAM_Param_DEFAULT_Set (&pInfoStation->userPin[0], ' ', USER_PIN_LENGTH);    
      infoSet = TRUE;
      break;    
    case RouterSsid:
      /* Force DEFAULT value into the param */
      SRAM_Param_DEFAULT_Set (&pInfoStation->routerSsid[0], 0, MAX_ROUTER_SSID_LENGTH);    
      infoSet = TRUE;
      break;    
    case RouterPass:
      /* Force DEFAULT value into the param */
      SRAM_Param_DEFAULT_Set (&pInfoStation->routerPass[0], 0, MAX_ROUTER_PASS_LENGTH);    
      infoSet = TRUE;
      break;    
    case ProductSN:
      /* Force DEFAULT value into the param */
      SRAM_Param_DEFAULT_Set (&pInfoStation->productSn[0], '0', PRODUCT_SN_LENGTH);    
      infoSet = TRUE;
      break;    
    case ProductCode:
      /* Force DEFAULT value into the param */
      SRAM_Param_DEFAULT_Set (&pInfoStation->productCode[0], 0, PRODUCT_CODE_LENGTH);    
      infoSet = TRUE;
      break;    
    case FakeProductCode:
      /* Force DEFAULT value into the param */
      SRAM_Param_DEFAULT_Set (&pInfoStation->fakeProductCode[0], 0, FAKE_CODE_LENGTH);    
      infoSet = TRUE;
      break;          
    case SimplCurr:
      /* Force DEFAULT value into the param */
      infoStation.max_currentSemp = VALUE_OF_SIMPLCURR;  
      break;    
    case TypCurr:
      /* Force DEFAULT value into the param */
      infoStation.max_current = VALUE_OF_TIPCURR;    
      break;          
    case MatrixConv:
      SRAM_Param_DEFAULT_Set ((char *)&socketPresence.matrixConv[0], '^', SCU_NUM);
      break;    
    case MatrixIdConn:
      SRAM_Param_DEFAULT_Set ((char *)&socketPresence.matrixIdConn[0], '^', SCU_NUM);      
      break; 
    
    default:
      break;
  }
  if (infoSet == TRUE)
  {
    /*                             destination                source                   len   */
    configASSERT(memCpyInfoSt((uint8_t*)&infoStation, (uint8_t*)pInfoStation, sizeof(infoStation_t)));
  }
  /* free allocated area */
  free(pInfoStation);
}

/**
*
* @brief       BKP CNT TRAP EE DATA Update
*
* @param [in]  none
*  
* @retval      none 
*  
****************************************************************/

void BKP_CNT_TRAP_EE_DATA_Update(void)
{
  uint32_t Value;
  uint8_t  cnt;
  /* Read value stored in BKP register 10 */
  Value = HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)getHandleRtc(), (uint32_t)BACKUP_HW_INFO);   
  /* Get counter value */
  cnt = (Value & CNT_TRAP_CHK_EE_DATA_MASK) >> CNT_TRAP_CHK_EE_DATA_Pos;
  /* Check counter overflow */
  if (cnt >= 0x0F)
    cnt = 0;
  else
    cnt++;
  /* Increment counter and calculate the new value to store in BKP register */
  Value = (Value & ~CNT_TRAP_CHK_EE_DATA_MASK) | (cnt << CNT_TRAP_CHK_EE_DATA_Pos);
  /* Save the new value */
  HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), (uint32_t)BACKUP_HW_INFO, Value); 
}

/**
*
* @brief       BKP Reload Param
*              Reload parameters from backup image
*
* @param [in]  none
*  
* @retval      none 
*  
****************************************************************/

void BKP_Reload_Param (uint8_t FromIdle)
{
  infoStation_t*      pInfoStation;
  
  pInfoStation = (infoStation_t*)malloc(sizeof(infoStation_t));

  /* Reload values from infostation BKP area */
  if (FromIdle)
    ReadFromEeprom (EDATA_BKP_SCU_EE_ADDRESS, (uint8_t *)&infoStation, sizeof (infoStation_t));     
  else
    ReadFromEeprom_no_Semaph (EDATA_BKP_SCU_EE_ADDRESS, (uint8_t *)&infoStation, sizeof (infoStation_t));       
  /* Set values in eeprom_param_array */
  eeprom_param_array[M3S_CURRENT_EADD] = infoStation.max_currentSemp / 1000;
  eeprom_param_array[M3T_CURRENT_EADD] = infoStation.max_current / 1000;
  
   /*              destination                source                   len   */
   configASSERT(memCpyInfoSt((uint8_t*)&infoStation, (uint8_t*)pInfoStation, sizeof(infoStation_t)));
   /* free allocated area */
   free(pInfoStation);

}

/**
*
* @brief       EEPROM Data Integrity check
*
* @param [in]  chek type (ON_SCU_DATA - ON_SEM_DATA - ON_ALL_DATA)
*  
* @retval      none 
*  
****************************************************************/
error_check_data_en SRAM_Data_Integrity_check(sram_check_type_en checkType, uint8_t FromIdle)
{
  error_check_data_en Error;
  
  /* Check integrity of sensible data and manage errors */  
  Error = Data_Integrity_Check(checkType);
  /* Check error */
  switch (Error)
  {    
    case ERROR_CHECKING_DATA_SCU:   /* Error on BKP_SCU data block */
      /* Increment counter of traps */
      BKP_CNT_TRAP_EE_DATA_Update();
      tPrintf (ANSI_COLOR_RED "ERROR checking SCU data\r\n" ANSI_COLOR_RESET);
      /* Show corrupted data */
      SRAM_Param_Show_Corrupted_Value(SRAM_SCU_Check.Param);
      /* Check how many attempts */
      if (SRAM_SCU_Check.ErrorCnt++ < 3)
      {
        /* BACKUP image restored */
        tPrintf (ANSI_COLOR_RED "BACKUP image restored\r\n" ANSI_COLOR_RESET);          
        BKP_Reload_Param (FromIdle);
      }
      else
      {
        /* Restoring backup copy of the data doesn't fix the problem...... */
        /* Force DEFAULT value for the parameter corrupted (doesn't pass the check) */
        SRAM_Param_DEFAULT_Force (SRAM_SCU_Check.Param);
        /* Reset error counter */
        SRAM_SCU_Check.ErrorCnt = 0; 
        /* Exit setting this error type */
        Error = ERROR_CHECKING_DATA_SCU_to_DEFAULT;
      }
        
      break;
      
    case ERROR_CHECKING_DATA_SEM:   /* Error on BKP_SEM data block */
      /* Increment counter of traps */
      BKP_CNT_TRAP_EE_DATA_Update();
      tPrintf (ANSI_COLOR_RED "Error checking SEM data\r\n" ANSI_COLOR_RESET);
      tPrintf (ANSI_COLOR_RED "BACKUP image restored\r\n" ANSI_COLOR_RESET);
      /* Check how many attempts */
      if (SRAM_SCU_Check.ErrorCnt++ < 3)
      {
        /* Restore BKP image from EEPROM */
        if (FromIdle)
        {
          if (SRAM_SCU_Check.Param == FakeProductCode)
            BKP_Reload_Param (FromIdle);
          else
            /* Read socket presence fr matrix conv */
            ReadFromEeprom (EDATA_BKP_SEM_EE_ADDRESS, (uint8_t *)&socketPresence, sizeof (socketPresence_t));      
        }
        else
        {
          if (SRAM_SCU_Check.Param == FakeProductCode)
            BKP_Reload_Param (FromIdle);
          else
            /* Read socket presence fr matrix conv */
            ReadFromEeprom_no_Semaph (EDATA_BKP_SEM_EE_ADDRESS, (uint8_t *)&socketPresence, sizeof (socketPresence_t));      
        }
      }
      else
      {
        /* Restoring backup copy of the data doesn't fix the problem...... */
        /* Force DEFAULT value for the parameter corrupted (doesn't pass the check) */
        SRAM_Param_DEFAULT_Force (SRAM_SCU_Check.Param);        
        /* Reset error counter */
        SRAM_SCU_Check.ErrorCnt = 0;
        /* Exit setting this error type */
        Error = ERROR_CHECKING_DATA_SEM_to_DEFAULT;
      }
      
      break;
      
    default:
      /* NO ERRORS checking data: they are valid */  
      SRAM_SCU_Check.ErrorCnt = 0;      
      break;
  }
  
  return Error;
}

/**
*
* @brief       EEPROM_Default_Write
*
* @param [in]  none
*  
* @retval      none 
*  
****************************************************************/

void EEPROM_Default_Write (void)
{

  uint8_t BCD_Data[4], Data[8];

  switch (SRAM_SCU_Check.Param)
  {
    
    case ScuADDR:
      /* Write product Serial number in EEPROM */
      WriteOnEeprom (RS485_ADD_EADD, (uint8_t*)&scuAddr, 1);            
      break;
    
    case Serial:
      /* Set to DEFAULT */
      memset (BCD_Data, 0xFF, sizeof (BCD_Data));
      memset (Data, 'F', sizeof (Data));
      /* Write SCU serial number */
      EEPROM_Save_Config (SERNUM_BYTE0_EADD, BCD_Data, sizeof (BCD_Data));
      setScuSerialNumberEeprom((char*)BCD_Data, (char*)Data);      
      break;
    case ProductSN:
      /* Write product Serial number */
      WriteOnEeprom (PRD_SN_EE_ADDRES, (uint8_t*)infoStation.productSn, PRODUCT_SN_LENGTH);      
      WriteOnEeprom(SCU_GENERAL_INFO_EE_ADDRES, (uint8_t*)&infoStation, sizeof(infoStation_t));
      break;
    case ProductCode:
      /* Write product Code */
      WriteOnEeprom (PRD_CODE_EE_ADDRES, (uint8_t*)&infoStation.productCode, PRODUCT_CODE_LENGTH);              
      WriteOnEeprom(SCU_GENERAL_INFO_EE_ADDRES, (uint8_t*)&infoStation, sizeof(infoStation_t));
      break;    
      
    case SimplCurr:
      /* Write simplified current */
      Data[0] = infoStation.max_currentSemp / 1000;
      EEPROM_Save_Config (M3S_CURRENT_EADD, &Data[0], 1);
      break;      
      
    case TypCurr: 
      /* Write typical current */
      Data[0] = infoStation.max_current / 1000;
      EEPROM_Save_Config (M3T_CURRENT_EADD, &Data[0], 1);
      break;      
      
    default:
      /* Write new infoStation data */
      WriteOnEeprom (SCU_GENERAL_INFO_EE_ADDRES, (uint8_t*)&infoStation, sizeof(infoStation_t));        
      break;
      
  }
}

/**
*
* @brief       EEPROM_Check_Data_Before_Write
*
* @param [in]  Address: address in eeprom to write
*  
* @retval      none 
*  
****************************************************************/

void EEPROM_Check_Data_Before_Write (unsigned short Address)
{
  /* Check integrity of sensible data and manage errors */
  if (infoStation.key == EDATA_VALID_PRG)
  {
    switch (Address)
    {
      case SCU_GENERAL_INFO_EE_ADDRES:        
      case SOCKETS_PRESENCE_EE_ADDRES:
      case RS485_ADD_EADD:        
        /* Check if a different default value (0xFF) is on productSn, productCode, fakeProductCode 
        Starting from v4.3.x and 4.6.x, the default value for these parameters is ' ' and not 0xFF */
        SRAM_Check_DEFAULT_of_Code();    
        /* Check SCU data (infoStation), if no errors, update the backup image */
        if (SRAM_Data_Integrity_check(ON_ALL_DATA, FALSE) == NO_ERROR_ON_SRAM_DATA)
          SRAM_SCU_Check.BKP_Store = TRUE;
        break;
        
      default:
        break;     
    }
  }
}

/**
*
* @brief       EEPROM_Check_Data_Idle
*
* @param [in]  none
*  
* @retval      none 
*  
****************************************************************/
          
void EEPROM_Check_Data_Idle (void)
{
  
    /* Wait while the eeprom data are ready to use */ 
    while (osSemaphoreAcquire (EEprom_semaphore, osWaitForever) != osOK);

    osSemaphoreRelease (EEprom_semaphore);

    switch (SRAM_Data_Integrity_check(ON_ALL_DATA, TRUE))
    {
      
      case ERROR_CHECKING_DATA_SEM_to_DEFAULT:   /* DEFAULT restored due to many attempts checking data */
        /* Update socket presence in EEPROM */
        WriteOnEeprom(SOCKETS_PRESENCE_EE_ADDRES, (uint8_t*)&socketPresence, sizeof(socketPresence_t));        
        /* Update also BKP image */
        BKP_SCU_Image_Store();
        break;
        
      case ERROR_CHECKING_DATA_SCU_to_DEFAULT:   /* update BKP image and store new value */
        /* Update DEFAULT value in EEPROM */
        EEPROM_Default_Write ();
        /* Update also BKP image */
        BKP_SCU_Image_Store();
        break;
        
      default:
        break;
        
    }
}


/**
*
* @brief       EEPROM_Save_Config
*               
*              Save parameter directliy in eeprom and in eeprom_array (RAM)
*
* @param [in]  Address: address in eeprom where data must be saved
*              Buffer: data in RAM to be saved              
*              Lenght: number of bytes to be saved
*  
* @retval      OK / NOT_OK 
*  
****************************************************************/

uint8_t EEPROM_Save_Config (unsigned short Address, unsigned char *Buffer, unsigned short Length)
{
  
  uint8_t result;
  
  /* Save configuration in RAM */
  // xx eeprom_array_set (Address, Buffer, Length);  
  eeprom_param_set (Address, Buffer, Length);  
  
  /* Save configuration in EEPROM */
  result = WriteOnEeprom(SCU_GENERAL_INFO_EE_ADDRES, (uint8_t*)&infoStation, sizeof(infoStation));
  
  /* Check writing result */
  if (result == 0)
  {
    tPrintf("Infostation data updated!\n\r");
    EVLOG_Message(EV_INFO, "Infostation data updated!");

    /* Check if a different default value (0xFF) is on productSn, productCode, fakeProductCode 
       Starting from v4.3.x and 4.6.x, the default value for these parameters is ' ' and not 0xFF */
    SRAM_Check_DEFAULT_of_Code();

    /* Reinit modbus registers according to the new settings */
    initModbusRegisters();
  }
  else
  {
    tPrintf("Error writing in eeprom\n\r");
    EVLOG_Message(EV_INFO, "Error writing in eeprom");      
  }
  
  return result;  
  
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

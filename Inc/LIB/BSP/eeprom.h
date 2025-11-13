// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//
//  File:           eeprom.h
//  Author:         Vania
//  Date:           06/11/2020
//
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#ifndef _EEPROM_H
#define _EEPROM_H
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global include ------------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global defines ------------------------------------------------------------------------------------------------------------------------- //
#define EDATA_BOARD_PRG         (uint8_t)(0x00)      // flag ripristino parametri eeprom di produzione scheda [Elsy]
#define EDATA_FACTORY_PRG       (uint8_t)(0x32)      // flag ripristino parametri eeprom di produzione EVS [SCAME]

#define EDATA_VALID_PRG         (uint8_t)(0xA6)      // flag validità dati in eeprom/flash
#define EDATA_DEFAULT_PRG       (uint8_t)(0xFF)      // default value for eeprom/flash data
//#define EDATA_VALID_PRG         (uint8_t)(0x02)      // flag validità dati in eeprom/flash

#define EDATA_VALID_EADD        (uint16_t)(0)       // flag validità dati in eeprom
#define EDATA_NUM_EADD          (uint16_t)(1)       // numero di byte in eeprom
#define SERNUM_BYTE0_EADD       (uint16_t)(2)       // byte numero di serie LSB (2 digit BDC)
#define SERNUM_BYTE1_EADD       (uint16_t)(3)       // byte numero di serie (2 digit BDC)
#define SERNUM_BYTE2_EADD       (uint16_t)(4)       // byte numero di serie (2 digit BDC)
#define SERNUM_BYTE3_EADD       (uint16_t)(5)       // byte numero di serie MSB (2 digit BDC)
#define SOCKET_ENABLE_EADD      (uint16_t)(6)       // abilitazione presa
#define BATTERY_CONFIG_EADD     (uint16_t)(7)       // modalità di funzionamento in presenza di batteria tampone
#define LANG_DEFAULT_EADD       (uint16_t)(8)       // lingua di default display lcd
#define RS485_ADD_EADD          (uint16_t)(9)       // indirizzo SCU su bus RS-485
#define RTC_VALID_EADD          (uint16_t)(10)      // flag rtc configurato correttamente
#define EVS_MODE_EADD           (uint16_t)(11)      // modalità di funzionamento della EVS
#define M3T_CURRENT_EADD        (uint16_t)(12)      // corrente massima in Modo3 standard [A]
#define M3S_CURRENT_EADD        (uint16_t)(13)      // corrente massima in Modo3 semplificato [A]
#define SOCKET_TYPE_EADD        (uint16_t)(14)      // tipo presa, socket type
#define EMETER_INT_EADD         (uint16_t)(15)      // tipo misuratore di energia interno
#define CONTROL_BYTE0_EADD      (uint16_t)(16)      // abilitazione controlli byte 0
#define CONTROL_BYTE1_EADD      (uint16_t)(17)      // abilitazione controlli byte 1
#define CONTROL_BYTE2_EADD      (uint16_t)(18)      // abilitazione controlli byte 2
#define CONTROL_BYTE3_EADD      (uint16_t)(19)      // abilitazione controlli byte 3
#define ACTUATORS_EADD          (uint16_t)(20)      // configurazione attuatori contattore, blocchi presa e bobina di sgancio
#define BLOCK_DIR_EADD          (uint16_t)(21)      // configurazione direzione blocchi presa
#define PERS_UIDNUM_EADD        (uint16_t)(22)      // personal: numero UID registrati [EVS_PERS_MODE]
#define PERS_MASTER_EADD        (uint16_t)(23)      // personal: flag UID master registrato
#define PMNG_MODE_EADD          (uint16_t)(24)      // power management: configurazione power management
#define PMNG_EMETER_EADD        (uint16_t)(25)      // power management: tipo misuratore di energia esterno
#define PMNG_PWRLSB_EADD        (uint16_t)(26)      // power management: potenza domestica installata LSB [KW * 10]
#define PMNG_PWRMSB_EADD        (uint16_t)(27)      // power management: potenza domestica installata MSB [KW * 10]
#define PMNG_ERROR_EADD         (uint16_t)(28)      // power management: errore ammesso nella regolazione di potenza [KW * 10]
#define PMNG_CURRENT_EADD       (uint16_t)(29)      // power management: corrente minima di ricarica del veicolo prima di andare in sospensione [ampere]
#define PMNG_MULTIP_EADD        (uint16_t)(30)      // power management: fattore moltiplicativo della potenza minima per uscire da no-power HPOWER
#define PMNG_DMAX_EADD          (uint16_t)(31)      // power management: fattore moltiplicativo della potenza per la sospensione immediata della ricarica
#define PMNG_TRANGE_EADD        (uint16_t)(32)      // power management: abilitazione della ricarica con potenza variabile per fasce orarie [fisse]
#define TCHARGE_MODE_EADD       (uint16_t)(33)      // configurazione ricarica a tempo
#define TCHARGE_TIME_EADD       (uint16_t)(34)      // numero step di 30 minuti in ricarica a tempo
#define PMNG_PWDB0_EADD         (uint16_t)(35)      // password menù nascosto byte 0
#define PMNG_PWDB1_EADD         (uint16_t)(36)      // password menù nascosto byte 1
#define PMNG_PWDB2_EADD         (uint16_t)(37)      // password menù nascosto byte 2
#define PMNG_UNBAL_EADD         (uint16_t)(38)      // power management: flag di consenso per carichi sbilanciati in trifase
#define LANG_CONFIG0_EADD       (uint16_t)(39)      // Configurazione lingue abilitate byte 0
#define LANG_CONFIG1_EADD       (uint16_t)(40)      // Configurazione lingue abilitate byte 1
#define LANG_CONFIG2_EADD       (uint16_t)(41)      // Configurazione lingue abilitate byte 2
#define LANG_CONFIG3_EADD       (uint16_t)(42)      // Configurazione lingue abilitate byte 3
#define TOT_ENERGY0_EADD        (uint16_t)(43)      // energia attiva totale erogata byte 0
#define TOT_ENERGY1_EADD        (uint16_t)(44)      // energia attiva totale erogata byte 1
#define TOT_ENERGY2_EADD        (uint16_t)(45)      // energia attiva totale erogata byte 2
#define TOT_ENERGY3_EADD        (uint16_t)(46)      // energia attiva totale erogata byte 3
#define STRIP_LED_TYPE_EADD     (uint16_t)(47)      // tipo di strip: 6 9 12 24 leds
#define TIME_ZONE_EADD          (uint16_t)(48)      // byte per il time zone 
#define DST_EADD                (uint16_t)(49)      // byte per abilitazione ora legale (daylight saving time) 
#define TIME_DST_OFFSET_EADD    (uint16_t)(50)      // byte per entità del DST (tipicamente 1h) 
#define DST_STATUS_EADD         (uint16_t)(51)      // byte per stato corrente dell'ora legale (attiva da aprile a ottobre, disattiva da nov a marzo) 
#define LCD_TYPE_EADD           (uint16_t)(52)      // byte per stato presenza LCD  
#define HIDDEN_MENU_VIS_EADD    (uint16_t)(53)      // visiblità dei setup [menu nascosto]
#define HIDDEN_MENU_ENB_EADD    (uint16_t)(54)      // abilitazione dei setup [menu nascosto]
#define ENRG_LIMIT_EADD         (uint16_t)(55)      // massima energia erogata per la ricarica a energia limitata [KWh]
#define SINAPSI_INST_EADD       (uint16_t)(56)      // flag di avvenuta configurazione e verifica della prima installazione sinapsi
#define EMETER_SCU_INT_EADD     (uint16_t)(57)      // 
#define OPERATIVE_MODE_EADD     (uint16_t)(58)      // modo operativo EMUMAX0 = 1 / SEM = 0
#define TEMP_CTRL_ENB_EADD      (uint16_t)(59)      // 
#define TEMP_CTRL_VAL_EADD      (uint16_t)(60)      // 
#define TEMP_DELTA_EADD         (uint16_t)(61)      // 
#define TEMP_HYSTERESIS_EADD    (uint16_t)(62)      // 
#define RS485_ADD_ALIAS_EADD    (uint16_t)(63)      // non usato 
#define SEM_FLAGS_CTRL_EADD     (uint16_t)(64)      // parametri aggiunti per SEM: bit 0 = fixed / adj SCU address bits 1-2-3 = num socket in the product
#define STATION_NOM_PWR_EADD    (uint16_t)(65)      // potenza nominale del prodotto [KW * 10] --> 7,4KW diventa 74
#define CONNECTOR_NUMBER_EADD   (uint16_t)(66)      // indice del connettore 1..4 nella quartina finale del fake code 
#define POST_SUSP_TIME_EADD     (uint16_t)(67)
                                
#define EEPROM_PARAM_NUM        (uint16_t)(68)      // numero elementi eeprom_param_array da caricare su eeprom_param_array 
#define EEPROM_PARAM_NUM_V421   (uint16_t)(58)      // numero elementi eeprom_param_array nella versione V4.2.1 
#define EEPROM_PARAM_NUM_V423   (uint16_t)(64)      // numero elementi eeprom_param_array nella versione V4.2.3 
//#define RFID_CARD_SIZE_V421     (uint16_t)(0x200 - 0x40)
#define RFID_CARD_SIZE_V421     (uint16_t)(0x364 - 0x40)
//#define RFID_CARD_SIZE_V423     (uint16_t)(0x3C4 - 0x40)
#define RFID_CARD_SIZE_V423     (uint16_t)(0x3A4 - 0x80)

#define CONTROL_BYTE_NUM        (uint8_t)(3)        // numero control byte gestiti
                                
/* ****** REMEMBER: all structures that have size more than 32bytes must aligned to 0x20 address ***** */
#define MASTER_UID_EEOFFSET     (uint16_t)(128)
//#define MASTER_UID_EEDIM      (CARD_UID_DIM)
                                
#define MASTER_UID00_EADD       (uint16_t)(MASTER_UID_EEOFFSET + 0)
#define MASTER_UID01_EADD       (uint16_t)(MASTER_UID_EEOFFSET + 1)
#define MASTER_UID02_EADD       (uint16_t)(MASTER_UID_EEOFFSET + 2)
#define MASTER_UID03_EADD       (uint16_t)(MASTER_UID_EEOFFSET + 3)
#define MASTER_UID04_EADD       (uint16_t)(MASTER_UID_EEOFFSET + 4)
#define MASTER_UID05_EADD       (uint16_t)(MASTER_UID_EEOFFSET + 5)
#define MASTER_UID06_EADD       (uint16_t)(MASTER_UID_EEOFFSET + 6)
#define MASTER_UID07_EADD       (uint16_t)(MASTER_UID_EEOFFSET + 7)  // 128 + 7 = 135
                                
#define USER_MAP_EEOFFSET       (uint16_t)(MASTER_UID_EEOFFSET + 16) // 128 + 16 = 144 = 0x90
#define USER_MAP_EEDIM          (uint16_t)(12)
                                
#define USER_MAP00_EADD         (uint16_t)(USER_MAP_EEOFFSET + 0)
#define USER_MAP01_EADD         (uint16_t)(USER_MAP_EEOFFSET + 1)
#define USER_MAP02_EADD         (uint16_t)(USER_MAP_EEOFFSET + 2)
#define USER_MAP03_EADD         (uint16_t)(USER_MAP_EEOFFSET + 3)
#define USER_MAP04_EADD         (uint16_t)(USER_MAP_EEOFFSET + 4)
#define USER_MAP05_EADD         (uint16_t)(USER_MAP_EEOFFSET + 5)
#define USER_MAP06_EADD         (uint16_t)(USER_MAP_EEOFFSET + 6)
#define USER_MAP07_EADD         (uint16_t)(USER_MAP_EEOFFSET + 7)
#define USER_MAP08_EADD         (uint16_t)(USER_MAP_EEOFFSET + 8)
#define USER_MAP09_EADD         (uint16_t)(USER_MAP_EEOFFSET + 9)
#define USER_MAP10_EADD         (uint16_t)(USER_MAP_EEOFFSET + 10)
#define USER_MAP11_EADD         (uint16_t)(USER_MAP_EEOFFSET + 11)   // 128 + 16 + 11 = 155 --> 164 = 0xA4
                                
#define USER_UID_EEOFFSET       (uint16_t)(164)                     // 0xA4
#define USER_UID_EENUM          (uint16_t)(USER_MAP_EEDIM * 8)	    // numero di UID registrabili 12 * 8 = 96 -> 0x60

#define USER_UID_BYTE_NUM       (uint16_t)(USER_UID_EENUM * 8)      // numero di Byte necessari per salvare il numero di UID registrabili 96 * 8 = 768 -> 0x300
#define EDATA_LAST_BYTE         (uint16_t)(0x03A4)                  // 0x300 + 0xA4 = 0x3A4 = 932
 /* ******************************************************************* */

#define NET_CONFIG_EEPROM_ADDRESS  ((unsigned short)0x0400)      /** size linked to NetworkConfiguration_t*/
#define RSE_CONFIG_EEPROM_ADDRESS  ((unsigned short)0x0440)      /** size linked to rseSetRegEEPROM_st*/

/* at this address we store general info for wifi app (structure infoBle_t) **/
#define SCU_GENERAL_INFO_EE_ADDRES            ((unsigned short)0x0500)  // area infoStation_t 380 bytes = 0x17C --> 0x067C  V5.0.0E del 23/06/2025
#define SCU_GENERAL_INFO_EE_v4_2_1_ADDRES     ((unsigned short)0x0200)
#define SCU_MASTER_UID_EEOFFSET_v4_2_1_ADDRES ((unsigned short)0x0040)

/* ****** REMEMBER: all structures that have size more than 32 bytes must aligned to 0x20 address ***** */
#define EDATA_DEFAULT_SKT_PRESENCE (uint16_t)(0x0690)          // area parametri info socket (socketPresence_t 40 bytes = 0x28 --> 0x06E8)

#define EDATA_DEFAULT_EADD         (uint16_t)(0x0700)          // area parametri di default prodotto

#define EDATA_DEFAULT_LAST_BYTE    (uint16_t)(EDATA_DEFAULT_EADD + EDATA_LAST_BYTE)   /* 0x700 + 0x3A4 = 0xAA4*/
/* ******************************************************************* */

#define EDATA_DEFAULT_ID_CODES     (uint16_t)(0x0B00)          /* Value of Serial Number codes: Board SN - Product SN - Product code - Technical code  */

/* reserved area for serial number  **/
#define START_EE_ADDRES                    ((unsigned short)0x0000)
#define EDATA_BKP_SCU_EE_ADDRESS           ((unsigned short)0x1000)  /* Backup image of infoStation parameters */
#define EDATA_BKP_SEM_EE_ADDRESS           ((unsigned short)0x1200)  /* Backup image of socket presence parameters */
#define EDATA_BKP_RS485_ADDR_EE_ADDRESS    ((unsigned short)0x1300)  /* Backup image of RS485 addr parameter */
#define BKP_REG_COPY_ADDRESS               ((unsigned short)0x1F00)  // 22bytes for backup register area 0x1F00...0x1F15
#define END_SN_EE_ADDRES                   ((unsigned short)0x1FFF)
#define SN_KEY_EE_ADDRES                   ((unsigned short)0x1FFE)
#define SCU_SN_EE_ADDRES                   ((unsigned short)0x1FF0)
#define PRD_SN_EE_ADDRES                   ((unsigned short)0x1FE0)
#define PRD_HV_EE_ADDRES                   ((unsigned short)0x1FD0)
#define PRD_CODE_EE_ADDRES                 ((unsigned short)0x1FA0)

#define EM_SCAME_KEY_EACT_EE_ADDRES        ((unsigned short)0x1FC0)
#define EM_SCAME_EACT_EE_ADDRES            ((unsigned short)0x1FC2)
#define EM_SCAME_V15LEV_EE_ADDRES          ((unsigned short)0x1FCF)
       
#define SOCKETS_PRESENCE_EE_ADDRES         ((unsigned short)0x1E80)   // New address for socket presence --> must be 0x20=32 byte aligned
       
#define BKP_REG_COPY_STORAGE_DONE          ((unsigned short)0xBC00)
#define BKP_REG_COPY_READ_DONE             ((unsigned short)0xBC11)
       
#define EDATA_BKP_SCU_SIGNATURE            ((unsigned short)0x5CA1)  /* Signature for a valid SCU backup data */

/* DEFAULT value for simplified and tipical current */
#define VALUE_OF_SIMPLCURR           16000
#define VALUE_OF_TIPCURR             63000

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global typedef ------------------------------------------------------------------------------------------------------------------------- //

typedef enum
{
EEPROM_NULL = 0,
EEPROM_EVENT_INIT,
EEPROM_UPDATE,  
EEPROM_TIMEOUT,
}EEpromMngEvent_en;

/* queue info structure */
typedef __packed struct
{
EEpromMngEvent_en    EEpromMngEvent;
}EEpromMngMsg_st;

typedef __packed struct
{
  uint32_t  Copy1;  
  uint32_t  Copy2;  
  uint32_t  Copy3;  
  uint32_t  Copy4;  
  uint32_t  Copy5;  
  uint16_t   Key;
  
}EEprom_BKP_Reg_Copy_st;

/* Error checking SRAM area */
typedef enum {

  ERROR_ON_SRAM_DATA,
  ERROR_CHECKING_DATA_SCU,
  ERROR_CHECKING_DATA_SEM,
  ERROR_CHECKING_DATA_SCU_to_DEFAULT,
  ERROR_CHECKING_DATA_SEM_to_DEFAULT,
  NO_SRAM_DATA_CHECKED,
  NO_ERROR_ON_SRAM_DATA,
  
} error_check_data_en;

/* Data checking type */
typedef enum {

  ON_SCU_DATA,
  ON_SEM_DATA,
  ON_ALL_DATA,
    
} sram_check_type_en;

typedef enum
{
  
  Param_NULL = 0,
  ScuADDR,
  Serial,
  UserPin,
  RouterSsid,
  RouterPass,
  ProductSN,
  ProductCode,
  FakeProductCode,
  MatrixConv,
  MatrixIdConn,
  SimplCurr,
  TypCurr,
  
} SRAM_SCU_Check_List_e;

/* Structure used to manage the SRAM check before Eeprom write */
typedef struct {
  
  uint8_t                BKP_Store;
  uint8_t                ErrorCnt;
  SRAM_SCU_Check_List_e  Param;
  
} SRAM_SCU_Check_t;

#define BKP_REG_COPY_KEY_ADDRESS    ((unsigned short)BKP_REG_COPY_ADDRESS + (sizeof(EEprom_BKP_Reg_Copy_st) - sizeof (EEprom_BKP_Reg_Copy.Key)))


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global function prototypes ------------------------------------------------------------------------------------------------------------- //
uint8_t               eeprom_busy_get                 (void);

void                  eeprom_param_set                (uint16_t eadd, uint8_t *src_ptr, uint8_t num);
void                  eeprom_param_get                (uint16_t eadd, uint8_t *dst_ptr, uint8_t num);
                      
void                  eeprom_array_set                (uint16_t eadd, uint8_t *src_ptr, uint8_t num);
                      
void                  eeprom_master_uid_get           (uint8_t *dst_ptr);
void                  eeprom_uid_map_get              (uint8_t *map_ptr);
void                  eeprom_uid_reg_get              (uint16_t uid_add, uint8_t *uid_ptr);
                      
#ifdef OLD_EEPROM_MANAGEMENT 
void                  send_to_eeprom                  (uint8_t eeprom_event);
void                  EEpromMngTask                   (void *pvParameters);
#endif

void                  eeprom_uc_reset_set             (void);
                      
void                  eeprom_default_set              (void);
void                  eeprom_board_default            (void);

uint8_t*              getEepromArray                  (void); 
void                  eeprom_ProductConfig_Param_Set  (void);
void                  setEepromArrayIsolatedMode      (void);
error_check_data_en   SRAM_Data_Integrity_check       (sram_check_type_en checkType, uint8_t FromIdle);
error_check_data_en   Data_Integrity_Check            (sram_check_type_en checkType);
void                  EEPROM_Check_Data_Idle          (void);

// ------------------------------- External variables  ------------------------------------------------------------------------------------------------------------- //

extern EEprom_BKP_Reg_Copy_st        EEprom_BKP_Reg_Copy;
extern SRAM_SCU_Check_t              SRAM_SCU_Check;
extern osSemaphoreId                 EEprom_semaphore;

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// ------------------------------- External Function  ------------------------------------------------------------------------------------------------------------- //

extern void EEpromManager_init(void);
extern void BKP_SCU_Image_Store (void);
extern void EEPROM_Check_Data_Before_Write (unsigned short Address);
extern uint8_t EEPROM_Save_Config (unsigned short Address, unsigned char *Buffer, unsigned short Length);

#endif


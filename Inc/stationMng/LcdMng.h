// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//
//  File:           LcdMng.h
//  Author:         Vania
//  Date:           06/11/2020
//
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#ifndef _LCDMNG_H
#define _LCDMNG_H
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global include ------------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global defines ------------------------------------------------------------------------------------------------------------------------- //
#define LCD_CHAR_NUM            (uint8_t)(20)       // numero caratteri per linea display
#define LCD_LINES_NUM           (uint8_t)(2)        // numero liee display

#define LANGUAGE_ENG_BIT        (uint8_t)(0x01)     // LANG_CONFIG0_EADD
#define LANGUAGE_ITA_BIT        (uint8_t)(0x02)
#define LANGUAGE_FRA_BIT        (uint8_t)(0x04)
#define LANGUAGE_DEU_BIT        (uint8_t)(0x08)
#define LANGUAGE_ESP_BIT        (uint8_t)(0x10)
#define LANGUAGE_POR_BIT        (uint8_t)(0x20)
#define LANGUAGE_RUM_BIT        (uint8_t)(0x40)
#define LANGUAGE_POL_BIT        (uint8_t)(0x80)

#define LANGUAGE_SVE_BIT        (uint8_t)(0x01)     // LANG_CONFIG1_EADD
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global typedef ------------------------------------------------------------------------------------------------------------------------- //

typedef enum    // possible value of LcdLanguage
{
LANGUAGE_ENG    = (uint8_t)(0x00),
LANGUAGE_ITA    = (uint8_t)(0x01),
LANGUAGE_FRA    = (uint8_t)(0x02),
LANGUAGE_DEU    = (uint8_t)(0x03),
LANGUAGE_ESP    = (uint8_t)(0x04),
LANGUAGE_POR    = (uint8_t)(0x05),
LANGUAGE_RUM    = (uint8_t)(0x06),
LANGUAGE_POL    = (uint8_t)(0x07),
LANGUAGE_SVE    = (uint8_t)(0x08),
// In mappa modbus v24 ma attualmente non gestiti dalla SCU --> LANGUAGE_FIN    = (uint8_t)(0x09),
// In mappa modbus v24 ma attualmente non gestiti dalla SCU --> LANGUAGE_NOR    = (uint8_t)(0x0A),
// In mappa modbus v24 ma attualmente non gestiti dalla SCU --> LANGUAGE_SLK    = (uint8_t)(0x0B),
NUM_LANGUAGE    
}LcdLanguage_en;

#define LANGUAGE_MIN    (LANGUAGE_ENG)
#define LANGUAGE_MAX    (LANGUAGE_SVE)

typedef enum
{
LCD_NULL = 0,
LCD_WRITE_SCAME,
LCD_NOPOWER,
LCD_EVS_ERROR,
LCD_MASTER_ERROR,
LCD_EEPARAM_UPDATE,
LCD_CURRENT_UPDATE,
LCD_EVS_DISABLED,
LCD_CARD_WAIT,
LCD_SOCKET_AVAILABLE,
LCD_PLUG_CHECK,
LCD_PLUG_OUT,
LCD_S2_WAIT,
LCD_CHARGING,
LCD_SUSPENDING,
LCD_RMWAITNG,
LCD_HTSWAITNG,
LCD_PENWAITNG,
LCD_EMWAITING,
LCD_INTERRUPTING_CHARGE,
LCD_UPDATE_WLIST,
LCD_UID_DELETE_CONFIRM,
LCD_UID_DELETED,
LCD_UID_ADDED,
LCD_UID_ERROR,
LCD_DELETE_WLIST_CONFIRM,
LCD_AUTHORIZATION_FAILED,
LCD_AUTH_PENDING,
LCD_CARD_PENDING,
LCD_AUTH_MISSED,
LCD_PLUG_WAIT,
LCD_WLIST_DELETED,
LCD_RESET_FAILED,
LCD_DELETE_ERROR,
LCD_WLIST_FULL,
LCD_CREDIT_DATE,
LCD_NEW_CREDIT_DATE,
LCD_ONLY_DATE,
LCD_EXPIRED_CARD,
LCD_ONLY_CREDIT,
LCD_NEW_ONLY_CREDIT,
LCD_NEW_DATE_DONE,
LCD_NEW_DATE_FAILED,
LCD_CREDIT_EXHAUSTED,
LCD_CLOSE_LID,
LCD_PERS_BACK,
LCD_TOTAL_ENERGY,
LCD_PMNG_ENABLE,
LCD_PMNG_TYPE,
LCD_ENTER_PASSWORD,
LCD_DOMESTIC_POWER,
LCD_MIN_CURRENT,
LCD_POWER_MULTIP,
LCD_POWER_ERROR,
LCD_POWER_DMAX,
LCD_UNBAL_ENB,
LCD_EMETER_CRL2,
LCD_TIME_RANGE,
LCD_TIMED_TIME,
LCD_TIMED_TIME_ENB,
LCD_ENRG_LIMIT,
LCD_CHANGE_PASSWORD,
LCD_NEW_PASSWORD,
REFRESH_TIME_EXPIRED,
LANGUAGE_TIME_EXPIRED,
LCD_DEF_LANG_SET,
PERS_TIME_EXPIRED,
EVS_TIME_EXPIRED,
EVSTIME_TIME_EXPIRED,
LCD_POWER_OFF,
LCD_ANTENNA_WIFI_ERROR,
LCD_ANTENNA_WIFI_OK
}LcdMngEvent_en;

/* queue info structure */
typedef __packed struct
{
LcdMngEvent_en    LcdMngEvent;
}LcdMngMsg_st;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global function prototypes ------------------------------------------------------------------------------------------------------------- //
void            send_to_lcd                     (uint8_t lcd_event);
void            lcd_blink_enb_set               (uint8_t blink);
void            lcd_language_config             (uint8_t *src_ptr);
uint8_t         lcd_language_def_enable_get     (void);
void            lcd_language_set                (uint8_t lang);
void            lcd_language_def_update         (void);
LcdMngEvent_en  LcdMngMsg_Old_get               (void);
void            lcd_language_scroll             (void);
void            lcd_external_em_set             (uint8_t val);
void            LcdMngTask                      (void *pvParameters);
char*           getLineString                   (uint8_t numLine, uint8_t* pLineLen);
void            LcdPwrDwnContrast               (void);
void            setPWMlcd2x20                   (void);
void            setPWM_LEDC                     (void);
void            lcdPwrOnSequence                (void);
void            LANG_Modbus_to_EEprom_Translate (uint32_t display_lang_reg);
void            LANG_Available_Mdb_to_EEprom_Translate (uint32_t display_avail_lang);

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#endif

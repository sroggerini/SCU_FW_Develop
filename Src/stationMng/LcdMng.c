// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//
//  File:           LcdMng.c
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
#include "eeprom.h"
#include "i2c.h"
#include "rtcApi.h"

#include "Em_Task.h"
#include "EnergyMng.h"
#include "EvsMng.h"
#include "EvsTimeMng.h"
#include "ExtInpMng.h"
#include "hts.h"
#include "LcdMng.h"
#include "PersMng.h"
#include "PwmMng.h"
#include "scuMdb.h"
#include "lcd.h"
#include "telnet.h"
#include "sbcSem.h"
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local defines -------------------------------------------------------------------------------------------------------------------------- //
#define EVS_ERROR_LCD_MASK0             (RCBO_ANOM0 | MIRROR_ANOM0 | BLOCK_ANOM0 | VENT_ANOM0 | LID_ANOM0 | RCDM_ANOM0)
#define EVS_ERROR_LCD_MASK1             (OVERCURRENT_ANOM1 | EMETER_INT_ANOM1 | MIFARE_ANOM1 | VBUS_ANOM1 | PPLOST_ANOM1 | CPLOST_ANOM1 | PPSHORT_ANOM1 | CPSHORT_ANOM1)
#define EVS_ERROR_LCD_MASK2             (SINAPSI_CHN2_CRL2 | EMETER_EXT_CRL2 | RECTIFIER_ANOM2)

#define WRITE_EVS_DISABLED              (uint8_t)(0 * LCD_LINES_NUM)
#define WRITE_CARD_WAIT                 (uint8_t)(1 * LCD_LINES_NUM)
#define WRITE_SOCKET_AVAILABLE          (uint8_t)(2 * LCD_LINES_NUM)
#define WRITE_PLUG_CHECK                (uint8_t)(3 * LCD_LINES_NUM)
#define WRITE_PLUG_OUT                  (uint8_t)(4 * LCD_LINES_NUM)
#define WRITE_S2_WAIT                   (uint8_t)(5 * LCD_LINES_NUM)
#define WRITE_CHARGING                  (uint8_t)(6 * LCD_LINES_NUM)
#define WRITE_SUSPENDING                (uint8_t)(7 * LCD_LINES_NUM)
#define WRITE_RMWAITING                 (uint8_t)(8 * LCD_LINES_NUM)
#define WRITE_INTERRUPTING_CHARGE       (uint8_t)(9 * LCD_LINES_NUM)
#define WRITE_UPDATE_WLIST              (uint8_t)(10 * LCD_LINES_NUM)
#define WRITE_UID_DELETE_CONFIRM        (uint8_t)(11 * LCD_LINES_NUM)
#define WRITE_UID_DELETED               (uint8_t)(12 * LCD_LINES_NUM)
#define WRITE_UID_ADDED                 (uint8_t)(13 * LCD_LINES_NUM)
#define WRITE_UID_ERROR                 (uint8_t)(14 * LCD_LINES_NUM)
#define WRITE_DELETE_WLIST_CONFIRM      (uint8_t)(15 * LCD_LINES_NUM)
#define WRITE_AUTHORIZATION_FAILED      (uint8_t)(16 * LCD_LINES_NUM)
#define WRITE_PLUG_WAIT                 (uint8_t)(17 * LCD_LINES_NUM)
#define WRITE_WLIST_DELETED             (uint8_t)(18 * LCD_LINES_NUM)
#define WRITE_RESET_FAILED              (uint8_t)(19 * LCD_LINES_NUM)
#define WRITE_DELETE_ERROR              (uint8_t)(20 * LCD_LINES_NUM)
#define WRITE_WLIST_FULL                (uint8_t)(21 * LCD_LINES_NUM)
#define WRITE_EXPIRED_CARD              (uint8_t)(22 * LCD_LINES_NUM)
#define WRITE_CREDIT_NULL               (uint8_t)(23 * LCD_LINES_NUM)
#define WRITE_CLOSE_LID                 (uint8_t)(24 * LCD_LINES_NUM)
#define WRITE_TOTAL_ENERGY              (uint8_t)(25 * LCD_LINES_NUM)
#define WRITE_AUTH_PENDING              (uint8_t)(26 * LCD_LINES_NUM)
#define WRITE_CREDIT_DATE               (uint8_t)(27 * LCD_LINES_NUM)
#define WRITE_NEW_CREDIT_DATE           (uint8_t)(28 * LCD_LINES_NUM)
#define WRITE_ONLY_DATE                 (uint8_t)(29 * LCD_LINES_NUM)
#define WRITE_NEW_DATE_DONE             (uint8_t)(30 * LCD_LINES_NUM)
#define WRITE_PLACE_PLUG                (uint8_t)(31 * LCD_LINES_NUM)
#define WRITE_AUTH_MISSED               (uint8_t)(32 * LCD_LINES_NUM)
#define WRITE_POWER_OFF                 (uint8_t)(33 * LCD_LINES_NUM)
#define WRITE_HTSWAITING                (uint8_t)(34 * LCD_LINES_NUM)
#define WRITE_EMWAITING                 (uint8_t)(35 * LCD_LINES_NUM)
#define WRITE_RESERVED                  (uint8_t)(36 * LCD_LINES_NUM)
#define WRITE_LINES_SIZE                (uint8_t)(37 * LCD_LINES_NUM)

#define LCD_AUX_NULL_EXINF              (uint16_t)(0x0000)
#define LCD_EVS_MODE_ORINF              (uint16_t)(0x0001)
#define LCD_PWM_ORINF                   (uint16_t)(0x0002)
#define LCD_BUSY_OUTLET_TIME_ORINF      (uint16_t)(0x0004)
#define LCD_CHARGING_ORINF              (uint16_t)(0x0008)
#define LCD_USER_UID_NUM_EXINF          (uint16_t)(0x0100)
#define LCD_UPDATE_WLIST_TIME_EXINF     (uint16_t)(0x0200)
#define LCD_DELETE_CONFIRM_TIME_EXINF   (uint16_t)(0x0300)
#define LCD_EVS_SEC_WAIT_EXINF          (uint16_t)(0x0400)
#define LCD_TOTAL_ENERGY_EXINF          (uint16_t)(0x0500)
#define LCD_ENTER_PASSWORD_EXINF        (uint16_t)(0x0600)
#define LCD_DOMESTIC_POWER_EXINF        (uint16_t)(0x0700)
#define LCD_MIN_CURRENT_EXINF           (uint16_t)(0x0800)
#define LCD_POWER_MULTIP_EXINF          (uint16_t)(0x0900)
#define LCD_POWER_ERROR_EXINF           (uint16_t)(0x0A00)
#define LCD_POWER_DMAX_EXINF            (uint16_t)(0x0B00)
#define LCD_EMETER_CRL2_EXINF           (uint16_t)(0x0C00)
#define LCD_TIME_RANGE_EXINF            (uint16_t)(0x0D00)
#define LCD_UNBAL_ENB_EXINF             (uint16_t)(0x0E00)
#define LCD_PMNG_ENB_EXINF              (uint16_t)(0x0F00)
#define LCD_TIMED_ENB_EXINF             (uint16_t)(0x1000)
#define LCD_TIMED_TIME_EXINF            (uint16_t)(0x1100)
#define LCD_CHANGE_PASSWORD_EXINF       (uint16_t)(0x1200)
#define LCD_CREDIT_DATE_EXINF           (uint16_t)(0x1300)
#define LCD_ONLY_DATE_EXINF             (uint16_t)(0x1400)
#define LCD_ONLY_CREDIT_EXINF           (uint16_t)(0x1500)
#define LCD_DATE_TIME_EXINF             (uint16_t)(0x1600)
#define LCD_ENRG_LIMIT_EXINF            (uint16_t)(0x1700)
#define LCD_PMNG_TYPE_EXINF             (uint16_t)(0x1800)

#define LCD_GARD_TIME                   pdMS_TO_TICKS((uint32_t)(100))
#define LCD_BLINK_TIME                  pdMS_TO_TICKS((uint32_t)(500))
#define LCD_REFRESH_TIME                pdMS_TO_TICKS((uint32_t)(3000))
#define LCD_LANGUAGE_TIME               pdMS_TO_TICKS((uint32_t)(30000))
#define LCD_MAIN_BLINK_TIME             pdMS_TO_TICKS((uint32_t)(3000))
#define LCD_DEF_LANGUAGE_TIME           pdMS_TO_TICKS((uint32_t)(3000))
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local typedef -------------------------------------------------------------------------------------------------------------------------- //
typedef enum // possible value of alignment [lcd_line_update]
{
CENTER_ALIGNMENT = 0,
LEFT_ALIGNMENT,
RIGHT_ALIGNMENT
}alignment_en;

typedef enum
{
LCD_LANGUAGE_TIM = 0,
LCD_MAIN_BLINK_TIM,
LCD_DEF_LANGUAGE_TIM,
LCD_REFRESH_TIM,
LCD_NUM_TIMER
}LcdTim_en;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local constats ------------------------------------------------------------------------------------------------------------------------- //
static const uint8_t mono_ph_lcd_energy_array[]         = {EM_SES_ACTIVE_ENERGY, EM_CURRENT_L, EM_ACTIVE_POWER};
static const uint8_t mono_pmng_lcd_energy_array[]       = {EM_SES_ACTIVE_ENERGY, EM_CURRENT_L, EM_ACTIVE_POWER, EM_EXT_ACTIVE_POWER};

static const uint8_t three_ph_lcd_energy_array[]        = {EM_SES_ACTIVE_ENERGY, EM_CURRENT_L1, EM_CURRENT_L2, EM_CURRENT_L3, EM_ACTIVE_POWER};
static const uint8_t three_pmng_lcd_energy_array[]      = {EM_EXT_L1_ACTIVE_POWER, EM_EXT_L2_ACTIVE_POWER, EM_EXT_L3_ACTIVE_POWER, EM_SES_ACTIVE_ENERGY,
                                                           EM_CURRENT_L1, EM_CURRENT_L2, EM_CURRENT_L3, EM_ACTIVE_POWER, EM_EXT_ACTIVE_POWER};

static const uint8_t three_sinapsi_lcd_energy_array[]   = {EM_SES_ACTIVE_ENERGY, EM_CURRENT_L1, EM_CURRENT_L2, EM_CURRENT_L3, EM_ACTIVE_POWER, EM_EXT_ACTIVE_POWER};

static const char lcd_active_sess_energy[12]    = "Etot     KWh";
static const char lcd_charge_auth_energy[12]    = "Eres     KWh";
static const char lcd_current_l1[12]            = "L1       A  ";
static const char lcd_current_l2[12]            = "L2       A  ";
static const char lcd_current_l3[12]            = "L3       A  ";
static const char lcd_active_power[12]          = "Pist     KW ";
static const char lcd_ext_active_power[12]      = "Pest     KW ";
static const char lcd_ext_l1_active_power[12]   = "Pes1     KW ";
static const char lcd_ext_l2_active_power[12]   = "Pes2     KW ";
static const char lcd_ext_l3_active_power[12]   = "Pes3     KW ";
static const char lcd_write_emex_null[10]       = "Pest  NULL";

static const char lcd_line_null[] = "                    ";
static const char lcd_line_error[] = "lcd_line_error";

static const char lcd_write_pen_fault[] = "PEN FAULT";

static const char lcd_write_scame_string1[] = "Scame Control Unit";
static const char lcd_write_scame_string2[] = FW_VERSION;

static const char lcd_write_nopower_string1[] = "NO POWER";
static const char lcd_write_enter_password[] = "PASSWORD";
static const char lcd_write_pmng_enable[] = "POWER MANAGEMENT";
static const char lcd_write_pmng_type[] = "PM MODE";
static const char lcd_write_domestic_power[] = "Pmax";
static const char lcd_write_min_current[] = "Imin";
static const char lcd_write_power_multip[] = "Hpower";
static const char lcd_write_power_error[] = "Dset";
static const char lcd_write_power_dmax[] = "Dmax";
static const char lcd_write_emeter_crl2[] = "EMEX FAULT";
static const char lcd_write_unbal_enb[] = "UNBALANCE";
static const char lcd_write_time_range[] = "TIME RANGE";
static const char lcd_write_timed_enable[] = "TIME/ENERGY LIMIT";
static const char lcd_write_timed_time[] = "CHARGE BY TIME";
static const char lcd_write_enrg_limit[] = "CHARGE BY ENERGY";
static const char lcd_write_change_password[] = "PASSWORD";
static const char lcd_write_new_password[] = "NEW PASSWORD";
static const char lcd_write_no_limit[] = "NO LIMIT";
static const char lcd_write_pmng_full[] = "FULL";
static const char lcd_write_pmng_eco_plus[] = "ECO PLUS";
static const char lcd_write_pmng_eco_smart[] = " ECO SMART";
static const char lcd_write_pmng_null[] = "NULL";
static const char lcd_write_on[] = "ON ";
static const char lcd_write_off[] = "OFF";
static const char lcd_write_minute[] = "MIN";
static const char lcd_write_KWh[] = "KWh ";
static const char lcd_write_antenna[] = "WIFI ANTENNA";
static const char lcd_write_antenna_error[] = "ERROR";
static const char lcd_write_antenna_ok[] = "PRESENT";

static const char lcd_write_error_string1[] = "LCD ERROR";
static const char lcd_write_master_error_string1[] = "MSTE ERROR";

static const char lcd_write_empty_line[] = "";

static const char lcd_write_rcdm_error[] = "RCDM FAULT";
static const char lcd_write_lidc_error[] = "LIDC FAULT";
static const char lcd_write_vent_error[] = "VENT FAULT";
static const char lcd_write_blck_error[] = "BLCK FAULT";
static const char lcd_write_rmen_error[] = "RMEN FAULT";
static const char lcd_write_stop_error[] = "STOP FAULT";
static const char lcd_write_mirr_error[] = "MIRR FAULT";
static const char lcd_write_rcbo_error[] = "RCBO FAULT";

static const char lcd_write_cpse_error[] = "CPSE FAULT";
static const char lcd_write_ppse_error[] = "PPSE FAULT";
static const char lcd_write_cpls_error[] = "CPLS FAULT";
static const char lcd_write_ppls_error[] = "PPLS FAULT";
static const char lcd_write_vbus_error[] = "VBUS FAULT";
static const char lcd_write_mfre_error[] = "MFRE FAULT";
static const char lcd_write_emtr_error[] = "EMTR FAULT";
static const char lcd_write_ovce_error[] = "OVCE FAULT";

static const char lcd_write_rcte_error[] = "RCTE FAULT";
static const char lcd_write_emex_error[] = "EMEX FAULT";
static const char lcd_write_chn2_error[] = "CHN2 FAULT";
static const char lcd_write_nu23_error[] = "NU23 FAULT";
static const char lcd_write_nu24_error[] = "NU24 FAULT";
static const char lcd_write_nu25_error[] = "NU25 FAULT";
static const char lcd_write_nu26_error[] = "NU26 FAULT";
static const char lcd_write_nu27_error[] = "NU27 FAULT";

static const char lcd_write_clke_error[] = "CLKE FAULT";

static const char *lcd_error_table[] =
    {
    lcd_write_rcdm_error, lcd_write_lidc_error,
    lcd_write_vent_error, lcd_write_blck_error,
    lcd_write_rmen_error, lcd_write_stop_error,
    lcd_write_mirr_error, lcd_write_rcbo_error,

    lcd_write_cpse_error, lcd_write_ppse_error,
    lcd_write_cpls_error, lcd_write_ppls_error,
    lcd_write_vbus_error, lcd_write_mfre_error,
    lcd_write_emtr_error, lcd_write_ovce_error,

    lcd_write_rcte_error, lcd_write_emex_error,
    lcd_write_chn2_error, lcd_write_nu23_error,
    lcd_write_nu24_error, lcd_write_nu25_error,
    lcd_write_nu26_error, lcd_write_nu27_error
    };

static const char lcd_write_evs_disabled_eng1[] = "SOCKET";                         // ENG: 00 - WRITE_EVS_DISABLED
static const char lcd_write_evs_disabled_eng2[] = "NOT AVAILABLE";
static const char lcd_write_evs_disabled_ita1[] = "PRESA";                          // ITA
static const char lcd_write_evs_disabled_ita2[] = "NON DISPONIBILE";
static const char lcd_write_evs_disabled_fra1[] = "PRISE";                          // FRA
static const char lcd_write_evs_disabled_fra2[] = "NON DISPONIBLE";
static const char lcd_write_evs_disabled_deu1[] = "STECKDOSE";                      // DEU
static const char lcd_write_evs_disabled_deu2[] = "NICHT VERFUGBAR";
static const char lcd_write_evs_disabled_esp1[] = "TOMA";                           // ESP
static const char lcd_write_evs_disabled_esp2[] = "NO DISPONIBLE";
static const char lcd_write_evs_disabled_por1[] = "TOMADA";                         // POR
static const char lcd_write_evs_disabled_por2[] = "NAO DISPONIVEL";
static const char lcd_write_evs_disabled_rum1[] = "PRIZA";                          // RUM
static const char lcd_write_evs_disabled_rum2[] = "INDISPONIBLA";
static const char lcd_write_evs_disabled_pol1[] = "GNIAZDO";                        // POL
static const char lcd_write_evs_disabled_pol2[] = "NIEDOSTEPNE";
static const char lcd_write_evs_disabled_sve1[] = "UTTAG";                          // SVE
static const char lcd_write_evs_disabled_sve2[] = "EJ TILLGANGLIGT";

static const char lcd_write_card_wait_eng1[] = "SHOW CARD";                         // ENG: 01 - WRITE_CARD_WAIT
//static const char lcd_write_card_wait_eng2[] = " ";
static const char lcd_write_card_wait_ita1[] = "PRESENTARE CARTA";                  // ITA
//static const char lcd_write_card_wait_ita2[] = " ";
static const char lcd_write_card_wait_fra1[] = "PRESENTER CARTE";                   // FRA
//static const char lcd_write_card_wait_fra2[] = " ";
static const char lcd_write_card_wait_deu1[] = "EINSTELLEN CARD";                   // DEU
//static const char lcd_write_card_wait_deu2[] = " ";
static const char lcd_write_card_wait_esp1[] = "MOSTRAR TARJETA";                   // ESP
//static const char lcd_write_card_wait_esp2[] = " ";
static const char lcd_write_card_wait_por1[] = "APRESENTAR CARTAO";                 // POR
//static const char lcd_write_card_wait_por2[] = " ";
static const char lcd_write_card_wait_rum1[] = "PREZENTATI CARD";                   // RUM
//static const char lcd_write_card_wait_rum2[] = " ";
static const char lcd_write_card_wait_pol1[] = "POKAZ KARTE";                       // POL
//static const char lcd_write_card_wait_pol2[] = " ";
static const char lcd_write_card_wait_sve1[] = "VISA KORT";                         // SVE
//static const char lcd_write_card_wait_sve2[] = " ";

static const char lcd_write_socket_available_eng1[] = "SOCKET";                     // ENG: 02 - WRITE_SOCKET_AVAILABLE
static const char lcd_write_socket_available_eng2[] = "AVAILABLE";
static const char lcd_write_socket_available_ita1[] = "PRESA";                      // ITA
static const char lcd_write_socket_available_ita2[] = "DISPONIBILE";
static const char lcd_write_socket_available_fra1[] = "PRISE";                      // FRA
static const char lcd_write_socket_available_fra2[] = "DISPONIBLE";
static const char lcd_write_socket_available_deu1[] = "STECKDOSE";                  // DEU
static const char lcd_write_socket_available_deu2[] = "VERFUGBAR";
static const char lcd_write_socket_available_esp1[] = "TOMA";                       // ESP
static const char lcd_write_socket_available_esp2[] = "DISPONIBLE";
static const char lcd_write_socket_available_por1[] = "TOMADA";                     // POR
static const char lcd_write_socket_available_por2[] = "DISPONIVEL";
static const char lcd_write_socket_available_rum1[] = "PRIZA";                      // RUM
static const char lcd_write_socket_available_rum2[] = "DISPONIBILA";
static const char lcd_write_socket_available_pol1[] = "GNIAZDO";                    // POL
static const char lcd_write_socket_available_pol2[] = "DOSTEPNE";
static const char lcd_write_socket_available_sve1[] = "UTTAG";                      // SVE
static const char lcd_write_socket_available_sve2[] = "TILLGANGLIGT";

static const char lcd_write_plug_check_eng1[] = "INSERTED";                         // ENG: 03 - WRITE_PLUG_CHECK
static const char lcd_write_plug_check_eng2[] = "PLUG";
static const char lcd_write_plug_check_ita1[] = "SPINA";                            // ITA
static const char lcd_write_plug_check_ita2[] = "INSERITA";
static const char lcd_write_plug_check_fra1[] = "CONNECTEUR";                       // FRA
static const char lcd_write_plug_check_fra2[] = "INSERE";
static const char lcd_write_plug_check_deu1[] = "STECKER";                          // DEU
static const char lcd_write_plug_check_deu2[] = "EINSETZEN";
static const char lcd_write_plug_check_esp1[] = "CONECTOR";                         // ESP
static const char lcd_write_plug_check_esp2[] = "INSERTADO";
static const char lcd_write_plug_check_por1[] = "CONECTOR";                         // POR
static const char lcd_write_plug_check_por2[] = "INSERIDO";
static const char lcd_write_plug_check_rum1[] = "CONECTOR";                         // RUM
static const char lcd_write_plug_check_rum2[] = "INTRODUS";
static const char lcd_write_plug_check_pol1[] = "UMIESZCZONA";                      // POL
static const char lcd_write_plug_check_pol2[] = "WTYCZKA";
static const char lcd_write_plug_check_sve1[] = "ISATT";                            // SVE
static const char lcd_write_plug_check_sve2[] = "STICKPROPP";

//static const char lcd_write_plug_out_eng1[] = " ";                                // ENG: 04 - WRITE_PLUG_OUT
static const char lcd_write_plug_out_eng2[] = "PLUG OUT";
//static const char lcd_write_plug_out_ita1[] = " ";                                // ITA
static const char lcd_write_plug_out_ita2[] = "ESTRARRE SPINA";
//static const char lcd_write_plug_out_fra1[] = " ";                                // FRA
static const char lcd_write_plug_out_fra2[] = "RETIRER LA FICHE";
//static const char lcd_write_plug_out_deu1[] = " ";                                // DEU
static const char lcd_write_plug_out_deu2[] = "STECKER ZIEHEN";
//static const char lcd_write_plug_out_esp1[] = " ";                                // ESP
static const char lcd_write_plug_out_esp2[] = "EXTRAER CONECTOR";
//static const char lcd_write_plug_out_por1[] = " ";                                // POR
static const char lcd_write_plug_out_por2[] = "REMOVER CONECTOR";
//static const char lcd_write_plug_out_rum1[] = " ";                                // RUM
static const char lcd_write_plug_out_rum2[] = "EXTRAGETI CONECTORUL";
//static const char lcd_write_plug_out_pol1[] = " ";                                // POL
static const char lcd_write_plug_out_pol2[] = "ODLACZ WTYCZKE";
//static const char lcd_write_plug_out_sve1[] = " ";                                // SVE
static const char lcd_write_plug_out_sve2[] = "TA UR STICKPROPP";

static const char lcd_write_s2_waiting_eng1[] = "EV WAITING";                       // ENG: 05 - WRITE_S2_WAIT
//static const char lcd_write_s2_waiting_eng2[] = " ";
static const char lcd_write_s2_waiting_ita1[] = "ATTESA EV";                        // ITA
//static const char lcd_write_s2_waiting_ita2[] = " ";
static const char lcd_write_s2_waiting_fra1[] = "ATTENTE EV";                       // FRA
//static const char lcd_write_s2_waiting_fra2[] = " ";
static const char lcd_write_s2_waiting_deu1[] = "EV ERWARTUNG";                     // DEU
//static const char lcd_write_s2_waiting_deu2[] = " ";
static const char lcd_write_s2_waiting_esp1[] = "ESPERA EV";                        // ESP
//static const char lcd_write_s2_waiting_esp2[] = " ";
static const char lcd_write_s2_waiting_por1[] = "ESPERA EV";                        // POR
//static const char lcd_write_s2_waiting_por2[] = " ";
static const char lcd_write_s2_waiting_rum1[] = "ASTEAPTA EV";                      // RUM
//static const char lcd_write_s2_waiting_rum2[] = " ";
static const char lcd_write_s2_waiting_pol1[] = "POJAZD OCZEKUJE";                  // POL
//static const char lcd_write_s2_waiting_pol2[] = " ";
static const char lcd_write_s2_waiting_sve1[] = "ELBIL VANTAR";                     // SVE
//static const char lcd_write_s2_waiting_sve2[] = " ";

static const char lcd_write_charging_eng1[] = "CHARGING";                           // ENG: 06 - WRITE_CHARGING
//static const char lcd_write_charging_eng2[] = " ";
static const char lcd_write_charging_ita1[] = "IN CARICA";                          // ITA
//static const char lcd_write_charging_ita2[] = " ";
static const char lcd_write_charging_fra1[] = "EN CHARGE";                          // FRA
//static const char lcd_write_charging_fra2[] = " ";
static const char lcd_write_charging_deu1[] = "LADUNG";                             // DEU
//static const char lcd_write_charging_deu2[] = " ";
static const char lcd_write_charging_esp1[] = "EN CARGA";                           // ESP
//static const char lcd_write_charging_esp2[] = " ";
static const char lcd_write_charging_por1[] = "ENCARREGADO";                        // POR
//static const char lcd_write_charging_por2[] = " ";
static const char lcd_write_charging_rum1[] = "INCARCARE";                          // RUM
//static const char lcd_write_charging_rum2[] = " ";
static const char lcd_write_charging_pol1[] = "LADOWANIE";                          // POL
//static const char lcd_write_charging_pol2[] = " ";
static const char lcd_write_charging_sve1[] = "LADDAR";                             // SVE
//static const char lcd_write_charging_sve2[] = " ";

static const char lcd_write_suspendig_eng1[] = "SUSPENDING";                        // ENG: 07 - WRITE_SUSPENDING
//static const char lcd_write_suspendig_eng2[] = " ";
static const char lcd_write_suspendig_ita1[] = "SOSPENSIONE";                       // ITA
//static const char lcd_write_suspendig_ita2[] = " ";
static const char lcd_write_suspendig_fra1[] = "SUSPENSION";                        // FRA
//static const char lcd_write_suspendig_fra2[] = " ";
static const char lcd_write_suspendig_deu1[] = "AUSSETZUNG";                        // DEU
//static const char lcd_write_suspendig_deu2[] = " ";
static const char lcd_write_suspendig_esp1[] = "SUSPENSION";                        // ESP
//static const char lcd_write_suspendig_esp2[] = " ";
static const char lcd_write_suspendig_por1[] = "SUSPENSAO";                         // POR
//static const char lcd_write_suspendig_por2[] = " ";
static const char lcd_write_suspendig_rum1[] = "SUSPENDARE";                        // RUM
//static const char lcd_write_suspendig_rum2[] = " ";
static const char lcd_write_suspendig_pol1[] = "WSTRZYMANO";                        // POL
//static const char lcd_write_suspendig_pol2[] = " ";
static const char lcd_write_suspendig_sve1[] = "AVSLUTANDE";                        // SVE
//static const char lcd_write_suspendig_sve2[] = " ";

static const char lcd_write_rmwaiting_eng1[] = "RM WAITING";                        // ENG: 08 - WRITE_RMWAITING
//static const char lcd_write_rmwaiting_eng2[] = " ";
static const char lcd_write_rmwaiting_ita1[] = "ATTESA RM";                         // ITA
//static const char lcd_write_rmwaiting_ita2[] = " ";
static const char lcd_write_rmwaiting_fra1[] = "ATTENTE RM";                        // FRA
//static const char lcd_write_rmwaiting_fra2[] = " ";
static const char lcd_write_rmwaiting_deu1[] = "RM ERWARTUNG";                      // DEU
//static const char lcd_write_rmwaiting_deu2[] = " ";
static const char lcd_write_rmwaiting_esp1[] = "ESPERA RM";                         // ESP
//static const char lcd_write_rmwaiting_esp2[] = " ";
static const char lcd_write_rmwaiting_por1[] = "ESPERA RM";                         // POR
//static const char lcd_write_rmwaiting_por2[] = " ";
static const char lcd_write_rmwaiting_rum1[] = "ASTEAPTA RM";                       // RUM
//static const char lcd_write_rmwaiting_rum2[] = " ";
static const char lcd_write_rmwaiting_pol1[] = "RM OCZEKUJE";                       // POL
//static const char lcd_write_rmwaiting_pol2[] = " ";
static const char lcd_write_rmwaiting_sve1[] = "RM VANTAR";                         // SVE
//static const char lcd_write_rmwaiting_sve2[] = " ";

static const char lcd_write_interrupting_charge_eng1[] = "INTERRUPTING";            // ENG: 09 - WRITE_INTERRUPTING_CHARGE
static const char lcd_write_interrupting_charge_eng2[] = "CHARGE PROCESS";
static const char lcd_write_interrupting_charge_ita1[] = "INTERRUZIONE";            // ITA
static const char lcd_write_interrupting_charge_ita2[] = "CARICA";
static const char lcd_write_interrupting_charge_fra1[] = "INTERRUPTION";            // FRA
static const char lcd_write_interrupting_charge_fra2[] = "DE CHARGE";
static const char lcd_write_interrupting_charge_deu1[] = "UNTERBRECHUNG";           // DEU
static const char lcd_write_interrupting_charge_deu2[] = "LADUNG";
static const char lcd_write_interrupting_charge_esp1[] = "INTERRUPCION";            // ESP
static const char lcd_write_interrupting_charge_esp2[] = "DE CARGA";
static const char lcd_write_interrupting_charge_por1[] = "INTERRUPCAO";             // POR
static const char lcd_write_interrupting_charge_por2[] = "ENCARREGADO";
static const char lcd_write_interrupting_charge_rum1[] = "INTRERUPERE";             // RUM
static const char lcd_write_interrupting_charge_rum2[] = "INCARCARE";
static const char lcd_write_interrupting_charge_pol1[] = "PRZERWANO";               // POL
static const char lcd_write_interrupting_charge_pol2[] = "LADOWANIE";
static const char lcd_write_interrupting_charge_sve1[] = "LADDNING";                // SVE
static const char lcd_write_interrupting_charge_sve2[] = "AVBRUTEN";

static const char lcd_write_update_wlist_eng1[] = "ARCHIVE MANAGEMENT";             // ENG: 10 - WRITE_UPDATE_WLIST
static const char lcd_write_update_wlist_eng2[] = "SHOW CARD";
static const char lcd_write_update_wlist_ita1[] = "GESTIONE ARCHIVIO";              // ITA
static const char lcd_write_update_wlist_ita2[] = "PRESENTARE CARTA";
static const char lcd_write_update_wlist_fra1[] = "GESTION ARCHIVES";               // FRA
static const char lcd_write_update_wlist_fra2[] = "PRESENTER CARTE";
static const char lcd_write_update_wlist_deu1[] = "ARCHIV MANAGEMENT";              // DEU
static const char lcd_write_update_wlist_deu2[] = "EINSTELLEN CARD";
static const char lcd_write_update_wlist_esp1[] = "GESTION ARCHIVO";                // ESP
static const char lcd_write_update_wlist_esp2[] = "MOSTRAR TARJETA";
static const char lcd_write_update_wlist_por1[] = "GESTAO ARQUIVO";                 // POR
static const char lcd_write_update_wlist_por2[] = "APRESENTAR CARTAO";
static const char lcd_write_update_wlist_rum1[] = "MANAGEMENT ARHIVA";              // RUM
static const char lcd_write_update_wlist_rum2[] = "PREZENTATI CARD";
static const char lcd_write_update_wlist_pol1[] = "ZARZADZANIE ARCHIWUM";           // POL
static const char lcd_write_update_wlist_pol2[] = "POKAZ KARTE";
static const char lcd_write_update_wlist_sve1[] = "HANDHA ARKIV";                   // SVE
static const char lcd_write_update_wlist_sve2[] = "VISA KORT";

static const char lcd_uid_delete_confirm_eng1[] = "DELETE USER?";                   // ENG: 11 - WRITE_UID_DELETE_CONFIRM
//static const char lcd_uid_delete_confirm_eng2[] = " ";
static const char lcd_uid_delete_confirm_ita1[] = "CANCELLARE UTENTE?";             // ITA
//static const char lcd_uid_delete_confirm_ita2[] = " ";
static const char lcd_uid_delete_confirm_fra1[] = "EFFACER UTILISATEUR?";           // FRA
//static const char lcd_uid_delete_confirm_fra2[] = " ";
static const char lcd_uid_delete_confirm_deu1[] = "STREICHEN BENUTZER?";            // DEU
//static const char lcd_uid_delete_confirm_deu2[] = " ";
static const char lcd_uid_delete_confirm_esp1[] = "ELIMINAR USUARIO?";              // ESP
//static const char lcd_uid_delete_confirm_esp2[] = " ";
static const char lcd_uid_delete_confirm_por1[] = "APAGAR USUARIO?";                // POR
//static const char lcd_uid_delete_confirm_por2[] = " ";
static const char lcd_uid_delete_confirm_rum1[] = "STERGETI UTILIZATOR?";           // RUM
//static const char lcd_uid_delete_confirm_rum2[] = " ";
static const char lcd_uid_delete_confirm_pol1[] = "USUNAC UZYTKOWNIKA?";            // POL
//static const char lcd_uid_delete_confirm_pol2[] = " ";
static const char lcd_uid_delete_confirm_sve1[] = "TA BORT ANVANDARE?";             // SVE
//static const char lcd_uid_delete_confirm_sve2[] = " ";

static const char lcd_uid_deleted_eng1[] = "ID DELETED";                            // ENG: 12 - WRITE_UID_DELETED
//static const char lcd_uid_users_eng2[] = "USERS";
static const char lcd_uid_deleted_ita1[] = "ID CANCELLATO";                         // ITA
//static const char lcd_uid_users_ita2[] = "UTENTI";
static const char lcd_uid_deleted_fra1[] = "ID EFFACE";                             // FRA
//static const char lcd_uid_users_fra2[] = "UTILISATEURS";
static const char lcd_uid_deleted_deu1[] = "ID GESTRICHEN";                         // DEU
//static const char lcd_uid_users_deu2[] = "BENUTZERS";
static const char lcd_uid_deleted_esp1[] = "ID CANCELADO";                          // ESP
//static const char lcd_uid_users_esp2[] = "USUARIOS";
static const char lcd_uid_deleted_por1[] = "ID CANCELADO";                          // POR
//static const char lcd_uid_users_por2[] = "USUARIOS";
static const char lcd_uid_deleted_rum1[] = "ID STERS";                              // RUM
//static const char lcd_uid_users_rum2[] = "UTILIZATORI";
static const char lcd_uid_deleted_pol1[] = "ID USUNIETO";                           // POL
//static const char lcd_uid_users_pol2[] = "UZYTKOWNICY";
static const char lcd_uid_deleted_sve1[] = "BORTAGEN";                              // SVE
//static const char lcd_uid_users_sve2[] = "ANVANDARE";

static const char lcd_uid_added_eng1[] = "ID ADDED";                                // ENG: 13 - WRITE_UID_ADDED
static const char lcd_uid_users_eng2[] = "USERS";
static const char lcd_uid_added_ita1[] = "ID REGISTRATO";                           // ITA
static const char lcd_uid_users_ita2[] = "UTENTI";
static const char lcd_uid_added_fra1[] = "ID ENREGISTRE";                           // FRA
static const char lcd_uid_users_fra2[] = "UTILISATEURS";
static const char lcd_uid_added_deu1[] = "ID REGISTRIERTER";                        // DEU
static const char lcd_uid_users_deu2[] = "BENUTZERS";
static const char lcd_uid_added_esp1[] = "ID REGISTRADO";                           // ESP
static const char lcd_uid_users_esp2[] = "USUARIOS";
static const char lcd_uid_added_por1[] = "ID REGISTRADO";                           // POR
static const char lcd_uid_users_por2[] = "USUARIOS";
static const char lcd_uid_added_rum1[] = "ID INREGISTRAT";                          // RUM
static const char lcd_uid_users_rum2[] = "UTILIZATORI";
static const char lcd_uid_added_pol1[] = "ID DODANO";                               // POL
static const char lcd_uid_users_pol2[] = "UZYTKOWNICY";
static const char lcd_uid_added_sve1[] = "REGISTRERA";                              // SVE
static const char lcd_uid_users_sve2[] = "ANVANDARE";

static const char lcd_uid_error_eng1[] = "UNAUTHORIZED";                            // ENG: 14 - WRITE_UID_ERROR
static const char lcd_uid_error_eng2[] = "USER";
static const char lcd_uid_error_ita1[] = "UTENTE";                                  // ITA
static const char lcd_uid_error_ita2[] = "NON AUTORIZZATO";
static const char lcd_uid_error_fra1[] = "UTILISATEUR";                             // FRA
static const char lcd_uid_error_fra2[] = "NON AUTORISE";
static const char lcd_uid_error_deu1[] = "BENUTZER";                                // DEU
static const char lcd_uid_error_deu2[] = "UNBEFUGT";
static const char lcd_uid_error_esp1[] = "USUARIO";                                 // ESP
static const char lcd_uid_error_esp2[] = "NO AUTORIZADO";
static const char lcd_uid_error_por1[] = "UTILIZADOR";                              // POR
static const char lcd_uid_error_por2[] = "NAO AUTORIZADO";
static const char lcd_uid_error_rum1[] = "UTILIZATOR";                              // RUM
static const char lcd_uid_error_rum2[] = "NEAUTORIZAT";
static const char lcd_uid_error_pol1[] = "NIEUPOWAZNIONY";                          // POL
static const char lcd_uid_error_pol2[] = "UZYTKOWNIK";
static const char lcd_uid_error_sve1[] = "OTILLATEN";                               // SVE
static const char lcd_uid_error_sve2[] = "ANVANDARE";

static const char lcd_delete_wlist_confirm_eng1[] = "DELETE ARCHIVE?";              // ENG: 15 - WRITE_DELETE_WLIST_CONFIRM
//static const char lcd_delete_wlist_confirm_eng2[] = " ";
static const char lcd_delete_wlist_confirm_ita1[] = "CANCELLARE ARCHIVIO?";         // ITA
//static const char lcd_delete_wlist_confirm_ita2[] = " ";
static const char lcd_delete_wlist_confirm_fra1[] = "EFFACER ARCHIVES?";            // FRA
//static const char lcd_delete_wlist_confirm_fra2[] = " ";
static const char lcd_delete_wlist_confirm_deu1[] = "STREICHEN ARCHIV?";            // DEU
//static const char lcd_delete_wlist_confirm_deu2[] = " ";
static const char lcd_delete_wlist_confirm_esp1[] = "ELIMINAR ARCHIVO?";            // ESP
//static const char lcd_delete_wlist_confirm_esp2[] = " ";
static const char lcd_delete_wlist_confirm_por1[] = "APAGAR ARQUIVO?";              // POR
//static const char lcd_delete_wlist_confirm_por2[] = " ";
static const char lcd_delete_wlist_confirm_rum1[] = "STERGETI ARHIVA?";             // RUM
//static const char lcd_delete_wlist_confirm_rum2[] = " ";
static const char lcd_delete_wlist_confirm_pol1[] = "USUNAC ARCHIWUM?";             // POL
//static const char lcd_delete_wlist_confirm_pol2[] = " ";
static const char lcd_delete_wlist_confirm_sve1[] = "KANCELLERA ARKIV?";            // SVE
//static const char lcd_delete_wlist_confirm_sve2[] = " ";

static const char lcd_authorization_failed_eng1[] = "AUTHORIZATION";                // ENG: 16 - WRITE_AUTHORIZATION_FAILED
static const char lcd_authorization_failed_eng2[] = "FAILED";
static const char lcd_authorization_failed_ita1[] = "AUTORIZZAZIONE";               // ITA
static const char lcd_authorization_failed_ita2[] = "MANCATA";
static const char lcd_authorization_failed_fra1[] = "AUTORISATION";                 // FRA
static const char lcd_authorization_failed_fra2[] = "MANQUEE";
static const char lcd_authorization_failed_deu1[] = "AUTORISIERUNG";                // DEU
static const char lcd_authorization_failed_deu2[] = "FEHLGESCHLAGEN";
static const char lcd_authorization_failed_esp1[] = "AUTORIZACION";                 // ESP
static const char lcd_authorization_failed_esp2[] = "PERDIDA";
static const char lcd_authorization_failed_por1[] = "AUTORIZACAO";                  // POR
static const char lcd_authorization_failed_por2[] = "PERDA";
static const char lcd_authorization_failed_rum1[] = "AUTORIZARE";                   // RUM
static const char lcd_authorization_failed_rum2[] = "ESUATA";
static const char lcd_authorization_failed_pol1[] = "AUTORYZACJA";                  // POL
static const char lcd_authorization_failed_pol2[] = "NIEUDANA";
static const char lcd_authorization_failed_sve1[] = "AUKTORISERING";                // SVE
static const char lcd_authorization_failed_sve2[] = "MISSLYCKADES";

static const char lcd_plug_wait_eng1[] = "PLUG IN";                                 // ENG: 17 - WRITE_PLUG_WAIT
//static const char lcd_plug_wait_eng2[] = " ";
static const char lcd_plug_wait_ita1[] = "INSERIRE SPINA";                          // ITA
//static const char lcd_plug_wait_ita2[] = " ";
static const char lcd_plug_wait_fra1[] = "INSERER LA FICHE";                        // FRA
//static const char lcd_plug_wait_fra2[] = " ";
static const char lcd_plug_wait_deu1[] = "EINFUGEN STECKER";                        // DEU
//static const char lcd_plug_wait_deu2[] = " ";
static const char lcd_plug_wait_esp1[] = "INSERTAR CONECTOR";                       // ESP
//static const char lcd_plug_wait_esp2[] = " ";
static const char lcd_plug_wait_por1[] = "INSERIR CONECTOR";                        // POR
//static const char lcd_plug_wait_por2[] = " ";
static const char lcd_plug_wait_rum1[] = "INTRODUCETI CONECTOR";                    // RUM
//static const char lcd_plug_wait_rum2[] = " ";
static const char lcd_plug_wait_pol1[] = "PODLACZ";                                 // POL
//static const char lcd_plug_wait_pol2[] = " ";
static const char lcd_plug_wait_sve1[] = "SATT I STICKPROPP";                       // SVE
//static const char lcd_plug_wait_sve2[] = " ";

static const char lcd_wlist_deleted_eng1[] = "ARCHIVE DELETED";                     // ENG: 18 - WRITE_WLIST_DELETED
//static const char lcd_wlist_deleted_eng2[] = " ";
static const char lcd_wlist_deleted_ita1[] = "ARCHIVIO CANCELLATO";                 // ITA
//static const char lcd_wlist_deleted_ita2[] = " ";
static const char lcd_wlist_deleted_fra1[] = "ARCHIVES EFFACE";                     // FRA
//static const char lcd_wlist_deleted_fra2[] = " ";
static const char lcd_wlist_deleted_deu1[] = "ARCHIV GESTREICHEN";                  // DEU
//static const char lcd_wlist_deleted_deu2[] = " ";
static const char lcd_wlist_deleted_esp1[] = "ARCHIVO CANCELADA";                   // ESP
//static const char lcd_wlist_deleted_esp2[] = " ";
static const char lcd_wlist_deleted_por1[] = "ARQUIVO CANCELADA";                   // POR
//static const char lcd_wlist_deleted_por2[] = " ";
static const char lcd_wlist_deleted_rum1[] = "ARHIVA STEARSA";                      // RUM
//static const char lcd_wlist_deleted_rum2[] = " ";
static const char lcd_wlist_deleted_pol1[] = "ARCHIWUM USUNIETE";                   // POL
//static const char lcd_wlist_deleted_pol2[] = " ";
static const char lcd_wlist_deleted_sve1[] = "ANNULERA ARKIV";                      // SVE
//static const char lcd_wlist_deleted_sve2[] = " ";

static const char lcd_reset_failed_eng1[] = "FAILED";                               // ENG: 19 - WRITE_RESET_FAILED
static const char lcd_reset_failed_eng2[] = "RESET";
static const char lcd_reset_failed_ita1[] = "CANCELLAZIONE";                        // ITA
static const char lcd_reset_failed_ita2[] = "FALLITA";
static const char lcd_reset_failed_fra1[] = "ECHOUE";                               // FRA
static const char lcd_reset_failed_fra2[] = "EFFACEMENT";
static const char lcd_reset_failed_deu1[] = "VERSAGT";                              // DEU
static const char lcd_reset_failed_deu2[] = "STREICHUNG";
static const char lcd_reset_failed_esp1[] = "FALLADO";                              // ESP
static const char lcd_reset_failed_esp2[] = "CANCELACION";
static const char lcd_reset_failed_por1[] = "FALHADO";                              // POR
static const char lcd_reset_failed_por2[] = "CANCELAMENTO";
static const char lcd_reset_failed_rum1[] = "ESUAT";                                // RUM
static const char lcd_reset_failed_rum2[] = "ANULARE";
static const char lcd_reset_failed_pol1[] = "NIEPOWODZENIE";                        // POL
static const char lcd_reset_failed_pol2[] = "ANULUJ";
static const char lcd_reset_failed_sve1[] = "FALLERAT";                             // SVE
static const char lcd_reset_failed_sve2[] = "KANCELLERA";

static const char lcd_delete_error_eng1[] = "ARCHIVE MANAGEMENT";                   // ENG: 20 - WRITE_DELETE_ERROR
static const char lcd_delete_error_eng2[] = "FAILED";
static const char lcd_delete_error_ita1[] = "GESTIONE ARCHIVIO";                    // ITA
static const char lcd_delete_error_ita2[] = "FALLITA";
static const char lcd_delete_error_fra1[] = "GESTION ARCHIVES";                     // FRA
static const char lcd_delete_error_fra2[] = "ECHOUE";
static const char lcd_delete_error_deu1[] = "ARCHIV MANAGEMENT";                    // DEU
static const char lcd_delete_error_deu2[] = "VERSAGT";
static const char lcd_delete_error_esp1[] = "GESTION ARCHIVO";                      // ESP
static const char lcd_delete_error_esp2[] = "FALLADO";
static const char lcd_delete_error_por1[] = "GESTAO ARQUIVO";                       // POR
static const char lcd_delete_error_por2[] = "FALHADO";
static const char lcd_delete_error_rum1[] = "MANAGEMENT ARHIVA";                    // RUM
static const char lcd_delete_error_rum2[] = "ESUAT";
static const char lcd_delete_error_pol1[] = "ZARZADZANIE ARCHIWUM";                 // POL
static const char lcd_delete_error_pol2[] = "NIEPOWODZENIE";
static const char lcd_delete_error_sve1[] = "HANDHA ARKIV";                         // SVE
static const char lcd_delete_error_sve2[] = "FALLERAT";

static const char lcd_wlist_full_eng1[] = "ARCHIVE FULL";                           // ENG: 21 - WRITE_WLIST_FULL
//static const char lcd_wlist_full_eng2[] = " ";
static const char lcd_wlist_full_ita1[] = "ARCHIVIO COMPLETO";                      // ITA
//static const char lcd_wlist_full_ita2[] = " ";
static const char lcd_wlist_full_fra1[] = "ARCHIVES COMPLET";                       // FRA
//static const char lcd_wlist_full_fra2[] = " ";
static const char lcd_wlist_full_deu1[] = "ARCHIV VOLL";                            // DEU
//static const char lcd_wlist_full_deu2[] = " ";
static const char lcd_wlist_full_esp1[] = "ARCHIVO COMPLETO";                       // ESP
//static const char lcd_wlist_full_esp2[] = " ";
static const char lcd_wlist_full_por1[] = "ARQUIVO CHEIO";                          // POR
//static const char lcd_wlist_full_por2[] = " ";
static const char lcd_wlist_full_rum1[] = "ARHIVA PLINA";                           // RUM
//static const char lcd_wlist_full_rum2[] = " ";
static const char lcd_wlist_full_pol1[] = "ARCHIWUM PLENE";                         // POL
//static const char lcd_wlist_full_pol2[] = " ";
static const char lcd_wlist_full_sve1[] = "ANNULERA FULLT";                         // SVE
//static const char lcd_wlist_full_sve2[] = " ";

static const char lcd_expired_card_eng1[] = "EXPIRED";                              // ENG: 22 - WRITE_EXPIRED_CARD
static const char lcd_expired_card_eng2[] = "CARD";
static const char lcd_expired_card_ita1[] = "CARTA";                                // ITA
static const char lcd_expired_card_ita2[] = "SCADUTA";
static const char lcd_expired_card_fra1[] = "CARTE";                                // FRA
static const char lcd_expired_card_fra2[] = "EXPIREE";
static const char lcd_expired_card_deu1[] = "ABGELAUFENE";                          // DEU
static const char lcd_expired_card_deu2[] = "CARD";
static const char lcd_expired_card_esp1[] = "TARJETA";                              // ESP
static const char lcd_expired_card_esp2[] = "EXPIRADA";
static const char lcd_expired_card_por1[] = "CARTAO";                               // POR
static const char lcd_expired_card_por2[] = "EXPIRADO";
static const char lcd_expired_card_rum1[] = "CARD";                                 // RUM
static const char lcd_expired_card_rum2[] = "EXPIRAT";
static const char lcd_expired_card_pol1[] = "WYGASLA";                              // POL
static const char lcd_expired_card_pol2[] = "KARTA";
static const char lcd_expired_card_sve1[] = "TGATT";                                // SVE
static const char lcd_expired_card_sve2[] = "KORT";

static const char lcd_credit_null_eng1[] = "NULL";                                  // ENG: 23 - WRITE_CREDIT_NULL
static const char lcd_credit_null_eng2[] = "CREDIT";
static const char lcd_credit_null_ita1[] = "CREDITO";                               // ITA
static const char lcd_credit_null_ita2[] = "ESAURITO";
static const char lcd_credit_null_fra1[] = "CREDIT";                                // FRA
static const char lcd_credit_null_fra2[] = "EPUISE";
static const char lcd_credit_null_deu1[] = "ERSCHOPFTES";                           // DEU
static const char lcd_credit_null_deu2[] = "KREDIT";
static const char lcd_credit_null_esp1[] = "CREDITO";                               // ESP
static const char lcd_credit_null_esp2[] = "AGOTADO";
static const char lcd_credit_null_por1[] = "CREDITO";                               // POR
static const char lcd_credit_null_por2[] = "ESGOTADO";
static const char lcd_credit_null_rum1[] = "CREDIT";                                // RUM
static const char lcd_credit_null_rum2[] = "EPUIZAT";
static const char lcd_credit_null_pol1[] = "BRAK";                                  // POL
static const char lcd_credit_null_pol2[] = "SRODKOW";
static const char lcd_credit_null_sve1[] = "INGEN";                                 // SVE
static const char lcd_credit_null_sve2[] = "KREDIT";

static const char lcd_close_lid_eng1[] = "CLOSE";                                   // ENG: 24 - WRITE_CLOSE_LID
static const char lcd_close_lid_eng2[] = "LID";
static const char lcd_close_lid_ita1[] = "CHIUDERE";                                // ITA
static const char lcd_close_lid_ita2[] = "COPERCHIO";
static const char lcd_close_lid_fra1[] = "FERMER";                                  // FRA
static const char lcd_close_lid_fra2[] = "LE COUVERCLE";
static const char lcd_close_lid_deu1[] = "DECKEL";                                  // DEU
static const char lcd_close_lid_deu2[] = "SCHLIESSEN";
static const char lcd_close_lid_esp1[] = "CIERRE";                                  // ESP
static const char lcd_close_lid_esp2[] = "LA TAPA";
static const char lcd_close_lid_por1[] = "FECHAR";                                  // POR
static const char lcd_close_lid_por2[] = "TAMPA";
static const char lcd_close_lid_rum1[] = "INCHIDETI";                               // RUM
static const char lcd_close_lid_rum2[] = "CAPACUL";
static const char lcd_close_lid_pol1[] = "ZAMKNIJ";                                 // POL
static const char lcd_close_lid_pol2[] = "POKRYWE";
static const char lcd_close_lid_sve1[] = "STANG";                                   // SVE
static const char lcd_close_lid_sve2[] = "LOCKET";

static const char lcd_total_energy_eng1[] = "SUPPLIED ENERGY";                      // ENG: 25 - WRITE_TOTAL_ENERGY
//static const char lcd_total_energy_eng2[] = " ";
static const char lcd_total_energy_ita1[] = "ENERGIA FORNITA";                      // ITA
//static const char lcd_total_energy_ita2[] = " ";
static const char lcd_total_energy_fra1[] = "ENERGIE FOURNIE";                      // FRA
//static const char lcd_total_energy_fra2[] = " ";
static const char lcd_total_energy_deu1[] = "ENERGIEVERSORGUNG";                    // DEU
//static const char lcd_total_energy_deu2[] = " ";
static const char lcd_total_energy_esp1[] = "ENERGIA SUMINIST.";                    // ESP
//static const char lcd_total_energy_esp2[] = " ";
static const char lcd_total_energy_por1[] = "ENERGIA FORNECIDA";                    // POR
//static const char lcd_total_energy_por2[] = " ";
static const char lcd_total_energy_rum1[] = "ENERGIE FURNIZATA";                    // RUM
//static const char lcd_total_energy_rum2[] = " ";
static const char lcd_total_energy_pol1[] = "DOSTARCZONA ENERGIA";                  // POL
//static const char lcd_total_energy_pol2[] = " ";
static const char lcd_total_energy_sve1[] = "TILLFÖRD ENERGI";                      // SVE
//static const char lcd_total_energy_sve2[] = " ";

static const char lcd_auth_pending_eng1[] = "PLEASE WAIT";                          // ENG: 26 - WRITE_AUTH_PENDING
//static const char lcd_auth_pending_eng2[] = " ";
static const char lcd_auth_pending_ita1[] = "ATTENDERE";                            // ITA
//static const char lcd_auth_pending_ita2[] = " ";
static const char lcd_auth_pending_fra1[] = "ATTENDRE";                             // FRA
//static const char lcd_auth_pending_fra2[] = " ";
static const char lcd_auth_pending_deu1[] = "WARTEN";                               // DEU
//static const char lcd_auth_pending_deu2[] = " ";
static const char lcd_auth_pending_esp1[] = "ESPERE";                               // ESP
//static const char lcd_auth_pending_esp2[] = " ";
static const char lcd_auth_pending_por1[] = "ESPERAR";                              // POR
//static const char lcd_auth_pending_por2[] = " ";
static const char lcd_auth_pending_rum1[] = "ASTEPTATI";                            // RUM
//static const char lcd_auth_pending_rum2[] = " ";
static const char lcd_auth_pending_pol1[] = "PROSZE CZEKAC";                        // POL
//static const char lcd_auth_pending_pol2[] = " ";
static const char lcd_auth_pending_sve1[] = "VANTA";                                // SVE
//static const char lcd_auth_pending_sve2[] = " ";

static const char lcd_credit_date_eng1[] = "CREDIT ";                               // ENG: 27 - WRITE_CREDIT_DATE
//static const char lcd_credit_date_eng2[] = " ";                                                                   
static const char lcd_credit_date_ita1[] = "CREDITO ";                              // ITA                         
//static const char lcd_credit_date_ita2[] = " ";                                                                    
static const char lcd_credit_date_fra1[] = "CREDIT ";                               // FRA                         
//static const char lcd_credit_date_fra2[] = " ";                                                                    
static const char lcd_credit_date_deu1[] = "KREDIT ";                               // DEU                         
//static const char lcd_credit_date_deu2[] = " ";                                                                    
static const char lcd_credit_date_esp1[] = "CREDITO ";                              // ESP                         
//static const char lcd_credit_date_esp2[] = " ";                                                                    
static const char lcd_credit_date_por1[] = "CREDITO ";                              // POR                         
//static const char lcd_credit_date_por2[] = " ";                                                                    
static const char lcd_credit_date_rum1[] = "CREDIT ";                               // RUM                         
//static const char lcd_credit_date_rum2[] = " ";                                                                    
static const char lcd_credit_date_pol1[] = "SALDO ";                                // POL                         
//static const char lcd_credit_date_pol2[] = " ";                                                                    
static const char lcd_credit_date_sve1[] = "SALDO ";                                // SVE                         
//static const char lcd_credit_date_sve2[] = " ";

static const char lcd_new_credit_date_eng1[] = "NEW CREDIT ";                       // ENG: 28 - WRITE_NEW_CREDIT_DATE
//static const char lcd_new_credit_date_eng2[] = " ";
static const char lcd_new_credit_date_ita1[] = "NUOVO CREDITO ";                    // ITA                        
//static const char lcd_new_credit_date_ita2[] = " ";
static const char lcd_new_credit_date_fra1[] = "NOUVEAU CREDIT ";                   // FRA                        
//static const char lcd_new_credit_date_fra2[] = " ";
static const char lcd_new_credit_date_deu1[] = "NEUER KREDIT ";                     // DEU                        
//static const char lcd_new_credit_date_deu2[] = " ";
static const char lcd_new_credit_date_esp1[] = "NUEVO CREDITO ";                    // ESP                        
//static const char lcd_new_credit_date_esp2[] = " ";
static const char lcd_new_credit_date_por1[] = "NOVO CREDITO ";                     // POR                        
//static const char lcd_new_credit_date_por2[] = " ";
static const char lcd_new_credit_date_rum1[] = "CREDIT NOU ";                       // RUM                        
//static const char lcd_new_credit_date_rum2[] = " ";
static const char lcd_new_credit_date_pol1[] = "NOWE SALDO ";                       // POL                        
//static const char lcd_new_credit_date_pol2[] = " ";
static const char lcd_new_credit_date_sve1[] = "NYTT SALDO ";                       // SVE                        
//static const char lcd_new_credit_date_sve2[] = " ";

static const char lcd_only_date_eng1[] = "CARD DATE";                               // ENG: 29 - WRITE_ONLY_DATE
//static const char lcd_only_date_eng2[] = " ";                                                                  
static const char lcd_only_date_ita1[] = "DATA CARTA";                              // ITA                        
//static const char lcd_only_date_ita2[] = " ";                                                                  
static const char lcd_only_date_fra1[] = "DATE CARTE";                              // FRA                        
//static const char lcd_only_date_fra2[] = " ";                                                                  
static const char lcd_only_date_deu1[] = "CARD DATUM";                              // DEU                        
//static const char lcd_only_date_deu2[] = " ";                                                                  
static const char lcd_only_date_esp1[] = "FECHA TARJ.";                             // ESP                        
//static const char lcd_only_date_esp2[] = " ";                                                                  
static const char lcd_only_date_por1[] = "DATA CARTAO";                             // POR                        
//static const char lcd_only_date_por2[] = " ";                                                                  
static const char lcd_only_date_rum1[] = "DATA CARD";                               // RUM                        
//static const char lcd_only_date_rum2[] = " ";                                                                  
static const char lcd_only_date_pol1[] = "DANE KARTY";                              // POL                        
//static const char lcd_only_date_pol2[] = " ";                                                                  
static const char lcd_only_date_sve1[] = "DATAKORT";                                // SVE                        
//static const char lcd_only_date_sve2[] = " ";                                                                

static const char lcd_new_date_done_eng1[] = "DATE AND TIME";                       // ENG: 30 - WRITE_NEW_DATE_DONE
//static const char lcd_new_date_done_eng2[] = " ";                                                                      
static const char lcd_new_date_done_ita1[] = "DATA E ORA";                          // ITA                      
//static const char lcd_new_date_done_ita2[] = " ";                                                                      
static const char lcd_new_date_done_fra1[] = "DATE ET HEURE";                       // FRA                      
//static const char lcd_new_date_done_fra2[] = " ";                                                                      
static const char lcd_new_date_done_deu1[] = "DATUM UHRZEIT";                       // DEU                      
//static const char lcd_new_date_done_deu2[] = " ";                                                                      
static const char lcd_new_date_done_esp1[] = "FECHA Y HORA";                        // ESP                      
//static const char lcd_new_date_done_esp2[] = " ";                                                                      
static const char lcd_new_date_done_por1[] = "DATA E HORA";                         // POR                      
//static const char lcd_new_date_done_por2[] = " ";                                                                      
static const char lcd_new_date_done_rum1[] = "DATA SI ORA";                         // RUM                      
//static const char lcd_new_date_done_rum2[] = " ";                                                                      
static const char lcd_new_date_done_pol1[] = "DATA I CZAS";                         // POL                      
//static const char lcd_new_date_done_pol2[] = " ";                                                                      
static const char lcd_new_date_done_sve1[] = "DATUM OCH TID";                       // SVE                      
//static const char lcd_new_date_done_sve2[] = " ";

static const char lcd_place_plug_eng1[] = "STOW";                                   // ENG: 31 - WRITE_PLACE_PLUG
static const char lcd_place_plug_eng2[] = "CONNECTOR";
static const char lcd_place_plug_ita1[] = "RIPORRE";                                // ITA
static const char lcd_place_plug_ita2[] = "CONNETTORE";
static const char lcd_place_plug_fra1[] = "REMETTRE";                               // FRA
static const char lcd_place_plug_fra2[] = "LE CONNECTEUR";
static const char lcd_place_plug_deu1[] = "STECKVERBINDER";                         // DEU
static const char lcd_place_plug_deu2[] = "ZURUCKSETZEN";
static const char lcd_place_plug_esp1[] = "COLOCAR";                                // ESP
static const char lcd_place_plug_esp2[] = "EL CONECTOR";
static const char lcd_place_plug_por1[] = "COLOQUE";                                // POR
static const char lcd_place_plug_por2[] = "O CONECTOR";
static const char lcd_place_plug_rum1[] = "PUNE INAPOI";                            // RUM
static const char lcd_place_plug_rum2[] = "CONECTORUL";
static const char lcd_place_plug_pol1[] = "ODLOZ";                                  // POL
static const char lcd_place_plug_pol2[] = "ZLACZE";
static const char lcd_place_plug_sve1[] = "STUVA";                                  // SVE
static const char lcd_place_plug_sve2[] = "KONTAKTDON";

static const char lcd_auth_missed_eng1[] = "AUTHORIZATION";                         // ENG: 32 - WRITE_AUTH_MISSED  
static const char lcd_auth_missed_eng2[] = "FAILED";                                                               
static const char lcd_auth_missed_ita1[] = "AUTORIZZAZIONE";                        // ITA                         
static const char lcd_auth_missed_ita2[] = "MANCATA";                                                              
static const char lcd_auth_missed_fra1[] = "AUTORISATION";                          // FRA                         
static const char lcd_auth_missed_fra2[] = "MANQUEE";                                                              
static const char lcd_auth_missed_deu1[] = "AUTORISIERUNG";                         // DEU                         
static const char lcd_auth_missed_deu2[] = "FEHLGESCHLAGEN";                                                       
static const char lcd_auth_missed_esp1[] = "AUTORIZACION";                          // ESP                         
static const char lcd_auth_missed_esp2[] = "PERDIDA";                                                              
static const char lcd_auth_missed_por1[] = "AUTORIZACAO";                           // POR                         
static const char lcd_auth_missed_por2[] = "PERDA";                                                                
static const char lcd_auth_missed_rum1[] = "AUTORIZARE";                            // RUM                         
static const char lcd_auth_missed_rum2[] = "ESUATA";                                                               
static const char lcd_auth_missed_pol1[] = "AUTORYZACJA";                           // POL                         
static const char lcd_auth_missed_pol2[] = "NIEUDANA";                                                             
static const char lcd_auth_missed_sve1[] = "AUKTORISERING";                         // SVE                         
static const char lcd_auth_missed_sve2[] = "MISSLYCKADES";                                                         

static const char lcd_power_off_eng1[] = "MAINS BREAKDOWN";                         // ENG: 33 - WRITE_POWER_OFF  
static const char lcd_power_off_eng2[] = " ";                                                                  
static const char lcd_power_off_ita1[] = "ASSENZA TENSIONE";                        // ITA                         
static const char lcd_power_off_ita2[] = " ";                                                                
static const char lcd_power_off_fra1[] = "PAS DE TENSION";                          // FRA                         
static const char lcd_power_off_fra2[] = " ";                                                                 
static const char lcd_power_off_deu1[] = "KEINE SPANNUNG";                          // DEU                         
static const char lcd_power_off_deu2[] = " ";                                                          
static const char lcd_power_off_esp1[] = "AUSENCIA TENSION";                        // ESP                         
static const char lcd_power_off_esp2[] = " ";                                                                 
static const char lcd_power_off_por1[] = "SEM TENSAO";                              // POR                         
static const char lcd_power_off_por2[] = " ";                                                                   
static const char lcd_power_off_rum1[] = "LIPSA TENSIUNE";                          // RUM                         
static const char lcd_power_off_rum2[] = " ";                                                                  
static const char lcd_power_off_pol1[] = "BRAK ZASILANIA";                          // POL                         
static const char lcd_power_off_pol2[] = " ";                                                                
static const char lcd_power_off_sve1[] = "INGEN SPANNING";                          // SVE                         
static const char lcd_power_off_sve2[] = " ";                                                         

static const char lcd_write_htswaiting_eng1[] = "OT WAITING";                        // ENG: 34 - WRITE_HTSWAITING
//static const char lcd_write_htswaiting_eng2[] = " ";
static const char lcd_write_htswaiting_ita1[] = "ATTESA OT";                         // ITA
//static const char lcd_write_htswaiting_ita2[] = " ";
static const char lcd_write_htswaiting_fra1[] = "ATTENTE OT";                        // FRA
//static const char lcd_write_htswaiting_fra2[] = " ";
static const char lcd_write_htswaiting_deu1[] = "OT ERWARTUNG";                      // DEU
//static const char lcd_write_htswaiting_deu2[] = " ";
static const char lcd_write_htswaiting_esp1[] = "ESPERA OT";                         // ESP
//static const char lcd_write_htswaiting_esp2[] = " ";
static const char lcd_write_htswaiting_por1[] = "ESPERA OT";                         // POR
//static const char lcd_write_htswaiting_por2[] = " ";
static const char lcd_write_htswaiting_rum1[] = "ASTEAPTA OT";                       // RUM
//static const char lcd_write_htswaiting_rum2[] = " ";
static const char lcd_write_htswaiting_pol1[] = "OT OCZEKUJE";                       // POL
//static const char lcd_write_htswaiting_pol2[] = " ";
static const char lcd_write_htswaiting_sve1[] = "OT VANTAR";                         // SVE
//static const char lcd_write_htswaiting_sve2[] = " ";

static const char lcd_write_emwaiting_eng1[] = "EM WAITING";                         // ENG: 35 - WRITE_EMWAITING
//static const char lcd_write_emswaiting_eng2[] = " ";
static const char lcd_write_emwaiting_ita1[] = "ATTESA EM";                          // ITA
//static const char lcd_write_emswaiting_ita2[] = " ";
static const char lcd_write_emwaiting_fra1[] = "ATTENTE EM";                         // FRA
//static const char lcd_write_emwaiting_fra2[] = " ";
static const char lcd_write_emwaiting_deu1[] = "EM ERWARTUNG";                       // DEU
//static const char lcd_write_emwaiting_deu2[] = " ";
static const char lcd_write_emwaiting_esp1[] = "ESPERA EM";                          // ESP
//static const char lcd_write_emwaiting_esp2[] = " ";
static const char lcd_write_emwaiting_por1[] = "ESPERA EM";                          // POR
//static const char lcd_write_emwaiting_por2[] = " ";
static const char lcd_write_emwaiting_rum1[] = "ASTEAPTA EM";                        // RUM
//static const char lcd_write_emwaiting_rum2[] = " ";
static const char lcd_write_emwaiting_pol1[] = "EM OCZEKUJE";                        // POL
//static const char lcd_write_emwaiting_pol2[] = " ";
static const char lcd_write_emwaiting_sve1[] = "EM VANTAR";                          // SVE
//static const char lcd_write_emwaiting_sve2[] = " ";

//static const char lcd_write_reserved_eng1[] = "SHOW CARD";                         // ENG: 01 - WRITE_RESERVED
static const char lcd_write_reserved_eng2[] = "RESERVED";
//static const char lcd_write_reserved_ita1[] = "PRESENTARE CARTA";                  // ITA
static const char lcd_write_reserved_ita2[] = "RISERVATO";
//static const char lcd_write_reserved_fra1[] = "PRESENTER CARTE";                   // FRA
static const char lcd_write_reserved_fra2[] = "RESERVE";
//static const char lcd_write_reserved_deu1[] = "EINSTELLEN CARD";                   // DEU
static const char lcd_write_reserved_deu2[] = "RESERVIERT";
//static const char lcd_write_reserved_esp1[] = "MOSTRAR TARJETA";                   // ESP
static const char lcd_write_reserved_esp2[] = "RESERVADO";
//static const char lcd_write_reserved_por1[] = "APRESENTAR CARTAO";                 // POR
static const char lcd_write_reserved_por2[] = "RESERVADO";
//static const char lcd_write_reserved_rum1[] = "PREZENTATI CARD";                   // RUM
static const char lcd_write_reserved_rum2[] = "REZERVAT";
//static const char lcd_write_reserved_pol1[] = "POKAZ KARTE";                       // POL
static const char lcd_write_reserved_pol2[] = "SKRYTY";
//static const char lcd_write_reserved_sve1[] = "VISA KORT";                         // SVE
static const char lcd_write_reserved_sve2[] = "RESERVERAD";

#ifndef USE_SLEEP_MODE_WHEN_VOLTAGE_IS_ABSENT
static const char lcd_power_off[] = "ÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿ";                      // Vin STATUS BAR                                              
#endif

static  const char* const lang_table[] = 
    {
    lcd_write_evs_disabled_eng1, lcd_write_evs_disabled_eng2,                       // 00 - WRITE_EVS_DISABLED        ENG
    lcd_write_card_wait_eng1, lcd_write_empty_line,                                 // 01 - WRITE_CARD_WAIT
    lcd_write_socket_available_eng1, lcd_write_socket_available_eng2,               // 02 - WRITE_SOCKET_AVAILABLE
    lcd_write_plug_check_eng1, lcd_write_plug_check_eng2,                           // 03 - WRITE_PLUG_CHECK
    lcd_write_empty_line, lcd_write_plug_out_eng2,                                  // 04 - WRITE_PLUG_OUT
    lcd_write_s2_waiting_eng1, lcd_write_empty_line,                                // 05 - WRITE_S2_WAIT
    lcd_write_charging_eng1, lcd_write_empty_line,                                  // 06 - WRITE_CHARGING
    lcd_write_suspendig_eng1, lcd_write_empty_line,                                 // 07 - WRITE_SUSPENDING
    lcd_write_rmwaiting_eng1, lcd_write_empty_line,                                 // 08 - WRITE_RMWAITING
    lcd_write_interrupting_charge_eng1, lcd_write_interrupting_charge_eng2,         // 09 - WRITE_INTERRUPTING_CHARGE
    lcd_write_update_wlist_eng1, lcd_write_update_wlist_eng2,                       // 10 - WRITE_UPDATE_WLIST
    lcd_uid_delete_confirm_eng1, lcd_write_empty_line,                              // 11 - WRITE_UID_DELETE_CONFIRM
    lcd_uid_deleted_eng1, lcd_uid_users_eng2,                                       // 12 - WRITE_UID_DELETED
    lcd_uid_added_eng1, lcd_uid_users_eng2,                                         // 13 - WRITE_UID_ADDED
    lcd_uid_error_eng1, lcd_uid_error_eng2,                                         // 14 - WRITE_UID_ERROR
    lcd_delete_wlist_confirm_eng1, lcd_write_empty_line,                            // 15 - WRITE_DELETE_WLIST_CONFIRM
    lcd_authorization_failed_eng1, lcd_authorization_failed_eng2,                   // 16 - WRITE_AUTHORIZATION_FAILED
    lcd_plug_wait_eng1, lcd_write_empty_line,                                       // 17 - WRITE_PLUG_WAIT
    lcd_wlist_deleted_eng1, lcd_write_empty_line,                                   // 18 - WRITE_WLIST_DELETED
    lcd_reset_failed_eng1, lcd_reset_failed_eng2,                                   // 19 - WRITE_RESET_FAILED
    lcd_delete_error_eng1, lcd_delete_error_eng2,                                   // 20 - WRITE_DELETE_ERROR
    lcd_wlist_full_eng1, lcd_write_empty_line,                                      // 21 - WRITE_WLIST_FULL
    lcd_expired_card_eng1, lcd_expired_card_eng2,                                   // 22 - WRITE_EXPIRED_CARD
    lcd_credit_null_eng1, lcd_credit_null_eng2,                                     // 23 - WRITE_CREDIT_NULL
    lcd_close_lid_eng1, lcd_close_lid_eng2,                                         // 24 - WRITE_CLOSE_LID
    lcd_total_energy_eng1, lcd_write_empty_line,                                    // 25 - WRITE_TOTAL_ENERGY
    lcd_auth_pending_eng1, lcd_write_empty_line,                                    // 26 - WRITE_AUTH_PENDING
    lcd_credit_date_eng1, lcd_write_empty_line,                                     // 27 - WRITE_CREDIT_DATE
    lcd_new_credit_date_eng1, lcd_write_empty_line,                                 // 28 - WRITE_NEW_CREDIT_DATE
    lcd_only_date_eng1, lcd_write_empty_line,                                       // 29 - WRITE_ONLY_DATE
    lcd_new_date_done_eng1, lcd_write_empty_line,                                   // 30 - WRITE_NEW_DATE_DONE
    lcd_place_plug_eng1, lcd_place_plug_eng2,                                       // 31 - WRITE_PLACE_PLUG
    lcd_auth_missed_eng1, lcd_auth_missed_eng2,                                     // 32 - WRITE_AUTH_MISSED  
    lcd_power_off_eng1, lcd_power_off_eng2,                                         // 33 - WRITE_POWER_OFF  
    lcd_write_htswaiting_eng1, lcd_write_empty_line,                                // 34 - WRITE_HTSWAITING  
    lcd_write_emwaiting_eng1, lcd_write_empty_line,                                 // 35 - WRITE_EMWAITING  
    lcd_write_empty_line, lcd_write_reserved_eng2,                                  // 36 - WRITE_RESERVED
    
    lcd_write_evs_disabled_ita1, lcd_write_evs_disabled_ita2,                       // 00 - WRITE_EVS_DISABLED        ITA
    lcd_write_card_wait_ita1, lcd_write_empty_line,                                 // 01 - WRITE_CARD_WAIT
    lcd_write_socket_available_ita1, lcd_write_socket_available_ita2,               // 02 - WRITE_SOCKET_AVAILABLE
    lcd_write_plug_check_ita1, lcd_write_plug_check_ita2,                           // 03 - WRITE_PLUG_CHECK
    lcd_write_empty_line, lcd_write_plug_out_ita2,                                  // 04 - WRITE_PLUG_OUT
    lcd_write_s2_waiting_ita1, lcd_write_empty_line,                                // 05 - WRITE_S2_WAIT
    lcd_write_charging_ita1, lcd_write_empty_line,                                  // 06 - WRITE_CHARGING
    lcd_write_suspendig_ita1, lcd_write_empty_line,                                 // 07 - WRITE_SUSPENDING
    lcd_write_rmwaiting_ita1, lcd_write_empty_line,                                 // 08 - WRITE_RMWAITING
    lcd_write_interrupting_charge_ita1, lcd_write_interrupting_charge_ita2,         // 09 - WRITE_INTERRUPTING_CHARGE
    lcd_write_update_wlist_ita1, lcd_write_update_wlist_ita2,                       // 10 - WRITE_UPDATE_WLIST
    lcd_uid_delete_confirm_ita1, lcd_write_empty_line,                              // 11 - WRITE_UID_DELETE_CONFIRM
    lcd_uid_deleted_ita1, lcd_uid_users_ita2,                                       // 12 - WRITE_UID_DELETED
    lcd_uid_added_ita1, lcd_uid_users_ita2,                                         // 13 - WRITE_UID_ADDED
    lcd_uid_error_ita1, lcd_uid_error_ita2,                                         // 14 - WRITE_UID_ERROR
    lcd_delete_wlist_confirm_ita1, lcd_write_empty_line,                            // 15 - WRITE_DELETE_WLIST_CONFIRM
    lcd_authorization_failed_ita1, lcd_authorization_failed_ita2,                   // 16 - WRITE_AUTHORIZATION_FAILED
    lcd_plug_wait_ita1, lcd_write_empty_line,                                       // 17 - WRITE_PLUG_WAIT
    lcd_wlist_deleted_ita1, lcd_write_empty_line,                                   // 18 - WRITE_WLIST_DELETED
    lcd_reset_failed_ita1, lcd_reset_failed_ita2,                                   // 19 - WRITE_RESET_FAILED
    lcd_delete_error_ita1, lcd_delete_error_ita2,                                   // 20 - WRITE_DELETE_ERROR
    lcd_wlist_full_ita1, lcd_write_empty_line,                                      // 21 - WRITE_WLIST_FULL
    lcd_expired_card_ita1, lcd_expired_card_ita2,                                   // 22 - WRITE_EXPIRED_CARD
    lcd_credit_null_ita1, lcd_credit_null_ita2,                                     // 23 - WRITE_CREDIT_NULL
    lcd_close_lid_ita1, lcd_close_lid_ita2,                                         // 24 - WRITE_CLOSE_LID
    lcd_total_energy_ita1, lcd_write_empty_line,                                    // 25 - WRITE_TOTAL_ENERGY
    lcd_auth_pending_ita1 , lcd_write_empty_line,                                   // 26 - WRITE_AUTH_PENDING
    lcd_credit_date_ita1, lcd_write_empty_line,                                     // 27 - WRITE_CREDIT_DATE
    lcd_new_credit_date_ita1, lcd_write_empty_line,                                 // 28 - WRITE_NEW_CREDIT_DATE
    lcd_only_date_ita1, lcd_write_empty_line,                                       // 29 - WRITE_ONLY_DATE
    lcd_new_date_done_ita1, lcd_write_empty_line,                                   // 30 - WRITE_NEW_DATE_DONE
    lcd_place_plug_ita1, lcd_place_plug_ita2,                                       // 31 - WRITE_PLACE_PLUG
    lcd_auth_missed_ita1, lcd_auth_missed_ita2,                                     // 32 - WRITE_AUTH_MISSED
    lcd_power_off_ita1, lcd_power_off_ita2,                                         // 33 - WRITE_POWER_OFF
    lcd_write_htswaiting_ita1, lcd_write_empty_line,                                // 34 - WRITE_HTSWAITING  
    lcd_write_emwaiting_ita1, lcd_write_empty_line,                                 // 35 - WRITE_EMWAITING
    lcd_write_empty_line, lcd_write_reserved_ita2,                                  // 36 - WRITE_RESERVED  

    lcd_write_evs_disabled_fra1, lcd_write_evs_disabled_fra2,                       // 00 - WRITE_EVS_DISABLED         FRA
    lcd_write_card_wait_fra1, lcd_write_empty_line,                                 // 01 - WRITE_CARD_WAIT
    lcd_write_socket_available_fra1, lcd_write_socket_available_fra2,               // 02 - WRITE_SOCKET_AVAILABLE
    lcd_write_plug_check_fra1, lcd_write_plug_check_fra2,                           // 03 - WRITE_PLUG_CHECK
    lcd_write_empty_line, lcd_write_plug_out_fra2,                                  // 04 - WRITE_PLUG_OUT
    lcd_write_s2_waiting_fra1, lcd_write_empty_line,                                // 05 - WRITE_S2_WAIT
    lcd_write_charging_fra1, lcd_write_empty_line,                                  // 06 - WRITE_CHARGING
    lcd_write_suspendig_fra1, lcd_write_empty_line,                                 // 07 - WRITE_SUSPENDING
    lcd_write_rmwaiting_fra1, lcd_write_empty_line,                                 // 08 - WRITE_RMWAITING
    lcd_write_interrupting_charge_fra1, lcd_write_interrupting_charge_fra2,         // 09 - WRITE_INTERRUPTING_CHARGE
    lcd_write_update_wlist_fra1, lcd_write_update_wlist_fra2,                       // 10 - WRITE_UPDATE_WLIST
    lcd_uid_delete_confirm_fra1, lcd_write_empty_line,                              // 11 - WRITE_UID_DELETE_CONFIRM
    lcd_uid_deleted_fra1, lcd_uid_users_fra2,                                       // 12 - WRITE_UID_DELETED
    lcd_uid_added_fra1, lcd_uid_users_fra2,                                         // 13 - WRITE_UID_ADDED
    lcd_uid_error_fra1, lcd_uid_error_fra2,                                         // 14 - WRITE_UID_ERROR
    lcd_delete_wlist_confirm_fra1, lcd_write_empty_line,                            // 15 - WRITE_DELETE_WLIST_CONFIRM
    lcd_authorization_failed_fra1, lcd_authorization_failed_fra2,                   // 16 - WRITE_AUTHORIZATION_FAILED
    lcd_plug_wait_fra1, lcd_write_empty_line,                                       // 17 - WRITE_PLUG_WAIT
    lcd_wlist_deleted_fra1, lcd_write_empty_line,                                   // 18 - WRITE_WLIST_DELETED
    lcd_reset_failed_fra1, lcd_reset_failed_fra2,                                   // 19 - WRITE_RESET_FAILED
    lcd_delete_error_fra1, lcd_delete_error_fra2,                                   // 20 - WRITE_DELETE_ERROR
    lcd_wlist_full_fra1, lcd_write_empty_line,                                      // 21 - WRITE_WLIST_FULL
    lcd_expired_card_fra1, lcd_expired_card_fra2,                                   // 22 - WRITE_EXPIRED_CARD
    lcd_credit_null_fra1, lcd_credit_null_fra2,                                     // 23 - WRITE_CREDIT_NULL
    lcd_close_lid_fra1, lcd_close_lid_fra2,                                         // 24 - WRITE_CLOSE_LID
    lcd_total_energy_fra1, lcd_write_empty_line,                                    // 25 - WRITE_TOTAL_ENERGY
    lcd_auth_pending_fra1 , lcd_write_empty_line,                                   // 26 - WRITE_AUTH_PENDING
    lcd_credit_date_fra1, lcd_write_empty_line,                                     // 27 - WRITE_CREDIT_DATE
    lcd_new_credit_date_fra1, lcd_write_empty_line,                                 // 28 - WRITE_NEW_CREDIT_DATE
    lcd_only_date_fra1, lcd_write_empty_line,                                       // 29 - WRITE_ONLY_DATE
    lcd_new_date_done_fra1, lcd_write_empty_line,                                   // 30 - WRITE_NEW_DATE_DONE
    lcd_place_plug_fra1, lcd_place_plug_fra2,                                       // 31 - WRITE_PLACE_PLUG
    lcd_auth_missed_fra1, lcd_auth_missed_fra2,                                     // 32 - WRITE_AUTH_MISSED  
    lcd_power_off_fra1, lcd_power_off_fra2,                                         // 33 - WRITE_POWER_OFF
    lcd_write_htswaiting_fra1, lcd_write_empty_line,                                // 34 - WRITE_HTSWAITING  
    lcd_write_emwaiting_fra1, lcd_write_empty_line,                                 // 35 - WRITE_EMWAITING
    lcd_write_empty_line, lcd_write_reserved_fra2,                                  // 36 - WRITE_RESERVED  

    lcd_write_evs_disabled_deu1, lcd_write_evs_disabled_deu2,                       // 00 - WRITE_EVS_DISABLED         DEU
    lcd_write_card_wait_deu1, lcd_write_empty_line,                                 // 01 - WRITE_CARD_WAIT
    lcd_write_socket_available_deu1, lcd_write_socket_available_deu2,               // 02 - WRITE_SOCKET_AVAILABLE
    lcd_write_plug_check_deu1, lcd_write_plug_check_deu2,                           // 03 - WRITE_PLUG_CHECK
    lcd_write_empty_line, lcd_write_plug_out_deu2,                                  // 04 - WRITE_PLUG_OUT
    lcd_write_s2_waiting_deu1, lcd_write_empty_line,                                // 05 - WRITE_S2_WAIT
    lcd_write_charging_deu1, lcd_write_empty_line,                                  // 06 - WRITE_CHARGING
    lcd_write_suspendig_deu1, lcd_write_empty_line,                                 // 07 - WRITE_SUSPENDING
    lcd_write_rmwaiting_deu1, lcd_write_empty_line,                                 // 08 - WRITE_RMWAITING
    lcd_write_interrupting_charge_deu1, lcd_write_interrupting_charge_deu2,         // 09 - WRITE_INTERRUPTING_CHARGE
    lcd_write_update_wlist_deu1, lcd_write_update_wlist_deu2,                       // 10 - WRITE_UPDATE_WLIST
    lcd_uid_delete_confirm_deu1, lcd_write_empty_line,                              // 11 - WRITE_UID_DELETE_CONFIRM
    lcd_uid_deleted_deu1, lcd_uid_users_deu2,                                       // 12 - WRITE_UID_DELETED
    lcd_uid_added_deu1, lcd_uid_users_deu2,                                         // 13 - WRITE_UID_ADDED
    lcd_uid_error_deu1, lcd_uid_error_deu2,                                         // 14 - WRITE_UID_ERROR
    lcd_delete_wlist_confirm_deu1, lcd_write_empty_line,                            // 15 - WRITE_DELETE_WLIST_CONFIRM
    lcd_authorization_failed_deu1, lcd_authorization_failed_deu2,                   // 16 - WRITE_AUTHORIZATION_FAILED
    lcd_plug_wait_deu1, lcd_write_empty_line,                                       // 17 - WRITE_PLUG_WAIT
    lcd_wlist_deleted_deu1, lcd_write_empty_line,                                   // 18 - WRITE_WLIST_DELETED
    lcd_reset_failed_deu1, lcd_reset_failed_deu2,                                   // 19 - WRITE_RESET_FAILED
    lcd_delete_error_deu1, lcd_delete_error_deu2,                                   // 20 - WRITE_DELETE_ERROR
    lcd_wlist_full_deu1, lcd_write_empty_line,                                      // 21 - WRITE_WLIST_FULL
    lcd_expired_card_deu1, lcd_expired_card_deu2,                                   // 22 - WRITE_EXPIRED_CARD
    lcd_credit_null_deu1, lcd_credit_null_deu2,                                     // 23 - WRITE_CREDIT_NULL
    lcd_close_lid_deu1, lcd_close_lid_deu2,                                         // 24 - WRITE_CLOSE_LID
    lcd_total_energy_deu1, lcd_write_empty_line,                                    // 25 - WRITE_TOTAL_ENERGY
    lcd_auth_pending_deu1 , lcd_write_empty_line,                                   // 26 - WRITE_AUTH_PENDING
    lcd_credit_date_deu1, lcd_write_empty_line,                                     // 27 - WRITE_CREDIT_DATE
    lcd_new_credit_date_deu1, lcd_write_empty_line,                                 // 28 - WRITE_NEW_CREDIT_DATE
    lcd_only_date_deu1, lcd_write_empty_line,                                       // 29 - WRITE_ONLY_DATE
    lcd_new_date_done_deu1, lcd_write_empty_line,                                   // 30 - WRITE_NEW_DATE_DONE
    lcd_place_plug_deu1, lcd_place_plug_deu2,                                       // 31 - WRITE_PLACE_PLUG
    lcd_auth_missed_deu1, lcd_auth_missed_deu2,                                     // 32 - WRITE_AUTH_MISSED  
    lcd_power_off_deu1, lcd_power_off_deu2,                                         // 33 - WRITE_POWER_OFF
    lcd_write_htswaiting_deu1, lcd_write_empty_line,                                // 34 - WRITE_HTSWAITING  
    lcd_write_emwaiting_deu1, lcd_write_empty_line,                                 // 35 - WRITE_EMWAITING
    lcd_write_empty_line, lcd_write_reserved_deu2,                                  // 36 - WRITE_RESERVED  

    lcd_write_evs_disabled_esp1, lcd_write_evs_disabled_esp2,                       // 00 - WRITE_EVS_DISABLED:         ESP
    lcd_write_card_wait_esp1, lcd_write_empty_line,                                 // 01 - WRITE_CARD_WAIT
    lcd_write_socket_available_esp1, lcd_write_socket_available_esp2,               // 02 - WRITE_SOCKET_AVAILABLE
    lcd_write_plug_check_esp1, lcd_write_plug_check_esp2,                           // 03 - WRITE_PLUG_CHECK
    lcd_write_empty_line, lcd_write_plug_out_esp2,                                  // 04 - WRITE_PLUG_OUT
    lcd_write_s2_waiting_esp1, lcd_write_empty_line,                                // 05 - WRITE_S2_WAIT
    lcd_write_charging_esp1, lcd_write_empty_line,                                  // 06 - WRITE_CHARGING
    lcd_write_suspendig_esp1, lcd_write_empty_line,                                 // 07 - WRITE_SUSPENDING
    lcd_write_rmwaiting_esp1, lcd_write_empty_line,                                 // 08 - WRITE_RMWAITING
    lcd_write_interrupting_charge_esp1, lcd_write_interrupting_charge_esp2,         // 09 - WRITE_INTERRUPTING_CHARGE
    lcd_write_update_wlist_esp1, lcd_write_update_wlist_esp2,                       // 10 - WRITE_UPDATE_WLIST
    lcd_uid_delete_confirm_esp1, lcd_write_empty_line,                              // 11 - WRITE_UID_DELETE_CONFIRM
    lcd_uid_deleted_esp1, lcd_uid_users_esp2,                                       // 12 - WRITE_UID_DELETED
    lcd_uid_added_esp1, lcd_uid_users_esp2,                                         // 13 - WRITE_UID_ADDED
    lcd_uid_error_esp1, lcd_uid_error_esp2,                                         // 14 - WRITE_UID_ERROR
    lcd_delete_wlist_confirm_esp1, lcd_write_empty_line,                            // 15 - WRITE_DELETE_WLIST_CONFIRM
    lcd_authorization_failed_esp1, lcd_authorization_failed_esp2,                   // 16 - WRITE_AUTHORIZATION_FAILED
    lcd_plug_wait_esp1, lcd_write_empty_line,                                       // 17 - WRITE_PLUG_WAIT
    lcd_wlist_deleted_esp1, lcd_write_empty_line,                                   // 18 - WRITE_WLIST_DELETED
    lcd_reset_failed_esp1, lcd_reset_failed_esp2,                                   // 19 - WRITE_RESET_FAILED
    lcd_delete_error_esp1, lcd_delete_error_esp2,                                   // 20 - WRITE_DELETE_ERROR
    lcd_wlist_full_esp1, lcd_write_empty_line,                                      // 21 - WRITE_WLIST_FULL
    lcd_expired_card_esp1, lcd_expired_card_esp2,                                   // 22 - WRITE_EXPIRED_CARD
    lcd_credit_null_esp1, lcd_credit_null_esp2,                                     // 23 - WRITE_CREDIT_NULL
    lcd_close_lid_esp1, lcd_close_lid_esp2,                                         // 24 - WRITE_CLOSE_LID
    lcd_total_energy_esp1, lcd_write_empty_line,                                    // 25 - WRITE_TOTAL_ENERGY
    lcd_auth_pending_esp1 , lcd_write_empty_line,                                   // 26 - WRITE_AUTH_PENDING
    lcd_credit_date_esp1, lcd_write_empty_line,                                     // 27 - WRITE_CREDIT_DATE
    lcd_new_credit_date_esp1, lcd_write_empty_line,                                 // 28 - WRITE_NEW_CREDIT_DATE
    lcd_only_date_esp1, lcd_write_empty_line,                                       // 29 - WRITE_ONLY_DATE
    lcd_new_date_done_esp1, lcd_write_empty_line,                                   // 30 - WRITE_NEW_DATE_DONE
    lcd_place_plug_esp1, lcd_place_plug_esp2,                                       // 31 - WRITE_PLACE_PLUG
    lcd_auth_missed_esp1, lcd_auth_missed_esp2,                                     // 32 - WRITE_AUTH_MISSED  
    lcd_power_off_esp1, lcd_power_off_esp2,                                         // 33 - WRITE_POWER_OFF
    lcd_write_htswaiting_esp1, lcd_write_empty_line,                                // 34 - WRITE_HTSWAITING  
    lcd_write_emwaiting_esp1, lcd_write_empty_line,                                 // 35 - WRITE_EMWAITING
    lcd_write_empty_line, lcd_write_reserved_esp2,                                  // 36 - WRITE_RESERVED  

    lcd_write_evs_disabled_por1, lcd_write_evs_disabled_por2,                       // 00 - WRITE_EVS_DISABLED:        POR
    lcd_write_card_wait_por1, lcd_write_empty_line,                                 // 01 - WRITE_CARD_WAIT
    lcd_write_socket_available_por1, lcd_write_socket_available_por2,               // 02 - WRITE_SOCKET_AVAILABLE
    lcd_write_plug_check_por1, lcd_write_plug_check_por2,                           // 03 - WRITE_PLUG_CHECK
    lcd_write_empty_line, lcd_write_plug_out_por2,                                  // 04 - WRITE_PLUG_OUT
    lcd_write_s2_waiting_por1, lcd_write_empty_line,                                // 05 - WRITE_S2_WAIT
    lcd_write_charging_por1, lcd_write_empty_line,                                  // 06 - WRITE_CHARGING
    lcd_write_suspendig_por1, lcd_write_empty_line,                                 // 07 - WRITE_SUSPENDING
    lcd_write_rmwaiting_por1, lcd_write_empty_line,                                 // 08 - WRITE_RMWAITING
    lcd_write_interrupting_charge_por1, lcd_write_interrupting_charge_por2,         // 09 - WRITE_INTERRUPTING_CHARGE
    lcd_write_update_wlist_por1, lcd_write_update_wlist_por2,                       // 10 - WRITE_UPDATE_WLIST
    lcd_uid_delete_confirm_por1, lcd_write_empty_line,                              // 11 - WRITE_UID_DELETE_CONFIRM
    lcd_uid_deleted_por1, lcd_uid_users_por2,                                       // 12 - WRITE_UID_DELETED
    lcd_uid_added_por1, lcd_uid_users_por2,                                         // 13 - WRITE_UID_ADDED
    lcd_uid_error_por1, lcd_uid_error_por2,                                         // 14 - WRITE_UID_ERROR
    lcd_delete_wlist_confirm_por1, lcd_write_empty_line,                            // 15 - WRITE_DELETE_WLIST_CONFIRM
    lcd_authorization_failed_por1, lcd_authorization_failed_por2,                   // 16 - WRITE_AUTHORIZATION_FAILED
    lcd_plug_wait_por1, lcd_write_empty_line,                                       // 17 - WRITE_PLUG_WAIT
    lcd_wlist_deleted_por1, lcd_write_empty_line,                                   // 18 - WRITE_WLIST_DELETED
    lcd_reset_failed_por1, lcd_reset_failed_por2,                                   // 19 - WRITE_RESET_FAILED
    lcd_delete_error_por1, lcd_delete_error_por2,                                   // 20 - WRITE_DELETE_ERROR
    lcd_wlist_full_por1, lcd_write_empty_line,                                      // 21 - WRITE_WLIST_FULL
    lcd_expired_card_por1, lcd_expired_card_por2,                                   // 22 - WRITE_EXPIRED_CARD
    lcd_credit_null_por1, lcd_credit_null_por2,                                     // 23 - WRITE_CREDIT_NULL
    lcd_close_lid_por1, lcd_close_lid_por2,                                         // 24 - WRITE_CLOSE_LID
    lcd_total_energy_por1, lcd_write_empty_line,                                    // 25 - WRITE_TOTAL_ENERGY
    lcd_auth_pending_por1 , lcd_write_empty_line,                                   // 26 - WRITE_AUTH_PENDING
    lcd_credit_date_por1, lcd_write_empty_line,                                     // 27 - WRITE_CREDIT_DATE
    lcd_new_credit_date_por1, lcd_write_empty_line,                                 // 28 - WRITE_NEW_CREDIT_DATE
    lcd_only_date_por1, lcd_write_empty_line,                                       // 29 - WRITE_ONLY_DATE
    lcd_new_date_done_por1, lcd_write_empty_line,                                   // 30 - WRITE_NEW_DATE_DONE
    lcd_place_plug_por1, lcd_place_plug_por2,                                       // 31 - WRITE_PLACE_PLUG
    lcd_auth_missed_por1, lcd_auth_missed_por2,                                     // 32 - WRITE_AUTH_MISSED  
    lcd_power_off_por1, lcd_power_off_por2,                                         // 33 - WRITE_POWER_OFF
    lcd_write_htswaiting_por1, lcd_write_empty_line,                                // 34 - WRITE_HTSWAITING  
    lcd_write_emwaiting_por1, lcd_write_empty_line,                                 // 35 - WRITE_EMWAITING
    lcd_write_empty_line, lcd_write_reserved_por2,                                  // 36 - WRITE_RESERVED  

    lcd_write_evs_disabled_rum1, lcd_write_evs_disabled_rum2,                       // 00 - WRITE_EVS_DISABLED:         RUM
    lcd_write_card_wait_rum1, lcd_write_empty_line,                                 // 01 - WRITE_CARD_WAIT
    lcd_write_socket_available_rum1, lcd_write_socket_available_rum2,               // 02 - WRITE_SOCKET_AVAILABLE
    lcd_write_plug_check_rum1, lcd_write_plug_check_rum2,                           // 03 - WRITE_PLUG_CHECK
    lcd_write_empty_line, lcd_write_plug_out_rum2,                                  // 04 - WRITE_PLUG_OUT
    lcd_write_s2_waiting_rum1, lcd_write_empty_line,                                // 05 - WRITE_S2_WAIT
    lcd_write_charging_rum1, lcd_write_empty_line,                                  // 06 - WRITE_CHARGING
    lcd_write_suspendig_rum1, lcd_write_empty_line,                                 // 07 - WRITE_SUSPENDING
    lcd_write_rmwaiting_rum1, lcd_write_empty_line,                                 // 08 - WRITE_RMWAITING
    lcd_write_interrupting_charge_rum1, lcd_write_interrupting_charge_rum2,         // 09 - WRITE_INTERRUPTING_CHARGE
    lcd_write_update_wlist_rum1, lcd_write_update_wlist_rum2,                       // 10 - WRITE_UPDATE_WLIST
    lcd_uid_delete_confirm_rum1, lcd_write_empty_line,                              // 11 - WRITE_UID_DELETE_CONFIRM
    lcd_uid_deleted_rum1, lcd_uid_users_rum2,                                       // 12 - WRITE_UID_DELETED
    lcd_uid_added_rum1, lcd_uid_users_rum2,                                         // 13 - WRITE_UID_ADDED
    lcd_uid_error_rum1, lcd_uid_error_rum2,                                         // 14 - WRITE_UID_ERROR
    lcd_delete_wlist_confirm_rum1, lcd_write_empty_line,                            // 15 - WRITE_DELETE_WLIST_CONFIRM
    lcd_authorization_failed_rum1, lcd_authorization_failed_rum2,                   // 16 - WRITE_AUTHORIZATION_FAILED
    lcd_plug_wait_rum1, lcd_write_empty_line,                                       // 17 - WRITE_PLUG_WAIT
    lcd_wlist_deleted_rum1, lcd_write_empty_line,                                   // 18 - WRITE_WLIST_DELETED
    lcd_reset_failed_rum1, lcd_reset_failed_rum2,                                   // 19 - WRITE_RESET_FAILED
    lcd_delete_error_rum1, lcd_delete_error_rum2,                                   // 20 - WRITE_DELETE_ERROR
    lcd_wlist_full_rum1, lcd_write_empty_line,                                      // 21 - WRITE_WLIST_FULL
    lcd_expired_card_rum1, lcd_expired_card_rum2,                                   // 22 - WRITE_EXPIRED_CARD
    lcd_credit_null_rum1, lcd_credit_null_rum2,                                     // 23 - WRITE_CREDIT_NULL
    lcd_close_lid_rum1, lcd_close_lid_rum2,                                         // 24 - WRITE_CLOSE_LID
    lcd_total_energy_rum1, lcd_write_empty_line,                                    // 25 - WRITE_TOTAL_ENERGY
    lcd_auth_pending_rum1 , lcd_write_empty_line,                                   // 26 - WRITE_AUTH_PENDING
    lcd_credit_date_rum1, lcd_write_empty_line,                                     // 27 - WRITE_CREDIT_DATE
    lcd_new_credit_date_rum1, lcd_write_empty_line,                                 // 28 - WRITE_NEW_CREDIT_DATE
    lcd_only_date_rum1, lcd_write_empty_line,                                       // 29 - WRITE_ONLY_DATE
    lcd_new_date_done_rum1, lcd_write_empty_line,                                   // 30 - WRITE_NEW_DATE_DONE
    lcd_place_plug_rum1, lcd_place_plug_rum2,                                       // 31 - WRITE_PLACE_PLUG
    lcd_auth_missed_rum1, lcd_auth_missed_rum2,                                     // 32 - WRITE_AUTH_MISSED  
    lcd_power_off_rum1, lcd_power_off_rum2,                                         // 33 - WRITE_POWER_OFF
    lcd_write_htswaiting_rum1, lcd_write_empty_line,                                // 34 - WRITE_HTSWAITING  
    lcd_write_emwaiting_rum1, lcd_write_empty_line,                                 // 35 - WRITE_EMWAITING
    lcd_write_empty_line, lcd_write_reserved_rum2,                                  // 36 - WRITE_RESERVED  

    lcd_write_evs_disabled_pol1, lcd_write_evs_disabled_pol2,                       // 00 - WRITE_EVS_DISABLED:         POL
    lcd_write_card_wait_pol1, lcd_write_empty_line,                                 // 01 - WRITE_CARD_WAIT
    lcd_write_socket_available_pol1, lcd_write_socket_available_pol2,               // 02 - WRITE_SOCKET_AVAILABLE
    lcd_write_plug_check_pol1, lcd_write_plug_check_pol2,                           // 03 - WRITE_PLUG_CHECK
    lcd_write_empty_line, lcd_write_plug_out_pol2,                                  // 04 - WRITE_PLUG_OUT
    lcd_write_s2_waiting_pol1, lcd_write_empty_line,                                // 05 - WRITE_S2_WAIT
    lcd_write_charging_pol1, lcd_write_empty_line,                                  // 06 - WRITE_CHARGING
    lcd_write_suspendig_pol1, lcd_write_empty_line,                                 // 07 - WRITE_SUSPENDING
    lcd_write_rmwaiting_pol1, lcd_write_empty_line,                                 // 08 - WRITE_RMWAITING
    lcd_write_interrupting_charge_pol1, lcd_write_interrupting_charge_pol2,         // 09 - WRITE_INTERRUPTING_CHARGE
    lcd_write_update_wlist_pol1, lcd_write_update_wlist_pol2,                       // 10 - WRITE_UPDATE_WLIST
    lcd_uid_delete_confirm_pol1, lcd_write_empty_line,                              // 11 - WRITE_UID_DELETE_CONFIRM
    lcd_uid_deleted_pol1, lcd_uid_users_pol2,                                       // 12 - WRITE_UID_DELETED
    lcd_uid_added_pol1, lcd_uid_users_pol2,                                         // 13 - WRITE_UID_ADDED
    lcd_uid_error_pol1, lcd_uid_error_pol2,                                         // 14 - WRITE_UID_ERROR
    lcd_delete_wlist_confirm_pol1, lcd_write_empty_line,                            // 15 - WRITE_DELETE_WLIST_CONFIRM
    lcd_authorization_failed_pol1, lcd_authorization_failed_pol2,                   // 16 - WRITE_AUTHORIZATION_FAILED
    lcd_plug_wait_pol1, lcd_write_empty_line,                                       // 17 - WRITE_PLUG_WAIT
    lcd_wlist_deleted_pol1, lcd_write_empty_line,                                   // 18 - WRITE_WLIST_DELETED
    lcd_reset_failed_pol1, lcd_reset_failed_pol2,                                   // 19 - WRITE_RESET_FAILED
    lcd_delete_error_pol1, lcd_delete_error_pol2,                                   // 20 - WRITE_DELETE_ERROR
    lcd_wlist_full_pol1, lcd_write_empty_line,                                      // 21 - WRITE_WLIST_FULL
    lcd_expired_card_pol1, lcd_expired_card_pol2,                                   // 22 - WRITE_EXPIRED_CARD
    lcd_credit_null_pol1, lcd_credit_null_pol2,                                     // 23 - WRITE_CREDIT_NULL
    lcd_close_lid_pol1, lcd_close_lid_pol2,                                         // 24 - WRITE_CLOSE_LID
    lcd_total_energy_pol1, lcd_write_empty_line,                                    // 25 - WRITE_TOTAL_ENERGY
    lcd_auth_pending_pol1 , lcd_write_empty_line,                                   // 26 - WRITE_AUTH_PENDING
    lcd_credit_date_pol1, lcd_write_empty_line,                                     // 27 - WRITE_CREDIT_DATE
    lcd_new_credit_date_pol1, lcd_write_empty_line,                                 // 28 - WRITE_NEW_CREDIT_DATE
    lcd_only_date_pol1, lcd_write_empty_line,                                       // 29 - WRITE_ONLY_DATE
    lcd_new_date_done_pol1, lcd_write_empty_line,                                   // 30 - WRITE_NEW_DATE_DONE
    lcd_place_plug_pol1, lcd_place_plug_pol2,                                       // 31 - WRITE_PLACE_PLUG
    lcd_auth_missed_pol1, lcd_auth_missed_pol2,                                     // 32 - WRITE_AUTH_MISSED  
    lcd_power_off_pol1, lcd_power_off_pol2,                                         // 33 - WRITE_POWER_OFF
    lcd_write_htswaiting_pol1, lcd_write_empty_line,                                // 34 - WRITE_HTSWAITING  
    lcd_write_emwaiting_pol1, lcd_write_empty_line,                                 // 35 - WRITE_EMWAITING
    lcd_write_empty_line, lcd_write_reserved_pol2,                                  // 36 - WRITE_RESERVED  

    lcd_write_evs_disabled_sve1, lcd_write_evs_disabled_sve2,                       // 00 - WRITE_EVS_DISABLED:         SVE
    lcd_write_card_wait_sve1, lcd_write_empty_line,                                 // 01 - WRITE_CARD_WAIT
    lcd_write_socket_available_sve1, lcd_write_socket_available_sve2,               // 02 - WRITE_SOCKET_AVAILABLE
    lcd_write_plug_check_sve1, lcd_write_plug_check_sve2,                           // 03 - WRITE_PLUG_CHECK
    lcd_write_empty_line, lcd_write_plug_out_sve2,                                  // 04 - WRITE_PLUG_OUT
    lcd_write_s2_waiting_sve1, lcd_write_empty_line,                                // 05 - WRITE_S2_WAIT
    lcd_write_charging_sve1, lcd_write_empty_line,                                  // 06 - WRITE_CHARGING
    lcd_write_suspendig_sve1, lcd_write_empty_line,                                 // 07 - WRITE_SUSPENDING
    lcd_write_rmwaiting_sve1, lcd_write_empty_line,                                 // 08 - WRITE_RMWAITING
    lcd_write_interrupting_charge_sve1, lcd_write_interrupting_charge_sve2,         // 09 - WRITE_INTERRUPTING_CHARGE
    lcd_write_update_wlist_sve1, lcd_write_update_wlist_sve2,                       // 10 - WRITE_UPDATE_WLIST
    lcd_uid_delete_confirm_sve1, lcd_write_empty_line,                              // 11 - WRITE_UID_DELETE_CONFIRM
    lcd_uid_deleted_sve1, lcd_uid_users_sve2,                                       // 12 - WRITE_UID_DELETED
    lcd_uid_added_sve1, lcd_uid_users_sve2,                                         // 13 - WRITE_UID_ADDED
    lcd_uid_error_sve1, lcd_uid_error_sve2,                                         // 14 - WRITE_UID_ERROR
    lcd_delete_wlist_confirm_sve1, lcd_write_empty_line,                            // 15 - WRITE_DELETE_WLIST_CONFIRM
    lcd_authorization_failed_sve1, lcd_authorization_failed_sve2,                   // 16 - WRITE_AUTHORIZATION_FAILED
    lcd_plug_wait_sve1, lcd_write_empty_line,                                       // 17 - WRITE_PLUG_WAIT
    lcd_wlist_deleted_sve1, lcd_write_empty_line,                                   // 18 - WRITE_WLIST_DELETED
    lcd_reset_failed_sve1, lcd_reset_failed_sve2,                                   // 19 - WRITE_RESET_FAILED
    lcd_delete_error_sve1, lcd_delete_error_sve2,                                   // 20 - WRITE_DELETE_ERROR
    lcd_wlist_full_sve1, lcd_write_empty_line,                                      // 21 - WRITE_WLIST_FULL
    lcd_expired_card_sve1, lcd_expired_card_sve2,                                   // 22 - WRITE_EXPIRED_CARD
    lcd_credit_null_sve1, lcd_credit_null_sve2,                                     // 23 - WRITE_CREDIT_NULL
    lcd_close_lid_sve1, lcd_close_lid_sve2,                                         // 24 - WRITE_CLOSE_LID
    lcd_total_energy_sve1, lcd_write_empty_line,                                    // 25 - WRITE_TOTAL_ENERGY
    lcd_auth_pending_sve1 , lcd_write_empty_line,                                   // 26 - WRITE_AUTH_PENDING
    lcd_credit_date_sve1, lcd_write_empty_line,                                     // 27 - WRITE_CREDIT_DATE
    lcd_new_credit_date_sve1, lcd_write_empty_line,                                 // 28 - WRITE_NEW_CREDIT_DATE
    lcd_only_date_sve1, lcd_write_empty_line,                                       // 29 - WRITE_ONLY_DATE
    lcd_new_date_done_sve1, lcd_write_empty_line,                                   // 30 - WRITE_NEW_DATE_DONE
    lcd_place_plug_sve1, lcd_place_plug_sve2,                                       // 31 - WRITE_PLACE_PLUG
    lcd_auth_missed_sve1, lcd_auth_missed_sve2,                                     // 32 - WRITE_AUTH_MISSED  
    lcd_power_off_sve1, lcd_power_off_sve2,                                         // 33 - WRITE_POWER_OFF
    lcd_write_htswaiting_sve1, lcd_write_empty_line,                                // 34 - WRITE_HTSWAITING  
    lcd_write_emwaiting_sve1, lcd_write_empty_line,                                 // 35 - WRITE_EMWAITING
    lcd_write_empty_line, lcd_write_reserved_sve2                                   // 36 - WRITE_RESERVED  
    };
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local references ----------------------------------------------------------------------------------------------------------------------- //
xQueueHandle LcdMngQueue =  NULL;

static LcdMngMsg_st         LcdMngMsg;
static LcdMngMsg_st         LcdMngMsg_Old;
static LcdMngMsg_st         LcdPersMsg_Old;

static TimerHandle_t        xLcdMngTimers[LCD_NUM_TIMER];

static uint8_t              lcd_language;

static uint8_t              lcd_toggle;
static uint8_t              lcd_blink_enb;
static uint8_t              lcd_main_blink;

static uint8_t              lcd_emeter_type_old;
static uint8_t              lcd_parameter_scroll;

static uint8_t              lcd_linestring_idx;

static char                 lcd_linestring1[LCD_CHAR_NUM];
static char                 lcd_linestring2[LCD_CHAR_NUM];
static char                 lcd_linestring3[LCD_CHAR_NUM];
static char                 lcd_linestring4[LCD_CHAR_NUM];
static char                 lcd_linestring5[LCD_CHAR_NUM];
static char                 lcd_linestring6[LCD_CHAR_NUM];
static char                 lcd_linestring7[LCD_CHAR_NUM];
static char                 lcd_linestring8[LCD_CHAR_NUM];

static char                 lcd_line1[LCD_CHAR_NUM];
static char                 lcd_line2[LCD_CHAR_NUM];

static uint8_t              lcd_language_def_enable;

static uint8_t              lcd_info_code_old;

static uint8_t              lcd_external_em;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local function prototypes -------------------------------------------------------------------------------------------------------------- //
static xQueueHandle getLcdMngQueueHandle(void);
static void LcdMngTimCallBack(TimerHandle_t pxTimer);
static void lcd_linestring_update(char *dst_ptr, char *src_ptr);
static void lcd_linestring_init(void);
static void lcd_set_timer(LcdTim_en timer, uint32_t set_time);
static void lcd_bcd08_to_ascii(uint8_t *src_ptr, char *dst_ptr, uint8_t msb);
static void lcd_dec08_to_ascii(unsigned char *src_ptr, char *dst_ptr);
static uint8_t lcd_hex08_to_ascii(uint8_t *src_ptr, char *dst_ptr, uint8_t msb);
static uint8_t lcd_hex16_to_ascii(uint16_t *src_ptr, char *dst_ptr, uint8_t msb);
static uint8_t lcd_hex32_to_ascii(uint32_t *src_ptr, char *dst_ptr, uint8_t msb);
static uint8_t lcd_char_left_push(uint16_t aux_inf, char *dst_ptr);
static uint8_t lcd_char_head_push(uint16_t aux_inf, char *dst_ptr, uint8_t len, uint8_t *num);
static void lcd_char_foot_push(uint16_t aux_inf, char *dst_ptr, uint8_t *num);
static void lcd_char_right_push(uint16_t aux_inf, char *dst_ptr);
static void lcd_line_update(char *dst_ptr, const char *src_ptr, alignment_en alignment, uint16_t aux_inf);
static uint8_t lcd_error_code_get(uint8_t *src_ptr);
static uint8_t lcd_charging_info_get(int16_t *dst_ptr, uint8_t *emeter_parameter);
static void lcd_charging_string_get(char *dst_ptr, uint8_t *emeter_parameter);
static void LcdManager_init(void);
static void LcdManager(LcdMngMsg_st *pMsg);
static void updateClearAllLineString (void);

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  getLcdMngQueueHandle
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static xQueueHandle getLcdMngQueueHandle(void)
{
return(LcdMngQueue);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  LcdMngTimCallBack
//
//  DESCRIPTION:    callback to manager timers
//
//  INPUT:          TimerHandle_t: the elapsed timer
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void LcdMngTimCallBack(TimerHandle_t pxTimer)
{
uint32_t        timer_id;
evs_state_en    state;

timer_id = (uint32_t)pvTimerGetTimerID(pxTimer);                                    // find the led  which the timer is referred

if (timer_id == (uint32_t)(LCD_MAIN_BLINK_TIM))                                     // check if timer exist
    {
    lcd_main_blink = 0;
//    lcd_set_timer(LCD_MAIN_BLINK_TIM, portMAX_DELAY);
    send_to_lcd(REFRESH_TIME_EXPIRED);
    }

if (timer_id == (uint32_t)(LCD_DEF_LANGUAGE_TIM))                                   // check if timer exist
    {
    lcd_language_def_enable = 0;
//    lcd_set_timer(LCD_DEF_LANGUAGE_TIM, portMAX_DELAY);
    }

if (timer_id == (uint32_t)(LCD_LANGUAGE_TIM))                                       // check if timer exist
    {
    state = evs_state_get();

    if ((state == EVSTATE_DISABLED) || (state == EVSTATE_AUTH_WAIT) || (state == EVSTATE_SOCKET_AVAILABLE))
        send_to_lcd(LANGUAGE_TIME_EXPIRED);
    else
        lcd_set_timer(LCD_LANGUAGE_TIM, LCD_LANGUAGE_TIME);                         // reload timer
    }

if (timer_id == (uint32_t)(LCD_REFRESH_TIM))                                        // check if timer exist
    send_to_lcd(REFRESH_TIME_EXPIRED);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_linestring_update
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void lcd_linestring_update(char *dst_ptr, char *src_ptr)
{
uint8_t i;

for (i=0; i<LCD_CHAR_NUM; i++)
    *(dst_ptr + i) = *(src_ptr + i);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_linestring_init
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void lcd_linestring_init(void)
{
lcd_linestring_update(lcd_linestring1, (char*)(lcd_line_null));
lcd_linestring_update(lcd_linestring2, (char*)(lcd_line_null));
lcd_linestring_update(lcd_linestring3, (char*)(lcd_line_null));
lcd_linestring_update(lcd_linestring4, (char*)(lcd_line_null));
lcd_linestring_update(lcd_linestring5, (char*)(lcd_line_null));
lcd_linestring_update(lcd_linestring6, (char*)(lcd_line_null));
lcd_linestring_update(lcd_linestring7, (char*)(lcd_line_null));
lcd_linestring_update(lcd_linestring8, (char*)(lcd_line_null));
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_set_timer
//
//  DESCRIPTION:    imposta il timer selezionato
//
//  INPUT:          timer da aggiornare: timer; tempo da impostare: set_time
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void lcd_set_timer(LcdTim_en timer, uint32_t set_time)
{
while ((xTimerChangePeriod (xLcdMngTimers[timer], set_time, LCD_GARD_TIME) != pdPASS));  // set timer
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  send_to_lcd
//
//  DESCRIPTION:    impacchetta l'evento da inviare a LcdMngTask
//
//  INPUT:          valore di LcdMngEvent
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void send_to_lcd(uint8_t lcd_event)
{
LcdMngMsg_st    msgLcdSend;

msgLcdSend.LcdMngEvent = (LcdMngEvent_en)(lcd_event);
configASSERT(xQueueSendToBack(getLcdMngQueueHandle(), (void *)&msgLcdSend, portMAX_DELAY) == pdPASS);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_bcd08_to_ascii
//
//  DESCRIPTION:    converte in ascii il byte bcd di ingresso [max 2 cifre]
//
//  INPUT:          puntatore sorgente: src_ptr; puntatore destinatario: dst_ptr; forzatura di conversione del msb
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void lcd_bcd08_to_ascii(uint8_t *src_ptr, char *dst_ptr, uint8_t msb)
{
uint8_t data08u;

data08u = (*src_ptr & 0xF0) >> 4;

if ((msb > 0) || (data08u > 0))
    *(dst_ptr + 0) = data08u + '0';
else
    *(dst_ptr + 0) = ' ';

*(dst_ptr + 1) = (*src_ptr & 0x0F) + '0';
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_dec08_to_ascii
//
//  DESCRIPTION:    converte in ascii il byte decimale di ingresso [max 2 cifre]
//
//  INPUT:          puntatore sorgente: src_ptr; puntatore destinatario: dst_ptr
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void lcd_dec08_to_ascii(unsigned char *src_ptr, char *dst_ptr)
{
*(dst_ptr + 0) = (*src_ptr / 10) + '0';
*(dst_ptr + 1) = (*src_ptr % 10) + '0';
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_hex08_to_ascii
//
//  DESCRIPTION:    converte in decimale ascii il byte esadecimale di ingresso [max 3 cifre]
//
//  INPUT:          puntatore sorgente: src_ptr; puntatore destinatario: dst_ptr; forzatura msb: msb
//
//  OUTPUT:         ret: numero cifre != 0
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t lcd_hex08_to_ascii(uint8_t *src_ptr, char *dst_ptr, uint8_t msb)
{
uint8_t ret;

if (((*src_ptr / 100) > 0) || (msb == 3))
    {
    *(dst_ptr + 0) = (*src_ptr / 100);
    *(dst_ptr + 2) = (*src_ptr % 10);
    *(dst_ptr + 1) = ((*src_ptr - *(dst_ptr + 2) - (*(dst_ptr + 0) * 100)) / 10);
    
    *(dst_ptr + 0) += '0';
    *(dst_ptr + 1) += '0';
    *(dst_ptr + 2) += '0';
    
    ret = 3;
    }
else if (((*src_ptr / 10) > 0) || (msb == 2))
    {
    *(dst_ptr + 0) = (*src_ptr / 10) + '0';
    *(dst_ptr + 1) = (*src_ptr % 10) + '0';
    
    ret = 2;
    }
else
    {
    *(dst_ptr + 0) = *src_ptr + '0';
    ret = 1;
    }

return ret;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_hex16_to_ascii
//
//  DESCRIPTION:    converte in decimale ascii la word esadecimale di ingresso [max 5 cifre]
//
//  INPUT:          puntatore sorgente: src_ptr; puntatore destinatario: dst_ptr; forzatura di conversione del msb
//
//  OUTPUT:         ret: numero cifre != 0
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t lcd_hex16_to_ascii(uint16_t *src_ptr, char *dst_ptr, uint8_t msb)
{
uint8_t     i, dsb, ret, data08u;
uint16_t    data_div, data16u;

for (i=0; i<4; i++)
    *(dst_ptr + i) = ' ';

*(dst_ptr + 4) = '0';

data16u = *src_ptr;
data_div = 10000;

dsb = 0;
ret = 0;

for (i=0; i<5; i++)
    {
    data08u = (uint8_t)(data16u / data_div);
    
    if (msb > 0)
        {
        if (i == (5 - msb))
            dsb = 1;
        }

    if (data08u > 0)
        {
        dsb = 1;
        *(dst_ptr + i) = (data08u + '0');
        ret ++;
        }
    else if (dsb == 1)
        {
        *(dst_ptr + i) = '0';
        ret ++;
        }
    
    data16u = (*src_ptr % data_div);
    data_div = (data_div / 10);
    }

if ((*src_ptr == 0) && (msb == 0))
    ret = 1;

return ret;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_hex32_to_ascii
//
//  DESCRIPTION:    converte in decimale ascii il long esadecimale di ingresso [max 10 cifre]
//
//  INPUT:          puntatore sorgente: src_ptr; puntatore destinatario: dst_ptr; forzatura di conversione del msb
//
//  OUTPUT:         ret: numero cifre
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t lcd_hex32_to_ascii(uint32_t *src_ptr, char *dst_ptr, uint8_t msb)
{
uint8_t     i, dsb, ret, data08u;
uint32_t    data_div, data32u;

for (i=0; i<9; i++)
    *(dst_ptr + i) = ' ';

*(dst_ptr + 9) = '0';

data32u = *src_ptr;
data_div = 1000000000L;

dsb = 0;
ret = 0;

for (i=0; i<10; i++)
    {
    data08u = (uint8_t)(data32u / data_div);
    
    if (msb > 0)
        {
        if (i == (10 - msb))
            dsb = 1;
        }

    if (data08u > 0)
        {
        dsb = 1;
        *(dst_ptr + i) = (data08u + '0');
        ret ++;
        }
    else if (dsb == 1)
        {
        *(dst_ptr + i) = '0';
        ret ++;
        }
    
    data32u = (*src_ptr % data_div);
    data_div = (data_div / 10);
    }

if ((*src_ptr == 0) && (msb == 0))
    ret = 1;

return ret;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_char_left_push
//
//  DESCRIPTION:    inserisce l'informazione relativa alla modalità [evs_mode] nei primi caratteri della riga
//
//  INPUT:          codice info ausiliaria: aux_inf; puntatore destinatario: dst_ptr
//
//  OUTPUT:         numero caratteri aggiunti
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t lcd_char_left_push(uint16_t aux_inf, char *dst_ptr)
{
uint8_t     i, sign, data08u, lcd_rs485_add, lcd_energy_info, error_array[EVS_ERROR_ARRAY_SIZE];
int16_t    data16u;
char        char_array[5];
evs_mode_en lcd_mode;

if (aux_inf & LCD_EVS_MODE_ORINF)                                   // l'info si allinea al margine sinistro
    {
    eeprom_param_get(EVS_MODE_EADD, (uint8_t*)(&lcd_mode), 1);

    lcd_rs485_add  = getIdNumberForLcd();  // indirizzi da 0 a 15 -> visualizzazione da 1 a 16

    if (lcd_mode == EVS_PERS_MODE)
        {
        *dst_ptr = 'P';
        *(dst_ptr + 1) = 'M';
        }
    else
        {
        lcd_hex08_to_ascii(&lcd_rs485_add, dst_ptr, 2);
        *(dst_ptr + 2) = ' ';
        }

    return 3;
    }

if (aux_inf & LCD_CHARGING_ORINF)
    {
    evs_error_get(error_array, 1, 1, 0);

    if (error_array[1] & EMETER_INT_ANOM1)
        {
        for (i = 0; i < 10; i++)
            *(dst_ptr + i) = *(lcd_write_emtr_error + i);

        lcd_linestring_idx = 0xFF;
        return 10;
        }
    else
        {
        if (lcd_charging_info_get(&data16u, &lcd_energy_info) == 1)
            {
            if ((error_array[2] & (EMETER_EXT_CRL2 | SINAPSI_CHN2_CRL2))
            && ((lcd_energy_info == EM_EXT_ACTIVE_POWER) || (lcd_energy_info == EM_EXT_L1_ACTIVE_POWER) || (lcd_energy_info == EM_EXT_L2_ACTIVE_POWER) || (lcd_energy_info == EM_EXT_L3_ACTIVE_POWER)))
                {
                if (error_array[2] & EMETER_EXT_CRL2)
                    {
                    for (i = 0; i < 10; i++)
                        *(dst_ptr + i) = *(lcd_write_emex_error + i);
                    }
                else    // if (error_array[2] & SINAPSI_CHN2_CRL2)
                    {
                    for (i = 0; i < 10; i++)
                        *(dst_ptr + i) = *(lcd_write_chn2_error + i);
                    }

                setErrorModbus(ERROR2_EMEX, 2);
        
                lcd_linestring_idx = 0xFF;
                return 10;
                }
            else if (((lcd_energy_info == EM_EXT_ACTIVE_POWER) || (lcd_energy_info == EM_EXT_L1_ACTIVE_POWER) || (lcd_energy_info == EM_EXT_L2_ACTIVE_POWER) || (lcd_energy_info == EM_EXT_L3_ACTIVE_POWER)) && (lcd_external_em == 0))
                {
                for (i = 0; i < 10; i++)
                    *(dst_ptr + i) = *(lcd_write_emex_null + i);
        
                lcd_linestring_idx = 0xFF;
                return 10;
                }
            else
                {
                lcd_charging_string_get(dst_ptr, &lcd_energy_info);
                
                sign = 1;

                if (data16u < 0)
                    data16u = - data16u;
                else
                    sign = 0;

                lcd_hex16_to_ascii((uint16_t*)(&data16u), char_array, 0);
                
                for (i=0; i<3; i++)
                    *(dst_ptr + i + 4) = *(char_array + i + 1);
                
                if (*(dst_ptr + 6) == ' ')
                    *(dst_ptr + 6) = '0';
    
                *(dst_ptr + 7) = '.';
                *(dst_ptr + 8) = *(char_array + 4);
                
                if (sign == 1)
                    *(dst_ptr + 4) = '-';
                }
            }

        return 12;
        }
    }

if (aux_inf == LCD_EVS_SEC_WAIT_EXINF)
    {
    evs_sec_get(&data08u);

//    if ((evs_state_get() == EVSTATE_AUTH_PENDING) || (evs_state_get() == EVSTATE_PAUT_LID))
//        data08u *= 3;
//    
    for (i=0; i<(data08u); i++)
        *(dst_ptr + i) = '.';

    while (i < LCD_CHAR_NUM)
        {
        *(dst_ptr + i) = ' ';
        i ++;
        }

    return data08u;
    }

return 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_char_head_push
//
//  DESCRIPTION:    inserisce in testa l'informazione ausiliaria soggetta ad allineamento
//
//  INPUT:          codice info ausiliaria: aux_inf; puntatore destinatario: dst_ptr; lunghezza del testo: len; puntatore numero caratteri da aggiungere: num
//
//  OUTPUT:         numero caratteri aggiunti
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t lcd_char_head_push(uint16_t aux_inf, char *dst_ptr, uint8_t len, uint8_t *num)
{
uint8_t                 i, j, ret, data08u, data_array[4];
uint16_t                data16u;
int32_t                 data32i;
struct DataAndTime_t*   pLocDateTime;

ret = 0;

if (aux_inf == LCD_DATE_TIME_EXINF)
    {
    // Aggiorno data e ora
    pLocDateTime = getCurrentLocalTime((uint32_t*)&data32i);

    //DateTimeGet(&locDateTime);

    lcd_dec08_to_ascii(&pLocDateTime->Day, (dst_ptr + 0));
    *(dst_ptr + 2) = '/';
    lcd_dec08_to_ascii(&pLocDateTime->Month, (dst_ptr + 3));
    *(dst_ptr + 5) = '/';
    data08u = (uint8_t)(pLocDateTime->Year % 2000);
    lcd_dec08_to_ascii((unsigned char*)(&data08u), (dst_ptr + 6));
    *(dst_ptr + 8) = ' ';
    lcd_dec08_to_ascii(&pLocDateTime->Hour, (dst_ptr + 9));
    *(dst_ptr + 11) = ':';
    lcd_dec08_to_ascii(&pLocDateTime->Minute, (dst_ptr + 12));

    *num = 14;
    ret = 14;
    }
else if (aux_inf == LCD_USER_UID_NUM_EXINF)
    {
    eeprom_param_get(PERS_UIDNUM_EADD, &data08u, 1);

    *num = lcd_hex08_to_ascii(&data08u, dst_ptr, 0);                // dato inserito in testa alla linea
    
    if (*num == 1)
        {
        *(dst_ptr + 1) = *(dst_ptr + 0);
        *(dst_ptr + 0) = ' ';
        }

    *(dst_ptr + 2) = ' ';

    *num = 3;
    ret = 3;
    }
else if (aux_inf == LCD_UPDATE_WLIST_TIME_EXINF)
    {
    pers_sec_get(&data08u);

    *(dst_ptr + len) = ' ';
    *num = lcd_hex08_to_ascii(&data08u, &dst_ptr[(len + 1)], 0);    // dato inserito in coda alla linea

    if (*num == 1)
        {
        *(dst_ptr + len + 2) = *(dst_ptr + len + 1);
        *(dst_ptr + len + 1) = ' ';
        }

    *num = 3;
    }
else if (aux_inf == LCD_DELETE_CONFIRM_TIME_EXINF)
    {
    pers_sec_get(&data08u);

    for (i=0; i<data08u; i++)
        *(dst_ptr + i) = '.';

    while (i < UID_DELETE_SEC)
        {
        *(dst_ptr + i) = ' ';
        i ++;
        }

    *num = UID_DELETE_SEC;
    }
else if (aux_inf == LCD_TOTAL_ENERGY_EXINF)
    {
    energy_param_get(EM_TOT_ACTIVE_ENERGY, &data32i, 1);
    
    *num = lcd_hex32_to_ascii((uint32_t*)(&data32i), dst_ptr, 0);
        
    j = 0;

    while (*(dst_ptr + j) == ' ')
       j ++;
    
    for (i=0; i<*num; i++)
       *(dst_ptr + i) = *(dst_ptr + i + j);

    if (*num == 1)
        {
        *(dst_ptr + 1) = *(dst_ptr + 0);
        *(dst_ptr + 0) = '0';
        *num = 2;
        }

    *(dst_ptr + *num) = *(dst_ptr + *num - 1);
    *(dst_ptr + *num - 1) = '.';
    *(dst_ptr + *num + 1) = 'K';
    *(dst_ptr + *num + 2) = 'W';
    *(dst_ptr + *num + 3) = 'h';

    *num += 4;
    }
else if (aux_inf == LCD_ENTER_PASSWORD_EXINF)
    {
    hidden_enter_array_get(data_array);
    data08u = hidden_array_index_get();
    
    for (i=0; i<3; i++)
        *(dst_ptr + i) = data_array[i];

    if (lcd_toggle == 0)
        *(dst_ptr + data08u) = ' ';

    if (lcd_blink_enb == 1)
        lcd_toggle ^= 1;

    *num = 3;
    }
else if ((aux_inf == LCD_DOMESTIC_POWER_EXINF) || (aux_inf == LCD_MIN_CURRENT_EXINF) || (aux_inf == LCD_POWER_ERROR_EXINF) || (aux_inf == LCD_CHANGE_PASSWORD_EXINF))
    {
    hidden_enter_array_get(data_array);
    data16u = ((uint16_t)(data_array[0]) * 100) + (data_array[1] * 10) + data_array[2];
    data08u = hidden_array_index_get();
    
    if ((data08u > 0) && (aux_inf != LCD_CHANGE_PASSWORD_EXINF))
        j = 2;
    else
        j = 3;
    
    *num = lcd_hex16_to_ascii(&data16u, dst_ptr, j);
    
    j = 0;

    while (*(dst_ptr + j) == ' ')
       j ++;
    
    for (i=0; i<*num; i++)
       *(dst_ptr + i) = *(dst_ptr + i + j);

    if (lcd_toggle == 0)
        {
        if (aux_inf == LCD_POWER_ERROR_EXINF)
            {
            *(dst_ptr + 0) = ' ';
            *(dst_ptr + 1) = ' ';
            }
        else
            {
            if (*num < 3)
                data08u --;

            *(dst_ptr + data08u) = ' ';
            }
        }

    *(dst_ptr + *num) = *(dst_ptr + *num - 1);
    
    if ((lcd_toggle == 0) && (aux_inf == LCD_POWER_ERROR_EXINF))
        *(dst_ptr + *num - 1) = ' ';
    else if (aux_inf != LCD_CHANGE_PASSWORD_EXINF)
        *(dst_ptr + *num - 1) = '.';

    if (lcd_blink_enb == 1)
        lcd_toggle ^= 1;

    if ((aux_inf == LCD_DOMESTIC_POWER_EXINF) || (aux_inf == LCD_POWER_ERROR_EXINF))
        {
        *(dst_ptr + *num + 1) = 'K';
        *(dst_ptr + *num + 2) = 'W';
        *num += 3;
        }
    else if (aux_inf == LCD_MIN_CURRENT_EXINF)
        {
        *(dst_ptr + *num + 1) = 'A';
        *(dst_ptr + *num + 2) = ' ';
        *num += 3;
        }
    }
else if ((aux_inf == LCD_POWER_MULTIP_EXINF) || (aux_inf == LCD_POWER_DMAX_EXINF))
    {
    hidden_enter_array_get(data_array);
    data08u = data_array[2];
    
    if (aux_inf == LCD_POWER_MULTIP_EXINF)
        data08u += 1;

    for (i=0; i<5; i++)
        *(dst_ptr + i) = ' ';

    if (lcd_toggle == 0)
        {
        if (data08u > 0)
            *(dst_ptr + 3) = '%';
        }
    else
        {
        if (data08u == 0)
            {
            for (i=0; i<3; i++)
                *(dst_ptr + i)= lcd_write_off[i];
            }
        else
            {
            *num = lcd_hex08_to_ascii(&data08u, (char*)(data_array), 0);
            
            for (i=0; i<*num; i++)
                *(dst_ptr + 2 - i) = data_array[(*num - 1 - i)];

            *(dst_ptr + 3) = '%';
            }
        }

    if (lcd_blink_enb == 1)
        lcd_toggle ^= 1;

    *(dst_ptr + 4) = ' ';
    *num = 5;
    }
else if ((aux_inf == LCD_EMETER_CRL2_EXINF) || (aux_inf == LCD_TIME_RANGE_EXINF) || (aux_inf == LCD_UNBAL_ENB_EXINF)
     || (aux_inf == LCD_PMNG_ENB_EXINF) || (aux_inf == LCD_TIMED_ENB_EXINF))
    {
    hidden_enter_array_get(data_array);

    if (lcd_toggle == 0)
        {
        for (i=0; i<3; i++)
            *(dst_ptr + i) = ' ';
        }
    else
        {
        if (((aux_inf == LCD_EMETER_CRL2_EXINF) && (data_array[2] & (EMETER_EXT_CRL2 | SINAPSI_CHN2_CRL2)))
         || ((aux_inf == LCD_TIME_RANGE_EXINF) && (data_array[2] == 1))
         || ((aux_inf == LCD_UNBAL_ENB_EXINF) && (data_array[2] == 1))
         || ((aux_inf == LCD_PMNG_ENB_EXINF) && (data_array[2] != 0))
         || ((aux_inf == LCD_TIMED_ENB_EXINF) && (data_array[2] != 0)))
            {
            for (i=0; i<sizeof(lcd_write_on); i++)
                *(dst_ptr + i)= lcd_write_on[i];
            }
        else
            {
            for (i=0; i<sizeof(lcd_write_off); i++)
                *(dst_ptr + i)= lcd_write_off[i];
            }
        }

    if (lcd_blink_enb == 1)
        lcd_toggle ^= 1;

    *num = 3;
    }
else if (aux_inf == LCD_TIMED_TIME_EXINF)
    {
    hidden_enter_array_get(data_array);
    data16u = ((uint16_t)(data_array[2]) * 30);

    if (lcd_toggle == 0)
        {
        for (i=0; i<3; i++)
            *(dst_ptr + i) = ' ';
        }
    else
        {
        if (data16u == 0)
            {
            for (i=0; i<sizeof(lcd_write_no_limit); i++)
                *(dst_ptr + i) = lcd_write_no_limit[i];

            *num = sizeof(lcd_write_no_limit);
            }
        else
            {
            lcd_hex16_to_ascii(&data16u, dst_ptr, 0);

            for (i=0; i<3; i++)
               *(dst_ptr + i) = *(dst_ptr + i + 2);

            *(dst_ptr + 3) = ' ';

            for (i=4; i<(4 + sizeof(lcd_write_minute)); i++)
                *(dst_ptr + i) = lcd_write_minute[(i - 4)];

            *num = (3 + sizeof(lcd_write_minute));
            }
        }

    if (lcd_blink_enb == 1)
        lcd_toggle ^= 1;
    }

else if (aux_inf == LCD_ENRG_LIMIT_EXINF)
    {
    hidden_enter_array_get(data_array);
    data08u = data_array[2];

    if (lcd_toggle == 0)
        {
        for (i=0; i<4; i++)
            *(dst_ptr + i) = ' ';
        }
    else
        {
        if (data08u == 0)
            {
            for (i=0; i<sizeof(lcd_write_no_limit); i++)
                *(dst_ptr + i) = lcd_write_no_limit[i];

            *num = sizeof(lcd_write_no_limit);
            }
        else
            {
            *num = lcd_hex08_to_ascii(&data08u, (char*)(data_array), 0);
            
            for (i=0; i<3; i++)
                *(dst_ptr + 2 - i) = ' ';

            for (i=0; i<*num; i++)
                *(dst_ptr + 2 - i) = data_array[(*num - 1 - i)];

            *(dst_ptr + 3) = ' ';

            for (i=4; i<(4 + sizeof(lcd_write_KWh)); i++)
                *(dst_ptr + i) = lcd_write_KWh[(i - 4)];
            
//            *(dst_ptr + (4 + sizeof(lcd_write_KWh) - 1)) = ' ';

            *num = (3 + sizeof(lcd_write_KWh));
            }
        }

    if (lcd_blink_enb == 1)
        lcd_toggle ^= 1;
    }

else if (aux_inf == LCD_PMNG_TYPE_EXINF)
    {
    hidden_enter_array_get(data_array);
    data08u = data_array[2];

    if (lcd_toggle == 0)
        {
        for (i=0; i<4; i++)
            *(dst_ptr + i) = ' ';
        }
    else
        {
        switch (data08u)
            {
            case PMNG_FULL:
                {
                for (i=0; i<sizeof(lcd_write_pmng_full); i++)
                    *(dst_ptr + i) = lcd_write_pmng_full[i];
    
                *num = sizeof(lcd_write_pmng_full);
                }
                break;

            case PMNG_ECO_PLUS:
                {
                for (i=0; i<sizeof(lcd_write_pmng_eco_plus); i++)
                    *(dst_ptr + i) = lcd_write_pmng_eco_plus[i];
    
                *num = sizeof(lcd_write_pmng_eco_plus);
                }
                break;

            case PMNG_ECO_SMART:
                {
                for (i=0; i<sizeof(lcd_write_pmng_eco_smart); i++)
                    *(dst_ptr + i) = lcd_write_pmng_eco_smart[i];
    
                *num = sizeof(lcd_write_pmng_eco_smart);
                }
                break;

            default:
                {
                for (i=0; i<sizeof(lcd_write_pmng_null); i++)
                    *(dst_ptr + i) = lcd_write_pmng_null[i];
    
                *num = sizeof(lcd_write_pmng_null);
                }
                break;
            }
        }

    if (lcd_blink_enb == 1)
        lcd_toggle ^= 1;
    }

return ret;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_char_foot_push
//
//  DESCRIPTION:    inserisce in coda l'informazione ausiliaria soggetta ad allineamento
//
//  INPUT:          codice info ausiliaria: aux_inf; puntatore destinatario: dst_ptr; puntatore numero caratteri da aggiungere: num
//
//  OUTPUT:         numero caratteri aggiunti
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void lcd_char_foot_push(uint16_t aux_inf, char *dst_ptr, uint8_t *num)
{
uint8_t     data_array[4];
uint16_t    data16u;

if (aux_inf == LCD_ONLY_DATE_EXINF)
    {
    *(dst_ptr + 0) = ' ';

    user_card_date_get(data_array);
    
    lcd_bcd08_to_ascii(&data_array[1], (dst_ptr + 1), 2);
    *(dst_ptr + 3) = '/';
    lcd_bcd08_to_ascii(&data_array[2], (dst_ptr + 4), 2);
    *(dst_ptr + 6) = '/';
    lcd_bcd08_to_ascii(&data_array[3], (dst_ptr + 7), 2);
    
    *num += 9;
    }
else if (aux_inf == LCD_CREDIT_DATE_EXINF)
    {
    user_card_credit_get(&data16u);
    lcd_hex16_to_ascii(&data16u, dst_ptr, 3);
    
    *(dst_ptr + 0) = *(dst_ptr + 2);
    *(dst_ptr + 1) = *(dst_ptr + 3);
    *(dst_ptr + 2) = *(dst_ptr + 4);
    *(dst_ptr + 3) = ' ';

    *num += 4;
    }
else if (aux_inf == LCD_ONLY_CREDIT_EXINF)
    {
    user_card_credit_get(&data16u);
    lcd_hex16_to_ascii(&data16u, dst_ptr, 3);
    
    *(dst_ptr + 0) = *(dst_ptr + 2);
    *(dst_ptr + 1) = *(dst_ptr + 3);
    *(dst_ptr + 2) = *(dst_ptr + 4);

    *num += 3;
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_char_right_push
//
//  DESCRIPTION:    inserisce l'informazione ausiliaria in coda alla linea di testo corrente
//
//  INPUT:          codice informazione ausiliaria: aux_inf; puntatore destinatario: dst_ptr;
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void lcd_char_right_push(uint16_t aux_inf, char *dst_ptr)
{
uint8_t     data_array[4];
char        time_udm, char_array[2];

if (aux_inf == LCD_CREDIT_DATE_EXINF)
    {
    user_card_date_get(data_array);
    
    lcd_bcd08_to_ascii(&data_array[1], (dst_ptr + 12), 2);
    *(dst_ptr + 14) = '/';
    lcd_bcd08_to_ascii(&data_array[2], (dst_ptr + 15), 2);
    *(dst_ptr + 17) = '/';
    lcd_bcd08_to_ascii(&data_array[3], (dst_ptr + 18), 2);
    }
else
    {
    if (aux_inf & LCD_PWM_ORINF)
        {
        pwm_currents_get(data_array);
    
        lcd_hex08_to_ascii(&data_array[0], char_array, 2);
    
        *(dst_ptr + 14) = *(char_array + 0);
        *(dst_ptr + 15) = *(char_array + 1);
        *(dst_ptr + 16) = '/';
    
        lcd_hex08_to_ascii(&data_array[1], char_array, 2);
    
        *(dst_ptr + 17) = *(char_array + 0);
        *(dst_ptr + 18) = *(char_array + 1);
    
        if (evs_charging_mode_get() == M3S_CHARGING_MODE)
            *(dst_ptr + 19) = 'S';
        else
            *(dst_ptr + 19) = 'T';
        }
    
    if (aux_inf & LCD_BUSY_OUTLET_TIME_ORINF)
        {
        time_udm = evstime_time_get(data_array);
    
        dst_ptr += (LCD_CHAR_NUM - 6);
    
        lcd_hex08_to_ascii(&data_array[0], dst_ptr, 2);
    
        dst_ptr += 2;
    
        if ((time_udm != 's') && (lcd_toggle == 0))
            *dst_ptr = ' ';
        else
            *dst_ptr = ':';
    
        dst_ptr ++;
    
        lcd_hex08_to_ascii(&data_array[1], dst_ptr, 2);
    
        lcd_toggle ^= 1;
    
        dst_ptr += 2;
    
        *dst_ptr = time_udm;
        }
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_line_update
//
//  DESCRIPTION:    esegue l'allineamento dsiderato del testo e gestisce le info ausiliarie
//
//  INPUT:            -
//
//  OUTPUT:            none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void lcd_line_update(char *dst_ptr, const char *src_ptr, alignment_en alignment, uint16_t aux_inf)
{
uint8_t i, len, num, start;
char    line[LCD_CHAR_NUM];

for (i = 0; i < LCD_CHAR_NUM; i++)
    *(dst_ptr + i) = ' ';

len = 0;

while (*(src_ptr + len) != 0)
    len ++;

num = 0;

start = lcd_char_head_push(aux_inf, line, len, &num);

for (i = 0; i < len; i++)
    *(line + start + i) = *(src_ptr + i);

lcd_char_foot_push(aux_inf, (line + start + len), &num);

len += num;

num = lcd_char_left_push(aux_inf, dst_ptr);

if (alignment == LEFT_ALIGNMENT)
    start = num;
else if (alignment == CENTER_ALIGNMENT)
    start = ((LCD_CHAR_NUM - len + 1) >> 1);
else if (alignment == RIGHT_ALIGNMENT)
    start = LCD_CHAR_NUM - len;

if ((num > 0) && (alignment == CENTER_ALIGNMENT) && (start < 3 ))
    start = 3;

for (i = 0; i < len; i++)
    *(dst_ptr + start + i) = *(line + i);

lcd_char_right_push(aux_inf, dst_ptr);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_error_code_get
//
//  DESCRIPTION:    carica il codice lcd degli errori presenti per visualizzarli a display [visualizza il primo che incontra nella scansione]
//
//  INPUT:          puntatore sorgente: src_ptr
//
//  OUTPUT:         codic errore: ret
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t lcd_error_code_get(uint8_t *src_ptr)
{
uint8_t ret = 23;   // lcd_write_nu27_error
uint8_t *error_ptr;

if (*src_ptr & EVS_ERROR_LCD_MASK0)
    ret = 0;
else if (*(src_ptr + 1) & EVS_ERROR_LCD_MASK1)
    ret = 8;
else if (*(src_ptr + 2) & EVS_ERROR_LCD_MASK2)
    ret = 16;

if (ret != 23)
    {
    error_ptr = (src_ptr + (ret / 8));

    while ((*error_ptr & 0x01) == 0)
        {
        *error_ptr >>= 1;
        ret ++;
        }
    }

return ret;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_charging_info_get
//
//  DESCRIPTION:    carica il codice del parametro da visualizzare a display durante la ricarica [scroll]
//
//  INPUT:          puntatore destinatario: dst_ptr; puntatore parametro emeter da aggiornare: emeter_parameter
//
//  OUTPUT:         0 = nessun emeter presenter
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint8_t lcd_charging_info_get(int16_t *dst_ptr, uint8_t *emeter_parameter)
{
uint8_t pmng_enable, emeter_type, array_num, parameter_scroll;
uint8_t *array_ptr;

*dst_ptr = 0;

eeprom_param_get(EMETER_INT_EADD, &emeter_type, 1);

eeprom_param_get(HIDDEN_MENU_ENB_EADD, &pmng_enable, 1);
pmng_enable &= HIDDEN_MENU_PMNG_ENB;

if (emeter_type != lcd_emeter_type_old)
    lcd_parameter_scroll = 0;

lcd_emeter_type_old = emeter_type;

parameter_scroll = (lcd_parameter_scroll / 3);                              // refresh del parametro ogni 3s
lcd_linestring_idx = parameter_scroll;

switch (emeter_type)
    {
    case EMETER_TAMP_3:
    case EMETER_TAMP:
        {
        array_ptr = (uint8_t*)(mono_ph_lcd_energy_array);
        array_num = 1;
        parameter_scroll = 1;
        }
        break;

    case EMETER_MONO_PH_GAVAZZI:
    case EMETER_MONO_PH_ALGO2:
    case EMETER_MONO_PH_PA775:
        {
        if (pmng_enable == 0)
            {
            array_ptr = (uint8_t*)(mono_ph_lcd_energy_array);
            array_num = (sizeof(mono_ph_lcd_energy_array) * 3);             // refresh del parametro ogni 3s
            }
        else
            {
            array_ptr = (uint8_t*)(mono_pmng_lcd_energy_array);
            array_num = (sizeof(mono_pmng_lcd_energy_array) * 3);           // refresh del parametro ogni 3s
            }
        }
        break;

    case EMETER_THREE_PH_GAVAZZI:
    case EMETER_THREE_PH_ALGO2:
    case EMETER_THREE_PH_PA775:
        {
        if (pmng_enable == 0)
            {
            array_ptr = (uint8_t*)(three_ph_lcd_energy_array);
            array_num = (sizeof(three_ph_lcd_energy_array) * 3);            // refresh del parametro ogni 3s
            }
        else
            {
            if (pmng_sinapsi_mode_get() == 1)
                {
                array_ptr = (uint8_t*)(three_sinapsi_lcd_energy_array);
                array_num = (sizeof(three_sinapsi_lcd_energy_array) * 3);   // refresh del parametro ogni 3s
                }
            else
                {
                array_ptr = (uint8_t*)(three_pmng_lcd_energy_array);
                array_num = (sizeof(three_pmng_lcd_energy_array) * 3);      // refresh del parametro ogni 3s
                }
            }
        }
        break;

//    case EMETER_TYPE_NULL:
    default:
        break;
    }

if (emeter_type != EMETER_TYPE_NULL)
    {
    *emeter_parameter = *(array_ptr + parameter_scroll);
    energy_param_get(*(emReadReg_e*)(emeter_parameter), (int32_t*)(dst_ptr), 1);
    
    if (lcd_parameter_scroll < (array_num - 1))
        lcd_parameter_scroll ++;
    else
        lcd_parameter_scroll = 0;
    
    return 1;
    }

return 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_charging_string_get
//
//  DESCRIPTION:    carica il codice del parametro da visualizzare a display durante la ricarica [scroll]
//
//  INPUT:          puntatore destinatario: dst_ptr; puntatore parametro emeter da aggiornare: emeter_parameter
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void lcd_charging_string_get(char *dst_ptr, uint8_t *emeter_parameter)
{
uint8_t i, enrg_limit;
uint8_t *string_ptr;

eeprom_param_get(TCHARGE_MODE_EADD, &enrg_limit, 1);

if (enrg_limit == 1)
    eeprom_param_get(ENRG_LIMIT_EADD, &enrg_limit, 1);

switch (*emeter_parameter)
    {
    case EM_SES_ACTIVE_ENERGY:
        {
        if ((user_card_auth_get() & ENERGY_CHARGE_AUTH) || (enrg_limit != 0))
            string_ptr = (uint8_t*)(lcd_charge_auth_energy);
        else
            string_ptr = (uint8_t*)(lcd_active_sess_energy);
        }
        break;

    case EM_CURRENT_L:
    case EM_CURRENT_L1:
        string_ptr = (uint8_t*)(lcd_current_l1);
        break;

    case EM_CURRENT_L2:
        string_ptr = (uint8_t*)(lcd_current_l2);
        break;

    case EM_CURRENT_L3:
        string_ptr = (uint8_t*)(lcd_current_l3);
        break;

    case EM_ACTIVE_POWER:
        string_ptr = (uint8_t*)(lcd_active_power);
        break;

    case EM_EXT_ACTIVE_POWER:
        {
        string_ptr = (uint8_t*)(lcd_ext_active_power);
        }
        break;

    case EM_EXT_L1_ACTIVE_POWER:
        {
        string_ptr = (uint8_t*)(lcd_ext_l1_active_power);
        }
        break;

    case EM_EXT_L2_ACTIVE_POWER:
        {
        string_ptr = (uint8_t*)(lcd_ext_l2_active_power);
        }
        break;

    case EM_EXT_L3_ACTIVE_POWER:
        {
        string_ptr = (uint8_t*)(lcd_ext_l3_active_power);
        }
        break;

    default:
        break;
    }

for (i = 0; i < 12; i++)
    *(dst_ptr + i) = *(string_ptr + i);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_blink_enb_set
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void lcd_blink_enb_set(uint8_t blink)
{
lcd_toggle = 1;
lcd_blink_enb = blink;
lcd_set_timer(LCD_REFRESH_TIM, pdMS_TO_TICKS((uint32_t)(50)));
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_language_config
//
//  DESCRIPTION:    configurazione lingue abilitate
//
//  INPUT:          array configurazione lingue: *src_ptr
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void lcd_language_config(uint8_t *src_ptr)
{
uint8_t lang_def_bit, lang_def_byte, lang_bit;

lang_def_bit = (*(src_ptr + 4) % 8);                    // ricerca del bit della lingua di default in language_array
lang_bit = 0x01;

while (lang_def_bit > 0)
    {
    lang_bit <<= 1;
    lang_def_bit --;
    }

lang_def_byte = (*(src_ptr + 4) / 8);

*(src_ptr + lang_def_byte) |= lang_bit;                 // si forza la lingua selezionata di default a essere fra quelle abilitate [si controlla e nel caso, si forza]

SCU_InfoStation_Set ((uint8_t *)&infoStation.default_Lang, (src_ptr + 4), 1);     /* ex LANG_DEFAULT_EADD */
SCU_InfoStation_Set ((uint8_t *)&infoStation.LangConfig, (src_ptr + 0), 4);     /* ex LANG_CONFIG0_EADD */
send_to_lcd(LANGUAGE_TIME_EXPIRED);                     // forza aggiornamento a nuova lingua di default
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_language_def_enable_get
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint8_t lcd_language_def_enable_get(void)
{
return lcd_language_def_enable;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_language_set
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void lcd_language_set(uint8_t lang)
{
lcd_language = lang;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_language_def_update
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void lcd_language_def_update(void)
{
lcd_language_def_enable = 0;
lcd_main_blink = 1;
lcd_toggle = 0;
SCU_InfoStation_Set ((uint8_t *)&infoStation.default_Lang, &lcd_language, 1);     /* ex LANG_DEFAULT_EADD */
lcd_set_timer(LCD_MAIN_BLINK_TIM, LCD_MAIN_BLINK_TIME);
send_to_lcd(REFRESH_TIME_EXPIRED);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  LcdMngMsg_Old_get
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         -
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
LcdMngEvent_en LcdMngMsg_Old_get(void)
{
return LcdMngMsg_Old.LcdMngEvent;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_language_scroll
//
//  DESCRIPTION:    aggiorna la lingua visualizzata a display
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void lcd_language_scroll(void)
{
uint8_t scroll, lang_set, lang_sel, lang_bit, lang_byte, language_array[4];

scroll = 0;

eeprom_param_get(LANG_CONFIG0_EADD, language_array, 4);

lang_set = (lcd_language % 8);
lang_sel = 0x01;

while (lang_set > 0)
    {
    lang_sel <<= 1;
    lang_set --;
    }

lang_bit = (lcd_language % 8);
lang_byte = lcd_language / 8;

while ((lang_set == 0) && (scroll < 2))
    {
    lang_sel <<= 1;
    lang_bit ++;

    if (lang_bit == 8)
        {
        lang_bit = 0;
        lang_sel = 0x01;
        
        if (lang_byte < 3)
            lang_byte ++;
        else
            {
            lang_byte = 0;
            scroll ++;
            }
        }
            
    if (language_array[lang_byte] & lang_sel)
        lang_set = 1;
    }

if (scroll == 2)
    lcd_language = LANGUAGE_ENG;
else
    lcd_language = (lang_byte * 8) + lang_bit;

if (lcd_language >= NUM_LANGUAGE)   /* Fixed ticket SCU-123 */
    lcd_language = LANGUAGE_ENG;   /* Fixed ticket SCU-123 */

lcd_language_def_enable = 1;

lcd_set_timer(LCD_LANGUAGE_TIM, LCD_LANGUAGE_TIME);
lcd_set_timer(LCD_DEF_LANGUAGE_TIM, LCD_DEF_LANGUAGE_TIME);
send_to_lcd(REFRESH_TIME_EXPIRED);
}

/**
  * @brief  LANG_Modbus_to_EEprom_Translate
  * @param  Language to set in modbus format
  * @retval None
  */

void LANG_Modbus_to_EEprom_Translate(uint32_t display_lang_reg)
{

  uint8_t  cnt;
  uint32_t bit;
  
  /* Check if language received is greater than the actual language managed by the SCU (9 languages) */
  /* Modbus provide 12 languages */
  if (display_lang_reg >= (0x01 << NUM_LANGUAGE))   /* Fixed ticket SCU-123 */
    display_lang_reg = 1;    /* Force ENGLISH */    /* Fixed ticket SCU-123 */
  
   /* save new default language *****/
   for (cnt = 0, bit = 1; cnt < NUM_LANGUAGE; cnt++, bit <<= 1)
   {
     if ((display_lang_reg & bit) != 0)
     {
       // xx eeprom_array_set(LANG_DEFAULT_EADD, (uint8_t*)&cnt, 1);
       SCU_InfoStation_Set ((uint8_t *)&infoStation.default_Lang, (uint8_t*)&cnt, 1);     /* ex LANG_DEFAULT_EADD */
       lcd_language_set(cnt);
       lcd_language_def_update();
       break;
     }
   }
            
}
     
/**
  * @brief  LANG_Available_Mdb_to_EEprom_Translate
  * @param  Languages available in modbus format
  * @retval None
  */

void LANG_Available_Mdb_to_EEprom_Translate (uint32_t display_avail_lang)
{

   static uint8_t   temp[4];

   /* Primo byte scritto in eeprom LSB */
   temp[0] = display_avail_lang & 0x000000FF;
   temp[1] = (display_avail_lang >> 8) & 0x000000FF;
   temp[2] = (display_avail_lang >> 16) & 0x000000FF;
   temp[3] = (display_avail_lang >> 24) & 0x000000FF;
   /* set in eeprom */
   SCU_InfoStation_Set ((uint8_t *)&infoStation.LangConfig, (uint8_t *)temp, 4);        /* ex LANG_CONFIG0_EADD */
   
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  LcdManager_init
//
//  DESCRIPTION:    -
//
//  INPUT:          none
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void LcdManager_init(void)
{
lcd_external_em = 0;
lcd_main_blink = 0;
lcd_language_def_enable = 0;
lcd_linestring_idx = 0;
lcd_linestring_init();
lcd_toggle = 0;
lcd_blink_enb = 0;
lcd_emeter_type_old = EMETER_TYPE_NULL;
LcdPersMsg_Old.LcdMngEvent = LCD_NULL;
LcdMngMsg_Old.LcdMngEvent = LCD_WRITE_SCAME;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  lcd_external_em_set
//
//  DESCRIPTION:    -
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void lcd_external_em_set(uint8_t val)
{
lcd_external_em = val;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

/**
*
* @brief       Reinit the lcd when needed: 
*              e.g when an external contactor is activated
*
* @param [in]  none
*  
* @retval      none 
*  
****************************************************************/

void lcd_Reinit_procedure(void)
{
  if (cnttChange == CNNT_CHANGE_LCD_INIT_START)
  {
    osDelay(20);            
    Lcd2x20FastInit();           /* init the display to remove possible spurious char displayed */
    osDelay(20);            
    cnttChange = CNNT_CHANGE_LCD_INIT_FINISHED;  /* mark the procedure is finished */
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  LcdManager
//
//  DESCRIPTION:    -
//
//  INPUT:          puntatore LcdMngEvent: pMsg
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void LcdManager(LcdMngMsg_st *pMsg)
{
  
uint8_t     socket_type, lcd_info_code, error_array[EVS_ERROR_ARRAY_SIZE], data8u[3];
char        fwBootVer[20];
uint16_t    lcd_language_offset;
#ifndef USE_SLEEP_MODE_WHEN_VOLTAGE_IS_ABSENT
uint16_t    Vin;
uint8_t     Adjust_Status_Bar = FALSE;
#endif

/* Wait while the eeprom data are ready to use */ 
while (osSemaphoreAcquire (EEprom_semaphore, osWaitForever) != osOK);

osSemaphoreRelease (EEprom_semaphore);

if (pMsg->LcdMngEvent == LANGUAGE_TIME_EXPIRED)
    eeprom_param_get(LANG_DEFAULT_EADD, &lcd_language, 1);

if ((pMsg->LcdMngEvent == LCD_PERS_BACK) && (LcdPersMsg_Old.LcdMngEvent != LCD_NULL))
    {
    pMsg->LcdMngEvent = LcdPersMsg_Old.LcdMngEvent;
    LcdPersMsg_Old.LcdMngEvent = LCD_NULL;
    }
else if (((pMsg->LcdMngEvent == EVSTIME_TIME_EXPIRED) || (pMsg->LcdMngEvent == PERS_TIME_EXPIRED)
       || (pMsg->LcdMngEvent == EVS_TIME_EXPIRED)     || (pMsg->LcdMngEvent == LCD_EEPARAM_UPDATE)
       || (pMsg->LcdMngEvent == REFRESH_TIME_EXPIRED) || (pMsg->LcdMngEvent == LCD_CURRENT_UPDATE)
       || (pMsg->LcdMngEvent == LANGUAGE_TIME_EXPIRED))
       && (LcdMngMsg_Old.LcdMngEvent != LCD_NULL))
    {
    pMsg->LcdMngEvent = LcdMngMsg_Old.LcdMngEvent;
    LcdMngMsg_Old.LcdMngEvent = LCD_NULL;
    }

lcd_language_offset = ((uint16_t)(lcd_language) * WRITE_LINES_SIZE);

switch (pMsg->LcdMngEvent)
    {
    case LCD_WRITE_SCAME:
        {
        eeprom_param_get(LANG_DEFAULT_EADD, &lcd_language, 1);
        lcd_line_update(lcd_line1, lcd_write_scame_string1, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);

        /*         destination       source */
        strcpy((char*)fwBootVer, (char*)lcd_write_scame_string2);
        strcat((char*)fwBootVer, (char*)" BOOT ");
        /*         destination       source */
        strncat((char*)fwBootVer, (char*)BOOT_ADDR_VER, BOOT_VER_SIZE);

        lcd_line_update(lcd_line2, fwBootVer, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_NOPOWER:
        {
        lcd_line_update(lcd_line1, lcd_write_nopower_string1, CENTER_ALIGNMENT, LCD_EVS_MODE_ORINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, (LCD_CHARGING_ORINF | LCD_BUSY_OUTLET_TIME_ORINF));
        updateClearAllLineString();
        }
        break;

    case LCD_EMWAITING:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_EMWAITING)], CENTER_ALIGNMENT, LCD_EVS_MODE_ORINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_EMWAITING + 1)], CENTER_ALIGNMENT, (LCD_CHARGING_ORINF | LCD_BUSY_OUTLET_TIME_ORINF));
        lcd_linestring_update(lcd_linestring1, lcd_line1);
        }
        break;

    case LCD_EVS_ERROR:
        {
        evs_error_get(error_array, 1, 1, 0);
        lcd_info_code = lcd_error_code_get(error_array);

        if (lcd_info_code < 23)     // lcd_write_nu27_error
            lcd_info_code_old = lcd_info_code;
        else
            lcd_info_code = lcd_info_code_old;

        lcd_line_update(lcd_line1, lcd_error_table[lcd_info_code], CENTER_ALIGNMENT, LCD_EVS_MODE_ORINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_PENWAITNG:
        {
        lcd_line_update(lcd_line1, lcd_write_pen_fault, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        setErrorModbus(ERROR2_PENF, 2); 
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_POWER_OFF:
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_POWER_OFF)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_POWER_OFF + 1)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
#ifndef USE_SLEEP_MODE_WHEN_VOLTAGE_IS_ABSENT  
        if ((uint16_t)getFlagV230() == BACKUP_230VAC_OFF_VAL_ACTIVE)        
        {
           lcd_line_update(lcd_line2, lcd_power_off, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);           
           Adjust_Status_Bar = TRUE;
        }
#endif        
        break;

    case LCD_MASTER_ERROR:
        {
        lcd_line_update(lcd_line1, lcd_write_master_error_string1, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_EVS_DISABLED:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_EVS_DISABLED)], CENTER_ALIGNMENT, LCD_EVS_MODE_ORINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_EVS_DISABLED + 1)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_CARD_WAIT:
        {
        LcdPersMsg_Old.LcdMngEvent = LCD_CARD_WAIT;

        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_CARD_WAIT)], CENTER_ALIGNMENT, LCD_EVS_MODE_ORINF);
        
        eeprom_param_get(EVS_MODE_EADD, &data8u[0], 1);
        eeprom_param_get(RTC_VALID_EADD, &data8u[1], 1);
        eeprom_param_get(SOCKET_ENABLE_EADD, &data8u[2], 1);

        if ((data8u[0] > EVS_FREE_MODE) && (data8u[1] == 0))
            lcd_line_update(lcd_line2, lcd_write_clke_error, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        else if (evs_reserved_get() == 1)
            lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_RESERVED + 1)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        else
            lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_CARD_WAIT + 1)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);

        updateClearAllLineString();
        }
        break;

    case LCD_SOCKET_AVAILABLE:
        {
        LcdPersMsg_Old.LcdMngEvent = LCD_SOCKET_AVAILABLE;
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_SOCKET_AVAILABLE)], CENTER_ALIGNMENT, LCD_EVS_MODE_ORINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_SOCKET_AVAILABLE + 1)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_PLUG_CHECK:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_PLUG_CHECK)], CENTER_ALIGNMENT, LCD_EVS_MODE_ORINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_PLUG_CHECK + 1)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_PLUG_OUT:
        {
        if (evs_error_get(error_array, 1, 1, 0) == 1)
            {
            lcd_info_code = lcd_error_code_get(error_array);
            lcd_line_update(lcd_line1, lcd_error_table[lcd_info_code], CENTER_ALIGNMENT, LCD_EVS_MODE_ORINF);
            }
        else
            lcd_line_update(lcd_line1, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_EVS_MODE_ORINF);

        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_PLUG_OUT + 1)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_S2_WAIT:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_S2_WAIT)], CENTER_ALIGNMENT, LCD_EVS_MODE_ORINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_S2_WAIT + 1)], CENTER_ALIGNMENT, LCD_BUSY_OUTLET_TIME_ORINF);
        updateClearAllLineString();
        }
        break;

    case LCD_CHARGING:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_CHARGING)], LEFT_ALIGNMENT, (LCD_EVS_MODE_ORINF | LCD_PWM_ORINF));
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_CHARGING + 1)], CENTER_ALIGNMENT, (LCD_CHARGING_ORINF | LCD_BUSY_OUTLET_TIME_ORINF));
        lcd_linestring_update(lcd_linestring1, lcd_line1);
        
        if (lcd_linestring_idx == 0)
            lcd_linestring_update(lcd_linestring2, lcd_line2);
        else if (lcd_linestring_idx == 1)
            lcd_linestring_update(lcd_linestring3, lcd_line2);
        else if (lcd_linestring_idx == 2)
            lcd_linestring_update(lcd_linestring4, lcd_line2);
        else if (lcd_linestring_idx == 3)
            lcd_linestring_update(lcd_linestring5, lcd_line2);
        else if (lcd_linestring_idx == 4)
            lcd_linestring_update(lcd_linestring6, lcd_line2);
        else if ((lcd_line2[0] == 'P') && (lcd_line2[1] == 'e')) //lcd_linestring_idx == EM_EXT_ACTIVE_POWER)
            lcd_linestring_update(lcd_linestring8, lcd_line2);
        else if (lcd_linestring_idx == 0xFF)
            lcd_linestring_update(lcd_linestring7, lcd_line2);
        }
        break;

    case LCD_SUSPENDING:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_SUSPENDING)], CENTER_ALIGNMENT, LCD_EVS_MODE_ORINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_SUSPENDING + 1)], CENTER_ALIGNMENT, (LCD_CHARGING_ORINF | LCD_BUSY_OUTLET_TIME_ORINF));
        lcd_linestring_update(lcd_linestring1, lcd_line1);
        }
        break;

    case LCD_RMWAITNG:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_RMWAITING)], CENTER_ALIGNMENT, LCD_EVS_MODE_ORINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_RMWAITING + 1)], CENTER_ALIGNMENT, (LCD_CHARGING_ORINF | LCD_BUSY_OUTLET_TIME_ORINF));
        lcd_linestring_update(lcd_linestring1, lcd_line1);
        }
        break;

    case LCD_HTSWAITNG:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_HTSWAITING)], LEFT_ALIGNMENT, (LCD_EVS_MODE_ORINF));
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_HTSWAITING + 1)], CENTER_ALIGNMENT, (LCD_CHARGING_ORINF | LCD_BUSY_OUTLET_TIME_ORINF));
        lcd_linestring_update(lcd_linestring1, lcd_line1);
        }
        break;

    case LCD_INTERRUPTING_CHARGE:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_INTERRUPTING_CHARGE)], CENTER_ALIGNMENT, LCD_EVS_MODE_ORINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_INTERRUPTING_CHARGE + 1)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_UPDATE_WLIST:
        {
        LcdPersMsg_Old.LcdMngEvent = LCD_UPDATE_WLIST;

        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_UPDATE_WLIST)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_UPDATE_WLIST + 1)], CENTER_ALIGNMENT, LCD_UPDATE_WLIST_TIME_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_UID_DELETE_CONFIRM:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_UID_DELETE_CONFIRM)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_UID_DELETE_CONFIRM + 1)], CENTER_ALIGNMENT, LCD_DELETE_CONFIRM_TIME_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_UID_DELETED:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_UID_DELETED)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_UID_DELETED + 1)], CENTER_ALIGNMENT, LCD_USER_UID_NUM_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_UID_ADDED:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_UID_ADDED)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_UID_ADDED + 1)], CENTER_ALIGNMENT, LCD_USER_UID_NUM_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_UID_ERROR:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_UID_ERROR)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_UID_ERROR + 1)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_DELETE_WLIST_CONFIRM:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_DELETE_WLIST_CONFIRM)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_DELETE_WLIST_CONFIRM + 1)], CENTER_ALIGNMENT, LCD_DELETE_CONFIRM_TIME_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_AUTHORIZATION_FAILED:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_AUTHORIZATION_FAILED)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_AUTHORIZATION_FAILED + 1)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_AUTH_PENDING:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_AUTH_PENDING)], CENTER_ALIGNMENT, LCD_EVS_MODE_ORINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_AUTH_PENDING + 1)], LEFT_ALIGNMENT, LCD_EVS_SEC_WAIT_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_CARD_PENDING:
        {
        LcdPersMsg_Old.LcdMngEvent = LCD_CARD_PENDING;

        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_CARD_WAIT)], CENTER_ALIGNMENT, LCD_EVS_MODE_ORINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_AUTH_PENDING + 1)], LEFT_ALIGNMENT, LCD_EVS_SEC_WAIT_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_AUTH_MISSED:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_AUTH_MISSED)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_AUTH_MISSED + 1)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_PLUG_WAIT:
        {
        LcdPersMsg_Old.LcdMngEvent = LCD_PLUG_WAIT;

        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_PLUG_WAIT)], CENTER_ALIGNMENT, LCD_EVS_MODE_ORINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_PLUG_WAIT + 1)], LEFT_ALIGNMENT, LCD_EVS_SEC_WAIT_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_WLIST_DELETED:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_WLIST_DELETED)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_WLIST_DELETED + 1)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_RESET_FAILED:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_RESET_FAILED)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_RESET_FAILED + 1)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_DELETE_ERROR:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_DELETE_ERROR)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_DELETE_ERROR + 1)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_WLIST_FULL:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_WLIST_FULL)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_WLIST_FULL + 1)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_CREDIT_DATE:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_CREDIT_DATE)], LEFT_ALIGNMENT, LCD_CREDIT_DATE_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_DATE_TIME_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_NEW_CREDIT_DATE:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_NEW_CREDIT_DATE)], CENTER_ALIGNMENT, LCD_ONLY_CREDIT_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_DATE_TIME_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_ONLY_DATE:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_ONLY_DATE)], CENTER_ALIGNMENT, LCD_ONLY_DATE_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_DATE_TIME_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_ONLY_CREDIT:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_CREDIT_DATE)], CENTER_ALIGNMENT, LCD_ONLY_CREDIT_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_NEW_ONLY_CREDIT:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_NEW_CREDIT_DATE)], CENTER_ALIGNMENT, LCD_ONLY_CREDIT_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_EXPIRED_CARD:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_EXPIRED_CARD)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_EXPIRED_CARD + 1)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_CREDIT_EXHAUSTED:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_CREDIT_NULL)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_CREDIT_NULL + 1)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_NEW_DATE_DONE:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_NEW_DATE_DONE)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_DATE_TIME_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_CLOSE_LID:
        {
        eeprom_param_get(SOCKET_TYPE_EADD, &socket_type, 1);

        if (socket_type & EVS_TETHERED)
            {
            lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_PLACE_PLUG)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
            lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_PLACE_PLUG + 1)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
            }
        else
            {
            lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_CLOSE_LID)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
            lcd_line_update(lcd_line2, lang_table[(lcd_language_offset + WRITE_CLOSE_LID + 1)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
            }

        updateClearAllLineString();
        }
        break;

    case LCD_TOTAL_ENERGY:
        {
        lcd_line_update(lcd_line1, lang_table[(lcd_language_offset + WRITE_TOTAL_ENERGY)], CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_TOTAL_ENERGY_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_ENTER_PASSWORD:
        {
        lcd_line_update(lcd_line1, lcd_write_enter_password, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_ENTER_PASSWORD_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_PMNG_ENABLE:
        {
        lcd_line_update(lcd_line1, lcd_write_pmng_enable, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_PMNG_ENB_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_PMNG_TYPE:
        {
        lcd_line_update(lcd_line1, lcd_write_pmng_type, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_PMNG_TYPE_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_DOMESTIC_POWER:
        {
        lcd_line_update(lcd_line1, lcd_write_domestic_power, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_DOMESTIC_POWER_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_MIN_CURRENT:
        {
        lcd_line_update(lcd_line1, lcd_write_min_current, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_MIN_CURRENT_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_POWER_MULTIP:
        {
        lcd_line_update(lcd_line1, lcd_write_power_multip, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_POWER_MULTIP_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_POWER_ERROR:
        {
        lcd_line_update(lcd_line1, lcd_write_power_error, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_POWER_ERROR_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_POWER_DMAX:
        {
        lcd_line_update(lcd_line1, lcd_write_power_dmax, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_POWER_DMAX_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_UNBAL_ENB:
        {
        lcd_line_update(lcd_line1, lcd_write_unbal_enb, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_UNBAL_ENB_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_EMETER_CRL2:
        {
        lcd_line_update(lcd_line1, lcd_write_emeter_crl2, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_EMETER_CRL2_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_TIME_RANGE:
        {
        lcd_line_update(lcd_line1, lcd_write_time_range, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_TIME_RANGE_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_TIMED_TIME:
        {
        lcd_line_update(lcd_line1, lcd_write_timed_time, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_TIMED_TIME_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_TIMED_TIME_ENB:
        {
        lcd_line_update(lcd_line1, lcd_write_timed_enable, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_TIMED_ENB_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_ENRG_LIMIT:
        {
        lcd_line_update(lcd_line1, lcd_write_enrg_limit, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_ENRG_LIMIT_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_CHANGE_PASSWORD:
        {
        lcd_line_update(lcd_line1, lcd_write_change_password, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_CHANGE_PASSWORD_EXINF);
        updateClearAllLineString();
        }
        break;

    case LCD_NEW_PASSWORD:
        {
        lcd_line_update(lcd_line1, lcd_write_new_password, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_CHANGE_PASSWORD_EXINF);
        updateClearAllLineString();
        }
        break;
        
     case LCD_ANTENNA_WIFI_ERROR:
      {
        lcd_line_update(lcd_line1, lcd_write_antenna, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_antenna_error, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
      }
      break;
      
      
     case LCD_ANTENNA_WIFI_OK:
      {
        lcd_line_update(lcd_line1, lcd_write_antenna, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_antenna_ok, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
      }
      break;

    default:
        {
        lcd_line_update(lcd_line1, lcd_write_error_string1, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        updateClearAllLineString();
        }
        break;
    }

/* Aggiunto qua da Martin per aggiornamento in Polling errori verso la app */
/*  JAPPT-227 "Errore LIDE non gestito da APP" --> changed mifare_error_hide & lid_error_hide from 1 to 0 */
evs_error_get(error_array, 0, 0, 0);
updateModbusErrorRegisters(error_array, FALSE);

LcdMngMsg_Old.LcdMngEvent = pMsg->LcdMngEvent;

if ((evs_state_get() != EVSTATE_CHARGING) && (evs_state_get() != EVSTATE_S2_WAITING) && (evs_state_get() != EVSTATE_SUSPENDING))
    {                                                                               // in questi stati il refresh avviene ogni 1s per cui non occorre quello automatico
    if ((lcd_main_blink == 1) || (lcd_blink_enb == 1))
        lcd_set_timer(LCD_REFRESH_TIM, LCD_BLINK_TIME);                             // reload blink timer
    else
        lcd_set_timer(LCD_REFRESH_TIM, LCD_REFRESH_TIME);                           // reload default timer
    }

if (lcd_main_blink == 1)
    {
    if (lcd_toggle == 0)
        {
        lcd_line_update(lcd_line1, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        lcd_line_update(lcd_line2, lcd_write_empty_line, CENTER_ALIGNMENT, LCD_AUX_NULL_EXINF);
        }

    lcd_toggle ^= 1;
    lcd_set_timer(LCD_REFRESH_TIM, LCD_BLINK_TIME);                                 // reload blink timer
    }

/* Check if an LCD init procedure is required (e.g for an external contactor activation) */
lcd_Reinit_procedure();

#ifdef USE_SLEEP_MODE_WHEN_VOLTAGE_IS_ABSENT    
putsxy_c(1, 1, lcd_line1);
putsxy_c(1, 2, lcd_line2);
#else
/* Draw the first row  */
putsxy_c(1, 1, lcd_line1);
/* Draw the second: need to adjust the status bar? */
if (Adjust_Status_Bar == FALSE)
  putsxy_c(1, 2, lcd_line2);  
else
{
  
   /* Check if status bar is alredy visualized */
   if (LCD_Check_StatusBar_Presence() == FALSE)
   {
      /* Clear the row */
      putsxy_c(1, 2, lcd_line_null);  
      /* Delay to read the right Vin, otherwise the first entry give even 24V */
      HAL_Delay(1500);
   }   
   /* Read Vin value */
   Vin = vinPwrValue();
   /* Draw the bar */
   putsxy_c(1, 2, lcd_line2);  
   /* Show the right status of Vin */
   BACKUP_Manage_Progress_Bar(Vin);
   
   Adjust_Status_Bar = FALSE;
}
#endif

}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  LcdMngTask
//
//  DESCRIPTION:    gestione display lcd 20x2
//
//  INPUT:          -
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void LcdMngTask(void *pvParameters)
{
uint8_t i;

/* init task */

/*-------- Creates an empty mailbox for LcdMngTask messages --------------------------*/
LcdMngQueue = xQueueCreate(4, sizeof(LcdMngMsg_st));
configASSERT(LcdMngQueue != NULL);

/*-------- Creates all timer for LcdMngTask ------------------------------------------*/
/* in this case we use one shot features */
for (i = 0; i < LCD_NUM_TIMER; i++)
    {
    xLcdMngTimers[i] = xTimerCreate("TimLcdMng", portMAX_DELAY, pdFALSE, (void*)(i), LcdMngTimCallBack);
    configASSERT(xLcdMngTimers[i] != NULL);
    }

LcdManager_init();

/* start task */

for (;;)
    {
    /* Wait for some event from SW */
    if (xQueueReceive(LcdMngQueue, (void *)&LcdMngMsg, portMAX_DELAY) == pdPASS)
        {
        LcdManager(&LcdMngMsg);
        }
    else
        {
        /* Wait for possible handled timeout */
        }
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

/**
*
* @brief        get the pointer to string LCD lines      
*
* @param [in]   uint8_t: row number 
* @param [in]   uint8_t*: row len 
*
* @retval       char*: pointer to row string 
*
***********************************************************************************************************************/
char* getLineString(uint8_t numLine, uint8_t* pLineLen)
{
char* pLine;

*pLineLen = (uint8_t)LCD_CHAR_NUM;

switch (numLine)
    {
    case 1:
        pLine = lcd_linestring1;
        break;

    case 2:
        pLine = lcd_linestring2;
        break;

    case 3:
        pLine = lcd_linestring3;
        break;

    case 4:
        pLine = lcd_linestring4;
        break;

    case 5:
        pLine = lcd_linestring5;
        break;

    case 6:
        pLine = lcd_linestring6;
        break;

    case 7:
        pLine = lcd_linestring7;
        break;

    case 8:
        pLine = lcd_linestring8;
        break;

    default:
        pLine = (char*)(lcd_line_error);
        break;
    }

return (pLine); 
}

/**
  * @brief  Update the 2 LCD lines and clear all string lines used in display visualization  
  *         
  * @param  none
  * 
  * @retval none
  */
static void updateClearAllLineString (void)
{
  lcd_linestring_update(lcd_linestring1, lcd_line1);
  lcd_linestring_update(lcd_linestring2, lcd_line2);
  lcd_linestring_update(lcd_linestring3, (char*)(lcd_line_null));
  lcd_linestring_update(lcd_linestring4, (char*)(lcd_line_null));
  lcd_linestring_update(lcd_linestring5, (char*)(lcd_line_null));
  lcd_linestring_update(lcd_linestring6, (char*)(lcd_line_null));
  lcd_linestring_update(lcd_linestring7, (char*)(lcd_line_null));
  lcd_linestring_update(lcd_linestring8, (char*)(lcd_line_null));
}
#ifdef STRIGHE_ENG
static const char lcd_write_evs_disabled_eng1[]         = "SOCKET";                         // ENG: 00 - WRITE_EVS_DISABLED
static const char lcd_write_card_wait_eng1[]            = "SHOW CARD";                      // ENG: 01 - WRITE_CARD_WAIT
static const char lcd_write_socket_available_eng1[]     = "SOCKET";                         // ENG: 02 - WRITE_SOCKET_AVAILABLE
static const char lcd_write_plug_check_eng1[]           = "INSERTED";                       // ENG: 03 - WRITE_PLUG_CHECK
static const char lcd_write_plug_out_eng2[]             = "PLUG OUT";		                    // ENG: 04 - WRITE_PLUG_OUT
static const char lcd_write_s2_waiting_eng1[]           = "EV WAITING";                     // ENG: 05 - WRITE_S2_WAIT
static const char lcd_write_charging_eng1[]             = "CHARGING";                       // ENG: 06 - WRITE_CHARGING
static const char lcd_write_suspendig_eng1[]            = "SUSPENDING";                     // ENG: 07 - WRITE_SUSPENDING
static const char lcd_write_rmwaiting_eng1[]            = "RM WAITING";                     // ENG: 08 - WRITE_RMWAITING
static const char lcd_write_interrupting_charge_eng1[]  = "INTERRUPTING";                   // ENG: 09 - WRITE_INTERRUPTING_CHARGE
static const char lcd_write_update_wlist_eng1[]         = "ARCHIVE MANAGEMENT";             // ENG: 10 - WRITE_UPDATE_WLIST
static const char lcd_uid_delete_confirm_eng1[]         = "DELETE USER?";                   // ENG: 11 - WRITE_UID_DELETE_CONFIRM
static const char lcd_uid_deleted_eng1[]                = "ID DELETED";                     // ENG: 12 - WRITE_UID_DELETED
static const char lcd_uid_added_eng1[]                  = "ID ADDED";                       // ENG: 13 - WRITE_UID_ADDED
static const char lcd_uid_error_eng1[]                  = "UNAUTHORIZED";                   // ENG: 14 - WRITE_UID_ERROR
static const char lcd_delete_wlist_confirm_eng1[]       = "DELETE ARCHIVE?";                // ENG: 15 - WRITE_DELETE_WLIST_CONFIRM
static const char lcd_authorization_failed_eng1[]       = "AUTHORIZATION";                  // ENG: 16 - WRITE_AUTHORIZATION_FAILED
static const char lcd_plug_wait_eng1[]                  = "PLUG IN";                        // ENG: 17 - WRITE_PLUG_WAIT
static const char lcd_wlist_deleted_eng1[]              = "ARCHIVE DELETED";                // ENG: 18 - WRITE_WLIST_DELETED
static const char lcd_reset_failed_eng1[]               = "FAILED";                         // ENG: 19 - WRITE_RESET_FAILED
static const char lcd_delete_error_eng1[]               = "ARCHIVE MANAGEMENT";             // ENG: 20 - WRITE_DELETE_ERROR
static const char lcd_wlist_full_eng1[]                 = "ARCHIVE FULL";                   // ENG: 21 - WRITE_WLIST_FULL
static const char lcd_expired_card_eng1[]               = "EXPIRED";                        // ENG: 22 - WRITE_EXPIRED_CARD
static const char lcd_credit_null_eng1[]                = "NULL";                           // ENG: 23 - WRITE_CREDIT_NULL
static const char lcd_close_lid_eng1[]                  = "CLOSE";                          // ENG: 24 - WRITE_CLOSE_LID
static const char lcd_total_energy_eng1[]               = "SUPPLIED ENERGY";                // ENG: 25 - WRITE_TOTAL_ENERGY
static const char lcd_auth_pending_eng1[]               = "PLEASE WAIT";                    // ENG: 26 - WRITE_AUTH_PENDING
static const char lcd_credit_date_eng1[]                = "CREDIT ";                        // ENG: 27 - WRITE_CREDIT_DATE
static const char lcd_new_credit_date_eng1[]            = "NEW CREDIT ";                    // ENG: 28 - WRITE_NEW_CREDIT_DATE
static const char lcd_only_date_eng1[]                  = "CARD DATE";                      // ENG: 29 - WRITE_ONLY_DATE
static const char lcd_new_date_done_eng1[]              = "DATE AND TIME";                  // ENG: 30 - WRITE_NEW_DATE_DONE
static const char lcd_place_plug_eng1[]                 = "STOW";                           // ENG: 31 - WRITE_PLACE_PLUG
static const char lcd_auth_missed_eng1[]                = "AUTHORIZATION";                  // ENG: 32 - WRITE_AUTH_MISSED  
static const char lcd_power_off_eng1[]                  = "MAINS BREAKDOWN";                // ENG: 33 - WRITE_POWER_OFF  
static const char lcd_write_htswaiting_eng1[]           = "OT WAITING";                     // ENG: 34 - WRITE_HTSWAITING
static const char lcd_write_emwaiting_eng1[]            = "EM WAITING";                     // ENG: 35 - WRITE_EMWAITING
static const char lcd_write_reserved_eng2[]             = "RESERVED";                       // ENG: 36 - WRITE_RESERVED
#endif

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//
//  File:           ExtInpMng.h
//  Author:         Vania
//  Date:           06/11/2020
//
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#ifndef _EXTINPMNG_H
#define _EXTINPMNG_H
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global include ------------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global defines ------------------------------------------------------------------------------------------------------------------------- //
#define HIDDEN_MENU_NULL		(uint8_t)(0x00) //

#define HIDDEN_MENU_PMNG_VIS    (uint8_t)(0x01) // 1 -> visibilità menù power management abilitato
#define HIDDEN_MENU_TMEG_VIS    (uint8_t)(0x02) // 1 -> visibilità menù ricarica a tempo abilitato
#define HIDDEN_MENU_BYTE02      (uint8_t)(0x04) // 
#define HIDDEN_MENU_BYTE03      (uint8_t)(0x08) // 
#define HIDDEN_MENU_BYTE04      (uint8_t)(0x10) // 
#define HIDDEN_MENU_BYTE05      (uint8_t)(0x20) // 
#define HIDDEN_MENU_BYTE06      (uint8_t)(0x40) // 
#define HIDDEN_MENU_BYTE07      (uint8_t)(0x80) // 

#define HIDDEN_MENU_PMNG_ENB    (HIDDEN_MENU_PMNG_VIS)
#define HIDDEN_MENU_BYTE11      (uint8_t)(0x02)
#define HIDDEN_MENU_SINAPSI     (uint8_t)(0x04)
#define HIDDEN_MENU_SEM_ENB     (uint8_t)(0x08) // SEM enable flag (remote)
#define HIDDEN_MENU_BYTE14      (uint8_t)(0x10)
#define HIDDEN_MENU_BYTE15      (uint8_t)(0x20)
#define HIDDEN_MENU_BYTE16      (uint8_t)(0x40)
#define HIDDEN_MENU_BYTE17      (uint8_t)(0x80)

#define HTML_MENU_PMNG_SEM      (uint8_t)(0x40)  

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global typedef ------------------------------------------------------------------------------------------------------------------------- //
typedef enum // possible value of hidden_menu_sel
{
HIDDEN_IDLE = 0,
HIDDEN_TOTAL_ENERGY,
HIDDEN_ENTER_PASSWORD,
HIDDEN_SHOW_PMNGENB,
HIDDEN_ENTER_PMNGENB,
HIDDEN_SHOW_PMNGTYPE,
HIDDEN_ENTER_PMNGTYPE,
HIDDEN_SHOW_DOMPOWER,
HIDDEN_ENTER_DOMPOWER,
HIDDEN_SHOW_MINCURRENT,
HIDDEN_ENTER_MINCURRENT,
HIDDEN_SHOW_PWRMULTIP,
HIDDEN_ENTER_PWRMULTIP,
HIDDEN_SHOW_PWRERROR,
HIDDEN_ENTER_PWRERROR,
HIDDEN_SHOW_PWRDMAX,
HIDDEN_ENTER_PWRDMAX,
HIDDEN_SHOW_UNBAL,
HIDDEN_ENTER_UNBAL,
HIDDEN_SHOW_EMETER_CRL2,
HIDDEN_ENTER_EMETER_CRL2,
HIDDEN_SHOW_TIME_RANGE,
HIDDEN_ENTER_TIME_RANGE,
HIDDEN_SHOW_TIMED_TIME_ENB,
HIDDEN_ENTER_TIMED_TIME_ENB,
HIDDEN_SHOW_TIMED_TIME,
HIDDEN_ENTER_TIMED_TIME,
HIDDEN_SHOW_ENRG_LIMIT,
HIDDEN_ENTER_ENRG_LIMIT,
HIDDEN_SHOW_CHANGE_PASSWORD,
HIDDEN_ENTER_CHANGE_PASSWORD,
HIDDEN_SEL_NUM
}hidden_menu_sel_en;

typedef enum // possible value of lid_state
{
LID_NULL_STATE = 0,
LID_OPEN,
LID_CLOSE
}lid_state_en;

typedef enum
{
EXTINP_EVENT_NULL = 0,
PULS_SWITCH_UPDATE,
PULS_TIME_EXPIRED,
REMOTE_SWITCH_UPDATE,
EXTINP_PULS_HELD,
EXTINP_PULS_HIDDEN,
HIDDEN_TIME_EXPIRED,
EXTINP_TIMEOUT,
EXTINP_SW_PROG,
SW_PROG_TIME_EXPIRED,
EXTINP_MASTER_CARD
}ExtInpMngEvent_en;

/* queue info structure */
typedef __packed struct
{
ExtInpMngEvent_en    ExtInpMngEvent;
}ExtInpMngMsg_st;
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global function prototypes ------------------------------------------------------------------------------------------------------------- //
void send_to_extinp(uint8_t extinp_event);
void hidden_menu_init(void);
hidden_menu_sel_en hidden_menu_sel_get(void);
uint8_t hidden_array_index_get(void);
void hidden_enter_array_get(uint8_t *dst_ptr);
void hidden_menu_enable_set(uint8_t enable);
void ExtInpMngTask(void *pvParameters);
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#endif

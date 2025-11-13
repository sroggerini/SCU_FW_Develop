/******************* (C) COPYRIGHT 2020 SCAME ********************************************************************/
/**
* @file        port.h
*
* @brief       Port manager - Definition -
*
* @author      Vania
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: port.h 76 2022-06-20 09:46:05Z npiergi $
*
*     $Revision: 76 $
*
*     $Author: npiergi $
*
*     $Date: 2022-06-20 11:46:05 +0200 (lun, 20 giu 2022) $
*
*
* @copyright
*       Copyright (C) 2017 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

/************************************************************
 * Include
 ************************************************************/

#ifndef _PORT_H
#define _PORT_H

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#define LEDA_ON     (uint8_t)(0x01)
#define LEDB_ON     (uint8_t)(0x02)
#define LEDC_ON     (uint8_t)(0x04)
#define LEDA_BLINK  (uint8_t)(0x10)
#define LEDB_BLINK  (uint8_t)(0x20)
#define LEDC_BLINK  (uint8_t)(0x40)
#define LEDX_BLINK  (LEDC_BLINK + LEDB_BLINK + LEDA_BLINK)

#define LED_PLGF    (LEDB_ON)                 // stato presa libera
#define LED_PLGB    (LEDA_ON)                 // stato presa occupata: in carica
#define LED_PLGT    (LEDA_BLINK)              // transizione senza errore ma presa non disponibile/impegnata
#define LED_PLGU    (LEDB_BLINK)              // stato presa occupata: richiesta estrazione spina

#define LED_PLGE    (LEDC_BLINK + LEDB_BLINK) // transizione con errore non bloccante
#define LED_PLGR    (LEDC_ON)                 // stato con errore bloccante: richiesta riavvio Max0
#define LED_PLGX    (LEDC_BLINK)              // stato con errore: richiesta estrazione spina

#define BLCK_ST     (uint8_t)(0x80)           // blocchi fermi

#define BLCK_NL		(uint8_t)(0x00)					// null info
#define BLCK_CK		(uint8_t)(0x01)					// posizione blocchi non nota
#define BLCK_DN		(uint8_t)(0x02 + BLCK_ST)		// posizione blocchi fermi bassi
#define BLCK_UP		(uint8_t)(0x03 + BLCK_ST)		// posizione blocchi fermi alti
#define BLCK_DR		(uint8_t)(0x04)					// drive attivo e blocchi in transizione
#define BLCK_PS		(uint8_t)(0x05)					// pausa lettura posizione blocchi a transizione esaurita
#define BLCK_NM		(uint8_t)(0x06)					// pausa drive prima di fare un nuovo tentativo
#define BLCK_EV		(uint8_t)(0x07)					// drive attivo e blocchi in transizione: valutazione preliminare switch di posizione

#define CNTT_OPEN   (uint8_t)(0x00)
#define CNTT_CLOSE  (uint8_t)(0x01)

#define STOP_STP    (uint8_t)(0x01)
#define STOP_INF    (uint8_t)(0x02)

#define ENABLE_UART0_INTERRUPT    IE |= (0x10);
#define DISABLE_UART0_INTERRUPT   IE &= (0xEF);
#define ENABLE_ALL_INTERRUPT    IE |= (0x80);
#define DISABLE_ALL_INTERRUPT   IE &= (0x7F);
#define ENABLE_TIMER5_INTERRUPT   EIE2 |= (0x20);
#define DISABLE_TIMER5_INTERRUPT  EIE2 &= (0xDF);

// --------------------------------------------------------------------------------------------------------------------------- //

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Global Constants                              **
**                                                                          **
******************************************************************************
*/

// --------------------------------------------------------------------------------------------------------------------------- //

/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/
extern uint8_t stop_down;

extern uint8_t rtsw_read;
extern uint8_t card_dtcd;
extern uint8_t rcdm_open;
extern uint8_t rmen_open;
extern uint8_t lang_butt;
extern uint8_t info_butt;
extern uint8_t stop_butt;
extern uint8_t mirr_open;
extern uint8_t rcbo_open;
extern uint8_t blck_psup;
extern uint8_t blck_psdn;
extern uint8_t vent_avbl;
extern uint8_t lidc_open;

extern uint8_t mstr_butt;

extern uint8_t rst_butt;
extern uint8_t ers_butt;

extern uint8_t cntt_att;
extern uint8_t cntt_fult;
extern uint8_t cntt_next;

extern uint8_t blck_fult;
extern uint8_t blck_stdy;
extern uint8_t blck_next;
extern uint8_t blck_state;

extern uint8_t led_drive;

extern uint8_t digt_rdok;

extern uint8_t card_renb;
extern uint8_t digt_renb;

extern uint8_t wdled;
// --------------------------------------------------------------------------------------------------------------------------- //

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Global Function Prototypes                    **
**                                                                          **
******************************************************************************
*/
void digt_init(void);
void digt_rstr(void);
void digt_scan(void);
void dled_manager(void);
void cntt_init(void);
void cntt_manager(void);
// --------------------------------------------------------------------------------------------------------------------------- //
// --------------------------------------------------------------------------------------------------------------------------- //
#endif // _PORT_H

/*************** END OF FILE ******************************************************************************************/

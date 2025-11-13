// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//																																									   //
//	File:		PilotMng.c																																			   //
//	Author:		Vania																																				   //
//	Date:		06/11/2020																																			   //
//                                                          										                                                         		   //
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

#include "adcTask.h"
#include "eeprom.h"
#include "InputsMng.h"

#include "BlockMng.h"
#include "EvsMng.h"
#include "PilotMng.h"
#include "PwmMng.h"

#include "sbcGsy.h"
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local defines -------------------------------------------------------------------------------------------------------------------------- //
#define S2_STATE_B_MAX		(uint16_t)(2873)			// 10,50V [2873mV da partitore] tensione limite SUPERIORE STATO B
#define S2_STATE_B_MIN		(uint16_t)(2198)			//  8,00V [2198mV da partitore] tensione limite INFERIORE STATO B

#define S2_STATE_C_MAX		(uint16_t)(2068)			//  7,50V [2068mV da partitore] tensione limite SUPERIORE STATO C
#define S2_STATE_C_MIN		(uint16_t)(1393)			//  5,00V [1393mV da partitore] tensione limite INFERIORE STATO C

#define S2_STATE_D_MAX		(uint16_t)(1265)			//  4,50V [1265mV da partitore] tensione limite SUPERIORE STATO D
#define S2_STATE_D_MIN		(uint16_t)(587)				//  2,00V [587mV da partitore] tensione limite INFERIORE STATO D

#define CP_PRESENCE_SUP		(uint16_t)(2937)		        // 10,75V [2937mV da partitore] tensione limite spina non connessa
#define CP_PRESENCE_INF		(S2_STATE_B_MAX)			// 10,00V [2736mV da partitore] tensione limite SUPERIORE STATO B

#define CP_SHORT_SUP		(S2_STATE_D_MIN)			//  2,00V [587mV da partitore] tensione limite SUPERIORE per uscire da CPshort
//#define CP_SHORT_INF		(uint16_t)(319)				//  1,00V [319mV da partitore] tensione limite INFERIORE per entrare in CPshort
#define CP_SHORT_INF		(uint16_t)(476)				//  1,50V [319mV da partitore] tensione limite INFERIORE per entrare in CPshort

#define PP_CABLE_13A_MAX	(uint16_t)(2900)			// 2700ohm PP 13A: 2700 > Rc > 1000
#define PP_CABLE_13A_MIN	(uint16_t)(2400)			// 1000ohm [2400mV da partitore]
#define PP_CABLE_20A_MAX	(PP_CABLE_13A_MIN)			// 1000ohm PP 20A: 1000 > Rc > 330
#define PP_CABLE_20A_MIN	(uint16_t)(1580)			// 330ohm [1580mV da partitore]
#define PP_CABLE_32A_MAX	(PP_CABLE_20A_MIN)			// 330ohm PP 32A: 330 > Rc > 150
//#define PP_CABLE_32A_MIN	(uint16_t)(1030)			// 150ohm [1030mV da partitore]
#define PP_CABLE_32A_MIN	(uint16_t)(1100)
#define PP_CABLE_63A_MAX	(PP_CABLE_32A_MIN)			// 150ohm PP 63A: 150 > Rc > 75
#define PP_CABLE_63A_MIN	(uint16_t)(680)				// 75ohm [680mV da partitore]

#define PP_SHORT_SUP		(PP_CABLE_63A_MIN)			// [680mV da partitore] tensione limite SUPERIORE per uscire da ppshort
//#define PP_SHORT_INF		(uint16_t)(340)				// [340mV da partitore] tensione limite INFERIORE per entrare in ppshort
#define PP_SHORT_INF		(uint16_t)(550)	/* RIVEDERE */

#define CP_INSTANT_COUNT        (uint8_t)(1)
#define PP_INSTANT_COUNT        (uint8_t)(1)
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local typedef -------------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local constats ------------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local references ----------------------------------------------------------------------------------------------------------------------- //
xQueueHandle PilotMngQueue = NULL;

static PilotMngMsg_st		PilotMngMsg;

static plug_state_en		plug_state;

static uint8_t				xpshort_enable;
static uint8_t				xppresence_enable;
static uint8_t				xp_presence_freeze;

static uint8_t				cpshort_set;
//static uint8_t				cpshort_counter;
static uint8_t				cplost_set;
static uint16_t				cppresence_instant;
static uint8_t				cppresence_instant_counter;
static uint16_t				cppresence_filtered;
static uint8_t				cppresence_filtered_counter;

static uint8_t				ppshort_set;
//static uint8_t				ppshort_counter;
static uint8_t				pplost_set;

#ifndef HW_MP28947
static uint16_t				pppresence_instant;
static uint8_t				pppresence_instant_counter;
static uint16_t				pppresence_filtered;
static uint8_t				pppresence_filtered_counter;
#endif

static uint16_t				ppcurrent;

static uint16_t				plug_presence;
static uint16_t				plug_presence_old;

static s2_state_en			s2_state;
static s2_state_en			s2_state_old;
static uint8_t				s2_enable;
static uint8_t				s2A_counter;
static uint8_t				s2Bx_counter;
static uint8_t				s2B_counter;
static uint8_t				s2C_counter;
static uint8_t				s2D_counter;

static rect_state_en		rect_state;
static uint8_t				rect_enable;
static uint8_t				rect_counter;

static uint16_t             s2_state_b_max_level;
static uint16_t             s2_state_b_min_level;
static uint16_t             s2_state_c_max_level;
static uint16_t             s2_state_c_min_level;
static uint16_t             s2_state_d_max_level;
//static uint16_t             s2_state_d_min_level;

static uint16_t             CPSN_VAL;
#ifndef HW_MP28947
static uint16_t             PPSN_VAL;
#endif
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- local function prototypes -------------------------------------------------------------------------------------------------------------- //
xQueueHandle getPilotMngQueueHandle(void);

static void cpshort_init(void);
static void cppresence_init(void);
static void cp_init(void);
#ifndef HW_MP28947
static void ppshort_init(void);
static void pppresence_init(void);
static void pp_init(void);
#endif
static void xp_short_manager(uint8_t enable);
static void xp_presence_manager(uint8_t enable);
static void plug_manager(uint8_t config);
static void s2_init(void);
static void s2_manager(void);
static void rect_init(void);
static void rect_manager(uint8_t enable);
uint32_t pilot_rtc_backup_get(uint32_t reg);
static void PilotManager_init(void);
static uint32_t PilotManager(PilotMngMsg_st *pMsg);
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global constats ------------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// -------------------------- global references ---------------------------------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	getPilotMngQueueHandle
//
//	DESCRIPTION:	-
//
//	INPUT:			none
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
xQueueHandle getPilotMngQueueHandle(void)
{
return(PilotMngQueue);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	send_to_pilot
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			-
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void send_to_pilot(uint8_t pilot_event)
{
PilotMngMsg_st	msgPilotSend;

msgPilotSend.PilotMngEvent = (PilotMngEvent_en)(pilot_event);
configASSERT(xQueueSendToBack(getPilotMngQueueHandle(), (void *)&msgPilotSend, portMAX_DELAY) == pdPASS);
tPrintf("send to pilot\n\r");
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//  FUNCTION NAME:  pilot_rtc_backup_get
//
//  DESCRIPTION:    legge parametri della ricarica per la gestione degli stati di low power mode
//
//  INPUT:          status
//
//  OUTPUT:         none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint32_t pilot_rtc_backup_get(uint32_t reg)
{
return HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)getHandleRtc(), reg);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	plug_state_get
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			-
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
plug_state_en plug_state_get(void)
{
return plug_state;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	ppcurrent_get
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			-
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint16_t ppcurrent_get(void)
{
return ppcurrent;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	pilot_lost_set
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			-
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void pilot_lost_set(uint8_t anom1, uint8_t val)
{
if (anom1 == CPLOST_ANOM1)
	cplost_set = val;

if (anom1 == PPLOST_ANOM1)
	pplost_set = val;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	pilot_anom1_update
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			-
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void pilot_anom1_update(void)
{
if (cpshort_set == 1)
    evs_error_set(CONTROL_BYTE_1, CPSHORT_ANOM1, 1);
else
    evs_error_set(CONTROL_BYTE_1, CPSHORT_ANOM1, 0);

if (ppshort_set == 1)
    evs_error_set(CONTROL_BYTE_1, PPSHORT_ANOM1, 1);
else
    evs_error_set(CONTROL_BYTE_1, PPSHORT_ANOM1, 0);

if (cplost_set == 1)
    evs_error_set(CONTROL_BYTE_1, CPLOST_ANOM1, 1);
else
    evs_error_set(CONTROL_BYTE_1, CPLOST_ANOM1, 0);

if (pplost_set == 1)
    evs_error_set(CONTROL_BYTE_1, PPLOST_ANOM1, 1);
else
    evs_error_set(CONTROL_BYTE_1, PPLOST_ANOM1, 0);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	pilot_anom2_update
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			-
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void pilot_anom2_update(void)
{
if (rect_state == RECT_ABSENT)
    evs_error_set(CONTROL_BYTE_2, RECTIFIER_ANOM2, 1);
else
    evs_error_set(CONTROL_BYTE_2, RECTIFIER_ANOM2, 0);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	pilot_error_reset
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			-
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void pilot_error_reset(void)
{
uint8_t		socket_type;

eeprom_param_get(SOCKET_TYPE_EADD, &socket_type, 1);

//if (cpshort_set == 0)
	cpshort_init();

cppresence_init();

#ifndef HW_MP28947
//if (ppshort_set == 0)
	ppshort_init();

pppresence_init();
#endif

rect_init();
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	cpshort_init
//
//	DESCRIPTION:	-
//
//	INPUT:			none
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void cpshort_init(void)
{
evs_error_set(CONTROL_BYTE_1, CPSHORT_ANOM1, 0);
//cpshort_counter = 0;
cpshort_set = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	cppresence_init
//
//	DESCRIPTION:	-
//
//	INPUT:			none
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void cppresence_init(void)
{
evs_error_set(CONTROL_BYTE_1, CPLOST_ANOM1, 0);
cppresence_filtered = 0;
cppresence_filtered_counter = 0;
cppresence_instant = 0;
cppresence_instant_counter = 0;
cplost_set = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	cp_init
//
//	DESCRIPTION:	-
//
//	INPUT:			none
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void cp_init(void)
{
cpshort_init();
cppresence_init();
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

#ifndef HW_MP28947
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	ppshort_init
//
//	DESCRIPTION:	-
//
//	INPUT:			none
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void ppshort_init(void)
{
evs_error_set(CONTROL_BYTE_1, PPSHORT_ANOM1, 0);
//ppshort_counter = 0;
ppshort_set = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	pppresence_init
//
//	DESCRIPTION:	-
//
//	INPUT:			none
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void pppresence_init(void)
{
evs_error_set(CONTROL_BYTE_1, PPLOST_ANOM1, 0);
pppresence_filtered = 0;
pppresence_filtered_counter = 0;
pppresence_instant = 0;
pppresence_instant_counter = 0;
pplost_set = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	pp_init
//
//	DESCRIPTION:	-
//
//	INPUT:			none
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void pp_init(void)
{
ppshort_init();
pppresence_init();
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
#endif

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	xp_short_enable_set
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void xp_short_enable_set(uint8_t enable)
{
if (enable == 1)
    xpshort_enable |= (PPSHORT_CRL1 | CPSHORT_CRL1);
else
    xpshort_enable = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	xp_presence_enable_set
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void xp_presence_enable_set(uint8_t enable)
{
//if (enable == 1)
//    xppresence_enable |= (PPLOST_CRL1 | CPLOST_CRL1);
//else
//    xppresence_enable = 0;
xppresence_enable = enable;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	xp_short_manager
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			-
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void xp_short_manager(uint8_t enable)
{
if ((enable & CPSHORT_CRL1) == 0)                                                   /* START cpshort */
	cpshort_init();
else
	{
	if ((cpshort_set == 0) && (CPSN_VAL < CP_SHORT_INF))
		{
		//cpshort_counter = 0;
		cpshort_set = 1;
		send_to_evs(EVS_xP_SHORT);
		}
//	else if ((cpshort_set == 1) && (CPSN_VAL > CP_SHORT_SUP))
//		{
//		if (cpshort_counter >= 100)	// 100 * 5ms = 500ms
//			{
//			cpshort_counter = 0;
//			cpshort_set = 0;
//			}
//		else
//			cpshort_counter ++;
//		}
//	else
//		cpshort_counter = 0;
	} /* END OF cpshort */

#ifndef HW_MP28947	
if ((enable & PPSHORT_CRL1) == 0)                                                   /* START ppshort */
	ppshort_init();
else
	{
	if ((ppshort_set == 0) && (PPSN_VAL < PP_SHORT_INF))
		{
		//ppshort_counter = 0;
		ppshort_set = 1;
		send_to_evs(EVS_xP_SHORT);
		}
//	else if ((ppshort_set == 1) && (PPSN_VAL > PP_SHORT_SUP))
//		{
//		if (ppshort_counter >= 100)	// 100 * 5ms = 500ms
//			{
//			ppshort_counter = 0;
//			ppshort_set = 0;
//			}
//		else
//			ppshort_counter ++;
//		}
//	else
//		ppshort_counter = 0;
	}
#endif
/* END OF ppshort */
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	xp_presence_freeze_set
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void xp_presence_freeze_set(uint8_t val)
{
xp_presence_freeze = val;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	xp_presence_manager
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void xp_presence_manager(uint8_t enable)
{
if (enable == 0)
    {
    cppresence_init();
//    pppresence_init();
    }

if (xp_presence_freeze == 1)
    {
	}
else if (CPSN_VAL >= CP_SHORT_INF)
	{
//	if (CPSN_VAL < CP_PRESENCE_INF)													/* START cppresence */
//		cppresence_instant = CP_INSTANT;
//	else if (CPSN_VAL > CP_PRESENCE_SUP)
//		cppresence_instant = 0;
	
	if ((cppresence_instant == 0) && (CPSN_VAL < CP_PRESENCE_INF))
		{
		cppresence_instant_counter ++;

		if (cppresence_instant_counter >= CP_INSTANT_COUNT)	// CP_INSTANT_COUNT * 5ms
			{
			cppresence_instant_counter = 0;
			cppresence_instant = CP_INSTANT;
			}
		}
	else if ((cppresence_instant == CP_INSTANT) && (CPSN_VAL > CP_PRESENCE_SUP))
		{
		cppresence_instant_counter ++;

		if (cppresence_instant_counter >= CP_INSTANT_COUNT)	// CP_INSTANT_COUNT * 5ms = 100ms
			{
			cppresence_instant_counter = 0;
			cppresence_instant = 0;
			}
		}
	else
		cppresence_instant_counter = 0;

	if ((cppresence_filtered == 0) && (CPSN_VAL < CP_PRESENCE_INF))
		{
		if (cppresence_filtered_counter >= 20)	// 20 * 5ms = 100ms
			{
			cppresence_filtered_counter = 0;
			cppresence_filtered = CP_FILTERED;
			}
		else
			cppresence_filtered_counter ++;
		}
	else if ((cppresence_filtered == CP_FILTERED) && (CPSN_VAL > CP_PRESENCE_SUP))
		{
		if (cppresence_filtered_counter >= 20)	// 20 * 5ms = 100ms
			{
			cppresence_filtered_counter = 0;
			cppresence_filtered = 0;
			}
		else
			cppresence_filtered_counter ++;
		}
	else
		cppresence_filtered_counter = 0;											/* END OF cppresence */
	}

#ifndef HW_MP28947
if (xp_presence_freeze == 1)
    {
	}
else if (PPSN_VAL >= PP_SHORT_INF)
	{
	if (PPSN_VAL < PP_CABLE_63A_MAX)												/* START pppresence */
		ppcurrent = 630;					// cable dimensioning: 63A [A*10]
	else if (PPSN_VAL < PP_CABLE_32A_MAX)
		ppcurrent = 320;					// cable dimensioning: 32A [A*10]
	else if (PPSN_VAL < PP_CABLE_20A_MAX)
		ppcurrent = 200;					// cable dimensioning: 20A [A*10]
	else if (PPSN_VAL < PP_CABLE_13A_MAX)
		ppcurrent = 130;					// cable dimensioning: 13A [A*10]
	else
		ppcurrent = 0;
	
//	if (ppcurrent > 0)
//		pppresence_instant = PP_INSTANT;
//	else
//		pppresence_instant = 0;
	
	if ((pppresence_instant == 0) && (ppcurrent > 0))
		{
        pppresence_instant_counter ++;

		if (pppresence_instant_counter >= PP_INSTANT_COUNT)	// PP_INSTANT_COUNT * 5ms
			{
			pppresence_instant_counter = 0;
			pppresence_instant = PP_INSTANT;
			}
		}
	else if ((pppresence_instant == PP_INSTANT) && (ppcurrent <= 0))
		{
		pppresence_instant_counter ++;

		if (pppresence_instant_counter >= PP_INSTANT_COUNT)	// PP_INSTANT_COUNT * 5ms = 100ms
			{
			pppresence_instant_counter = 0;
			pppresence_instant = 0;
			}
		}
	else
		pppresence_instant_counter = 0;

	if ((pppresence_filtered == 0) && (ppcurrent > 0))
		{
		if (pppresence_filtered_counter >= 20)	// 20 * 50ms = 100ms
			{
			pppresence_filtered_counter = 0;
			pppresence_filtered = PP_FILTERED;
			}
		else
			pppresence_filtered_counter ++;
		}
	else if ((pppresence_filtered == PP_FILTERED) && (ppcurrent == 0))
		{
		if (pppresence_filtered_counter >= 20)	// 20 * 50ms = 100ms
			{
			pppresence_filtered_counter = 0;
			pppresence_filtered = 0;
			}
		else
			pppresence_filtered_counter ++;
		}
	else
		pppresence_filtered_counter = 0;											/* END OF pppresence */
	}

plug_presence =  (cppresence_instant | cppresence_filtered | pppresence_instant | pppresence_filtered);
#else
plug_presence =  (cppresence_instant | cppresence_filtered);
#endif

}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	plug_presence_get
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			-
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
uint16_t plug_presence_get(void)
{
return plug_presence;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	plug_manager
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void plug_manager(uint8_t config)
{
uint8_t		socket_type, actuator_mode, plug_error, pers_evs_mode, plug_presence_pp;
uint16_t	plug_configuration;

eeprom_param_get(EVS_MODE_EADD, &pers_evs_mode, 1);
eeprom_param_get(SOCKET_TYPE_EADD, &socket_type, 1);

plug_presence_pp = 0;
plug_configuration = CPLOST_CRL1;

plug_configuration |= (config & PPLOST_CRL1);

if ((socket_type == SOCKET_T2_NO_LID) || (socket_type == SOCKET_T2_CLOSE_LID) || (socket_type == SOCKET_T2_OPEN_LID)
 || (socket_type == SOCKET_3C_OPEN_LID) || (socket_type == SOCKET_3C_NO_LID))
    {
    plug_presence_pp = 1;
    plug_configuration |= PPLOST_CRL1;
    }

plug_configuration |= (plug_configuration << 8);

eeprom_param_get(ACTUATORS_EADD, &actuator_mode, 1);
actuator_mode &= PAUT_ATT0;

switch (plug_state)
    {
	case PLUG_NOT_PRESENT:    // no plug detected
        {
    	if (plug_presence_pp == 0)
            {
            if (plug_presence & (PP_INSTANT | PP_FILTERED))
            	pplost_set = 0;
            else if (plug_presence == 0)
                {
            	if (pplost_set == 0)
                    {
                    gsy_quick_polling_update(PLUGGED_OUTLET, 0);
                    send_to_evs(EVS_PLUG_OUT);
                    }

            	pplost_set = 1;
                }

            if (plug_presence & (CP_INSTANT | CP_FILTERED))
                {
            	plug_presence_old = plug_presence;
            	plug_state = PLUG_DETECTED;
                
            	if (((actuator_mode & PAUT_ATT0) == 0x00) || (pers_evs_mode < EVS_NET_MODE) || (evs_state_get() == EVSTATE_PLUG_WAIT))
                    gsy_quick_polling_update(PLUGGED_OUTLET, 1);

            	send_to_evs(EVS_PLUG_DETECTED);
                }
            }
    	else if (plug_presence != 0)
            {
            plug_presence_old = plug_presence;
            plug_state = PLUG_DETECTED;
            
            if (((actuator_mode & PAUT_ATT0) == 0x00) || (pers_evs_mode < EVS_NET_MODE) || (evs_state_get() == EVSTATE_PLUG_WAIT))
                gsy_quick_polling_update(PLUGGED_OUTLET, 1);

        	send_to_evs(EVS_PLUG_DETECTED);
            }
        }
    	break;

	case PLUG_DETECTED:
        {
    	if ((plug_presence == 0) || (((plug_presence_pp == 0) || (evs_charging_resume_get() == 1)) && ((plug_presence & (CP_INSTANT | CP_FILTERED)) == 0)))
            {
        	if (plug_presence_old & (CP_INSTANT | CP_FILTERED))
                {
                if (evs_state_get() != EVSTATE_PLUG_OUT)
                    cplost_set = 1;
                }
        	else	// if (plug_presence_old & (PP_INSTANT | PP_FILTERED))
                {
                if (evs_state_get() != EVSTATE_PLUG_OUT)
                	pplost_set = 1;
                }

            evs_error_get(&plug_error, 1, 1, 6);

            if ((plug_error & (RCBO_ANOM0 | MIRROR_ANOM0)) == 0)
                gsy_quick_polling_update(PLUGGED_OUTLET, 0);

            plug_state = PLUG_NOT_PRESENT;
            send_to_evs(EVS_PLUG_OUT);
            }
    	else
            {
        	if (plug_presence > plug_presence_old)
            	plug_presence_old = plug_presence;

        	if ((plug_presence == plug_configuration) || ((plug_presence_pp == 0) && (plug_presence == 0x0C0C)))    // CPLOST_CRL1 | CPLOST_CRL1<<4
                {
            	plug_presence_old = plug_presence;
            	plug_state = PLUG_INSERTED;
                
            	if (((actuator_mode & PAUT_ATT0) == 0x00) || (pers_evs_mode < EVS_NET_MODE) || (evs_state_get() == EVSTATE_PLUG_WAIT))
                    gsy_quick_polling_update(PLUGGED_OUTLET, 1);

            	send_to_evs(EVS_PLUG_INSERTED);
                }
            }

        evs_charging_resume_reset();
        }
    	break;

	case PLUG_INSERTED:    // plug completely inserted according to plug_presence
        {
    	if (plug_presence != plug_configuration)
            {
        	if ((plug_configuration & CPLOST_CRL1) && ((plug_presence & (CP_INSTANT | CP_FILTERED)) != (CP_INSTANT | CP_FILTERED)))
                {
                if (evs_state_get() != EVSTATE_PLUG_OUT)
                	cplost_set = 1;
                }
        	else	// if ((plug_configuration & PPLOST_CRL1) && ((plug_presence & (PP_INSTANT | PP_FILTERED)) != (PP_INSTANT | PP_FILTERED)))
                {
                if (evs_state_get() != EVSTATE_PLUG_OUT)
                	pplost_set = 1;
                }

        	if (plug_presence == 0)
                {
                evs_error_get(&plug_error, 1, 1, 6);
                
                if ((plug_error & (RCBO_ANOM0 | MIRROR_ANOM0)) == 0)
                    gsy_quick_polling_update(PLUGGED_OUTLET, 0);
                
                plug_state = PLUG_NOT_PRESENT;
                send_to_evs(EVS_PLUG_OUT);
                }
        	else
                {
            	plug_presence_old = plug_presence;
            	plug_state = PLUG_DETECTED;
            	send_to_evs(EVS_PLUG_BACK);
                }
            }
        }
    	break;

	default:
    	break;
    }
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	s2_control_set
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void s2_control_set(uint8_t enable)
{
if (enable == 1)
    s2_enable = 1;
else
    s2_enable = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	s2_state_get
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
s2_state_en s2_state_get(void)
{
if (s2_state == S2_STATE_Bx)
    return S2_STATE_B;
else
    return s2_state;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	s2_init
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void s2_init(void)
{
s2_enable = 0;
s2A_counter = 0;
s2B_counter = 0;
s2C_counter = 0;
s2D_counter = 0;
s2_state_old = S2_STATE_A;
s2_state = S2_STATE_A;

s2_state_b_max_level = S2_STATE_B_MAX;
s2_state_b_min_level = S2_STATE_B_MIN;
s2_state_c_max_level = S2_STATE_C_MAX;
s2_state_c_min_level = S2_STATE_C_MIN;
s2_state_d_max_level = S2_STATE_D_MAX;
//s2_state_d_min_level = S2_STATE_D_MIN;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	s2_manager
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void s2_manager(void)
{
if (evs_iso15118_run_get() == 0)
    {
    s2_state_b_max_level = S2_STATE_B_MAX;
    s2_state_b_min_level = S2_STATE_B_MIN;
    s2_state_c_max_level = S2_STATE_C_MAX;
    s2_state_c_min_level = S2_STATE_C_MIN;
    s2_state_d_max_level = S2_STATE_D_MAX;
//    s2_state_d_min_level = S2_STATE_D_MIN;
    }

if (s2_enable == 0)
	s2_init();
else if (CPSN_VAL >= CP_SHORT_INF)
	{
	switch (s2_state)
		{
		case S2_STATE_A:
		    {
            s2C_counter = 0;
            s2D_counter = 0;
            
            if (CPSN_VAL <= S2_STATE_B_MAX)
                {
                s2B_counter ++;
                         
                if (s2B_counter >= 5)	// 5 * 5ms = 25ms
                    {
                    if (evs_iso15118_run_get() == 1)
                        {
                        s2Bx_counter = 20;
             	        s2_state = S2_STATE_Bx;
             	        }
                    else
             	        s2_state = S2_STATE_B;
             	    }
                }
            else
                s2B_counter = 0;
		    }
		    break;

		case S2_STATE_Bx:
		    {
            s2B_counter = 0;
            s2C_counter = 0;
            s2D_counter = 0;
            
            if (is_pwm_CP_Active() == TRUE)
                {
                if (CPSN_VAL <= S2_STATE_B_MIN)
                    {
                    s2Bx_counter ++;
                             
                    if (s2Bx_counter >= 20)	// 10 * 5ms
                        {
                        s2_state_b_max_level = (CPSN_VAL + 100);
                        s2_state_b_min_level = (CPSN_VAL - 100);
                        s2_state_c_max_level = (s2_state_b_min_level - 200);
                        
                        if (s2_state_c_max_level <= S2_STATE_C_MIN);
                            {
                            s2_state_c_min_level = (s2_state_c_max_level - 150);
                            s2_state_d_max_level = (s2_state_c_min_level - 150);
//                            s2_state_d_min_level = (s2_state_d_max_level - 150);
                            }

                        s2_state = S2_STATE_B;
                 	    }
                 	}
                else if (s2Bx_counter)
                    {
                    s2Bx_counter --;
                    
                    if (s2Bx_counter == 0)	// 10 * 5ms
                        s2_state = S2_STATE_B;
                    }
                }
		    }
		    break;

		case S2_STATE_B:
		case S2_STATE_C:
		case S2_STATE_D:
			{
			if (CPSN_VAL > CP_PRESENCE_SUP)
				{
				s2A_counter ++;

				if (s2A_counter >= 5)	// 5 * 5ms = 25ms
                   {
                   s2_state_b_max_level = S2_STATE_B_MAX;
                   s2_state_b_min_level = S2_STATE_B_MIN;
                   s2_state_c_max_level = S2_STATE_C_MAX;
                   s2_state_c_min_level = S2_STATE_C_MIN;
                   s2_state_d_max_level = S2_STATE_D_MAX;
//                   s2_state_d_min_level = S2_STATE_D_MIN;
				   s2_state = S2_STATE_A;
                   }
				}
	    	else
	    	    {
	    	    s2A_counter = 0;

    			if (CPSN_VAL < s2_state_d_max_level)
    				{
    				s2B_counter = 0;
    	    	    s2C_counter = 0;
    				s2D_counter ++;
    
    				if (s2D_counter >= 25)	// 25 * 5ms = 125ms
    					s2_state = S2_STATE_D;
    				}
    			else
    			    {
    			    s2D_counter = 0;
    
                    if (((s2_state > S2_STATE_C) && (CPSN_VAL >= S2_STATE_C_MIN))
                     || ((s2_state < S2_STATE_C) && (CPSN_VAL <= s2_state_c_max_level)))
        				{
        				s2B_counter = 0;
        				s2C_counter ++;
        
        				if (s2C_counter >= 25)	// 25 * 5ms = 125ms
        					s2_state = S2_STATE_C;
        				}
        			else
        			    {
        			    s2C_counter = 0;
        			    
                        if (((s2_state > S2_STATE_B) && (CPSN_VAL >= s2_state_b_min_level))
                         || ((s2_state < S2_STATE_B) && (CPSN_VAL <= s2_state_b_max_level)))
        				    {
        				    s2B_counter ++;
                            
        				    if (s2B_counter >= 5)	// 5 * 5ms = 25ms
        				    	s2_state = S2_STATE_B;
        				    }
        				else
        				    s2B_counter = 0;
        				}
        			}
    			}
			}
			break;

		default:
			break;
		}
	}
else
    {
	s2A_counter = 0;
	s2B_counter = 0;
	s2C_counter = 0;
	s2D_counter = 0;
    }

if (s2_state_old != s2_state)
	{
	s2A_counter = 0;
	s2B_counter = 0;
	s2C_counter = 0;
	s2D_counter = 0;
	send_to_evs(EVS_S2_STATE_UPDATE);
	}

s2_state_old = s2_state;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	rect_enable_set
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void rect_enable_set(uint8_t enable)
{
if (enable == 1)
    rect_enable = 1;
else
    rect_enable = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	rect_init
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void rect_init(void)
{
rect_state = RECT_NULL;
rect_counter = 0;
rect_enable = 0;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	rect_manager
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void rect_manager(uint8_t enable)
{
if (enable == 0)
	rect_init();
else
	{
	if (getInput(IN0_DIODO_ADC_PIN_UP) == GPIO_PIN_RESET)
		{
		if (rect_state != RECT_PRESENT)
			rect_counter ++;
		
		if (rect_counter >= 10)	// 10 * 5ms = 50ms
			{
			rect_counter = 0;
			rect_state = RECT_PRESENT;
			send_to_evs(EVS_RECT_UPDATE);
			}
		}
	else	// if (getInput(IN0_DIODO_ADC_PIN_UP) == GPIO_PIN_SET)
		{
		if (rect_state != RECT_ABSENT)
			rect_counter ++;
		
		if (rect_counter >= 10)	// 10 * 5ms = 50ms
			{
			rect_counter = 0;
			rect_state = RECT_ABSENT;
			send_to_evs(EVS_RECT_UPDATE);
			}
		}
	}
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	PilotManager_init
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static void PilotManager_init(void)
{
cp_init();
#ifndef HW_MP28947
pp_init();
#endif
s2_init();
rect_init();
xpshort_enable = 0;
xppresence_enable = 0;
xp_presence_freeze = 0;
plug_state = PLUG_NOT_PRESENT;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	PilotManager
//
//	DESCRIPTION:	-
//
//	INPUT:			-
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
static uint32_t PilotManager(PilotMngMsg_st *pMsg)
{
uint8_t     control_enable;
uint32_t    newTimeTick = pdMS_TO_TICKS(5);

CPSN_VAL = getADCmV(CP_ADC_IN);

#ifndef HW_MP28947
PPSN_VAL = getADCmV(PP_ADC_IN);
#endif

eeprom_param_get(CONTROL_BYTE1_EADD, &control_enable, 1);							// legge abilitazione dei controlli CP e PP in eeprom

xp_short_manager((control_enable & xpshort_enable));
//xp_presence_manager((control_enable & xppresence_enable));
xp_presence_manager(xppresence_enable);
plug_manager(control_enable);
s2_manager();
rect_manager(rect_enable);

return newTimeTick;
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
//	FUNCTION NAME:	PilotMngTask
//
//	DESCRIPTION:	Handle CP and PP line
//
//	INPUT:			none
//
//	OUTPUT:			none
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
void PilotMngTask(void *pvParameters)
{
uint32_t timeTick;

/* init task */

/*-------- Creates an empty mailbox for EvsMngTask messages --------------------------*/
PilotMngQueue = xQueueCreate(6, sizeof(PilotMngMsg_st));
configASSERT(PilotMngQueue != NULL);

PilotManager_init();
timeTick = portMAX_DELAY;

/* start task */

for (;;)
	{
	/* Wait for some event from SW */
	if (xQueueReceive(PilotMngQueue, (void *)&PilotMngMsg, timeTick) == pdPASS)
		{
                tPrintf("receive to pilot\n\r");
		timeTick = PilotManager(&PilotMngMsg);
		}
	else
		{
		/* Wait for possible handled timeout */
		PilotMngMsg.PilotMngEvent = PILOT_TIMEOUT;
		timeTick = PilotManager(&PilotMngMsg);
		}
	}
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------- //

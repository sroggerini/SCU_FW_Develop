/**
* @file        scuMdb.c
*
* @brief       Uart for RS485 SCU mobus communication  - Implementation -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: scuMdb.c 775 2025-07-14 11:32:59Z npiergi $
*
*     $Revision: 775 $
*
*     $Author: npiergi $
*
*     $Date: 2025-07-14 13:32:59 +0200 (lun, 14 lug 2025) $
*
*
* @copyright
*       Copyright (C) 2016 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

/************************************************************
 * Include
 ************************************************************/
//#include <main.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_rcc.h"
#else
#include "stm32h5xx_hal.h"
#include "stm32h5xx_ll_rcc.h"
#include "stm32h5xx_hal_uart_legacy.h"
#endif
#include "cmsis_os.h"
#include "prot_OnUsart.h"
#include "wrapper.h"
#include "sbcUart.h"
#include "scuMdb.h"
#include "eeprom.h"
#include "telnet.h"
#include "sbcGsy.h"
#include "uart_Legacy.h"
#include "inputsMng.h"
#include "rtcApi.h"
#include "scheduleMng.h"   
#include "httpserver-socket.h"
#include "ExtInpMng.h"
#include "RfidMng.h"
#include "sbcSem.h"
#include "ledMng.h"
#include "lcdMng.h"
// --------------------------------------------------------------------------------------------------------------------------- //

/*
***********************************SCAME**************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#define     TIMEOUT_RESP_500                ((uint16_t)500)

/* Macros for GD32F4xx microcontroller */

#ifdef GD32F4xx

#define _GET_SCU_RX_DMA_CNT         UART_SCU_DMA_STREAM->NDTR
#define _IS_SCU_RX_DMA_TC_FLAG      DMA2->HISR & DMA_HISR_TCIF5
#define _CLEAR_SCU_RX_DMA_TC_FLAG   DMA2->HIFCR = DMA_HISR_TCIF5

#define _REINIT_SCU_RX_DMA_CHANNEL(pSource)   UART_SCU_DMA->HIFCR = DMA_HISR_DMEIF5 | DMA_HISR_FEIF5 |  \
                                                                     DMA_HISR_HTIF5 | DMA_HISR_TCIF5 | DMA_HISR_TEIF5;  \
                                              UART_SCU_DMA_STREAM->M0AR = (uint32_t)pData;                                  /* Set memory address for DMA again */  \
                                              UART_SCU_DMA_STREAM->NDTR = NUM_BUFF_SBC_MSG_RX;                              /* Set number of bytes to receive */    \
                                              UART_SCU_DMA_STREAM->CR |= DMA_SxCR_EN;                                       /* Start DMA transfer */      
#else

/* Macros for STM32H5xx microcontroller */
                                              
#define _GET_SCU_RX_DMA_CNT         (UART_SCU_RX_DMA_STREAM->CBR1 & DMA_CBR1_BNDT_Msk)                                             
#define _IS_SCU_RX_DMA_TC_FLAG      (UART_SCU_RX_DMA_STREAM->CSR & DMA_CSR_HTF)                                             
#define _CLEAR_SCU_RX_DMA_TC_FLAG   (UART_SCU_RX_DMA_STREAM->CFCR |= DMA_CFCR_HTF)
                                              
#define _REINIT_SCU_RX_DMA_CHANNEL(pSource)   UART_SCU_RX_DMA_STREAM->CFCR = DMA_CFCR_TCF | DMA_CFCR_HTF | DMA_CFCR_DTEF |  \
                                                                             DMA_CFCR_USEF | DMA_CFCR_ULEF | DMA_CFCR_SUSPF | DMA_CFCR_TOF; \
                                              UART_SCU_RX_DMA_STREAM->CSAR = (uint32_t)pSource;      \
                                              UART_SCU_RX_DMA_STREAM->CBR1 = NUM_BUFF_SBC_MSG_RX;  \
                                              UART_SCU_RX_DMA_STREAM->CCR |= DMA_CCR_EN; 
                                                                                                                                         
#endif


// --------------------------------------------------------------------------------------------------------------------------- //

/*
***********************************SCAME**************************************
**                                                                          **
**                            Local Const                                   **
**                                                                          **
****************************************************************************** 
*/ 

   /*******   rispecchia rigorosamente l'enumerate scuAllRegIdx_e ADDR_RPM_POW_RDD ******************* */
static const scuRegInfo_st   scuRegister[SCU_LAST_REG_IDX]    = 
  {{ADDR_BOARD_FW_VERSION_RO, LEN_BOARD_FW_VERSION_RO, RO},                                       {ADDR_MIFARE_FW_VERSION_RO, LEN_MIFARE_FW_VERSION_RO, RO},                              {ADDR_BOARD_SN_RO, LEN_BOARD_SN_RO, RO},        
   {ADDR_PLC_FW_VERSION_RO, LEN_PLC_FW_VERSION_RO, RO},                                           {ADDR_PLC_MAC_ADDRESS_RO, LEN_PLC_MAC_ADDRESS_RO, RO},                                  {ADDR_HW_REVISION_RO, LEN_HW_REVISION_RO, RO},
   {ADDR_WIFI_MAC_ADDRESS_RO, LEN_WIFI_MAC_ADDRESS_RO, RO},                                       {ADDR_POWER_MODULE_SN_RO, LEN_POWER_MODULE_SN_RO, RO},                                  {ADDR_BLE_MAC_ADDRESS_RO, LEN_BLE_MAC_ADDRESS_RO, RO},
   {ADDR_PRODUCT_SN_RO, LEN_PRODUCT_SN_RO, RO},                                                   {ADDR_PRODUCT_CODE_RO, LEN_PRODUCT_CODE_RO, RO},                                        {ADDR_BOOT_FW_VERSION_RO, LEN_BOOT_FW_VERSION_RO, RO}, 
   {ADDR_MODBUS_VERSION_RO, LEN_MODBUS_VERSION_RO, RO},                                           {ADDR_SCU_RESERVED1, LEN_SCU_RESERVED1, RO},                                            {ADDR_NOTIFY_EVSE_PRESENCE_RO, LEN_NOTIFY_EVSE_PRESENCE_RO, RO},                        
   {ADDR_NOTIFY_EVSE_CHANGE_RO, LEN_NOTIFY_EVSE_CHANGE_RO, RO},                                   {ADDR_EVSE_ERROR1_RO, LEN_EVSE_ERROR1_RO, RO},                                          {ADDR_EVSE_ERROR2_RO, LEN_EVSE_ERROR2_RO, RO},                                          
   {ADDR_EVSE_CHANGE_REGISTERS_RO, LEN_EVSE_CHANGE_REGISTERS_RO, RO},                             {ADDR_SCU_RESERVED2, LEN_SCU_RESERVED2, RO},     
                                                 
/******          Registers for App local         *********************************************************************************************************************************************************** */   
   {ADDR_APP_EXCHANGE_AREA_RO, LEN_APP_EXCHANGE_AREA_RO, RO},  
   
   {ADDR_EVSE_EVENT_FLAGS_RO, LEN_EVSE_EVENT_FLAGS_RO, RO},                                       {ADDR_EVSE_CHARGE_STATUS_RO, LEN_EVSE_CHARGE_STATUS_RO, RO},                            
   {ADDR_TEMPERATURE_SYSTEM_RO, LEN_TEMPERATURE_SYSTEM_RO, RO},                                   {ADDR_SCU_RESERVED3, LEN_RESERVED3_RDD, RO},                                                    {ADDR_VOLTAGE_AC_RO, LEN_VOLTAGE_AC_RO, RO},                                            
   {ADDR_CURRENT_AC_L1_RO, LEN_CURRENT_AC_L1_RO, RO},                                             {ADDR_CURRENT_AC_L2_RO, LEN_CURRENT_AC_L2_RO, RO},                                              {ADDR_CURRENT_AC_L3_RO, LEN_CURRENT_AC_L3_RO, RO},                                      
   {ADDR_ACTIVE_POWER_DC_RO, LEN_ACTIVE_POWER_DC, RO},                                            {ADDR_ACTIVE_POWER_AC_RO, LEN_ACTIVE_POWER_AC_RO, RO},                                          {ADDR_ACTIVE_POWER_AC_L1_RO, LEN_ACTIVE_POWER_AC_L1_RO, RO},              
   {ADDR_REACTIVE_POWER_AC_RO, LEN_REACTIVE_POWER_AC_RO, RO},                                     {ADDR_TOTAL_ACTIVE_ENERGY_AC_EV_RO, LEN_TOTAL_ACTIVE_ENERGY_AC_EV_RO, RO},                      {ADDR_TOTAL_ACTIVE_ENERGY_DC_EV_RO, LEN_TOTAL_ACTIVE_ENERGY_DC_EV_RO, RO},              
   {ADDR_TOTAL_REACTIVE_ENERGY_AC_EV_RO, LEN_TOTAL_REACTIVE_ENERGY_AC_EV_RO, RO},                 {ADDR_TRANSACTION_ACTIVE_ENERGY_AC_EV_RO, LEN_TRANSACTION_ACTIVE_ENERGY_AC_EV_RO, RO},          {ADDR_ACTIVE_POWER_AC_L2_RO, LEN_ACTIVE_POWER_AC_L2_RO, RO},              
   {ADDR_TRANSACTION_REACTIVE_ENERGY_AC_EV_RO, LEN_TRANSACTION_REACTIVE_ENERGY_AC_EV_RO, RO},     {ADDR_FREQUENCY_SUPPLY_RO, LEN_FREQUENCY_SUPPLY_RO, RO},                                        {ADDR_SCU_RESERVED4, LEN_RESERVED4_RDD, RO},                                            
   {ADDR_MAX_CURRENT_ROTATORY_SWITCH_RO, LEN_MAX_CURRENT_ROTATORY_SWITCH_RO, RO},                 {ADDR_UNDEFINED_EX_RSC_CURR_RDD, LEN_UNDEFINED_EX_RSC_CURR_RDD, RO},                            {ADDR_ACTIVE_POWER_AC_L3_RO, LEN_ACTIVE_POWER_AC_L3_RO, RO},                    
   {ADDR_EMPTY_063A, LEN_EMPTY_063A, RO},                                                         {ADDR_MAX_CURRENT_RESULT_RO, LEN_MAX_CURRENT_RESULT_RO, RO},                                    {ADDR_TRANSACTION_TIME_RO, LEN_TRANSACTION_TIME_RO, RO},                                
   {ADDR_TRANSACTION_COUNTDOWN_RO, LEN_TRANSACTION_COUNTDOWN_RO, RO},                             {ADDR_EV_BATTERY_RO, LEN_EV_BATTERY_RO, RO},                                                    {ADDR_SESSION_ID_RO, LEN_SESSION_ID_RO, RO},                                            
   {ADDR_EVSE_BOOT_EVENT_RO, LEN_EVSE_BOOT_EVENT_RO, RO},                                         {ADDR_TOTAL_ACTIVE_ENERGY_AC_GRID_RO, LEN_TOTAL_ACTIVE_ENERGY_AC_GRID_RO, RO},                  {ADDR_TOTAL_REACTIVE_ENERGY_AC_GRID_RO, LEN_TOTAL_REACTIVE_ENERGY_AC_GRID_RO, RO},      
   {ADDR_TRANSACTION_ACTIVE_ENERGY_AC_GRID_RO, LEN_TRANSACTION_ACTIVE_ENERGY_AC_GRID_RO, RO},     {ADDR_TRANSACTION_REACTIVE_ENERGY_AC_GRID_RO, LEN_TRANSACTION_REACTIVE_ENERGY_AC_GRID_RO, RO},  {ADDR_SCU_RESERVED6, LEN_SCU_RESERVED6, RO},                                            
   {ADDR_UID_AUTHORIZATION_RO, LEN_UID_AUTHORIZATION_RO, RO},                                     {ADDR_TIME_IN_CHARGE_RO, LEN_TIME_IN_CHARGE_RO, RO},                                            {ADDR_SCU_RESERVED6B, LEN_SCU_RESERVED6B, RO},

   /****** end read-only registers, start RW registers ****************************************************************************************************************************************************** */   
   {ADDR_CONNECTOR_TYPE_RW, LEN_CONNECTOR_TYPE_RW, RW},                                         {ADDR_HWC_FLAGS_RW, LEN_HWC_FLAGS_RW, RW},                                              {ADDR_ENERGY_METERS_RW, LEN_ENERGY_METERS_RW, RW},                              
   {ADDR_POWER_OUTAGE_HW_RW, LEN_POWER_OUTAGE_HW_RW, RW},                                       {ADDR_CONNECTOR_IDS_RW, LEN_CONNECTOR_IDS_RW, RW},                                      {ADDR_BOARD_SN_RW, LEN_BOARD_SN_RW , RW},                                                    
   {ADDR_PRODUCT_SN_RW, LEN_PRODUCT_SN_RW, RW},                                                 {ADDR_MASTER_HARDWARE_TYPE_RW, LEN_MASTER_HARDWARE_TYPE_RW, RW},                        {ADDR_UTC_DATE_TIME_RW, LEN_UTC_DATE_TIME_RW, RW},                                           
   {ADDR_POWER_MODULE_BRAND_RW, LEN_POWER_MODULE_BRAND_RW, RW},                                 {ADDR_POWER_MODULE_MODEL_RW, LEN_CHARGE_PROTOCOL_SELECTED_RW, RW},                      {ADDR_POWER_MODULE_N_RW, LEN_POWER_MODULE_N_RW, RW},                                         
   {ADDR_EVSE_TM_RW, LEN_EVSE_TM_MODE_RW, RW},                                                  {ADDR_EVSE_OPERATION_MODE_RW, LEN_EVSE_OPERATION_MODE_RW, RW},                          {ADDR_CHARGE_PROTOCOL_SELECTED_RW, LEN_CHARGE_PROTOCOL_SELECTED_RW, RW},                     
   {ADDR_DISPLAY_DEFAULT_LANGUAGE_RW, LEN_DISPLAY_DEFAULT_LANGUAGE_RW, RW},                     {ADDR_HW_CHECKS1_RW, LEN_HW_CHECKS1_RW, RW},                                            {ADDR_HW_CHECKS2_RW, LEN_HW_CHECKS2_RW, RW},
   {ADDR_HW_ACTUATORS_RW, LEN_HW_ACTUATORS_RW, RW},                                             {ADDR_SCU_RESERVED7, LEN_SCU_RESERVED7, RW},                                            {ADDR_MAX_TEMPORARY_CURRENT_RW, LEN_MAX_TEMPORARY_CURRENT_RW, RW},
   {ADDR_LEN_EMPTY_0033_RW_RWD, LEN_EMPTY_0033_RW, RW},                                         {ADDR_LED_STRIP_RW, LEN_LED_STRIP_RW, RW},                                              {ADDR_REMOTE_COMMANDS_RW, LEN_REMOTE_COMMANDS_RW, RW},
   {ADDR_ENVIRONMENT_BRIGHTNESS_RW, LEN_ENVIRONMENT_BRIGHTNESS_RW, RW},                         {ADDR_MIN_POWER_DC_RWD, LEN_MIN_POWER_DC_RWD, RW},                                      {ADDR_UNDEFINED_EX_PRX_MOTION_RWD, LEN_UNDEFINED_EX_PRX_MOTION_RWD, RW},                
   {ADDR_MAX_TEMPORARY_POWER_AC, LEN_MAX_TEMPORARY_POWER_AC, RW},                               {ADDR_PM_MODE_RW, LEN_PM_MODE_RW, RW},                                                  {ADDR_ENABLE_LOG_RW, LEN_ENABLE_LOG_RW, RW},                                            
   {ADDR_AUTHORIZATION_FEEDBACK_RW, LEN_AUTHORIZATION_FEEDBACK_RW, RW},                         {ADDR_TIMEZONE_RW, LEN_TIMEZONE_RW, RW},                                                {ADDR_MENU_VISIBILITY_RW, LEN_MENU_VISIBILITY_RW, RW},              
   {ADDR_TIMEOUT_RANGE1_RW, LEN_TIMEOUT_RANGE1_RW, RW},                                         {ADDR_TIMEOUT_RANGE2_RW, LEN_TIMEOUT_RANGE2_RW, RW},                                    {ADDR_PRODUCT_CODE_RW, LEN_PRODUCT_CODE_RW, RW},                                        
   {ADDR_TECN_PRODUCT_CODE_RW, LEN_TECN_PRODUCT_CODE_RW, RW},                                   {ADDR_CONNECTOR_N_RW, LEN_CONNECTOR_N_RW, RW},                                          {ADDR_DISPLAY_LANGUAGES_RW, LEN_DISPLAY_LANGUAGES_RW, RW},                              
   {ADDR_MAX_TYPICAL_CURRENT_RW, LEN_MAX_TYPICAL_CURRENT_RW, RW},                               {ADDR_MAX_SIMPLIFIED_CURRENT_RW, LEN_MAX_SIMPLIFIED_CURRENT_RW, RW},                    {ADDR_CHARGE_TIME_RW, LEN_CHARGE_BY_TIME_RW, RW},                                          
   {ADDR_CHARGE_MAX_ENERGY_RW, LEN_CHARGE_BY_ENERGY_RW, RW},                                    {ADDR_PM_IMIN_RW, LEN_PM_IMIN_RW, RW},                                                  {ADDR_PM_PMAX_RW, LEN_PM_PMAX_RW, RW},                                                       
   {ADDR_PM_FLAGS_RW, LEN_PM_FLAGS_RW, RW},                                                     {ADDR_PM_HPOWER_RW, LEN_PM_HPOWER_RW, RW},                                              {ADDR_PM_DSET_RW, LEN_PM_DSET_RW, RW},                                                       
   {ADDR_PM_DMAX_RW, LEN_PM_DMAX_RW, RW},                                                       {ADDR_PIN_EXTERNAL_ENERGY_METER_RW, LEN_PIN_EXTERNAL_ENERGY_METER_RW, RW},              {ADDR_LIM_MAX_POWER_AC_RW, LEN_LIM_MAX_POWER_AC_RW, RW}, 
   {ADDR_RESERVED_A, LEN_RESERVED_A, RW},
   /******  read-write registers for download   ****************************************************************************************************************************************************** */   
   {ADDR_FILE_COMMAND_RW, LEN_FILE_COMMAND_RW, RW},                                             {ADDR_FILE_SIZE_RW, LEN_FILE_SIZE_RW, RW},                                              {ADDR_FILE_PACKET_RW, LEN_FILE_PACKET_RW, RW}, 

   /*  ***********************************         start RSE-SINAPSI  registers ****************************************************************************************************************************************************** */   
   {ADDR_RSE_IOM_CTRL_REG, LEN_IOM_CTRL_REG, RW},                                               {ADDR_RSE_M2_POT_ATT_IMM_IST_TS, LEN_M2_POT_ATT_IMM_IST_TS, RW},                        {ADDR_RSE_M2_POT_ATT_IMM_IST, LEN_M2_POT_ATT_IMM_IST, RW},   
   {ADDR_RSE_M2_POT_CONTRATTUALE_TS, LEN_M2_POT_CONTRATTUALE_TS, RW},                           {ADDR_RSE_M2_POT_CONTRATTUALE, LEN_M2_POT_CONTRATTUALE, RW},                            {ADDR_RSE_M1_POT_CONTRATTUALE_TS, LEN_M1_POT_CONTRATTUALE_TS, RW},   
   {ADDR_RSE_M1_POT_CONTRATTUALE, LEN_M1_POT_CONTRATTUALE, RW},                                 {ADDR_RSE_M1_POT_DISPONIBILE_TS, LEN_M1_POT_DISPONIBILE_TS, RW},                        {ADDR_RSE_M1_POT_DISPONIBILE, LEN_M1_POT_DISPONIBILE, RW},   
   {ADDR_RSE_M1_POT_ATT_PRE_IST_TS, LEN_M1_POT_ATT_PRE_IST_TS, RW},                             {ADDR_RSE_M1_POT_ATT_PRE_IST, LEN_M1_POT_ATT_PRE_IST, RW},                              {ADDR_RSE_M1_POT_ATT_IMM_IST_TS, LEN_M1_POT_ATT_IMM_IST_TS, RW},   
   {ADDR_RSE_M1_POT_ATT_IMM_IST, LEN_M1_POT_ATT_IMM_IST, RW},                                   {ADDR_RSE_M1_RES_DISTACCO_TS, LEN_M1_DISTACCO_TS, RW},                                  {ADDR_RSE_M1_TEMPO_RES_DIST, LEN_M1_TEMPO_RES_DIST, RW},   
   {ADDR_RSE_M1_PROF_FASCIA_ORARIA_TS, LEN_M1_PROF_FASCIA_ORARIA_TS, RW},                       {ADDR_RSE_M1_PROF_FASCIA_ORARIA, LEN_M1_PROF_FASCIA_ORARIA, RW},                        {ADDR_RSE_M1_POT_MAX_RIC_TLIM, LEN_M1_POT_MAX_RIC_TLIM, RW},   
   {ADDR_RSE_M1_POT_TEMPO_LIMITE_TS, LEN_M1_POT_TEMPO_LIMITE_TS, RW},                           {ADDR_RSE_M1_SOSP_RIC_TLIM_TS, LEN_M1_SOSP_RIC_TLIM_TS, RW},                            {ADDR_RSE_M1_UNIXTIME_SYNC, LEN_M1_UNIXTIME_SYNC, RW},
   {ADDR_RSE_SET_CONFIG_MODE, LEN_SET_CONFIG_MODE, RW},

   /*  ***********************************         start PEN Test  registers ****************************************************************************************************************************************************** */   
   {ADDR_PEN_RELE_REG, LEN_PEN_RELE_RWD, RW},   

   /*  ***********************************         start MACCHINA DI COLLAUDO  registers ****************************************************************************************************************************************************** */   
   {ADDR_TM_MEASURED_CURRENT_RW, LEN_TM_MEASURED_CURRENT_RW, RW},                               {ADDR_TM_MEASURED_VOLTAGE_RW, LEN_TM_MEASURED_VOLTAGE_RW, RW},                         {ADDR_TM_EVSE_READY_RO, LEN_TM_EVSE_READY_RO, RO},
   {ADDR_ADDR_S_CONN_RW, LEN_TM_ADDR_S_CONN_RW, RW},                                            {ADDR_TM_ADDR_REQ_RW, LEN_TM_ADDR_REQ_RW, RW},                                         {ADDR_TM_ADDR_ASS_RW, LEN_TM_ADDR_ASS_RW, RW},
   {ADDR_TM_ERROR1_RO, LEN_ADDR_TM_ERROR1_RO, RW},                                              {ADDR_TM_UID_MASTER_RW, LEN_TM_UID_MASTER_RW, RW},                                     {ADDR_TM_UID_USER_RW, LEN_TM_UID_USER_RW, RW},
   {ADDR_TM_MEASURED_CURRENT_L2_RW, LEN_TM_MEASURED_CURRENT_L2_RW, RW},                         {ADDR_TM_MEASURED_VOLTAGE_L2_RW, LEN_TM_MEASURED_VOLTAGE_L2_RW, RW},                   {ADDR_TM_MEASURED_CURRENT_L3_RW, LEN_TM_MEASURED_CURRENT_L3_RW, RW},                         
   {ADDR_TM_MEASURED_VOLTAGE_L3_RW, LEN_TM_MEASURED_VOLTAGE_L3_RW, RW},                         {ADDR_GET_EEPROM_RW, LEN_TM_GET_EEPROM_RW, RW},

   /*  ***********************************         start APP - WIFI  registers ****************************************************************************************************************************************************** */   
   {ADDR_CONNECTOR_INIT_STATUS_RO, LEN_CONNECTOR_INIT_STATUS_RO, RO},                           {ADDR_CONNECTOR_OPERATION_STATUS_RO, LEN_CONNECTOR_OPERATION_STATUS_RO, RO},           {ADDR_PROCEDURE_RESPONSE_RO, LEN_PROCEDURE_RESPONSE_RO, RO},                           
   {ADDR_PM_TOTAL_POWER_RO, LEN_PM_TOTAL_POWER_RO, RO},                                         {ADDR_PM_L1_POWER_RO, LEN_PM_L1_POWER_RO, RO},                                         {ADDR_PM_L2_POWER_RO, LEN_PM_L2_POWER_RO, RO},                                         
   {ADDR_PM_L3_POWER_RO, LEN_PM_L3_POWER_RO, RO},                                               {ADDR_APP_LOGIN_USER_TYPE_RO, LEN_APP_LOGIN_USER_TYPE_RO, RO},                         {ADDR_CONNECTOR_CHECK_ACTIV_KEY_RW, LEN_CONNECTOR_CHECK_ACTIV_KEY_RW, RW},                   
   {ADDR_CONNECTOR_NAME_RW, LEN_CONNECTOR_NAME_RW, RW},                                         {ADDR_CONNECTOR_PIN_RW, LEN_CONNECTOR_PIN_RW, RW},                                     {ADDR_ROUTER_NET_SSID_RW, LEN_ROUTER_NET_SSID_RW, RW},                                 
   {ADDR_ROUTER_NET_PASSWORD_RW, LEN_ROUTER_NET_PASSWORD_RW, RW},                               {ADDR_CONNECTOR_ROUTER_IP_RW, LEN_CONNECTOR_ROUTER_IP_RW, RW},                         {ADDR_PROCEDURE_REQUEST_RW, LEN_PROCEDURE_REQUEST_RW, RW},                             
   {ADDR_SCHEDULE_FLAG1_RW, LEN_SCHEDULE_FLAG1_RW, RW},                                         {ADDR_SCHEDULE_START1_RW, LEN_SCHEDULE_START1_RW, RW},                                 {ADDR_SCHEDULE_STOP1_RW, LEN_SCHEDULE_STOP1_RW, RW},                                   
   {ADDR_SCHEDULE_POWER1_RW, LEN_SCHEDULE_POWER1_RW, RW},                                       {ADDR_SCHEDULE_FLAG2_RW, LEN_SCHEDULE_FLAG2_RW, RW},                                   {ADDR_SCHEDULE_START2_RW, LEN_SCHEDULE_START2_RW, RW},                                       
   {ADDR_SCHEDULE_STOP2_RW, LEN_SCHEDULE_STOP2_RW, RW},                                         {ADDR_SCHEDULE_POWER2_RW, LEN_SCHEDULE_POWER2_RW, RW},                                 {ADDR_SCHEDULE_FLAG3_RW, LEN_SCHEDULE_FLAG3_RW, RW},                                         
   {ADDR_SCHEDULE_START3_RW, LEN_SCHEDULE_START3_RW, RW},                                       {ADDR_SCHEDULE_STOP3_RW, LEN_SCHEDULE_STOP3_RW, RW},                                   {ADDR_SCHEDULE_POWER3_RW, LEN_SCHEDULE_POWER3_RW, RW},
   {ADDR_SCHEDULE_FLAG4_RW, LEN_SCHEDULE_FLAG4_RW, RW},                                         {ADDR_SCHEDULE_START4_RW, LEN_SCHEDULE_START4_RW, RW},                                 {ADDR_SCHEDULE_STOP4_RW, LEN_SCHEDULE_STOP4_RW, RW},                                          
   {ADDR_SCHEDULE_POWER4_RW, LEN_SCHEDULE_POWER4_RW, RW},                                       {ADDR_CONNECTOR_CONNECTION_STATUS_RO, LEN_CONNECTOR_CONNECTION_STATUS_RO, RO},         {ADDR_MAX_POWER_RO, LEN_MAX_POWER_RO, RO}, 
   {ADDR_RANDOM_DELAY_FLAG_RW, LEN_RANDOM_DELAY_FLAG_RW, RW},                                   {ADDR_UID_AUTHORIZATION_APP_RW, LEN_UID_AUTHORIZATION_APP_RW, RW},                     {ADDR_BT_SINAPSI_STATUS_RW, LEN_BT_SINAPSI_STATUS_RW, RW},
   {ADDR_FW_UPDATE_NAME_RW, LEN_FW_UPDATE_NAME_RW, RW},                                         {ADDR_FW_UPDATE_LEN_RW, LEN_FW_UPDATE_LEN_RW, RW},                                     {ADDR_FW_UPDATE_STATUS_RO, LEN_FW_UPDATE_STATUS_RO, RO},
   {ADDR_EVSE_POWER_MODE_RO, LEN_EVSE_POWER_MODE_RO, RO},

   /*  ***********************************         start LOVATO EM SIMULATOR   registers ****************************************************************************************************************************************************** */   
   {ADDR_EM_LOVATO_V_SYS, LEN_EM_LOVATO_V_SYS, RO},                                             {ADDR_EM_LOVATO_P_AREA, LEN_EM_LOVATO_P_AREA, RO}  
   };

static const scuOpModes_e  scuOpModeDef = {SCU_MODE_UNDEF}; 

// --------------------------------------------------------------------------------------------------------------------------- {ADDR_LIM_POWER_RWD, LEN_LIM_POWER_RWD, RW}, //

/*
***********************************SCAME**************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 
static frameScu_st        frameScuRx;
static frameScu_st        frameScuSemRx;
static frameScu_st        scuSemTxMsg;
static frameScuRx_st      frameScuDlRx, scuMsgRx_IT;
static frameSbcRx_st      tmpFrameSbcRx;
static scuInfoMng_st      scuInfoMng;
static scuInfoMng_st      scuSemInfoMng;
static scuDlInfoMng_st    scuDlInfoMng;
static microId_t          STMMicroID;
static endianessType_e    defEndian;


static scuRoMapRegister_st      scuMapRoRegister[SCU_NUM]; 
static scuRwMapRegister_st      scuMapRwRegister[SCU_NUM]; 
static tmMapRegister_st         tmMapRegister[SCU_NUM];
#ifdef MODBUS_TCP_EM_LOVATO
static scuMapRegLovatoEmSim_st  scuMapRegLovatoEmSim;
#endif

static uint8_t*             pMallocTx485;

static uint32_t             noiseErr;

static uint8_t              illegalAddress;

struct DataAndTime_t        utcTimeScu;

static uint8_t              uart1ReinitCounter;

static uint8_t              collaudoRunning, configParamDone;

/*
***********************************SCAME**************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 

scuOpModes_e            scuOpMode;
scuTypeModes_e          scuTypeModes;
scuAddModeSktNum_e      scuAddressMode;
uint8_t                 scuAddr, numPrdSocket;
uint32_t                numRS485Errors;
frameMdbRx_st*          pMsgShared;
uint16_t                frameLength;
uint8_t                 InitModbusRegisters_Stop = FALSE;

appMapRwRegister_st     appMapRwRegister[SCU_NUM];
appMapRoRegister_st     appMapRoRegister[SCU_NUM];
appMapRwRegister_st*    pAppRwRegs;

uint32_t                bufferLevel;
__no_init uint16_t      keyForEmError;
__no_init uint16_t      retryForEmError;

// --------------------------------------------------------------------------------------------------------------------------- //

/*
***********************************SCAME**************************************
**                                                                          **
**                            Local  Variables                              **
**                                                                          **
****************************************************************************** 
*/ 

TimerHandle_t     xMdbUartTimers;
TimerHandle_t     xSbcScuTimers;
TimerHandle_t     xSlaveScuTimers;

// --------------------------------------------------------------------------------------------------------------------------- //

/*
***********************************SCAME**************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
static uint32_t         scuMsgProcess                 (frameScu_st* pMsg);
static void             uartScuError_Handler          (char * file, int line);
//static xQueueHandle   getScuAnswerQueueHandle       (void);
static scuAllRegIdx_e   getIndexFromAddress           (uint16_t addr);
static uint32_t         scuDlProcess                  (frameScuRx_st* pMsg);
static void             scuSemMsgProcess              (frameScu_st* pMsg);
static regStartInfo_t*  findBaseAddressInMap          (uint16_t regAddr, uint16_t idScu, uint8_t function, uint8_t wordLen, endianessType_e *pEndian);
static void             sendSemRegister               (uint16_t regAddr, uint16_t size, uint8_t lpoAddr, regStartInfo_t* pInfoBase, uint8_t function, endianessType_e endianess, scuEvents_e ev);
static uint16_t         setScuRegister                (uint16_t regAddr, uint16_t size, uint8_t lpoAddr, regStartInfo_t* pInfoBase, uint8_t* pDataSrc, endianessType_e endianess);
static void             setToScuRegister              (scuRWmultipleReg_st* pScuRWmR);
static void             findScuObject                 (uint16_t addrMdb, uint16_t iScu, regScuAddr_t* pScuObj, scuEvents_e originEvent);
static void             mdbUartTimCallBack            (TimerHandle_t pxTimer);
static uint8_t          isThisMsgForUs                (uint8_t addrId, uint16_t modbusAddr);
static void             startSbcGardTimeConn          (scuOpModes_e locScuOpMode);
static void             SbcScuTimCallBack             (TimerHandle_t pxTimer);
static uint8_t          checkSlaveDwldRemoteScu       (uint16_t rAddr);
static void             restartTimerToResetRs485Uart  (void);

// --------------------------------------------------------------------------------------------------------------------------- //

/*
***********************************SCAME**************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 
/*  queue  declaration */
xQueueHandle scuQueue = NULL;
xQueueHandle scuDataLink = NULL;
xQueueHandle scuSemQueue = NULL;
xQueueHandle scuUartRxTimeoutQueue = NULL;
osSemaphoreId uartScuInitSemaphoreHandle, uartScuSinapsiTxSemaphoreHandle;



// --------------------------------------------------------------------------------------------------------------------------- //

/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/
extern uint32_t   counterSlaveDwnl;

#ifdef GD32F4xx
extern DMA_HandleTypeDef hdma_usart1_tx;
extern statusFlag_e      fastBridgeStatus;
#endif

// --------------------------------------------------------------------------------------------------------------------------- //

/*
***********************************SCAME**************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/



/*
***********************************SCAME**************************************
**                                                                          **
**                        External Function Definition                      **
**                                                                          **
******************************************************************************
*/

extern void uSecWwaitTime (uint16_t usDelay);
extern void UART_SCU_Reactivate_Rx(void);
extern GPIO_PinState GPIO_Read_IN3_MIRROR (void);

#ifdef GD32F4xx

/**
  * @brief Manage the UART TIMeout functionality not present in GD32F4xx device.
  * @param pvParameters
  * @retval None
  */

void scuUartRxTimeoutTask (void * pvParameters)
{
  
  uint32_t              timeTick;
  
  /*-------- Creates an empty mailbox for uart  messages --------------------------*/
  scuUartRxTimeoutQueue = xQueueCreate(NUM_BUFF_SCU_RX_TIMEOUT, sizeof(frameScu_st));
  configASSERT(scuUartRxTimeoutQueue != NULL);
  
  /* We want this queue to be viewable in a RTOS kernel aware debugger, so register it. */
  vQueueAddToRegistry(scuUartRxTimeoutQueue, "SCU_UART_Rx_QUEUE" );
  
  //timeTick = pdMS_TO_TICKS(TIMER_TICK_500);
  timeTick = portMAX_DELAY; 

  for (;;)
  {
    /* Wait for some event from Rx uart SCU (UART1)  */
    if (xQueueReceive(scuUartRxTimeoutQueue, (void *)&frameScuDlRx, timeTick) == pdPASS)
    {
        /* Refresh the timer everytime the IDLE condition is detected */
        if (frameScuDlRx.headerScuRx.dlEvent == SCU_DL_RX_IDLE)
          timeTick = pdMS_TO_TICKS(UART_RX_TIMEOUT);   
    }
    else
    {
        /* --------------------------- Timeout condition detected ----------------------------------------*/
        timeTick = portMAX_DELAY;                     /* stop the timer */
        UART_SCU_RX_DMA_STREAM->CR &= ~DMA_SxCR_EN;   /* Disabling DMA will force transfer complete interrupt if enabled */        
    }
  }
}

/**
  * @brief Get the queue handle for Uart Rx timeout management
  * @param  None
  * @retval None
  */

xQueueHandle getScuUartRxTimeoutQueueHandle (void)
{
    return(scuUartRxTimeoutQueue); 
}

#endif

void scuGestTask (void * pvParameters)
{
  uint32_t       timeTick;
  uint8_t        tmpVal;

  /*-------- Creates an empty mailbox for uart  messages --------------------------*/
  scuQueue = xQueueCreate(NUM_BUFF_SCU_RX, sizeof(frameScu_st));
  configASSERT(scuQueue != NULL);
  
  /* We want this queue to be viewable in a RTOS kernel aware debugger, so register it. */
  vQueueAddToRegistry(scuQueue, "scuQueueRx" );

  /* init structure for management */
  scuInfoMng.scuStates = SCU_IDLE;
  // Leggo l'ID del microprocessore e lo salvo
  STMMicroID.microId1 = HAL_GetUIDw0();
  STMMicroID.microId2 = HAL_GetUIDw1();
  STMMicroID.microId3 = HAL_GetUIDw2();
  /* calcolo il crc. LSB sarà usato come ritardo casuale in caso di conflitti sul bus e come indirizzo temporaneo */
  scuInfoMng.crcCpuId = crcEvaluation ((uint8_t *)&STMMicroID, (uint16_t)sizeof(STMMicroID));

  /* set initial value for buffer used to transmit over UART1 for SINIPSI and RS485 */
  pMallocTx485 = NULL;
#ifdef COME_ERA
  ignoreErrors = (uint8_t)FALSE;
#endif
  
  timeTick = pdMS_TO_TICKS(TIMEOUT_RESP_500);
  //timeTick = portMAX_DELAY;

  if (scuOpModeDef == SCU_MODE_UNDEF)
  {
    /* SCU_EMUMAX0_S: SCU starts in emulation slave MAX0 --> from USART1=RS485 bus there is a direct connection with GSY  */
    /* SCU_M_P: SCU starts in SCU principal --> from USART5 direct connection with SBC (SEM)  */
    /* SCU_S_S: SCU starts in SCU secondary --> connected to principal SCU  */
    // scuOpMode = SCU_M_P; 
    // scuAddr = SCU_M_P_ADDR;
    /*** without SEM the choise is possibile cheking the SBC presence: With SEM this info is not enough --> to do somethink  ***/
    osDelay(100); /* wait 100msec for setup time 3V3 for SBC 20msec is the time from DC/DC to give 3V3 to SBC */
    /***** the RS485 autoaddres has been moved to rs485AutoAddress() in eeprom.c *****/
    /* set SCU operative type GSY / SEM */
    eeprom_param_get(SEM_FLAGS_CTRL_EADD, (uint8_t *)&tmpVal, 1);
    scuAddressMode = (scuAddModeSktNum_e)(tmpVal & SCU_ADDR_MODE_MASK);
    numPrdSocket = ((tmpVal & SCU_SKT_NUM_MASK) >> 1);
    if (scuTypeModes == SCU_TYPE_UNDEF)
    {
      /* this is the case when upload FW from 4.2.x to 4.3.x In this location 0xFF is present */
      scuTypeModes = SCU_GSY;
      // xx eeprom_array_set(OPERATIVE_MODE_EADD, (uint8_t*)&scuTypeModes, 1);
      EEPROM_Save_Config (OPERATIVE_MODE_EADD, (uint8_t*)&scuTypeModes, 1);
      /* Write configurations in eeprom array */
      eeprom_ProductConfig_Param_Set();
    }
    activeSemTask();  // check if is possible to activate task Sem
#ifndef DEBUG_TRACE_PIN   
    /* in V4.3.x we dont't use this pin as semaphore for moroe block movement */
    gpio0IoExpRead();
    gpio1IoExpRead();
#endif
    
    startLcdTask();  // here the EEPROM has been read 

    if (scuTypeModes == SCU_GSY)
    {
      noiseErr = USART_FLAG_NE;
      if (sbcPresence())
      {
        scuOpMode = SCU_EMUMAX0_M;
      }
      else
      {
        scuOpMode = SCU_EMUMAX0_S;
      }
    }
    else
    {
      noiseErr = 0;
      if (sbcPresence())
      {
        gsy_connected_set((uint8_t)1); // the SEM is present 
#ifdef DA_VALUTARE
        gpio1IoExpWrite(GPIO_PIN_SET);
        (void)gpio0IoExpRead();
#endif
        if (scuTypeModes == SCU_SEM_M) 
        {
          scuOpMode = SCU_M_P;  // SEM master principale
        }
        else if (scuTypeModes == SCU_SEM_STAND_ALONE)
        {
          scuOpMode = SCU_M_STAND_ALONE; // SEM master in stand alone mode --> outside of the wallbox or pillar          
        }
        else 
        {
          scuOpMode = SCU_S_P; // SEM master secondaria
        }
      }
      else
      {
        gsy_connected_set((uint8_t)1); // the SEM is present Verificare se sia il caso di attendere il primo messaggio su Rs485 
        scuOpMode = SCU_S_S; // SEM slave secondaria
      }
      osDelay(200);
      startSbcSemProcess(); 
    }

    startSbcGardTimeConn(scuOpMode); /* start timeout on uart5 connection SCU <--> SBC and check serial OK in stand-alone SCU */

    /* set the right baude rate for RS485 */
    reInitScuUart();

    /*  SINAPSI-RSE */
    if (getSinapsiEepromEn() == ENABLED)  
    {
      setSinapsiStatusForApp(SCU_SINAPSI_PRESENT); 
    }
    else
    {
      setSinapsiStatusForApp(SCU_SINAPSI_NOT_PRESENT);
    }
  }
  defEndian = LITTLE_ENDIANESS; 

      

  for (;;)
  {
    /* Wait for some event from Rx/Tx uart SBC (typically UART5)  */
    if (xQueueReceive(scuQueue, (void *)&frameScuRx, timeTick) == pdPASS)
    {
      timeTick = scuMsgProcess(&frameScuRx);
    }
    else
    {
      if (scuInfoMng.scuStates == SCU_IDLE)
      {
        frameScuRx.scuEvents = SCU_EVENT_START;
      }
      else
      {
        frameScuRx.scuEvents = SCU_EVENT_MSG_NO_ANSWER;
      }
      timeTick = scuMsgProcess(&frameScuRx);
    }
  }
}

/**
*
* @brief        Decoder message coming from SBC on UART5, or for internal task as debug 
*
* @param [in]   none
*
* @retval       uint8_t: TRUE, if an answer must be sent
*
***********************************************************************************************************************/
static uint32_t scuMsgProcess(frameScu_st* pMsg)
{
  uint16_t        ixScu, rAddr;
  uint8_t*        pDest;
  frameMdbRx_st*  pRxFrame;
  uint32_t        timeTickTmp;

 timeTickTmp = portMAX_DELAY;

  switch (scuInfoMng.scuStates)
  {
    case SCU_IDLE:
      if(pMsg->scuEvents == SCU_EVENT_START)
      {
        /* start low level interface */
        scuInfoMng.frameScuDlRxTmp.dlEvent = SCU_DL_START;
        configASSERT(xQueueSendToBack(getScuDataLinkQueueHandle(), (void *)&scuInfoMng.frameScuDlRxTmp, portMAX_DELAY) == pdPASS);  // SEM --> scuDlProcess()
        scuInfoMng.scuStates = SCU_OPERATIVE;
      }
      break;

    case SCU_OPERATIVE:
      switch (pMsg->scuEvents)
      {
        case SCU_EVENT_MSG_INTERNAL_RD:
          /* set index register, size for th*/
          scuInfoMng.frameScuDlRxTmp.idSCU = ixScu = pMsg->idSCU; 
          if (pMsg->reqRegAddr == ADDR_UNDEF)
          {
            if (pMsg->scuRegIdx < SCU_LAST_REG_IDX)
            {
              rAddr = scuRegister[pMsg->scuRegIdx].regAdd;
            }
            else
            {
              tPrintf("Wrong read command!\n\r");
              return (SCU_ADDRESS_ERROR);
            }
          }
          else
          {
            rAddr = pMsg->reqRegAddr;
            pMsg->scuRegIdx = getIndexFromAddress(rAddr);
          } 
          if ((rAddr > ADDR_RESERVED_C)) 
          {
            return (SCU_ADDRESS_ERROR);
          }
          scuInfoMng.frameScuDlRxTmp.scuRegIdx = pMsg->scuRegIdx;
          scuInfoMng.frameScuDlRxTmp.reqRegAddr = rAddr;
          scuInfoMng.frameScuDlRxTmp.totalLen = pMsg->totalLen;
          scuInfoMng.frameScuDlRxTmp.scuDlOp = SCU_DL_OP_RD;

          scuInfoMng.frameScuDlRxTmp.dlEvent = SCU_DL_SEND;
          configASSERT(xQueueSendToBack(getScuDataLinkQueueHandle(), (void *)&scuInfoMng.frameScuDlRxTmp, portMAX_DELAY) == pdPASS); // scuDataLinkTask() --> scuDlProcess()
          /* change state and wait answer */
          scuInfoMng.scuStates = SCU_REQ_INT_RD_WR;
          break;

        default:
          break;

      }
      break;

    case SCU_REQ_INT_RD_WR:
      switch (pMsg->scuEvents)
      {
        case SCU_EVENT_RD_REQ_WR_ACK:
          if ((scuInfoMng.frameScuDlRxTmp.idSCU == pMsg->idSCU) && (scuInfoMng.frameScuDlRxTmp.reqRegAddr == pMsg->reqRegAddr))
          {
            pRxFrame = (frameMdbRx_st*)pMsg->pMessage;
            ixScu = pMsg->idSCU;
            switch (pRxFrame->message.headerAnwsRIR.function)
            {
              case FUNCTION_READ_HOLDING_REG:
              case FUNCTION_READ_INPUT_REG:
                rAddr = pMsg->reqRegAddr;
                /* the frame coming from data link is OK, so copy the received info in the SCU's modbus map   */
                pDest = (uint8_t*)((uint32_t)&scuMapRoRegister[ixScu] + (uint32_t)2 * (uint32_t)rAddr);
                /*      destination      source                  length */
                memcpy((void*)pDest, (void*)pMsg->pMessage, (size_t)pMsg->totalLen);
                break;

              case FUNCTION_WRITE_MULTIPLE_REG:
                /* this is the ACK at previous writing request */
                scuInfoMng.scuStates = SCU_OPERATIVE;
                break;

              default:
                break;
            }
          }
          /* free temporary payload buffer */
          free(pMsg->pMessage);
          /* and then return in operative state  */
          scuInfoMng.scuStates = SCU_OPERATIVE;
          break;

        default:
          break;
      }

    default:
      break;
  }

  return (timeTickTmp);
}
// --------------------------------------------------------------------------------------------------------------------------- //

/**
*
* @brief        Get the pointer to task queue
*
* @param [in]   none
*
* @retval       xQueueHandle: pointer to defined  queue
*
***********************************************************************************************************************/
xQueueHandle getScuQueueHandle(void)
{
   return(scuQueue);
}

void scuSemGestTask (void * pvParameters)
{

  /*-------- Creates an empty mailbox for uart  messages --------------------------*/
  scuSemQueue = xQueueCreate(NUM_BUFF_SCU_RX, sizeof(frameScu_st));
  configASSERT(scuSemQueue != NULL);
  
  /* We want this queue to be viewable in a RTOS kernel aware debugger, so register it. */
  vQueueAddToRegistry(scuSemQueue, "scuSemQueueRx" );

  /* init structure for management */
  scuSemInfoMng.scuStates = SCU_OPERATIVE;

  for (;;)
  {
    /* Wait for some event from Rx/Tx uart SBC (typically UART5)  */
    if (xQueueReceive(scuSemQueue, (void *)&frameScuSemRx, portMAX_DELAY) == pdPASS)
    {
      scuSemMsgProcess(&frameScuSemRx); 
    }
    else
    {
      ;
    }
  }
}

/**
*
* @brief        Fill Data used by app during login
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/

void APP_FillDataForLogin(appMapRoRegister_st  *pData)
{
  
  uint8_t               mdbAddr;
  scuRwMapRegister_st*  pRwRegs;
  scuRoMapRegister_st*  pRoRegs; 
  appMapRwRegister_st*  pAppRwRegs;

  mdbAddr = getLogicalMdbAddrSem();

  pRwRegs = getRwMdbRegs(mdbAddr);
  pRoRegs = getRoMdbRegs(mdbAddr);

  pAppRwRegs = getAppMdbRwRegs(mdbAddr);
  
  /* Fill required data */
  pData->rswCurr     = pRoRegs->scuMapRegStatusMeas.rswCurr;
  pData->maxTypCurr  = pRwRegs->scuSetRegister.maxTypCurr;
  pData->evsePwrMode = pAppRwRegs->evsePwrMode;
  pData->maxPower    = pAppRwRegs->maxPower;
  pData->maxTypCurr  = pRwRegs->scuSetRegister.maxTypCurr;
  pData->evseMod     = pRwRegs->scuSetRegister.evseMod;
  memcpy(pData->mfwVer, pRoRegs->scuMapRegInfoVer.mfwVer, (LEN_BOARD_FW_VERSION_RO * 2));  // 12 registers      
  memcpy(pData->rfdVer, pRoRegs->scuMapRegInfoVer.rfdVer, (LEN_MIFARE_FW_VERSION_RO * 2)); // 10 registers 
  pData->mtType      = pRwRegs->scuSetRegister.mtType;
  pData->connectStatus = pAppRwRegs->connectStatus;
  memcpy(pData->RoutSSID, (uint8_t *)&pAppRwRegs->RoutSSID, LEN_ROUTER_NET_SSID_RW * 2);  // 16 registers 
  memcpy(pData->schFlag1, (uint8_t *)&pAppRwRegs->schFlag1, 20 * 2);                      // 20 registers 
  pData->menuVisible = pRwRegs->scuSetRegister.menuVisible;
  memcpy((uint16_t *)&pData->pmImin,(uint16_t *)&pRwRegs->scuSetRegister.pmImin, 6 * 2);  // 6 registers
  pData->pmMode = pRwRegs->scuSetRegister.pmMode;
  pData->hwChecks1 = pRwRegs->scuSetRegister.hwChecks1;
  pData->hwChecks2 = pRwRegs->scuSetRegister.hwChecks2;
  pData->hwActuators = pRwRegs->scuSetRegister.hwActuators;
  
}


/**
*
* @brief        Decoder message coming from SBC on UART5, or for internal task as debug 
*
* @param [in]   none
*
* @retval       uint8_t: TRUE, if an answer must be sent
*
***********************************************************************************************************************/
static void scuSemMsgProcess(frameScu_st* pMsg)
{
  frameMdbRx_st*        pRxFrame;
  uint8_t              *pDataSource;
  regStartInfo_t       *pDataInfo;
  uint16_t              rAddr, ixScu, len, lenB, ixScuLogic;
  endianessType_e       endian;
  regScuAddr_t          scuObject;
  frameSbcSem_st        tmpFrameSbcSem;  

  switch (scuSemInfoMng.scuStates)
  {
    case SCU_OPERATIVE:
      if ((pMsg->scuEvents == SCU_EVENT_MSG_FROM_SBC) || (pMsg->scuEvents == SCU_EVENT_RD_ANSW_WR_REQ) || (pMsg->scuEvents == SCU_EVENT_MSG_FROM_WIFI))
      {
        pRxFrame = (frameMdbRx_st*)pMsg->pMessage;
        rAddr = swapW(pRxFrame->message.nodeReadInputReg.regAdd);
        ixScu = pRxFrame->message.nodeReadInputReg.unitId; 
        pMsg->scuRegIdx = getIndexFromAddress(rAddr);
        len = swapW(pRxFrame->message.nodeReadInputReg.numWords);
        ixScuLogic = getScuModbusTableId(ixScu);
        /* set the pointer for App */
        pAppRwRegs = getAppMdbRwRegs(ixScuLogic);

        
        // NULL id ? exit
        if (ixScuLogic == NULL_ID)
          return;
        
        /* the frame coming from data link check is OK */
        switch (pRxFrame->message.nodeReadInputReg.function)
        {
          case FUNCTION_READ_HOLDING_REG:
          case FUNCTION_READ_INPUT_REG:            
            len = swapW(pRxFrame->message.nodeReadInputReg.numWords);           
            pDataInfo = findBaseAddressInMap(rAddr, ixScuLogic, pRxFrame->message.nodeReadInputReg.function, (uint8_t)len, &endian);
            
            if (pDataInfo != NULL)
            {
              if (pDataInfo->index == UTC_DATE_TIME_RW)
              {
                /* a RTC read has been required: before to send the data&Time update the modbus RTC regs  */
                setCurrentDateTimeInSem();
              }

              if (pDataInfo->index == TM_ERROR1_IN_TEST_RO)
              {
                if (getCollaudoRunning() != TRUE)
                {
                  /* send the info to notify the testing maching starting  */
                  sendEventToSemMng(NOTIFY_TO_MASTER_TX, ADDR_TM_EVSE_READY_RO);
                }
                setCollaudoRunning();
              }
                              
              if (pDataInfo->index == TM_GET_EEPROM_INFO)
              {
                /* a request to send EEPROM info has been received from master */
                if (getAndsendAllSlaveParameters(ixScuLogic) == 0)  // 0 means NO_ERROR
                {
                  osDelay(50); /* 50ms from end transmission eeprom parameters and start send ack */
                }
              }
                              
              /* this address range can be accepted: send the value of request register on RS485 (SCUs) or UART (SBC/SEM) */
              sendSemRegister(rAddr, len, (uint8_t)ixScu, pDataInfo, pRxFrame->message.nodeReadInputReg.function, endian, pMsg->scuEvents);
              
              if ((pMsg->scuEvents == SCU_EVENT_MSG_FROM_SBC) || (pMsg->scuEvents == SCU_EVENT_RD_ANSW_WR_REQ))
              {
                if (pMsg->scuEvents == SCU_EVENT_MSG_FROM_SBC) 
                {
                  tmpFrameSbcSem.sbcSemEvent = SCU_EVENT_MSG_FROM_SBC_RD;
                }
                else
                {
                  tmpFrameSbcSem.sbcSemEvent = SCU_EVENT_MSG_FROM_MASTER_RD;
                }
                tmpFrameSbcSem.data.index = ixScu;                     // physical address 1..247
                tmpFrameSbcSem.data.rAddr = rAddr;
                tmpFrameSbcSem.dataToSend.len = len;                   // lenght in word of received data
                tmpFrameSbcSem.dataToSend.pData = pDataInfo->pRegAdd;  // address where data are stored
                tmpFrameSbcSem.timeEntry = getPacketStatusNum();

                configASSERT(xQueueSendToBack(getSbcSemQueueHandle(), (void *)&tmpFrameSbcSem, portMAX_DELAY) == pdPASS);  // sbcSemMsgProcess()
              }
              
              /* Se arriva una richiesta di lettura del motivo del reboot --> reset boot event */
              if(rAddr == ADDR_EVSE_BOOT_EVENT_RO)
              {
               setBootEvent(REGULAR_BOOT);
              }
                                  
              /* Lettura dalla app dell'esito dell'aggiornamento FW*/
              if(rAddr == ADDR_FW_UPDATE_STATUS_RO)
              {
                /* Resetto il valore */
                setEsitoUpdateFw(0x00);
                pAppRwRegs->fwUpdateStatus = 0x00;
              }
                                                
              free((void*)pDataInfo);
              
            }
            else
            {
              /* error answer  */
              sendAnswToSem(MB_ILLEGAL_ADDR, (uint8_t)ixScu, rAddr, len, pMsg->scuEvents, pRxFrame->message.nodeReadInputReg.function);
              free((void*)pDataInfo);
            }
            break;

          case FUNCTION_WRITE_MULTIPLE_REG:
            len = swapW(pRxFrame->message.nodeRWmultipleReg.numWords);
            lenB = pRxFrame->message.nodeRWmultipleReg.numBytes;
            if (lenB == (uint16_t)(2 * (uint16_t)len))
            {
              findScuObject(rAddr, ixScu, &scuObject, pMsg->scuEvents);
              ixScuLogic = getScuModbusTableId(scuObject.index);
              
              // NULL id ? exit
              if (ixScuLogic == NULL_ID)
                return;
              
              pDataInfo = findBaseAddressInMap(scuObject.rAddr, ixScuLogic, FUNCTION_WRITE_MULTIPLE_REG, (uint8_t)len, &endian);
              if (pDataInfo != NULL)
              {
                pDataSource = (uint8_t*)pRxFrame->message.nodeRWmultipleReg.data;
                /***** ATTENZIONE sembra che venga passata BIG_ENDIANESS e quindi il  setScuRegister NON Funziona NICK!!! */
                /* this address range can be accepted */
                tmpFrameSbcSem.status = setScuRegister(rAddr, len, (uint8_t)scuObject.index, pDataInfo, pDataSource, endian);
                /* send the info to sbc notify manager */
                if ((pMsg->scuEvents == SCU_EVENT_MSG_FROM_SBC) || (pMsg->scuEvents == SCU_EVENT_RD_ANSW_WR_REQ))
                {
                  tmpFrameSbcSem.sbcSemEvent = (pMsg->scuEvents == SCU_EVENT_MSG_FROM_SBC) ? SBC_SEM_EVENT_UART5 : SBC_SEM_EVENT_RS485;
                  if (ixScu == MODBUS_BROADCAST_ADDR)
                  {
                    tmpFrameSbcSem.data.index = ixScu; // broadcast = 0
                  }
                  else
                  {
                    tmpFrameSbcSem.data.index = scuObject.index; // physical address 1...247 
                  }
                  tmpFrameSbcSem.data.rAddr = scuObject.rAddr;
                  tmpFrameSbcSem.dataToSend.len = len;                   // lenght in word of received data
                  tmpFrameSbcSem.dataToSend.pData = pDataInfo->pRegAdd;  // address where data are stored
                  tmpFrameSbcSem.timeEntry = getPacketStatusNum();
                  configASSERT(xQueueSendToBack(getSbcSemQueueHandle(), (void *)&tmpFrameSbcSem, portMAX_DELAY) == pdPASS);  // sbcSemMsgProcess()
                }
                  
                free((void*)pDataInfo);

                if ((ixScu != MODBUS_BROADCAST_ADDR) && 
                    (((ixScu == SCU_M_P_ADDR) || (checkSlaveDwldRemoteScu(rAddr) == TRUE)) || 
                    (tmpFrameSbcSem.sbcSemEvent == SBC_SEM_EVENT_RS485) || (pMsg->scuEvents == SCU_EVENT_MSG_FROM_WIFI)))
                {
                  /* send the answer immediatly if: no broadcast and message for SCU master or the message coming from RS485 */ 
                  /* if the message coming from SBC and the destination is a slave the ACK will be sent only after ACK from RS485 */
                  /* NOTE: all the request from Wifi must be processed */
                  if(illegalAddress)
                  {
                    sendAnswToSem(MB_ILLEGAL_DATA, (uint8_t)ixScu, rAddr, len, pMsg->scuEvents, pRxFrame->message.nodeReadInputReg.function);
                    illegalAddress = REC_ILLEGAL_ADDRESS_RST;
                  }
                  else
                  {    
                    if (((getScuAddressTypeMode() == SCU_FIXED_ADDR) || (collaudoRunning == TRUE)) && (isSlaveWaitToBeOperative() == FALSE))
                    {
                      sendAnswToSem(NO_ERROR, (uint8_t)ixScu, rAddr, len, pMsg->scuEvents, pRxFrame->message.nodeReadInputReg.function);
                    }
                    
                    // Luca: se viene richiesta la scrittura del registro UTC_DATE_TIME_RW, viene aggiornato l'RTC
                    if(rAddr == ADDR_UTC_DATE_TIME_RW)
                    {
                      setCurrentTimestamp();
                    }
                  }
                }
              }
              else
              { 
                /* error answer  */
                sendAnswToSem(MB_ILLEGAL_ADDR, (uint8_t)ixScu, rAddr, len, pMsg->scuEvents, pRxFrame->message.nodeReadInputReg.function);
              }
            }
            else
            {
              /* error answer  */
              sendAnswToSem(MB_ILLEGAL_DATA, (uint8_t)ixScu, rAddr, len, pMsg->scuEvents, pRxFrame->message.nodeReadInputReg.function);
            }
            break;

          /* Illegal code fuction received */  
          default:
            sendAnswToSem(MB_ILLEGAL_FUNC, (uint8_t)ixScu, rAddr, len, pMsg->scuEvents, pRxFrame->message.nodeReadInputReg.function);
            break;
        }
        /* free previous allocated area */
        if (pMsg->pMessage != NULL)
        {
          free(pMsg->pMessage);
          pMsg->pMessage = NULL;
        }
      }
      else
      {
        if (pMsg->scuEvents == SCU_EVENT_RD_REQ_WR_ACK)
        {
          pRxFrame = (frameMdbRx_st*)pMsg->pMessage;
          switch (pRxFrame->message.nodeReadInputReg.function)
          {
            case FUNCTION_READ_HOLDING_REG:
            case FUNCTION_READ_INPUT_REG:
              /* a request to read registers has been made to a SCU e the answer is arrived, so save the result in linked area */
              ixScu = pMsg->idSCU;
              rAddr = pMsg->reqRegAddr;
              if (ixScu > 0)
              {
                ixScuLogic = ixScu - 1;
              }

              len = swapW(pRxFrame->message.nodeReadInputReg.numWords);
              pDataInfo = findBaseAddressInMap(rAddr, ixScuLogic, pRxFrame->message.nodeReadInputReg.function, (uint8_t)len, &endian);
              if (pDataInfo != NULL)
              {
                /* the frame coming from data link is OK, so copy the received info in the SCU's modbus map   */
                pDataInfo->pRegAdd = (uint8_t*)((uint32_t)&scuMapRoRegister[ixScu] + (uint32_t)2 * (uint32_t)rAddr);
                /*      destination                             source                                   length */
                memcpy((void*) pDataInfo->pRegAdd, (void*)&pRxFrame->message.scuAnwsRIR.data, (size_t)pRxFrame->message.scuAnwsRIR.numBytes);
                free((void*)pDataInfo);
              }
              else
              {
                /* error answer  */
                sendAnswToSem(MB_ILLEGAL_ADDR, (uint8_t)ixScu, rAddr, len, pMsg->scuEvents, pRxFrame->message.nodeReadInputReg.function);
              }
              break;

            default:
              break;
          }
        }
        /* free previous allocated area */
        if (pMsg->pMessage != NULL)
        {
          free(pMsg->pMessage);
          pMsg->pMessage = NULL;
        }
      }
      break;

    default:
      break;
  }
}
// --------------------------------------------------------------------------------------------------------------------------- //


/**
*
* @brief        Get the pointer to SEM task queue
*
* @param [in]   none
*
* @retval       xQueueHandle: pointer to defined  queue
*
***********************************************************************************************************************/
xQueueHandle getScuSemQueueHandle(void)
{
   return(scuSemQueue);
}


/**       ---------------------------  DATA LINK MANAGER ----------------------------- **/

void scuDataLinkTask (void * pvParameters)
{
  uint32_t       timeTick;

  /*-------- Creates an empty mailbox for uart  messages --------------------------*/
  scuDataLink = xQueueCreate(NUM_BUFF_SCU_RX, sizeof(frameScuRx_st));
  configASSERT(scuDataLink != NULL); 
  
  /* We want this queue to be viewable in a RTOS kernel aware debugger, so register it. */
  vQueueAddToRegistry(scuDataLink, "scuDLQueue" );

  /* in this case we don't use the auto-reload features */
  xMdbUartTimers = xTimerCreate("MdbUart", TIMER_UART_WD, pdFALSE, (void*)(TIMER_MDB_UART_ID), mdbUartTimCallBack);
  configASSERT(xMdbUartTimers != NULL);

  /* create a binary semaphore used for uart init access  */
  uartScuInitSemaphoreHandle = osSemaphoreNew(1, 1, NULL);
  uartScuSinapsiTxSemaphoreHandle = osSemaphoreNew(1, 1, NULL);

  timeTick = pdMS_TO_TICKS(TIMER_UART_WD);
  uart1ReinitCounter = 0;

  timeTick = portMAX_DELAY;
  /* init state */
  scuDlInfoMng.scuStates = SCU_DL_IDLE;

  collaudoRunning = FALSE;
  configParamDone = FALSE;

  
  xTimerStart (xMdbUartTimers, 0);

  for (;;)
  {
    /* Wait for some event from Rx (another SCU on the bus) or Tx (from management layer or for framing error)  */
    if (xQueueReceive(scuDataLink, (void *)&frameScuDlRx, timeTick) == pdPASS)
    {
      if(frameScuDlRx.headerScuRx.dlEvent == SCU_DL_RX_COLLAUDO)
      {
        timeTick = scuDlProcess(&frameScuDlRx);
      }
      else
      {
        
        if ((scuOpMode < SCU_M_P) && (scuDlInfoMng.scuStates != SCU_DL_IDLE)) 
        {
          if ((frameScuDlRx.headerScuRx.dlEvent == SCU_DL_ERR_UART) || (frameScuDlRx.headerScuRx.dlEvent == SCU_DL_CHANGE_TO_SEM))
          {
            if (frameScuDlRx.headerScuRx.dlEvent == SCU_DL_ERR_UART)
            {
              /* restart a timer to reset UART for RS485 on timeout */
              restartTimerToResetRs485Uart();
              //uartReintialization();           
            }
            else
            {
              /* this is the case when default is EMUMAX0 but the SCU is in SEM enviroment  */
              if ((HAL_GPIO_ReadPin(WIFI_EN_GPIO_Port, WIFI_EN_Pin) == GPIO_PIN_RESET) || (getWifiSbcEnv() == SBC_WIFI_ON))
              {
                scuTypeModes = SCU_SEM_S;
                // xx eeprom_array_set(OPERATIVE_MODE_EADD, (uint8_t*)&scuTypeModes, 1);
                EEPROM_Save_Config (OPERATIVE_MODE_EADD, (uint8_t*)&scuTypeModes, 1);
                /* Write configurations in eeprom array */
                eeprom_ProductConfig_Param_Set();
                if (getPhysicalMdbAddr() == SCU_S_S_COLLAUDO_ADDR)
                {
                  osDelay(3000); // a principal slave (address 11) restart a bit later from secondary slave (12, 13, 14)
                }
                /** restart the system by NVIC reset */
                activeImmediateReset();
              }
            }
          }
          else
          {
            /* we are working in MAX0 emulation */
            /* the received message from RS485 bus must be sent to GSY task  */
            tmpFrameSbcRx.totalLen = frameScuDlRx.headerScuRx.totalLen;
            tmpFrameSbcRx.messageEv = UART_RX_OK;
            if (tmpFrameSbcRx.totalLen > NUM_BUFF_SBC_MSG_RX)
            {
              tmpFrameSbcRx.totalLen = NUM_BUFF_SBC_MSG_RX;
            }
            /*      destination                                  source                  length */
            memcpy((void*)&tmpFrameSbcRx.messageRx, (void*)&frameScuDlRx.message, (size_t)tmpFrameSbcRx.totalLen);
            if (getSbcAnswerQueueHandle() != NULL)
            {
              configASSERT(xQueueSendToBack(getSbcAnswerQueueHandle(), (void *)&tmpFrameSbcRx, portMAX_DELAY) == pdPASS);  // sbcGestTask()
            }
          }
        }
        else
        {
          /* SINAPSI / RSE / SEM enviroment works with SCU as EMUMAX0 but use modbus on UART1 */
          timeTick = scuDlProcess(&frameScuDlRx);
        }
      }
    }
    else
    {
      if (getStatusDwnl() == FALSE)  
      {
        /* If MASTER in MAX0 emulation, disable task timeout */
        if (scuOpMode == SCU_EMUMAX0_M) 
          timeTick = pdMS_TO_TICKS(portMAX_DELAY);
        else
        {
          frameScuDlRx.headerScuRx.dlEvent = SCU_DL_TIMEOUT;      // mi finisci qui durante il caricamento e resetta la seriale !!! Nick da qui 
          timeTick = scuDlProcess(&frameScuDlRx);
        }        
      }
    }
  }
}


/**
*
* @brief        Decoder message coming from RS485 on UART1, prepares and sends the answer 
*
* @param [in]   none
*
* @retval       uint32_t: timer for the queue receive 
*
***********************************************************************************************************************/
static uint32_t scuDlProcess(frameScuRx_st* pMsg)
{
  uint32_t              timeTick;
  uint16_t              rAddr, ixScu, msgLen;
  uint8_t               len, function, *ptr;
  nodeRWmultipleReg_st* pWrRxFrame;
  crcMode_u             crc, *pCrcRx;
  frameSbcSem_st        tmpFrameSbcSem;  
  headerRHR_t*          pReqRdRxFrame;
  frameScu_st           frameScuMng;
  headerAnswRIR_t*      pRdRxFrame;
  uint8_t               *pDest;

  timeTick = portMAX_DELAY;
  if (((pMsg->headerScuRx.dlEvent == SCU_DL_START) && (scuDlInfoMng.scuStates == SCU_DL_IDLE)) ||
      (pMsg->headerScuRx.dlEvent == SCU_DL_ERR_UART)) 
  {
    UART_SCU_Reactivate_Rx(); 
    scuDlInfoMng.scuStates = SCU_DL_READY; 
    // Rilascio il semaforo
    osSemaphoreRelease(uartScuInitSemaphoreHandle); 
    uart1ReinitCounter++;
    return (timeTick);
  }
 
  uart1ReinitCounter = 0;
  ixScu = pMsg->headerScuRx.idSCU; 
#ifdef COME_ERA
  configASSERT(ixScu < SCU_NUM);
#else
#ifdef ADDR_NO_TRANSLATION
  if (ixScu > SCU_NUM)
  {
    tPrintf("Error on SCU index at %d\n\r", HAL_GetTick());
    return (timeTick);
  }
#else
  if (pMsg->headerScuRx.dlEvent == SCU_DL_RECEIVE_FROM_RS485)
  {
    /* restart gard timer on MDB UART  */
    xTimerReset (xMdbUartTimers, 0);  
    /* previuos version works with addr RS485 <--> idConn New version has a matrix connection so address modbus can have a value > 16 [1..253]*/
    pMsg->headerScuRx.idSCU = fromRs485ToSem(pMsg->headerScuRx.idSCU);
  }
#endif
#endif
  ixScu = pMsg->headerScuRx.idSCU; 

  // NULL id ? exit
  if (ixScu == NULL_ID)
    return(timeTick);

  switch (scuDlInfoMng.scuStates)
  {
    case SCU_DL_READY:
    case SCU_DL_READING:
      switch (pMsg->headerScuRx.dlEvent)
      {
        case SCU_DL_ERR_UART:
          /*----- Try to Initializate UART1 RS485 FOR SCUs link -------------------------------------------*/
          /* an irrecuperable error occurred. Reinit uart is necessary */
          reInitScuUart();

          scuDlInfoMng.scuStates = SCU_DL_READY;
          /* end initialization */
          break;

        case SCU_DL_CHANGE_TO_SEM_FROM_TEST:
#ifndef SOSPESA
          /* the automatic test is ended. It is necessary to come back in EMUMAX0 mode  */
          function  = (uint8_t)SCU_GSY;
          // xx eeprom_array_set(OPERATIVE_MODE_EADD, (uint8_t*)&function, 1);
          EEPROM_Save_Config (OPERATIVE_MODE_EADD, (uint8_t*)&function, 1);
          /* Write configurations in eeprom array */
          eeprom_ProductConfig_Param_Set();
          /** restart the system by NVIC reset */
          //activeImmediateResetFromRemove();
#else
          osDelay(20);
#endif
          break;

        case SCU_DL_SEND:
          /*----- Data link frame wrapper  -------------------------------------------*/
          ixScu = pMsg->headerScuRx.idSCU; 
          rAddr = pMsg->headerScuRx.reqRegAddr;
          if (pMsg->headerScuRx.scuDlOp == SCU_DL_OP_RD)
          {
            function = (scuRegister[pMsg->headerScuRx.scuRegIdx].regType == RO) ? FUNCTION_READ_INPUT_REG : FUNCTION_READ_HOLDING_REG;
            if (pMsg->headerScuRx.totalLen == (uint16_t)0)
            {
              /* a single register must be read: get the reg size in word */
              scuDlInfoMng.lastRd.numWords = scuRegister[pMsg->headerScuRx.scuRegIdx].size;
            }
            else
            {
              scuDlInfoMng.lastRd.numWords = pMsg->headerScuRx.totalLen;
            }
            scuDlInfoMng.lastRd.unitId = ixScu;
            scuDlInfoMng.lastRd.regAdd = rAddr;
            getFromScuRegister (rAddr, (uint8_t)scuDlInfoMng.lastRd.numWords, ixScu, function);  // introdurre controllo bus prima di trasmettere Nick
            xTimerReset(xMdbUartTimers, 0);
            scuDlInfoMng.scuStates = SCU_DL_READING;
          }
          else
          {
            if (pMsg->headerScuRx.scuDlOp == SCU_DL_OP_WR)
            {
              if ((scuRegister[pMsg->headerScuRx.scuRegIdx].regType == WO) || (scuRegister[pMsg->headerScuRx.scuRegIdx].regType == RW))
              {
                pMsg->message.nodeRWmultipleReg.function = function = FUNCTION_WRITE_MULTIPLE_REG;
                pMsg->message.nodeRWmultipleReg.unitId = scuDlInfoMng.lastRd.unitId = (uint8_t)ixScu;
                pMsg->message.nodeRWmultipleReg.regAdd = scuDlInfoMng.lastRd.regAdd = rAddr;
                pMsg->message.nodeRWmultipleReg.numWords = scuDlInfoMng.lastRd.numWords; 
                pMsg->message.nodeRWmultipleReg.numBytes = 2 * scuDlInfoMng.lastRd.numWords; 

                setToScuRegister ((scuRWmultipleReg_st*)&pMsg->message.nodeRWmultipleReg);
             }
            }
          }
          break;
  
        case SCU_DL_RECEIVE_FROM_RS485:
        case SCU_DL_RX_COLLAUDO: 
#ifdef ADDR_NO_TRANSLATION
          /* restart gard timer on MDB UART  */
          xTimerReset (xMdbUartTimers, 0);
#endif            
          /* point to start received message */
          pReqRdRxFrame = (headerRHR_t*)&pMsg->message;
          switch (pReqRdRxFrame->function)
          {
            case FUNCTION_READ_HOLDING_REG:
            case FUNCTION_READ_INPUT_REG:
              ptr = (uint8_t*)pReqRdRxFrame; // here we take a message as an array
              pCrcRx = (crcMode_u*)&ptr[pMsg->headerScuRx.totalLen - 2]; // this is the pointer to CRC_L No swap is necessary
              if (((pMsg->headerScuRx.totalLen % 2) == 0) && (pMsg->headerScuRx.totalLen  == (uint8_t)8))
              {
                msgLen = sizeof(headerRHR_t); /* a read request incoming */
              }
              else
              {
                msgLen = pMsg->message.headerAnwsRIR.numBytes + sizeof(headerAnswRHR_t); /* data from a read request are arrived  */
              }
              crc.crcW = crcEvaluation ((uint8_t*)pReqRdRxFrame, msgLen);
              if (crc.crcW == pCrcRx->crcW)
              {
                rAddr = swapW(pReqRdRxFrame->regAdd);
                if (isThisMsgForUs(pReqRdRxFrame->unitId, rAddr))
                {
                  frameScuMng.idSCU = (uint16_t)pReqRdRxFrame->unitId;
                  frameScuMng.reqRegAddr = pReqRdRxFrame->regAdd;
                  if (((pMsg->headerScuRx.totalLen % 2) == 0) && (pMsg->headerScuRx.totalLen  == (uint8_t)8))
                  {
                    /* a request to read register is arrived for this SCU */
                    frameScuMng.scuEvents = SCU_EVENT_RD_ANSW_WR_REQ;
                    frameScuMng.totalLen = pReqRdRxFrame->numBytes;
                    pDest =  (uint8_t*)malloc(sizeof(headerRHR_t));
                    /*      destination      source                  length */
                    memcpy((void*)pDest, (void*)pReqRdRxFrame, (size_t)sizeof(headerRHR_t));
                    /* send the payload to manager */
                    frameScuMng.pMessage = pDest;
                    configASSERT(xQueueSendToBack(getScuSemQueueHandle(), (void *)&frameScuMng, portMAX_DELAY) == pdPASS);  // scuSemMsgProcess()                                       
                  }
                  else
                  {
                    /* an answer to a previous request is arrived to this SCU */
                    pRdRxFrame = (headerAnswRIR_t*)&pMsg->message;
                    frameScuMng.totalLen = pMsg->headerScuRx.totalLen - 2;  /* the last two crc byte no more necessary */
#ifdef MODBUS_TCP_EM_ETH
                    /*      destination      source                  length */
                    memcpy((void*)getLovatoEthRegs(), (void*)pRdRxFrame, (size_t)frameScuMng.totalLen);   
                    tmpFrameSbcSem.sbcSemEvent = NOTIFY_MODBUS_RD_ACK;  // notify the reading 
                    configASSERT(xQueueSendToBack(getEmLovatoQueueHandle(), (void *)&tmpFrameSbcSem, portMAX_DELAY) == pdPASS);  // tcp_thread() 
#else
                    pDest =  (uint8_t*)malloc(sizeof(headerAnwsRIR_t));
                    /*      destination      source                  length */
                    memcpy((void*)pDest, (void*)pRdRxFrame, (size_t)frameScuMng.totalLen);   
                    /* send the message to manager */
                    tmpFrameSbcSem.sbcSemEvent = NOTIFY_MODBUS_RD_ACK;  // notify the reading 
                    tmpFrameSbcSem.data.index = pMsg->message.scuAnwsRIR.unitId;     // physical address 1..32
                    tmpFrameSbcSem.dataToSend.pData = pDest;
                    tmpFrameSbcSem.dataToSend.len = (uint16_t)pMsg->message.scuAnwsRIR.numBytes;
                    tmpFrameSbcSem.timeEntry = getPacketStatusNum();
                    //configASSERT(xQueueSendToBack(getScuQueueHandle(), (void *)&frameScuMng, portMAX_DELAY) == pdPASS);  // scuMsgProcess()
                    configASSERT(xQueueSendToBack(getSbcSemQueueHandle(), (void *)&tmpFrameSbcSem, portMAX_DELAY) == pdPASS);  // sbcSemMsgProcess()                                        
#endif
                  }
                }
              }
              break;

            case FUNCTION_WRITE_MULTIPLE_REG:
              pWrRxFrame = (nodeRWmultipleReg_st *)pReqRdRxFrame;
              ptr = (uint8_t*)pWrRxFrame; // here we take a message as an array
              len = pMsg->headerScuRx.totalLen - 2;
              pCrcRx = (crcMode_u*)&ptr[len]; // this is the pointer to CRC_L No swap is necessary
              crc.crcW = crcEvaluation ((uint8_t*)pReqRdRxFrame, (uint16_t)len);
              if (crc.crcW == pCrcRx->crcW)
              {
                rAddr = swapW(pWrRxFrame->regAdd);
                if (isThisMsgForUs(pWrRxFrame->unitId, rAddr))
                {
#ifdef COME_ERA
                  frameScuMng.idSCU = (uint16_t)pWrRxFrame->unitId;
                  frameScuMng.reqRegAddr = pWrRxFrame->regAdd;
                  if (((pMsg->headerScuRx.totalLen % 2) == 0) && (pMsg->headerScuRx.totalLen  == (uint8_t)8))
                  {
                    /* this the ACK to WR multiple register previous request */
                    frameScuMng.scuEvents = SCU_EVENT_RD_REQ_WR_ACK;
                    pRdRxFrame = (headerAnswRIR_t*)&pMsg->message;
                    frameScuMng.totalLen = pMsg->headerScuRx.totalLen;
                    pDest =  (uint8_t*)malloc(frameScuMng.totalLen);
                    /*      destination      source                  length */
                    memcpy((void*)pDest, (void*)pRdRxFrame, (size_t)frameScuMng.totalLen);
                    /* send the payload to manager */
                    frameScuMng.pMessage = pDest;
                    configASSERT(xQueueSendToBack(getScuQueueHandle(), (void *)&frameScuMng, portMAX_DELAY) == pdPASS); // scuMsgProcess()
                  }
                  else
                  {
                    /* this the REQ to WR multiple register with data inside the message  */
                    frameScuMng.scuEvents = SCU_EVENT_RD_ANSW_WR_REQ;
                    pWrRxFrame = (nodeRWmultipleReg_st*)&pMsg->message;
                    frameScuMng.totalLen = pMsg->headerScuRx.totalLen;
                    pDest =  (uint8_t*)malloc(frameScuMng.totalLen);
                    /*      destination      source                  length */
                    memcpy((void*)pDest, (void*)pWrRxFrame, (size_t)frameScuMng.totalLen);
                    /* send the payload to manager */
                    frameScuMng.pMessage = pDest;
                    configASSERT(xQueueSendToBack(getScuSemQueueHandle(), (void *)&frameScuMng, portMAX_DELAY) == pdPASS);  // scuSemMsgProcess()
                  }
#else
                  frameScuMng.idSCU = (uint16_t)pWrRxFrame->unitId;
                  frameScuMng.reqRegAddr = pWrRxFrame->regAdd;
                  if (((pMsg->headerScuRx.totalLen % 2) == 0) && (pMsg->headerScuRx.totalLen  == (uint8_t)8))
                  {
                    /* this the ACK to WR multiple register previous request */
                    tmpFrameSbcSem.sbcSemEvent = NOTIFY_MODBUS_WR_ACK;  // notify the writing 
                    tmpFrameSbcSem.data.index = pWrRxFrame->unitId ; // physical address 1..32
                    rAddr = swapW(pWrRxFrame->regAdd);
                    tmpFrameSbcSem.data.rAddr = rAddr % SIZE_MODBUS_MAP; 
                    pRdRxFrame = (headerAnswRIR_t*)&pMsg->message;
                    frameScuMng.totalLen = pMsg->headerScuRx.totalLen;
                    pDest =  (uint8_t*)malloc(frameScuMng.totalLen);
                    /*      destination      source                  length */
                    memcpy((void*)pDest, (void*)pRdRxFrame, (size_t)frameScuMng.totalLen);
                    tmpFrameSbcSem.dataToSend.pData = pDest;
                    tmpFrameSbcSem.dataToSend.len = swapW(pWrRxFrame->numWords);
                    tmpFrameSbcSem.timeEntry = getPacketStatusNum();
                    configASSERT(xQueueSendToBack(getSbcSemQueueHandle(), (void *)&tmpFrameSbcSem, portMAX_DELAY) == pdPASS);  // sbcSemMsgProcess()
                  }
                  else
                  {
                    /* this the REQ to WR multiple register with data inside the message  */
                    frameScuMng.scuEvents = SCU_EVENT_RD_ANSW_WR_REQ;
                    pWrRxFrame = (nodeRWmultipleReg_st*)&pMsg->message;
                    frameScuMng.totalLen = pMsg->headerScuRx.totalLen;
                    pDest =  (uint8_t*)malloc(frameScuMng.totalLen);
                    /*      destination      source                  length */
                    memcpy((void*)pDest, (void*)pWrRxFrame, (size_t)frameScuMng.totalLen);
                    /* send the payload to manager */
                    frameScuMng.pMessage = pDest;
                    configASSERT(xQueueSendToBack(getScuSemQueueHandle(), (void *)&frameScuMng, portMAX_DELAY) == pdPASS);  // scuSemMsgProcess()
                  }

#endif
                }
              }
              break;

            default:
              break;
          }
          break;

        default:
          break;
      }
      break;

    default:
      break;
  }
  return(timeTick);
}

// --------------------------------------------------------------------------------------------------------------------------- //

/**
*
* @brief        Get the pointer to task queue
*
* @param [in]   none
*
* @retval       xQueueHandle: pointer to defined  queue
*
***********************************************************************************************************************/
xQueueHandle getScuDataLinkQueueHandle(void)
{
   return(scuDataLink);
}

/**
*
* @brief        Get the pointer to task queue
*
* @param [in]   none
*
* @retval       xQueueHandle: pointer to defined  queue
*
***********************************************************************************************************************/
static scuAllRegIdx_e getIndexFromAddress(uint16_t addr)
{
  uint8_t ix;

  for (ix = 0; ix < SCU_LAST_REG_IDX; ix++)
  {
    if (scuRegister[ix].regAdd == addr)
    {
      break;
    }
  }
  return((scuAllRegIdx_e)ix);
}



/**
*
* @brief        build an async message to SCU   
*
* @param [in]   uint16_t : reg address to be read
* @param [in]   uint8_t : num word to be read (< 32)
* @param [in]   uint8_t* : pointer to buffer where to put the answer
* @param [in]   uint8_t : SCU phisical address on RS485 bus (0 for broadcast)
* @param [in]   uint8_t : when TRUE a check on address reg is done
*
* @retval       none:  
*
***********************************************************************************************************************/
void  getFromScuRegister (uint16_t regAddr, uint8_t size, uint8_t lpoAddr, uint8_t function)
{
  uint8_t*      pEnd;
  headerRIR_t*  pMsg;
  uint8_t*      pMallocTx;
  uint16_t      length;
  crcMode_u     crc;

  pMallocTx = (uint8_t*)malloc((uint16_t)size + (uint16_t)sizeof(crc)); // remember + 2 for checksum 
  pMsg = (headerRIR_t*)pMallocTx;
  /* the command must be executed    */
  length = (uint16_t)sizeof(headerRIR_t);
  pMsg->unitId = lpoAddr; 
  pMsg->function = function;
  pMsg->regAdd = swapW(regAddr);
  pMsg->numWords = swapW((uint16_t)size);
  /* now found the CRC message */
  crc.crcW = crcEvaluation ((uint8_t*)pMsg, length);
  /* position pEnd pointer on crc field */
  pEnd =  (uint8_t*)((uint32_t)pMsg + (uint32_t)length);
  *pEnd = crc.crcLH_st.crcL;
  pEnd++;
  *pEnd = crc.crcLH_st.crcH;
  length += sizeof(crc.crcW);

  if (xSemaphoreTake(uartScuSinapsiTxSemaphoreHandle, portMAX_DELAY) == pdTRUE)
  {
    pMallocTx485 = pMallocTx; 
    /* start transmission on UART RS485 to SCU RS485 Bus using DMA */
    UART_SCU_DMA_Tx((uint8_t*)pMsg, (uint16_t)length); 
  }

}

/**
*
* @brief        build an async WRMR message to SCU on RS485  
*
* @param [in]   scuRWmultipleReg_st* : pointer to struct where is store the message 
*
* @retval       none:  
*
***********************************************************************************************************************/
static void  setToScuRegister (scuRWmultipleReg_st* pScuRWmR)
{
  uint8_t*            pEnd;
  headerReqRWMR_st*   pMsg;
  uint8_t*            pMallocTx;
  uint16_t            length;
  crcMode_u           crc;

  pMallocTx = (uint8_t*)malloc(sizeof(headerReqRWMR_st) + pScuRWmR->numBytes + (uint16_t)sizeof(crc)); // remember + 2 for checksum 
  pMsg = (headerReqRWMR_st*)pMallocTx;
  /* the command must be executed    */
  length = (uint16_t)sizeof(headerReqRWMR_st) + pScuRWmR->numBytes;
  pMsg->unitId = pScuRWmR->unitId; 
  pMsg->function = pScuRWmR->function;
  pMsg->regAdd = swapW(pScuRWmR->regAdd);
  pMsg->numWords = swapW(pScuRWmR->numWords);
  pMsg->numBytes = pScuRWmR->numBytes;
  /* now found the CRC message */
  crc.crcW = crcEvaluation ((uint8_t*)pMsg, length);
  /* position pEnd pointer on crc field */
  pEnd =  (uint8_t*)((uint32_t)pMsg + (uint32_t)length);
  *pEnd = crc.crcLH_st.crcL;
  pEnd++;
  *pEnd = crc.crcLH_st.crcH;
  length += sizeof(crc.crcW);

  if (xSemaphoreTake(uartScuSinapsiTxSemaphoreHandle, portMAX_DELAY) == pdTRUE)
  {
    pMallocTx485 = pMallocTx; 
    /* start transmission on UART RS485 to SCU RS485 Bus using DMA */
    UART_SCU_DMA_Tx((uint8_t*)pMsg, (uint16_t)length); 
  }

}

/**
*
* @brief        build the answer getting data from modbus map    
*
* @param [in]   uint16_t : reg address to be read
* @param [in]   uint8_t : num word to be read (< 32)
* @param [in]   uint8_t* : pointer to buffer where to put the answer
* @param [in]   uint8_t : SCU phisical address on RS485 bus (0 for broadcast)
* @param [in]   uint8_t : when TRUE a check on address reg is done
*
* @retval       none:  
*
***********************************************************************************************************************/
void   getFromModbusMap (frameMdbRx_st* pRdRxFrame, uint16_t totalLen, scuOpModes_e opMode)
{
  uint8_t           *ptr, *pDest;
  crcMode_u         crc, *pCrcRx;
  frameScu_st       frameScuMng;
  int16_t           len;

  /* point to start received message */
  
  ptr = (uint8_t*)pRdRxFrame; // here we take a message as an array
  frameScuMng.idSCU = pRdRxFrame->message.nodeReadInputReg.unitId;
  len = (int16_t)(totalLen - sizeof(crc.crcW));
  if (((isSemMasterFz() == TRUE) || (opMode == SCU_S_P)) && (len >= sizeof(headerRIR_t)))
  {
    pCrcRx = (crcMode_u*)&ptr[len]; // this is the pointer to CRC_L No swap is necessary
    if (((pRdRxFrame->message.nodeReadInputReg.function == FUNCTION_READ_HOLDING_REG) || (pRdRxFrame->message.nodeReadInputReg.function == FUNCTION_READ_INPUT_REG)) ||
        (pRdRxFrame->message.nodeReadInputReg.function == FUNCTION_WRITE_MULTIPLE_REG))
    {
      crc.crcW = crcEvaluation ((uint8_t*)pRdRxFrame, len);
      if (crc.crcW == pCrcRx->crcW)
      {
        frameScuMng.scuEvents = SCU_EVENT_MSG_FROM_SBC;
        pDest =  (uint8_t*)malloc(len);
        /*      destination      source             length */
        memcpy((void*)pDest, (void*)pRdRxFrame, (size_t)len);
        /* send the payload to manager */
        frameScuMng.pMessage = pDest;
        
        configASSERT(xQueueSendToBack(getScuSemQueueHandle(), (void *)&frameScuMng, portMAX_DELAY) == pdPASS);  // scuSemMsgProcess()
      }
    }
  }
}


/**
*
* @brief        Find base address in modbus map   
*
* @param [in]   uint16_t  : reg address to be found
*
* @retval       none:  
*
***********************************************************************************************************************/
static regStartInfo_t*  findBaseAddressInMap (uint16_t rAddr, uint16_t idScu, uint8_t function, uint8_t wordLen, endianessType_e *pEndian)
{
  regStartInfo_t*       pBaseAdd;
  uint8_t               idx, cnt;

  idx = (uint8_t)idScu;  // logic address 0..31
  *pEndian = defEndian;
  pBaseAdd = NULL;

  if (rAddr >= ADDR_RSE_M1_UNIXTIME_SYNC)
  {
    pBaseAdd = 0;
  }
  // Registri da 0x0300 a 0x036E --> Registri RO VERSIONI FW - HW
  if ((rAddr >= ADDR_START_RDD) && (rAddr < ADDR_SCU_RESERVED1))
  {
    if ((rAddr + (uint16_t)wordLen) <= ADDR_SCU_RESERVED1)
    {
      for (cnt = 0; cnt < SCU_RESERVED1; cnt++)
      {
        if (scuRegister[cnt].regAdd == rAddr)
        {
          /* find the start element request  */
          break;
        }
      }
      pBaseAdd = malloc(sizeof(regStartInfo_t));
      if (pBaseAdd != NULL)
      {
        pBaseAdd->index = cnt;
        pBaseAdd->pRegAdd = (uint8_t*)((uint32_t)&scuMapRoRegister[idx].scuMapRegInfoVer + (uint32_t)( 2 * (rAddr - ADDR_START_RDD)));
        if ((rAddr == ADDR_BOARD_FW_VERSION_RO) || (rAddr == ADDR_MIFARE_FW_VERSION_RO)  || (rAddr == ADDR_BOARD_SN_RO) || (rAddr == ADDR_BOOT_FW_VERSION_RO))
        {
          *pEndian = LITTLE_ENDIANESS;
        }
      }
    }
  }
  else
  {
    // Registri da 0x0400 a 0x0426 --> Registri RO di NOTIFICA
    if ((rAddr >= ADDR_NOTIFY_EVSE_PRESENCE_RO) && (rAddr  < ADDR_SCU_RESERVED2))
    {
      if ((rAddr + (uint16_t)wordLen) <= ADDR_SCU_RESERVED2)
      {
        for (cnt = (uint8_t)NOTIFY_EVSE_PRESENCE_RO; cnt < (uint8_t)SCU_RESERVED2; cnt++)
        {
          if (scuRegister[cnt].regAdd == rAddr)
          {
            /* find the start element request  */
            break;
          }
        }
        pBaseAdd = malloc(sizeof(regStartInfo_t));
        if (pBaseAdd != NULL)
        {
          pBaseAdd->index = cnt;
          pBaseAdd->pRegAdd = (uint8_t*)((uint32_t)&scuMapRoRegister[idx].scuMapRegNotify + (uint32_t)( 2 * (rAddr - ADDR_NOTIFY_EVSE_PRESENCE_RO)));
        }
      }
    }
    else if (rAddr == ADDR_APP_EXCHANGE_AREA_RO)   // Registri da 0x0500 --> EXCHANGE AREA FOR APP during the login phase
    {
       pBaseAdd = malloc(sizeof(regStartInfo_t));
       pBaseAdd->pRegAdd = (uint8_t *)&appMapRoRegister[idx];              
       APP_FillDataForLogin(&appMapRoRegister[idx]);
       *pEndian = LITTLE_ENDIANESS;              
    }
    else
    {
      // Registri da 0x0600 a 0x066A --> Registri RO di MISURE
      if ((rAddr >= ADDR_EVSE_EVENT_FLAGS_RO) && (rAddr  < ADDR_SCU_RESERVED6B))
      {
        if ((rAddr + (uint16_t)wordLen) <= ADDR_SCU_RESERVED6B)
        {
          for (cnt = (uint8_t)EVSE_EVENT_FLAGS_RO; cnt < (uint8_t)SCU_RESERVED6; cnt++)
          {
            if (scuRegister[cnt].regAdd == rAddr)
            {
              /* find the start element request  */
              break;
            }
          }
          pBaseAdd = malloc(sizeof(regStartInfo_t));
          if (pBaseAdd != NULL)
          {
            pBaseAdd->index = cnt;
            pBaseAdd->pRegAdd = (uint8_t*)((uint32_t)&scuMapRoRegister[idx].scuMapRegStatusMeas + (uint32_t)( 2 * (rAddr - ADDR_EVSE_EVENT_FLAGS_RO)));
          }
        }
      }
//        else
//        {
//          if ((rAddr >= ADDR_UID_AUTHORIZATION_RO) && (rAddr  < ADDR_TOTAL_ACTIVE_ENERGY_AC_GRID_RO))
//          {
//            pBaseAdd->pRegAdd = (uint8_t*)((uint32_t)&scuMapRoRegister[idx].rfdUid + (uint32_t)( 2 * (rAddr - ADDR_UID_AUTHORIZATION_RO)));
//          }
        
        /* Lettura registri RW */
        else
        {
          if ((rAddr  < ADDR_RESERVED_A))
          {
            if ((rAddr + (uint16_t)wordLen) <= ADDR_RESERVED_A)
            {
              for (cnt = (uint8_t)CONNECTOR_TYPE_RW; cnt < (uint8_t)RESERVED_A; cnt++)
              {
                if (scuRegister[cnt].regAdd == rAddr)
                {
                  /* find the start element request  */
                  break;
                }
              }
              pBaseAdd = malloc(sizeof(regStartInfo_t));
              if (pBaseAdd != NULL)
              {
                pBaseAdd->index = cnt;
                pBaseAdd->pRegAdd = (uint8_t*)((uint32_t)&scuMapRwRegister[idx].scuSetRegister + (uint32_t)( 2 * (rAddr - ADDR_CONNECTOR_TYPE_RW)));
              }
            }             
          }
          else
          {
            /* Registro 0x500 a 0x532 per macchina di collaudo */
            if((rAddr >= ADDR_TM_MEASURED_CURRENT_RW && rAddr < ADDR_RESERVED_B))
            {
              for (cnt = (uint8_t)TM_MEASURED_CURRENT_RW; cnt < (uint8_t)CONNECTOR_INIT_STATUS_RO; cnt++)
              {
                if (scuRegister[cnt].regAdd == rAddr)
                {
                  /* find the start element request  */
                  break;
                }
              }
              pBaseAdd = malloc(sizeof(regStartInfo_t));
              if (pBaseAdd != NULL)
              {
                pBaseAdd->index = cnt;
                pBaseAdd->pRegAdd = (uint8_t*)((uint32_t)&tmMapRegister[idx] + (uint32_t)( 2 * (rAddr - ADDR_TM_MEASURED_CURRENT_RW)));
              }
            }
            else
            {
              /* aggiunti qui anche i registri sul WMR */

              // Registri da 0x0000 a 0x0068 --> Registri RW 
              if (rAddr < ADDR_FILE_COMMAND_RW)
              {
                if ((rAddr + (uint16_t)wordLen) <= ADDR_FILE_COMMAND_RW)
                {
                  for (cnt = (uint8_t)CONNECTOR_TYPE_RW; cnt < (uint8_t)RESERVED_A; cnt++)
                  {
                    if (scuRegister[cnt].regAdd == rAddr)
                    {
                      /* find the start element request  */
                      break;
                    }
                  }
                  pBaseAdd = malloc(sizeof(regStartInfo_t));
                  if (pBaseAdd != NULL)
                  {
                    pBaseAdd->index = cnt;
                    pBaseAdd->pRegAdd = (uint8_t*)((uint32_t)&scuMapRwRegister[idx].scuSetRegister + (uint32_t)( 2 * (rAddr - ADDR_CONNECTOR_TYPE_RW)));
                  }
                }
                
              }

              // Registri da 0x0100 a 0x0102 --> Registri RW di AGGIORNAMENTO FW
              if ((rAddr >= ADDR_FILE_COMMAND_RW) && (rAddr < ADDR_END_RW_REGS_WO))
              {
                pBaseAdd = malloc(sizeof(regStartInfo_t));
                if (pBaseAdd != NULL)
                {
                  if (rAddr == ADDR_FILE_COMMAND_RW) pBaseAdd->index = FILE_COMMAND_RW; else pBaseAdd->index = FILE_SIZE_RW;
                  pBaseAdd->pRegAdd = (uint8_t*)((uint32_t)&scuMapRwRegister[idx].scuSetRegFwUpd + (uint32_t)( 2 * (rAddr - ADDR_FILE_COMMAND_RW)));
                }
              }

              // Registri da 0x590 a 0x591 --> Registri RW del PEN
              if ((rAddr > ADDR_PEN_RELE_REG) && (rAddr < ADDR_PEN_RELE_END))
              {
                pBaseAdd = malloc(sizeof(regStartInfo_t));
                *pEndian = LITTLE_ENDIANESS;
                if (pBaseAdd != NULL)
                {
                  pBaseAdd->pRegAdd = (uint8_t*)&scuMapRwRegister[idx].penReleReg;
                  pBaseAdd->index = (uint8_t)PEN_RELE;
                }
              }
#ifdef MODBUS_TCP_EM_LOVATO
              // Registri da 0x550 a 0x582 --> Registri RO energy meter Lovato Ethernet
              if ((rAddr >= ADDR_EM_LOVATO_V_SYS) && (rAddr < ADDR_EM_LOVATO_END))
              {
                pBaseAdd = malloc(sizeof(regStartInfo_t));
                *pEndian = LITTLE_ENDIANESS;
                if (pBaseAdd != NULL)
                {
                  pBaseAdd->pRegAdd = (uint8_t*)&scuMapRegLovatoEmSim.sysVoltage;
                  pBaseAdd->index = (uint8_t)EM_LOVATO_SIM_V_SYS;
                  if (isSemMasterFz() == TRUE)
                  {
                    /* il master calcola le potenze delle varie prese */
                    scuMapRegLovatoEmSim.sysVoltage = (uint16_t)scuMapRoRegister[0].scuMapRegStatusMeas.mtVsys;
                    for (cnt = 0; cnt < SCU_NUM; cnt++)
                    {
                      scuMapRegLovatoEmSim.socketActivePower[cnt][0]= (uint16_t)((scuMapRoRegister[cnt].scuMapRegStatusMeas.mtPh1Cur * scuMapRegLovatoEmSim.sysVoltage) / 1000);
                      scuMapRegLovatoEmSim.socketActivePower[cnt][1]= (uint16_t)((scuMapRoRegister[cnt].scuMapRegStatusMeas.mtPh2Cur * scuMapRegLovatoEmSim.sysVoltage) / 1000);
                      scuMapRegLovatoEmSim.socketActivePower[cnt][2]= (uint16_t)((scuMapRoRegister[cnt].scuMapRegStatusMeas.mtPh3Cur * scuMapRegLovatoEmSim.sysVoltage) / 1000);
                    }
                  }
                }
              }
#endif
              /* Registri da 8200 a 0x827E per la app */
              if((rAddr >= ADDR_CONNECTOR_INIT_STATUS_RO && rAddr < ADDR_RESERVED_C))
              {
                for (cnt = (uint8_t)CONNECTOR_INIT_STATUS_RO; cnt < (uint8_t)SCU_LAST_REG_IDX; cnt++)
                {
                  if (scuRegister[cnt].regAdd == rAddr)
                  {
                    /* find the start element request  */
                    break;
                  }
                }
                pBaseAdd = malloc(sizeof(regStartInfo_t));
                if (pBaseAdd != NULL)
                {
                  pBaseAdd->index = cnt;
                  pBaseAdd->pRegAdd = (uint8_t*)((uint32_t)&appMapRwRegister[idx] + (uint32_t)( 2 * (rAddr - ADDR_CONNECTOR_INIT_STATUS_RO)));
                }
              }  
              /* end aggiunta */
            }
          }
        }
    }
    //}
  }
              
  return(pBaseAdd);
}

/**
*
* @brief        build an async message to SBC (SEM)   
*
* @param [in]   uint16_t          : reg address to be read
* @param [in]   uint16_t          : num word to be read 
* @param [in]   regStartInfo_t*   : pointer to buffer where to put the answer
* @param [in]   uint8_t           : SCU phisical address on RS485 bus (0 for broadcast)
* @param [in]   uint8_t           : when TRUE a check on address reg is done
*
* @retval       none:  
*
***********************************************************************************************************************/
static void  sendSemRegister (uint16_t regAddr, uint16_t size, uint8_t lpoAddr, regStartInfo_t* pInfoBase, uint8_t function, endianessType_e endianess, scuEvents_e ev)
{
  uint8_t         *pEnd, *pData;
  frameMdbRx_st*  pMsg;
  uint16_t        length;
  crcMode_u       crc;
  uint8_t*        pMallocTx;
  uint8_t         nReg;

  length = (uint16_t)size * (uint16_t)2 + sizeof(headerAnswRHR_t);  
  pMallocTx = (uint8_t*)malloc(length  + (uint16_t)sizeof(crc));  // remember + 2 for checksum
  pMsg = (frameMdbRx_st*)pMallocTx;
  /* the command must be executed    */
  pMsg->message.headerAnwsRIR.unitId = lpoAddr; 
  pMsg->message.headerAnwsRIR.function = function;
  pMsg->message.headerAnwsRIR.numBytes =(uint8_t)(size * (uint8_t)2);
  /* filling data area */
  pData =  (uint8_t*)((uint32_t)&pMsg->message.headerAnwsRIR.numBytes + (uint32_t)1);
  if (endianess == LITTLE_ENDIANESS)
  {
    /* point at register of modbus map   */
    pEnd = (uint8_t*)(pInfoBase->pRegAdd);
    /*     destination       source                   length   */
    memcpy((void*)pData, (void*)pEnd, (size_t)size * (size_t)sizeof(uint16_t));
  }
  else
  {
    /* point at register of  modbus map   */
    pEnd = (uint8_t*)(pInfoBase->pRegAdd);
    for (nReg = 0; nReg < ((uint8_t)size * (uint8_t)sizeof(uint16_t)); nReg += sizeof(uint16_t))
    {
      /* ABCD where A is the most significant byte */
      /* Big Endian = Motorola      = ABCD */
      /* Big Endian swapped = ??    = BADC */
      /* Little Endian = ??         = CDAB */
      /* Little Endian = Intel      = DCBA */
      /** we transmit in big endian     */
      pData[nReg]     = pEnd[nReg + 1];
      pData[nReg + 1] = pEnd[nReg];
    }
  }

  /* now found the CRC message */
  crc.crcW = crcEvaluation ((uint8_t*)pMsg, length);
  /* position pEnd pointer on crc field */
  pEnd =  (uint8_t*)((uint32_t)pMsg + (uint32_t)length);
  *pEnd = crc.crcLH_st.crcL;
  pEnd++;
  *pEnd = crc.crcLH_st.crcH;
  length += sizeof(crc.crcW);

  if (ev == SCU_EVENT_MSG_FROM_SBC)
  {
    scuSemTxMsg.totalLen = length;
    scuSemTxMsg.pMessage = pMallocTx;
    /* start transmission on UART5 (SBC) using DMA */
    UART_SBC_DMA_Tx((uint8_t*)pMsg, (uint16_t)scuSemTxMsg.totalLen);  // at the end of transmission free the malloc area freeSemTxBuffer()
  }
  else
  {
    if (ev == SCU_EVENT_RD_ANSW_WR_REQ)
    {
      if (xSemaphoreTake(uartScuSinapsiTxSemaphoreHandle, portMAX_DELAY) == pdTRUE)
      {
        pMallocTx485 = pMallocTx;
        /* start transmission on UART RS485 to SCU RS485 Bus using DMA */
        txOnRs485Bus((uint8_t*)pMsg, (uint16_t)length);
      }
    }
  }
  
  //free(pMallocTx);  
  
}



/**
*
* @brief        build an async message to SBC (SEM)   
*
* @param [in]   uint16_t          : reg address to be read
* @param [in]   uint16_t          : num words to be write on the map 
* @param [in]   uint8_t           : SCU phisical address on RS485 bus (0 for broadcast)
* @param [in]   regStartInfo_t*   : pointer to modbus address map where to put the answer
* @param [in]   uint8_t*          : pointer to incoming data from SEM 
* @param [in]   uint8_t           : modbus function
* @param [in]   uint8_t           : endianess
*
* @retval       none:  
*
***********************************************************************************************************************/
static uint16_t setScuRegister (uint16_t regAddr, uint16_t size, uint8_t lpoAddr, regStartInfo_t* pInfoBase, uint8_t* pDataSrc, endianessType_e endianess)
{
  uint8_t         *pEnd, *pData, *pSrc;
  uint16_t        lenRegByte, tmp, numWord, retVal;
  uint16_t        *pData16;
  GPIO_PinState   pinVal, mirror, error;
  uint8_t         nReg, idx, progressCnt;
 
  retVal = 0xFF; progressCnt = 0;
  if (endianess == LITTLE_ENDIANESS)
  {
    idx = (uint8_t)pInfoBase->index;
    pSrc = pDataSrc; 
    pData = pInfoBase->pRegAdd;
    do
    {
      /* calcolo lunghezza in byte del registro da scrivere */
      if (size >= scuRegister[idx].size)
      {
        numWord = scuRegister[idx].size;
        lenRegByte = numWord * (uint16_t)2;
      }
      else
      {
        numWord = size;
        lenRegByte = size * (uint16_t)2;
      }
      /* new data: save it                               */
      /* but for change status the change cannot do directly on the map. The new state is returned by the function   */
      /* and is managed in higher level (SBC_SEM_OPERATIVE in sbcSemMsgProcess()                                     */
      if (pInfoBase->index != (uint16_t)EVSE_CHARGE_STATUS_RO)
      {        
        /*          destination       source      length   */
        memcpy((void*)pData, (void*)pSrc, (size_t)lenRegByte);
      }
#ifdef COLLAUDO_PEN
      if (pInfoBase->index == (uint16_t)PEN_RELE)
      {
        /* nel caso del test del PEN aziono subito i relè del gig di collaudo */
        setRelaysGigPen(*((uint16_t*)pSrc));
      }
#endif    
      if (pInfoBase->index == (uint16_t)TM_COMMAND_RW)
      {
        pData16 = (uint16_t*)pDataSrc;
        
        if (((*pData16) & TM_MODO_FREE_MASK) == TM_MODE_FREE_ON)          // bit 0 contattore 0001, bit 2 ritorno in free 0004
        {
          idx = EVS_FREE_MODE;
          /* set free as  operative mode                       */
          EEPROM_Save_Config (EVS_MODE_EADD, (uint8_t*)&idx, 1);
          send_to_evs(EVS_AUTORIZATION_MODE);
          /* the change mode has been executed: a restart it is necessary */
          activeImmediateReset();
        }
        else
        {
          mirror_contact_check_enable();                                      // si alimenta il pin di alimentazione del MIRROR per andare a leggerlo
          if (((*pData16) & TM_CONTATTORE_MASK) == TM_CONTATTORE_ON)          // bit 0 contattore 0001, bit 2 ritorno in free 0004
          {
            mirror = GPIO_PIN_RESET;  // contattore ON --> MIRROR open --> IN3 = 0
            setOutputState(CNTCT, GPIO_PIN_SET);  // eccito bobina contattore
          }
          else
          {
            mirror = GPIO_PIN_SET;  // contattore OFF --> MIRROR close --> IN3 = 1
            setOutputState(CNTCT, GPIO_PIN_RESET);  // diseccito bobina contattore
          }
          osDelay(500);
          /* wait for hardware transient and then check the MIRROR IN3 pin Status */
#ifndef HW_MP28947          
          pinVal = ioExpRead((uint8_t)IN3_MASK);
#else
          pinVal =  GPIO_Read_IN3_MIRROR();
#endif          
          error = GPIO_PIN_SET;
          if (pinVal == mirror)
          {
            /* the read value is the expected value --> no error */
            error = GPIO_PIN_RESET;
          }
          setModbusErrorTesting(IN3_MIRROR_EXP0, error);
        }
        if (collaudoRunning != TRUE)
        {
          /* send the info to notify the testing maching starting  */
          sendEventToSemMng(NOTIFY_TO_MASTER_TX, ADDR_TM_EVSE_READY_RO);
        }
        collaudoRunning = TRUE;
      }
      else 
      {
        if ((pInfoBase->index == (uint16_t)BOARD_SN_RW) && (lenRegByte == 14))
        {
          /* 01234567890123 String must be 14 chars */
          /* S 00012345 8-D */
          setBoardSnHwVerString((char *)pSrc, lenRegByte);
        }
        else
        {
/********************************************************************************************************************/
/********************************         FW for TESTING MACHINE START              *********************************/
/********************************************************************************************************************/
          if (((collaudoRunning) || (pInfoBase->index == TM_UID_MASTER_RW)) && (progressCnt == 0))
          {
            /* if base write address is ADDR_TM_UID_MASTER_RW=0x517: to avoid discovery procedure come in when write 4 word starting from 0x514  */
            switch (pInfoBase->index)
            {
              case TM_UID_USER_RW:
                modbus_uid_write(UID_USER, pDataSrc, ASCII_UID);
                break;
                
              case TM_UID_MASTER_RW:
                modbus_uid_write(UID_MASTER, pDataSrc, ASCII_UID);
                if (getCollaudoRunning() != TRUE)
                {
                  /* send the info to notify the testing maching starting  */
                  sendEventToSemMng(NOTIFY_TO_MASTER_TX, ADDR_TM_EVSE_READY_RO);
                }
                setCollaudoRunning();
                break;
                
              case FILE_COMMAND_RW:
              case FILE_SIZE_RW:
                checkStartFwDwld((char*)pSrc, lenRegByte, pInfoBase->index, lpoAddr);
                break;
                
              case PRODUCT_CODE_RW:
                setProductCodeString((char *)pSrc, lenRegByte);
                configParamDone = TRUE;
                break;
                
              case TECN_PRODUCT_CODE_RW:
                setTecnProductCode((char *)pSrc, lenRegByte);
                break;
                
              case HW_CHECKS1_RW:
              case HW_CHECKS2_RW:
              case HW_ACTUATORS_RW:
                if (configParamDone != TRUE)
                {
                  HW_CHECKS_ACTUATORS_EEprom_Save(regAddr);
                }
                break;
                
              case MAX_TYPICAL_CURRENT_RW:
                if (configParamDone != TRUE)
                {
                  saveMaxTypicalCurrentEeprom(pSrc[0]);
                }
                break;
                
              case MAX_SIMPLIFIED_CURRENT_RW:
                if (configParamDone != TRUE)
                {
                  saveMaxSimplifiedCurrentEeprom(pSrc[0]);
                }
                break;
                
              case PM_MODE_RW:
                if (configParamDone != TRUE)
                {
                  savePmModeEeprom(pSrc[0]);
                }
                break;
                
              case PM_PMAX_RW:
                if (configParamDone != TRUE)
                {
                  savePmPmaxEeprom(pSrc);
                }
                break;
                
              case PM_IMIN_RW:
                if (configParamDone != TRUE)
                {
                  savePmIminEeprom(pSrc[0]);
                }
                break;
                
              case PM_HPOWER_RW:
                if (configParamDone != TRUE)
                {
                  savePmHpowerEeprom(pSrc[0]);
                }
                break;
                
              case PM_DSET_RW:
                if (configParamDone != TRUE)
                {
                  savePmDsetEeprom(pSrc);
                }
                break;
                
              case PM_DMAX_RW:
                if (configParamDone != TRUE)
                {
                  savePmDmaxEeprom(pSrc[0]);
                }
                else
                {
                  /* on this reset a parameters program will be exececuted starting frpm product code only */
                  activeImmediateReset();
                  osDelay(400);
                }
                break;
                
              case DISPLAY_DEFAULT_LANGUAGE_RW:
                if (configParamDone != TRUE)
                {
                  saveDefaultDisplayLanguageEeprom(pSrc[0]);
                }
                break;
                
              case DISPLAY_LANGUAGES_ENABLED_RW:
                if (configParamDone != TRUE)
                {
                  saveDisplayLanguagesEeprom(pSrc);
                }
                break;
                
              case CONNECTOR_IDS_RW:
                if (configParamDone != TRUE)
                {
                  saveConnectorAliasEeprom(pSrc[0]);
                }
                break;
                
              case CONNECTOR_TYPE_RW:
                if (configParamDone != TRUE)
                {
                  saveConnectorTypeEeprom(pSrc[0]);
                }
                break;
                
              case ENERGY_METERS_RW:
                if (configParamDone != TRUE)
                {
                  saveEnergyMetersTypeEeprom(pSrc);
                }
                break;
                
              case MENU_VISIBILITY_RW:
                if (configParamDone != TRUE)
                {
                  saveVisibleFlagsEeprom(pSrc[0]);
                }
                break;
                
              case CHARGE_TIME_RW:
                if (configParamDone != TRUE)
                {
                  saveChargeTimeEeprom(pSrc[0]);
                }
                break;
                
              case PM_FLAGS_RW:
                if (configParamDone != TRUE)
                {
                  savePowerManagementFlagsEeprom(pSrc[0]);
                }
                break;
                
              case CHARGE_MAX_ENERGY_RW:
                if (configParamDone != TRUE)
                {
                  saveChargeEnergyEeprom(pSrc[0]);
                }
                break;

              case TM_MEASURED_CURRENT_RW:
                /* end test on automatic machine. It is necessary to come back in EMUMAX0 */
                sendScuDataLinkMsg(SCU_DL_CHANGE_TO_SEM_FROM_TEST);
                break;
                
            }
/********************************************************************************************************************/
/*********************************        FW for TESTING MACHINE END               **********************************/           
/********************************************************************************************************************/            
          }
          else if ((pInfoBase->index == (uint16_t)PRODUCT_SN_RW) && (lenRegByte == 10))
          {
            setProductSerialNumberEeprom((char*)pSrc, 9, (uint8_t)TRUE);
          } 
          else if (pInfoBase->index == (uint16_t)HWC_FLAGS_RW) 
          {
            uint16_t tmp = *((uint16_t*)pSrc);
            uint8_t idx;
            eeprom_param_get(LCD_TYPE_EADD, &idx, 1);
            if (tmp & 0x08) idx |= WIFI_ON;
            if (tmp & 0x02) idx |= LCD_2X20;           
            /*** SAVE ON EEPROM ***/
            EEPROM_Save_Config (LCD_TYPE_EADD, &idx, 1);
          }
          else
          {
            if (pInfoBase->index == (uint16_t)TM_UID_USER_RW)
            {
              modbus_uid_write(UID_USER, pDataSrc, ASCII_UID);
            }
            else
            {
              if (pInfoBase->index == (uint16_t)TM_UID_MASTER_RW)
              {
                modbus_uid_write(UID_MASTER, pDataSrc, ASCII_UID);
              }
              else
              {
                if ((pInfoBase->index == (uint16_t)FILE_COMMAND_RW) || (pInfoBase->index == (uint16_t)FILE_SIZE_RW))
                {
                  checkStartFwDwld((char*)pSrc, lenRegByte, pInfoBase->index, lpoAddr);  // Nick download
                }
                else
                {
                  if (pInfoBase->index == (uint16_t)EVSE_CHARGE_STATUS_RO) 
                  {
                    pData16 = (uint16_t*)pDataSrc;
                    retVal = (*pData16);   // return the received status 
                  }
                }
              }
            }
          }
        }
      }
      /*  SINAPSI-RSE */
      if (getSinapsiEepromEn() == ENABLED)    
      {
        tmp = IOM_CTRL;
        if (pInfoBase->index == (uint16_t)tmp)
        {
          /* a change on IOM2G control reg is occurred */
          pData16 = (uint16_t*)pDataSrc;
          if (*pData16 != 0)
          {
            /* send new data trigger */
            newSinapsiIom2GData();
          }
        }
        else
        {
          tPrintf ("Reg Id = %d at %d\r\n", idx, HAL_GetTick());
        }
      }
      pData = (uint8_t*)((uint32_t)pData + (uint32_t)lenRegByte);
      pSrc =  (uint8_t*)((uint32_t)pSrc  + (uint32_t)lenRegByte);
      size -= (numWord);
      /* set the next index register */
      idx++;
      pInfoBase->index++;
      progressCnt++;
      
      /* Case modbus command illegal address or size received */
      if(size == 0xFFFF || lenRegByte == 0x0000)
      {
        illegalAddress = REC_ILLEGAL_ADDRESS;
        break;
      }
      
    } while (size != (uint16_t)0);

    /*          destination       source      length   */
    //memcpy((void*)pInfoBase->pRegAdd, (void*)pDataSrc, size);
  }
  else
  {
    /* point at the last address of  modbus map   */
    pEnd = (uint8_t*)((uint32_t)pInfoBase->pRegAdd + (uint32_t)((uint32_t)size * 2) - (uint32_t)1);
    for (nReg = 0, pData = pDataSrc; nReg < ((uint8_t)size * (uint8_t)sizeof(uint16_t)); nReg++, pData++, pEnd--)
    {
      /* ABCD where A is the most significant byte */
      /* Big Endian = Motorola      = ABCD */
      /* Big Endian swapped = ??    = BADC */
      /* Little Endian = ??         = CDAB */
      /* Little Endian = Intel      = DCBA */
      *pEnd = *pData;
    }
  }
  return (retVal);
}

/**
*
* @brief        build the answer on WMR to SBC (SEM)   
*
* @param [in]   uint16_t  : reg address to be read
* @param [in]   mbError_e : codice di errore  
* @param [in]   uint8_t   : SCU phisical address on RS485 bus (0 for broadcast)
* @param [in]   uint16_t  : Start address
* @param [in]   uint16_t  : len data written
* @param [in]   uint8_t   : function command received   
*
* @retval       none:  
*
***********************************************************************************************************************/
void  sendAnswToSem (mbError_e codeError, uint8_t ixScu, uint16_t rAddr, uint16_t len, scuEvents_e ev, uint8_t funct)
{
  frameMdbRx_st*  pMsg;
  uint8_t         *pEnd;
  uint16_t        length;
  crcMode_u       crc;
  uint8_t*        pMallocTx;

  if (codeError == NO_ERROR)
  {
    length = (uint16_t)sizeof(headerAnwsWMR_t); 
    scuSemTxMsg.totalLen = length;
    pMallocTx = (uint8_t*)malloc(scuSemTxMsg.totalLen  + (uint16_t)sizeof(crc));  // remember + 2 for checksum
    pMsg = (frameMdbRx_st*)pMallocTx;
    pMsg->message.headerAnwsWMR.function = FUNCTION_WRITE_MULTIPLE_REG;
    pMsg->message.headerAnwsWMR.unitId = ixScu;
    pMsg->message.headerAnwsWMR.regAdd = swapW(rAddr);
    pMsg->message.headerAnwsWMR.numWord = swapW(len);
  }
  else
  {
    length = (uint16_t)sizeof(headerAnwsError_t);  
    scuSemTxMsg.totalLen = length;
    pMallocTx = (uint8_t*)malloc(scuSemTxMsg.totalLen  + (uint16_t)sizeof(crc));  // remember + 2 for checksum
    pMsg = (frameMdbRx_st*)pMallocTx;
    pMsg->message.headerAnwsError.unitId = ixScu;
    /* Case Read Holding Register (RHR) */
    if(funct == FUNCTION_READ_HOLDING_REG)
    {
      pMsg->message.headerAnwsError.function = (FUNCTION_ERROR_OCCURED | FUNCTION_READ_HOLDING_REG);
    }
    /* Case Write Multiple Register (WMR) */
    else
    {
     pMsg->message.headerAnwsError.function = (FUNCTION_ERROR_OCCURED | FUNCTION_WRITE_MULTIPLE_REG);
    }
    pMsg->message.headerAnwsError.errorCode = codeError;
  }

  /* now found the CRC message */
  crc.crcW = crcEvaluation ((uint8_t*)pMsg, length);
  /* position pEnd pointer on crc field */
  pEnd =  (uint8_t*)((uint32_t)pMsg + (uint32_t)length);
  *pEnd = crc.crcLH_st.crcL;
  pEnd++;
  *pEnd = crc.crcLH_st.crcH;
  length += sizeof(crc.crcW);

  if (ev == SCU_EVENT_MSG_FROM_SBC)
  {
    scuSemTxMsg.totalLen = length;
    scuSemTxMsg.pMessage = pMallocTx;
    /* start transmission on UART5 (SBC) using DMA */
    UART_SBC_DMA_Tx((uint8_t*)pMsg, (uint16_t)scuSemTxMsg.totalLen);  // at the end of transmission free the malloc area freeSemTxBuffer()
  }
  else
  {
    if (ev == SCU_EVENT_RD_ANSW_WR_REQ)
    {
      if (xSemaphoreTake(uartScuSinapsiTxSemaphoreHandle, portMAX_DELAY) == pdTRUE)
      {
        setMallocTx485(pMallocTx);
        /* start transmission on UART RS485 to SCU RS485 Bus using DMA */
        txOnRs485Bus((uint8_t*)pMsg, (uint16_t)length); 
      }
    }
  }
}

/**
*
* @brief       free buffer area acquired by malloc to send a 
*              message over UART5 SBC bus
*
* @param [in]  none  
*  
* @retval      none
*  
****************************************************************/
void freeSemTxBuffer (void)
{
  if (scuSemTxMsg.pMessage != NULL)
  {
    free(scuSemTxMsg.pMessage);
  }
  scuSemTxMsg.pMessage = NULL;
}



/**       ---------------------------  UART  MANAGER ----------------------------- **/

/**
*
* @brief       Global interrupt handler for USART1 used on 
*              RS485 with modbus
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/
void scuModbus_IRQHandler(void) 
{
  /* la gestione della ricezione sulle USART1 è fatta con il DMA e con l'interrupt di idle */
  /* In pratica ogni carattere ricevuto viene trasferito in memoria tramite DMA e solo    */
  /* dopo che la periferiche vede un "idle" fa scattare questo interrupt che decreta la   */
  /* fine del messaggio di ricezione. Il messaggio viene recuperato forzando interrupt    */
  /* del DMA. Nick 17/01/2021                                                             */
  HAL_UART_IRQHandler(&UART_SCU_HANDLE);
  uint32_t  regISR;
  portBASE_TYPE xHigherPriorityTaskWoken;

  regISR = UART_SCU_ISR;
  /* Check for IDLE flag */
  if ((regISR & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE | noiseErr)) != (uint32_t)0) 
  {   
    /* Clear error flags  */
    __HAL_UART_CLEAR_IDLEFLAG(&UART_SCU_HANDLE); 
    __HAL_UART_CLEAR_OREFLAG(&UART_SCU_HANDLE); 
    __HAL_UART_CLEAR_PEFLAG(&UART_SCU_HANDLE);
    __HAL_UART_CLEAR_FEFLAG(&UART_SCU_HANDLE);
    __HAL_UART_CLEAR_NEFLAG(&UART_SCU_HANDLE);
    if (((regISR & noiseErr) != 0) && (scuOpMode <= SCU_EMUMAX0_M))
    {
      /* default is EMUMAX0, so if the slave is put in SEM enviroment (230400 vs 19200) a noise error happen */
      /* it is necessary to change mode and restart to work in SEM mode at 230400 */
      noiseErr = 0;
      scuMsgRx_IT.headerScuRx.dlEvent = SCU_DL_CHANGE_TO_SEM;
      /*send a message to restart UART1 */
      xQueueSendToBackFromISR(getScuDataLinkQueueHandle(), (void *)&scuMsgRx_IT, &xHigherPriorityTaskWoken);
      return;
    }
#ifndef GD32F4xx    
    __HAL_UART_CLEAR_RTOFLAG(&UART_SCU_HANDLE); 
#endif
    if (xQueueIsQueueFullFromISR(getScuDataLinkQueueHandle()) == pdFALSE)
    {
      scuMsgRx_IT.headerScuRx.dlEvent = SCU_DL_ERR_UART;
      if (xSemaphoreTakeFromISR(getScuUartInitSemaphoreHandle(), &xHigherPriorityTaskWoken) == pdTRUE)
      {
        /* there are some error: send a message to restart UART1 */
        xQueueSendToBackFromISR(getScuDataLinkQueueHandle(), (void *)&scuMsgRx_IT, &xHigherPriorityTaskWoken);
      }
    }
  }
  else
  {
    noiseErr = 0;
    /* Check for IDLE flag */
    //if (UART_SBC->ISR & USART_FLAG_IDLE)   /* Nick modifica perchè SBC a 115200 introduce un idle >> di uno sto bit */
#ifndef GD32F4xx       
    if (UART_SCU_ISR & USART_FLAG_RTOF) 
    {   
      /* We want IDLE flag only */
      /* This part is important */
      /* Clear IDLE and RTO flags by reading status register first */
      __HAL_UART_CLEAR_IDLEFLAG(&UART_SCU_HANDLE);     /* Nick modifica perchè SINAPSI introduce un idle lungo >> di uno stop bit 03/11/2021 */
   
      __HAL_UART_CLEAR_RTOFLAG(&UART_SCU_HANDLE); 
      /* And follow by reading data register */
      volatile uint32_t tmp;                        /* Must be volatile to prevent optimizations */
      tmp = UART_SBC_ISR;                           /* Read status register */
      tmp = UART_SBC_RDR;                           /* Read data register */
      (void)tmp;                                    /* Prevent compiler warnings */
      UART_SCU_RX_DMA_STREAM->CCR &= ~DMA_CCR_EN;   /* Disabling DMA will force transfer complete interrupt if enabled */      
    }
#else
    if (UART_SCU_ISR & USART_FLAG_IDLE) 
    {   
      /* We want IDLE flag only */
      /* This part is important */
      /* Clear IDLE and RTO flags by reading status register first */
      __HAL_UART_CLEAR_IDLEFLAG(&UART_SCU_HANDLE);     /* Nick modifica perchè SINAPSI introduce un idle lungo >> di uno stop bit 03/11/2021 */
      /* And follow by reading data register */
      volatile uint32_t tmp;                        /* Must be volatile to prevent optimizations */
      tmp = UART_SCU_ISR;                           /* Read status register */
      tmp = UART_SCU_RDR;                           /* Read data register */
      (void)tmp;                                    /* Prevent compiler warnings */

      /* During the FW download, the end of the stream is managed checking the DMA counter */
      if (codeInfo.state == FW_STATE_RX)
      {
        if (UART_SCU_RX_DMA_STREAM->NDTR == (NUM_BUFF_SBC_MSG_RX - BUFFER_FW_PAYLOAD_CKS))
        {
            UART_SCU_RX_DMA_STREAM->CR &= ~DMA_SxCR_EN;   /* Disabling DMA will force transfer complete interrupt if enabled */
        }        
        return;
      }

//  REMOVED SINCE it doesn't work with more traffic -->    if (xQueueIsQueueFullFromISR(getScuUartRxTimeoutQueueHandle()) == pdFALSE)
//  REMOVED SINCE it doesn't work with more traffic -->      {
//  REMOVED SINCE it doesn't work with more traffic -->        /* Reception is ongoing */
//  REMOVED SINCE it doesn't work with more traffic -->        scuMsgRx_IT.headerScuRx.dlEvent = SCU_DL_RX_IDLE;
//  REMOVED SINCE it doesn't work with more traffic -->        /* Send a message to the task responsible to manage the timeout condition */
//  REMOVED SINCE it doesn't work with more traffic -->        xQueueSendToBackFromISR(getScuUartRxTimeoutQueueHandle(), (void *)&scuMsgRx_IT, &xHigherPriorityTaskWoken);   // sbcUartRxTimeoutTask()        
//  REMOVED SINCE it doesn't work with more traffic -->      }
      
       UART_SCU_RX_DMA_STREAM->CR &= ~DMA_SxCR_EN;   /* Disabling DMA will force transfer complete interrupt if enabled */
    }
    
    /* In STM32F4xx device, we need to check the Transmit complete flag in order to move the DE pin */
    if (UART_SCU_ISR & UART_FLAG_TC) 
    {   
      /* We want TC flag only */
      /* This part is important */
      /* Clear TC flag */
      __HAL_UART_CLEAR_FLAG(&UART_SCU_HANDLE, UART_FLAG_TC);        
      /* During the FW download, the end of the stream is managed checking the DMA counter */
      //if (getBroadcastDownload() == FALSE)
      if (UART_SCU_TX_DMA_STREAM->NDTR == 0)
      {
        /* Move the DE pin in order to manage the RS-485 communication: enable the RX part */
        HAL_GPIO_WritePin(UART1_DE_GPIO_Port, UART1_DE_Pin, GPIO_PIN_RESET);   
           
      }
    }
    
#endif      
  }
}


/**
*
* @brief       Global interrupt handler for DMA2 stream5 for 
*              USART1 used for Rx from SCU RS485 bus
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/
void UART_SCU_DMA_RX_IRQHandler(void)  
{
  portBASE_TYPE       xHigherPriorityTaskWoken;
  uint8_t*            pData;
  xQueueHandle        hQueue;
  frameSbcRx_st*      pFrameSbcRx;
  headerFrameSbcRx_st hFrameSbcRx;
  frameScuToSbcTx_st  tmpFrameScuToSbcTx;
  uint8_t             ix, cnt, len;
  
  xHigherPriorityTaskWoken = pdFALSE;
  /* Check transfer complete flag */
  if (_IS_SCU_RX_DMA_TC_FLAG) 
  { 
      _CLEAR_SCU_RX_DMA_TC_FLAG;           /* Clear transfer complete flag */      
      /* Calculate number of bytes actually transfered by DMA so far */
      /**
       * Transfer could be completed by 2 events:
       *  - All data actually transfered (NDTR = 0)
       *  - Stream disabled inside USART IDLE line detected interrupt (NDTR != 0)
       *  For both events we send DMA RX buffer using debug queue 
       **/
      pData = (uint8_t*)getDMAptr(PROT_UART_SCU);
      /* Get number of byte received */
      if (_GET_SCU_RX_DMA_CNT < NUM_BUFF_SBC_MSG_RX)        
      {
        scuMsgRx_IT.headerScuRx.totalLen = (uint16_t)(NUM_BUFF_SBC_MSG_RX - _GET_SCU_RX_DMA_CNT); 
        if (scuMsgRx_IT.headerScuRx.totalLen <= SBC_BUFF_SIZE_GSY)
        {
          if ((scuTypeModes == SCU_GSY) && ((scuMsgRx_IT.headerScuRx.totalLen >= SBC_MIN_SIZE_GSY) || (pData[0] == 0x40) || (pData[0] == 0x43))) 
          {
            if ((scuOpMode >= SCU_M_P) || (scuOpMode == SCU_EMUMAX0_S)) /** [1] funzioni ammesse solo 03, 04, 16 */
            {    
              if (scuOpMode == SCU_EMUMAX0_S)
              {
                if ((pData[0] != 0x24) && (pData[0] != 0x40))  /* packet starting with $ or @ refer to MAX0 FW download --> can be skipped */
                {
                  if (((uint8_t*)pData)[2] + 3 == scuMsgRx_IT.headerScuRx.totalLen)
                  {
                    /* possible answer to request FW download*/
                    if ((pData[3] == (uint8_t)0xFC) && (pData[4] == (uint8_t)0xFF) && (scuMsgRx_IT.headerScuRx.totalLen == (uint16_t)7))
                    {
                      /* onother slave has accepted the FW download request */
                      hFrameSbcRx.messageEv = OTHER_SLAVE_IN_DWNL;
                      hFrameSbcRx.totalLen = scuMsgRx_IT.headerScuRx.totalLen;
                      /* I am a master so, after that, it is necessary to: */
                      /* - change baude rate from 19200 to 115200 on RS485 (UART5) and SBC UART (UART1) after the current answer */
                      configASSERT(xQueueSendToBackFromISR(getScuGsyDwldQueueHandle(), (void *)&hFrameSbcRx, &xHigherPriorityTaskWoken) == pdPASS); // scuGsyDwldTask
                    }
                    else
                    {
                      scuMsgRx_IT.headerScuRx.dlEvent = SCU_DL_RECEIVE_FROM_RS485;
                      if (xQueueIsQueueFullFromISR(getScuDataLinkQueueHandle()) == pdFALSE)
                      {
                        /* if one o more bytes has been received, put it in the queue */
                        //configASSERT(xQueueSendToBackFromISR(getScuDataLinkQueueHandle(), (void *)&scuMsgRx_IT, &xHigherPriorityTaskWoken) == pdPASS); // SEM --> scuDlProcess()
                        xQueueSendToBackFromISR(getScuDataLinkQueueHandle(), (void *)&scuMsgRx_IT, &xHigherPriorityTaskWoken); // scuDataLinkTask()
                      }
                    }
                  }
                  else
                  {
                    if (scuMsgRx_IT.headerScuRx.totalLen <= SBC_BUFF_SIZE_GSY)
                    {
                      /* in any case the first byte must be the opening flag */
                      if (pData[0] == GSY_STX)
                      {
                        len = (uint8_t)scuMsgRx_IT.headerScuRx.totalLen;
                        /* a volte è possibile che SBC accorpi  due messaggi uno immediatamente in coda all'altro */
                        scuMsgRx_IT.headerScuRx.totalLen = (uint16_t)((uint8_t*)pData)[2] + 3;
                        if (len >= scuMsgRx_IT.headerScuRx.totalLen)
                        {
                          ix = (uint8_t)scuMsgRx_IT.headerScuRx.totalLen;
                          if (xQueueIsQueueFullFromISR(getScuDataLinkQueueHandle()) == pdFALSE)
                          {
                            //configASSERT(xQueueSendToBackFromISR(getSbcAnswerQueueHandle(), (void *)&sbcMsgRx_IT, &xHigherPriorityTaskWoken) == pdPASS);
                            xQueueSendToBackFromISR(getScuDataLinkQueueHandle(), (void *)&scuMsgRx_IT, &xHigherPriorityTaskWoken);
                            /* extract second messaggio */
                            if (len > (uint8_t)scuMsgRx_IT.headerScuRx.totalLen)
                            {
                              if ((len - (uint8_t)scuMsgRx_IT.headerScuRx.totalLen) >= SBC_MIN_SIZE_GSY)
                              {
                                scuMsgRx_IT.headerScuRx.totalLen = (uint16_t)(len - (uint8_t)scuMsgRx_IT.headerScuRx.totalLen);
                                for (cnt = 0; ix < len; ix++, cnt++)
                                { 
                                  ((uint8_t*)pData)[cnt] = ((uint8_t*)pData)[ix];
                                }
                                /* if one o more bytes has been received, put it in the queue */
                                configASSERT(xQueueSendToBackFromISR(getScuDataLinkQueueHandle(), (void *)&scuMsgRx_IT, &xHigherPriorityTaskWoken) == pdPASS);
                              }
                            }
                          }
                        }
                      }
                      
                      /* Caso comando da macchina di collaudo */
                      else
                      {
                        if (xQueueIsQueueFullFromISR(getScuDataLinkQueueHandle()) == pdFALSE)
                        {
                          scuMsgRx_IT.headerScuRx.dlEvent = SCU_DL_RX_COLLAUDO;
                          xQueueSendToBackFromISR(getScuDataLinkQueueHandle(), (void *)&scuMsgRx_IT, &xHigherPriorityTaskWoken); // scuDlProcess()
                        }
                      }
                    }
                  }
                }
              }
              else
              {
                /* here we are in SEM enviroment */
                scuMsgRx_IT.headerScuRx.dlEvent = SCU_DL_RECEIVE_FROM_RS485;
                if (xQueueIsQueueFullFromISR(getScuDataLinkQueueHandle()) == pdFALSE)
                {
                  /* if one o more bytes has been received, put it in the queue */
                  xQueueSendToBackFromISR(getScuDataLinkQueueHandle(), (void *)&scuMsgRx_IT, &xHigherPriorityTaskWoken); // scuDataLinkTask()  SEM --> scuDlProcess()
                }
              }
            }
            else
            {
              /* caso SCU in emulazione MAX0 Master ossia connesso direttamente a SBC */
              /* bridge del messaggio direttamente verso SBC su UART5 */
              /* we are working in MAX0 master emulation: so the answer must be sent on SBC UART (UART5-DMA1-Stream7) */
              if (scuMsgRx_IT.headerScuRx.totalLen == (uint16_t)7)
              {
                /* possible answer to request FW download*/
                if ((pData[3] == (uint8_t)0xFC) && (pData[4] == (uint8_t)0xFF))
                {
                  /* a slave has accepted the FW download request */
                  if (scuOpMode == SCU_EMUMAX0_M)
                  {
                    hFrameSbcRx.messageEv = SLAVE_IN_DWNL;
                    hFrameSbcRx.totalLen = scuMsgRx_IT.headerScuRx.totalLen;
                    /* I am a master so, after that, it is necessary to: */
                    /* - change baude rate from 19200 to 115200 on RS485 (UART5) and SBC UART (UART1) after the current answer */
                    configASSERT(xQueueSendToBackFromISR(getScuGsyDwldQueueHandle(), (void *)&hFrameSbcRx, &xHigherPriorityTaskWoken) == pdPASS); // scuGsyDwldTask
                  }
                }
              }
              counterSlaveDwnl++;
#ifdef FAST_DMA
              UART_SBC_DMA_Tx((uint8_t*)pData, scuMsgRx_IT.headerScuRx.totalLen);
#else
              tmpFrameScuToSbcTx.msgEv = SCU_NEW_TX_MSG;
              tmpFrameScuToSbcTx.pDataToSend = pData;
              tmpFrameScuToSbcTx.totalLen = scuMsgRx_IT.headerScuRx.totalLen;
              configASSERT(xQueueSendToBackFromISR(getScuToSbcTxQueueHandle(), (void *)&tmpFrameScuToSbcTx, &xHigherPriorityTaskWoken) == pdPASS); // scuTxToSbcTask
#endif
            }
          }
          else
          {
            /* caso SCU in SEM Master ossia connesso direttamente a SBC */
            /* bridge del messaggio direttamente verso SBC su UART5 */
            /* we are working in SEM master: so the answer must be sent on SBC UART (UART5-DMA1-Stream7) */
            if (((pData[3] == (uint8_t)0xFC) && (pData[4] == (uint8_t)0xFF)) && (scuMsgRx_IT.headerScuRx.totalLen == (uint16_t)7))
            {
              if (scuOpMode == SCU_M_P)
              {
                /* possible answer to request FW download*/
                /* a slave has accepted the FW download request */
                hFrameSbcRx.messageEv = SLAVE_IN_DWNL;
                hFrameSbcRx.totalLen = scuMsgRx_IT.headerScuRx.totalLen;
                /* I am a master so, after that, it is necessary to: */
                /* - change baude rate from 19200 to 115200 on RS485 (UART5) and SBC UART (UART1) after the current answer */
                configASSERT(xQueueSendToBackFromISR(getScuGsyDwldQueueHandle(), (void *)&hFrameSbcRx, &xHigherPriorityTaskWoken) == pdPASS); // scuGsyDwldTask
                tmpFrameScuToSbcTx.msgEv = SCU_NEW_TX_MSG;
                tmpFrameScuToSbcTx.pDataToSend = pData;
                tmpFrameScuToSbcTx.totalLen = scuMsgRx_IT.headerScuRx.totalLen;
                configASSERT(xQueueSendToBackFromISR(getScuToSbcTxQueueHandle(), (void *)&tmpFrameScuToSbcTx, &xHigherPriorityTaskWoken) == pdPASS); // scuTxToSbcTask
              }
              else
              {
                /* onother slave has accepted the FW download request */
                hFrameSbcRx.messageEv = OTHER_SLAVE_IN_DWNL;
                hFrameSbcRx.totalLen = scuMsgRx_IT.headerScuRx.totalLen;
                configASSERT(xQueueSendToBackFromISR(getScuGsyDwldQueueHandle(), (void *)&hFrameSbcRx, &xHigherPriorityTaskWoken) == pdPASS); // scuGsyDwldTask
              }
            }
            else
            {
              /* here we are in SEM enviroment */
              scuMsgRx_IT.headerScuRx.dlEvent = SCU_DL_RECEIVE_FROM_RS485;
              if (xQueueIsQueueFullFromISR(getScuDataLinkQueueHandle()) == pdFALSE)
              {
                /* if one o more bytes has been received, put it in the queue */
                xQueueSendToBackFromISR(getScuDataLinkQueueHandle(), (void *)&scuMsgRx_IT, &xHigherPriorityTaskWoken); // scuDataLinkTask()  SEM --> scuDlProcess()
              }
              else
              {
                /* the queue full means a hard problem... reset is the strategy!! */
                activeImmediateReset();
              }
            }
          }
        }
        else
        {
          hQueue = getScuGsyDwldQueueHandle();
          if (hQueue != NULL)
          {
            pFrameSbcRx = (frameSbcRx_st*)&scuMsgRx_IT.headerScuRx.scuRegIdx; /* pointer 3 bytes before Rx message: space for totalLen e messageEv */
            pFrameSbcRx->totalLen = scuMsgRx_IT.headerScuRx.totalLen;
            pFrameSbcRx->messageEv = UART_RX_DWLD;
            /* long message has been received: only for download FW it is possible  scuGsyDwldTask () */
            configASSERT(xQueueSendToBackFromISR(hQueue, (void *)pFrameSbcRx, &xHigherPriorityTaskWoken) == pdPASS);
          }
        }
      } 
      else
      {
        if ((USART_IsEnabledDMAReq_RX(UART_SCU) == 0) && (xQueueIsQueueFullFromISR(getScuDataLinkQueueHandle()) == pdFALSE))
        {
          scuMsgRx_IT.headerScuRx.dlEvent = SCU_DL_ERR_UART;
          scuMsgRx_IT.headerScuRx.totalLen = 0;
         /* an error condition occurred  */
          configASSERT(xQueueSendToBackFromISR(getScuDataLinkQueueHandle(), (void *)&scuMsgRx_IT, &xHigherPriorityTaskWoken) == pdPASS); // SEM --> scuDlProcess()
        }
      }

      /* Prepare DMA for next transfer */
      /* Important! DMA stream won't start if all flags are not cleared first */
      _REINIT_SCU_RX_DMA_CHANNEL((uint32_t)pData);
  }
  else
  {
    pData = (uint8_t*)getDMAptr(PROT_UART_SCU);
    _REINIT_SCU_RX_DMA_CHANNEL((uint32_t)pData);
  }
}

/**
*
* @brief       Starts transmission on protocol Uart1  using 
*              DMA2 Stream7
*
* @param [in]  uint8_t*: pointer to string to be transmitted  
* @param [in]  uint8_t : message len   
*  
* @retval      none 
*  
****************************************************************/
void UART_SCU_DMA_Tx(uint8_t* pTxBuffer, uint16_t len) 
{
  UART_HandleTypeDef *huart;

  huart = &UART_SCU_HANDLE;

  /* Uart dbg must be enabled */
  if (UART_CheckIdleState(huart) != HAL_OK)
  {
    /* Transfer error in transmission process */
    uartScuError_Handler(__FILE__, __LINE__);
    
  }

  if (HAL_DMA_GetState(huart->hdmatx) != HAL_DMA_STATE_READY)
  {
    return;
  }
  
#ifdef GD32F4xx    
  /* Enable the Transmit complete interrupt in order to move the DE pin required to manage the RS-485 RX phase */
  SET_BIT(huart->Instance->CR1, USART_CR1_TCIE);
  /* Since it's managed like an RS485, we have to move the DE pin to HIGH level */
  HAL_GPIO_WritePin(UART1_DE_GPIO_Port, UART1_DE_Pin, GPIO_PIN_SET);  
#endif    
  /*## Start the transmission process #####################################*/
  /* User start transmission data through "pTxBuffer" buffer */
  //if(HAL_UART_Transmit_DMA(&huart6, (uint8_t*)pTxBuffer, len)!= HAL_OK)
  if(HAL_UART_Transmit_DMA(huart, (uint8_t*)pTxBuffer, len)!= HAL_OK)
  {
    /* Transfer error in transmission process */
    uartScuError_Handler(__FILE__, __LINE__); 
  }
}

/*
  * @brief  Check RX pin level.
  * @param [in]  None
  *  
  * @retval      FALSE --> Low level detected for at least 50 samples --> possible collision
  *              TRUE --> High level detect --> no collision

  */
GPIO_PinState UART1_RX_Pin_Check (void)
{
  uint8_t LOW_LEVEL_cnt = 0;
    
  /* intercept LOW level of UART RX (probable collision) */
  if (HAL_GPIO_ReadPin(UART1_RX_GPIO_Port, UART1_RX_Pin) == GPIO_PIN_RESET)
  {
    /* at least 50 samples of LOW level detected --> someone is doing TX --> collision */
    while (LOW_LEVEL_cnt < 50)
    {
      if (HAL_GPIO_ReadPin(UART1_RX_GPIO_Port, UART1_RX_Pin) == GPIO_PIN_RESET)
        LOW_LEVEL_cnt++;
      else
        break;        
    } 
    
  } 

  /* Check number of samples */
  if (LOW_LEVEL_cnt < 50)
    return GPIO_PIN_SET;          /* pin RX @ high level */
  else
    return GPIO_PIN_RESET;        /* pin RX @ low level */
  
}

/*
  * @brief  Generate usec  delay Based on SysTick->VAL (decrement in the time).
  * @param [in]  uint8_t : duration in usec of the check  
  *  
  * @retval      FALSE --> no collison intercepted 
  *              TRUE --> possible collision intercepted

  */
uint8_t UART1_Collision_Check (uint16_t usDelay)
{
  uint32_t  toSysTick;
  
  /* Adjust in MHz */
  uint8_t SysCoreClockRef = SystemCoreClock / 1000000;
    
  if (SysTick->VAL > (SysCoreClockRef * usDelay))
  {
    /* remember: sysTick is a timer with decrement */
    toSysTick = SysTick->VAL - (SysCoreClockRef * usDelay);
  }
  else
  {
    toSysTick = (SysCoreClockRef * 1000) - ((SysCoreClockRef * usDelay) - SysTick->VAL);
           
   /* Wait for the first part of the delay */
    while (((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != SysTick_CTRL_COUNTFLAG_Msk))
    {
      /* Check if RX pin is at low level --> POSSIBLE COLLISION */
      if (UART1_RX_Pin_Check() == GPIO_PIN_RESET)
        return TRUE;
    }
        
  }

  /* Wait for the rest of delay */
  while (SysTick->VAL > toSysTick  )
  {
      /* Check if RX pin is at low level --> POSSIBLE COLLISION */
     if (UART1_RX_Pin_Check() == GPIO_PIN_RESET)
       return TRUE;
  }

  /* NO COLLISION detected */
  return FALSE;
  
}

/**
*
* @brief       Starts immediatly on  Uart1  using DMA2 Stream7
*
* @param [in]  uint8_t*: pointer to string to be transmitted  
* @param [in]  uint8_t : message len   
*  
* @retval      none 
*  
****************************************************************/
void txOnRs485Bus(uint8_t* pTxBuffer, uint16_t len) 
{
    
  uint8_t Collision = FALSE, Coll_happened = FALSE;
  
  /************** COLLISION CHECK  *********************/
  /*** check collision on UART bus for at least 50uS ***/
  /*****************************************************/
  
  do {
    
    /* Check if there is a collision on the bus for 50uS */
    Collision = UART1_Collision_Check (50);
    if (_GET_SCU_RX_DMA_CNT < NUM_BUFF_SBC_MSG_RX) Collision = TRUE;
    /* Collision ? */
    if (Collision == FALSE)
    {
      /* If a collision happens */
      if (Coll_happened == TRUE)
        /* Wait for 2ms */
        osDelay(2);        
      /* NO --> Start transmission on RS485 bus using DMA */
      UART_SCU_DMA_Tx((uint8_t*)pTxBuffer, len); 
    }
    else
    {
      /* a collision happens */
      Coll_happened = TRUE;
      /* If slave do a random delay */
      if (getScuOpMode() > SCU_M_P) 
        /* Wait for a random delay */
        osDelay(getRandomDelay());  /* delay in the range 2...50 msec */
      else
        /* Wait for a fixed delay */
        osDelay(2);
    }
      /* and retry */
  } while (Collision == TRUE);
  
}

/**
*
* @brief       Get the pointer for the SCU Rx DMA transfer 
*
* @param [in]  none  
*  
* @retval      uint8_t*: the Rx DMA pointer as first element in the MODBUS answer
*  
****************************************************************/
uint8_t* getBuffRxScu (void)
{
  return ((uint8_t*)&scuMsgRx_IT.message);
}

/**
*
* @brief       free buffer area acquired by malloc to semd a 
*              message over SCU RS485 bus
*
* @param [in]  none  
*  
* @retval      none
*  
****************************************************************/
void freeTxBuffer (void)
{
  portBASE_TYPE xHigherPriorityTaskWoken;

  if (pMallocTx485 != NULL)
  {
    free(pMallocTx485);
  }
  pMallocTx485 = NULL;
  xSemaphoreGiveFromISR(uartScuSinapsiTxSemaphoreHandle, &xHigherPriorityTaskWoken);
}

/**
*
* @brief        Set the pointer used to send on RS485 bus
*
* @param [in]   uint8_t*: pointer to defined  buffer
*
* @retval       none
*
***********************************************************************************************************************/
void setMallocTx485(uint8_t* pBuff)
{
   pMallocTx485 = pBuff;
}


/**
*
* @brief       get info for reg starting from reg index and SCU 
*              index 
*
* @param [in]  uint16_t: SCU index 1..16  
* @param [in]  scuAllRegIdx_e: enumerate for reg  
* @param [in]  uint16_t*: pointer where store the register len 
*        in bytes
*  
* @retval      uint8_t*: pointer where the reg info is stored 
*  
****************************************************************/
uint8_t* getRegInfoFromIdx (uint16_t scuIdx, scuAllRegIdx_e scuRegIdx, uint16_t* pLen)
{
  uint16_t rAddr;
  uint8_t* pDest;

  rAddr = scuRegister[scuRegIdx].regAdd;
  /* the frame coming from data link is OK, so copy the received info in the SCU's modbus map   */
  pDest = (uint8_t*)((uint32_t)&scuMapRoRegister[scuIdx] + (uint32_t)2 * (uint32_t)rAddr);
  *pLen = (scuRegister[scuRegIdx].size * 2);
  return(pDest);
}

/**
*
* @brief       check if this message is for this SCU 
*
* @param [in]  uint8_t: address inside the message   
* @param [in]  uint16_t: modbus address inside the message   
*  
* @retval      uint8_t: 1 if the message is for this SCU, 0 
*              otherwise
*  
****************************************************************/
static uint8_t isThisMsgForUs (uint8_t addrId, uint16_t modbusAddr)
{
  uint8_t   esito;

  esito = FALSE;
  if (isSemMasterFz() == TRUE)
  {
    esito = TRUE;
  }
  else
  {
    if (modbusAddr == ADDR_EM_LOVATO_V_SYS) return(FALSE);
     
    if ((scuOpMode == SCU_S_P ) && ((addrId == scuAddr) || (addrId == (scuAddr + 1))))
    {
      esito = TRUE;
    }
    else
    {
      if (((scuOpMode == SCU_S_S ) && (addrId == scuAddr + 1)) || (addrId == MODBUS_BROADCAST_ADDR))
      {
        esito = TRUE;
      }
      else
      {
        if (((scuOpMode == SCU_S_S ) && (addrId == SCU_M_P_ADDR) && ((uint8_t)(modbusAddr / SIZE_MODBUS_MAP) == scuAddr)))
        {
          /* nella fase che precede l'assegnazione dell'indirizzo i messaggi diretti al master devono esere ignorati */
          esito = TRUE;
        }
        else
        {
          if ((addrId == scuAddr + 1))
          {
            esito = TRUE;
          }
          else
          {
            if ((scuOpMode == SCU_EMUMAX0_S) && (addrId == scuAddr))
            {
              /* testing with automatic machine */
              esito = TRUE;
            }
          }
        }
      }
    }
  }
  return(esito);
}

/**
*
* @brief       Get the current SCU operation mode 
*
* @param [in]  none  
*  
* @retval      scuOpModes_e: SCU operation mode code 
*  
****************************************************************/
scuOpModes_e  getScuOpMode (void)
{
  return (scuOpMode);
}

/**
*
* @brief       Check if this SCU has master role 
*
* @param [in]  none  
*  
* @retval      uint8_t: SCU has a master role 
*  
****************************************************************/
uint8_t isSemMasterFz (void)
{
  if ((scuOpMode == SCU_M_P) || (scuOpMode == SCU_M_STAND_ALONE))
  {
    return(TRUE);
  }
  return(FALSE);
}


/**
*
* @brief       Get the current SCU operation mode 
*
* @param [in]  scuOpModes_e: SCU operation mode code  
*  
* @retval      none 
*  
****************************************************************/
void setScuOpMode (scuOpModes_e mode)
{
  scuOpMode = mode;
}

/**
*
* @brief       Get the current SCU type mode 
*
* @param [in]  none  
*  
* @retval      scuTypeModes_e: SCU type mode code 
*  
****************************************************************/
scuTypeModes_e getScuTypeMode (void)
{
  return (scuTypeModes);
}

/**
*
* @brief       Get the current SCU type mode 
*
* @param [in]  scuTypeModes_e: SCU type mode code  
*  
* @retval      none 
*  
****************************************************************/
void setScuTypeMode (scuTypeModes_e typeMode)
{
  scuTypeModes = typeMode;
}

/**
*
* @brief       Get the current SCU type mode from EEPROM 
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/
void setScuTypeModeFromEeprom (void)
{
  uint8_t tmpVal;

  eeprom_param_get(OPERATIVE_MODE_EADD, (uint8_t *)&tmpVal, 1);
  scuTypeModes = (scuTypeModes_e)tmpVal;
}

/**
*
* @brief       Get the current SCU address mode 
*
* @param [in]  none  
*  
* @retval      scuAddModeSktNum_e: SCU address type mode code 
*              FIXED / ADJUSTABLE
*  
****************************************************************/
scuAddModeSktNum_e getScuAddressTypeMode (void)
{  
  /* In Emumax0, address type is only fixed  */
  switch (getScuOpMode ())
  {    
    case SCU_EMUMAX0_M:
    case SCU_EMUMAX0_S:
      return SCU_FIXED_ADDR;
    /* In SEM mode, address type should be fixed or variable */
    default:   
      return (scuAddressMode);          
  }   
}

/**
*
* @brief       set to FIXED the current SCU address mode 
*
* @param [in]  none  
*  
* @retval      uint8_t: 0 when no error  
*  
****************************************************************/
uint8_t setScuAddressTypeMode(scuAddModeSktNum_e addrType)
{
  uint8_t semFlag, result;

  scuAddressMode = addrType;

  /* read current value for SEM Flags */
  eeprom_param_get(SEM_FLAGS_CTRL_EADD, (uint8_t*)&semFlag, 1);
 
  semFlag &= (~SCU_ADDR_MODE_MASK);
  semFlag |= addrType;
  /* read current value for SEM Flags */
  // xx eeprom_array_set(SEM_FLAGS_CTRL_EADD, (uint8_t*)&semFlag, 1);  
  result = EEPROM_Save_Config (SEM_FLAGS_CTRL_EADD, (uint8_t*)&semFlag, 1);

  if (result == (uint8_t)osOK)
  {
    result = setPmRemoteSemFlag(TRUE);
  }
  return(result);
}

/**
*
* @brief       Get the number of the socket inside the product 
*              derivated from fake code 
*
* @param [in]  none  
*  
* @retval      uint8_t: number of socket 
*  
****************************************************************/
uint8_t getNumSktInProduct (void)
{
  return (numPrdSocket);
}




/**
*
* @brief       check if SEM  mode 
*
* @param [in]  none  
*  
* @retval      uint8_t: TRUE if SEM 
*  
****************************************************************/
uint8_t isSemMode (void)
{
  if((scuTypeModes == SCU_SEM_M) || (scuTypeModes == SCU_SEM_MS) || 
     (scuTypeModes == SCU_SEM_S) || (scuTypeModes == SCU_SEM_STAND_ALONE)) 
    return(TRUE); 
  else 
    return(FALSE);
}


/**
*
* @brief       Get the current SCU modbus address  
*
* @param [in]  none  
*  
* @retval      uint8_t: logical modbus addres (index for modbus 
*              structure)
*  
****************************************************************/
uint8_t getLogicalMdbAddr (void)
{
  return (scuAddr);
}

/**
*
* @brief       Get the current SCU device id  
*
* @param [in]  uint8_t: modbus address 1...247  
*  
* @retval      uint8_t: logical modbus addres (index for modbus 
*              structure)
*  
****************************************************************/
uint8_t getLogicalMdbAddrSem (void)
{
  uint8_t idDev;

  if (isSemMode() == TRUE)
  {
    idDev = fromRs485ToSem(scuAddr + 1);  // from physical address 1..247 to device id 0...15
  }
  else
  {
    idDev = scuAddr;
  }
  return (idDev);
}


/**
*
* @brief       Get the current SCU modbus physical address  
*
* @param [in]  none  
*  
* @retval      uint8_t: physical modbus addres 1..31 (index for
*              modbus structure)
*  
****************************************************************/
uint8_t getPhysicalMdbAddr (void)
{
  return (scuAddr + 1);
}


uint16_t getScuModbusTableId ( uint16_t idFromInterface )
{
  if( isSemMode() == TRUE )
  {
    return fromRs485ToSem( idFromInterface );
  }
  else
  {
    return ( idFromInterface - 1 );
  }
}


/**
*
* @brief       Get the current Read Only modbus Register 
*              address 
*
* @param [in]  uint8_t: lofical address  
*  
* @retval      scuRoMapRegister_st*: pointer to Read only 
*              modbus register 
*  
****************************************************************/
scuRoMapRegister_st* getRoMdbRegs (uint8_t addr)
{
  return (&scuMapRoRegister[addr]);
}


/**
*
* @brief       Get the current Read Write modbus Register 
*              address 
*
* @param [in]  uint8_t: logical address  
*  
* @retval      scuRwMapRegister_st*: pointer to Read Write 
*              modbus register 
*  
****************************************************************/
scuRwMapRegister_st* getRwMdbRegs (uint8_t addr)
{
  return (&scuMapRwRegister[addr]);
}

/**
*
* @brief       Get the current Testing Areaa Read Write modbus 
*              Register address
*
* @param [in]  uint8_t: logical address  
*  
* @retval      tmMapRegister_st*: pointer to Read Write modbus 
*              register
*  
****************************************************************/
tmMapRegister_st* getTmMdbRegs (uint8_t addr)
{
  return (&tmMapRegister[addr]);
}

/**
*
* @brief       Get the current App register of Modbus map
*
* @param [in]  uint8_t: logical address  
*  
* @retval      scuRwMapRegister_st*: pointer to Read Write 
*              modbus register 
*  
****************************************************************/
appMapRwRegister_st* getAppMdbRwRegs (uint8_t addr)
{
  return (&appMapRwRegister[addr]);
}


/**
*
* @brief       Get the pointer to Sinapsi info area 
*
* @param [in]  none  
*  
* @retval      rseSetReg_st*: pointer to RW Sinapsi structure  
*              
*  
****************************************************************/
rseSetReg_st* getRWSinapsiInfo (void)
{
  return (&scuMapRwRegister[SCU_M_P_ADDR_LOG].rseSetReg);
}

/**
*
* @brief       Reset IOM2G Control reg 
*
* @param [in]  none  
*  
* @retval      none  
*              
*  
****************************************************************/
void resetSinapsiIOM2CtrlReg (void)
{
  scuMapRwRegister[SCU_M_P_ADDR_LOG].rseSetReg.iomCtrl = (uint16_t)0;
}

#ifdef NON_USATA_EVS_MNG_687
/**
*
* @brief       set in the modbus map the current value of input 
*              signal 
*
* @param [in]  uint8_t: input signal Id  
* @param [in]  GPIO_PinState: input signal status  
*  
* @retval      none 
*  
****************************************************************/
void  setModbusInpReg (uint8_t inpSig, GPIO_PinState state)
{
  uint16_t bitMask, bitVal;
  uint8_t  addrLog;
  uint16_t *pReg;

  addrLog = scuAddr;
  bitMask = (uint16_t)0;
  pReg = NULL;
  switch (inpSig)
  {
    case IN1_REMOTE_EXP0:
      /* nel sistema di collaudo PEN corrisponde allo stato del coperchio */
      bitMask = LID_MASK;
      bitVal = (state == GPIO_PIN_SET) ? bitMask : (uint16_t)0;
      pReg = (uint16_t*)&scuMapRoRegister[addrLog].scuMapRegNotify.ntfErr1;
      break;

    case IN4_RCBO_EXP0:
      /* nel sistema di collaudo PEN corrisponde allo stato del relè del PEN */
      bitMask = RCBO_MASK;
      bitVal = (state == GPIO_PIN_SET) ? bitMask : (uint16_t)0;
      pReg = (uint16_t*)&scuMapRoRegister[addrLog].scuMapRegNotify.ntfErr1;
      break;

    case PEN_ALARM:
      /* nel sistema di collaudo PEN corrisponde allo stato di allarme PEN  */
      bitMask = PEN_ERR_MASK;
      bitVal = (state == GPIO_PIN_SET) ? bitMask : (uint16_t)0;
      pReg = (uint16_t*)&scuMapRoRegister[addrLog].scuMapRegNotify.ntfErr2;
      break;

    case IN3_MIRROR_EXP0:
      /* nel SEM bit  MIRROR Mask is 0x0040 */
      bitMask = MIRROR_SEM_MASK;
      bitVal = (state == GPIO_PIN_SET) ? bitMask : (uint16_t)0;
      pReg = (uint16_t*)&scuMapRoRegister[addrLog].scuMapRegNotify.ntfErr1;
      break;

    default: 
      break;
    
  }
  if (pReg != NULL)
  {
    *pReg &= (uint16_t)~bitMask;   // reset the bit
    *pReg |= (uint16_t)bitVal;     // set the bit if necessary
  }
}
#endif

/**
*
* @brief       set in the modbus testing error area the current
*              value of the error
*
* @param [in]  uint8_t: input signal Id  
* @param [in]  GPIO_PinState: input signal status  
*  
* @retval      none 
*  
****************************************************************/
void  setModbusErrorTesting (uint8_t inpSig, GPIO_PinState state)
{
  uint16_t bitMask, bitVal;
  uint8_t  addrLog;
  uint16_t *pReg;

  addrLog = scuAddr;
  bitMask = (uint16_t)0;
  pReg = NULL;
  switch (inpSig)
  {
    case IN3_MIRROR_EXP0:
      /* nel SEM bit  MIRROR Mask is 0x0040 */
      bitMask = MIRROR_SEM_MASK;
      bitVal = (state == GPIO_PIN_SET) ? bitMask : (uint16_t)0;
      pReg = (uint16_t*)&tmMapRegister[addrLog].tmError1InTest;
      break;

    case RCDM_IN_UP_PIN_UP:
      /* nel SEM bit  RCDM Mask is 0x0001 */
      bitMask = RCDM_SEM_MASK;
      bitVal = (state == GPIO_PIN_SET) ? bitMask : (uint16_t)0;
      pReg = (uint16_t*)&tmMapRegister[addrLog].tmError1InTest;
      break;

    case IN2_PULSANTE_UP_PIN_UP:
      /* nel SEM bit  PULS Mask is 0x8000: 06-12-23 abbiamo in Eltec deciso di trattare il pulsante come un allarme  */
      bitMask = PULS_SEM_MASK;
      bitVal = (state == GPIO_PIN_SET) ? bitMask : (uint16_t)0;
      pReg = (uint16_t*)&tmMapRegister[addrLog].tmError1InTest;
      break;

    default: 
      break;
    
  }
  if (pReg != NULL)
  {
    *pReg &= (uint16_t)~bitMask;   // reset the bit
    *pReg |= (uint16_t)bitVal;     // set the bit if necessary
  }
}

/**
*
* @brief       set in the modbus map the current station 
*              charging status 
*
* @param [in]  uint16_t: station status  
*  
* @retval      none 
*  
****************************************************************/
void  setStationStatusInModbusMapApp (uint16_t state) 
{
  uint8_t               idDev;

  /* get address on modbus and relative modbus pointer area   */
  idDev = getLogicalMdbAddrSem();
  
  if( isSemMode() == FALSE )
  {
    getRoMdbRegs( idDev )->scuMapRegStatusMeas.ntfChgStat = state;
  }
}

/**
*
* @brief       get from modbus map the current register status 
*
* @param [in]  uint16_t: station status  
*  
* @retval      uint16_t: the request register value  
*  
****************************************************************/
uint16_t  getRegisterFromModbusMap (uint16_t addr) 
{
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;
  uint16_t              result;

  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  //pRoRegs = getRoMdbRegs(mdbAddr-1); // Nick nel modbus salva la SCU di idx=1 nella struct 0.
  pRoRegs = getRoMdbRegs(mdbAddr);

  result = (uint16_t)0;

  switch (addr)
  {
    case ADDR_EVSE_EVENT_FLAGS_RO:
      result = pRoRegs->scuMapRegStatusMeas.ntfSktEvent; // set event connector 
      break;

    case ADDR_EVSE_CHARGE_STATUS_RO:
      result = pRoRegs->scuMapRegStatusMeas.ntfChgStat;  // get current socket state 
      break;

    default:
      break;
  }

  return(result);
}


/**
*
* @brief       get the current station charging status from Modbus map 
*  
*  
* @retval      uint16_t current station charging status 
*  
****************************************************************/
uint16_t  getStationStatusFromModbusMap () 
{
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;
  uint16_t              mdbState;
  uint8_t               evseState;

  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRoRegs = getRoMdbRegs(mdbAddr);
  mdbState = pRoRegs->scuMapRegStatusMeas.ntfChgStat; // get state 
  
  /* Convert from modbus value to EVSE state */
  switch(mdbState)
  {
   case INITIAL_EVSE_STATE:
    evseState = EVSTATE_INIT;
    break;
    
   case IDLE_STATE:
    evseState = EVSTATE_SOCKET_AVAILABLE;
    break;
    
   case PREPARING_STATE:
    evseState = EVSTATE_SOCKET_CHECK;
    break;
    
   case EV_CONNECTED_STATE:
    evseState = EVSTATE_S2_WAITING;
    break;
    
   case CHARGING_STATE:
    evseState = EVSTATE_CHARGING;
    break;
    
   case SUSPENDED_EVSE_STATE:
   case SUSPENDED_EV_STATE:             // Fixed SCU-28
   case SUSPENDED_NOPOWER_STATE:
    evseState = EVSTATE_SUSPENDING;
    break;
    
   case END_CHARGE_STATE:
    evseState = EVSTATE_PLUG_OUT;
    break;
    
  }
  return evseState;
}


/**
*
* @brief       set in the modbus map the current station 
*              charging event 
*
* @param [in]  uint16_t: station event  
*  
* @retval      none 
*  
****************************************************************/
void  setStationEventInModbusMap (uint16_t event) 
{
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;

  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRoRegs = getRoMdbRegs(mdbAddr);
  pRoRegs->scuMapRegStatusMeas.ntfSktEvent = event; // set new state 
}


/**
*
* @brief       get the current station charging status from Modbus map 
*  
*  
* @retval      uint16_t current station charging status 
*  
****************************************************************/
uint16_t  getStationEventFromModbusMap () 
{
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;

  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRoRegs = getRoMdbRegs(mdbAddr);
  return pRoRegs->scuMapRegStatusMeas.ntfSktEvent; // get state 
}


/**
*
* @brief       set in the modbus map the current value set by rotatory switch 
*
* @param [in]  rotaryPos_e: position rotatory switch  
*  
* @retval      none 
*  
****************************************************************/
void  setModbusRotarySwitchCurr (rotaryPos_e position) 
{
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;
  uint8_t               value;
  
  switch(position)
  {
    
   /* Caso 6A */ 
   case POS0_6A:
    value = 6;
    break;
    
   /* Caso 10A */ 
   case POS1_10A:
    value = 10;
    break;
  
   /* Caso 13A */ 
   case POS2_13A:
    value = 13;
    break;

   /* Caso 16A */ 
   case POS3_16A:
    value = 16;
    break;
    
   /* Caso 20A */ 
   case POS4_20A:
    value = 20;
    break;
  
   /* Caso 25A */ 
   case POS5_25A:
    value = 25;
    break;

   /* Caso 32A */ 
   case POS6_32A:
    value = 32;
    break;
    
   /* Caso 40A */ 
   case POS7_40A:
    value = 40;
    break;
  
   /* Caso 50A */ 
   case POS8_50A:
    value = 50;
    break;    
    
   /* Caso 63A */ 
   case POS9_63A:
    value = 63;
    break; 

   default:    
    value = 6;
    break;

  }
  
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  //pRoRegs = getRoMdbRegs(mdbAddr-1); // Nick nel modbus salva la SCU di idx=1 nella struct 0.
  pRoRegs = getRoMdbRegs(mdbAddr);
  pRoRegs->scuMapRegStatusMeas.rswCurr = value;  
}


/**
*
* @brief       set in the modbus map the max power value 
*              resulting from calculations
*
* @param [in]  uint16_t: station status  
*  
* @retval      none 
*  
****************************************************************/
void  setModbusMaxPowerResult (uint32_t value) 
{
  appMapRwRegister_st*  pAppRwRegs;
  uint8_t               mdbAddr;
  
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pAppRwRegs = getAppMdbRwRegs(mdbAddr);
  pAppRwRegs->maxPower = value;
}

/**
*
* @brief       set in the modbus map the max current value 
*              resulting from calculations
*
* @param [in]  uint16_t: station status  
*  
* @retval      none 
*  
****************************************************************/
void setModbusMaxCurrentResult (uint16_t value)
{
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;
  
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRoRegs = getRoMdbRegs(mdbAddr);
  pRoRegs->scuMapRegStatusMeas.pwrCurr = value; 
}


/**
*
* @brief       set in the modbus map the session number ID
*
* @param [in]  uint16_t: station status  
*  
* @retval      none 
*  
****************************************************************/
void  setSessionId (unsigned int numSession) 
{
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;
  uint8_t               devAlias;

  mdbAddr = getLogicalMdbAddrSem();
  devAlias = getPhysicalMdbAddr();  // we use the physical address on RS485 in LSB session Id
  
  pRoRegs = getRoMdbRegs(mdbAddr);
  //pRoRegs->scuMapRegStatusMeas.evSessionId = (devAlias << 24) | (numSession);  
  pRoRegs->scuMapRegStatusMeas.evSessionId = (uint32_t)((uint32_t)(devAlias) | (uint32_t)(numSession << 8));  
  if (isSemMode() == TRUE)
  {
    saveSessionId(pRoRegs->scuMapRegStatusMeas.evSessionId);
    /* in according to modbus V24 when session Id is creted the time in charge must be cleared  */
    pRoRegs->scuMapRegStatusMeas.timeInCharge = (uint32_t)0;
    /* send the info to notify manager */
    sendEventToSemMng(NOTIFY_TO_MASTER_TX, ADDR_SESSION_ID_RO);
  }
}


/**
*
* @brief       get from the modbus map the session number ID
*
* @param [in]    
*  
* @retval      none 
*  
****************************************************************/
uint32_t  getSessionId () 
{
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;
  
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();  
  pRoRegs = getRoMdbRegs(mdbAddr);
  return pRoRegs->scuMapRegStatusMeas.evSessionId;  
}



/**
*
* @brief       set in the modbus map the harware flags
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  setHwFlags () 
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  uint16_t              hwFlag = HW_FLAGS_UNDEFINED;
  uint8_t               httpFlags = 0;
  
   eeprom_param_get(LCD_TYPE_EADD, &httpFlags, 1);
  
   /* Bit 0 differenziabile riarmabile abilitato */
   if((httpFlags & DIRI_ON) == DIRI_ON)
   {
     hwFlag |= DIFF_RIARM_FLAG;
   }
   
   /* Bit 1 Presenza LCD */
   if((httpFlags & LCD_2X20) == LCD_2X20)
   {
     hwFlag |= DISPLAY_LCD_FLAG;
   }
   
   /* Bit 2 Lettore RFID sempre presente */
   hwFlag |= RFID_READER_FLAG;
   
   /* Bit 3 modulo WIFI abilitato */
   if((httpFlags & WIFI_ON) == WIFI_ON)
   {
     hwFlag |= WIFI_CONNECTION_FLAG;
   }
   
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  pRwRegs->scuSetRegister.hwcFlags = hwFlag; 
}

/**
*
* @brief       set in the modbus map the number of leds 
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  setNumberOfLeds (void) 
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr, ledNum, ledCode;

  /* set energy meter type  from STRIP_LED_TYPE_EADD   */
  eeprom_param_get(STRIP_LED_TYPE_EADD, (uint8_t *)&ledNum, 1);
  switch (ledNum)
  {
    case LED_STRIP_18:
      ledCode = 6;  
      break;
    case LED_STRIP_09:   
      ledCode = 3;  
      break;
    case LED_STRIP_12:
      ledCode = 4;  
      break;
    default: 
    case LED_STRIP_06: 
      ledCode = 2;  
      break;
  }
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  pRwRegs->scuSetRegister.ledStrip = (uint16_t)ledCode; 
}

/**
*
* @brief       get the hardware flags from modbus map
*
* @param [in]  
*  
* @retval      uint16_t hardware flags 
*  
****************************************************************/
uint16_t getHwFlags () 
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;

  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  return pRwRegs->scuSetRegister.hwcFlags; 
}


/**
*
* @brief       set in the modbus map the internal and external energy meter type
*
* @param [in]    
*  
* @retval      none 
*  
****************************************************************/
void  setEnergyMetersType () 
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  energy_meter_e        extEm;
  energy_meter_e        intEm;
  
  intEm = getStationEmTypeInt();
  extEm = getStationEmTypeExt();
  
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
    
  pRwRegs->scuSetRegister.mtType = (extEm << 8) | (intEm); 
#ifdef NON_SERVE
  /* In SEM energy meter type is never removed, can be only changed  */
  if ((getScuOpMode() > SCU_M_P) && (isModbusManagerActive() == TRUE))
  {
    /* SEM enviroment: slave SCU must comunicate the new energy meter type to master */
    /* send the info to notify manager */
    sendEventToSemMng(NOTIFY_TO_MASTER_TX, ADDR_ENERGY_METERS_RW);
  }
#endif
}

/**
  * @brief  set board serial number (8 digit, ex: 00013440)  
  *         
  * @param  char*: product serial number
  * @param  uint8_t: length of the product serial number (8 digit, ex: 00013440)
  * 
  * @retval none
  */
void iniBoardSerialNumber(char* boardSn, uint8_t length)
{
  scuRwMapRegister_st*  pRwRegs;
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;
  
  mdbAddr = getLogicalMdbAddrSem();
  pRoRegs = getRoMdbRegs(mdbAddr);
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  strncpy((char*)pRoRegs->scuMapRegInfoVer.devSn, boardSn , length);
  strncpy((char*)pRwRegs->scuSetRegister.devSn, boardSn , length);
}



/**
  * @brief  set product serial number   
  *         
  * @param  char*: product serial number
  * @param  uint8_t: length of the product serial number (9 digit)
  * 
  * @retval none
  */
void iniProductSerialNumber(char* productSn, uint8_t length)
{
  scuRwMapRegister_st*  pRwRegs;
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRoRegs = getRoMdbRegs(mdbAddr);
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  memcpy(pRoRegs->scuMapRegInfoVer.prodSn, productSn, length);
  memcpy(pRwRegs->scuSetRegister.prdSn, productSn, length);
}

/**
  * @brief  Init product code string   
  *         
  * @param  char*: product code string pointer
  * @param  uint8_t: length of the product code
  * 
  * @retval none
  */
void iniProductCodeString(char* productCode, uint8_t length)
{
  scuRwMapRegister_st*  pRwRegs;
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRoRegs = getRoMdbRegs(mdbAddr);
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  memcpy(pRoRegs->scuMapRegInfoVer.prodCode, productCode, length);
  memcpy(pRwRegs->scuSetRegister.prdCode, productCode, length);
}

/**
  * @brief  Init fake code string   
  *         
  * @param  char*: fake code string pointer
  * @param  uint8_t: length of the fake code
  * 
  * @retval none
  */
void iniProductFakeString(char* fakeCode, uint8_t length)
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  memcpy(pRwRegs->scuSetRegister.fakePrdCode, fakeCode, length);
}


/**
  * @brief  set board serial number (8 digit, ex: 00013440)  
  *         
  * @param  char*: product serial number
  * @param  uint8_t: length of the product serial number (8 digit, ex: 00013440)
  * 
  * @retval none
  */
void setBoardSerialNumber(char* boardSn, uint8_t length)
{
  uint8_t               mdbAddr;
  scuRwMapRegister_st*  pRwRegs;
  scuRoMapRegister_st*  pRoRegs;
  
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  pRoRegs = getRoMdbRegs(mdbAddr);
  
  strncpy((char*)pRwRegs->scuSetRegister.devSn, boardSn , length);
  strncpy((char*)pRoRegs->scuMapRegInfoVer.devSn, boardSn , length);
}

/**
  * @brief  set product code (max 24 alfa-numeric  ex: 204.CA23B-T2T2W1)  
  *         
  * @param  char*: product code
  * @param  uint8_t: length of the product code
  * 
  * @retval none
  */
void setProductCodeString(char* pProductCode, uint8_t length)
{
  uint8_t               mdbAddr;
  scuRwMapRegister_st*  pRwRegs;
  scuRoMapRegister_st*  pRoRegs;
    
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  pRoRegs = getRoMdbRegs(mdbAddr);
  
  memset((char*)pRwRegs->scuSetRegister.prdCode, 0x00, PRODUCT_CODE_LENGTH);
  memset((char*)pRoRegs->scuMapRegInfoVer.prodCode, 0x00, PRODUCT_CODE_LENGTH);
  strncpy((char*)pRwRegs->scuSetRegister.prdCode, pProductCode , length);
  strncpy((char*)pRoRegs->scuMapRegInfoVer.prodCode, pProductCode , length);

  setStationProductCode(pProductCode, length);

}

/**
  * @brief  set product code (max 24 alfa-numeric  ex: 204.CA23B-T2T2W1)  
  *         
  * @param  char*: product code
  * @param  uint8_t: length of the product code
  * 
  * @retval none
  */
void setBoardSnHwVerString(char* pBoardStr, uint8_t length)
{
  uint8_t               mdbAddr;
  scuRwMapRegister_st*  pRwRegs;
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               SerNum[4];

  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  pRoRegs = getRoMdbRegs(mdbAddr);
  
  memset(getStationSerialNumber(), 0x00, BOARD_SN_LENGTH);
  memset((char*)pRwRegs->scuSetRegister.devSn, 0x00, LEN_BOARD_SN_RW  * 2);
  strncpy((char*)pRwRegs->scuSetRegister.devSn, (char*)&pBoardStr[2], BOARD_SN_LENGTH);
  strncpy(getStationSerialNumber(), (char*)&pBoardStr[2], BOARD_SN_LENGTH);
  memset((char*)pRoRegs->scuMapRegInfoVer.devSn, 0x00, LEN_BOARD_SN_RO  * 2);
  strncpy((char*)pRoRegs->scuMapRegInfoVer.devSn, (char*)&pBoardStr[2] , BOARD_SN_LENGTH);
  /* save in EEPROM also  */
  SerNum[3] = 0; SerNum[3] = (pBoardStr[9] - '0'); SerNum[3] |= ((pBoardStr[8] - '0') << 4);  /* BCD, first two LSB digit  */
  SerNum[2] = 0; SerNum[2] = (pBoardStr[7] - '0'); SerNum[2] |= ((pBoardStr[6] - '0') << 4);  
  SerNum[1] = 0; SerNum[1] = (pBoardStr[5] - '0'); SerNum[1] |= ((pBoardStr[4] - '0') << 4);  
  SerNum[0] = 0; SerNum[0] = (pBoardStr[3] - '0'); SerNum[0] |= ((pBoardStr[2] - '0') << 4);  /* BCD, first two MSB digit  */
  EEPROM_Save_Config (SERNUM_BYTE0_EADD, SerNum, 4);
  /* save SCU SN also in reserved area in EEPROM */
  if (setScuSerialNumberEeprom((char*)SerNum, (char*)&pBoardStr[2]) != 0)
  {
    tPrintf( "Error writing SCU SN\r\n");
  }

  /* set versione HW and manifacture */
  memset((char*)pRoRegs->scuMapRegInfoVer.hwRev, 0x00, LEN_HW_REVISION_RO  * 2);
#ifndef HW_MP28947  
  if (pBoardStr[0] == 'K')
  {
    pBoardStr[13] -= 0x20; /* scheda Sacchi lettere minuscole  */
  }
  pBoardStr[12] = pBoardStr[13];
  strncpy((char*)pRoRegs->scuMapRegInfoVer.hwRev, (char*)&pBoardStr[11] , 2);
  if (setScuHardwareVersionEeprom((char*)&pBoardStr[11]) != 0)
  {
    tPrintf( "Error writing HwVer!\r\n");
  }
#else
  /* Usiamo il default per MP28947          */
  /* 01234567890123 String must be 14 chars */
  /* S 00012345 7-A */
  /*         destination       source                len */
  strncpy((char*)pRoRegs->scuMapRegInfoVer.hwRev, (char*)"7R" , 2);
  if (setScuHardwareVersionEeprom((char*)"7A") != 0)
  {
    tPrintf( "Error writing HwVer!\r\n");
  }
#endif
}




/**
  * @brief  set product serial number   
  *         
  * @param  char*: product serial number
  * @param  uint8_t: length of the product serial number (9 digit: 100xxxxxx)
  * 
  * @retval none
  */
void setProductSerialNumber(char* productSn, uint8_t length)
{
  scuRwMapRegister_st*  pRwRegs;
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  pRoRegs = getRoMdbRegs(mdbAddr);
  
  memcpy(pRwRegs->scuSetRegister.prdSn, productSn, length);
  memcpy(pRoRegs->scuMapRegInfoVer.prodSn, productSn, length);
}



/**
  * @brief  set product serial number   
  *         
  * @param  
  * 
  * @retval char* pointer of product serial number
  */
char* getProductSerialNumber()
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  uint8_t               prdCurrent[9]; 
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  if (ReadFromEeprom(PRD_SN_EE_ADDRES, prdCurrent, 9) == osOK)    // recupero product SN 
  {
    if (memcmp((void*)prdCurrent, (void*)pRwRegs->scuSetRegister.prdSn, 9) != 0)
    {
      /*      destination                             source          length */
      memcpy((void*)pRwRegs->scuSetRegister.prdSn, (void*)prdCurrent, (size_t)9);
    }
  }
  return (char*)pRwRegs->scuSetRegister.prdSn;
}


/**
  * @brief  set product code   
  *         
  * @param  
  * 
  * @retval char* pointer of product code
  */
char* getProductCode()
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  return (char*)pRwRegs->scuSetRegister.prdCode;
}


/**
  * @brief  set fake product code  
  *         
  * @param  char*: fake product code
  * @param  uint8_t: length of the product code (14 digit)
  * 
  * @retval none
  */
void setTecnProductCode(char* fakeCode, uint8_t length)
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);

  memset(pRwRegs->scuSetRegister.fakePrdCode, 0x00, sizeof(pRwRegs->scuSetRegister.fakePrdCode));
  memcpy(pRwRegs->scuSetRegister.fakePrdCode, fakeCode, length);

 /* save new parameter in EEPROM */
  (void)setStationFakeProductCode(fakeCode, length);

}


/**
  * @brief  get fake product code   
  *         
  * @param  
  * 
  * @retval char* pointer of product code
  */
char* getFakeProductCode()
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  return (char*)pRwRegs->scuSetRegister.fakePrdCode;
}


/**
*
* @brief       set in the modbus map an error
*
* @param [in]  uint16_t: error code for modbus v20
* @param [in]  uint8_t: error register ERROR1 or ERROR2  
*  
* @retval      none 
*  
****************************************************************/
void  setErrorModbus (uint16_t error, uint8_t errorReg) 
{
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;
  
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRoRegs = getRoMdbRegs(mdbAddr);
  
  if(errorReg == 1)
  {
    pRoRegs->scuMapRegNotify.ntfErr1 |= error ; 
  }
  else if(errorReg == 2)
  {
    pRoRegs->scuMapRegNotify.ntfErr2 |= error;
  }
}


/**
*
* @brief       set in the modbus map the station operation mode (FREE, PERSONAL or NET)
*
* @param [in]    
*  
* @retval      none 
*  
****************************************************************/
void  setStationOperationMode () 
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  uint8_t               modeStat;
  uint16_t              modeModbus;
  
  modeStat = getStationModeWorking();
  
  /* Switch case needed because Modbus v19.2 changes order of byte */
  switch(modeStat)
  {
   case FREE:
    modeModbus = OP_MODE_FREE | MASTER_MODE_LOCAL;
    break;
    
   case PERSONAL:
    modeModbus = OP_MODE_PERSONAL | MASTER_MODE_LOCAL;
    break;
    
   case NET:
    modeModbus = OP_MODE_NET | MASTER_MODE_LOCAL;
    break;
    
   case OCPP:
   /* Ref modbus map par 7.2.1.14, in OCPP mode, the high byte has the value 2 */
   /* Also Low byte is filled with value NET */
    modeModbus = OP_MODE_NET | MASTER_MODE_OCPP;  
    break;
      
  }
  
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  pRwRegs->scuSetRegister.evseMod = modeModbus; 
}



/**
*
* @brief       set in the modbus map the status/presence of the sinapsi board
*              0 = Not present, 1 = present but Bluetooth OFF, 2 = Sinapsi Bluetooth ON
*
* @param [in]  uint8_t = availability of the socket  
*  
* @retval      none 
*  
****************************************************************/
void  setSinapsiStatusForApp (uint8_t state) 
{
  appMapRwRegister_st*  pAppRwRegs;
  uint8_t               mdbAddr;

  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pAppRwRegs = getAppMdbRwRegs(mdbAddr);
  pAppRwRegs->btSinapsiStatus = state;
}
  
  
/**
*
* @brief       set in the modbus map the socket availability
*
* @param [in]  uint8_t = power outage type  
*  
* @retval      none 
*  
****************************************************************/
void  setPowerOutage (uint8_t power) 
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;

  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  pRwRegs->scuSetRegister.pwrOutage = power; 
}

/**
*
* @brief       Get Power stop mode: MONITOR, BACKUP, PEN 
*
* @param [in]  None
*  
* @retval      uint8_t = power outage type   
*  
****************************************************************/
uint16_t getPowerOutage (void) 
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;

  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  return pRwRegs->scuSetRegister.pwrOutage; 
}

/**
*
* @brief       set in the modbus map the board fw version
*
* @param [in]  char*: string of fw version
*  
* @retval      none 
*  
****************************************************************/
void  setFwVersion (char* fwVersion) 
{
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;

  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRoRegs = getRoMdbRegs(mdbAddr);
  memcpy(pRoRegs->scuMapRegInfoVer.mfwVer, fwVersion, strlen(fwVersion));
  /*              destination                             source                                         6 = len("Vw.ya\0" or "Vw.y\0")   */
  memcpy(pRoRegs->scuMapRegInfoVer.bootfwVer, ((fwInfoVersion_u *)fwVersion)->fwBootVer.bootVer, sizeof(((fwInfoVersion_u *)fwVersion)->fwBootVer.bootVer));
}


/**
*
* @brief       set in the modbus map the board hardware version
*
* @param [in]  char*: string of hw version
*  
* @retval      none 
*  
****************************************************************/
void  setHwVersion (char* hwVersion) 
{
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;

  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRoRegs = getRoMdbRegs(mdbAddr);
  memcpy(pRoRegs->scuMapRegInfoVer.hwRev, hwVersion, strlen(hwVersion));
}

/**
*
* @brief       set the current modbus version used 
*
* @param [in]  uint16_t: modbus version
*  
* @retval      none 
*  
****************************************************************/
void  setModbusVersion (uint16_t  mdbVersion) 
{
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;

  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRoRegs = getRoMdbRegs(mdbAddr);
  pRoRegs->scuMapRegInfoVer.modbusVersion = mdbVersion;
}




/**
*
* @brief       set in the modbus map the mifare module fw version
*
* @param [in]  char*: string of mifare module fw version
*  
* @retval      none 
*  
****************************************************************/
void  setMifareFwVersion (char* MifarefwVersion) 
{
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr, len;

  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRoRegs = getRoMdbRegs(mdbAddr);
  len = (uint8_t)strlen(MifarefwVersion);
  if (len != 0)
  {
    memcpy(pRoRegs->scuMapRegInfoVer.rfdVer, MifarefwVersion, len);
  }
  else
  {
    memset(pRoRegs->scuMapRegInfoVer.rfdVer, 0x00, (LEN_MIFARE_FW_VERSION_RO * 2));
  }
}


/**
*
* @brief       set in the modbus map the connector number in the EVSE
*
* @param [in]  uint8_t connector position in the EVSE
* @param [in]  uint8_t connector number in the EVSE
*  
* @retval      none 
*  
****************************************************************/
void  setConnectorNumber (uint8_t connPos, uint8_t connNumb) 
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;

  if (connNumb >= 4) connNumb = 1;

  configASSERT((connPos <= SKT_LOW_DX_MB) && (connNumb <= 4));

  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  pRwRegs->scuSetRegister.connNumb = ((uint16_t)connPos + (uint16_t)connNumb * (uint16_t)0x100);  // MSB = connector number LSB = Connector position
}

/**
*
* @brief       set in the modbus map the connector type in the 
*              EVSE
*
* @param [in]  none
*  
* @retval      none 
*  
****************************************************************/
void  setConnectorType (void) 
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;

  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  /* Tipo di presa */
  pRwRegs->scuSetRegister.connType = getStationSocketType();
}


/**
*
* @brief       set in the modbus map the max typical current
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
uint16_t  setMaxTypicalCurrent (void) 
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  uint8_t               current;

  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  eeprom_param_get(M3T_CURRENT_EADD, (uint8_t *)&current, 1);  
  
  pRwRegs->scuSetRegister.maxTypCurr = current;
  
  return(current);
}

/**
*
* @brief       save in eeprom the max typical current
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  saveMaxTypicalCurrentEeprom (uint8_t current) 
{
  EEPROM_Save_Config (M3T_CURRENT_EADD, &current, 1);
}

/**
*
* @brief       save in eeprom the max simplified current
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  saveMaxSimplifiedCurrentEeprom (uint8_t current) 
{
  EEPROM_Save_Config (M3S_CURRENT_EADD, &current, 1);
}


/**
*
* @brief       save in eeprom the Power Management mode
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  savePmModeEeprom (uint8_t pmMode) 
{
  EEPROM_Save_Config (PMNG_MODE_EADD, &pmMode, 1);
}

                  
/**
*
* @brief       save in eeprom the Power Management pMax parameter
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  savePmPmaxEeprom (uint8_t* pMax) 
{
  
  uint8_t pMaxLsb = pMax[0];
  uint8_t pMaxMsb = pMax[1];
  
  /* save PMAX value as KW * 10 unit measure  *****/
  // xx eeprom_array_set(PMNG_PWRLSB_EADD, &pMaxLsb, 1);
  EEPROM_Save_Config (PMNG_PWRLSB_EADD, &pMaxLsb, 1);
  // xx eeprom_array_set(PMNG_PWRMSB_EADD, &pMaxMsb, 1);
  EEPROM_Save_Config (PMNG_PWRMSB_EADD, &pMaxMsb, 1);
  // xx send_to_eeprom(EEPROM_UPDATE);
}

/**
*
* @brief       save in eeprom the Power Management Imin parameter
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  savePmIminEeprom (uint8_t iMin) 
{
  EEPROM_Save_Config (PMNG_CURRENT_EADD, &iMin, 1);
}


/**
*
* @brief       save in eeprom the Power Management hPower parameter
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  savePmHpowerEeprom (uint8_t hPower) 
{
  uint8_t hPowerSave = hPower - 1;
  EEPROM_Save_Config (PMNG_MULTIP_EADD, &hPowerSave, 1);
}


/**
*
* @brief       save in eeprom the Power Management dSet parameter
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  savePmDsetEeprom (uint8_t* dset) 
{
  uint16_t dsetTotal = dset[1] << 8 | dset[0];
  uint8_t dsetSave = dsetTotal / 100;
  
  EEPROM_Save_Config (PMNG_ERROR_EADD, &dsetSave, 1);
}


/**
*
* @brief       save in eeprom the Power Management dMax parameter
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  savePmDmaxEeprom (uint8_t dMax) 
{
  EEPROM_Save_Config (PMNG_DMAX_EADD, &dMax, 1);
}


/**
*
* @brief       save in eeprom the display default language
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  saveDefaultDisplayLanguageEeprom (uint8_t lang) 
{
  //xx eeprom_array_set(LANG_DEFAULT_EADD, &lang, 1);
  EEPROM_Save_Config (LANG_DEFAULT_EADD, &lang, 1);
  lcd_language_set(lang);
  lcd_language_def_update();
}


/**
*
* @brief       save in eeprom the display allowed languages
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  saveDisplayLanguagesEeprom (uint8_t* langs) 
{
  // xx eeprom_array_set(LANG_CONFIG0_EADD, (uint8_t *)langs, 4);
  EEPROM_Save_Config (LANG_CONFIG0_EADD, (uint8_t *)langs, 4);
  // xx send_to_eeprom(EEPROM_UPDATE);
}


/**
*
* @brief       save in eeprom the connector alias 0..15
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  saveConnectorAliasEeprom (uint8_t alias) 
{  
  alias -= 1;
  /* Check if the message is for the proper SCU */
  if (alias != scuAddr)
    return;
  // xx eeprom_array_set(RS485_ADD_EADD, &alias, 1);
  EEPROM_Save_Config (RS485_ADD_EADD, &alias, 1);
  // xx send_to_eeprom(EEPROM_UPDATE);
}


/**
*
* @brief       save in eeprom the connector type
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  saveConnectorTypeEeprom (uint8_t conn) 
{
  uint8_t connSave = modbusToHttpSocketConverter(conn);
  // xx eeprom_array_set(SOCKET_TYPE_EADD, &connSave, 1);
  EEPROM_Save_Config (SOCKET_TYPE_EADD, &connSave, 1);
  // xx send_to_eeprom(EEPROM_UPDATE);
}


/**
*
* @brief       convert socket type from modbus value to http page value
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
uint8_t  modbusToHttpSocketConverter (uint8_t conn) 
{
  uint8_t result = 0x00;
  
  switch(conn)
  {
  case PRESA_TIPO_3A:
    result = SOCKET_3A_OPEN_LID;
    break;
    
  case CONNETTORE_TETHERED_TIPO_1:
    result = SOCKET_T1_TETHERED;
    break;
    
  case CONNETTORE_TETHERED_TIPO_2:
    result = SOCKET_T2_TETHERED;
    break;
    
  case PRESA_SCHUKO:
    result = SOCKET_SK_NO_LID;
    break;
    
  case PRESA_TIPO_2:
    result = SOCKET_T2_OPEN_LID;
    break;
  }
  
  return result;
}


/**
*
* @brief       save in eeprom the internal and external energy meters type
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  saveEnergyMetersTypeEeprom (uint8_t* em) 
{
  uint8_t intEm = 0x00;
  uint8_t extEm = 0x00;
  
  intEm = modbusToHttpEnergyMeterConverter(em[0]);
  extEm = modbusToHttpEnergyMeterConverter(em[1]);
  
  // xx eeprom_array_set(EMETER_INT_EADD, &intEm, 1);
  // xx eeprom_array_set(HIDDEN_MENU_ENB_EADD, &extEm, 1);
  EEPROM_Save_Config (EMETER_INT_EADD, &intEm, 1);
  EEPROM_Save_Config (HIDDEN_MENU_ENB_EADD, &extEm, 1);
  
  // xx send_to_eeprom(EEPROM_UPDATE);
}


/**
*
* @brief       convert socket type from modbus value to http page value
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
uint8_t  modbusToHttpEnergyMeterConverter (uint8_t em) 
{
  uint8_t result = 0x00;
  
  switch(em)
  {
  case TA:
    result = EMETER_TAMP;
    break;
    
  case MONO_GAVAZZI:
    result = EMETER_MONO_PH_GAVAZZI;
    break;
    
  case MONO_ALGO2:
    result = EMETER_MONO_PH_ALGO2;
    break;
    
  case MONO_LOVATO:
    result = EMETER_MONO_PH_LOVATO;
    break;
    
  case MONO_SCAME:
    result = EMETER_MONO_PH_SCAME;
    break;
  
    case TA_TRI:
    result = EMETER_TAMP_3;
    break;

  case TRI_GAVAZZI:
    result = EMETER_THREE_PH_GAVAZZI;
    break;

  case TRI_ALGO2:
    result = EMETER_THREE_PH_ALGO2;
    break;

  case TRI_LOVATO:
    result = EMETER_THREE_PH_LOVATO;
    break;

  case TRI_SCAME:
    result = EMETER_THREE_PH_SCAME;
    break;
    
  case MONO_SINAPSI:
    result = HIDDEN_MENU_SINAPSI;
  }
  
  return result;
}


/**
*
* @brief       save in eeprom the power management visibility flag
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  saveVisibleFlagsEeprom (uint8_t flags) 
{
  // xx eeprom_array_set(HIDDEN_MENU_VIS_EADD, &flags, 1);
  EEPROM_Save_Config (HIDDEN_MENU_VIS_EADD, &flags, 1);
  // xx send_to_eeprom(EEPROM_UPDATE);
}


/**
*
* @brief       save in eeprom the max time of charge
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  saveChargeTimeEeprom (uint8_t timeEnum) 
{
  // xx eeprom_array_set(TCHARGE_TIME_EADD, &timeEnum, 1);
  EEPROM_Save_Config (TCHARGE_TIME_EADD, &timeEnum, 1);
  // xx send_to_eeprom(EEPROM_UPDATE);
}


/**
*
* @brief       save in eeprom the power management flags
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  savePowerManagementFlagsEeprom (uint8_t pmFlags) 
{
  uint8_t valueToSave = 0x00;
  // Attivazione PM stazione bit n. 0
  if((pmFlags & 0x01) == 0x01)
  {
    valueToSave |= 0x01;
    // xx eeprom_array_set(HIDDEN_MENU_ENB_EADD, &valueToSave, 1);
    EEPROM_Save_Config (HIDDEN_MENU_ENB_EADD, &valueToSave, 1);
  }
  // Sbilanciamento carico su L1 bit n. 1
  if((pmFlags & 0x02) == 0x02)
  {
    valueToSave = 0x01;
    // xx eeprom_array_set(PMNG_UNBAL_EADD, &valueToSave, 1);
    EEPROM_Save_Config (PMNG_UNBAL_EADD, &valueToSave, 1);
  }
  // Attivazione PM remoto bit n. 2
  if((pmFlags & 0x04) == 0x04)
  {
    valueToSave |= 0x08;
    // xx eeprom_array_set(HIDDEN_MENU_ENB_EADD, &valueToSave, 1);
    EEPROM_Save_Config (HIDDEN_MENU_ENB_EADD, &valueToSave, 1);
  }
  // Funzionalità Time Range bit n. 15
  if((pmFlags & 0x80) == 0x80)
  {
    valueToSave = 0x01;
    // xx eeprom_array_set(PMNG_TRANGE_EADD, &valueToSave, 1);
    EEPROM_Save_Config (PMNG_TRANGE_EADD, &valueToSave, 1);
  }
  // xx send_to_eeprom(EEPROM_UPDATE);
}


/**
*
* @brief       save in eeprom the max energy of charge
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  saveChargeEnergyEeprom (uint8_t enrgEnum) 
{
  // xx eeprom_array_set(ENRG_LIMIT_EADD, &enrgEnum, 1);
  EEPROM_Save_Config (ENRG_LIMIT_EADD, &enrgEnum, 1);
  // xx send_to_eeprom(EEPROM_UPDATE);
}


/**
*
* @brief       set in the modbus map the uid authorization code from App
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void setUidAuthorizationByApp(char* uidApp) 
{
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;

  mdbAddr = getLogicalMdbAddrSem();
  pRoRegs = getRoMdbRegs(mdbAddr);
  
  /* 0xFD codice modbus per identificare app Wi-Fi */
  pRoRegs->scuMapRegStatusMeas.uidAuth[0] = 0x0A;                       /* JAPPT-239: UID len as in Modbus specifications */
  pRoRegs->scuMapRegStatusMeas.uidAuth[1] = UID_AUTHORIZATION_BY_APP;   /* JAPPT-239: CARD type as in Modbus specifications */
  
  memcpy(&pRoRegs->scuMapRegStatusMeas.uidAuth[2], uidApp, 10);

}


/**
*
* @brief       get in the modbus map the uid authorization code from App
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void getUidAuthorizationByCard(char* dest) 
{
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;
  
 
  mdbAddr = getLogicalMdbAddrSem();
  pRoRegs = getRoMdbRegs(mdbAddr);
  
  memcpy(dest, &pRoRegs->scuMapRegStatusMeas.uidAuth[2], 8);

}


/**
*
* @brief       set in the modbus map the uid authorization code from card
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void setUidAuthorizationByCard(uint8_t* uidCard) 
{
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;
  char                  uidChar[8];

  mdbAddr = getLogicalMdbAddrSem();
  pRoRegs = getRoMdbRegs(mdbAddr);
  
  memset(&pRoRegs->scuMapRegStatusMeas.uidAuth[0], 0x00, 10);
  
  sprintf(&uidChar[0], "%x", (char)uidCard[0]);
  sprintf(&uidChar[2], "%x", (char)uidCard[1]);
  sprintf(&uidChar[4], "%x", (char)uidCard[2]);
  sprintf(&uidChar[6], "%x", (char)uidCard[3]);
 
  pRoRegs->scuMapRegStatusMeas.uidAuth[0] = CARD_UID_DIM;
  memcpy(&pRoRegs->scuMapRegStatusMeas.uidAuth[2], uidChar, 8);
}


/**
*
* @brief       set in the modbus map the max simplified current
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  setMaxSimplifiedCurrent () 
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  uint8_t               current;

  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  eeprom_param_get(M3S_CURRENT_EADD, (uint8_t *)&current, 1);  
  
  pRwRegs->scuSetRegister.maxSimplCurr = current;
}

/**
*
* @brief       set in the modbus map the max simplified current
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
uint16_t getMaxTypicalCurrent ()
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  return pRwRegs->scuSetRegister.maxTypCurr;
}

/**
*
* @brief       get Max Simplified Current from modbus struct
*
* @param [in]  
*  
* @retval      value
*  
****************************************************************/

uint16_t getMaxSimplifiedCurrent ()
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  return pRwRegs->scuSetRegister.maxSimplCurr;
}

/**
*
* @brief       set in the modbus map the max simplified current
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  setUtcDateTimeRegister () 
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  uint8_t               timezone;
  uint8_t               dst;

  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  DateTimeGet(&utcTimeScu);
  
  /* Get timezone saved in eeprom */
  eeprom_param_get(TIME_ZONE_EADD, (uint8_t*)&timezone, 1);
    
  /* Get dst flag saved in eeprom */
  eeprom_param_get(DST_EADD, (uint8_t*)&dst, 1);
  
  pRwRegs->scuSetRegister.rtcInf[0] = utcTimeScu.Second;    /* Ticket SCU-100  - Start */
  pRwRegs->scuSetRegister.rtcInf[1] = 0; 
  pRwRegs->scuSetRegister.rtcInf[2] = utcTimeScu.Minute;    
  pRwRegs->scuSetRegister.rtcInf[3] = 0; 
  pRwRegs->scuSetRegister.rtcInf[4] = utcTimeScu.Hour;
  pRwRegs->scuSetRegister.rtcInf[5] = 0; 
  pRwRegs->scuSetRegister.rtcInf[6] = utcTimeScu.Day;
  pRwRegs->scuSetRegister.rtcInf[7] = 0; 
  pRwRegs->scuSetRegister.rtcInf[8] = utcTimeScu.Month - 1;  /* In modbus map, month starts from 0 */
  pRwRegs->scuSetRegister.rtcInf[10] = (utcTimeScu.Year - 1900);  
  pRwRegs->scuSetRegister.rtcInf[11] = ((utcTimeScu.Year - 1900) & 0xFF00) >> 8;  
  pRwRegs->scuSetRegister.rtcInf[12] = utcTimeScu.DayWeek;  
  pRwRegs->scuSetRegister.rtcInf[16] = dst;              /* Ticket SCU-100 - End */
   
  pRwRegs->scuSetRegister.rtcTimeZone = timezone * 60;
}


/**
*
* @brief       set in the modbus map the max simplified current
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  setSchedulationInModbusTable (sck_schedule_t *schedulations)
{
  appMapRwRegister_st*  pAppRwRegs;
  uint8_t               mdbAddr;
  
  mdbAddr = getLogicalMdbAddrSem();
  pAppRwRegs = getAppMdbRwRegs(mdbAddr);
  
  if( schedulations[0].enable )
    pAppRwRegs->schFlag1 = SCHEDULATION_ENABLE_BIT | schedulations[0].days;
  else
    pAppRwRegs->schFlag1 = schedulations[0].days;
  pAppRwRegs->schStart1 = (schedulations[0].start_hour << 8) | schedulations[0].start_min;
  pAppRwRegs->schStop1 = (schedulations[0].end_hour << 8) | schedulations[0].end_min;
  pAppRwRegs->schPower1 = schedulations[0].power;
  
  if( schedulations[1].enable )
    pAppRwRegs->schFlag2 = SCHEDULATION_ENABLE_BIT | schedulations[1].days;
  else
    pAppRwRegs->schFlag2 = schedulations[1].days;
  pAppRwRegs->schStart2 = (schedulations[1].start_hour << 8) | schedulations[1].start_min;
  pAppRwRegs->schStop2 = (schedulations[1].end_hour << 8) | schedulations[1].end_min;
  pAppRwRegs->schPower2 = schedulations[1].power;
  
  if( schedulations[2].enable )
    pAppRwRegs->schFlag3 = SCHEDULATION_ENABLE_BIT | schedulations[2].days;
  else
    pAppRwRegs->schFlag3 = schedulations[2].days;
  pAppRwRegs->schStart3 = (schedulations[2].start_hour << 8) | schedulations[2].start_min;
  pAppRwRegs->schStop3 = (schedulations[2].end_hour << 8) | schedulations[2].end_min;
  pAppRwRegs->schPower3 = schedulations[2].power;
  
  if( schedulations[3].enable )
    pAppRwRegs->schFlag4 = SCHEDULATION_ENABLE_BIT | schedulations[3].days;
  else
    pAppRwRegs->schFlag4 = schedulations[3].days;
  pAppRwRegs->schStart4 = (schedulations[3].start_hour << 8) | schedulations[3].start_min;
  pAppRwRegs->schStop4 = (schedulations[3].end_hour << 8) | schedulations[3].end_min;
  pAppRwRegs->schPower4 = schedulations[3].power;
}


/**
*
* @brief       set in the modbus map the update firmware result
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  setUpdateFirmwareByAppResult () 
{
  appMapRwRegister_st*  pAppRwRegs;
  uint8_t               mdbAddr;
  uint8_t               esitoUpdate;

  mdbAddr = getLogicalMdbAddrSem();
  pAppRwRegs = getAppMdbRwRegs(mdbAddr);
    
  esitoUpdate = getEsitoUpdateFw(); 
  
  /* Qualsiasi valore diverso da 0x01 (esito OK) per ora viene messo a 0 nella mappa modbus */
  if(esitoUpdate == 0x02)
  {
    esitoUpdate = 0x00;
    
    setEsitoUpdateFw(esitoUpdate);
  }
  
  
  pAppRwRegs->fwUpdateStatus = esitoUpdate;
}


/**
*
* @brief       set in the modbus map the power management register
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  setPowerManagementRegisters () 
{
  scuRwMapRegister_st*    pRwRegs;
  uint8_t                 mdbAddr;
  uint8_t                 pmOn, pmUnbal, pmImin, lsbPmax, msbPmax, pmHpower, pmDset, pmDmax;
  uint16_t                pmPmax;

  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
    
  
  /* Set registro PM_FLAGS_RW */
//  eeprom_param_get(PMNG_MODE_EADD, (uint8_t *)&pmOn, 1);
  eeprom_param_get(HIDDEN_MENU_ENB_EADD, (uint8_t *)&pmOn, 1);
  pmOn &= HIDDEN_MENU_PMNG_ENB;
  eeprom_param_get(PMNG_UNBAL_EADD, (uint8_t *)&pmUnbal, 1);
  pRwRegs->scuSetRegister.pmFlags = pmOn | (pmUnbal << 1);
  // Set in eeprom
  eeprom_param_get(PMNG_TRANGE_EADD, &pmOn, 1); 
  // Set modbus register
  pRwRegs->scuSetRegister.pmFlags |= pmOn << PM_TIME_RANGE_FUNC_BIT_POS;
  
  /* Set registro PM_PMAX_RW */
  eeprom_param_get(PMNG_PWRLSB_EADD, (uint8_t*)&lsbPmax, 1);
  eeprom_param_get(PMNG_PWRMSB_EADD, (uint8_t*)&msbPmax, 1);
  pmPmax = (msbPmax << 8) | lsbPmax;
  pRwRegs->scuSetRegister.pmPmax = pmPmax;
  
  /* Set registro PM_IMIN_RW */
  eeprom_param_get(PMNG_CURRENT_EADD, (uint8_t*)&pmImin, 1);
  pRwRegs->scuSetRegister.pmImin = pmImin;
  
  /* Set registro PM_HPOWER_RW */
  eeprom_param_get(PMNG_MULTIP_EADD, (uint8_t*)&pmHpower, 1);
  pRwRegs->scuSetRegister.pmHpower = pmHpower + 1;
  
  /* Set registro PM_DSET_RW */
  eeprom_param_get(PMNG_ERROR_EADD, (uint8_t*)&pmDset, 1);
  pRwRegs->scuSetRegister.pmDset = pmDset;
  
  /* Set registro PM_DMAX_RW */
  eeprom_param_get(PMNG_DMAX_EADD, (uint8_t*)&pmDmax, 1);
  pRwRegs->scuSetRegister.pmDmax = pmDmax;
}

/**
*
* @brief       set in the modbus map the nominal station power 
*              from  the typical current 
*
* @param [in]    
*  
* @retval      none 
*  
****************************************************************/
uint8_t setNominalPower (uint16_t ITypical) 
{
  uint32_t              resVoltage;
  uint32_t              resPower;
  appMapRwRegister_st*  pAppRwRegs;
  energy_meter_e        value;
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;

  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  
  pAppRwRegs = getAppMdbRwRegs(mdbAddr);
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  value = getStationEmTypeInt(); 
  
  switch (value)
  {
   case MONO_GAVAZZI:
   case MONO_ALGO2:
   case MONO_LOVATO:
   case TA:
   case MONO_SCAME:
    resVoltage = 230;
    resPower = ITypical * resVoltage;
    break;
    
   case TRI_GAVAZZI:
   case TRI_ALGO2:
   case TRI_LOVATO:
   case TA_TRI:
   case TRI_SCAME:
    resVoltage = 400;
    resPower = (uint16_t)(ITypical * resVoltage * 1.7320);
    break;
    
   default:
    resVoltage = 230;
    resPower = ITypical * resVoltage;
    break;
  }
  
  pAppRwRegs->maxPower = resPower;
  pRwRegs->scuSetRegister.limMaxPowerAc = resPower;

  return ((uint8_t)((pRwRegs->scuSetRegister.limMaxPowerAc / (uint32_t)100) + 1));
  
}

/**
*
* @brief       get the boot event saved in eeprom and write it in the modbus map
*
* @param [in]    
*  
* @retval      none 
*  
****************************************************************/
void setModbusBootEvent () 
{
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;
  bootReg_e             bootEvent;
  
  /* get boot event saved in the eeprom */
  bootEvent = getBootEvent();
  
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRoRegs = getRoMdbRegs(mdbAddr);
  pRoRegs->scuMapRegStatusMeas.evRstFlag = bootEvent;
}


/**
*
* @brief       set in the modbus map the update firmware result
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  setEvsePowerMode () 
{
  appMapRwRegister_st*  pAppRwRegs;
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  energy_meter_e        value;
  uint8_t valueToRegister;

  mdbAddr = getLogicalMdbAddrSem();
  pAppRwRegs = getAppMdbRwRegs(mdbAddr);
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  value = (energy_meter_e)pRwRegs->scuSetRegister.mtType;
  
  switch (value)
  {
   case MONO_GAVAZZI: 
   case MONO_ALGO2:
   case MONO_LOVATO:
   case TA:
   case MONO_SCAME:
   case MONO_PA775:
    valueToRegister = 0x01;
    break;
    
   case TRI_GAVAZZI:
   case TRI_ALGO2:
   case TRI_LOVATO:
   case TA_TRI:
   case TRI_SCAME:
   case TRI_PA775:
    valueToRegister = 0x02;
    break;
    
   default:
    valueToRegister = 0x00;
    break;
  }
    
  pAppRwRegs->evsePwrMode = valueToRegister;
}


/**
*
* @brief       set in the modbus map hardware checks and the actuators
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  setHwChecks(uint16_t checks1, uint16_t checks2) 
{
  scuRwMapRegister_st*    pRwRegs;
  uint8_t              mdbAddr;

  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  pRwRegs->scuSetRegister.hwChecks1 = checks1;
  pRwRegs->scuSetRegister.hwChecks2 = checks2;
}


/**
*
* @brief       get from the modbus map the hardware checks1
*
* @param [in]    
*  
* @retval      none 
*  
****************************************************************/
uint16_t getHwChecks1 (void)
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();  
  pRwRegs = getRwMdbRegs(mdbAddr);
  return pRwRegs->scuSetRegister.hwChecks1;  
}


/**
*
* @brief       get from the modbus map the hardware checks2
*
* @param [in]    
*  
* @retval      none 
*  
****************************************************************/
uint16_t getHwChecks2 (void)
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();  
  pRwRegs = getRwMdbRegs(mdbAddr);
  return pRwRegs->scuSetRegister.hwChecks2;  
}


/**
*
* @brief       get from the modbus map the hardware checks2
*
* @param [in]    
*  
* @retval      none 
*  
****************************************************************/
uint16_t getHwActuators (void)
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();  
  pRwRegs = getRwMdbRegs(mdbAddr);
  return pRwRegs->scuSetRegister.hwActuators;  
}

/**
*
* @brief       get from the modbus map the max temporary power 
*              AC coming fro SEM load balacing 
*
* @param [in]    
*  
* @retval      none 
*  
****************************************************************/
uint32_t getMaxTempPowerAc (void)
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();  
  pRwRegs = getRwMdbRegs(mdbAddr);
  return pRwRegs->scuSetRegister.maxTempPowerAc;  
}


/**
*
* @brief       set modbus register HW_ACTUATORS_RW
*
* @param [in]    
*  
* @retval      none 
*  
****************************************************************/
void setHwActuators (uint8_t actEeprom)
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr, tmp;
  uint16_t              value = 0;
  
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();  
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  /* Get LCD_TYPE address from eeprom, where WIFI (actuator) is saved */
  eeprom_param_get(LCD_TYPE_EADD, (uint8_t *)&tmp, 1);
  
  if((actEeprom & BLOCK_ATT0) == BLOCK_ATT0)
  {
    value |= ACT_BLCK;
  }
  
  if((actEeprom & CONTACT_ATT0) == CONTACT_ATT0)
  {
    value |= ACT_MIRR;
  }
  
  if((actEeprom & RCBO_ATT0) == RCBO_ATT0)
  {
    value |= ACT_RCBO;
  }
  
  if((actEeprom & BBCK_ATT0) == BBCK_ATT0)
  {
    value |= ACT_BACK;
  }
  
  /* Check if WIFI is enabled or not */
  if((tmp & WIFI_ON) == WIFI_ON)
    value |= ACT_WIFI;        /* Enable WIFI actuator */    
  
  pRwRegs->scuSetRegister.hwActuators = value;
}


/**
*
* @brief       save modbus value HW_ACTUATORS_RW in Eeprom
*
* @param [in]    
*  
* @retval      none 
*  
****************************************************************/
void setActuatorsInEeprom (uint16_t actModbus)
{
  
  uint8_t               value = 0;
    
  if((actModbus & ACT_BLCK) == ACT_BLCK)
  {
    value |= BLOCK_ATT0;
  }
  
  if((actModbus & ACT_MIRR) == ACT_MIRR)
  {
    value |= CONTACT_ATT0;
  }
  
  if((actModbus & ACT_RCBO) == ACT_RCBO)
  {
    value |= RCBO_ATT0;
  }
  
  EEPROM_Save_Config (ACTUATORS_EADD, &value, 1);
}


/**
*
* @brief       get from the modbus map the operation mode
*
* @param [in]    
*  
* @retval      none 
*  
****************************************************************/
uint16_t getOperationMode (void)
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();  
  pRwRegs = getRwMdbRegs(mdbAddr);
  return pRwRegs->scuSetRegister.evseMod;  
}

/**
*
* @brief       set in the modbus map max recharge time 
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  setChargeByTime(uint16_t chargeTime) 
{
  scuRwMapRegister_st*    pRwRegs;
  uint8_t              mdbAddr;

  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  /* tempo di carica in minuti */
  pRwRegs->scuSetRegister.chgTime = chargeTime;
}

/**
*
* @brief       set in the modbus map max recharge energy 5..100 
*              KW
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  setChargeByEnergy(uint16_t chargeEnrg) 
{
  scuRwMapRegister_st*    pRwRegs;
  uint8_t              mdbAddr;

  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  /* tempo di carica in minuti */
  pRwRegs->scuSetRegister.chgEnrg = chargeEnrg;
}


/**
*
* @brief       set in the modbus map Power Management mode
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
void  setPmMode(uint8_t mode) 
{
  scuRwMapRegister_st*    pRwRegs;
  uint8_t              mdbAddr;

  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  /* Offset di 1 tra enumerativo modbus e pagina web */
  pRwRegs->scuSetRegister.pmMode = mode;
}


/**
*
* @brief       get from the modbus map Power Management mode
*
* @param [in]  
*  
* @retval      none 
*  
****************************************************************/
uint8_t  getPmMode() 
{
  scuRwMapRegister_st*    pRwRegs;
  uint8_t              mdbAddr;

  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  /* Offset di 1 tra enumerativo modbus e pagina web */
  return (uint8_t)(pRwRegs->scuSetRegister.pmMode);
}


/**
  * @brief  get power management flags   
  *         
  * @param  
  * 
  * @retval power management flags
  */
uint16_t getPmFlags()
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  return pRwRegs->scuSetRegister.pmFlags;
}


/**
  * @brief  get power management pMax   
  *         
  * @param  
  * 
  * @retval power management pMax
  */
uint16_t getPmPmax()
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  return pRwRegs->scuSetRegister.pmPmax;
}


/**
  * @brief  get power management iMin   
  *         
  * @param  
  * 
  * @retval power management iMin
  */
uint16_t getPmImin()
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  return pRwRegs->scuSetRegister.pmImin;
}


/**
  * @brief  get power management hPower   
  *         
  * @param  
  * 
  * @retval power management hPower
  */
uint16_t getPmHpower()
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  return pRwRegs->scuSetRegister.pmHpower;
}


/**
  * @brief  get power management dSet   
  *         
  * @param  
  * 
  * @retval power management dSet
  */
uint16_t getPmDset()
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  return pRwRegs->scuSetRegister.pmDset;
}


/**
  * @brief  get power management dMax   
  *         
  * @param  
  * 
  * @retval power management dMax
  */
uint16_t getPmDmax()
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  return pRwRegs->scuSetRegister.pmDmax;
}


/**
  * @brief  get power management external energy meter   
  *         
  * @param  
  * 
  * @retval power management eMex
  */
uint16_t getPmEmex()
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  uint16_t              checkValue = 0;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  checkValue = pRwRegs->scuSetRegister.hwChecks2;
  
  return (checkValue & 0x0002);
}


/**
  * @brief  set power management external energy meter check in modbus map   
  *         
  * @param  
  * 
  * @retval power management eMex flag
  */
void setPmEmexInModbus(uint8_t emexFlag)
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  if((emexFlag & 0x02) == 0x00)
  {
    pRwRegs->scuSetRegister.hwChecks2 &= 0xFFFD;
  }
  else if((emexFlag & 0x02) == 0x02)
  {
    pRwRegs->scuSetRegister.hwChecks2 |= 0x0002;
  }
}


/**
  * @brief  get charge by time flag  
  *         
  * @param  
  * 
  * @retval charging time value
  */
uint16_t getChargeByTime()
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  return pRwRegs->scuSetRegister.chgTime;
}

/**
  * @brief  get charge by energy flag  
  *         
  * @param  
  * 
  * @retval charging t
  */
uint16_t getChargeByEnergy()
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  return pRwRegs->scuSetRegister.chgEnrg;
}


/**
  * @brief  get default language for the station  
  *         
  * @param  
  * 
  * @retval default language
  */
uint16_t getDefaultLanguage()
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  return (pRwRegs->scuSetRegister.defDispLng);
}


/**
  * @brief  set default language for the station  
  *         
  * @param  default language
  * 
  * @retval 
  */
void setDefaultLanguage(uint8_t lang)
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  pRwRegs->scuSetRegister.defDispLng = lang;
}


/**
  * @brief  set available languages for the station  
  *         
  * @param  available languages
  * 
  * @retval
  */
void setAvailableLanguages(uint32_t availLang)
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  pRwRegs->scuSetRegister.availDispLng = availLang;
}


/**
  * @brief  get the group of languages choosen  
  *         
  * @param  
  * 
  * @retval group of language
  */
uint32_t getAvailableLanguages()
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  return pRwRegs->scuSetRegister.availDispLng;
}


/**
*
* @brief       get from the modbus map the operation mode
*
* @param [in]    
*  
* @retval      none 
*  
****************************************************************/
uint16_t getPmMenuVisibility (void)
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t                mdbAddr;
  
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();  
  pRwRegs = getRwMdbRegs(mdbAddr);
  return pRwRegs->scuSetRegister.menuVisible;   
}

/**
*
* @brief       get Connector ID from modbus map
*
* @param       none    
*  
* @retval      connector Id 
*  
****************************************************************/
uint16_t getConnectorId (void)
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t                mdbAddr;
  
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();  
  pRwRegs = getRwMdbRegs(mdbAddr);
  return pRwRegs->scuSetRegister.devAlias;     
}
                      
/**
*
* @brief       set power management menu visibility
*
* @param [in]    
*  
* @retval      none 
*  
****************************************************************/
void setPmMenuVisibility (uint8_t visibility)
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();  
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  if((visibility & 0x01) == 0x00)
  {
    pRwRegs->scuSetRegister.menuVisible &= 0x00FE;
  }
  else if((visibility & 0x01) == 0x01)
  {
    pRwRegs->scuSetRegister.menuVisible |= 0x0001;
  }
}


/**
*
* @brief       set charge by time menu visibility
*
* @param [in]    
*  
* @retval      none 
*  
****************************************************************/
void setChargeTimeVisibility (uint8_t visibility)
{
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();  
  pRwRegs = getRwMdbRegs(mdbAddr);
  
  if((visibility & 0x02) == 0x00)
  {
    pRwRegs->scuSetRegister.menuVisible &= 0x00FD;
  }
  else if((visibility & 0x02) == 0x02)
  {
    pRwRegs->scuSetRegister.menuVisible |= 0x0002;
  }
}

/**
*
* @brief       set time zone in modbus register
*
* @param [in]  time zone
*  
* @retval      none 
*  
****************************************************************/
void setTimeZone (uint8_t timeZone)
{
  
  scuRwMapRegister_st*  pRwRegs;
  uint8_t               mdbAddr;
  
   /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRwRegs = getRwMdbRegs(mdbAddr);  
  
  pRwRegs->scuSetRegister.rtcTimeZone = (int8_t)timeZone * 60;

}

/**
*
* @brief        Get the semaphore handle for uart re-init 
*
* @param [in]   none
*
* @retval       osSemaphoreId: handle
*
***********************************************************************************************************************/
osSemaphoreId getScuUartInitSemaphoreHandle(void)
{
   return(uartScuInitSemaphoreHandle); 
}

/**
*
* @brief        Get the semaphore handle for uart1 RS485 SCU's link and SINAPSI 
*
* @param [in]   none
*
* @retval       osSemaphoreId: handle
*
***********************************************************************************************************************/
osSemaphoreId getScuSinapsiTxUartSemaphoreHandle(void)
{
   return(uartScuSinapsiTxSemaphoreHandle); 
}


/**
*
* @brief        Get the CPUId checksum on one word 
*
* @param [in]   none
*
* @retval       uint16_t: checksum
*
***********************************************************************************************************************/
uint16_t cpuIdCheksum16(void)
{
  uint16_t cpuCckSum;

  cpuCckSum = 0;
  cpuCckSum += (uint16_t)(STMMicroID.microId1 >> 0);
  cpuCckSum += (uint16_t)(STMMicroID.microId1 >> 8);
  cpuCckSum += (uint16_t)(STMMicroID.microId1 >> 16);
  cpuCckSum += (uint16_t)(STMMicroID.microId1 >> 24);
  cpuCckSum += (uint16_t)(STMMicroID.microId2 >> 0);
  cpuCckSum += (uint16_t)(STMMicroID.microId2 >> 8);
  cpuCckSum += (uint16_t)(STMMicroID.microId2 >> 16);
  cpuCckSum += (uint16_t)(STMMicroID.microId3 >> 24);
  cpuCckSum += (uint16_t)(STMMicroID.microId3 >> 0);
  cpuCckSum += (uint16_t)(STMMicroID.microId3 >> 8);
  cpuCckSum += (uint16_t)(STMMicroID.microId3 >> 16);
  return(cpuCckSum);
}

/**
*
* @brief        Re initi UART1 used for RS485 SCU bus  
*
* @param [in]   none
*
* @retval       none
*
***********************************************************************************************************************/
void  uartReintialization(void)
{
  xQueueReset(uartScuInitSemaphoreHandle);
  /*----- Initialization UART1 RS485 FOR SCUs link -------------------------------------------*/
  if (uart1ReinitCounter <= 1)
  {
    tPrintf("MDB UART1 Re-Init at %s\n\r", getHmsStr());
  }
  /* an irrecuperable error occurred. Reinit uart is necessary */
  reInitScuUart();
  // Rilascio il semaforo
  osSemaphoreRelease(uartScuInitSemaphoreHandle);

  scuDlInfoMng.scuStates = SCU_DL_READY;
  /* end initialization */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void uartScuError_Handler(char * file, int line)
{
  while(1)
  {
    ;
  }
}

/**
  * @brief  get scu index linked at the address    
  *         
  * @param  uint32_t: address modbus
  * @param  uint16_t: index SCU
  * @param  regStartInfo_t*: struct where put the Scu index and normalized address 
  * 
  * @retval none 
  */
static void   findScuObject(uint16_t addrMdb, uint16_t iScu, regScuAddr_t* pScuObj, scuEvents_e originEvent)
{
  
  if (originEvent == SCU_EVENT_MSG_FROM_WIFI)
  {
    pScuObj->index = iScu; 
    pScuObj->rAddr = addrMdb;
    return;
  }
  
  if ((scuOpMode == SCU_M_P) && (originEvent != SCU_EVENT_MSG_FROM_SBC))
  {
    /* SEM SCU master */
#ifdef ADDR_NO_TRANSLATION
    pScuObj->index = (addrMdb / SIZE_MODBUS_MAP) + 1; /* return physical address 1..32 */
#else
    /* in this case the slave address is the first byte of modbus message on RS485 */
    pScuObj->index = iScu; 
#endif
  }
  else
  {
    if (iScu > MODBUS_BROADCAST_ADDR) 
    {
      pScuObj->index = iScu; 
    }
    else 
    {
      /* broadcast address */
      pScuObj->index = getPhysicalMdbAddr();
    }
  }
  pScuObj->rAddr = (addrMdb % SIZE_MODBUS_MAP);
}


/**
  * @brief  get random delay in the range 2..50msec    
  *         
  * @param  uint32_t: address modbus
  * @param  uint16_t: index SCU
  * @param  regStartInfo_t*: struct where put the Scu index and normalized address 
  * 
  * @retval none 
  */
uint8_t getRandomDelay(void)
{
  uint8_t lsb;

  lsb = (uint8_t)scuInfoMng.crcCpuId;
  return((uint8_t)((lsb / 10) + 1) * 2);
}

/**
  * @brief  get testing running flag     
  *         
  * @param  none 
  * 
  * @retval uint8_t: flag status 
  */
uint8_t getCollaudoRunning(void)
{
  return(collaudoRunning);
}

/**
  * @brief  get testing running flag     
  *         
  * @param  none 
  * 
  * @retval uint8_t: flag status 
  */
uint8_t getCollaudoParamWritten(void)
{
  return(configParamDone);
}


/**
  * @brief  set testing running flag     
  *         
  * @param  none 
  * 
  * @retval none 
  */
void setCollaudoRunning(void)
{
  collaudoRunning = TRUE;
  configParamDone = FALSE;
}

/**
  * @brief  get random identifier get from micro id    
  *         
  * @param  none
  *  
  *  
  * @retval uint16_t: random number 
  */
uint16_t getRandomId16(void)
{
  return(scuInfoMng.crcCpuId);
}

/**
*
* @brief        callback to manager timers   
*
* @param [in]   TimerHandle_t: the elapsed timer 
*
* @retval       none
*
***********************************************************************************************************************/
static void mdbUartTimCallBack (TimerHandle_t pxTimer)
{
  uint32_t              timer_id;

  /* find the led  which the timer is referred */
  timer_id = (uint32_t)pvTimerGetTimerID(pxTimer);

  switch (timer_id)
  {
    case (uint32_t)TIMER_MDB_UART_ID:
      
        uartReintialization();
        uart1ReinitCounter++;
        // while ((xTimerChangePeriod (xMdbUartTimers, TIMER_UART_WD, TIMER_UART_GARD_TIME) != pdPASS)); 
        xTimerReset (xMdbUartTimers, 0);
      
      break;

    default:
      break;
  }
}

/**
*
* @brief        restart the timer to reset RS485 UART1    
*
* @param [in]   none 
*
* @retval       none
*
***********************************************************************************************************************/

static void restartTimerToResetRs485Uart()
{
  while ((xTimerChangePeriod (xMdbUartTimers, TIMER_UART_TO_RESET, TIMER_UART_GARD_TIME) != pdPASS)); 
}

/**
*
* @brief        Upgrade SCU modbus address   
*
* @param [in]   none 
*
* @retval       none
*
***********************************************************************************************************************/
void updateScuModbusAddrr (void)
{
  /* Read RS485 address */
  eeprom_param_get(RS485_ADD_EADD, &scuAddr, 1);
}

/**
*
* @brief        Upgrade remote PM Sem flag   
*
* @param [in]   none 
*
* @retval       none
*
***********************************************************************************************************************/
uint8_t setPmRemoteSemFlag (uint8_t writeImm)
{
  uint8_t   semFlag;

  eeprom_param_get(HIDDEN_MENU_ENB_EADD, (uint8_t *)&semFlag, 1);
  semFlag |= HIDDEN_MENU_SEM_ENB; 
  /* save PM enable flag (remote) *****/
  // xx eeprom_array_set(HIDDEN_MENU_ENB_EADD, (uint8_t*)&semFlag, 1);
  eeprom_param_set(HIDDEN_MENU_ENB_EADD, (uint8_t*)&semFlag, 1);
  if (writeImm != 0)
  {
    /* save new parameter in EEPROM */
    return(WriteOnEeprom(HIDDEN_MENU_ENB_EADD, (uint8_t*)&semFlag, 1));
  }
  else
    return(0);
}


/**
*
* @brief        Reset remote PM Sem flag   
*
* @param [in]   none 
*
* @retval       none
*
***********************************************************************************************************************/
uint8_t ResetPmRemoteSemFlag (uint8_t writeImm)
{
  uint8_t   semFlag;

  /* Get PM configuration */
  eeprom_param_get(HIDDEN_MENU_ENB_EADD, (uint8_t *)&semFlag, 1);
  /* Disable PM remote flag (remote) */
  semFlag &= ~HIDDEN_MENU_SEM_ENB; 
  /* Stoe configuration */
  // xx eeprom_array_set(HIDDEN_MENU_ENB_EADD, (uint8_t*)&semFlag, 1);
  eeprom_param_set(HIDDEN_MENU_ENB_EADD, (uint8_t*)&semFlag, 1);
  if (writeImm != 0)
  {
    /* save new parameter in EEPROM */
    return(WriteOnEeprom(HIDDEN_MENU_ENB_EADD, (uint8_t*)&semFlag, 1));
  }
  else
    return(0);
}

/**
*
* @brief       set in charge time (effective charging time - no 
*              sospension time)
*
* @param [in]  uint32_t: time in sec 
*  
* @retval      none 
*  
****************************************************************/
void  setInChargeTime (uint32_t timeCharge) 
{
  scuRoMapRegister_st*  pRoRegs;
  uint8_t               mdbAddr;
  
  /* get address on modbus and relative modbus pointer area   */
  mdbAddr = getLogicalMdbAddrSem();
  pRoRegs = getRoMdbRegs(mdbAddr);
  
  pRoRegs->scuMapRegStatusMeas.timeInCharge = timeCharge; 
}

/**
*
* @brief        start gard timer for SCU <---> SBC link    
*
* @param [in]   none 
*
* @retval       none
*
***********************************************************************************************************************/
static void startSbcGardTimeConn (scuOpModes_e locScuOpMode)
{
  if ((locScuOpMode == SCU_EMUMAX0_M) || (locScuOpMode == SCU_M_P) || (locScuOpMode == SCU_M_STAND_ALONE))
  {
    /* in this case we don't use the auto-reload features */
    xSbcScuTimers = xTimerCreate("SbcScuTim", TIMER_SBC_GARD_TIME, pdTRUE, (void*)(TIMER_SBC_GARD_ID), SbcScuTimCallBack);
    configASSERT(xSbcScuTimers != NULL);
    xTimerReset (xSbcScuTimers, TIMER_GARD_BLOCK_TIME);                  
  }
  xSlaveScuTimers = xTimerCreate("SlaveScuTim", TIMER_SLAVE_GARD_TIME, pdTRUE, (void*)(TIMER_SLAVE_GARD_ID), SbcScuTimCallBack);
  configASSERT(xSlaveScuTimers != NULL);
  xTimerReset (xSlaveScuTimers, TIMER_GARD_BLOCK_TIME);                  
}

/**
*
* @brief        callback to manager the gard timer for SCU <---> SBC link    
*
* @param [in]   TimerHandle_t: the elapsed timer 
*
* @retval       none
*
***********************************************************************************************************************/
static void SbcScuTimCallBack (TimerHandle_t pxTimer)
{
  uint32_t              timer_id;

  /* find the led  which the timer is referred */
  timer_id = (uint32_t)pvTimerGetTimerID(pxTimer);

  switch (timer_id)
  {
    case (uint32_t)TIMER_SBC_GARD_ID:
      /** SBC and router OFF**/
      if (getDowngradeStatus() != (uint8_t)0xAB)
      {
        sbcPowerControl(DISABLED);
      }
      osDelay(1000);
#ifndef HW_MP28947      
      /** SBC and router ON **/
      sbcPowerControl(ENABLED);
#endif 
      
      tPrintf("SBC UART5 Re-Init\n\r");
      reInitSbcUart ();  /* Reinit UART5 */     
      //activeImmediateReset();
      break;

    case (uint32_t)TIMER_SLAVE_GARD_ID:
      /* the fail condition is:                               */
      /* 1) we come from a power  on                          */
      /* 2) an energy meter error come out but it is a fake   */
      if (keyForEmError != (uint16_t)0xA7B4)
      {
        keyForEmError = (uint16_t)0xA7B4;
        retryForEmError = 1;
        if (getStationEmType(INTERNAL_EM) == NONE)
        {
          osDelay(100);
          setFlagForNvic();
          NVIC_SystemReset();
        }
      }
      else
      {
        if ((getStationEmType(INTERNAL_EM) == NONE) && (retryForEmError < 2))
        {
          retryForEmError++;
          osDelay(100);
          setFlagForNvic();
          NVIC_SystemReset();
        }
        else
        {
          if (retryForEmError >= 2)
          {
            if (getStationEmType(INTERNAL_EM) != NONE)
            {
              keyForEmError = (uint16_t)0xFFFF;
            }
          }
        }
      }
      break;

    default:
      break;
  }
}

/**
*
* @brief        callback to manager the gard timer for SCU <---> SBC link    
*
* @param [in]   TimerHandle_t: the elapsed timer 
*
* @retval       none
*
***********************************************************************************************************************/
void resetSbcGardTime (void)
{
  if (xSbcScuTimers != NULL)
  {
    xTimerReset (xSbcScuTimers, 0);
  }
}

/**
*
* @brief        check for download in SCU remote for a slave    
*
* @param [in]   uint32_t: message address  
*
* @retval       uint8_t: TRUE if SCU remote and incoming download message received
*
***********************************************************************************************************************/
static uint8_t checkSlaveDwldRemoteScu (uint16_t rAddr)
{
  if ((getScuOpMode() == SCU_M_STAND_ALONE) &&
      ((rAddr == ADDR_FILE_COMMAND_RW) || (rAddr == ADDR_FILE_SIZE_RW)))
  {
    return(TRUE);
  }
  else
  {
    return(FALSE);
  }
}

/**
*
* @brief       Send a  message a scuDataLinkTask      
*
* @param [in]  scuDlEvents_e: event to be managed    
*  
* @retval      none   
*  
****************************************************************/
void sendScuDataLinkMsg (scuDlEvents_e frameScuEvent) 
{
  headerScuRx_st      headerScuRx;
  
  headerScuRx.dlEvent = frameScuEvent; // for example SCU_DL_CHANGE_TO_SEM_FROM_TEST;
  configASSERT(xQueueSendToBack(getScuDataLinkQueueHandle(), (void *)&headerScuRx, portMAX_DELAY) == pdPASS); // scuDataLinkTask() --> scuDlProcess()
}


/**
  * @brief  set the EM fault trap to null value    
  *         
  * @param  none
  *  
  *  
  * @retval none
  */
void setEmFaultTrap (void)
{
  keyForEmError = (uint16_t)0xFFFF;
}


#ifdef MODBUS_TCP_EM_LOVATO
/**
*
* @brief       Get the current Lovato Em area 
*
* @param [in]  none  
*  
* @retval      scuMapRegLovatoEmSim_st*: pointer to Lovato area
*  
****************************************************************/
scuMapRegLovatoEmSim_st* getLovatoEthRegs (void)
{
  return (&scuMapRegLovatoEmSim);
}
#endif

/*************** END OF FILE ******************************************************************************************/



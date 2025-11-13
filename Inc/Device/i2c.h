/**
* @file        i2c.c
*
* @brief       API for I2C - Definition -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: i2c.h 620 2024-10-28 12:00:36Z stefano $
*
*     $Revision: 620 $
*
*     $Author: stefano $
*
*     $Date: 2024-10-28 13:00:36 +0100 (lun, 28 ott 2024) $
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H
#define __I2C_H

#define HI2Cx                   hi2c3
// Definizioni per la io expander
#define DEVICE_IO_EXP		        0xD0
#define DEVICE_IO_EXP_BASE_ADD	0x00

// Definizioni per EEPROM ST 24C64 (adatta anche per 24LC1025 di microchip)
#define DEVICE_EEPROM     0xA0
#define EEPROM_PAGE_SIZE  32
// Definizioni solo per EEPROM ST 24C64 Identification page
#define DEVICE_EEPROM_ID 0xB8

/* IO expander FXL6408 definition */
#define DEVICE_IOEXP_1      ((uint16_t)0x88)   /* U39 */
#define DEVICE_IOEXP_0      ((uint16_t)0x86)   /* U40 */

/** define for test audio amplifier  */
#define DEVICE_AUDIO        ((uint8_t)0x92)
#define DEVICE_DATA_SIZE    ((uint16_t)1)


/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/
extern unsigned char  checkI2c(void);

extern unsigned char  ReadFromEeprom        (unsigned short Address, unsigned char *Buffer, unsigned short Length);
extern unsigned char  WriteOnEeprom         (unsigned short Address, unsigned char *Buffer, unsigned short Length);
extern unsigned char  ReadFromEeprom_no_Semaph (unsigned short Address, unsigned char *Buffer, unsigned short Length);

extern unsigned char  ReadFromIoExp         (uint16_t slaveAddr, unsigned char Address, unsigned char *Buffer, unsigned short Length);
extern unsigned char  WriteToIoExp          (uint16_t slaveAddr, unsigned short Address, unsigned char data);
extern unsigned char  WriteOnAudio          (unsigned char volume);

extern void           reinitI2CforEprom     (void);
extern void           i2c3SemaphoreCreate   (void);

#endif //__I2C_H


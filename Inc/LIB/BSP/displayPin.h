/**
* @file        rtcApi.h
*
* @brief       API for RTC   - Definition -
*
* @author      Nick
*
* @riskClass   C
*
* @moduleID
*
* @vcsInfo
*     $Id: displayPin.h 599 2024-09-26 07:03:24Z stefano $
*
*     $Revision: 599 $
*
*     $Author: stefano $
*
*     $Date: 2024-09-26 09:03:24 +0200 (gio, 26 set 2024) $
*
*
* @copyright
*       Copyright (C) 2016 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

#ifndef __DISPLAY_PIN_H
#define __DISPLAY_PIN_H

/************************************************************
 * Include
 ************************************************************/
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#ifdef GD32F4xx
#include "stm32f4xx_hal.h"
#else
#include "stm32h5xx_hal.h"
#endif
#include "cmsis_os.h"
#include "ledMng.h"
#include "lcd.h"




/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#define LCDn                             7 

/**
  * @brief DB0  --> 
  */
#define DB0_PIN                    GPIO_PIN_3
#define DB0_GPIO_PORT              GPIOE
#define DB0_GPIO_CLK_ENABLE()     __GPIOE_CLK_ENABLE()  
#define DB0_GPIO_CLK_DISABLE()    __GPIOE_CLK_DISABLE()

/**
  * @brief DB1  --> 
  */
#define DB1_PIN                    GPIO_PIN_4
#define DB1_GPIO_PORT              GPIOE
#define DB1_GPIO_CLK_ENABLE()     __GPIOE_CLK_ENABLE()  
#define DB1_GPIO_CLK_DISABLE()    __GPIOE_CLK_DISABLE()

/**
  * @brief DB2  --> 
  */
#define DB2_PIN                    GPIO_PIN_5
#define DB2_GPIO_PORT              GPIOE
#define DB2_GPIO_CLK_ENABLE()     __GPIOE_CLK_ENABLE()  
#define DB2_GPIO_CLK_DISABLE()    __GPIOE_CLK_DISABLE()

/**
  * @brief DB3  --> 
  */
#define DB3_PIN                   GPIO_PIN_6
#define DB3_GPIO_PORT              GPIOE
#define DB3_GPIO_CLK_ENABLE()     __GPIOE_CLK_ENABLE()  
#define DB3_GPIO_CLK_DISABLE()    __GPIOE_CLK_DISABLE()

/**
  * @brief RW  --> 
  */

#define RW_PIN                     GPIO_PIN_10
#define RW_GPIO_PORT               GPIOE
#define RW_GPIO_CLK_ENABLE()      __GPIOE_CLK_ENABLE()  
#define RW_GPIO_CLK_DISABLE()     __GPIOE_CLK_DISABLE()

/**
  * @brief RS  --> 
  */
#define RS_PIN                     GPIO_PIN_0
#define RS_GPIO_PORT               GPIOE
#define RS_GPIO_CLK_ENABLE()      __GPIOE_CLK_ENABLE()  
#define RS_GPIO_CLK_DISABLE()     __GPIOE_CLK_DISABLE()

/**
  * @brief CS  --> 
  */
#define CS_PIN                     GPIO_PIN_1
#define CS_GPIO_PORT               GPIOE
#define CS_GPIO_CLK_ENABLE()      __GPIOE_CLK_ENABLE()  
#define CS_GPIO_CLK_DISABLE()     __GPIOE_CLK_DISABLE()

/**
  * @brief PWM  --> 
  */
#define PWM_LCD_PIN                     GPIO_PIN_14
#define PWM_LCD_GPIO_PORT               GPIOE
#define PWM_LCD_GPIO_CLK_ENABLE()      __GPIOE_CLK_ENABLE()  
#define PWM_LCD_GPIO_CLK_DISABLE()     __GPIOE_CLK_DISABLE()



#define LCDx_GPIO_CLK_ENABLE(__INDEX__)  do{if((__INDEX__) == 0) DB0_GPIO_CLK_ENABLE(); else \
                                               if((__INDEX__) == 1) DB1_GPIO_CLK_ENABLE(); else \
                                               if ((__INDEX__) == 2) DB2_GPIO_CLK_ENABLE(); else \
                                               if ((__INDEX__) == 3) DB3_GPIO_CLK_ENABLE(); else \
                                               if ((__INDEX__) == 4) RW_GPIO_CLK_ENABLE(); else \
                                               if ((__INDEX__) == 5) RS_GPIO_CLK_ENABLE(); else \
                                               if ((__INDEX__) == 6) CS_GPIO_CLK_ENABLE(); \
                                               }while(0)

typedef enum 
{
  LCD_DB0 = 0,
  LCD_DB1 = 1,
  LCD_DB2 = 2,
  LCD_DB3 = 3,
  LCD_RW  = 4,
  LCD_RS  = 5,
  LCD_CS  = 6
}Lcd_TypeDef;

#ifndef ON
#define ON		1
#endif
#ifndef OFF
#define OFF		0
#endif
#ifndef FALSE
typedef enum {FALSE = 0, TRUE = 1} Boolean;
#endif


/* *******  Commands for display *************** */
#define CLEAR_DISPLAY      0x01
#define RETURN_HOME        0x02

// *  Comandi cursore  *
#define CURSOR_OFF         0x0C
#define CURSOR_ON_NO_BLINK 0x0E
#define CURSOR_ON_BLINK    0x0F

/*********************
 *  Tipi di cursore  *
 *********************/

#define NO_CURSOR          0
#define CURSOR_UNDERSCORE  1
#define CURSOR_BLINK       2

/*********************************************
 * Definizione terminatore stringa (default) *
 *********************************************/
#define TERMINATORE        '\0'
#define NUM_CAR_DISP        20

// *  Comandi di posizionamento righe  *
#define POS_RIGA           0x80
#define POS_RIGA_1         0x80
#define POS_RIGA_2         0xC0
#define POS_RIGA_3         0x94
#define POS_RIGA_4         0xD4

#define	REGIR		0
#define	REGDR		1

#define LcdCmdNoWait(a)		LcdWrite(a,REGIR,FALSE)
#define	LcdCmdNibbleNW(a)	LcdWrite(a,REGIR,TRUE)

#define REG8(a,b)  ((volatile unsigned char *)((a)+(b)))
#define REG16(a,b) ((volatile unsigned short *)((a)+(b)))
#define REG32(a,b) ((volatile unsigned int *)((a)+(b)))

#define HAL_READ_UINT32(a,b)	((b)=*(a->ODR))
#define HAL_WRITE_UINT32(a,b)	(*(a->ODR) = (b))

/* *	Definizione macro Gestione Contrasto LCD     			*/
#define MIN_CONTRAST_LEV    150
#define MAX_CONTRAST_LEV    999


/* MACRO for function alias  */
#define LcdEnable(a)		Lcd_Enable(a)

#define LcdRs(a)			        Lcd_Rs(a)
#define LcdRw(a)			        Lcd_Rw(a)

#define	WriteLcdData(a)		    Write_LcdData(a) 
#define ReadLcdData(a)		    a=Read_LcdData()  
#define	LcdSetWriteDir()	    LCD_DB_Output()
#define LcdSetReadDir()		    LCD_DB_Input() 

#define MAX_BUSYFLAG_WAIT	    2000
#define START_CONTRAST        500
#define CONTRAST_STEP_VAL     1200

/***************** definizione timer PWM LCD4X20    **************************/
#define TIMLCD4X20                       TIM1
#define TIMLCD4X20_CLK_ENABLE()          __HAL_RCC_TIM1_CLK_ENABLE();
#define PWMLCD4X20_AF                    GPIO_AF1_TIM1
#define PWMLCD4X20_TIM_CHx               TIM_CHANNEL_4

#define PWMLCD4X20_PERIOD_VALUE               ((uint32_t)100)
#define PWMLCD4X20_MIN_DC                     ((uint32_t)60)   

/**
  * @brief PWMLCD4X20 --> PE14 TIM1 CH4
  */
#define PWMLCD4X20_PIN                   GPIO_PIN_14
#define PWMLCD4X20_GPIO_PORT             GPIOE
#define PWMLCD4X20_GPIO_CLK_ENABLE()     __GPIOE_CLK_ENABLE()  
#define PWMLCD4X20_GPIO_CLK_DISABLE()    __GPIOE_CLK_DISABLE()


/***************** definizione timer PWM LED A-B-C    **************************/
/**
  * @brief PWMLED_A --> PE9 TIM1 CH1
  */
#define TIMLED_A                        TIM1
#define TIMLED_A_CLK_ENABLE()           __HAL_RCC_TIM1_CLK_ENABLE();
#define PWMLED_A_AF                     GPIO_AF1_TIM1
#define PWMLED_A_TIM_CHx                TIM_CHANNEL_1

#define PWMLED_A_PERIOD_VALUE           ((uint32_t)100)
#define PWMLED_A_MIN_DC                 ((uint32_t)40)   

#define PWMLED_A_PIN                    GPIO_PIN_9
#define PWMLED_A_GPIO_PORT              GPIOE
#define PWMLED_A_GPIO_CLK_ENABLE()      __GPIOE_CLK_ENABLE()  
#define PWMLED_A_GPIO_CLK_DISABLE()     __GPIOE_CLK_DISABLE()

/**
  * @brief PWMLED_B --> PE11 TIM1 CH2
  */
#define TIMLED_B                        TIM1
#define TIMLED_B_CLK_ENABLE()           __HAL_RCC_TIM1_CLK_ENABLE();
#define PWMLED_B_AF                     GPIO_AF1_TIM1
#define PWMLED_B_TIM_CHx                TIM_CHANNEL_2

#define PWMLED_B_PERIOD_VALUE           ((uint32_t)100)
#define PWMLED_B_MIN_DC                 ((uint32_t)40)   

#define PWMLED_B_PIN                    GPIO_PIN_11
#define PWMLED_B_GPIO_PORT              GPIOE
#define PWMLED_B_GPIO_CLK_ENABLE()      __GPIOE_CLK_ENABLE()  
#define PWMLED_B_GPIO_CLK_DISABLE()     __GPIOE_CLK_DISABLE()

/**
  * @brief PWMLED_C --> PE13 TIM1 CH3
  */
#define TIMLED_C                        TIM1
#define TIMLED_C_CLK_ENABLE()           __HAL_RCC_TIM1_CLK_ENABLE();
#define PWMLED_C_AF                     GPIO_AF1_TIM1
#define PWMLED_C_TIM_CHx                TIM_CHANNEL_3

#define PWMLED_C_PERIOD_VALUE           ((uint32_t)100)
#define PWMLED_C_MIN_DC                 ((uint32_t)40)   

#define PWMLED_C_PIN                    GPIO_PIN_13
#define PWMLED_C_GPIO_PORT              GPIOE
#define PWMLED_C_GPIO_CLK_ENABLE()      __GPIOE_CLK_ENABLE()  
#define PWMLED_C_GPIO_CLK_DISABLE()     __GPIOE_CLK_DISABLE()

/* Definition for DACx clock resources to contrast regulation */
#define DACx                            DAC
#define DACx_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define DMAx_CLK_ENABLE()               __HAL_RCC_DMA1_CLK_ENABLE()

#define DACx_CLK_ENABLE()               __HAL_RCC_DAC_CLK_ENABLE()
#define DACx_FORCE_RESET()              __HAL_RCC_DAC_FORCE_RESET()
#define DACx_RELEASE_RESET()            __HAL_RCC_DAC_RELEASE_RESET()

/* Definition for DACx Channel Pin */
#define DACx_CHANNEL_PIN                GPIO_PIN_4 
#define DACx_CHANNEL_GPIO_PORT          GPIOA

#define DACx_CHANNEL                    DAC_CHANNEL_1

/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/

/****************** FUNCTIONS IMPLEMENTATION **************************************************************************/
void              uSecWwaitTime                 (uint16_t usDelay);
void              Lcd2x20Init                   (uint8_t fullInit, LcdInitState State);
HAL_StatusTypeDef HAL_TIM_PWM_ChangeDutyChannel (TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef* sConfig, uint32_t Channel);
void              setBacklightPWMlcd4x20        (uint8_t percentValue);
void              putsxy_c                      (uint8_t x, uint8_t y, char const *s);
void              LcdUpDownContrast             (uint32_t ContrastLevel);
void              test_pic                      (uint8_t picIx);
void              putsxyDwnl_c                  (uint8_t x, uint8_t y, uint8_t n);

#endif  // __DISPLAY_PIN_H

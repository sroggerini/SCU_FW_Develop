/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : stm32f7xx_periph_init.h
  * @brief          : stm32f7xx_periph_init.c file.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F7xx_PERIPH_INIT_H
#define __STM32F7xx_PERIPH_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
  
//-----------------------------------------------------------------

/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* define pin for Port A  */
#define I2C3_SCL_Pin            GPIO_PIN_8
#define I2C3_SCL_GPIO_Port      GPIOA
#define CAN1_RX_Pin             GPIO_PIN_11
#define CAN1_RX_GPIO_Port       GPIOA
#define CP_ADC_Pin              GPIO_PIN_0
#define CP_ADC_GPIO_Port        GPIOA
#define ETH_REF_CLK_Pin         GPIO_PIN_1
#define ETH_REF_CLK_GPIO_Port   GPIOA
#define ETH_MDIO_Pin            GPIO_PIN_2
#define ETH_MDIO_GPIO_Port      GPIOA
#define IM_ADC_Pin              GPIO_PIN_3
#define IM_ADC_GPIO_Port        GPIOA
#define LCD_V0_Pin              GPIO_PIN_4
#define LCD_V0_GPIO_Port        GPIOA
#ifndef USE_VOICE
#define SINAPSI_Pin             GPIO_PIN_5
#define SINAPSI_GPIO_Port       GPIOA
#else
#define VOCE_Pin                GPIO_PIN_5
#define VOCE_GPIO_Port          GPIOA
#endif

#define RCDM_Pin                GPIO_PIN_6
#define RCDM_GPIO_Port          GPIOA
#define ETH_CRS_DV_Pin          GPIO_PIN_7
#define ETH_CRS_DV_GPIO_Port    GPIOA
#define I2C3_SCL_Pin            GPIO_PIN_8
#define I2C3_SCL_Port           GPIOA
   
/* define pin for Port B  */
#define PP_ADC_Pin              GPIO_PIN_0
#define PP_ADC_GPIO_Port        GPIOB
#define GEN1_ADC_Pin            GPIO_PIN_1
#define GEN1_ADC_GPIO_Port      GPIOB
#define IN2_Pin                 GPIO_PIN_10
#define IN2_GPIO_Port           GPIOB
#define ETH_TXD1_Pin            GPIO_PIN_13
#define ETH_TXD1_GPIO_Port      GPIOB
#define SBC_PWR_Pin             GPIO_PIN_14
#define SBC_PWR_GPIO_Port       GPIOB
#define SPI1_CLK_Pin            GPIO_PIN_3
#define SPI1_CLK_GPIO_Port      GPIOB
#ifdef NUCLEO_F746
#define H_LED_Pin               GPIO_PIN_7
#define H_LED_GPIO_Port         GPIOB
#else
#define H_LED_Pin               GPIO_PIN_1
#define H_LED_GPIO_Port         GPIOH
#endif
#define SMB0_SDA_Pin            GPIO_PIN_7
#define SMB0_SDA_GPIO_Port      GPIOB
#define SMB0_SCL_Pin            GPIO_PIN_8
#define SMB0_SCL_GPIO_Port      GPIOB
#define SBC_CONN_Pin            GPIO_PIN_9
#define SBC_CONN_GPIO_Port      GPIOB
#define RFID_DTC_Pin            GPIO_PIN_15
#define RFID_DTC_GPIO_Port      GPIOB

/*********** BLUETOOTH AREA *****************************************/
#define BT_RST_Pin              GPIO_PIN_15
#define BT_RST_GPIO_Port        GPIOD

/* define pin for Port C  */
#define TA1_ADC_Pin              GPIO_PIN_0
#define TA1_ADC_GPIO_Port        GPIOC
#define ETH_MDC_Pin              GPIO_PIN_1
#define ETH_MDC_GPIO_Port        GPIOC
#define RSW_IN_ADC_Pin           GPIO_PIN_2
#define RSW_IN_ADC_GPIO_Port     GPIOC
#define TEMP_ADC_Pin             GPIO_PIN_3
#define TEMP_ADC_GPIO_Port       GPIOC
#define ETH_RXD0_Pin             GPIO_PIN_4
#define ETH_RXD0_GPIO_Port       GPIOC
#define ETH_RXD1_Pin             GPIO_PIN_5
#define ETH_RXD1_GPIO_Port       GPIOC
#define PWM_CP_Pin               GPIO_PIN_6
#define PWM_CP_GPIO_Port         GPIOC
#define RFID_PWR_Pin             GPIO_PIN_7
#define RFID_PWR_GPIO_Port       GPIOC
#define VAC230_DET_Pin           GPIO_PIN_8
#define VAC230_DET_GPIO_Port     GPIOC
#define I2C3_SDA_Pin             GPIO_PIN_9
#define I2C3_SDA_Port            GPIOC
#define IN6_Pin                  GPIO_PIN_10
#define IN6_GPIO_Port            GPIOC
#define PWRDWN1L_GPIO_Port       GPIOC
#define PWRDWN1L_Pin             GPIO_PIN_11
#define BT_IRQ_GPIO_Port         GPIOC
#define BT_IRQ_Pin               GPIO_PIN_13
#define LCD_PWR_GPIO_Port        GPIOC
#define LCD_PWR_Pin              GPIO_PIN_13

/* define pin for Port D  */
#define IN5_Pin                  GPIO_PIN_0
#define IN5_GPIO_Port            GPIOD
#define CAN1_TX_Pin              GPIO_PIN_1
#define CAN1_TX_GPIO_Port        GPIOD
#define OUTBL1_M_Pin             GPIO_PIN_3
#define OUTBL1_M_GPIO_Port       GPIOD
#define SGCBOB_Pin               GPIO_PIN_7
#define SGCBOB_GPIO_Port         GPIOD
#define OUTBL1_P_Pin             GPIO_PIN_8
#define OUTBL1_P_GPIO_Port       GPIOD
#define DIODO_ADC_Pin            GPIO_PIN_9
#define DIODO_ADC_GPIO_Port      GPIOD
#define CNTCT_Pin                GPIO_PIN_10
#define CNTCT_GPIO_Port          GPIOD
#define SW_PROG_Pin              GPIO_PIN_14
#define SW_PROG_GPIO_Port        GPIOD
#define WIFI_EN_Pin              GPIO_PIN_15
#define WIFI_EN_GPIO_Port        GPIOD

/* define pin for Port E  */
#define LCD_D0_Pin               GPIO_PIN_3
#define LCD_D0_GPIO_Port         GPIOE
#define LCD_D1_Pin               GPIO_PIN_4
#define LCD_D1_GPIO_Port         GPIOE
#define LCD_D2_Pin               GPIO_PIN_5
#define LCD_D2_GPIO_Port         GPIOE
#define LCD_D3_Pin               GPIO_PIN_6
#define LCD_D3_GPIO_Port         GPIOE
#define DLED_A_Pin               GPIO_PIN_9
#define DLED_A_GPIO_Port         GPIOE
#define LCD_RW_Pin               GPIO_PIN_10    /* defined in lcd.c */
#define LCD_RW_GPIO_Port         GPIOE
#define DLED_B_Pin               GPIO_PIN_11
#define DLED_B_GPIO_Port         GPIOE
#define INT_INx_Pin              GPIO_PIN_12
#define INT_INx_GPIO_Port        GPIOE
#define DLED_C_Pin               GPIO_PIN_13
#define DLED_C_GPIO_Port         GPIOE
#define LCD_PWM_T1CH4_Pin        GPIO_PIN_14
#define LCD_PWM_T1CH4_GPIO_Port  GPIOE
#define LCD_RS_Pin               GPIO_PIN_0
#define LCD_RS_GPIO_Port         GPIOE
#define LCD_CS_Pin               GPIO_PIN_1
#define LCD_CS_GPIO_Port         GPIOE
#define SPI1_NSS2_Pin            GPIO_PIN_15
#define SPI1_NSS2_GPIO_Port      GPIOE
#define WIFI_NSS2_Pin            GPIO_PIN_15
#define WIFI_NSS2_GPIO_Port      GPIOE
#define SPI1_NSS2_Pin            GPIO_PIN_15
#define SPI1_NSS2_GPIO_Port      GPIOE

                                 
#define BT_CS_Pin                GPIO_PIN_15
#define BT_CS_GPIO_Port          GPIOA

/* Defines related to Clock configuration */
#define RTC_ASYNCH_PREDIV  0x7F   /* LSE as RTC clock */
#define RTC_SYNCH_PREDIV   0x00FF /* LSE as RTC clock */

/*----------------------------------------------------------------------------
 *                               Global variables
 *----------------------------------------------------------------------------
 */
#ifndef stm32f7xx_periph_init_c

extern UART_HandleTypeDef                  huart7;
extern I2C_HandleTypeDef                   hSmb0;

#endif

/*----------------------------------------------------------------------------
 *                               Private section
 *----------------------------------------------------------------------------
 */

#ifdef stm32f7xx_periph_init_c

#endif


/*
*********************************** SCAME ************************************
**                                                                          **
**                            Function prototypes                           **
**                                                                          **
******************************************************************************
*/

#ifdef __cplusplus
}
#endif

#endif /* __STM32F7xx_PERIPH_INIT_H */

/***************************** END OF FILE ************************************/

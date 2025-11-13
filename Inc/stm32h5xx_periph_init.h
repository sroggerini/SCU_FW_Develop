/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for gd32f4xx_periph_init.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __STM32H5xx_PERIPH_INIT_H
#define __STM32H5xx_PERIPH_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
  
#include "stm32h5xx_hal.h"

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
 
  /* Defines related to Clock configuration */
#define RTC_ASYNCH_PREDIV  0x7F   /* LSE as RTC clock */
#define RTC_SYNCH_PREDIV   0x00FF /* LSE as RTC clock */

#define DEFAULT_CALM_VALUE 0x00000001
#define DEFAULT_CALP_VALUE RTC_SMOOTHCALIB_PLUSPULSES_SET
  
/*
*********************************** SCAME ************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/

/* Private defines -----------------------------------------------------------*/

/*    PA port     */
#define CP_ADC_Pin                        GPIO_PIN_0
#define CP_ADC_GPIO_Port                  GPIOA
#define ETH_MDIO_Pin                      GPIO_PIN_2
#define ETH_MDIO_GPIO_Port                GPIOA
#define H_LED_Pin                         GPIO_PIN_2  /* GPIO_PIN_3 in HW v1.6 */
#define H_LED_GPIO_Port                   GPIOA
#define PWM_CP_Pin                        GPIO_PIN_6
#define PWM_CP_GPIO_Port                  GPIOA
#define USART1_TX_Pin			  GPIO_PIN_9
#define USART1_TX_GPIO_Port		  GPIOA
#define USART1_RX_Pin			  GPIO_PIN_10
#define USART1_RX_GPIO_Port		  GPIOA
#define USART1_DE_Pin                     GPIO_PIN_12
#define USART1_DE_GPIO_Port               GPIOA 
#define SPI1_NSS_Pin                      GPIO_PIN_15
#define SPI1_NSS_GPIO_Port                GPIOA
#define I2C3_SCL_Pin                      GPIO_PIN_8
#define I2C3_SCL_GPIO_Port                GPIOA
#define SYS_JTMS_SWDIO_Pin                GPIO_PIN_13
#define SYS_JTMS_SWDIO_GPIO_Port          GPIOA
#define SYS_JTCK_Pin                      GPIO_PIN_14
#define SYS_JTCK_GPIO_Port                GPIOA

/*    PB port     */
#define VIN_ADC_Pin                       GPIO_PIN_1
#define VIN_ADC_GPIO_Port                 GPIOB
#define SYN3PuP_Pin                       GPIO_PIN_2
#define SYN3PuP_GPIO_Port                 GPIOB
#define SYN1PuP_Pin                       GPIO_PIN_6
#define SYN1PuP_GPIO_Port                 GPIOB
#define SMB0_SDA_Pin                      GPIO_PIN_7
#define SMB0_SDA_GPIO_Port                GPIOB
#define SMB0_SCL_Pin                      GPIO_PIN_8
#define SMB0_SCL_GPIO_Port                GPIOB
#define SBC_CONN_Pin                      GPIO_PIN_9
#define SBC_CONN_GPIO_Port                GPIOB
#define I2C1_SCL_Pin                      GPIO_PIN_10
#define I2C1_SCL_GPIO_Port                GPIOB
#define IN1_Pin                           GPIO_PIN_13
#define IN1_GPIO_Port                     GPIOB
#define IN3_Pin                           GPIO_PIN_9
#define IN3_GPIO_Port                     GPIOB
#define i2C1_SDA_Pin                      GPIO_PIN_11
#define i2C1_SDA_GPIO_Port                GPIOB
#define I2C1_SCL_Pin                      GPIO_PIN_10
#define I2C1_SCL_GPIO_Port                GPIOB
#define I2C1_SDA_Pin                      GPIO_PIN_11
#define I2C1_SDA_GPIO_Port                GPIOB
#define SPI2_MISO_Pin                     GPIO_PIN_14
#define SPI2_MISO_GPIO_Port               GPIOB

/*    PC port     */
#define EN_PM1uP_Pin                      GPIO_PIN_0
#define EN_PM1uP_GPIO_Port                GPIOC
#define SPI2_MOSI_Pin                     GPIO_PIN_3
#define SPI2_MOSI_GPIO_Port               GPIOC
#define TEMP_ADC_Pin                      GPIO_PIN_2
#define TEMP_ADC_GPIO_Port                GPIOC
#define ETH_RXD0_Pin                      GPIO_PIN_4
#define ETH_RXD0_GPIO_Port                GPIOC
#define ETH_RXD1_Pin                      GPIO_PIN_5
#define ETH_RXD1_GPIO_Port                GPIOC
#define UART6_TX_Pin                      GPIO_PIN_6
#define UART6_TX_GPIO_Port                GPIOC
#define UART6_RX_Pin                      GPIO_PIN_7
#define UART6_RX_GPIO_Port                GPIOC  
#define VAC230_DET_Pin                    GPIO_PIN_8
#define VAC230_DET_GPIO_Port              GPIOC
#define I2C3_SDA_Pin                      GPIO_PIN_9
#define I2C3_SDA_GPIO_Port                GPIOC
#define USART3_TX_Pin                     GPIO_PIN_10
#define USART3_TX_GPIO_Port               GPIOC  
#define USART3_RX_Pin                     GPIO_PIN_11
#define USART3_RX_GPIO_Port               GPIOC 
#define PRG_TX_Pin                        GPIO_PIN_12
#define PRG_TX_GPIO_Port                  GPIOC    
#define TAMPER_Pin                        GPIO_PIN_13
#define TAMPER_GPIO_Port                  GPIOC
  
/*    PD port     */
#define SW_IN_ADC_Pin                     GPIO_PIN_0
#define SW_IN_ADC_GPIO_Port               GPIOD
#define WIFI_EN_Pin                       GPIO_PIN_1
#define WIFI_EN_GPIO_Port                 GPIOD
#define PRG_RX_Pin                        GPIO_PIN_2
#define PRG_RX_GPIO_Port                  GPIOD
#define UART2_DE_Pin                      GPIO_PIN_4
#define UART2_DE_GPIO_Port                GPIOD
#define SGCBOB_Pin                        GPIO_PIN_7
#define SGCBOB_GPIO_Port                  GPIOD
#define RFID_DTC_Pin                      GPIO_PIN_7
#define RFID_DTC_GPIO_Port                GPIOD  
#define DIODO_ADC_Pin                     GPIO_PIN_9
#define DIODO_ADC_GPIO_Port               GPIOD
#define CNTCT_Pin                         GPIO_PIN_10
#define CNTCT_GPIO_Port                   GPIOD
#define RFID_PWR_Pin                      GPIO_PIN_11
#define RFID_PWR_GPIO_Port                GPIOD
#define BOOT_EN_Pin                       GPIO_PIN_12
#define BOOT_EN_GPIO_Port                 GPIOD
#define WIFI_IO9_Pin                      GPIO_PIN_13
#define WIFI_IO9_GPIO_Port                GPIOD
#define SW_PROG_Pin                       GPIO_PIN_14
#define SW_PROG_GPIO_Port                 GPIOD
#define ETH_RST_Pin                       GPIO_PIN_15
#define ETH_RST_GPIO_Port                 GPIOD
#define GPIO1_uP_Pin                      GPIO_PIN_0   // This pin is sconnected in HW_MP28947, it's defined just to avoid compiler errors */
#define GPIO1_uP_GPIO_Port                GPIOD  

/*    PE port     */  
#define EN_PM3uP_Pin                      GPIO_PIN_2
#define EN_PM3uP_GPIO_Port                GPIOE
#define CNTCT_PWM_Pin                     GPIO_PIN_5
#define CNTCT_PWM_GPIO_Port               GPIOE
#define RCDM_Pin                          GPIO_PIN_7 
#define RCDM_GPIO_Port                    GPIOE
#define SCS3PuP_Pin                       GPIO_PIN_8
#define SCS3PuP_GPIO_Port                 GPIOE
#define DLED_A_Pin                        GPIO_PIN_9
#define DLED_A_GPIO_Port                  GPIOE
#define ETH_SPI_CS_Pin                    GPIO_PIN_10
#define ETH_SPI_CS_GPIO_Port              GPIOE
#define DLED_B_Pin                        GPIO_PIN_11
#define DLED_B_GPIO_Port                  GPIOE
#define INT_INx_Pin                       GPIO_PIN_12
#define INT_INx_GPIO_Port                 GPIOE
#define DLED_C_Pin                        GPIO_PIN_13
#define DLED_C_GPIO_Port                  GPIOE
#define PWM_LUM_Pin                       GPIO_PIN_14
#define PWM_LUM_GPIO_Port                 GPIOE
#define SPI_485_Pin                       GPIO_PIN_14
#define SPI_485_GPIO_Port                 GPIOE 
#define SCS1PuP_Pin                       GPIO_PIN_15
#define SCS1PuP_GPIO_Port                 GPIOE


/* Definition for legacy purpose ( ALL mapped at GPIOE pin3 ) */

#define GPIO0_uP_Pin                      GPIO_PIN_11
#define GPIO0_uP_GPIO_Port                GPIOA
#define SPI1_NSS2_Pin                     GPIO_PIN_15
#define SPI1_NSS2_GPIO_Port               GPIOE
#define OUTBL1_M_Pin                      GPIO_PIN_3
#define OUTBL1_M_GPIO_Port                GPIOE
#define OUTBL1_P_Pin                      GPIO_PIN_3
#define OUTBL1_P_GPIO_Port                GPIOE
#define SBC_PWR_Pin                       GPIO_PIN_3
#define SBC_PWR_GPIO_Port                 GPIOE
#define PWRDWN1L_Pin                      GPIO_PIN_3
#define PWRDWN1L_GPIO_Port                GPIOE


/*----------------------------------------------------------------------------
 *                               Global variables
 *----------------------------------------------------------------------------
 */
#ifndef stm32h5xx_periph_init_c

extern UART_HandleTypeDef                  huart6;
extern I2C_HandleTypeDef                   hi2c1;

#endif

/*----------------------------------------------------------------------------
 *                               Private section
 *----------------------------------------------------------------------------
 */

#ifdef stm32h5xx_periph_init_c

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

#endif /* __STM32H5xx_PERIPH_INIT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

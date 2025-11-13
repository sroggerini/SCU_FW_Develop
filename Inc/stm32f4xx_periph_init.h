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

#ifndef __STM32F4xx_PERIPH_INIT_H
#define __STM32F4xx_PERIPH_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
  
#include "stm32f4xx_hal.h"

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
#define ETH_REF_CLK_Pin                   GPIO_PIN_1
#define ETH_REF_CLK_GPIO_Port             GPIOA
#define ETH_MDIO_Pin                      GPIO_PIN_2
#define ETH_MDIO_GPIO_Port                GPIOA
#define IM_ADC_Pin                        GPIO_PIN_3
#define IM_ADC_GPIO_Port                  GPIOA
#define TA23TYPE_Pin                      GPIO_PIN_4
#define TA23TYPE_GPIO_Port                GPIOA
#define LCD_V0_Pin                        GPIO_PIN_4
#define LCD_V0_GPIO_Port                  GPIOA
#define LCD_PWR_Pin                       GPIO_PIN_5
#define LCD_PWR_GPIO_Port                 GPIOA
#define PWM_CP_Pin                        GPIO_PIN_6
#define PWM_CP_GPIO_Port                  GPIOA
#define ETH_CRS_DV_Pin                    GPIO_PIN_7
#define ETH_CRS_DV_GPIO_Port              GPIOA
#define I2C3_SCL_Pin                      GPIO_PIN_8
#define I2C3_SCL_GPIO_Port                GPIOA
#define UART1_TX_Pin                      GPIO_PIN_9
#define UART1_TX_GPIO_Port                GPIOA
#define UART1_RX_Pin                      GPIO_PIN_10
#define UART1_RX_GPIO_Port                GPIOA  
#define GPIO0_uP_Pin                      GPIO_PIN_11
#define GPIO0_uP_GPIO_Port                GPIOA
#define UART1_DE_Pin                      GPIO_PIN_12
#define UART1_DE_GPIO_Port                GPIOA 
#define SPI1_NSS_Pin                      GPIO_PIN_15
#define SPI1_NSS_GPIO_Port                GPIOA


/*    PB port     */
#define TA1TYPE_Pin                       GPIO_PIN_0
#define TA1TYPE_GPIO_Port                 GPIOB  
#define PP_ADC_Pin                        GPIO_PIN_0
#define PP_ADC_GPIO_Port                  GPIOB
#define VIN_ADC_Pin                       GPIO_PIN_1
#define VIN_ADC_GPIO_Port                 GPIOB
#define REM_ACTOFF_Pin                    GPIO_PIN_2
#define REM_ACTOFF_GPIO_Port              GPIOB
#define REM_ACTON_Pin                     GPIO_PIN_6
#define REM_ACTON_GPIO_Port               GPIOB  
#define SMB0_SDA_Pin                      GPIO_PIN_7
#define SMB0_SDA_GPIO_Port                GPIOB
#define SMB0_SCL_Pin                      GPIO_PIN_8
#define SMB0_SCL_GPIO_Port                GPIOB
#define SBC_CONN_Pin                      GPIO_PIN_9
#define SBC_CONN_GPIO_Port                GPIOB
#define IN2_Pin                           GPIO_PIN_10
#define IN2_GPIO_Port                     GPIOB
#ifdef HW_MP28947
#define IN3_Pin                           GPIO_PIN_9
#define IN3_GPIO_Port                     GPIOB
#define I2C1_SCL_Pin                      GPIO_PIN_10
#define I2C1_SCL_GPIO_Port                GPIOB
#define I2C1_SDA_Pin                      GPIO_PIN_11
#define I2C1_SDA_GPIO_Port                GPIOB
#else
#define RFID_DTC_Pin                      GPIO_PIN_15
#define RFID_DTC_GPIO_Port                GPIOB
#endif
#define ETH_TXD1_Pin                      GPIO_PIN_13
#define ETH_TXD1_GPIO_Port                GPIOB
#define SBC_PWR_Pin                       GPIO_PIN_14
#define SBC_PWR_GPIO_Port                 GPIOB

/*    PC port     */
#define TA_ADC_Pin                        GPIO_PIN_0
#define TA_ADC_GPIO_Port                  GPIOC
#define ETH_MDC_Pin                       GPIO_PIN_1
#define ETH_MDC_GPIO_Port                 GPIOC
#ifndef HW_MP28947  
#define SW_IN_ADC_Pin                     GPIO_PIN_2
#define SW_IN_ADC_GPIO_Port               GPIOC
#define TEMP_ADC_Pin                      GPIO_PIN_3
#define TEMP_ADC_GPIO_Port                GPIOC  
#else
#define TEMP_ADC_Pin                      GPIO_PIN_2
#define TEMP_ADC_GPIO_Port                GPIOC  
#endif  
#define ETH_RXD0_Pin                      GPIO_PIN_4
#define ETH_RXD0_GPIO_Port                GPIOC
#define ETH_RXD1_Pin                      GPIO_PIN_5
#define ETH_RXD1_GPIO_Port                GPIOC
#define VAC230_DET_Pin                    GPIO_PIN_8
#define VAC230_DET_GPIO_Port              GPIOC
#define I2C3_SDA_Pin                      GPIO_PIN_9
#define I2C3_SDA_GPIO_Port                GPIOC
#ifndef HW_MP28947  
#define RFID_PWR_Pin                      GPIO_PIN_13
#define RFID_PWR_GPIO_Port                GPIOC
#else
#define PRG_TX_Pin                        GPIO_PIN_12
#define PRG_TX_GPIO_Port                  GPIOC  
#define TAMPER_Pin                      GPIO_PIN_13
#define TAMPER_GPIO_Port                GPIOC
#endif
  
/*    PD port     */
#ifdef HW_MP28947  
#define SW_IN_ADC_Pin                     GPIO_PIN_0
#define SW_IN_ADC_GPIO_Port               GPIOD
#endif  
#define IN5_Pin                           GPIO_PIN_0
#define IN5_GPIO_Port                     GPIOD
#define ETH_RST_Pin                       GPIO_PIN_1
#define ETH_RST_GPIO_Port                 GPIOD
#define OUTBL1_M_Pin                      GPIO_PIN_3
#define OUTBL1_M_GPIO_Port                GPIOD
#define UART2_DE_Pin                      GPIO_PIN_4
#define UART2_DE_GPIO_Port                GPIOD
#define UART2_TX_Pin                      GPIO_PIN_5  
#define UART2_TX_GPIO_Port                GPIOD  
#define UART2_RX_Pin                      GPIO_PIN_6  
#define UART2_RX_GPIO_Port                GPIOD  
#define SGCBOB_Pin                        GPIO_PIN_7
#define SGCBOB_GPIO_Port                  GPIOD
#ifdef HW_MP28947
#define IN1_Pin                           GPIO_PIN_8
#define IN1_GPIO_Port                     GPIOD
#endif  
#define OUTBL1_P_Pin                      GPIO_PIN_8
#define OUTBL1_P_GPIO_Port                GPIOD
#define DIODO_ADC_Pin                     GPIO_PIN_9
#define DIODO_ADC_GPIO_Port               GPIOD
#define CNTCT_Pin                         GPIO_PIN_10
#define CNTCT_GPIO_Port                   GPIOD
#define OSC_50MHz_ON_Pin                  GPIO_PIN_11
#define OSC_50MHz_ON_GPIO_Port            GPIOD
#define PWRDWN1L_Pin                      GPIO_PIN_12
#define PWRDWN1L_GPIO_Port                GPIOD
#define IN6_Pin                           GPIO_PIN_13
#define IN6_GPIO_Port                     GPIOD
#define SW_PROG_Pin                       GPIO_PIN_14
#define SW_PROG_GPIO_Port                 GPIOD
#ifdef HW_MP28947
#define GPIO1_uP_Pin                      GPIO_PIN_0   // This pin is sconnected in HW_MP28947, it's defined just to avoid compiler errors */
#define GPIO1_uP_GPIO_Port                GPIOD  
#define PRG_RX_Pin                        GPIO_PIN_2
#define PRG_RX_GPIO_Port                  GPIOD
#define RFID_DTC_Pin                      GPIO_PIN_7
#define RFID_DTC_GPIO_Port                GPIOD
#define RFID_PWR_Pin                      GPIO_PIN_11
#define RFID_PWR_GPIO_Port                GPIOD
#define BOOT_EN_Pin                       GPIO_PIN_12
#define BOOT_EN_GPIO_Port                 GPIOD  
#define WIFI_IO9_Pin                      GPIO_PIN_13
#define WIFI_IO9_GPIO_Port                GPIOD
#else  
#define GPIO1_uP_Pin                      GPIO_PIN_1
#define GPIO1_uP_GPIO_Port                GPIOD
#endif  
#define WIFI_EN_Pin                       GPIO_PIN_15
#define WIFI_EN_GPIO_Port                 GPIOD

/*    PE port     */  
#define LCD_RS_Pin                        GPIO_PIN_0
#define LCD_RS_GPIO_Port                  GPIOE
#define LCD_CS_Pin                        GPIO_PIN_1
#define LCD_CS_GPIO_Port                  GPIOE
#define RACT_STATUS_Pin                   GPIO_PIN_2
#define RACT_STATUS_GPIO_Port             GPIOE
#define LCD_D0_Pin                        GPIO_PIN_3
#define LCD_D0_GPIO_Port                  GPIOE
#define RST_DEV_Pin                       GPIO_PIN_3
#define RST_DEV_GPIO_Port                 GPIOE
#define LCD_D1_Pin                        GPIO_PIN_4
#define LCD_D1_GPIO_Port                  GPIOE
#define LCD_D2_Pin                        GPIO_PIN_5
#define LCD_D2_GPIO_Port                  GPIOE
#define LCD_D3_Pin                        GPIO_PIN_6
#define LCD_D3_GPIO_Port                  GPIOE     
#define RCDM_Pin                          GPIO_PIN_7 
#define RCDM_GPIO_Port                    GPIOE
#define SINAPSI_Pin                       GPIO_PIN_8
#define SINAPSI_GPIO_Port                 GPIOE
#define DLED_A_Pin                        GPIO_PIN_9
#define DLED_A_GPIO_Port                  GPIOE
#define LCD_RW_Pin                        GPIO_PIN_10
#define LCD_RW_GPIO_Port                  GPIOE
#define SPI2_CS_1_Pin                     GPIO_PIN_10
#define SPI2_CS_1_GPIO_Port               GPIOE
#define DLED_B_Pin                        GPIO_PIN_11
#define DLED_B_GPIO_Port                  GPIOE
#define INT_INx_Pin                       GPIO_PIN_12
#define INT_INx_GPIO_Port                 GPIOE
#define DLED_C_Pin                        GPIO_PIN_13
#define DLED_C_GPIO_Port                  GPIOE
#define PWM_LUM_Pin                       GPIO_PIN_14
#define PWM_LUM_GPIO_Port                 GPIOE
#define SPI1_NSS2_Pin                     GPIO_PIN_15
#define SPI1_NSS2_GPIO_Port               GPIOE
#define SPI_485_Pin                       GPIO_PIN_14
#define SPI_485_GPIO_Port                 GPIOE 

/*    PH port     */    
#ifndef HW_MP28947
#define H_LED_Pin                         GPIO_PIN_1
#define H_LED_GPIO_Port                   GPIOH
#else
#define H_LED_Pin                         GPIO_PIN_3
#define H_LED_GPIO_Port                   GPIOA
#endif

#ifdef HW_MP28947
#define up_ZCR_Pin                        GPIO_PIN_10
#define up_ZCR_GPIO_Port                  GPIOB
#define SPI2_MISO_Pin                     GPIO_PIN_14
#define SPI2_MISO_GPIO_Port               GPIOB 
#define EN_PM1uP_Pin                      GPIO_PIN_0
#define EN_PM1uP_GPIO_Port                GPIOC
#define EN_PM3uP_Pin                      GPIO_PIN_2
#define EN_PM3uP_GPIO_Port                GPIOE
#define ETH_REF_CK_EN_Pin                 GPIO_PIN_12
#define ETH_REF_CK_EN_GPIO_Port           GPIOE
#define GP_INT_Pin                        GPIO_PIN_14
#define GP_INT_GPIO_Port                  GPIOE
#define SYN1PuP_Pin                       GPIO_PIN_6                 
#define SYN3PuP_Pin                       GPIO_PIN_2
#define SCS1PuP_Pin                       GPIO_PIN_15
#define SCS3PuP_Pin                       GPIO_PIN_8
#define CNTCT_PWM_Pin                     GPIO_PIN_5
#define CNTCT_PWM_GPIO_Port               GPIOE
#endif
  
/*----------------------------------------------------------------------------
 *                               Global variables
 *----------------------------------------------------------------------------
 */
#ifndef stm32f4xx_periph_init_c

extern UART_HandleTypeDef                  huart6;
extern I2C_HandleTypeDef                   hi2c1;

#endif

/*----------------------------------------------------------------------------
 *                               Private section
 *----------------------------------------------------------------------------
 */

#ifdef stm32f4xx_periph_init_c

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

#endif /* __STM32F4xx_PERIPH_INIT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

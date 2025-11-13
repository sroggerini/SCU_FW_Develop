/**
 ******************************************************************************
  * File Name          : LWIP.c
  * Description        : This file provides initialization code for LWIP
  *                      middleWare.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#if defined ( __CC_ARM )  /* MDK ARM Compiler */
#include "lwip/sio.h"
#endif /* MDK ARM Compiler */
#include "TCP_socket.h"
#include "main.h"
#include "telnet.h"
#include "ethInitTask.h"   

/*
*********************************** SCAME ************************************
**                                                                          **
**           LOCAL MACROS, TYPEDEF, STRUCTURE, ENUM                         **
**                                                                          **
******************************************************************************
*/
#define   NUM_RX_ETH_RETRY    ((unsigned int)1)
#define   RESET_ETH_COMPLETE
/*
*********************************** SCAME ************************************
**                                                                          **
**                            Local Variables                               **
**                                                                          **
****************************************************************************** 
*/ 
static uint16_t oldLinkStatus, errRxProcess;
static uint16_t newLinkStatus, tickLinkDown;

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 
struct netif gnetif;
ip4_addr_t ipaddr;
ip4_addr_t netmask;
ip4_addr_t gw;
uint8_t IP_ADDRESS[4];
uint8_t NETMASK_ADDRESS[4];
uint8_t GATEWAY_ADDRESS[4];
#ifdef GD32F4xx
extern ETH_HandleTypeDef EthHandle;  
#endif
/*
*********************************** SCAME ************************************
**                                                                          **
**                            Internal Function prototype                   **
**                                                                          **
******************************************************************************
*/
void Error_Handler(void);

/*
*********************************** SCAME ************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/

/**
  * LwIP initialization function
  */
void MX_LWIP_Init(void)
{
  uint32_t regvalue;

 /* IP addresses initialization */
  IP_ADDRESS[0]       = NetworkConfiguration.IpAddress[0];
  IP_ADDRESS[1]       = NetworkConfiguration.IpAddress[1];
  IP_ADDRESS[2]       = NetworkConfiguration.IpAddress[2];
  IP_ADDRESS[3]       = NetworkConfiguration.IpAddress[3];
  NETMASK_ADDRESS[0]  = NetworkConfiguration.SubnetMask[0];
  NETMASK_ADDRESS[1]  = NetworkConfiguration.SubnetMask[1];
  NETMASK_ADDRESS[2]  = NetworkConfiguration.SubnetMask[2];
  NETMASK_ADDRESS[3]  = NetworkConfiguration.SubnetMask[3];
  GATEWAY_ADDRESS[0]  = NetworkConfiguration.Gateway[0];
  GATEWAY_ADDRESS[1]  = NetworkConfiguration.Gateway[1];
  GATEWAY_ADDRESS[2]  = NetworkConfiguration.Gateway[2];
  GATEWAY_ADDRESS[3]  = NetworkConfiguration.Gateway[3];
  
  /* Initilialize the LwIP stack with RTOS */
  tcpip_init( NULL, NULL );

  /* IP addresses initialization without DHCP (IPv4) */
  IP4_ADDR(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
  IP4_ADDR(&netmask, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
  IP4_ADDR(&gw, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);

  /* add the network interface (IPv4/IPv6) with RTOS */
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

#ifdef HW_MP28947  
  if (heth.State == HAL_ETH_STATE_TIMEOUT)
    return;
#endif  
  
  /* Registers the default network interface */
  netif_set_default(&gnetif);

  if (netif_is_link_up(&gnetif))
  {
    /* When the netif is fully configured this function must be called */
    netif_set_up(&gnetif);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif);
  }

  HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &regvalue);  
  oldLinkStatus = (regvalue & PHY_LINKED_STATUS);
  tickLinkDown = 0;
  errRxProcess = 0U;

#if  LWIP_DHCP 
  // Se ho configurato il DHCP faccio partire il servizio
  // ATTENZIONE!!! MAI PROVATO!!!
  if (NetworkConfiguration.DHCP == 1)
    dhcp_start(&gnetif);
#endif

	// Socket TCP
  setHttpPort(NetworkConfiguration.portHttp);
  tcp_socket_init();

#ifdef MODBUS_TCP_EM_ETH
	// Socket TCP EM
  emLovato_socket_init();
#endif
}

/**
*
* @brief       Check the ethernet status
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/
void updateEthLinkStatus(void)
{
  uint32_t u32RegValue;
  uint16_t regvalue;

  /* Read Register Configuration */
  HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &u32RegValue);  
  regvalue = (uint16_t)u32RegValue;
  newLinkStatus = (regvalue & PHY_LINKED_STATUS);

  // Controllo se è cambiato lo stato del link del fisico ETH
  if ((oldLinkStatus != newLinkStatus) || ((heth.Instance->DMASR & ETH_DMASR_RPS) == (0x4U << ETH_DMASR_RPS_Pos)))
      //((heth.Instance->DMAMFBOCR & ETH_DMAMFBOCR_MFC) != 0) || ((heth.Instance->DMASR & ETH_DMASR_RPS) != (0x3U << ETH_DMASR_RPS_Pos)) )
  {
    // Se siamo passati da un link DOWN ad un link UP...
    if (newLinkStatus == PHY_LINKED_STATUS)
    {
      /* if RPS = 100: Suspended, Receive descriptor unavailable. Link active but problems on Rx. Retry and restarts if condition remains */
      if ((errRxProcess > NUM_RX_ETH_RETRY) && ((heth.Instance->DMASR & ETH_DMASR_RPS) == (0x4U << ETH_DMASR_RPS_Pos)))
      {
        RefreshWatchDog = FALSE;
      }
      else
      {
        if ((heth.Instance->DMASR & ETH_DMASR_RPS) == (0x4U << ETH_DMASR_RPS_Pos))
        {
          errRxProcess++;
        }
        else
        {
          errRxProcess = 0U;
        }
      }
#ifndef RESET_ETH_COMPLETE
      tPrintf("ETH MAC Restart\n\r");
      /* Read control Register */
      HAL_ETH_ReadPHYRegister(&heth, PHY_BCR, &u32RegValue);
      regvalue = (uint16_t)u32RegValue;
      /* Put the PHY isolated from RMII */
      regvalue |= PHY_PHY_ISO_FROM_RMII;
      if((HAL_ETH_WritePHYRegister(&heth, PHY_BCR, regvalue)) == HAL_OK)
      {
        // ...rimuovo la vecchia interfaccia del fisico...
        netif_remove(&gnetif);
        
        // ... e la ricreo come in LWIP_init
        netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);
        netif_set_default(&gnetif);
        netif_set_up(&gnetif);
        osDelay(200);

        /* Reconnect  PHY to RMII */
        regvalue = (uint16_t)u32RegValue;
        HAL_ETH_WritePHYRegister(&heth, PHY_BCR, regvalue);
      }
#else
      tPrintf("ETH: Link is up!!\n\r");
      /* link is up: notify this information */
      netif_set_link_up(&gnetif);
      netif_set_up(&gnetif); 
      tickLinkDown = (uint16_t)0;
#endif
      // Questo perchè se la CPU parte con il link DOWN poi non si riprendeva più
    }
    else
    {
      netif_set_link_down(&gnetif);
      tPrintf("ETH: Link is down!!\n\r");
      freeHtmlMemory();
    }
  }
  oldLinkStatus = newLinkStatus;

  if (newLinkStatus != PHY_LINKED_STATUS)
  {
    if (tickLinkDown >  (uint16_t)8)   // 8 * ETH_POLLING_TIME_TICK = 1600msec
    {
      /* Put the PHY in reset mode */
      if((HAL_ETH_WritePHYRegister(&heth, PHY_BCR, PHY_RESET)) == HAL_OK)
      {
        osDelay(2000);
      }
      tickLinkDown = (uint16_t)0;
    }
    else
    {
      tickLinkDown++;
    }
  }
}

/**
*
* @brief       Put the ethernet in power down 
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/
void ethPowerDown(void)
{
  uint32_t regvalue;

  /* Read Register Configuration */
  HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &regvalue);  

  /* Set Power down bit in BCR register  */
  regvalue |= PHY_POWERDOWN;
  
  HAL_ETH_WritePHYRegister(&heth, PHY_BCR, regvalue);
#ifdef NOT_USED
  /* Set slow oscillator mode in AFE1 register  */
  HAL_ETH_WritePHYRegister(&heth, PHY_AFE1_CTRL, PHY_CLOCKDOWN);
#endif

}

/**
*
* @brief       Put the ethernet in power up 
*
* @param [in]  none  
*  
* @retval      none 
*  
****************************************************************/
void ethPowerUp(void)
{
  uint32_t regvalue;

  /* Read Register Configuration */
  HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &regvalue);  

  /* Set Power down bit in BCR register  */
  regvalue &= (~PHY_POWERDOWN);
  
  HAL_ETH_WritePHYRegister(&heth, PHY_BCR, regvalue);
#ifdef NOT_USED
  /* Set slow oscillator mode in AFE1 register  */
  HAL_ETH_WritePHYRegister(&heth, PHY_AFE1_CTRL, PHY_CLOCKDOWN);
#endif
}


#ifdef USE_OBSOLETE_USER_CODE_SECTION_4
/* Kept to help code migration. (See new 4_1, 4_2... sections) */
/* Avoid to use this user section which will become obsolete. */
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
#endif

#if defined ( __CC_ARM )  /* MDK ARM Compiler */
/**
 * Opens a serial device for communication.
 *
 * @param devnum device number
 * @return handle to serial device if successful, NULL otherwise
 */
sio_fd_t sio_open(u8_t devnum)
{
  sio_fd_t sd;

/* USER CODE BEGIN 7 */
  sd = 0; // dummy code
/* USER CODE END 7 */
	
  return sd;
}

/**
 * Sends a single character to the serial device.
 *
 * @param c character to send
 * @param fd serial device handle
 *
 * @note This function will block until the character can be sent.
 */
void sio_send(u8_t c, sio_fd_t fd)
{
/* USER CODE BEGIN 8 */
/* USER CODE END 8 */
}

/**
 * Reads from the serial device.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received - may be 0 if aborted by sio_read_abort
 *
 * @note This function will block until data can be received. The blocking
 * can be cancelled by calling sio_read_abort().
 */
u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 9 */
  recved_bytes = 0; // dummy code
/* USER CODE END 9 */	
  return recved_bytes;
}

/**
 * Tries to read from the serial device. Same as sio_read but returns
 * immediately if no data is available and never blocks.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received
 */
u32_t sio_tryread(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 10 */
  recved_bytes = 0; // dummy code
/* USER CODE END 10 */	
  return recved_bytes;
}
#endif /* MDK ARM Compiler */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

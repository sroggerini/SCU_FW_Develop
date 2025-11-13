/*
 *  scame-protocol-unit - Vehicle communication device
 *
 *  Copyright (C) 2017  Manuele Conti (manuele.conti@archimede-energia.com)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * This code is made available on the understanding that it will not be
 * used in safety-critical situations without a full and competent review.
 */

/* For standard defines and includes */

#include "spi_drv.h"
#include "mxconstants.h"
#include "stm32f4xx.h"
#include "ExtFlash.h"

SPI_HandleTypeDef spi_handle1;
SPI_HandleTypeDef spi_handle2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

volatile uint8_t transfer_state_1 = transfer_state_complete_ok;
volatile uint8_t transfer_state_2 = transfer_state_complete_ok;

/** Initialize spi channel
 *
 * Clock SPI = FREQ_PER / (64 * 4) = 156.25 kHz
 *
 * @param channel Channel spi number.
 *
 * @return  0 OK
 * @return -1 Error init
 *
 */
int spi_init(unsigned channel, uint8_t speed, uint16_t flags)
{
    if(channel == 1) 
    {
        spi_handle1.Instance = SPI1;
        if(flags & SPI_OPEN_MSTEN)
          spi_handle1.Init.Mode = SPI_MODE_MASTER;
        else
          spi_handle1.Init.Mode = SPI_MODE_SLAVE;
        spi_handle1.Init.Direction = SPI_DIRECTION_2LINES;
        
        if (flags & SPI_OPEN_MODE16)
          spi_handle1.Init.DataSize = SPI_DATASIZE_16BIT;
        else
          spi_handle1.Init.DataSize = SPI_DATASIZE_8BIT;
        
        spi_handle1.Init.CLKPolarity = (flags & 0x2) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
        spi_handle1.Init.CLKPhase = (flags & 0x1) ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
        
        spi_handle1.Init.NSS = SPI_NSS_SOFT;
        if (speed == SPI_CK_SPEED_125K)
          spi_handle1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
        else if (speed == SPI_CK_SPEED_250K)
          spi_handle1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
        else if (speed == SPI_CK_SPEED_500K)
          spi_handle1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
        spi_handle1.Init.FirstBit = SPI_FIRSTBIT_MSB;
        spi_handle1.Init.TIMode = SPI_TIMODE_DISABLE;
        spi_handle1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        spi_handle1.Init.CRCPolynomial = 10;
        if (HAL_SPI_Init(&spi_handle1) != HAL_OK) 
        {
          return -1;
        }
        
        /* SPI1 interrupt activation */
        HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(SPI1_IRQn);
        
    }
    else if (channel == 2) 
    {
        spi_handle2.Instance = SPI2;
        if (flags & SPI_OPEN_MSTEN)
          spi_handle2.Init.Mode = SPI_MODE_MASTER;
        else
          spi_handle2.Init.Mode = SPI_MODE_SLAVE;
        spi_handle2.Init.Direction = SPI_DIRECTION_2LINES;
        
        if (flags & SPI_OPEN_MODE16)
          spi_handle2.Init.DataSize = SPI_DATASIZE_16BIT;
        else
          spi_handle2.Init.DataSize = SPI_DATASIZE_8BIT;
        
        spi_handle2.Init.CLKPolarity = (flags & 0x2) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
        spi_handle2.Init.CLKPhase = (flags & 0x1) ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
        
        spi_handle2.Init.NSS = SPI_NSS_SOFT;
        if (speed == SPI_CK_SPEED_125K)
          spi_handle2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
        else if (speed == SPI_CK_SPEED_250K)
          spi_handle2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
        else if (speed == SPI_CK_SPEED_500K)
          spi_handle2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
        spi_handle2.Init.FirstBit = SPI_FIRSTBIT_MSB;
        spi_handle2.Init.TIMode = SPI_TIMODE_DISABLE;
        spi_handle2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        spi_handle2.Init.CRCPolynomial = 10;
        
        if (HAL_SPI_Init(&spi_handle2) != HAL_OK) 
        {
          return -1;
        }
    }
    
    spi_set_cs(channel, 0, 1);
        
    return 0;
}

/** Set CS SPI
 *
 * @param channel Channel spi number.
 * @param csn Chip select number
 * @param value Set state value
 *
 *
 */
void spi_set_cs(unsigned channel, unsigned csn, unsigned value)
{
    UNUSED(csn);
    
    if ((channel < 1) || (channel > 2))
      return ;
    
    if (channel == 1) 
    {
      switch (value)
      {
        case GPIO_PIN_RESET:
          NSS2_TO_LOW_LEVEL     // SPI1_NSS2 @ low level    
          break;
        case GPIO_PIN_SET:
          NSS2_TO_HIGH_LEVEL    // SPI1_NSS2 @ high level
          break;
      }        
    }
    else if(channel == 2) 
    {
      // HAL_GPIO_WritePin(SPI2_CS_1_GPIO_Port, SPI2_CS_1_Pin, (value ? GPIO_PIN_SET : GPIO_PIN_RESET));
    }
}

/** Write on spi
 *
 * @param channel Channel spi number.
 * @param src source buffer
 * @param n number of byte
 *
 * @return   0 write performing
 * @return  -1 error
 * @return  >0 byte already wrote
 *
 */
int spi_write_deferred(const unsigned channel, const void *src, size_t n)
{
    static uint8_t st = 0;
    
    if ((channel < 1) || (channel > 2) || (n == 0))
      return -1;
    
    switch (st) 
    {
      case 0:
          if (channel == 1) 
          {
            transfer_state_1 = transfer_state_pending;
            if(HAL_SPI_Transmit_IT(&spi_handle1, (uint8_t *)src, n) != HAL_OK) 
            {
              transfer_state_1 = transfer_state_complete_error;
              return -1;
            }
          }
          else if (channel == 2) 
          {
            transfer_state_2 = transfer_state_pending;
            if(HAL_SPI_Transmit_IT(&spi_handle2, (uint8_t *)src, n) != HAL_OK) 
            {
              transfer_state_2 = transfer_state_complete_error;
              return -1;
            }
          }
          st = 1;
        break;
    
      case 1:
          if (channel == 1) 
          {
            if (transfer_state_1 == transfer_state_complete_ok) 
            {
              st = 0;
              return(spi_handle1.TxXferSize - spi_handle1.TxXferCount);
            }
            if (transfer_state_1 == transfer_state_complete_error) 
            {
              st = 0;
              return -1;
            }
          }
          else if (channel == 2) 
          {
            if (transfer_state_2 == transfer_state_complete_ok) 
            {
              st = 0;
              return(spi_handle2.TxXferSize - spi_handle2.TxXferCount);
            }
            if (transfer_state_2 == transfer_state_complete_error) 
            {
            st = 0;
            return -1;
          }
      }
      break;
  
    default:
      break;
  }
  
  return 0;
}

/** Blocking write on spi
 *
 * @param channel Channel spi number.
 * @param src source buffer
 * @param n number of byte
 *
 * @return   0 Ok
 * @return -1 error
 *
 */
int spi_write(const unsigned channel, const void *src, size_t n)
{
  int8_t ret = 0;
  
  if ((channel < 1) || (channel > 2) || (n == 0))
    return -1;
  
  do 
  {
    ret = spi_write_deferred(channel, src, n);
  } while (ret == 0);
  
  if (ret == -1)
    return ret;
  
  return 0;
}

/** Read on spi
 *
 * @param channel Channel spi number.
 * @param dst destination buffer
 * @param n number of byte
 *
 * @return   0 read performing
 * @return  -1 error
 * @return  >0 byte already read
 *
 */
int spi_read_deferred(const unsigned channel, void *dst, size_t n)
{
	static uint8_t st = 0;
	static uint32_t to = 0;
	static HAL_StatusTypeDef ret;

	if ((channel < 1) || (channel > 2) || (n == 0))
          return -1;

	switch (st) 
        {
          case 0:
            if (channel == 1) 
            {
              transfer_state_1 = transfer_state_pending;
              if (HAL_SPI_Receive_IT(&spi_handle1, (uint8_t *)dst, n) != HAL_OK) 
              {
                transfer_state_1 = transfer_state_complete_error;
                return -1;
              }
            }
            else if (channel == 2) 
            {
              to = HAL_GetTick();
              transfer_state_2 = transfer_state_pending;
              ret = HAL_SPI_Receive_IT(&spi_handle2, (uint8_t *)dst, n);
              if (ret != HAL_OK) 
              {
                transfer_state_2 = transfer_state_complete_error;
                return -1;
              }
            }
            st = 1;
            break;
  
          case 1:
            if (channel == 1) 
            {
              if (transfer_state_1 == transfer_state_complete_ok) 
              {
                st = 0;
                return (spi_handle1.RxXferSize - spi_handle1.RxXferCount);
              }
              if (transfer_state_1 == transfer_state_complete_error) 
              {
                st = 0;
                return -1;
              }
            }
            else if (channel == 2) 
            {
              if (transfer_state_2 == transfer_state_complete_ok) 
              {
                st = 0;
                return (spi_handle2.RxXferSize - spi_handle2.RxXferCount);
              }
              if ((transfer_state_2 == transfer_state_complete_error) || ((HAL_GetTick() - to) > 1000)) 
              {
                st = 0;
                return -1;
              }
            }
            break;
  
          default:
		break;
	}

	return 0;
}

/** Read on spi
 *
 * @param channel Channel spi number.
 * @param dst destination buffer
 * @param n number of byte
 *
 * @return   0 read performing
 * @return  -1 error
 * @return  >0 byte already read
 *
 */
int spi_writeread_deferred(const unsigned channel, void *src, void *dst, size_t n)
{
  static uint8_t st = 0;
  static uint32_t to = 0;
  static HAL_StatusTypeDef ret;
  
  if ((channel < 1) || (channel > 2) || (n == 0))
    return -1;
  
  switch (st) 
  {
  case 0:
    if (channel == 1) 
    {
      transfer_state_1 = transfer_state_pending;
      ret = HAL_SPI_TransmitReceive_IT(&spi_handle1, (uint8_t *)src, (uint8_t *)dst, n);
      if (ret != HAL_OK) 
      {
        transfer_state_1 = transfer_state_complete_error;
        return -1;
      }
    }
    else if (channel == 2) 
    {
      to = HAL_GetTick();
      transfer_state_2 = transfer_state_pending;
      ret = HAL_SPI_TransmitReceive_DMA(&spi_handle2, (uint8_t *)src, (uint8_t *)dst, n);
      if (ret != HAL_OK) 
      {
        transfer_state_2 = transfer_state_complete_error;
        return -1;
      }
    }
    st = 1;
    break;
  
  case 1:
    if (channel == 1) 
    {
      if (transfer_state_1 == transfer_state_complete_ok) 
      {
        st = 0;
        return (spi_handle1.RxXferSize - spi_handle1.RxXferCount);
      }
      if (transfer_state_1 == transfer_state_complete_error) 
      {
        st = 0;
        return -1;
      }
    }
    else if (channel == 2) 
    {
      if (transfer_state_2 == transfer_state_complete_ok) 
      {
        st = 0;
        return (spi_handle2.RxXferSize - spi_handle2.RxXferCount);
      }
      if ((transfer_state_2 == transfer_state_complete_error) || ((HAL_GetTick() - to) > 1000)) 
      {
        HAL_SPI_Abort_IT(&spi_handle2);
        st = 0;
        return -1;
      }
    }
    break;
  
  default:
    break;
  }

	return 0;
}

/** Blocking read on spi
 *
 * @param channel Channel spi number.
 * @param dst destination buffer
 * @param n number of byte
 *
 * @return   0 read performing
 * @return -1 error
 *
 */
int spi_read(const unsigned channel, void *dst, size_t n)
{
  int8_t ret = 0;
  
  if ((channel < 1) || (channel > 2) || (n == 0))
          return -1;  
  
  do {
          ret = spi_read_deferred(channel, dst, n);
  } while (ret == 0);
  
  if (ret == -1)
          return ret;
  
  return 0;
}

/** Blocking read on spi
 *
 * @param channel Channel spi number.
 * @param dst destination buffer
 * @param n number of byte
 *
 * @return   0 read performing
 * @return -1 error
 *
 */
int spi_writeread(const unsigned channel, void *src, void *dst, size_t n)
{
  int8_t ret = 0;
  
  if ((channel < 1) || (channel > 2) || (n == 0))
    return -1;
  
  
  do {
    ret = spi_writeread_deferred(channel, src, dst, n);
  } while (ret == 0);
  
  if (ret == -1)
    return ret;
  
  return 0;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &spi_handle1)
    transfer_state_1 = transfer_state_complete_ok;
  else if (hspi == &spi_handle2)
    transfer_state_2 = transfer_state_complete_ok;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &spi_handle1)
    transfer_state_1 = transfer_state_complete_ok;
  else if (hspi == &spi_handle2)
    transfer_state_2 = transfer_state_complete_ok;
  
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &spi_handle1)
    transfer_state_1 = transfer_state_complete_ok;
  else if (hspi == &spi_handle2)
    transfer_state_2 = transfer_state_complete_ok;
}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &spi_handle1)
    transfer_state_1 = transfer_state_complete_error;
  else if (hspi == &spi_handle2)
    transfer_state_2 = transfer_state_complete_error;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &spi_handle1)
    transfer_state_1 = transfer_state_complete_error;
  else if (hspi == &spi_handle2)
    transfer_state_2 = transfer_state_complete_error;
}



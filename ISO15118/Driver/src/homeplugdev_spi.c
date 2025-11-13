/*
 *  scame-devolo-greenphy - PLC Modem device
 *
 *  Copyright (C) 2010  Manuele Conti (manuele.conti@archimede-energia.com)
 *  Copyright (C) 2019  Bruno Zavettieri (bruno.zavettieri@archimede-energia.com)
 *  Copyright (C) 2019  Luca Valtellina (info@vlengineering.it)
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

#include "stm32f4xx_hal.h"
#include "homeplugdev_spi.h"
#include "homeplugdev.h"
#include "spi_drv.h"


homeplugdev_info info;

static int8_t homeplug_err;

static int8_t homeplugdev_send_with_crc(const void *data, size_t len);
static int8_t transactionCmdTxRx(enum homeplugdev_cmd cmd, void *ptr, size_t len);

uint16_t crc16(uint16_t crc, const uint8_t *buf, size_t length)
{
  uint8_t x = 0U;
  
  while (length--) 
  {
    x = crc >> 8 ^ *(buf++);
    x ^= x >> 4;
    crc = (crc << 8) ^ ((uint16_t) (x << 12)) ^
          ((uint16_t) (x << 5)) ^ ((uint16_t) x);
  }
  
  return crc;
}

void homeplugdev_init(void)
{
    homeplugdev_hw_reset();
    timerCounter[timerCCSHomeplug] = 10000;
    homeplug_err = 0;
    memset(&info, 0, sizeof(info));
    spi_set_cs(1, 0, SPI_CS_IDLE);
}

int8_t homeplugdev_error(void)
{
   return homeplug_err;
}

int8_t homeplugdev_readwrite_info(void)
{
   return transactionCmdTxRx(TRINF, &info, sizeof(info));
}

int8_t homeplugdev_send_cmd_and_crc(enum homeplugdev_cmd cmd)
{
    int8_t err = 0;
  
    err = homeplugdev_send_with_crc(&cmd, HOMEPLUGDEV_CMDSIZE);
    if (err)
      return err;
  
    return 0;
}

int8_t homeplugdev_send_with_crc(const void *data, size_t len)
{
    int8_t err = 0;
    uint8_t *trmBuff = NULL;
    uint16_t crc = 0xFFFF;
  
    if (data == NULL)
      return -1;
    
    trmBuff = (uint8_t *)malloc(sizeof(uint8_t) * (len + HOMEPLUGDEV_CRCSIZE));

    memcpy(trmBuff, data, len);
    crc = crc16(0xFFFF, data, len);
    memcpy(trmBuff + len, &crc, HOMEPLUGDEV_CRCSIZE);

    err = spi_write(1, trmBuff, (len + HOMEPLUGDEV_CRCSIZE));

    free(trmBuff);
    return err;
}

int8_t homeplugdev_txrx_data_check_crc(void *data, size_t len)
{
    int8_t err = 0;
    uint8_t *trmBuff = NULL;
    uint8_t *rcvBuff = NULL;
    uint16_t rcrc, crc = 0xFFFF;
    
    if (data == NULL)
      return -1;
    
    trmBuff = (uint8_t *)malloc(sizeof(uint8_t) * (len + HOMEPLUGDEV_CRCSIZE));
    rcvBuff = (uint8_t *)malloc(sizeof(uint8_t) * (len + HOMEPLUGDEV_CRCSIZE));
    memcpy(trmBuff, data, len);
    crc = crc16(0xFFFF, data, len);
    memcpy(trmBuff + len, &crc, HOMEPLUGDEV_CRCSIZE);
    
    err = spi_writeread(1, trmBuff, rcvBuff, (len + HOMEPLUGDEV_CRCSIZE));
    if (err)
      goto exit;
    
    crc = crc16(0xFFFF, rcvBuff, len);
    memcpy(&rcrc, rcvBuff + len, HOMEPLUGDEV_CRCSIZE);
    if (crc != rcrc) 
    {
      err = -1;
      homeplug_err = 1;
      goto exit;
    } 
    else 
    {
      homeplug_err = 0;
    }

    memcpy(data, rcvBuff, len);

exit:
    free(trmBuff);
    free(rcvBuff);
    return err;
}

int8_t transactionCmdTxRx(enum homeplugdev_cmd cmd, void *ptr, size_t len)
{
    static homeplugdev_state_t st;
    int8_t err = 0;
  
    switch(st) 
    {
      case ST_SEND_CMD:
        /* Enable CS */
        spi_set_cs(1, 0, SPI_CS_BUSY);
        /* Send cmd */
        err = homeplugdev_send_cmd_and_crc(cmd);
        if (err != -1) 
        {
            /* Next */
            st = ST_SEND_DATA;
        }
        /* Disable CS */
        spi_set_cs(1, 0, SPI_CS_IDLE);
        break;

      case ST_SEND_DATA:
        /* Enable CS */
        spi_set_cs(1, 0, SPI_CS_BUSY);
        /* Send data */
        err = homeplugdev_txrx_data_check_crc(ptr, len);
        /* Disable CS */
        spi_set_cs(1, 0, SPI_CS_IDLE);
        /* Next */
        st = ST_SEND_CMD;
        break;

      default:
        break;
    }
    
    return err;
}

void homeplugdev_hw_reset(void)
{
//   /* Reset Devolo */
//   HAL_GPIO_WritePin(RESET_DEVOLO_GPIO_Port, RESET_DEVOLO_Pin, GPIO_PIN_RESET);
//   HAL_Delay(5);
//   HAL_GPIO_WritePin(RESET_DEVOLO_GPIO_Port, RESET_DEVOLO_Pin, GPIO_PIN_SET);
}


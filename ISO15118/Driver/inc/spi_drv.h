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

#include <stdint.h>
#include <stdlib.h>
#include "stm32f4xx_periph_init.h"

#ifndef _SPI_DRV_H
#define _SPI_DRV_H


#define SPI_CH_MAX		2

#define SPI_CK_SPEED_125K    0
#define SPI_CK_SPEED_250K    1
#define SPI_CK_SPEED_500K    2

#define SPI_OPEN_MSTEN  (1 << 7)
#define SPI_OPEN_MODE8  (0 << 8)
#define SPI_OPEN_MODE16 (1 << 8)

#define SPI_MODE_0 (0)
#define SPI_MODE_1 (1)
#define SPI_MODE_2 (2)
#define SPI_MODE_3 (3)

#define SPI_CS_IDLE 1
#define SPI_CS_BUSY 0

typedef enum {
	transfer_mode_read = 0,
	transfer_mode_write
} e_spi_drv_transfer_mode;

typedef enum {
	transfer_state_complete_ok = 0,
	transfer_state_pending,
	transfer_state_complete_error = 255
} e_spi_drv_transfer_state;

int spi_init(unsigned channel, uint8_t speed, uint16_t flags);
void spi_set_cs(unsigned channel, unsigned csn, unsigned value);
int spi_write_deferred(const unsigned channel, const void *src, size_t n);
int spi_write(const unsigned channel, const void *src, size_t n);
int spi_read_deferred(const unsigned channel, void *dst, size_t n);
int spi_read(const unsigned, void *, size_t);
int spi_writeread_deferred(const unsigned channel, void *src, void *dst, size_t n);
int spi_writeread(const unsigned channel, void *src, void *dst, size_t n);





#endif

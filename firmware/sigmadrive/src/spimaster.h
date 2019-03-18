/*
 *  Sigmadrone
 *  Copyright (c) 2013-2015 The Sigmadrone Developers
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
 *  Martin Stoilov <martin@sigmadrone.org>
 *  Svetoslav Vassilev <svassilev@sigmadrone.org>
 */

#ifndef _SPIMASTER_H_
#define _SPIMASTER_H_

#include <stdint.h>
#include <vector>
#include "stm32f4xx.h"
#include "stm32f4xx_ll_spi.h"
#include "gpiopin.h"

class SPIMaster
{
public:
	enum DataWidth {
		Data8Bit = LL_SPI_DATAWIDTH_8BIT,
		Data16Bit = LL_SPI_DATAWIDTH_16BIT,
	};

	enum BaudRatePrescaler {
		Div2 = LL_SPI_BAUDRATEPRESCALER_DIV2,
		Div4 = LL_SPI_BAUDRATEPRESCALER_DIV4,
		Div8 = LL_SPI_BAUDRATEPRESCALER_DIV8,
		Div16 = LL_SPI_BAUDRATEPRESCALER_DIV16,
		Div32 = LL_SPI_BAUDRATEPRESCALER_DIV32,
		Div64 = LL_SPI_BAUDRATEPRESCALER_DIV64,
		Div128 = LL_SPI_BAUDRATEPRESCALER_DIV128,
		Div256 = LL_SPI_BAUDRATEPRESCALER_DIV256,
	};

	enum ClockPolarity {
		PolarityLow = LL_SPI_POLARITY_LOW,
		PolarityHigh = LL_SPI_POLARITY_HIGH,
	};

	enum ClockPhase {
		Edge1 = LL_SPI_PHASE_1EDGE,			/*!< First clock transition is the first data capture edge  */
		Edge2 = LL_SPI_PHASE_2EDGE,			/*!< Second clock transition is the first data capture edge */
	};

	SPIMaster(SPI_TypeDef* spi_device = SPI3,
			DataWidth data_width = Data8Bit,
			BaudRatePrescaler clk_prescale = Div32,
			ClockPolarity clk_polarity = PolarityLow,
			ClockPhase clk_phase = Edge2,
			const std::vector<GPIOPin>& data_pins = {}, const std::vector<GPIOPin>& cs_pins = {});
	virtual ~SPIMaster();

	uint8_t spi_write_read8(uint8_t data = 0x0);
	uint16_t spi_write_read16(uint16_t data = 0x0);
	void spi_chip_select(uint8_t chip, bool select);

protected:
	SPI_TypeDef* SPIx_;
	std::vector<GPIOPin> data_pins_;
	std::vector<GPIOPin> cs_pins_;
};

#endif /* _SPIMASTER_H_ */

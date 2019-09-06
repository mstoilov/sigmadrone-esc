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

#include <cstring>
#include <stdexcept>
#include "spimaster.h"
#include "stm32f7xx_ll_bus.h"

SPIMaster::SPIMaster(SPI_TypeDef* spi_device,
		DataWidth data_width,
		BaudRatePrescaler clk_prescale,
		ClockPolarity clk_polarity,
		ClockPhase clk_phase,
		const std::vector<GPIOPin>& data_pins, const std::vector<GPIOPin>& cs_pins)
	: data_pins_(data_pins)
	, cs_pins_(cs_pins)
{
	for (auto& data_pin : data_pins_)
		data_pin.Init();
	for (auto& cs_pin : cs_pins_) {
		cs_pin.Init();
		LL_GPIO_SetOutputPin(cs_pin.gpio_port_, cs_pin.Pin);
	}
	LL_SPI_InitTypeDef spi_init;
	memset(&spi_init, 0, sizeof(spi_init));

	SPIx_ = spi_device;

	spi_init.BaudRate = clk_prescale;
	spi_init.TransferDirection = LL_SPI_FULL_DUPLEX;
	spi_init.ClockPolarity = clk_polarity;
	spi_init.ClockPhase = clk_phase;
	spi_init.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	spi_init.CRCPoly = 10;
	spi_init.DataWidth = data_width;
	spi_init.BitOrder = LL_SPI_MSB_FIRST;
	spi_init.NSS = LL_SPI_NSS_SOFT;
	spi_init.Mode = LL_SPI_MODE_MASTER;

	if (spi_device == SPI1)
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	else if (spi_device == SPI2)
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
	else if (spi_device == SPI3)
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
#ifdef SPI4
	else if (spi_device == SPI4)
		LL_APB1_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI4);
#endif
#ifdef SPI5
	else if (spi_device == SPI5)
		LL_APB1_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI5);
#endif
#ifdef SPI6
	else if (spi_device == SPI6)
		LL_APB1_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI5);
#endif
	ErrorStatus error = LL_SPI_Init(SPIx_, &spi_init);
	if (error != SUCCESS) {
		printf("SPI init failed\n");
	}
}

SPIMaster::~SPIMaster()
{
	LL_SPI_DeInit(SPIx_);
}

uint8_t SPIMaster::spi_write_read8(uint8_t data)
{
	uint8_t received = 0;

	LL_SPI_Enable(SPIx_);

	/* Send a Byte through the SPI peripheral */
	/* Read byte from the SPI bus */
	LL_SPI_TransmitData8(SPIx_, data);
	while (LL_SPI_IsActiveFlag_RXNE(SPIx_) == 0)
		;
	received = LL_SPI_ReceiveData8(SPIx_);
	LL_SPI_Disable(SPIx_);

	return received;
}

uint16_t SPIMaster::spi_write_read16(uint16_t data)
{
	uint16_t received = 0;

	LL_SPI_Enable(SPIx_);

	/* Send a Byte through the SPI peripheral */
	/* Read byte from the SPI bus */
	LL_SPI_TransmitData16(SPIx_, data);
	while (LL_SPI_IsActiveFlag_RXNE(SPIx_) == 0)
		;
	received = LL_SPI_ReceiveData16(SPIx_);
	LL_SPI_Disable(SPIx_);

	return received;
}


void SPIMaster::spi_chip_select(uint8_t chip, bool select)
{
	if (chip < cs_pins_.size()) {
		if (select)
			LL_GPIO_ResetOutputPin(cs_pins_[chip].gpio_port_, cs_pins_[chip].Pin);
		else
			LL_GPIO_SetOutputPin(cs_pins_[chip].gpio_port_, cs_pins_[chip].Pin);
	}
}



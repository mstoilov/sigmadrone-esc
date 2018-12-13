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

#ifndef GPIOINIT_H_
#define GPIOINIT_H_

//#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_gpio.h"
#include "pinnames.h"

class GPIOPin : public LL_GPIO_InitTypeDef {
public:
	GPIOPin(GPIO_TypeDef* gpio_port,
			uint16_t pin,				// pin number (0..15)
			uint32_t mode,				// GPIO_mode_define
			uint32_t pull,				// GPIO_pull_define
			uint32_t speed,				// GPIO_speed_define
			uint32_t af,				// Peripheral to be connected to the selected pin (Alternative function)
			uint32_t output = LL_GPIO_OUTPUT_PUSHPULL
			);
	GPIOPin(PinName pn,
			uint32_t mode,				// GPIO_mode_define
			uint32_t pull,				// GPIO_pull_define
			uint32_t speed,				// GPIO_speed_define
			uint32_t af,			    // Peripheral to be connected to the selected pin (Alternative function)
			uint32_t output = LL_GPIO_OUTPUT_PUSHPULL
			);
	~GPIOPin();

	void Init() const;
	void CleanUp() const;
public:
	GPIO_TypeDef* gpio_port_;
	uint16_t pinnum_;
	PinName pn_;
};

#endif /* GPIOINIT_H_ */

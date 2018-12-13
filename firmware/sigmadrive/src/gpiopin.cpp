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

#include "stm32f4xx_ll_bus.h"
#include "gpiopin.h"

#ifndef GET_GPIO_SOURCE
#define GET_GPIO_SOURCE(__GPIOx__) \
(((uint32_t)(__GPIOx__) == ((uint32_t)GPIOA_BASE))? (uint32_t)0 :\
 ((uint32_t)(__GPIOx__) == ((uint32_t)(GPIOA_BASE + 0x0400)))? (uint32_t)1 :\
 ((uint32_t)(__GPIOx__) == ((uint32_t)(GPIOA_BASE + 0x0800)))? (uint32_t)2 :\
 ((uint32_t)(__GPIOx__) == ((uint32_t)(GPIOA_BASE + 0x0C00)))? (uint32_t)3 :\
 ((uint32_t)(__GPIOx__) == ((uint32_t)(GPIOA_BASE + 0x1000)))? (uint32_t)4 :\
 ((uint32_t)(__GPIOx__) == ((uint32_t)(GPIOA_BASE + 0x1400)))? (uint32_t)5 :\
 ((uint32_t)(__GPIOx__) == ((uint32_t)(GPIOA_BASE + 0x1800)))? (uint32_t)6 :\
 ((uint32_t)(__GPIOx__) == ((uint32_t)(GPIOA_BASE + 0x1C00)))? (uint32_t)7 :\
 ((uint32_t)(__GPIOx__) == ((uint32_t)(GPIOA_BASE + 0x2000)))? (uint32_t)8 :\
 ((uint32_t)(__GPIOx__) == ((uint32_t)(GPIOA_BASE + 0x2400)))? (uint32_t)9 : (uint32_t)10)
#endif

GPIOPin::GPIOPin(
		GPIO_TypeDef* gpio_port,
		uint16_t pinnum,			// pin number (0..15)
		uint32_t mode,				// GPIO_mode_define
		uint32_t pull,				// GPIO_pull_define
		uint32_t speed,				// GPIO_speed_define
		uint32_t af,				// Peripheral to be connected to the selected pin (Alternative function)
		uint32_t output
		)
{
	Pin = 1 << pinnum;
	Mode = mode;
	Pull = pull;
	Speed = speed;
	Alternate = af;
	OutputType = output;
	gpio_port_ = gpio_port;
	pinnum_ = pinnum;
	pn_ = (PinName)((GET_GPIO_SOURCE(gpio_port) << 8) | ((uint32_t)pinnum & 0xFF));
}

GPIOPin::GPIOPin(
		PinName pn,
		uint32_t mode,				// GPIO_mode_define
		uint32_t pull,				// GPIO_pull_define
		uint32_t speed,				// GPIO_speed_define
		uint32_t af,				// Peripheral to be connected to the selected pin (Alternative function)
		uint32_t output
		)
		: gpio_port_((GPIO_TypeDef *) (GPIOA_BASE + 0x0400 * LL_PORTNUM(pn)))
		, pinnum_(LL_PINNUM(pn))
		, pn_(pn)
{
	Pin = 1 << pinnum_;
	Mode = mode;
	Pull = pull;
	Speed = speed;
	Alternate = af;
	OutputType = output;
}

GPIOPin::~GPIOPin()
{
}

void GPIOPin::Init() const
{
	unsigned long portnum = (((unsigned long)gpio_port_) - GPIOA_BASE) / 0x0400;
	switch (portnum) {
		case 0:	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA); break;
		case 1:	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB); break;
		case 2:	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC); break;
#ifdef GPIOD
		case 3:	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD); break;
#endif
#ifdef GPIOE
		case 4:	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE); break;
#endif
#ifdef GPIOF
		case 5:	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF); break;
#endif
#ifdef GPIOG
		case 6:	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOG); break;
#endif
#ifdef GPIOH
		case 7:	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH); break;
#endif
#ifdef GPIOI
		case 8:	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOI); break;
#endif
#ifdef GPIOJ
		case 9:	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOJ); break;
#endif
#ifdef GPIOK
		case 10: LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOK); break;
#endif
	}
	LL_GPIO_Init(gpio_port_, (LL_GPIO_InitTypeDef*)this);
}

void GPIOPin::CleanUp() const
{
}

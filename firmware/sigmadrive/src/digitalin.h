#ifndef DIGITALIN_H_
#define DIGITALIN_H_

#include <assert.h>
#include <functional>
#include <array>
#include "pinnames.h"
#include "sigmadrive.h"

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


class DigitalIn
{
public:
	enum InterruptMode {
		InterruptNone = 0,
		InterruptRising,
		InterruptFalling,
		InterruptRisingFalling,
		InterruptDefault = InterruptNone
	};

	enum PullMode {
	    PullNone  = 0,
	    PullUp    = 1,
	    PullDown  = 2,
	    PullDefault = PullNone
	};

	static const size_t max_ports = 8;
	static const size_t pins_per_ports = 16;

	/** Create a DigitalIn connected to the specified pin
	 *
	 *  @param pin DigitalIn pin to connect to
	 *  @param mode the initial mode of the pin
	 */
	DigitalIn(PinName pin, PullMode pmode = PullDefault, InterruptMode imode = InterruptDefault, uint32_t irq_priority = 0UL,
			const std::function<void(void)>& callback = [](void)->void{})
		: pin_(LL_PIN(pin))
		, GPIOx_(LL_PORT(pin))
		, callback_(callback)
	{
		clock_enable(LL_PORTNUM(pin));
		LL_GPIO_SetPinMode(GPIOx_, pin_, LL_GPIO_MODE_INPUT);
		LL_GPIO_SetPinPull(GPIOx_, pin_, pmode);
		LL_EXTI_EnableIT_0_31(pin_);
		if (imode != InterruptNone) {
			extiline_enable(LL_PORTNUM(pin), LL_PINNUM(pin), irq_priority);
		}
		if (imode == InterruptRising) {
			LL_EXTI_EnableIT_0_31((0x1U << LL_PINNUM(pin)));
			LL_EXTI_EnableRisingTrig_0_31(pin_);
		} else if (imode == InterruptFalling) {
			LL_EXTI_EnableIT_0_31((0x1U << LL_PINNUM(pin)));
			LL_EXTI_EnableFallingTrig_0_31(pin_);
		} else if (imode == InterruptRisingFalling) {
			LL_EXTI_EnableIT_0_31((0x1U << LL_PINNUM(pin)));
			LL_EXTI_EnableFallingTrig_0_31(pin_);
			LL_EXTI_EnableRisingTrig_0_31(pin_);
		}
	}


	/** Destroy a DigitalIn object
	 *
	 */
	virtual ~DigitalIn();

	/** Read the input, represented as 0 or 1 (int)
	 *
	 *  @returns
	 *    An integer representing the state of the input pin,
	 *    0 for logical 0, 1 for logical 1
	 */
	uint32_t read()
	{
		return LL_GPIO_IsInputPinSet(GPIOx_, pin_);
	}

	/** An operator shorthand for read()
	 */
	operator uint32_t()
	{
		return read();
	}

	/** Attach a member function to call when a rising edge occurs on the input
	 *
	 *  @param tptr pointer to the object to call the member function on
	 *  @param mptr pointer to the member function to be called
	 */

#define USE_BIND
	template<typename T>
	void callback(T* object, void (T::*func)(void))
	{
#if defined(USE_BIND)
		callback_ = std::bind(func, object);
#else
		callback_ = [=](void){(object->*func)();};
#endif
	}

	void callback(const std::function<void(void)>& callback)
	{
		callback_ = callback;
	}

	static void vector_handler(size_t line);
	static void vector_handlers(size_t begin, size_t size);

protected:
	void clock_enable(unsigned int gpioport)
	{
		switch (gpioport) {
		case 0:
			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
			break;
		case 1:
			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
			break;
		case 2:
			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
			break;
		case 3:
			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
			break;
		default:
			break;
		};
	}

	void extiline_enable(uint32_t portnum, uint32_t linenum, uint32_t irq_priority);

protected:
	uint32_t pin_;
	GPIO_TypeDef *GPIOx_;
	std::function<void(void)> callback_;
};


#endif /* DIGITALIN_H_ */

#ifndef _BLINKLED_H_
#define _BLINKLED_H_


#include <stm32f7xx_hal_gpio.h>

class BlinkLed
{
public:

	/**
	 * BlinkLed Constructor
	 * @param port      GPIO port for the blinking led
	 * @param pin	    GPIO pin for the blinking led
	 * @param ontime    Time the blinking led will be on (mSeconds)
	 * @param offtime   Time the blinking led will be off (mSeconds)
	 */
	BlinkLed(GPIO_TypeDef* port, uint16_t pin, uint32_t ontime, uint32_t offtime)
        : port_(port)
        , pin_(pin)
        , ontime_(ontime)
        , offtime_(offtime)
    {
    }

	/**
	 * Run one blink cycle.
	 */
	void Blink()
	{
        HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
        osDelay(ontime_);
        HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
        osDelay(offtime_);
	}

protected:
	GPIO_TypeDef* port_;
	uint16_t pin_;
	uint32_t ontime_;
	uint32_t offtime_;
};


#endif //_BLINKLED_H_

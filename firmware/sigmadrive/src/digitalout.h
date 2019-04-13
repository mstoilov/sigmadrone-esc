#ifndef DIGITALOUT_H_
#define DIGITALOUT_H_

#include "sigmadrive.h"
#include "pinnames.h"


class DigitalOut {

public:
	enum PullMode {
	    PullNone  = LL_GPIO_PULL_NO,
	    PullUp    = LL_GPIO_PULL_UP,
	    PullDown  = LL_GPIO_PULL_DOWN,
	    PullDefault = PullNone
	};

	enum SpeedMode {
		SpeedLow	= LL_GPIO_SPEED_FREQ_LOW,
	    SpeedMedium	= LL_GPIO_SPEED_FREQ_MEDIUM,
	    SpeedHigh	= LL_GPIO_SPEED_FREQ_HIGH,
	    SpeedDefault = SpeedLow
	};

	enum ActiveMode {
		ActiveHigh = 0,
		ActiveLow = 1,
		ActiveDefault = ActiveHigh
	};

	enum OutputType {
		OutputPushPull = LL_GPIO_OUTPUT_PUSHPULL,
		OutputOpenDrain = LL_GPIO_OUTPUT_OPENDRAIN,
		OutputDefault = OutputPushPull
	};

	/** Create a DigitalOut connected to the specified pin
	 *
	 *  @param pin DigitalOut pin to connect to
	 *  @param value the initial pin value
	 */
	DigitalOut(PinName pin, SpeedMode smode = SpeedDefault, OutputType output = OutputPushPull, PullMode pmode = PullDefault, ActiveMode amode = ActiveDefault, uint32_t value = 0)
		: pin_(LL_PIN(pin))
		, GPIOx_(LL_PORT(pin))
		, amode_(amode)
	{
		EnableClock(LL_PORTNUM(pin));
		LL_GPIO_SetPinMode(GPIOx_, pin_, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinPull(GPIOx_, pin_, pmode);
		LL_GPIO_SetPinSpeed(GPIOx_, pin_, smode);
		LL_GPIO_SetPinOutputType(GPIOx_, pin_, output);

		if (pin_ < 8)
			LL_GPIO_SetAFPin_0_7(GPIOx_, pin_, smode);
		else
			LL_GPIO_SetAFPin_8_15(GPIOx_, pin_, smode);
		Write(value);
	}

	DigitalOut(const DigitalOut& rhs) = default;
	~DigitalOut()
	{

	}

	/** Set the output, specified as 0 or 1 (int)
	 *
	 *  @param value An integer specifying the pin output value,
	 *      0 for logical 0, 1 (or any other non-zero value) for logical 1
	 */
	void Write(uint32_t value)
	{
		if ((value ? 1 : 0) ^ amode_)
			LL_GPIO_SetOutputPin(GPIOx_, pin_);
		else
			LL_GPIO_ResetOutputPin(GPIOx_, pin_);
	}

	void On()
	{
		Write(1);
	}

	void Off()
	{
		Write(0);
	}

	/** Toggle the current output
	 */
	void Toggle()
	{
		LL_GPIO_TogglePin(GPIOx_, pin_);
	}

	/** Return the output setting, represented as 0 or 1 (int)
	 *
	 *  @returns
	 *    an integer representing the output setting of the pin,
	 *    0 for logical 0, 1 for logical 1
	 */
	uint32_t Read()
	{
		return (LL_GPIO_IsOutputPinSet(GPIOx_, pin_) ? 1 : 0) ^ amode_;
	}

	/** A shorthand for write()
	 */

	DigitalOut& operator=(int value)
	{
		Write(value);
		return *this;
	}

	DigitalOut& operator=(DigitalOut& rhs)
	{
		Write(rhs.Read());
		return *this;
	}

	/** A shorthand for read()
	 */
	operator uint32_t()
	{
		return Read();
	}

protected:
	void EnableClock(unsigned int gpioport)
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
		case 4:
			#if defined(GPIOE)
			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
			#endif /* GPIOE */
			break;
		case 5:
			#if defined(GPIOF)
			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
			#endif /* GPIOF */
			break;
		case 6:
			#if defined(GPIOG)
			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOG);
			#endif /* GPIOG */
			break;
		case 7:
			#if defined(GPIOH)
			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
			#endif /* GPIOH */
			break;
		default:
			break;
		};
	}

protected:
	uint32_t pin_;
	GPIO_TypeDef *GPIOx_;
	ActiveMode amode_;
};

#endif /* DIGITALOUT_H_ */

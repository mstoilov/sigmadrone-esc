#include "bldc-config.h"
#include "bldc-gpio.h"

static void bldc_gpio_clock_enable(uint32_t gpioport)
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


static void bldc_gpio_config_input(uint32_t pin, uint32_t pull, uint32_t speed)
{
	bldc_gpio_clock_enable(LL_PORTNUM(pin));
	LL_GPIO_SetPinMode(LL_PORT(pin), LL_PINMASK(pin), LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(LL_PORT(pin), LL_PINMASK(pin), pull);
	LL_GPIO_SetPinSpeed(LL_PORT(pin), LL_PINMASK(pin), speed);
}


static void bldc_gpio_config_output(uint32_t pin, uint32_t pull, uint32_t speed)
{
	bldc_gpio_clock_enable(LL_PORTNUM(pin));
	LL_GPIO_SetPinMode(LL_PORT(pin), LL_PINMASK(pin), LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinPull(LL_PORT(pin), LL_PINMASK(pin), pull);
	LL_GPIO_SetPinSpeed(LL_PORT(pin), LL_PINMASK(pin), speed);
	LL_GPIO_SetPinOutputType(LL_PORT(pin), LL_PINMASK(pin), LL_GPIO_OUTPUT_PUSHPULL);
}


static void bldc_gpio_output_config()
{
	/*
	 * Init warning led
	 */
	bldc_gpio_config_output(LED_WARNING_PIN, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_MEDIUM);

	/*
	 * Init status led
	 */
	bldc_gpio_config_output(LED_STATUS_PIN, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_MEDIUM);

}

static void bldc_gpio_config_alternate(uint32_t pin, uint32_t pull, uint32_t speed, uint32_t af)
{
	bldc_gpio_clock_enable(LL_PORTNUM(pin));
	LL_GPIO_SetPinMode(LL_PORT(pin), LL_PINMASK(pin), LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinPull(LL_PORT(pin), LL_PINMASK(pin), pull);
	LL_GPIO_SetPinSpeed(LL_PORT(pin), LL_PINMASK(pin), speed);
	if (LL_PINNUM(pin) < 8)
		LL_GPIO_SetAFPin_0_7(LL_PORT(pin), LL_PINMASK(pin), af);
	else
		LL_GPIO_SetAFPin_8_15(LL_PORT(pin), LL_PINMASK(pin), af);
}

static void bldc_gpio_config_analog(uint32_t pin, uint32_t pull, uint32_t speed)
{
	bldc_gpio_clock_enable(LL_PORTNUM(pin));
	LL_GPIO_SetPinMode(LL_PORT(pin), LL_PINMASK(pin), LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinPull(LL_PORT(pin), LL_PINMASK(pin), pull);
	LL_GPIO_SetPinSpeed(LL_PORT(pin), LL_PINMASK(pin), speed);
}


static void bldc_gpio_input_config()
{
	/*
	 * Init user button
	 */
	bldc_gpio_config_input(BTN_USER_PIN, LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_LOW);


	/*
	 * Init current fault pin
	 */
	bldc_gpio_config_input(CURRENT_FAULT_PIN, LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_LOW);

	/*
	 * PWM input
	 */
	bldc_gpio_config_alternate(PWM_INPUT_PIN, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, PWM_INPUT_AF);


	/*
	 * Init ADC input
	 */
	bldc_gpio_config_analog(PIN_ADC1, LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_HIGH);
	bldc_gpio_config_analog(PIN_ADC2, LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_HIGH);
	bldc_gpio_config_analog(PIN_ADC3, LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_HIGH);
	bldc_gpio_config_analog(PIN_ADC4, LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_HIGH);
	bldc_gpio_config_analog(PIN_ADC5, LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_HIGH);

	/*
	 * Init Motor control timer channels
	 */
	bldc_gpio_config_alternate(PWM_AH, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1);
	bldc_gpio_config_alternate(PWM_BH, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1);
	bldc_gpio_config_alternate(PWM_CH, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1);

	bldc_gpio_config_alternate(PWM_AL, LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1);
	bldc_gpio_config_alternate(PWM_BL, LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1);
	bldc_gpio_config_alternate(PWM_CL, LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1);


	/*
	 * Init USART pins
	 */
	bldc_gpio_config_alternate(USART_TX, LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_7);
	bldc_gpio_config_alternate(USART_RX, LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_7);
}

void bldc_gpio_write(GPIO_TypeDef *GPIOx, uint32_t PinMask, uint32_t value)
{
	if (value) {
		LL_GPIO_SetOutputPin(GPIOx, PinMask);
	} else {
		LL_GPIO_ResetOutputPin(GPIOx, PinMask);
	}
}

void bldc_gpio_activelow_write(GPIO_TypeDef *GPIOx, uint32_t PinMask, uint32_t value, uint32_t activelow)
{
	if ((value ? 1 : 0) ^ (activelow ? 1 : 0)) {
		LL_GPIO_SetOutputPin(GPIOx, PinMask);
	} else {
		LL_GPIO_ResetOutputPin(GPIOx, PinMask);
	}

}

uint32_t bldc_gpio_read(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
	return LL_GPIO_IsInputPinSet(GPIOx, PinMask);
}

uint32_t bldc_gpio_activelow_read(GPIO_TypeDef *GPIOx, uint32_t PinMask, uint32_t activelow)
{
	return LL_GPIO_IsInputPinSet(GPIOx, PinMask) ^ (activelow ? 1 : 0);
}

void bldc_gpio_toggle(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
	LL_GPIO_TogglePin(GPIOx, PinMask);
}

void bldc_gpio_config()
{
	bldc_gpio_output_config();
	bldc_gpio_input_config();

}

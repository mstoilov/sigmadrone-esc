#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "cmsis_device.h"

#include "interruptmanager.h"
#include "digitalin.h"
#include "digitalout.h"
#include "pwmdecoder.h"
#include "pwmsine.h"
#include "usart.h"

#include <iostream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

DigitalOut led_warn(PA_5, DigitalOut::SpeedHigh, DigitalOut::OutputDefault, DigitalOut::PullDown, DigitalOut::ActiveDefault, 0);
DigitalOut led_status(PA_6, DigitalOut::SpeedHigh, DigitalOut::OutputDefault, DigitalOut::PullNone, DigitalOut::ActiveLow, 0);
DigitalIn btn_user(PA_4, DigitalIn::PullDefault, DigitalIn::InterruptFalling);

PWMDecoder pwm3(TIM3, TimeSpan::from_milliseconds(25), Frequency::from_hertz(SystemCoreClock), {
		{PB_4, LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_2}
});


PWMSine pwm1(TIM1, Frequency::from_hertz(50000), Frequency::from_hertz(SystemCoreClock), {
		{PA_8,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PA_9,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PA_10, LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PB_13,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PB_14,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PB_15,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
});

extern USART* ptrUsart1;

void pwm1_toggle()
{
	led_warn.Toggle();
	if (led_warn.Read()) {
		pwm1.Start();
	} else {
		pwm1.Stop();
	}
}

int main(int argc, char* argv[])
{
	InterruptManager& im = InterruptManager::instance();

	btn_user.Callback(pwm1_toggle);

	// At this stage the system clock should have already been configured
	// at high speed.
	printf("System clock: %lu Hz\n", SystemCoreClock);

	pwm3.Start();

	pwm1.SetDutyCycle(0.07);
	pwm1.SetRotationsPerSecond(Frequency::from_millihertz(800));

	while (1) {
		std::string tmp;
		HAL_Delay(50UL);
		led_status.Toggle();

		std::cout << "PWM: " << pwm3.GetPulseLength().seconds_float() * 1000.0 << " mSec, (Pulse/Period): " << pwm3.GetPWMPulse() << " / " << pwm3.GetPWMPeriod() << std::endl;


	}
}


extern "C" void SysTick_Handler(void)
{
#if defined(USE_HAL_DRIVER)
	HAL_IncTick();
#endif
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

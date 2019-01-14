#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <algorithm>
#include "diag/Trace.h"
#include "cmsis_device.h"

#include "interruptmanager.h"
#include "digitalin.h"
#include "digitalout.h"
#include "pwmdecoder.h"
#include "quadraturedecoder.h"
#include "pwmsine.h"
#include "pwm6step.h"
#include "trigger.h"
#include "adc.h"
#include "usart.h"

#include <iostream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

#define CURRENT_FB_A	LL_ADC_CHANNEL_13
#define CURRENT_FB_B	LL_ADC_CHANNEL_14
#define CURRENT_FB_C	LL_ADC_CHANNEL_15
#define CURRENT_FB_R	LL_ADC_CHANNEL_4

#define BEMF_FB_A	LL_ADC_CHANNEL_10
#define BEMF_FB_B	LL_ADC_CHANNEL_11
#define BEMF_FB_C	LL_ADC_CHANNEL_12


DigitalOut led_warn(PA_5, DigitalOut::SpeedHigh, DigitalOut::OutputDefault, DigitalOut::PullDown, DigitalOut::ActiveDefault, 0);
DigitalOut led_status(PA_6, DigitalOut::SpeedHigh, DigitalOut::OutputDefault, DigitalOut::PullNone, DigitalOut::ActiveLow, 0);
DigitalIn btn_user(PA_4, DigitalIn::PullDefault, DigitalIn::InterruptFalling);

PWMDecoder pwm3(TIM3, TimeSpan::from_milliseconds(25), Frequency::from_hertz(SystemCoreClock), 0, {
		{PB_4, LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_2}
});


QuadratureDecoder pwm4(TIM4,  65535, 0, {
		{PB_6, LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_2},
		{PB_7, LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_2}
});


#define USE_6STEP
#ifdef USE_6STEP
PWM6Step pwm1(TIM1, Frequency::from_hertz(50000), Frequency::from_hertz(SystemCoreClock), Timer::TrigUpdate, Timer::PWM1, Timer::High, Timer::Low, 30, 0, {
		{PA_8,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PA_9,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PA_10, LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PB_13,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PB_14,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PB_15,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
});

Adc adc(ADC1, LL_ADC_RESOLUTION_12B, LL_ADC_SAMPLINGTIME_28CYCLES, LL_ADC_INJ_TRIG_EXT_TIM2_CH1, 0, {
		{PC_0,  LL_GPIO_MODE_ANALOG, LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_AF_0},
		{PC_1,  LL_GPIO_MODE_ANALOG, LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_AF_0},
		{PC_2,  LL_GPIO_MODE_ANALOG, LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_AF_0},},
		{
				BEMF_FB_A, BEMF_FB_B, BEMF_FB_C,
		});

Trigger adc_trigger(TIM2, TimeSpan::from_nanoseconds(750), Frequency::from_hertz(SystemCoreClock));


#else
PWMSine pwm1(TIM1, Frequency::from_hertz(50000), Frequency::from_hertz(SystemCoreClock), Timer::PWM1, 0, {
		{PA_8,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PA_9,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PA_10, LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PB_13,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PB_14,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PB_15,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
});

Adc adc(ADC1, LL_ADC_RESOLUTION_12B, LL_ADC_SAMPLINGTIME_3CYCLES, LL_ADC_INJ_TRIG_EXT_TIM2_CH1, {
		{PC_3,  LL_GPIO_MODE_ANALOG, LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_AF_0},
		{PC_4,  LL_GPIO_MODE_ANALOG, LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_AF_0},
		{PC_5,  LL_GPIO_MODE_ANALOG, LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_AF_0},
		{PA_3,  LL_GPIO_MODE_ANALOG, LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_AF_0},},
		{
				CURRENT_FB_A, CURRENT_FB_B, CURRENT_FB_C, CURRENT_FB_R,
		});

Trigger adc_trigger(TIM2, TimeSpan::from_nanoseconds(17000), Frequency::from_hertz(SystemCoreClock));

#endif



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

void CallbackJeos(int32_t* data, size_t size)
{
	static size_t counter = 0;

	if ((counter++ % 64) == 0) {
		std::for_each(data, data + size, [](int32_t& a){a = 3300/2 - a;});
		printf("JeosCallback: %7ld %7ld %7ld (%7ld)\n", data[0], data[1], data[2], data[0] + data[1] + data[2]);
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
	pwm4.Start();

	adc.CallbackJEOS(CallbackJeos);
	adc.Start();


#ifdef USE_6STEP
	pwm1.SetDutyCycle(0.30);
	adc.CallbackJEOS(&pwm1, &PWM6Step::CallbackJEOS);
#else
	pwm1.SetRotationsPerSecond(Frequency::from_millihertz(1800));
	pwm1.SetDutyCycle(0.15);
#endif

	PWM6Step::LogEntry log;

	while (1) {
		std::string tmp;
		HAL_Delay(50UL);
		led_status.Toggle();

#ifdef USE_6STEP
		if (pwm1.log_entry_.serial_ != log.serial_) {
			__disable_irq();
			log = pwm1.log_entry_;
			__enable_irq();
			printf("%1lu: slp: %7ld, incpt: %5ld, ibemf: %5ld, zero_c: %3ld (%3ld) %2d->[%6ld %6ld %6ld ] %2d->[%6ld %6ld %6ld]\n",
					log.state_,
					log.bemf_mslope_,
					log.bemf_intercept_,
					log.integral_bemf_,
					log.zero_crossing_,
					log.counter_,
					pwm1.adc_data_counter1,
					log.adc_data1_[0],
					log.adc_data1_[1],
					log.adc_data1_[2],
					pwm1.adc_data_counter2,
					log.adc_data2_[0],
					log.adc_data2_[1],
					log.adc_data2_[2]
			);

		}
#endif

//		std::cout << "Counter: " << pwm1.GetCounterValue() << std::endl;
//		std::cout << "PWM: " << pwm3.GetPulseLength().seconds_float() * 1000.0 << " mSec, (Pulse/Period): " << pwm3.GetPWMPulse() << " / " << pwm3.GetPWMPeriod() << std::endl;

//		std::cout << "Counter: " << pwm4.GetCounterValue() << std::endl;

//		std::cout << adc.injdata_[0] << ", "  << adc.injdata_[1] << ", "  << adc.injdata_[2] << ", " << std::endl;

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

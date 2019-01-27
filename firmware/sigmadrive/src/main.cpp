#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <algorithm>
#include "diag/Trace.h"
#include "cmsis_device.h"

#include "cortexm/ExceptionHandlers.h"
#include "arm/semihosting.h"


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
DigitalIn encoder_z(PB_5, DigitalIn::PullDown, DigitalIn::InterruptRising);

PWMDecoder pwm3(TIM3, TimeSpan::from_milliseconds(25), Frequency::from_hertz(SystemCoreClock), 0, {
		{PB_4, LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_2}
});


QuadratureDecoder pwm4(TIM4,  2048*4, 0, {
		{PB_6, LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_2},
		{PB_7, LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_2}
});

QuadratureDecoder *p_encoder = &pwm4;

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
PWMSine pwm1(TIM1, Frequency::from_hertz(50000), Frequency::from_hertz(SystemCoreClock), Timer::PWM2, 0, {
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

void emergency_stop()
{
	pwm1.DisableOutputs();
	pwm1.DisableCounter();
}


extern USART* ptrUsart1;

void pwm1_toggle()
{
	pwm1.Toggle();
}

void CallbackJeos(int32_t* data, size_t size)
{
	static size_t counter = 0;

	if ((counter++ % 64) == 0) {
		std::for_each(data, data + size, [](int32_t& a){a = 3300/2 - a;});
		printf("JeosCallback: %7ld %7ld %7ld (%7ld)\n", data[0], data[1], data[2], data[0] + data[1] + data[2]);
	}
}

void CallbackPWMCC(uint32_t pulse, uint32_t period)
{
	float throttle = pwm3.GetPulseLength().seconds_float() * 1000.0 - 0.7;
	pwm1.SetThrottle(throttle);
	if (!pwm1.IsEnabledCounter() && throttle > PWM6Step::MIN_THROTTLE)
		pwm1.Start();
	if (pwm1.IsEnabledCounter() && throttle < PWM6Step::MIN_THROTTLE)
		pwm1.Stop();
}

int main(int argc, char* argv[])
{
	InterruptManager& im = InterruptManager::instance();

	btn_user.Callback(pwm1_toggle);

	// At this stage the system clock should have already been configured
	// at high speed.
	printf("System clock: %lu Hz\n", SystemCoreClock);

	pwm3.Callback_PWMCC(CallbackPWMCC);

	encoder_z.Callback([&](){ pwm4.ResetPosition(0); });


	pwm3.Start();
	pwm4.Start();

	adc.CallbackJEOS(CallbackJeos);
	adc.Start();


#ifdef USE_6STEP
	pwm1.SetThrottle(0.35);
	adc.CallbackJEOS(&pwm1, &PWM6Step::CallbackJEOS);
#else
	pwm1.SetRotationsPerSecond(Frequency::from_millihertz(1800));
	pwm1.SetThrottle(0.15);
#endif

	PWM6Step::LogEntry log;

	while (1) {
		std::string tmp;
//		HAL_Delay(50UL);
		led_status.Toggle();
		led_warn.Write(pwm1.IsEnabledCounter());


#ifdef USE_6STEP
		if (pwm1.log_entry_.serial_ != log.serial_) {
			__disable_irq();
			log = pwm1.log_entry_;
			__enable_irq();

			uint32_t hz = (uint32_t) (pwm1.GetSwitchingFrequency() / log.last_counter_ / PWM6Step::MECHANICAL_DEGREES_RATIO / PWM6Step::SINE_STATES).hertz();

			printf("%1lu: Speed: %5lu, slp: %7ld, incpt: %5ld, ibemf: %5ld, zero_c: %3ld (%3ld), %2d->[%6ld %6ld %6ld ] %2d->[%6ld %6ld %6ld]\n",
					log.state_,
					hz,
					log.bemf_mslope_,
					log.bemf_intercept_,
					log.integral_bemf_,
					log.zero_crossing_,
					log.last_counter_,
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

		pwm1.SetThrottle(pwm3.GetPulseLength().seconds_float() * 1000.0 - 0.7);



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

extern "C" void
HardFault_Handler_C (ExceptionStackFrame* frame __attribute__((unused)),
                     uint32_t lr __attribute__((unused)))
{
	emergency_stop();



#if defined(TRACE)
  uint32_t mmfar = SCB->MMFAR; // MemManage Fault Address
  uint32_t bfar = SCB->BFAR; // Bus Fault Address
  uint32_t cfsr = SCB->CFSR; // Configurable Fault Status Registers
#endif

#if defined(OS_USE_SEMIHOSTING) || defined(OS_USE_TRACE_SEMIHOSTING_STDOUT) || defined(OS_USE_TRACE_SEMIHOSTING_DEBUG)

  // If the BKPT instruction is executed with C_DEBUGEN == 0 and MON_EN == 0,
  // it will cause the processor to enter a HardFault exception, with DEBUGEVT
  // in the Hard Fault Status register (HFSR) set to 1, and BKPT in the
  // Debug Fault Status register (DFSR) also set to 1.

  if (((SCB->DFSR & SCB_DFSR_BKPT_Msk) != 0)
      && ((SCB->HFSR & SCB_HFSR_DEBUGEVT_Msk) != 0))
    {
      if (isSemihosting (frame, 0xBE00 + (AngelSWI & 0xFF)))
        {
          // Clear the exception cause in exception status.
          SCB->HFSR = SCB_HFSR_DEBUGEVT_Msk;

          // Continue after the BKPT
          return;
        }
    }

#endif

#if defined(TRACE)
  trace_printf ("[HardFault]\n");
  dumpExceptionStack (frame, cfsr, mmfar, bfar, lr);
#endif // defined(TRACE)

#if defined(DEBUG)
  __DEBUG_BKPT();
#endif
  while (1)
    {
    }
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

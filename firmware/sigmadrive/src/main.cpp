#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include "diag/Trace.h"
#include "cmsis_device.h"

#include "cortexm/ExceptionHandlers.h"
#include "arm/semihosting.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

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
#include "mathtest.h"


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

#undef USE_6STEP
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


//Trigger adc_trigger(TIM3, TimeSpan::from_nanoseconds(750), Frequency::from_hertz(SystemCoreClock),
//		Timer::SlaveTrigger, Timer::TriggerInternal0, Timer::TrigOC2REF, Timer::CH2, Timer::PWM1);


#else
PWMSine pwm1(TIM1, Frequency::from_hertz(50000), Frequency::from_hertz(SystemCoreClock), Timer::PWM1, 0, {
		{PA_8,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PA_9,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PA_10, LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_DOWN, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PB_13,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PB_14,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
		{PB_15,  LL_GPIO_MODE_ALTERNATE, LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_AF_1},
});

Adc adc(ADC1, LL_ADC_RESOLUTION_12B, LL_ADC_SAMPLINGTIME_3CYCLES, LL_ADC_INJ_TRIG_EXT_TIM2_CH1, 0, {
		{PC_3,  LL_GPIO_MODE_ANALOG, LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_AF_0},
		{PC_4,  LL_GPIO_MODE_ANALOG, LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_AF_0},
		{PC_5,  LL_GPIO_MODE_ANALOG, LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_AF_0},
		{PA_3,  LL_GPIO_MODE_ANALOG, LL_GPIO_PULL_NO, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_AF_0},},
		{
				CURRENT_FB_A, CURRENT_FB_B, CURRENT_FB_C, CURRENT_FB_R,
		});

Trigger adc_trigger(TIM2, TimeSpan::from_nanoseconds(17000), Frequency::from_hertz(SystemCoreClock));

#endif

volatile uint64_t jiffies = 0;

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

void CallbackPWMCC(uint32_t pulse, uint32_t period)
{
#ifdef USE_6STEP
	float throttle = pwm3.GetPulseLength().seconds_float() * 1000.0 - 0.7;
	pwm1.SetThrottle(throttle);
	if (!pwm1.IsEnabledCounter() && throttle > PWM6Step::MIN_THROTTLE)
		pwm1.Start();
	if (pwm1.IsEnabledCounter() && throttle < PWM6Step::MIN_THROTTLE)
		pwm1.Stop();
#endif
}

void RunFloatingPointTest()
{
	int64_t jend = 0, jbegin = jiffies;
	int iterations = 4000000;

	printf("----- Starting FPU Test\n");
	printf("Iterations : %d\n", iterations);

	for (int i = 0; i < iterations; i++) {
		mul_float(2.0, 3.0);
	}
	jend = jiffies;

	printf("mul_float  : %5ld\n", (int32_t)(jend - jbegin));

	jbegin = jiffies;
	for (int i = 0; i < iterations; i++) {
		div_float(2.0, 3.0);
	}
	jend = jiffies;
	printf("div_float  : %5ld\n", (int32_t)(jend - jbegin));

	std::complex<float> a = 0.25 + 2.0i, b = 0.5 + 1.25i;
	jbegin = jiffies;
	for (int i = 0; i < iterations; i++) {
		dot(a, b);
	}
	jend = jiffies;
	printf("dot        : %5ld\n", (int32_t)(jend - jbegin));
	printf("----- Done.\n");
}


void main_task(void *pvParameters)
{
	uint32_t old_encoder_idx = 0, new_encoder_idx = 0;
	uint32_t old_decoder_value = 0, new_decoder_value = 0;

	btn_user.Callback(pwm1_toggle);

	// At this stage the system clock should have already been configured
	// at high speed.
	printf("System clock: %lu Hz\n", SystemCoreClock);

	pwm3.Callback_PWMCC(CallbackPWMCC);
	pwm3.Start();

	encoder_z.Callback([&](){ p_encoder->CallbackIndex(); });
	pwm4.Start();

	adc.Start();


#ifdef USE_6STEP
	pwm1.SetThrottle(0.35);
	adc.CallbackJEOS(&pwm1, &PWM6Step::CallbackJEOS);
#else
	pwm1.SetElectricalRotationsPerSecond(Frequency::from_millihertz(500 * PWMSine::M2E_RATIO));
	pwm1.SetThrottle(0.04);
#endif

	PWM6Step::LogEntry log;

	while (1) {
		std::string tmp;
		HAL_Delay(10UL);
		led_status.Toggle();
		led_warn.Write(pwm1.IsEnabledCounter());


#ifdef USE_6STEP
		if (pwm1.log_entry_.serial_ != log.serial_) {
			__disable_irq();
			log = pwm1.log_entry_;
			__enable_irq();

			uint32_t hz = (uint32_t) (pwm1.GetSwitchingFrequency() / log.last_counter_ / PWM6Step::M2E_RATIO / PWM6Step::SINE_STATES).hertz();

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

//		pwm1.SetThrottle(pwm3.GetPulseLength().seconds_float() * 1000.0 - 0.7);



#endif

//		std::cout << "Counter: " << pwm1.GetCounterValue() << std::endl;
//		std::cout << "PWM: " << pwm3.GetPulseLength().seconds_float() * 1000.0 << " mSec, (Pulse/Period): " << pwm3.GetPWMPulse() << " / " << pwm3.GetPWMPeriod() << std::endl;

#if 0
		new_decoder_value = (pwm4.GetPosition() >> 2);
		if (new_decoder_value != old_decoder_value) {
			old_decoder_value = new_decoder_value;
			std::cout << jiffies << " : " << old_decoder_value;

			if (new_encoder_idx != old_encoder_idx) {
				old_encoder_idx = new_encoder_idx;
				std::cout << " (Encoder Z)";
			}

			std::cout << std::endl;
		}
#endif

	}

}

int main(int argc, char* argv[])
{
	InterruptManager& im = InterruptManager::instance();

	TaskHandle_t main_task_handle = 0;

	vTaskSuspendAll();

	/* Create tasks */
	xTaskCreate(
			main_task, /* Function pointer */
			"main_task", /* Task name - for debugging only*/
			4 * configMINIMAL_STACK_SIZE, /* Stack depth in words */
			(void*) NULL, /* Pointer to tasks arguments (parameter) */
			tskIDLE_PRIORITY + 3UL, /* Task priority*/
			&main_task_handle /* Task handle */
	);

	im.VectorHandler(SVCall_IRQn, vPortSVCHandler);
	im.VectorHandler(PendSV_IRQn, xPortPendSVHandler);
	im.VectorHandler(SysTick_IRQn, xPortSysTickHandler);

	xTaskResumeAll();

	vTaskStartScheduler();
	vTaskSuspendAll();

#if defined(DEBUG)
  __DEBUG_BKPT();
#endif
	while(1) {
	}
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

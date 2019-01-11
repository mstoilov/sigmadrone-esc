#include <iostream>
#include "pwmsine.h"
#include "adc.h"

#define UH_PIN		PA_8
#define UL_PIN		PA_7

#define VH_PIN		PA_9
#define VL_PIN		PB_0

#define WH_PIN		PA_10
#define WL_PIN		PB_1

extern Adc *p_adc;

PWMSine::PWMSine(TIM_TypeDef *TIMx, const Frequency& switching_freq, const Frequency& system_clock, OCMode pwm_mode, const std::vector<GPIOPin>& pins)
	: Timer(TIMx, (switching_freq + switching_freq / 8).period(), system_clock, pins)
	, switching_freq_(switching_freq)
	, state_(0UL)
	, pwm_mode_(pwm_mode)
{
	SetAutoReloadPeriod(switching_freq_.period());

	SetDeadTime(30);

//	CCEnablePreload();
	EnableARRPreload();

	GenerateEvent(EventUpdate);

	SetTriggerOutput(TrigUpdate);

}

PWMSine::~PWMSine()
{
	Stop();
}

void PWMSine::Start()
{
	SetOCMode(CH1, pwm_mode_);
	SetOCMode(CH2, pwm_mode_);
	SetOCMode(CH3, pwm_mode_);

	SetOCPolarity(CH1, High);
	SetOCPolarity(CH2, High);
	SetOCPolarity(CH3, High);
	SetOCPolarity(CH1N, Low);
	SetOCPolarity(CH2N, Low);
	SetOCPolarity(CH3N, Low);

	EnableChannel(CH1);
	EnableChannel(CH2);
	EnableChannel(CH3);
	EnableChannel(CH1N);
	EnableChannel(CH2N);
	EnableChannel(CH3N);

	EnableInterrupt(InterruptUpdate);
	EnableOutputs();
	base::Start();
}

void PWMSine::Stop()
{
	DisableOutputs();

	SetOCMode(CH1, ForcedInactive);
	SetOCMode(CH2, ForcedInactive);
	SetOCMode(CH3, ForcedInactive);

	SetOCPeriod(CH1, 0);
	SetOCPeriod(CH2, 0);
	SetOCPeriod(CH3, 0);

	DisableInterrupt(InterruptUpdate);
	base::Stop();
	GenerateEvent(base::EventUpdate);

	DisableChannel(CH1);
	DisableChannel(CH1N);
	DisableChannel(CH2);
	DisableChannel(CH2N);
	DisableChannel(CH3);
	DisableChannel(CH3N);

}

void PWMSine::Toggle()
{
	if (IsEnabledCounter())
		Stop();
	else
		Start();
}

void PWMSine::SetDutyPeriod(const TimeSpan& period)
{
	duty_ = period;
}

float PWMSine::GetDutyCycle()
{
	return 100.0 * duty_.nanoseconds()/switching_freq_.period().nanoseconds() / 100.0;
}

void PWMSine::SetDutyCycle(float percent)
{
	duty_ = TimeSpan::from_nanoseconds(switching_freq_.period().nanoseconds() * percent);

	for (unsigned int i = 0; i < SINE_STATES; i++) {
		float sine = (1.0 + sin(i * 2.0 * 3.1415 / SINE_STATES)) / 2.0 * 1000.0;
		TimeSpan t = duty_ * sine / 1000;
		sine_duty_[i] = t;
	}
}

Frequency PWMSine::GetSwitchingFrequency()
{
	return switching_freq_;
}

void PWMSine::SetRotationsPerSecond(const Frequency& f)
{
//	uint64_t switching_f = switching_freq_.hertz();
//	uint64_t rotation_f = f.hertz();
//
//	if (rotation_f) {
//		uint32_t repetions = switching_f / rotation_f / SINE_STATES;
//		SetRepetionCounterValue(repetions);
//	}

	uint64_t swFreq = switching_freq_.millihertz();
	uint64_t rotFreq = f.millihertz();
	uint64_t repCount = swFreq/rotFreq/SINE_STATES;

	if (f.millihertz() != 0)
		SetRepetionCounterValue(repCount); //switching_freq_.millihertz()/f.millihertz()/SINE_STATES);
	else
		SetRepetionCounterValue(switching_freq_.hertz()/SINE_STATES);
}


void PWMSine::SineDriving()
{
	state_ = (state_ + 1) % SINE_STATES;

#if 1
	SetOCPeriod(CH1, sine_duty_[(state_ + 0 * SINE_STATES / 3) % SINE_STATES ]);
	SetOCPeriod(CH2, sine_duty_[(state_ + 1 * SINE_STATES / 3) % SINE_STATES ]);
	SetOCPeriod(CH3, sine_duty_[(state_ + 2 * SINE_STATES / 3) % SINE_STATES ]);
#else
	SetOCPeriod(CH1, duty_ / 2);
	SetOCPeriod(CH2, duty_ );
	SetOCPeriod(CH3, duty_ );
#endif
}

void PWMSine::HandleUpdate()
{
	SineDriving();

	if ((++counter_ % 64) == 0) {
		printf("%7ld %7ld %7ld (%7ld)\n", p_adc->injdata_[0], p_adc->injdata_[1], - (p_adc->injdata_[0] + p_adc->injdata_[1]), p_adc->injdata_[2]);
#if 0

		printf("<%5lu> SWFREQ: %lu, OUTPUT PSC: %lu, OUTPUT_RELOAD: %lu, Repeat: %lu, Counter: %5lu, "
				"OC1: %5lu, OC2: %5lu, OC3: %5lu, OC4: %5lu\n",
				counter_,
				(unsigned long)GetSwitchingFrequency().hertz(),
				GetPrescaler(),
				GetAutoReloadValue(),
				GetRepetitionCounterValue(),
				GetCounterValue(),
				GetOCValue(Timer::CH1),
				GetOCValue(Timer::CH2),
				GetOCValue(Timer::CH3),
				GetOCValue(Timer::CH4));
#endif

	}
}

#include "pwmsine.h"


#define UH_PIN		PA_8
#define UL_PIN		PA_7

#define VH_PIN		PA_9
#define VL_PIN		PB_0

#define WH_PIN		PA_10
#define WL_PIN		PB_1


PWMSine::PWMSine(TIM_TypeDef *TIMx, const Frequency& switching_freq, const Frequency& system_clock, const std::vector<GPIOPin>& pins)
	: Timer(TIMx, (switching_freq + switching_freq / 8).period(), system_clock, pins)
	, switching_freq_(switching_freq)
	, state_(0UL)
{
	SetAutoReloadPeriod(switching_freq_.period());

	SetOCMode(CH1, PWM1);
	SetOCMode(CH2, PWM1);
	SetOCMode(CH3, PWM1);

	SetOCPolarity(CH1, High);
	SetOCPolarity(CH2, High);
	SetOCPolarity(CH3, High);
	SetOCPolarity(CH1N, Low);
	SetOCPolarity(CH2N, Low);
	SetOCPolarity(CH3N, Low);


	SetDeadTime(30);

	EnableChannel(CH1);
	EnableChannel(CH1N);
	EnableChannel(CH2);
	EnableChannel(CH2N);
	EnableChannel(CH3);
	EnableChannel(CH3N);

	EnableOutputs();
	GenerateEvent(EventUpdate);

}

PWMSine::~PWMSine()
{
	Stop();
}

void PWMSine::Start()
{
	SetOCMode(CH1, PWM1);
	SetOCMode(CH2, PWM1);
	SetOCMode(CH3, PWM1);
	EnableChannel(CH1);
	EnableChannel(CH1N);
	EnableChannel(CH2);
	EnableChannel(CH2N);
	EnableChannel(CH3);
	EnableChannel(CH3N);

	EnableInterrupt(InterruptUpdate);
	base::Start();
}

void PWMSine::Stop()
{
	DisableInterrupt(InterruptUpdate);
	base::Stop();
	GenerateEvent(base::EventUpdate);

	DisableChannel(CH1);
	DisableChannel(CH1N);
	DisableChannel(CH2);
	DisableChannel(CH2N);
	DisableChannel(CH3);
	DisableChannel(CH3N);

	SetOCPeriod(CH1, 0);
	SetOCPeriod(CH2, 0);
	SetOCPeriod(CH3, 0);

	SetOCMode(CH1, ForcedInactive);
	SetOCMode(CH2, ForcedInactive);
	SetOCMode(CH3, ForcedInactive);
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
	SetOCPeriod(CH1, sine_duty_[(state_ + 0 * SINE_STATES / 3) % SINE_STATES ]);
	SetOCPeriod(CH2, sine_duty_[(state_ + 1 * SINE_STATES / 3) % SINE_STATES ]);
	SetOCPeriod(CH3, sine_duty_[(state_ + 2 * SINE_STATES / 3) % SINE_STATES ]);
}

void PWMSine::HandleUpdate()
{
	SineDriving();

	if ((++counter_ % 32) == 0) {
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
	}
}

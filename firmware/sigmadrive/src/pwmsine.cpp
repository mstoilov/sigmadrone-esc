#include <iostream>
#include <complex>
#include <math.h>
#include "pwmsine.h"
#include "adc.h"
#include "quadraturedecoder.h"

#define UH_PIN		PA_8
#define UL_PIN		PA_7

#define VH_PIN		PA_9
#define VL_PIN		PB_0

#define WH_PIN		PA_10
#define WL_PIN		PB_1

extern Adc *p_adc;
extern QuadratureDecoder *p_encoder;

PWMSine::PWMSine(TIM_TypeDef *TIMx, const Frequency& switching_freq, const Frequency& system_clock, OCMode pwm_mode, uint32_t irq_priority, const std::vector<GPIOPin>& pins)
	: Timer(TIMx, (switching_freq + switching_freq / 8).period(), system_clock, irq_priority, pins)
	, switching_freq_(switching_freq)
	, pwm_mode_(pwm_mode)
{
	SetAutoReloadPeriod(switching_freq_.period());

	SetDeadTime(30);

//	CCEnablePreload();
	EnableARRPreload();

	GenerateEvent(EventUpdate);

	SetTriggerOutput(TrigUpdate);
	p1 = std::polar<float>(1.0f, 0.0f);
	p2 = std::polar<float>(1.0f, M_PI * 2.0f / 3.0f);
	p3 = std::polar<float>(1.0f, M_PI * 4.0f / 3.0f);
	v = std::polar<float>(1.0f, 0.0f);
	r = std::polar<float>(1.0f, 0.0f);
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

	update_counter_ = 0;
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

float PWMSine::GetDutyCycle()
{
	return 100.0 * duty_.nanoseconds()/switching_freq_.period().nanoseconds() / 100.0;
}

void PWMSine::SetThrottle(float percent)
{
	percent = std::max(std::min(percent, (float)MAX_THROTTLE), (float)MIN_THROTTLE);
	duty_ = TimeSpan::from_nanoseconds(switching_freq_.period().nanoseconds() * percent);
}

Frequency PWMSine::GetSwitchingFrequency()
{
	return switching_freq_;
}

void PWMSine::SetElectricalRotationsPerSecond(const Frequency& f)
{
	uint64_t swFreq = switching_freq_.millihertz();
	uint64_t rotFreq = f.millihertz();

	SINE_STEPS = swFreq/rotFreq/5;
	uint64_t repCount = swFreq/rotFreq/SINE_STEPS;
	SetRepetionCounterValue(repCount);

	r = std::polar<float>(1.0f, 2.0f * M_PI / SINE_STEPS);
}


void PWMSine::SineDriving()
{

#if 1
	SetOCPeriod(CH1, duty_ * ((v.real()*p1.real() + v.imag()*p1.imag()) + 1.0f) / 2.0f);
	SetOCPeriod(CH2, duty_ * ((v.real()*p2.real() + v.imag()*p2.imag()) + 1.0f) / 2.0f);
	SetOCPeriod(CH3, duty_ * ((v.real()*p3.real() + v.imag()*p3.imag()) + 1.0f) / 2.0f);


#else
	SetOCPeriod(CH1, duty_ / 2);
	SetOCPeriod(CH2, duty_ );
	SetOCPeriod(CH3, duty_ );
#endif
}


void PWMSine::HandleUpdate()
{
	float enc_theta = 2 * M_PI * (p_encoder->GetPosition() % (2048 / M2E_RATIO)) / (2048 / M2E_RATIO) ;

	if (update_counter_ > 2 * M2E_RATIO * SINE_STEPS) {
//		Stop();

		v = std::polar<float>(1.0f, enc_theta) * std::polar<float>(1.0f, M_PI/2);
	} else {
		if (update_counter_ < 1 * M2E_RATIO * SINE_STEPS) {
			v = std::polar<float>(1.0f, 0.0f);
			p_encoder->SetIndexOffset(-1);
			p_encoder->ResetPosition(0);
		} else {
			v = v * r;
		}
		update_counter_++;
	}

	SineDriving();

	if ((++counter_ % 128) == 0) {

#if 1
		float theta = std::arg(v);
		if (theta < 0.0f)
			theta += 2.0f * M_PI;

//		if (theta < enc_theta)
//			theta += 2.0f * M_PI;

		printf("%6ld : %7.3f -> %7.3f ( %7.3f )   %6ld\n", update_counter_, theta, enc_theta, theta - enc_theta, p_encoder->GetIndexOffset());

#endif

	}
}

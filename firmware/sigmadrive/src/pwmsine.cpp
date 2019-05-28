#include <iostream>
#include <complex>
#include <algorithm>
#include <math.h>
#include "pwmsine.h"
#include "adc.h"
#include "quadraturedecoder.h"
#include "minas_a4_abs_encoder.h"

#define UH_PIN		PA_8
#define UL_PIN		PA_7

#define VH_PIN		PA_9
#define VL_PIN		PB_0

#define WH_PIN		PA_10
#define WL_PIN		PB_1

extern QuadratureDecoder *p_encoder;
#ifdef PANASONIC_MOTOR
extern MinasA4AbsEncoder *ma4_abs_encoder_ptr;
#endif

PWMSine::PWMSine(const std::vector<GPIOPin>& pins,
		TIM_TypeDef *TIMx,
		const Frequency& switching_freq,
		const Frequency& system_clock,
		TriggerOutput trigger_output,
		OCMode pwm_mode,
		OCPolarity polarity,
		OCPolarity npolarity,
		uint32_t deadtime,
		uint32_t irq_priority)
	: Timer(pins, TIMx, (switching_freq + switching_freq / 8).period(), system_clock, irq_priority)
	, switching_freq_(switching_freq)
	, pwm_mode_(pwm_mode)
	, polarity_(polarity)
	, npolarity_(npolarity)
	, opamp_bias_(CURRENT_SAMPLES, 0)
{
	SetAutoReloadPeriod(switching_freq_.period());

	SetDeadTime(deadtime);

	EnableARRPreload();

	GenerateEvent(EventUpdate);

	/*
	 * ADC trigger
	 */
	SetTriggerOutput(trigger_output);


	p1 = std::polar<float>(1.0f, 0.0f);
	p2 = std::polar<float>(1.0f, M_PI * 2.0f / 3.0f);
	p3 = std::polar<float>(1.0f, M_PI * 4.0f / 3.0f);
	v = std::polar<float>(1.0f, 0.0f);
	r = std::polar<float>(1.0f, 0.0f);

	/*
	 * Initialize v and the encoder.
	 * Wait for the time of one full 360 electrical degree cycle.
	 */
	run_stack_.push_back([&]()->bool {
		v = std::polar<float>(1.0f, 0.0f);
#ifndef PANASONIC_MOTOR
		p_encoder->InvalidateIndexOffset();
		p_encoder->ResetPosition(0);
#endif
		if (update_counter_++ >= 1 * SINE_STEPS) {
			update_counter_ = 0;
			return true;
		}
		return false;
	});

	run_stack_.push_back([&]()->bool {
		v = v * r;
		if (update_counter_++ >= 1 * M2E_RATIO * SINE_STEPS) {
			return true;
		}
		return false;
	});

	run_stack_.push_back([&]()->bool {
		update_counter_++;
		v = std::polar<float>(1.0f, GetEncoderTheta()) * std::complex<float>(0.0f, 1.0f);
		return false;
	});


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

	EnableOCPreload(CH1);
	EnableOCPreload(CH2);
	EnableOCPreload(CH3);

	SetOCPolarity(CH1, polarity_);
	SetOCPolarity(CH2, polarity_);
	SetOCPolarity(CH3, polarity_);
	SetOCPolarity(CH1N, npolarity_);
	SetOCPolarity(CH2N, npolarity_);
	SetOCPolarity(CH3N, npolarity_);

	EnableChannel(CH1);
	EnableChannel(CH2);
	EnableChannel(CH3);
	EnableChannel(CH1N);
	EnableChannel(CH2N);
	EnableChannel(CH3N);

	update_counter_ = 0;
	run_index_ = 0;
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

void PWMSine::SetOCValue(Channel ch, uint32_t value)
{
	if (pwm_mode_ == PWM2) {
		uint32_t arr = GetAutoReloadValue();
		value = arr - value;
	}
	base::SetOCValue(ch, value);

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

float PWMSine::GetEncoderTheta()
{
#ifdef PANASONIC_MOTOR
		float enc_theta = 2 * M_PI * (ma4_abs_encoder_ptr->get_counter() % (MA4_ABS_ENCODER_RESOLUTION / M2E_RATIO)) / (MA4_ABS_ENCODER_RESOLUTION / M2E_RATIO);
#else
		float enc_theta = 2 * M_PI * (p_encoder->GetPosition() % (2048 / M2E_RATIO)) / (2048 / M2E_RATIO);
#endif
		return enc_theta;
}

void PWMSine::SetThrottle(float percent)
{
	percent = std::max(std::min(percent, (float)MAX_THROTTLE), (float)MIN_THROTTLE);

	duty_ = TimeSpan::from_nanoseconds(switching_freq_.period().nanoseconds() * percent);
	duty_oc_ = __LL_TIM_CALC_ARR(system_clock_.hertz(), GetPrescaler(), duty_.to_frequency().hertz());
}

Frequency PWMSine::GetSwitchingFrequency()
{
	return switching_freq_;
}

void PWMSine::SetElectricalRotationsPerSecond(const Frequency& f)
{
	uint64_t swFreq = switching_freq_.millihertz();
	uint64_t rotFreq = f.millihertz();
	uint32_t repCount = 0;

	SetRepetionCounterValue(repCount);
	SINE_STEPS = swFreq/rotFreq/(repCount + 1);

	/*
	 * Rotation Delta
	 */
	r = std::polar<float>(1.0f, 2.0f * M_PI / SINE_STEPS);
}

void PWMSine::SetOpAmpBias(const std::vector<int32_t> bias)
{
	assert(bias.size() >= CURRENT_SAMPLES); opamp_bias_ = bias;
}

void PWMSine::HandleCurrentJEOS(int32_t *injdata, size_t idx, size_t size)
{
	assert(idx + size <= CURRENT_SAMPLES);

	if (GetDirection() == CountDirectionUp) {
		std::copy(injdata, injdata + size, adc_data_ + idx);
		std::for_each(adc_data_ + idx, adc_data_ + idx + size, [&](auto &a){ a = -(a - opamp_bias_[idx]);});
	}
}

void PWMSine::SineDriving()
{

#if 1
	SetOCValue(CH1, duty_oc_ * ((v.real()*p1.real() + v.imag()*p1.imag()) + 1.0f) * 0.5f);
	SetOCValue(CH2, duty_oc_ * ((v.real()*p2.real() + v.imag()*p2.imag()) + 1.0f) * 0.5f);
	SetOCValue(CH3, duty_oc_ * ((v.real()*p3.real() + v.imag()*p3.imag()) + 1.0f) * 0.5f);


#else
	SetOCPeriod(CH1, duty_ / 2);
	SetOCPeriod(CH2, duty_ );
	SetOCPeriod(CH3, duty_ );
#endif
}


void PWMSine::HandleUpdate()
{
		float enc_theta = GetEncoderTheta();

#if 0
	if (update_counter_ < 2 * M2E_RATIO * SINE_STEPS) {
		if (update_counter_ < 1 * M2E_RATIO * SINE_STEPS) {
			v = std::polar<float>(1.0f, 0.0f);
			p_encoder->SetIndexOffset(-1);
			p_encoder->ResetPosition(0);
		} else {
			v = v * r;
		}
		update_counter_++;
	} else {
//		Stop();

		v = std::polar<float>(1.0f, enc_theta) * std::polar<float>(1.0f, M_PI/2);
	}
#endif

	if (run_index_ < run_stack_.size()) {
		if (run_stack_[run_index_]())
			run_index_++;
	}

	SineDriving();

#if 1

	if ((update_counter_ % 1000) == 0) {

		float theta = std::arg(v);
		if (theta < 0.0f)
			theta += 2.0f * M_PI;

		printf("%6ld : Elec: %7.3f -> Enc: %7.3f (Delta: %7.3f )   %6ld, Current: %6ld, %6ld, %6ld ( %6ld )\n",
				update_counter_, theta, enc_theta, theta - enc_theta, p_encoder->GetIndexOffset(), adc_data_[0], adc_data_[1], adc_data_[2],
				adc_data_[0] + adc_data_[1] + adc_data_[2]);
	}

#endif

}

/*
 * pwm6step.cpp
 *
 *  Created on: Jun 23, 2017
 *      Author: mstoilov
 */
#include <assert.h>
#include <algorithm>
#include "pwm6step.h"


PWM6Step::PWM6Step(TIM_TypeDef *TIMx,
		const Frequency& switching_freq,
		const Frequency& system_clock,
		TriggerOutput trigger_output,
		OCMode pwm_mode,
		OCPolarity polarity,
		OCPolarity npolarity,
		uint32_t deadtime,
		uint32_t irq_priority,
		const std::vector<GPIOPin>& output_pins)
	: Timer(TIMx, switching_freq.period(), system_clock, irq_priority, output_pins)
	, pwm_mode_(pwm_mode)
	, polarity_(polarity)
	, npolarity_(npolarity)
	, com_cycles_max_(GetSwitchingFrequency()/Frequency::from_millihertz(MIN_MILLIHERTZ * SINE_STATES * MECHANICAL_DEGREES_RATIO))
{
	/*
	 * init sine samples
	 */
	for (size_t i = 0; i < SINE_SAMPLES; i++)
		bootstrap_sine_[i] = (1.0 + std::sin(1.0 * M_PI / 2.0 + i * 2.0 * M_PI / SINE_SAMPLES)) / 2.0;

	/*
	 * Disable all OC channels
	 */
	DisableCCPreload();
	DisableChannel(CH1 | CH2 | CH3 | CH1N | CH2N | CH3N);

	SetOCMode(CH1, pwm_mode_);
	SetOCMode(CH2, pwm_mode_);
	SetOCMode(CH3, pwm_mode_);

	SetOCPolarity(CH1, polarity_);
	SetOCPolarity(CH2, polarity_);
	SetOCPolarity(CH3, polarity_);
	SetOCPolarity(CH1N, npolarity_);
	SetOCPolarity(CH2N, npolarity_);
	SetOCPolarity(CH3N, npolarity_);

	EnableOCPreload(CH1);
	EnableOCPreload(CH2);
	EnableOCPreload(CH3);

	EnableCCPreload();
	EnableARRPreload();

	/*
	 * Set-up dead time
	 */
	SetDeadTime(deadtime);

	/*
	 * ADC trigger
	 */
	SetTriggerOutput(trigger_output);
}

PWM6Step::~PWM6Step()
{
	Stop();
}

void PWM6Step::Start()
{
	DisableARRPreload();
	DisableChannel(CH1 | CH2 | CH3 | CH1N | CH2N | CH3N);
	EnableARRPreload();
	EnableOutputs();
//	DisableOutputs();

	SetJeosState(JEOS_STATE_BOOTSTRAP_INIT);
	SetJeosState(JEOS_STATE_MEASUREMENT1);
	bootstrap_coundown_ = 20;
	EnableCounter();

}

void PWM6Step::Stop()
{
	DisableOutputs();
	DisableCounter();
	DisableARRPreload();
	DisableChannel(CH1 | CH2 | CH3 | CH1N | CH2N | CH3N);
}

void PWM6Step::Toggle()
{
	if (IsEnabledCounter())
		Stop();
	else
		Start();
}

void PWM6Step::SetDutyPeriod(const TimeSpan& period)
{
	duty_ = period;
}

void PWM6Step::SetThrottle(float percent)
{
	percent = std::max(std::min(percent, (float)MAX_THROTTLE), (float)MIN_THROTTLE);
	SetDutyPeriod(GetSwitchingFrequency().period() * percent);
}

float PWM6Step::GetDutyCycle()
{
	return 100.0 * duty_.nanoseconds() / GetSwitchingFrequency().period().nanoseconds() / 100.0;
}

Frequency PWM6Step::GetSwitchingFrequency()
{
	return GetAutoReloadPeriod().to_frequency();
}

void PWM6Step::SetupChannels(uint32_t state)
{
	if (state == 0) {

		/* BEMF CH2 */
		DisableChannel(CH1N | CH3 | CH2 | CH2N);
		EnableChannel(CH1|CH3N);

	} else if (state == 1) {

		/* BEMF CH1 */
		DisableChannel(CH2N | CH3 | CH1 | CH1N);
		EnableChannel(CH2|CH3N);

	} else if (state == 2) {

		/* BEMF CH3 */
		DisableChannel(CH2N | CH1 | CH3 | CH3N);
		EnableChannel(CH2|CH1N);

	} else if (state == 3) {

		/* BEMF CH2 */
		DisableChannel(CH3N | CH1 | CH2 | CH2N);
		EnableChannel(CH3|CH1N);

	} else if (state == 4) {

		/* BEMF CH1 */
		DisableChannel(CH3N | CH2 | CH1 | CH1N);
		EnableChannel(CH3|CH2N);

	} else if (state == 5) {

		/* BEMF CH3 */
		DisableChannel(CH1N | CH2 | CH3 | CH3N);
		EnableChannel(CH1|CH2N);

	}
}

int32_t PWM6Step::CalculateBEMF(int32_t Vh, int32_t Vb, int32_t Vl)
{
	return (2 * Vb  - (Vh + Vl)) / 3;
}

int32_t PWM6Step::DetectBEMF(uint32_t state, int32_t *injdata)
{
	if (state == 0) {
		/* BEMF CH2 */
		return CalculateBEMF(injdata[ADC_A], injdata[ADC_B], injdata[ADC_C]);

	} else if (state == 1) {
		/* BEMF CH1 */
		return CalculateBEMF(injdata[ADC_B], injdata[ADC_A], injdata[ADC_C]);

	} else if (state == 2) {
		/* BEMF CH3 */
		return CalculateBEMF(injdata[ADC_B], injdata[ADC_C], injdata[ADC_A]);

	} else if (state == 3) {
		/* BEMF CH2 */
		return CalculateBEMF(injdata[ADC_C], injdata[ADC_B], injdata[ADC_A]);

	} else if (state == 4) {

		/* BEMF CH1 */
		return CalculateBEMF(injdata[ADC_C], injdata[ADC_A], injdata[ADC_B]);

	} else if (state == 5) {

		/* BEMF CH3 */
		return CalculateBEMF(injdata[ADC_A], injdata[ADC_C], injdata[ADC_B]);
	}

	return 0;
}

void PWM6Step::SetJeosState(JeosState state)
{
	jeos_state_ = state;
}

bool PWM6Step::Bootstrap()
{
	uint32_t com_cycles = com_cycles_max_ / (boots_.elrev_counter + 1);
	float bootstrap_throttle = 0.1 + 1.2f * boots_.elrev_counter / 100.0;

	if (boots_.delay_counter < BOOTSTRAP_DELAY) {
		boots_.delay_counter++;
		DisableCCPreload();
		SetupChannels(state_);
		uint32_t arr = GetAutoReloadValue();
		SetOCValue(CH1, arr * 0.35 * (BOOTSTRAP_DELAY - boots_.delay_counter) / BOOTSTRAP_DELAY);
		SetOCValue(CH2, arr * 0.35 * (BOOTSTRAP_DELAY - boots_.delay_counter) / BOOTSTRAP_DELAY);
		SetOCValue(CH3, arr * 0.35 * (BOOTSTRAP_DELAY - boots_.delay_counter) / BOOTSTRAP_DELAY);
		EnableCCPreload();
		return false;
	}

	if (counter_++ > com_cycles) {
		boots_.com_counter_++;

		if ((boots_.com_counter_ % SINE_STATES) == 0) {
			if (boots_.elrev_counter < BOOTSTRAP_STAGES) {
				boots_.elrev_counter++;
			} else {
//					Stop();
			}
		}

		if (boots_.elrev_counter == BOOTSTRAP_STAGES && state_ == 0) {
			SetThrottle(MIN_THROTTLE);
			state_ = 2;
			SetupChannels(state_);
			GenerateComEvent();
			return true;
		}
		state_ = (state_ + 1) % 6;
		last_counter_ = counter_;
		counter_ = 0;
	}

	DisableCCPreload();
	uint32_t sine_driving_state = SINE_SAMPLES * (state_ * com_cycles + counter_) / (com_cycles * 6);
	uint32_t arr = GetAutoReloadValue();
	uint32_t i1 = (sine_driving_state + SINE_SAMPLES * 0 / 3) % SINE_SAMPLES;
	uint32_t i2 = (sine_driving_state + SINE_SAMPLES * 2 / 3) % SINE_SAMPLES;
	uint32_t i3 = (sine_driving_state + SINE_SAMPLES * 1 / 3) % SINE_SAMPLES;
	uint32_t c1 = bootstrap_sine_[i1] * arr * bootstrap_throttle;
	uint32_t c2 = bootstrap_sine_[i2] * arr * bootstrap_throttle;
	uint32_t c3 = bootstrap_sine_[i3] * arr * bootstrap_throttle;
	SetOCValue(CH1, c1);
	SetOCValue(CH2, c2);
	SetOCValue(CH3, c3);
	EnableChannel(CH1 | CH2 | CH3 | CH1N | CH2N| CH3N);
	EnableCCPreload();

	return false;
}

void PWM6Step::CallbackJEOS(int32_t *injdata, size_t size)
{
	assert(size >= 3);

	jiffies_++;

	std::for_each(injdata, injdata + size, [](auto &a){ a = a * (47000/4700);});

	if (counter_ == adc_data_counter1)
		std::copy(injdata, injdata + adc_data_size, adc_data1_);
	if (counter_ == adc_data_counter2)
		std::copy(injdata, injdata + adc_data_size, adc_data2_);


	int32_t bemf = DetectBEMF(state_, injdata);

	switch(jeos_state_) {
	default:
	case JEOS_STATE_MEASUREMENT1:
		msr_[0].bemf = bemf;
		msr_[0].counter = counter_;
		if (counter_ > 0)
			SetJeosState(JEOS_STATE_MEASUREMENT2);
		break;
	case JEOS_STATE_MEASUREMENT2:
		msr_[1].bemf = bemf;
		msr_[1].counter = counter_;
		/*
		 * Line equation:
		 * y = slope * x + intercept
		 */
		bemf_mslope_ = (msr_[1].bemf - msr_[0].bemf) * 1000 / (msr_[1].counter - msr_[0].counter);
		bemf_intercept_ = (msr_[1].bemf * 1000 - bemf_mslope_ * msr_[1].counter) / 1000;
		if ((state_ % 2 == 0 && ((bemf_mslope_ <= 0 && bemf_intercept_ <= 0))) ||
			(state_ % 2 == 1 && ((bemf_mslope_ >= 0 && bemf_intercept_ >= 0)))) {
			msr_[0] = msr_[1];
			break;
		}

		zero_crossing_ = -bemf_intercept_ * 1000 / bemf_mslope_;
		if ((int32_t)counter_ > zero_crossing_) {
			integral_bemf_ = bemf;
			SetJeosState(JEOS_STATE_ZERODETECTED);
		}
		break;

	case JEOS_STATE_ZERODETECTED:
		integral_bemf_ += bemf;
		if ((abs(integral_bemf_) > (long)BEMF_INTEGRAL_THRESHOLD)) {
			GenerateComEvent();
		}
		break;

	case JEOS_STATE_BOOTSTRAP_INIT:
		boots_ = BootStrap();
		counter_ = 0;
		state_ = 0;
		SetJeosState(JEOS_STATE_BOOTSTRAP);
		break;
	case JEOS_STATE_BOOTSTRAP:
		if (Bootstrap())
			SetJeosState(JEOS_STATE_MEASUREMENT1);
		return;
	}

	if (counter_++ > com_cycles_max_ / 10) {
		if (--bootstrap_coundown_ == 0) {
			SetJeosState(JEOS_STATE_BOOTSTRAP_INIT);
			bootstrap_coundown_ = 20;
		} else {
			GenerateComEvent();
			return;
		}
	}
}

void PWM6Step::GenerateComEvent()
{
	GenerateEvent(EventCOM);

	callback_COM_();
	LogComEvent();

	SetOCPeriod(CH1, duty_);
	SetOCPeriod(CH2, duty_);
	SetOCPeriod(CH3, duty_);

	SetJeosState(JEOS_STATE_MEASUREMENT1);
	last_counter_ = counter_;
	zero_crossing_ = 0UL;
	integral_bemf_ = 0UL;
	counter_ = 0UL;
	state_= (state_ + 1) % 6;
	/*
	 * Prepare the preload bits for the next COM event
	 */
	SetupChannels((state_ + 1) % 6);
}

void PWM6Step::LogComEvent()
{
	if ((log_counter_++ % 145) == 0) {
//		std::copy(adc_data1_, adc_data1_ + adc_data_size, log_entry_.adc_data1_);
//		std::copy(adc_data2_, adc_data2_ + adc_data_size, log_entry_.adc_data2_);
		log_entry_.state_ = state_;
		log_entry_.bemf_intercept_ = bemf_intercept_;
		log_entry_.bemf_mslope_ = bemf_mslope_;
		log_entry_.zero_crossing_ = zero_crossing_;
		log_entry_.integral_bemf_ = integral_bemf_;
		log_entry_.last_counter_ = last_counter_;
		log_entry_.serial_++;
	}

}


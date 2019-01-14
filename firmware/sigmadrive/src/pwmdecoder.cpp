/*
 * PWMDecoder.cpp
 *
 *  Created on: May 4, 2017
 *      Author: mstoilov
 */

#include "pwmdecoder.h"


PWMDecoder::PWMDecoder(TIM_TypeDef *TIMx, const TimeSpan& pwm_period, const Frequency& system_clock, uint32_t irq_priority, const std::vector<GPIOPin>& pins)
	: Timer(TIMx, pwm_period, system_clock, irq_priority, pins)
	, pwm_pulse_(0)
	, pwm_period_(0)
{
	DisableChannel(CH1);
	DisableChannel(CH2);
	DisableChannel(CH3);
	DisableChannel(CH4);


	SetICActiveInput(CH1, DirectTI);
	SetICFilter(CH1, FilterDiv1);
	SetICPrescaler(CH1, PrescalerDiv1);
	SetICPolarity(CH1, PolarityRising);

	SetICActiveInput(CH2, IndirectTI);
	SetICFilter(CH2, FilterDiv1);
	SetICPrescaler(CH2, PrescalerDiv1);
	SetICPolarity(CH2, PolarityFalling);

	SetSlaveMode(SlaveReset);
	SetTriggerInput(TriggerTimerInput1);
	EnableInterrupt(InterruptCC1);
}

PWMDecoder::~PWMDecoder()
{

}

void PWMDecoder::HandleCC1()
{
	__disable_irq();
	pwm_period_ = GetICValue(CH1);
	pwm_pulse_ = GetICValue(CH2);
	__enable_irq();
	callback_PWMCC_(pwm_pulse_, pwm_period_);
}

void PWMDecoder::Start()
{
	EnableChannel(CH1);
	EnableChannel(CH2);
	base::Start();
}

void PWMDecoder::Stop()
{
	base::Stop();
	DisableChannel(CH1);
	DisableChannel(CH2);
	DisableChannel(CH3);
	DisableChannel(CH4);
}

uint32_t PWMDecoder::GetPWMPulse()
{
	return pwm_pulse_;
}

uint32_t PWMDecoder::GetPWMPeriod()
{
	return pwm_period_;
}

TimeSpan PWMDecoder::GetPulseLength()
{
	return system_clock_.period() * (GetPrescaler() + 1) * pwm_pulse_;
}

/*
 * pwm_generator.cpp
 *
 *  Created on: Oct 9, 2019
 *      Author: mstoilov
 */

#include <assert.h>
#include "quadrature_encoder.h"
#include "pwm_generator.h"
#include "main.h"

extern QuadratureEncoder tim4;

PwmGenerator::handle_map_type PwmGenerator::handle_map_;

extern "C"
void TimPeriodElapsedCallback(struct __TIM_HandleTypeDef *htim)
{
	PwmGenerator *tim = PwmGenerator::handle_map_[htim];
	if (tim) {
		tim->PeriodElapsedCallback();
	}
}

void PwmGenerator::PeriodElapsedCallback()
{

#if 0
	if (update_counter_ < (SystemCoreClock / TIM_PERIOD_CLOCKS) / 4) {
		/*
		 * Reset the rotor for 1/4 Second
		 */
		tql_->SetTimings(*this, std::polar<float>(1.0, 0.0));
		tim4.ResetCounter(0);
	} else {
		float rot = tim4.GetElectricPosition();
		std::complex<float> rotor = std::polar(1.0f, rot);
		rotor = rotor * std::complex<float>(0.0f, 1.0f);
		tql_->SetTimings(*this, rotor);
	}
	update_counter_++;
#endif
}

PwmGenerator::PwmGenerator()
	: htim_(nullptr)
{

}

PwmGenerator::~PwmGenerator()
{

}

void PwmGenerator::Attach(TIM_HandleTypeDef* htim)
{
	htim_ = htim;
	assert(handle_map_.find(htim_) == handle_map_.end());
	handle_map_[htim_] = this;
//	LL_TIM_EnableIT_UPDATE(htim_->Instance);
//	EnableCounter(true);
}

uint32_t PwmGenerator::GetTiming(uint32_t ch)
{
	uint32_t ret = 0;
	switch (ch) {
	case 1:
		ret = LL_TIM_OC_GetCompareCH1(htim_->Instance);
		break;
	case 2:
		ret = LL_TIM_OC_GetCompareCH2(htim_->Instance);
		break;
	case 3:
		ret = LL_TIM_OC_GetCompareCH3(htim_->Instance);
		break;
	case 4:
		ret = LL_TIM_OC_GetCompareCH4(htim_->Instance);
		break;
	default:
		throw std::range_error("Invalid PWM channel: " + std::to_string(ch));
		break;
	}
	return ret;
}

void PwmGenerator::SetTiming(uint32_t ch, uint32_t value)
{
	switch (ch) {
	case 1:
		LL_TIM_OC_SetCompareCH1(htim_->Instance, value);
		break;
	case 2:
		LL_TIM_OC_SetCompareCH2(htim_->Instance, value);
		break;
	case 3:
		LL_TIM_OC_SetCompareCH3(htim_->Instance, value);
		break;
	case 4:
		LL_TIM_OC_SetCompareCH4(htim_->Instance, value);
		break;
	default:
		throw std::range_error("Invalid PWM channel: " + std::to_string(ch));
		break;
	}
}

void PwmGenerator::Start()
{
	LL_TIM_EnableIT_UPDATE(htim_->Instance);
	LL_TIM_CC_EnableChannel(htim_->Instance,
			LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
			LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
			LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N |
			LL_TIM_CHANNEL_CH4);
	LL_TIM_GenerateEvent_UPDATE(htim_->Instance);
	LL_TIM_EnableCounter(htim_->Instance);
	EnableOutputs(true);
}

void PwmGenerator::Stop()
{
	EnableOutputs(false);
	LL_TIM_DisableIT_UPDATE(htim_->Instance);
	LL_TIM_CC_DisableChannel(htim_->Instance,
			LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
			LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
			LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N |
			LL_TIM_CHANNEL_CH4);
	LL_TIM_DisableCounter(htim_->Instance);

}

bool PwmGenerator::IsStarted()
{
	return LL_TIM_IsEnabledCounter(htim_->Instance) ? true : false;
}

uint32_t PwmGenerator::GetPeriod()
{
	return LL_TIM_GetAutoReload(htim_->Instance);
}

void PwmGenerator::SetPeriod(uint32_t period)
{
	LL_TIM_SetAutoReload(htim_->Instance, period);
}

void PwmGenerator::GetTimings(uint32_t* values, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		values[i] = GetTiming(i + 1);
	}
}


void PwmGenerator::SetTimings(const uint32_t* values, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		SetTiming(i + 1, values[i]);
	}
}

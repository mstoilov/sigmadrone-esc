/*
 * pwm_generator.cpp
 *
 *  Created on: Oct 9, 2019
 *      Author: mstoilov
 */

#include <stdexcept>
#include <assert.h>
#include "pwm_generator.h"


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
	LL_TIM_EnableIT_UPDATE(htim_->Instance);
//    EnableCounter(true);
}

/** Get compare value set for the specified output channel
 *
 * @param ch The specified channel
 * @return the compare value for the channel
 */
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

/** Set compare value for specified output channel
 *
 * @param ch specified channel
 * @param value Compare value
 */
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

/** Load timings for all channels such that when the timer outputs
 * are enabled, no current will flow through the windings.
 *
 */
void PwmGenerator::LoadSafeTimings()
{
	uint32_t half_period = GetPeriod() / 2;
	SetTiming(1, half_period);
	SetTiming(2, half_period);
	SetTiming(3, half_period);
	SetTiming(4, half_period);
}

/** Enable the timer channels.
 *
 * This enables the outputs of the timer. By calling
 * this function the outputs of the timer channels
 * will be applied to the power FETS gates.
 */
void PwmGenerator::Start()
{
	LoadSafeTimings();
	LL_TIM_CC_EnableChannel(htim_->Instance,
			LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
			LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
			LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N |
			LL_TIM_CHANNEL_CH4);
	EnableOutputs(true);
}

/** Disable the timer channels.
 *
 * This disables the outputs of the timer. By calling
 * this function the outputs of the timer channels
 * will be removed from the power FETS gates.
 */
void PwmGenerator::Stop()
{
	EnableOutputs(false);
	LL_TIM_CC_DisableChannel(htim_->Instance,
			LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
			LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
			LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N |
			LL_TIM_CHANNEL_CH4);
	LoadSafeTimings();
}

/** Get the timer counter
 *
 * @return The current value of the counter
 */
uint32_t PwmGenerator::GetCounter()
{
	return LL_TIM_GetCounter(htim_->Instance);
}


/** Get the timer counter direction
 *
 * @return 0 if the counter is counting up, 1 if the counter is counting down
 */
uint32_t PwmGenerator::GetCounterDirection()
{
	return LL_TIM_GetDirection(htim_->Instance);
}

/** Return the status of the timer counter
 *
 * @return true if the counter is enabled, false if it is disabled
 */
bool PwmGenerator::IsStarted()
{
	return LL_TIM_IsEnabledCounter(htim_->Instance) ? true : false;
}

/** Get the auto reload value (period) the timer counter is counting to.
 *
 * @return Auto-reload value of the counter (period)
 */
uint32_t PwmGenerator::GetPeriod()
{
	return LL_TIM_GetAutoReload(htim_->Instance);
}

/** Set the auto reload value (period) of the timer counter
 *
 * @param period The auto reload value.
 */
void PwmGenerator::SetPeriod(uint32_t period)
{
	LL_TIM_SetAutoReload(htim_->Instance, period);
}

/** Get timings for all channels
 *
 * @param values buffer for the timings to be written in
 * @param count the size of the buffer
 */
void PwmGenerator::GetTimings(uint32_t* values, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		values[i] = GetTiming(i + 1);
	}
}

/** Apply the timings to the output channels
 *
 * @param values buffer for the timings to be read from
 * @param count the size of the buffer
 */
void PwmGenerator::SetTimings(const uint32_t* values, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		SetTiming(i + 1, values[i]);
	}
}

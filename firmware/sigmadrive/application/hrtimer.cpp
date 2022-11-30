/*
 * HRTimer.cpp
 *
 *  Created on: Dec 14, 2019
 *      Author: mstoilov
 */

#include "hrtimer.h"

HRTimer::HRTimer(uint32_t clock_hz, uint32_t counter_mask)
	: htim_(nullptr)
	, clock_hz_(clock_hz)
	, counter_mask_(counter_mask)
{
	// TODO Auto-generated constructor stub

}

HRTimer::~HRTimer()
{
	// TODO Auto-generated destructor stub
}

void HRTimer::Attach(TIM_HandleTypeDef* htim)
{
	htim_ = htim;
	__HAL_TIM_SET_COUNTER(htim_, 0);
	__HAL_TIM_ENABLE(htim_);
}

uint32_t HRTimer::GetCounter()
{
	return __HAL_TIM_GET_COUNTER(htim_);
}

uint32_t HRTimer::GetTimeElapsedMicroSec(uint32_t counter1, uint32_t counter2)
{
	return (uint64_t)GetTimeElapsedClockTicks(counter1, counter2) * (uint64_t)1000000 / clock_hz_;
}

uint32_t HRTimer::GetTimeElapsedNanoSec(uint32_t counter1, uint32_t counter2)
{
	return (uint64_t)GetTimeElapsedClockTicks(counter1, counter2) * (uint64_t)1000000000 / clock_hz_;
}

uint32_t HRTimer::GetTimeElapsedClockTicks(uint32_t counter1, uint32_t counter2)
{
	return (counter2 - counter1) & counter_mask_;
}

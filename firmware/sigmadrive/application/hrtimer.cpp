/*
 * HRTimer.cpp
 *
 *  Created on: Dec 14, 2019
 *      Author: mstoilov
 */

#include "hrtimer.h"

HRTimer::HRTimer()
	: htim_(nullptr)
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

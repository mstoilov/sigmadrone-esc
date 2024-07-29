/*
 * HRTimer.h
 *
 *  Created on: Dec 14, 2019
 *      Author: mstoilov
 */

#ifndef _HRTIMER_H_
#define _HRTIMER_H_

#include "stm32f745xx.h"
#include "stm32f7xx.h"
#include "stm32f7xx_hal_tim.h"
#include "stm32f7xx_ll_tim.h"

class HRTimer {
public:
	HRTimer(uint32_t clock_hz, uint32_t counter_mask);
	virtual ~HRTimer();
	void Attach(TIM_HandleTypeDef* htim);
	uint32_t GetCounter();
	uint32_t GetTimeElapsedMicroSec(uint32_t counter1, uint32_t counter2);
	uint32_t GetTimeElapsedNanoSec(uint32_t counter1, uint32_t counter2);
	uint32_t GetTimeElapsedClockTicks(uint32_t counter1, uint32_t counter2);


public:
	TIM_HandleTypeDef* htim_;
	uint32_t clock_hz_;
	uint32_t counter_mask_;
};

#endif /* APPLICATION_HRTIMER_H_ */

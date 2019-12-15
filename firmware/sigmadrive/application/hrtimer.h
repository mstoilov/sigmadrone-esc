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
	HRTimer();
	virtual ~HRTimer();
	void Attach(TIM_HandleTypeDef* htim);
	uint32_t GetCounter();

public:
	TIM_HandleTypeDef* htim_;
};

#endif /* APPLICATION_HRTIMER_H_ */

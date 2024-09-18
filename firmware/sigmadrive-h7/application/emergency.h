#ifndef _EMERGENCY_H_
#define _EMERGENCY_H_

#include "stm32h743xx.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_ll_tim.h"

inline static void DisableAllMotors()
{
	LL_TIM_DisableAllOutputs(TIM1);
	LL_TIM_DisableAllOutputs(TIM8);

	LL_TIM_OC_SetCompareCH1(TIM1, 0);
	LL_TIM_OC_SetCompareCH2(TIM1, 0);
	LL_TIM_OC_SetCompareCH3(TIM1, 0);

	LL_TIM_OC_SetCompareCH1(TIM8, 0);
	LL_TIM_OC_SetCompareCH2(TIM8, 0);
	LL_TIM_OC_SetCompareCH3(TIM8, 0);
}

inline static void Panic()
{
	DisableAllMotors();
}

#endif

/*
 *  Sigmadrone
 *  Copyright (c) 2013-2015 The Sigmadrone Developers
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Martin Stoilov <martin@sigmadrone.org>
 *  Svetoslav Vassilev <svassilev@sigmadrone.org>
 */

#include <assert.h>
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_rcc.h"

#include "interruptmanager.h"
#include "timer.h"

Timer::Timer(TIM_TypeDef *TIMx, const TimeSpan& timer_period, const Frequency& system_clock, uint32_t irq_priority, const std::vector<GPIOPin>& pins)
	: TIMx_(TIMx)
	, system_clock_(system_clock)
	, pins_(pins)
{
	for (auto& pin : pins_)
		pin.Init();

	id_ = bsp_init_timer(TIMx_, irq_priority);
	assert(id_);
//	g_timers[id_] = this;

	DisableOutputs();
	SetClock(Frequency::from_timespan(timer_period + timer_period / 8) * GetCounterMaxValue());
	SetAutoReloadPeriod(timer_period);
	SetCounterValue(0UL);
	SetCounterMode(Up);
	EnableARRPreload();
}

Timer::~Timer()
{
}

void Timer::Start()
{
	GenerateEvent(EventUpdate);
	EnableCounter();
}

void Timer::Stop()
{
	DisableCounter();
	GenerateEvent(EventUpdate);
}


void Timer::SetOCPeriod(Channel ch, const TimeSpan& period)
{
	SetOCValue(ch, __LL_TIM_CALC_ARR(system_clock_.hertz(), GetPrescaler(), period.to_frequency().hertz()));
}

void Timer::SetOCValue(Channel ch, uint32_t value)
{
	switch (ch) {
	case CH1:
	case CH1N:
		LL_TIM_OC_SetCompareCH1(TIMx_, value);
		break;
	case CH2:
	case CH2N:
		LL_TIM_OC_SetCompareCH2(TIMx_, value);
		break;
	case CH3:
	case CH3N:
		LL_TIM_OC_SetCompareCH3(TIMx_, value);
		break;
	case CH4:
		LL_TIM_OC_SetCompareCH4(TIMx_, value);
		break;
	default:;
	};
}

uint32_t Timer::GetOCValue(Channel ch)
{
	switch (ch) {
		case CH1:
		case CH1N:
			return LL_TIM_OC_GetCompareCH1(TIMx_);
		case CH2:
		case CH2N:
			return LL_TIM_OC_GetCompareCH2(TIMx_);
		case CH3:
		case CH3N:
			return LL_TIM_OC_GetCompareCH3(TIMx_);
		case CH4:
			return LL_TIM_OC_GetCompareCH4(TIMx_);
		default:
			return 0;
	};
	return 0;
}


uint32_t Timer::bsp_init_timer(TIM_TypeDef* TIMx, uint32_t irq_priority)
{
	InterruptManager& IM = InterruptManager::instance();
	if (TIMx == TIM1) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
		IM.Callback_EnableIRQ(TIM1_UP_TIM10_IRQn, irq_priority, [=](void){IrqHandlerUP();});
		IM.Callback_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn, irq_priority, [=](void){IrqHandlerTRG_COM();});
		IM.Callback_EnableIRQ(TIM1_CC_IRQn, irq_priority, [=](void){IrqHandlerCC();});

		/*
		 * TIM1_BRK and TIM9_IRQn are shared. Chain the old handler to the end of the new one.
		 */
		auto OldIrqHandler_BRK = IM.GetIrqHandler(TIM1_BRK_TIM9_IRQn);
		IM.Callback_EnableIRQ(TIM1_BRK_TIM9_IRQn, irq_priority, [=](void){IrqHandlerBRK(); OldIrqHandler_BRK();});

		return 1;
	} else if (TIMx == TIM2) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

		IM.Callback_EnableIRQ(TIM2_IRQn, irq_priority, [=](void){IrqHandler();});
		return 2;
	} else if (TIMx == TIM3) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
		IM.Callback_EnableIRQ(TIM3_IRQn, irq_priority, [=](void){IrqHandler();});
		return 3;
	} else if (TIMx == TIM4) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
		IM.Callback_EnableIRQ(TIM4_IRQn, irq_priority, [=](void){IrqHandler();});
		return 4;
	} else if (TIMx == TIM5) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);
		IM.Callback_EnableIRQ(TIM5_IRQn, irq_priority, [=](void){IrqHandler();});
		return 5;
	} else if (TIMx == TIM9) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM9);

		/*
		 * TIM1_BRK and TIM9_IRQn are shared. Chain the old handler to the end of the new one.
		 */
		auto OldIrqHandler = IM.GetIrqHandler(TIM1_BRK_TIM9_IRQn);
		IM.Callback_EnableIRQ(TIM1_BRK_TIM9_IRQn, irq_priority, [=](void){IrqHandler(); OldIrqHandler();});
		return 9;
	}
	return 0;
}

void Timer::UpdatePrescaler(const Frequency& timer_clock)
{
	uint32_t max_counter = GetCounterMaxValue();
	uint32_t psc = __LL_TIM_CALC_PSC(system_clock_.hertz(), timer_clock.hertz());
	uint32_t arr = 0;

	/*
	 * Make sure we can fit the timer period in the counter register.
	 * Increase the prescaler until we can.
	 */
	for (arr = CalculateARR(system_clock_.hertz(), psc, timer_clock.hertz() / max_counter); arr > max_counter; ) {
		psc += 1;
		arr = CalculateARR(system_clock_.hertz(), psc, timer_clock.hertz() / max_counter);
	}
	LL_TIM_SetPrescaler(TIMx_, psc);
}

uint32_t Timer::CalculateARR(uint32_t sysclk,  uint32_t psc, uint32_t freq)
{
	uint32_t arr = __LL_TIM_CALC_ARR(sysclk, psc, freq);
	return arr;
}

uint32_t Timer::bsp_max_counter(TIM_TypeDef* TIMx)
{
	if (TIMx == TIM1) {
		return 0xffff;
	} else if (TIMx == TIM2) {
		return 0xffffffff;
	} else if (TIMx == TIM3) {
		return 0xffff;
	} else if (TIMx == TIM4) {
		return 0xffff;
	} else if (TIMx == TIM5) {
		return 0xffffffff;
	} else if (TIMx == TIM9) {
		return 0xffff;
	}
	return 0;
}

void Timer::HandleBreak()
{
}

void Timer::HandleUpdate()
{
}

void Timer::HandleTrigger()
{
}

void Timer::HandleCOM()
{
}

void Timer::HandleCC1()
{
}

void Timer::HandleCC2()
{
}

void Timer::HandleCC3()
{
}

void Timer::HandleCC4()
{
}

void Timer::HandleCC1Over()
{
}

void Timer::HandleCC2Over()
{
}

void Timer::HandleCC3Over()
{
}

void Timer::HandleCC4Over()
{
}

void Timer::IrqHandlerUP()
{
	if (LL_TIM_IsActiveFlag_UPDATE(TIMx_)) {
		LL_TIM_ClearFlag_UPDATE(TIMx_);
		HandleUpdate();
		callback_Update_();
	}

}

void Timer::IrqHandlerBRK()
{
	if (LL_TIM_IsActiveFlag_BRK(TIMx_)) {
		LL_TIM_ClearFlag_BRK(TIMx_);
		HandleBreak();
		callback_Break_();
	}
}

void Timer::IrqHandlerTRG_COM()
{
	if (LL_TIM_IsActiveFlag_TRIG(TIMx_)) {
		LL_TIM_ClearFlag_TRIG(TIMx_);
		HandleTrigger();
		callback_Trigger_();
	}
	if (LL_TIM_IsActiveFlag_COM(TIMx_)) {
		LL_TIM_ClearFlag_COM(TIMx_);
		HandleCOM();
		callback_COM_();
	}
}

void Timer::IrqHandlerCC()
{
	if (LL_TIM_IsActiveFlag_CC1(TIMx_)) {
		LL_TIM_ClearFlag_CC1(TIMx_);
		HandleCC1();
		callback_CC1_();
	}
	if (LL_TIM_IsActiveFlag_CC2(TIMx_)) {
		LL_TIM_ClearFlag_CC2(TIMx_);
		HandleCC2();
		callback_CC2_();
	}
	if (LL_TIM_IsActiveFlag_CC3(TIMx_)) {
		LL_TIM_ClearFlag_CC3(TIMx_);
		HandleCC3();
		callback_CC3_();
	}
	if (LL_TIM_IsActiveFlag_CC4(TIMx_)) {
		LL_TIM_ClearFlag_CC4(TIMx_);
		HandleCC4();
		callback_CC4_();
	}
	if (LL_TIM_IsActiveFlag_CC1OVR(TIMx_)) {
		LL_TIM_ClearFlag_CC1OVR(TIMx_);
		HandleCC1Over();
		callback_CC1Over_();
	}
	if (LL_TIM_IsActiveFlag_CC2OVR(TIMx_)) {
		LL_TIM_ClearFlag_CC2OVR(TIMx_);
		HandleCC2Over();
		callback_CC2Over_();
	}
	if (LL_TIM_IsActiveFlag_CC3OVR(TIMx_)) {
		LL_TIM_ClearFlag_CC3OVR(TIMx_);
		HandleCC3Over();
		callback_CC3Over_();
	}
	if (LL_TIM_IsActiveFlag_CC4OVR(TIMx_)) {
		LL_TIM_ClearFlag_CC4OVR(TIMx_);
		HandleCC4Over();
		callback_CC4Over_();
	}


}


void Timer::IrqHandler()
{
	IrqHandlerUP();
	IrqHandlerBRK();
	IrqHandlerTRG_COM();
	IrqHandlerCC();
}

#if 0
extern "C" void TIM1_UP_TIM10_IRQHandler(void)
{
	if (g_timers[1])
		IrqHandler(g_timers[1]);
	if (g_timers[10])
		IrqHandler(g_timers[10]);
}

extern "C" void TIM1_BRK_TIM9_IRQHandler(void)
{
	if (g_timers[1])
		IrqHandler(g_timers[1]);
	if (g_timers[9])
		IrqHandler(g_timers[9]);
}

extern "C" void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
	if (g_timers[1])
		IrqHandler(g_timers[1]);
	if (g_timers[11])
		IrqHandler(g_timers[11]);
}

extern "C" void TIM1_CC_IRQHandler(void)
{
	if (g_timers[1])
		IrqHandler(g_timers[1]);
}

extern "C" void TIM2_IRQHandler(void)
{
	if (g_timers[2])
		IrqHandler(g_timers[2]);
}

extern "C" void TIM3_IRQHandler(void)
{
	if (g_timers[3])
		IrqHandler(g_timers[3]);
}

extern "C" void TIM4_IRQHandler(void)
{
	if (g_timers[4])
		IrqHandler(g_timers[4]);
}

#endif

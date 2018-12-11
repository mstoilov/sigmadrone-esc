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

#include "hwtimer.h"

static HwTimer* g_timers[] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
		nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};


HwTimer::HwTimer(TIM_TypeDef *TIMx, const TimeSpan& timer_period, const Frequency& system_clock, const std::vector<GPIOPin>& output_pins)
	: TIMx_(TIMx)
	, system_clock_(system_clock)
	, output_pins_(output_pins)
{
	for (auto& pin : output_pins_)
		pin.init();

	id_ = bsp_init_timer(TIMx_);
	assert(id_);
	g_timers[id_] = this;

	SetClock(Frequency::from_timespan(timer_period + timer_period / 8) * GetCounterMaxValue());
	SetAutoReloadPeriod(timer_period);
	SetCounterValue(0UL);
	SetCounterMode(Up);
	EnableARRPreload();
}

HwTimer::~HwTimer()
{
	g_timers[id_] = nullptr;
}

void HwTimer::Start()
{
	GenerateEvent(EventUpdate);
	EnableCounter();
}

void HwTimer::Stop()
{
	DisableCounter();
	GenerateEvent(EventUpdate);
}


void HwTimer::SetOCPeriod(Channel ch, const TimeSpan& period)
{
	SetOCValue(ch, __LL_TIM_CALC_ARR(system_clock_.hertz(), GetPrescaler(), period.to_frequency().hertz()));
}

void HwTimer::SetOCValue(Channel ch, uint32_t value)
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

uint32_t HwTimer::GetOCValue(Channel ch)
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

void HwTimer::IrqHandlerDmaCh1()
{

}

void HwTimer::IrqHandlerDmaCh2()
{

}

void HwTimer::IrqHandlerDmaCh3()
{

}


uint32_t HwTimer::bsp_init_timer(TIM_TypeDef* TIMx)
{
	if (TIMx == TIM1) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
		NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 8);
		NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 8);
		NVIC_SetPriority(TIM1_CC_IRQn, 8);
		NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
		NVIC_EnableIRQ(TIM1_CC_IRQn);
		NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 8);
		NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
		return 1;
	} else if (TIMx == TIM2) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
		NVIC_SetPriority(TIM2_IRQn, 8);
		NVIC_EnableIRQ(TIM2_IRQn);
		return 2;
	} else if (TIMx == TIM3) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
		NVIC_SetPriority(TIM3_IRQn, 8);
		NVIC_EnableIRQ(TIM3_IRQn);
		return 3;
	} else if (TIMx == TIM4) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
		NVIC_SetPriority(TIM4_IRQn, 8);
		NVIC_EnableIRQ(TIM4_IRQn);
		return 4;
	} else if (TIMx == TIM5) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);
		NVIC_SetPriority(TIM5_IRQn, 8);
		NVIC_EnableIRQ(TIM5_IRQn);
		return 5;
	} else if (TIMx == TIM9) {
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM9);
		NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 8);
		NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
		return 9;
	}
	return 0;
}

void HwTimer::UpdatePrescaler(const Frequency& timer_clock)
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

uint32_t HwTimer::CalculateARR(uint32_t sysclk,  uint32_t psc, uint32_t freq)
{
	uint32_t arr = __LL_TIM_CALC_ARR(sysclk, psc, freq);
	return arr;
}

void HwTimer::ClearDMAFlags(DMA_TypeDef *dma_device, uint32_t dma_stream)
{
	switch(dma_stream){
	case LL_DMA_STREAM_0:
		LL_DMA_ClearFlag_TC0(dma_device);
		LL_DMA_ClearFlag_TE0(dma_device);
		LL_DMA_ClearFlag_HT0(dma_device);
		break;
	case LL_DMA_STREAM_1:
		LL_DMA_ClearFlag_TC1(dma_device);
		LL_DMA_ClearFlag_TE1(dma_device);
		LL_DMA_ClearFlag_HT1(dma_device);
		break;
	case LL_DMA_STREAM_2:
		LL_DMA_ClearFlag_TC2(dma_device);
		LL_DMA_ClearFlag_TE2(dma_device);
		LL_DMA_ClearFlag_HT2(dma_device);
		break;
	case LL_DMA_STREAM_3:
		LL_DMA_ClearFlag_TC3(dma_device);
		LL_DMA_ClearFlag_TE3(dma_device);
		LL_DMA_ClearFlag_HT3(dma_device);
		break;
	case LL_DMA_STREAM_4:
		LL_DMA_ClearFlag_TC4(dma_device);
		LL_DMA_ClearFlag_TE4(dma_device);
		LL_DMA_ClearFlag_HT4(dma_device);
		break;
	case LL_DMA_STREAM_5:
		LL_DMA_ClearFlag_TC5(dma_device);
		LL_DMA_ClearFlag_TE5(dma_device);
		LL_DMA_ClearFlag_HT5(dma_device);
		break;
	case LL_DMA_STREAM_6:
		LL_DMA_ClearFlag_TC6(dma_device);
		LL_DMA_ClearFlag_TE6(dma_device);
		LL_DMA_ClearFlag_HT6(dma_device);
		break;
	case LL_DMA_STREAM_7:
		LL_DMA_ClearFlag_TC7(dma_device);
		LL_DMA_ClearFlag_TE7(dma_device);
		LL_DMA_ClearFlag_HT7(dma_device);
		break;
	}

}



uint32_t HwTimer::bsp_max_counter(TIM_TypeDef* TIMx)
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

void HwTimer::IrqHandlerBreak()
{
}

void HwTimer::IrqHandlerUpdate()
{
}

void HwTimer::IrqHandlerTrigger()
{
}

void HwTimer::IrqHandlerCOM()
{
}

void HwTimer::IrqHandlerCC1()
{
}

void HwTimer::IrqHandlerCC2()
{
}

void HwTimer::IrqHandlerCC3()
{
}

void HwTimer::IrqHandlerCC4()
{
}

void HwTimer::IrqHandlerCC1Over()
{
}

void HwTimer::IrqHandlerCC2Over()
{
}

void HwTimer::IrqHandlerCC3Over()
{
}

void HwTimer::IrqHandlerCC4Over()
{
}

static void IrqHandler(HwTimer* timer)
{
	if (LL_TIM_IsActiveFlag_UPDATE(timer->TIMx_)) {
		LL_TIM_ClearFlag_UPDATE(timer->TIMx_);
		timer->IrqHandlerUpdate();
		timer->callback_Update_.call();
	}
	if (LL_TIM_IsActiveFlag_COM(timer->TIMx_)) {
		LL_TIM_ClearFlag_COM(timer->TIMx_);
		timer->IrqHandlerCOM();
		timer->callback_COM_.call();
	}
	if (LL_TIM_IsActiveFlag_CC1(timer->TIMx_)) {
		LL_TIM_ClearFlag_CC1(timer->TIMx_);
		timer->IrqHandlerCC1();
		timer->callback_CC1_.call();
	}
	if (LL_TIM_IsActiveFlag_CC2(timer->TIMx_)) {
		LL_TIM_ClearFlag_CC2(timer->TIMx_);
		timer->IrqHandlerCC2();
		timer->callback_CC2_.call();
	}
	if (LL_TIM_IsActiveFlag_CC3(timer->TIMx_)) {
		LL_TIM_ClearFlag_CC3(timer->TIMx_);
		timer->IrqHandlerCC3();
		timer->callback_CC3_.call();
	}
	if (LL_TIM_IsActiveFlag_CC4(timer->TIMx_)) {
		LL_TIM_ClearFlag_CC4(timer->TIMx_);
		timer->IrqHandlerCC4();
		timer->callback_CC4_.call();
	}
	if (LL_TIM_IsActiveFlag_TRIG(timer->TIMx_)) {
		LL_TIM_ClearFlag_TRIG(timer->TIMx_);
		timer->IrqHandlerTrigger();
		timer->callback_Trigger_.call();
	}
	if (LL_TIM_IsActiveFlag_BRK(timer->TIMx_)) {
		LL_TIM_ClearFlag_BRK(timer->TIMx_);
		timer->IrqHandlerBreak();
		timer->callback_Break_.call();

	}
	if (LL_TIM_IsActiveFlag_CC1OVR(timer->TIMx_)) {
		LL_TIM_ClearFlag_CC1OVR(timer->TIMx_);
		timer->IrqHandlerCC1Over();
		timer->callback_CC1Over_.call();
	}
	if (LL_TIM_IsActiveFlag_CC2OVR(timer->TIMx_)) {
		LL_TIM_ClearFlag_CC2OVR(timer->TIMx_);
		timer->IrqHandlerCC2Over();
		timer->callback_CC2Over_.call();
	}
	if (LL_TIM_IsActiveFlag_CC3OVR(timer->TIMx_)) {
		LL_TIM_ClearFlag_CC3OVR(timer->TIMx_);
		timer->IrqHandlerCC3Over();
		timer->callback_CC3Over_.call();
	}
	if (LL_TIM_IsActiveFlag_CC4OVR(timer->TIMx_)) {
		LL_TIM_ClearFlag_CC4OVR(timer->TIMx_);
		timer->IrqHandlerCC4Over();
		timer->callback_CC4Over_.call();
	}
}

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



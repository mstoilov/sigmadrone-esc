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

#ifndef _TIMER_H_
#define _TIMER_H_

#include <stdint.h>
#include <vector>
#include <functional>
#include "sigmadrive.h"
#include "gpiopin.h"
#include "units.h"

class Timer
{
public:
	enum CounterMode {
		Up				= LL_TIM_COUNTERMODE_UP,
		Down			= LL_TIM_COUNTERMODE_DOWN,
		CenterUp		= LL_TIM_COUNTERMODE_CENTER_UP,
		CenterDown		= LL_TIM_COUNTERMODE_CENTER_DOWN,
		CenterUpDown	= LL_TIM_COUNTERMODE_CENTER_UP_DOWN,
	};

	enum TriggerOutput {
		TrigReset		= LL_TIM_TRGO_RESET,
		TrigEnable		= LL_TIM_TRGO_ENABLE,
		TrigUpdate		= LL_TIM_TRGO_UPDATE,
		TrigCC1IF		= LL_TIM_TRGO_CC1IF,
		TrigOC1REF		= LL_TIM_TRGO_OC1REF,
		TrigOC2REF		= LL_TIM_TRGO_OC2REF,
		TrigOC3REF		= LL_TIM_TRGO_OC3REF,
		TrigOC4REF		= LL_TIM_TRGO_OC4REF,
	};


	enum Channel {
		CH1				= LL_TIM_CHANNEL_CH1,
		CH1N			= LL_TIM_CHANNEL_CH1N,
		CH2				= LL_TIM_CHANNEL_CH2,
		CH2N			= LL_TIM_CHANNEL_CH2N,
		CH3				= LL_TIM_CHANNEL_CH3,
		CH3N			= LL_TIM_CHANNEL_CH3N,
		CH4				= LL_TIM_CHANNEL_CH4,
	};

	enum OCMode {
		Forzen			= LL_TIM_OCMODE_FROZEN,
		Active			= LL_TIM_OCMODE_ACTIVE,
		Inactive		= LL_TIM_OCMODE_INACTIVE,
		Toggle			= LL_TIM_OCMODE_TOGGLE,
		ForcedInactive	= LL_TIM_OCMODE_FORCED_INACTIVE,
		ForcedActive	= LL_TIM_OCMODE_FORCED_ACTIVE,
		PWM1			= LL_TIM_OCMODE_PWM1,
		PWM2			= LL_TIM_OCMODE_PWM2,
	};

	enum OCPolarity {
		High			= LL_TIM_OCPOLARITY_HIGH,
		Low				= LL_TIM_OCPOLARITY_LOW,
	};

	enum EventGeneration {
		EventUpdate				= TIM_EGR_UG,
		EventCC1				= TIM_EGR_CC1G,
		EventCC2				= TIM_EGR_CC2G,
		EventCC3				= TIM_EGR_CC3G,
		EventCC4				= TIM_EGR_CC4G,
		EventCOM				= TIM_EGR_COMG,
		EventTrigger			= TIM_EGR_TG,
		EventBreak				= TIM_EGR_BG,
	};

	enum InterruptGeneration {
		InterruptUpdate			= TIM_DIER_UIE,
		InterruptCC1			= TIM_DIER_CC1IE,
		InterruptCC2			= TIM_DIER_CC2IE,
		InterruptCC3			= TIM_DIER_CC3IE,
		InterruptCC4			= TIM_DIER_CC4IE,
		InterruptCOM			= TIM_DIER_COMIE,
		InterruptTrigger		= TIM_DIER_TIE,
		InterruptBreak			= TIM_DIER_BIE,
	};

	enum DmaGeneration {
		DmaUpdate				= TIM_DIER_UDE,
		DmaCC1					= TIM_DIER_CC1DE,
		DmaCC2					= TIM_DIER_CC2DE,
		DmaCC3					= TIM_DIER_CC3DE,
		DmaCC4					= TIM_DIER_CC4DE,
		DmaCOM					= TIM_DIER_COMDE,
		DmaTrigger				= TIM_DIER_TDE,
	};

	enum Statuse {
		Update					= TIM_SR_UIF,
		CC1						= TIM_SR_CC1IF
	};

	enum StatusFlag {
		FlagUpdate				= TIM_SR_UIF,
		FlagCC1					= TIM_SR_CC1IF,
		FlagCC2					= TIM_SR_CC2IF,
		FlagCC3					= TIM_SR_CC3IF,
		FlagCC4					= TIM_SR_CC4IF,
		FlagCOM					= TIM_SR_COMIF,
		FlagTrigger				= TIM_SR_TIF,
		FlagBreak				= TIM_SR_BIF,
		FlagCC1Over				= TIM_SR_CC1OF,
		FlagCC2Over				= TIM_SR_CC2OF,
		FlagCC3Over				= TIM_SR_CC3OF,
		FlagCC4Over				= TIM_SR_CC4OF,
	};

	enum ActiveInput {
		DirectTI				= LL_TIM_ACTIVEINPUT_DIRECTTI,
		IndirectTI				= LL_TIM_ACTIVEINPUT_INDIRECTTI,
		TRC						= LL_TIM_ACTIVEINPUT_TRC,
	};

	enum ICFilter {
		FilterDiv1				= LL_TIM_IC_FILTER_FDIV1,
		FilterDiv1N2			= LL_TIM_IC_FILTER_FDIV1_N2,
		FilterDiv1N4			= LL_TIM_IC_FILTER_FDIV1_N4,
		FilterDiv1N8			= LL_TIM_IC_FILTER_FDIV1_N8,
		FilterDiv2N6			= LL_TIM_IC_FILTER_FDIV2_N6,
		FilterDiv2N8			= LL_TIM_IC_FILTER_FDIV2_N8,
		FilterDiv4N6			= LL_TIM_IC_FILTER_FDIV4_N6,
		FilterDiv4N8			= LL_TIM_IC_FILTER_FDIV4_N8,
		FilterDiv8N6			= LL_TIM_IC_FILTER_FDIV8_N6,
		FilterDiv8N8			= LL_TIM_IC_FILTER_FDIV8_N8,
		FilterDiv16N5			= LL_TIM_IC_FILTER_FDIV16_N5,
		FilterDiv16N4			= LL_TIM_IC_FILTER_FDIV16_N6,
		FilterDiv16N8			= LL_TIM_IC_FILTER_FDIV16_N8,
		FilterDiv32N5			= LL_TIM_IC_FILTER_FDIV32_N5,
		FilterDiv32N4			= LL_TIM_IC_FILTER_FDIV32_N6,
		FilterDiv32N8			= LL_TIM_IC_FILTER_FDIV32_N8,
	};

	enum ICPrescaler {
		PrescalerDiv1			= LL_TIM_ICPSC_DIV1,
		PrescalerDiv2			= LL_TIM_ICPSC_DIV2,
		PrescalerDiv4			= LL_TIM_ICPSC_DIV4,
		PrescalerDiv8			= LL_TIM_ICPSC_DIV8,
	};

	enum ICPolarity {
		PolarityRising			= LL_TIM_IC_POLARITY_RISING,
		PolarityFalling			= LL_TIM_IC_POLARITY_FALLING,
		PolarityRisingFalling	= LL_TIM_IC_POLARITY_BOTHEDGE,
	};

	enum SlaveMode {
		SlaveDisabled			= LL_TIM_SLAVEMODE_DISABLED,
		SlaveReset				= LL_TIM_SLAVEMODE_RESET,
		SlaveGated				= LL_TIM_SLAVEMODE_GATED,
		SlaveTrigger			= LL_TIM_SLAVEMODE_TRIGGER,
	};

	enum EncoderMode {
		EncoderModeTI1 = LL_TIM_ENCODERMODE_X2_TI1,
		EncoderModeTI2 = LL_TIM_ENCODERMODE_X2_TI2,
		EncoderModeTI12 = LL_TIM_ENCODERMODE_X4_TI12,
	};

	enum TriggerInput {
		TriggerInternal0		= LL_TIM_TS_ITR0,
		TriggerInternal1		= LL_TIM_TS_ITR1,
		TriggerInternal2		= LL_TIM_TS_ITR2,
		TriggerInternal3		= LL_TIM_TS_ITR3,
		TriggerEdgeDetectorTI1	= LL_TIM_TS_TI1F_ED,
		TriggerTimerInput1		= LL_TIM_TS_TI1FP1,
		TriggerTimerInput2		= LL_TIM_TS_TI2FP2,
		TriggerExternal			= LL_TIM_TS_ETRF,
	};

	enum UpdateSource {
		UpdateSourceRegular = LL_TIM_UPDATESOURCE_REGULAR,
		UpdateSourceCounter = LL_TIM_UPDATESOURCE_COUNTER,
	};

	enum CountDirction {
		CountDirectionUp = LL_TIM_COUNTERDIRECTION_UP,
		CountDirectionDown = LL_TIM_COUNTERDIRECTION_DOWN,
	};

	Timer(const std::vector<GPIOPin>& pins,
			TIM_TypeDef *TIMx,
			const TimeSpan& timer_period,
			const Frequency& system_clock = Frequency::from_hertz(SystemCoreClock),
			uint32_t irq_priority = 0
	);

	void			SetICActiveInput(Channel ch, ActiveInput ai)	{ LL_TIM_IC_SetActiveInput(TIMx_, ch, ai); }
	void			SetICFilter(Channel ch, ICFilter filter)	{ LL_TIM_IC_SetFilter(TIMx_, ch, filter); }
	void			SetICPrescaler(Channel ch, ICPrescaler pre)	{ LL_TIM_IC_SetPrescaler(TIMx_, ch, pre); }
	void			SetICPolarity(Channel ch, ICPolarity pol)	{ LL_TIM_IC_SetPolarity(TIMx_, ch, pol); }
	uint32_t		GetICValue(Channel ch)						{ return GetOCValue(ch); }
	virtual void	SetOCValue(Channel ch, uint32_t value);
	uint32_t		GetOCValue(Channel ch);
	void			SetEncoderMode(EncoderMode mode)			{ LL_TIM_SetEncoderMode(TIMx_, mode); }
	void			SetSlaveMode(SlaveMode mode)				{ LL_TIM_SetSlaveMode(TIMx_, mode); }
	void			SetTriggerInput(TriggerInput trigger)		{ LL_TIM_SetTriggerInput(TIMx_, trigger); }
	void			SetTriggerOutput(TriggerOutput trigger)		{ LL_TIM_SetTriggerOutput(TIMx_, trigger); }
	void			EnableCounter()								{ LL_TIM_EnableCounter(TIMx_); }
	void			DisableCounter()							{ LL_TIM_DisableCounter(TIMx_); }
	bool			IsEnabledCounter()							{ return LL_TIM_IsEnabledCounter(TIMx_) ? true : false; }
	void			EnableOutputs()								{ LL_TIM_EnableAllOutputs(TIMx_); }
	void			DisableOutputs()							{ LL_TIM_DisableAllOutputs(TIMx_); }
	bool			IsEnalbedOutputs()							{ return LL_TIM_IsEnabledAllOutputs(TIMx_) ? true : false; }
	void			EnableMasterSlaveMode()						{ LL_TIM_EnableMasterSlaveMode(TIMx_); }
	void			DisableMasterSlaveMode()					{ LL_TIM_DisableMasterSlaveMode(TIMx_); }
	bool			IsEnabledMasterSlaveMode()					{ return LL_TIM_IsEnabledMasterSlaveMode(TIMx_) ? true : false; }
	void			EnableChannel(uint32_t ch)					{ LL_TIM_CC_EnableChannel(TIMx_, ch);}
	void			DisableChannel(uint32_t ch)					{ LL_TIM_CC_DisableChannel(TIMx_, ch);}
	bool			IsEnabledChannel(Channel ch)				{ return LL_TIM_CC_IsEnabledChannel(TIMx_, ch) ? true : false;}
	void			SetOCPolarity(Channel ch, OCPolarity p) 	{ LL_TIM_OC_SetPolarity(TIMx_, ch, p);}
	OCPolarity		GetOCPolarity(Channel ch)					{ return static_cast<OCPolarity>(LL_TIM_OC_GetPolarity(TIMx_, ch));}
	void 			SetOCMode(Channel ch, OCMode mode)			{ LL_TIM_OC_SetMode(TIMx_, ch, mode); }
	OCMode 			GetOCMode(Channel ch)						{ return static_cast<OCMode>(LL_TIM_OC_GetMode(TIMx_, ch)); }
	void 			SetCounterMode(CounterMode mode)			{ LL_TIM_SetCounterMode(TIMx_, mode);}
	uint32_t		GetPrescaler()								{ return LL_TIM_GetPrescaler(TIMx_); }
	void			SetPrescaler(uint32_t psc)					{ LL_TIM_SetPrescaler(TIMx_, psc); }
	void			SetClock(const Frequency& frequency)		{ UpdatePrescaler(frequency); }
	void			SetUpdateSource(UpdateSource source)		{ LL_TIM_SetUpdateSource(TIMx_, source); }
	UpdateSource	GetUpdateSource()							{ return static_cast<UpdateSource>(LL_TIM_GetUpdateSource(TIMx_));}
	void			EnableUpdateEvent()							{ LL_TIM_EnableUpdateEvent(TIMx_); }
	void			DisableUpdateEvent()						{ LL_TIM_DisableUpdateEvent(TIMx_); }
	void			EnableARRPreload()							{ LL_TIM_EnableARRPreload(TIMx_); }
	void			DisableARRPreload()							{ LL_TIM_DisableARRPreload(TIMx_); }
	void			EnableCCPreload()							{ LL_TIM_CC_EnablePreload(TIMx_); }
	void			DisableCCPreload()							{ LL_TIM_CC_DisablePreload(TIMx_); }
	void			EnableOCPreload(Channel ch)					{ LL_TIM_OC_EnablePreload(TIMx_, ch); }
	void			DisableOCPreload(Channel ch)				{ LL_TIM_OC_DisablePreload(TIMx_, ch); }
	bool			IsEnabledOCPreload(Channel ch)				{ return LL_TIM_OC_IsEnabledPreload(TIMx_, ch) ? true : false; }
	void 			SetAutoReloadValue(uint32_t value)			{ LL_TIM_SetAutoReload(TIMx_, value); }
	void 			SetAutoReloadPeriod(const TimeSpan& period)	{ SetAutoReloadValue(__LL_TIM_CALC_ARR(system_clock_.hertz(), GetPrescaler(), period.to_frequency().hertz())); }
	TimeSpan		GetAutoReloadPeriod()						{ return GetClock().period() * GetAutoReloadValue(); }
	uint32_t		GetAutoReloadValue()						{ return LL_TIM_GetAutoReload(TIMx_); }
	uint32_t		GetCounterValue()							{ return LL_TIM_GetCounter(TIMx_); }
	CountDirction	GetDirection()								{ return static_cast<CountDirction>(LL_TIM_GetDirection(TIMx_)); }
	void			SetCounterValue(uint32_t value)				{ LL_TIM_SetCounter(TIMx_, value); }
	uint32_t		GetCounterMaxValue()						{ return bsp_max_counter(TIMx_); }
	void 			SetRepetionCounterValue(uint32_t value)		{ LL_TIM_SetRepetitionCounter(TIMx_, value); }
	uint32_t		GetRepetitionCounterValue()					{ return LL_TIM_GetRepetitionCounter(TIMx_); }
	void			GenerateEvent(EventGeneration event)		{ SET_BIT(TIMx_->EGR, event); }
	void			EnableInterrupt(InterruptGeneration ig)		{ SET_BIT(TIMx_->DIER, ig); }
	void			DisableInterrupt(InterruptGeneration ig)	{ CLEAR_BIT(TIMx_->DIER, ig); }
	void			ClearFlag_UPDATE()							{ LL_TIM_ClearFlag_UPDATE(TIMx_);}
	void			EnableDma(DmaGeneration dg)					{ SET_BIT(TIMx_->DIER, dg); }
	void			DisableDma(DmaGeneration dg)				{ CLEAR_BIT(TIMx_->DIER, dg); }
	TIM_TypeDef*	GetTIM()									{ return TIMx_; }
	Frequency 		GetClock()									{ return system_clock_ / (GetPrescaler() + 1); }
	Frequency 		GetSystemClock()							{ return system_clock_; }
	uint32_t 		GetId()										{ return id_; }
	uint32_t		GetStatus()									{ return TIMx_->SR; }
	void			SetClockDivision(uint32_t value)			{ LL_TIM_SetClockDivision(TIMx_, value); }
	uint32_t		GetClockDivision()							{ return LL_TIM_GetClockDivision(TIMx_); }
	void			SetDeadTime(uint32_t ns)					{ LL_TIM_OC_SetDeadTime(TIMx_, __LL_TIM_CALC_DEADTIME(80000000, GetClockDivision(), ns)); }
	void			SetOCPeriod(Channel ch, const TimeSpan& period);
	virtual ~Timer();
	virtual void HandleBreak();
	virtual void HandleUpdate();
	virtual void HandleTrigger();
	virtual void HandleCOM();
	virtual void HandleCC1();
	virtual void HandleCC2();
	virtual void HandleCC3();
	virtual void HandleCC4();
	virtual void HandleCC1Over();
	virtual void HandleCC2Over();
	virtual void HandleCC3Over();
	virtual void HandleCC4Over();

	virtual void Start();
	virtual void Stop();

	static uint32_t CalculateARR(uint32_t sysclk,  uint32_t psc, uint32_t freq);

	template<typename T>
	void Callback_Break(T* object, void (T::*func)(void))			{ callback_Break_ = [=](void){(object->*func)();}; }
	template<typename T>
	void Callback_Update(T* object, void (T::*func)(void))			{ callback_Update_ = [=](void){(object->*func)();}; }
	template<typename T>
	void Callback_Trigger(T* object, void (T::*func)(void))			{ callback_Trigger_ = [=](void){(object->*func)();}; }
	template<typename T>
	void Callback_COM(T* object, void (T::*func)(void))				{ callback_COM_ = [=](void){(object->*func)();}; }
	template<typename T>
	void Callback_CC1(T* object, void (T::*func)(void))				{ callback_CC1_ = [=](void){(object->*func)();}; }
	template<typename T>
	void Callback_CC2(T* object, void (T::*func)(void))				{ callback_CC2_ = [=](void){(object->*func)();}; }
	template<typename T>
	void Callback_CC3(T* object, void (T::*func)(void))				{ callback_CC3_ = [=](void){(object->*func)();}; }
	template<typename T>
	void Callback_CC4(T* object, void (T::*func)(void))				{ callback_CC4_ = [=](void){(object->*func)();}; }
	template<typename T>
	void Callback_CC1Over(T* object, void (T::*func)(void))			{ callback_CC1Over_ = [=](void){(object->*func)();}; }
	template<typename T>
	void Callback_CC2Over(T* object, void (T::*func)(void))			{ callback_CC2Over_ = [=](void){(object->*func)();}; }
	template<typename T>
	void Callback_CC3Over(T* object, void (T::*func)(void))			{ callback_CC3Over_ = [=](void){(object->*func)();}; }
	template<typename T>
	void Callback_CC4Over(T* object, void (T::*func)(void))			{ callback_CC4Over_ = [=](void){(object->*func)();}; }

	void Callback_Break(const std::function<void(void)>& callback)	{ callback_Break_ = callback;	}
	void Callback_Update(const std::function<void(void)>& callback)	{ callback_Update_ = callback;	}
	void Callback_Trigger(const std::function<void(void)>& callback){ callback_Trigger_ = callback;	}
	void Callback_COM(const std::function<void(void)>& callback)	{ callback_COM_ = callback;	}
	void Callback_CC1(const std::function<void(void)>& callback)	{ callback_CC1_ = callback;	}
	void Callback_CC2(const std::function<void(void)>& callback)	{ callback_CC2_ = callback;	}
	void Callback_CC3(const std::function<void(void)>& callback)	{ callback_CC3_ = callback;	}
	void Callback_CC4(const std::function<void(void)>& callback)	{ callback_CC4_ = callback;	}
	void Callback_CC1Over(const std::function<void(void)>& callback){ callback_CC1Over_ = callback;	}
	void Callback_CC2Over(const std::function<void(void)>& callback){ callback_CC2Over_ = callback;	}
	void Callback_CC3Over(const std::function<void(void)>& callback){ callback_CC3Over_ = callback;	}
	void Callback_CC4Over(const std::function<void(void)>& callback){ callback_CC4Over_ = callback;	}


public:
	TIM_TypeDef *TIMx_;

protected:
	void UpdatePrescaler(const Frequency& timer_clock);

public:
	uint32_t bsp_init_timer(TIM_TypeDef* TIMx, uint32_t irq_priority);
	static uint32_t bsp_max_counter(TIM_TypeDef* TIMx);

	void IrqHandlerUP();
	void IrqHandlerBRK();
	void IrqHandlerTRG_COM();
	void IrqHandlerCC();
	void IrqHandler();

protected:
	Frequency system_clock_;
	std::vector<GPIOPin> pins_;
	uint32_t id_;

public:
	std::function<void(void)> callback_Break_ = [](){};
	std::function<void(void)> callback_Update_ = [](){};
	std::function<void(void)> callback_Trigger_ = [](){};
	std::function<void(void)> callback_COM_ = [](){};
	std::function<void(void)> callback_CC1_ = [](){};
	std::function<void(void)> callback_CC2_ = [](){};
	std::function<void(void)> callback_CC3_ = [](){};
	std::function<void(void)> callback_CC4_ = [](){};
	std::function<void(void)> callback_CC1Over_ = [](){};
	std::function<void(void)> callback_CC2Over_ = [](){};
	std::function<void(void)> callback_CC3Over_ = [](){};
	std::function<void(void)> callback_CC4Over_ = [](){};
};

#endif


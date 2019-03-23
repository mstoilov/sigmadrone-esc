#include "trigger.h"

Trigger::Trigger(
		TIM_TypeDef *TIMx,
		const TimeSpan& trigger_period,
		const Frequency& system_clock,
		SlaveMode slave_mode,
		TriggerInput trigger_input,
		TriggerOutput trigger_output,
		Channel ch,
		OCMode pwm_mode)
	: Timer({}, TIMx, trigger_period * 2, system_clock)
{
	SetCounterMode(Up);
	SetCounterValue(0);
	SetSlaveMode(slave_mode);
	SetTriggerInput(trigger_input);
	SetTriggerOutput(trigger_output);
	SetOCMode(ch, pwm_mode);
	LL_TIM_SetOnePulseMode(TIMx, LL_TIM_ONEPULSEMODE_SINGLE);
	SetOCValue(ch, GetAutoReloadValue());
	EnableChannel(ch);
	EnableCCPreload();
	EnableARRPreload();
	GenerateEvent(EventUpdate);
}

Trigger::~Trigger()
{

}

#include "bldc-adctrigger.h"


void bldc_adc_trigger_init(TIM_TypeDef *TIMx, uint32_t timer_hz, uint32_t system_hz, uint32_t max_counter, uint32_t pwm_mode)
{
	bldc_timer_init(TIMx, timer_hz, system_hz, max_counter);
	LL_TIM_SetCounterMode(TIMx, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetCounter(TIMx, 0);
	LL_TIM_SetSlaveMode(TIMx, LL_TIM_SLAVEMODE_TRIGGER);
	LL_TIM_SetTriggerInput(TIMx, LL_TIM_TS_ITR0);
	LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_OC1REF);
	LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, pwm_mode);
	LL_TIM_SetOnePulseMode(TIMx, LL_TIM_ONEPULSEMODE_SINGLE);
	LL_TIM_OC_SetCompareCH1(TIMx, LL_TIM_GetAutoReload(TIMx) / 2);
	LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnablePreload(TIMx);
	LL_TIM_EnableARRPreload(TIMx);
	LL_TIM_GenerateEvent_UPDATE(TIMx);
}


void bldc_adc_trigger_start(TIM_TypeDef *TIMx)
{
	LL_TIM_EnableCounter(TIMx);

}


void bldc_adc_trigger_stop(TIM_TypeDef *TIMx)
{
	LL_TIM_DisableCounter(TIMx);
}


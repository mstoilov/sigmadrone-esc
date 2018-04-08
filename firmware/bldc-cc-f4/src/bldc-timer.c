#include "bldc-timer.h"

void bldc_timer_init(TIM_TypeDef *TIMx, uint32_t timer_hz, uint32_t system_hz, uint32_t max_counter)
{
	uint32_t prescaler = __LL_TIM_CALC_PSC((uint64_t)system_hz, (uint64_t)timer_hz * max_counter);
	uint32_t arr = __LL_TIM_CALC_ARR(system_hz, prescaler, timer_hz);

	while (arr == 0 || arr > max_counter) {
		prescaler += 1;
		arr = __LL_TIM_CALC_ARR(system_hz, prescaler, timer_hz);
	}

	/*
	 * Default init disables preload, so the registers
	 * can be updated immediately updated.
	 */
	LL_TIM_CC_DisablePreload(TIMx);
	LL_TIM_DisableARRPreload(TIMx);

	/*
	 * Setup counter and prescaler.
	 */
	LL_TIM_SetPrescaler(TIMx, prescaler);
	LL_TIM_SetAutoReload(TIMx, arr);
	LL_TIM_SetCounter(TIMx, 0);
	LL_TIM_SetCounterMode(TIMx, LL_TIM_COUNTERMODE_UP);
	LL_TIM_EnableARRPreload(TIMx);

}

void bldc_timer_start(TIM_TypeDef *TIMx)
{
	LL_TIM_GenerateEvent_UPDATE(TIMx);
	LL_TIM_EnableCounter(TIMx);
}

void bldc_timer_stop(TIM_TypeDef *TIMx)
{
	LL_TIM_DisableCounter(TIMx);
}

uint32_t bldc_timer_get_clock_frequency(TIM_TypeDef *TIMx, uint32_t system_clock)
{
	return system_clock / (LL_TIM_GetPrescaler(TIMx) + 1);
}


void bldc_timer_init_pwm_decoder(TIM_TypeDef *TIMx, uint32_t timer_hz, uint32_t system_hz, uint32_t max_counter)
{
	bldc_timer_init(TIMx, timer_hz, system_hz, max_counter);

	/*
	 * Disable all CC channels.
	 */
	LL_TIM_CC_DisableChannel(TIMx,
							LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
							LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
							LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N |
							LL_TIM_CHANNEL_CH4);

	/*
	 * Set up Input Filter & Edge detector for channel 1
	 */
	LL_TIM_IC_SetFilter(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1_N8);

	/*
	 * Setup channel 1 for direct input, i.e. TI1
	 * Detect rising edge
	 */
	LL_TIM_IC_SetActiveInput(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
	LL_TIM_IC_SetPrescaler(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
	LL_TIM_IC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);

	/*
	 * Setup channel 2 for indirect input, i.e. TI1
	 * Detect falling edge
	 */
	LL_TIM_IC_SetActiveInput(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_INDIRECTTI);
	LL_TIM_IC_SetPrescaler(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
	LL_TIM_IC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_FALLING);

	/*
	 * Setup trigger slave mode
	 */
	LL_TIM_SetSlaveMode(TIMx, LL_TIM_SLAVEMODE_RESET);
	LL_TIM_SetTriggerInput(TIMx, LL_TIM_TS_TI1FP1);
	LL_TIM_EnableIT_CC1(TIMx);
}

void bldc_timer_pwm_decoder_start(TIM_TypeDef *TIMx)
{
	LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
	LL_TIM_GenerateEvent_UPDATE(TIMx);
	LL_TIM_EnableCounter(TIMx);
}

void bldc_timer_pwm_decoder_stop(TIM_TypeDef *TIMx)
{
	LL_TIM_DisableCounter(TIMx);
	LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
	LL_TIM_GenerateEvent_UPDATE(TIMx);
}


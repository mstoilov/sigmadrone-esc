#include "bldc-timer.h"

void bldc_6step_init(TIM_TypeDef *TIMx, uint32_t timer_hz, uint32_t system_hz, uint32_t max_counter,
		uint32_t pwm_mode, uint32_t polarity, uint32_t npolarity, uint32_t deadtime_ns, uint32_t repetions)
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

	LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, pwm_mode);
	LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH2, pwm_mode);
	LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH3, pwm_mode);


	LL_TIM_OC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH1, polarity);
	LL_TIM_OC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH2, polarity);
	LL_TIM_OC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH3, polarity);

	LL_TIM_OC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH1N, npolarity);
	LL_TIM_OC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH2N, npolarity);
	LL_TIM_OC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH3N, npolarity);

	LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
	LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);

	LL_TIM_OC_SetDeadTime(TIMx, __LL_TIM_CALC_DEADTIME(system_hz, LL_TIM_GetClockDivision(TIMx), deadtime_ns));
	LL_TIM_SetRepetitionCounter(TIMx, repetions);

	LL_TIM_CC_EnablePreload(TIMx);
	LL_TIM_EnableARRPreload(TIMx);

	/*
	 * ADC trigger
	 */
	LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_UPDATE);
}

void bldc_6step_start(TIM_TypeDef *TIMx)
{
	LL_TIM_CC_DisablePreload(TIMx);

	LL_TIM_CC_DisableChannel(TIMx,
							LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
							LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
							LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);


	LL_TIM_CC_EnablePreload(TIMx);
	LL_TIM_EnableAllOutputs(TIMx);
//	LL_TIM_DisableAllOutputs(TIMx);
	LL_TIM_EnableCounter(TIMx);
}

void bldc_6step_stop(TIM_TypeDef *TIMx)
{
	LL_TIM_DisableAllOutputs(TIMx);
	LL_TIM_DisableCounter(TIMx);
	LL_TIM_CC_DisablePreload(TIMx);
	LL_TIM_CC_DisableChannel(TIMx,
							LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
							LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
							LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);

}

#define BLDC_MAX_THROTTLE 90
void bldc_6step_set_throttle(TIM_TypeDef *TIMx, uint32_t percent /* 0 .. 100 */ )
{
	uint32_t arr = 0;
	uint32_t value = 0;

	if (percent > BLDC_MAX_THROTTLE)
		percent = BLDC_MAX_THROTTLE;
	arr = LL_TIM_GetAutoReload(TIMx);
	value = arr * percent / 100;
	LL_TIM_OC_SetCompareCH1(TIMx, value);
	LL_TIM_OC_SetCompareCH2(TIMx, value);
	LL_TIM_OC_SetCompareCH3(TIMx, value);
}

#define BLDC_THROTTLE_STEP 2
void bldc_6step_updown_throttle(TIM_TypeDef *TIMx, uint32_t percent /* 0 .. 100 */ )
{
	uint32_t arr = 0;
	uint32_t value = 0;
	uint32_t current_value = LL_TIM_OC_GetCompareCH1(TIMx);

	if (percent > BLDC_MAX_THROTTLE)
		percent = BLDC_MAX_THROTTLE;
	arr = LL_TIM_GetAutoReload(TIMx);
	value = arr * percent / 100;

	if (value < current_value) {
		current_value -= BLDC_THROTTLE_STEP;
		if (current_value < value)
			current_value = value;
	} else {
		current_value += BLDC_THROTTLE_STEP;
		if (current_value > value)
			current_value = value;
	}

	LL_TIM_OC_SetCompareCH1(TIMx, current_value);
	LL_TIM_OC_SetCompareCH2(TIMx, current_value);
	LL_TIM_OC_SetCompareCH3(TIMx, current_value);
}


void bldc_6step_toggle(TIM_TypeDef *TIMx)
{
	if (LL_TIM_IsEnabledCounter(TIMx))
		bldc_6step_stop(TIMx);
	else
		bldc_6step_start(TIMx);
}

uint32_t bldc_6step_switching_frequency(TIM_TypeDef *TIMx, uint32_t system_clock)
{
	return bldc_timer_get_clock_frequency(TIMx, system_clock) / LL_TIM_GetAutoReload(TIMx);
}

void bldc_6step_next(TIM_TypeDef *TIMx, uint32_t state)
{
	if (state == 0) {

		/* BEMF CH2 */
		LL_TIM_CC_DisableChannel(TIMx,
									LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
									LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
									LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
		LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3N);

	} else if (state == 1) {

		/* BEMF CH1 */
		LL_TIM_CC_DisableChannel(TIMx,
									LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
									LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
									LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
		LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3N);

	} else if (state == 2) {

		/* BEMF CH3 */
		LL_TIM_CC_DisableChannel(TIMx,
									LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
									LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
									LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
		LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH1N);

	} else if (state == 3) {

		/* BEMF CH2 */
		LL_TIM_CC_DisableChannel(TIMx,
									LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
									LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
									LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
		LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH1N);

	} else if (state == 4) {

		/* BEMF CH1 */
		LL_TIM_CC_DisableChannel(TIMx,
									LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
									LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
									LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
		LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH2N);

	} else if (state == 5) {

		/* BEMF CH3 */
		LL_TIM_CC_DisableChannel(TIMx,
									LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
									LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
									LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
		LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2N);
	}

}


void bldc_6step_sine_next(TIM_TypeDef *TIMx, uint32_t state)
{
	if (state == 0) {

		/* BEMF CH2 */
		LL_TIM_CC_DisableChannel(TIMx,
									LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
									LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
									LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
		LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3N);

	} else if (state == 1) {

		/* BEMF CH1 */
		LL_TIM_CC_DisableChannel(TIMx,
									LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
									LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
									LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
		LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3N);

	} else if (state == 2) {

		/* BEMF CH3 */
		LL_TIM_CC_DisableChannel(TIMx,
									LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
									LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
									LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
		LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH1N);

	} else if (state == 3) {

		/* BEMF CH2 */
		LL_TIM_CC_DisableChannel(TIMx,
									LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
									LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
									LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
		LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH1N);

	} else if (state == 4) {

		/* BEMF CH1 */
		LL_TIM_CC_DisableChannel(TIMx,
									LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
									LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
									LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
		LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH2N);

	} else if (state == 5) {

		/* BEMF CH3 */
		LL_TIM_CC_DisableChannel(TIMx,
									LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |
									LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |
									LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
		LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2N);
	}

}
